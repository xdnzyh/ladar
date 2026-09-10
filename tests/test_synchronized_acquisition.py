import importlib.util
import math
from pathlib import Path
import queue
import sys
import types
import unittest
import json
from unittest.mock import Mock, patch

from data_fusion import parse_trigger
from radar_core import CalibrationModel, RotationTracker
from scan_acquisition import TimedSweepBuilder
from serial_backend import _PySerialTransport
from synchronized_acquisition import ClockEstimate, SynchronizedAcquisition


class Endpoint:
    def __init__(self):
        self.messages = []
        self.now = 0.0
        self.rejected_prefix = None

    def write_line(self, line, on_sent=None):
        self.messages.append(line)
        if self.rejected_prefix and line.startswith(self.rejected_prefix):
            return False
        if on_sent:
            on_sent(self.now)
        return True


class CallbackEndpoint(Endpoint):
    def __init__(self):
        super().__init__()
        self.callbacks = []

    def write_line(self, line, on_sent=None):
        self.messages.append(line)
        if on_sent:
            self.callbacks.append(on_sent)
        return True


class ClockTests(unittest.TestCase):
    def test_asymmetric_exchange_contains_actual_offset(self):
        clock = ClockEstimate.exchange(10.0, 7_004_000, 7_009_000, 10.049, 0)
        self.assertAlmostEqual(clock.offset, -3.018)
        self.assertLessEqual(abs(clock.offset + 3.0), clock.uncertainty)
        mapped, error = clock.map(8_000_000, 11.0)
        self.assertLessEqual(abs(mapped - 11.0), error)

    def test_clock_age_increases_error(self):
        clock = ClockEstimate(0, 0.001, 0, 500)
        self.assertAlmostEqual(clock.map(1_000_000, 10)[1], 0.0015 / 0.9995)
        self.assertEqual(clock.map(1_000_000, 10), clock.map(1_000_000, 2))

    def test_invalid_exchange_rejected(self):
        with self.assertRaises(ValueError):
            ClockEstimate.exchange(0, 0, 100000, 0.01)

    def test_named_and_legacy_trigger(self):
        self.assertEqual(parse_trigger('TRIG 3 TICK_US=123456'), (3, 123456))
        self.assertEqual(parse_trigger('TRIG 3 123456'), (3, 123456))
        self.assertIsNone(parse_trigger('SLIDE_REPORT WINDOW=1 AVG_RPS=0.4'))


class SweepTests(unittest.TestCase):
    def build(self, error=0.0001, gap=False):
        builder = TimedSweepBuilder({})
        builder.trigger(0, error, 1)
        result = []
        for count in range(2, 5):
            for i in range(1, 40):
                if gap and 5 <= i <= 15:
                    continue
                builder.sample((count - 2) * 2 + i * 0.05, error, 800, 1.0)
            result = builder.trigger((count - 1) * 2, error, count)
        return builder, result

    def test_steady_complete_sweep(self):
        builder, points = self.build()
        self.assertEqual(len(points), 39)
        self.assertAlmostEqual(points[19].angle_rad, math.pi)
        self.assertEqual(builder.period_s, 2)

    def test_timing_error_blocks_mapping(self):
        builder, points = self.build(error=0.03)
        self.assertEqual(points, [])
        self.assertIn('有效测距不足', builder.reason)
        self.assertGreater(builder.rejected_points, 0)

    def test_missing_sector_blocks_mapping(self):
        builder, points = self.build(gap=True)
        self.assertEqual(points, [])
        self.assertIn('角度空缺', builder.reason)

    def test_missing_zero_is_not_one_slow_revolution(self):
        tracker = RotationTracker()
        tracker.trigger(0, 1)
        tracker.add_sample(1, 800, 0.5)
        self.assertEqual(tracker.trigger(2, 3), [])
        self.assertEqual(list(tracker.period_history), [])

    def test_duplicate_zero_keeps_pending_samples(self):
        tracker = RotationTracker()
        tracker.trigger(0, 1)
        tracker.add_sample(1, 800, 0.5)
        tracker.trigger(0.1, 1)
        self.assertEqual(len(tracker.trigger(1, 2)), 1)

class AcquisitionTests(unittest.TestCase):
    def finish_clock_sync(self, acquisition, endpoints):
        for i in range(8):
            now = 0.2 + i * 0.1
            for endpoint in endpoints:
                endpoint.now = now
            acquisition.poll(now)
            for source in acquisition.endpoints:
                token = acquisition.outstanding[source][0]
                acquisition.feed(source, f'SYNC {token} {round((now+0.002)*1e6)} {round((now+0.003)*1e6)}\n'.encode(), now+0.005)
            acquisition.poll(now+0.005)

    def test_motor_waits_for_measurement_start_ack(self):
        a, endpoints, _ = self.make()
        self.finish_clock_sync(a, endpoints)
        self.assertFalse(any(line.startswith('ROT ') for line in endpoints[1].messages))
        a.feed('measurement', f'OK START {a.session}\n'.encode(), 1.0)
        a.poll(1.0)
        self.assertEqual(endpoints[1].messages[-1], 'ROT ' + a.session)
        self.assertNotEqual(a.state, 'running')
        a.feed('rotation', f'OK ROT {a.session}\n'.encode(), 1.1)
        a.poll(1.1)
        self.assertEqual(a.state, 'running')

    def test_rejected_start_does_not_start_motor(self):
        a, endpoints, output = self.make()
        endpoints[0].rejected_prefix = 'START '
        self.finish_clock_sync(a, endpoints)
        self.assertEqual(a.state, 'stopped')
        self.assertFalse(any(line.startswith('ROT ') for line in endpoints[1].messages))
        self.assertTrue(any(event[0] == 'sync_error' for event in output))

    def test_missing_start_ack_stops_both_devices(self):
        a, endpoints, output = self.make()
        self.finish_clock_sync(a, endpoints)
        a.poll(5.0)
        self.assertEqual(a.state, 'stopped')
        self.assertFalse(any(line.startswith('ROT ') for line in endpoints[1].messages))
        self.assertEqual(endpoints[1].messages[-1], 'OFF')
        self.assertTrue(any('启动' in str(event[1]) for event in output if event[0] == 'sync_error'))

    def test_stale_and_duplicate_start_acks_cannot_restart_motor(self):
        a, endpoints, _ = self.make()
        self.finish_clock_sync(a, endpoints)
        a.feed('measurement', b'OK START old-session\n', 1.0)
        a.poll(1.0)
        self.assertFalse(any(line.startswith('ROT ') for line in endpoints[1].messages))
        acknowledgement = f'OK START {a.session}\n'.encode()
        a.feed('measurement', acknowledgement * 2, 1.1)
        a.poll(1.1)
        self.assertEqual(sum(line.startswith('ROT ') for line in endpoints[1].messages), 1)

    def test_rotation_start_send_failure_stops_measurement(self):
        a, endpoints, output = self.make()
        self.finish_clock_sync(a, endpoints)
        endpoints[1].rejected_prefix = 'ROT '
        a.feed('measurement', f'OK START {a.session}\n'.encode(), 1.0)
        a.poll(1.0)
        self.assertEqual(a.state, 'stopped')
        self.assertEqual(endpoints[0].messages[-2:], ['STOP', 'LASER 0'])
        self.assertTrue(any(event[0] == 'sync_error' for event in output))

    def test_pixels_waiting_for_rotation_ack_are_preserved(self):
        a, _, _ = self.make()
        self.finish_clock_sync(a, a.endpoints.values())
        session = a.session
        a.feed('measurement', f'OK START {session}\nPIX {session} 1 1049000 1051000 800\n'.encode(), 1.1)
        a.poll(1.1)
        self.assertEqual(a.last_sequence['measurement'], 1)
        self.assertEqual(len(a.pending), 1)
        self.assertIsNone(a.builder.anchor)
        a.feed('rotation', f'OK ROT {session}\n'.encode(), 1.2)
        a.poll(1.2)
        self.assertEqual(a.state, 'running')
        self.assertEqual(len(a.pending), 1)

    def test_late_rotation_ack_does_not_bypass_start_timeout(self):
        a, endpoints, output = self.make()
        self.finish_clock_sync(a, endpoints)
        session = a.session
        a.feed('measurement', f'OK START {session}\n'.encode(), 1.0)
        a.poll(1.0)
        a.feed('rotation', f'OK ROT {session}\n'.encode(), 3.1)
        a.poll(3.1)
        self.assertEqual(a.state, 'stopped')
        self.assertTrue(any(event[0] == 'sync_error' for event in output))

    def make(self):
        endpoints = [Endpoint(), Endpoint()]
        output = []
        acquisition = SynchronizedAcquisition(*endpoints, CalibrationModel(p0=700, k=100),
            {'clock_drift_bound_ppm': 0, 'irq_timestamp_uncertainty_ms': 0, 'sync_max_age_s': 60},
            lambda *args: output.append(args))
        acquisition.start(0)
        return acquisition, endpoints, output

    def running(self):
        a, endpoints, output = self.make()
        a.state = 'running'
        a.clocks = {source: ClockEstimate(0, 0.0001, 0, 0) for source in a.endpoints}
        a.last_arrival = {source: 0 for source in a.endpoints}
        return a, endpoints, output

    def test_complete_eight_probe_handshake(self):
        a, endpoints, output = self.make()
        self.finish_clock_sync(a, endpoints)
        a.feed('measurement', f'OK START {a.session}\n'.encode(), 1.0)
        a.feed('rotation', f'OK ROT {a.session}\n'.encode(), 1.1)
        a.poll(1.1)
        self.assertEqual(a.state, 'running')
        self.assertIn(f'START {a.session} 100 5 fffe', endpoints[0].messages)
        self.assertTrue(any(message.startswith('ROT ') for message in endpoints[1].messages))
        self.assertTrue(any('测距 8/8' in str(value) and '旋转 8/8' in str(value)
                            for kind, value, _ in output if kind == 'sync_status'))

    def test_sync_failure_distinguishes_zero_byte_response(self):
        a, endpoints, output = self.make()
        endpoints[0].port = 'COM3'
        endpoints[1].port = 'COM4'
        for timestamp in (0.1 + i * 1.1 for i in range(17)):
            a.poll(timestamp)
        errors = [str(value) for kind, value, _ in output if kind == 'sync_error']
        self.assertEqual(a.state, 'stopped')
        self.assertTrue(any('测距端(COM3)未收到任何字节' in value for value in errors))
        self.assertTrue(any('已发送 16 次' in value and '有效回应 0/8' in value for value in errors))

    def test_sync_failure_reports_received_invalid_frames(self):
        a, endpoints, output = self.make()
        endpoints[0].port = 'COM3'
        a.feed('measurement', b'not-a-sync\n', 0.2)
        a.poll(0.2)
        for timestamp in (1.3 + i * 1.1 for i in range(16)):
            a.poll(timestamp)
        errors = [str(value) for kind, value, _ in output if kind == 'sync_error']
        self.assertTrue(any('收到' in value and '有效 SYNC' in value and 'COM3' in value for value in errors))
        self.assertGreater(a.received_bytes['measurement'], 0)

    def test_rotation_waits_for_delayed_measurements(self):
        a, _, output = self.running()
        a.receiver.reorder_s = 1.1
        a.last_arrival['measurement'] = 0.2
        s = a.session
        a.feed('rotation', f'TRIG {s} 1 1000000\nTRIG {s} 2 2000000\n'.encode(), 2.01)
        a.poll(2.01)
        self.assertIsNone(a.builder.anchor)
        a.feed('measurement', f'PIX {s} 1 1499000 1501000 800\nPIX {s} 2 2099000 2101000 800\n'.encode(), 2.1)
        a.poll(3.2)
        self.assertEqual(a.builder.anchor[2], 2)
        self.assertFalse(any(event[0] == 'sync_sweep' for event in output))

    def test_old_session_ignored(self):
        a, _, _ = self.running()
        a.feed('rotation', b'TRIG old-session 1 1000000\n', 1)
        a.poll(1)
        self.assertEqual(a.pending, [])

    def test_range_sequence_gap_keeps_partial_sweep(self):
        a, _, _ = self.running()
        s = a.session
        a.builder.trigger(0, 0, 1)
        a.builder.sample(0.2, 0, 800, 1)
        a._line('measurement', f'PIX {s} 1 499000 501000 800', 0.51)
        a._line('measurement', f'PIX {s} 3 699000 701000 800', 0.71)
        self.assertEqual(a.builder.anchor, (0, 0, 1))
        self.assertEqual(len(a.builder.samples), 1)

    def test_ready_status_does_not_abort_scan_but_data_gap_stops(self):
        a, _, output = self.running()
        a.feed('rotation', b'READY ROTATION_SYNC_V1\n', 1)
        a.poll(1)
        self.assertEqual(a.state, 'running')
        a.poll(2.1)
        self.assertEqual(a.state, 'running')
        a.poll(12.1)
        self.assertEqual(a.state, 'stopped')
        self.assertTrue(any(item[0] == 'sync_error' for item in output))

    def test_error_command_and_foreign_protocol_do_not_abort_valid_capture(self):
        a, _, _ = self.running()
        session = a.session
        a.feed('rotation', b'ERROR COMMAND\nERROR COMMAND_ENCODING\nERR,node=MOTOR01,status=BUSY\nSTATUS MOTOR=1\n', 0.2)
        a.feed('measurement', f'PIX {session} 1 1000000 1000000 800\n'.encode(), 0.2)
        a.poll(0.2)
        self.assertEqual(a.state, 'running')
        self.assertGreaterEqual(a.sync_stats['rotation']['ignored'], 4)
        self.assertGreaterEqual(a.sync_stats['measurement']['success'], 0)
        self.assertTrue(any(reason == '外来协议消息' for _, _, reason, _ in a.recent_anomalies))

    def test_start_drains_previous_error_before_new_session(self):
        a, _, output = self.make()
        a.feed('rotation', b'ERROR COMMAND\n', 0.5)
        a.start(2.0)
        a.feed('rotation', b'ERROR COMMAND\nERR,node=MOTOR01,status=BUSY\n', 2.05)
        a.poll(2.0)
        self.assertEqual(a.state, 'syncing')
        a.poll(2.05)
        self.assertEqual(a.state, 'syncing')
        self.assertFalse(any(kind == 'sync_error' for kind, _, _ in output))
        self.assertEqual(a.sync_stats['rotation']['ignored'], 2)

    def test_late_old_session_range_and_trigger_do_not_enter_new_run(self):
        a, _, _ = self.running()
        a.feed('measurement', b'PIX old-session 1 1000000 1000000 800\n', 0.2)
        a.feed('rotation', b'TRIG old-session 1 1000000\n', 0.2)
        a.poll(0.2)
        self.assertEqual(a.state, 'running')
        self.assertEqual(a.pending, [])

    def test_old_send_callback_cannot_populate_new_session(self):
        endpoints = [CallbackEndpoint(), CallbackEndpoint()]
        a = SynchronizedAcquisition(*endpoints, CalibrationModel(p0=700, k=100),
            {'clock_drift_bound_ppm': 0}, lambda *args: None)
        a.start(0.0)
        a.poll(0.1)
        old_callback = endpoints[0].callbacks[0]
        a.start(1.0)
        old_callback(1.01)
        a.poll(1.01)
        self.assertEqual(a.sent_times, {})
        self.assertEqual(a.state, 'syncing')

    def test_stop_reports_written_and_received_confirmation_separately(self):
        a, endpoints, _ = self.make()
        a.stop()
        self.assertTrue(a.stop_status['measurement']['written'])
        self.assertFalse(a.stop_status['measurement']['confirmed'])
        a.feed('measurement', b'OK STOP\n', 1.0)
        a.feed('rotation', b'OK OFF\n', 1.0)
        a.poll(1.0)
        self.assertTrue(a.stop_status['measurement']['confirmed'])
        self.assertTrue(a.stop_status['rotation']['confirmed'])
        self.assertEqual(endpoints[0].messages[-2:], ['STOP', 'LASER 0'])

    def test_only_noise_or_no_return_still_times_out(self):
        a, _, _ = self.running()
        session = a.session
        for timestamp in (0.5, 1.0, 1.5, 1.9):
            a.feed('measurement', b'noise\xff\nERR,node=MOTOR01\n', timestamp)
            a.feed('measurement', f'PIX {session} {int(timestamp * 100)} 1000000 1000000 -1\n'.encode(), timestamp)
            a.poll(timestamp)
            self.assertEqual(a.state, 'running')
        a.poll(2.1)
        self.assertEqual(a.state, 'running')
        a.poll(12.1)
        self.assertEqual(a.state, 'stopped')
        self.assertGreater(a.sync_stats['measurement']['format_error'], 0)

    def test_explicit_watchdog_fault_stops_capture(self):
        a, _, output = self.running()
        a.feed('rotation', b'ERROR WATCHDOG\n', 0.5)
        a.poll(0.5)
        self.assertEqual(a.state, 'stopped')
        self.assertTrue(any('WATCHDOG' in str(value) for kind, value, _ in output if kind == 'sync_error'))

    def test_current_start_argument_error_stops_startup(self):
        a, endpoints, output = self.make()
        self.finish_clock_sync(a, endpoints)
        a.feed('measurement', b'ERROR START_ARGUMENTS\n', 1.0)
        a.poll(1.0)
        self.assertEqual(a.state, 'stopped')
        self.assertTrue(any('START_ARGUMENTS' in str(value) for kind, value, _ in output if kind == 'sync_error'))

    def test_ten_start_stop_cycles_keep_sessions_isolated(self):
        endpoints = [Endpoint(), Endpoint()]
        output = []
        a = SynchronizedAcquisition(*endpoints, CalibrationModel(p0=700, k=100),
            {'clock_drift_bound_ppm': 0, 'irq_timestamp_uncertainty_ms': 0},
            lambda *args: output.append(args))
        sessions = []
        for _ in range(10):
            a.start(0.0)
            self.finish_clock_sync(a, endpoints)
            sessions.append(a.session)
            a.feed('measurement', f'OK START {a.session}\n'.encode(), 1.0)
            a.poll(1.0)
            a.feed('rotation', f'OK ROT {a.session}\n'.encode(), 1.1)
            a.poll(1.1)
            self.assertEqual(a.state, 'running')
            a.stop()
            self.assertEqual(a.state, 'stopped')
        self.assertEqual(len(set(sessions)), 10)
        self.assertFalse(any(kind == 'sync_error' for kind, _, _ in output))

    def test_old_radar_sync_error_event_is_ignored_after_restart(self):
        from radar_app import RadarApp
        app = object.__new__(RadarApp)
        app.demo = True
        app.scanning = False
        app.events = queue.Queue()
        app.sync = types.SimpleNamespace(generation=2)
        app.root = Mock()
        app.events.put(('sync_error', (1, 'old failure'), 0.0))
        app._poll()
        self.assertFalse(app.scanning)

    def test_full_scan_with_different_clocks_and_delayed_radio_stream(self):
        a, _, output = self.running()
        s = a.session
        a.clocks = {'measurement': ClockEstimate(100, 0.0001, 0, 0),
                    'rotation': ClockEstimate(250, 0.0001, 0, 0)}
        packets = []
        for count in range(1, 6):
            t = 1 + (count - 1) * 2
            packets.append((t + 0.002, 'rotation', f'TRIG {s} {count} {round((t+250)*1e6)}\n'))
        for sequence in range(1, 163):
            t = 1 + (sequence - 0.5) * 0.05
            packets.append((t + 0.25, 'measurement',
                f'PIX {s} {sequence} {round((t+100-0.001)*1e6)} {round((t+100+0.001)*1e6)} 800\n'))
        for arrival, source, line in sorted(packets):
            a.feed(source, line.encode(), arrival)
            a.poll(arrival)
        sweeps = [event[1] for event in output if event[0] == 'sync_sweep']
        self.assertGreaterEqual(len(sweeps), 1)
        for _, sequence, points, period in sweeps:
            self.assertAlmostEqual(period, 2)
            for point in points:
                expected = (-math.tau * ((point.timestamp - 1) % 2) / 2) % math.tau
                self.assertAlmostEqual(point.angle_rad, expected, places=6)


class SerialTests(unittest.TestCase):
    def test_read_does_not_wait_for_4096_bytes(self):
        class Serial:
            in_waiting = 4
            def read(self, size):
                self.size = size
                return b'1234'[:size]
        serial = Serial()
        self.assertEqual(_PySerialTransport(serial).read(4096), b'1234')
        self.assertEqual(serial.size, 4)

    @unittest.skipUnless(sys.platform == 'win32', 'Win32 backend')
    def test_native_backend_reads_available_bytes_or_first_byte(self):
        from serial_backend import _Win32SerialTransport
        class Kernel:
            waiting = 4
            def ClearCommError(self, handle, errors, state):
                state._obj.cbInQue = self.waiting
                return True
            def ReadFile(self, handle, buffer, size, count, overlapped):
                self.requested = size
                buffer.raw = b'x' * size
                count._obj.value = size
                return True
        transport = object.__new__(_Win32SerialTransport)
        transport.handle = 1
        transport.kernel32 = Kernel()
        self.assertEqual(transport.read(4096), b'xxxx')
        self.assertEqual(transport.kernel32.requested, 4)
        transport.kernel32.waiting = 0
        self.assertEqual(transport.read(4096), b'x')
        self.assertEqual(transport.kernel32.requested, 1)


class ConfigurationTests(unittest.TestCase):
    def test_valid_radar_calibration_and_optics_override_stale_navigation_copy(self):
        import navigation_app
        radar = types.SimpleNamespace(exists=lambda: True, read_text=lambda **kwargs: json.dumps(
            {'calibration': {'p0': 700, 'k': 100}, 'ccd_parser': 'raw2', 'exposure_index': 4}))
        nav = types.SimpleNamespace(exists=lambda: True, read_text=lambda **kwargs: json.dumps(
            {'calibration': {'p0': None, 'k': None}, 'measurement_mode': 'fffe', 'exposure_index': 8}))
        with patch.object(navigation_app, 'RADAR_CONFIG_PATH', radar), patch.object(navigation_app, 'NAV_CONFIG_PATH', nav):
            config = navigation_app.load_configuration()
        self.assertEqual(config['calibration']['k'], 100)
        self.assertEqual(config['measurement_mode'], 'raw2')
        self.assertEqual(config['exposure_index'], 5)


@unittest.skip("历史 MEASUREMENT_SYNC_V2 回归；当前设备基准由 test_firmware_baseline.py 校验")
class MeasurementFirmwareTests(unittest.TestCase):
    def setUp(self):
        self.tick = 0
        class Pin:
            OUT, IN, PULL_UP = 1, 0, 2
            def __init__(self, number, mode, pull=None, value=1):
                self.level = value
            def value(self, value=None):
                if value is not None: self.level = value
                return self.level
        class UART:
            def __init__(self, *args, **kwargs):
                self.rx = bytearray()
                self.writes = []
            def any(self): return len(self.rx)
            def read(self, count):
                data = bytes(self.rx[:count]); del self.rx[:count]; return data
            def write(self, data): self.writes.append(data); return len(data)
        modulus = 1 << 30
        utime = types.SimpleNamespace(ticks_us=lambda: self.tick % modulus,
            ticks_diff=lambda a,b: (a-b+modulus//2) % modulus-modulus//2)
        machine = types.SimpleNamespace(Pin=Pin, UART=UART)
        path = Path(__file__).resolve().parents[1] / 'esp32_measurement_sync.py'
        spec = importlib.util.spec_from_file_location('measurement_firmware_test', path)
        self.firmware = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'utime': utime, 'machine': machine}):
            spec.loader.exec_module(self.firmware)

    def test_running_sync_and_keepalive_preserve_capture_and_session(self):
        f = self.firmware
        f.handle(b'START abc 20 8 fffe', 0)
        f.poll()
        self.tick = 200000
        f.poll()
        capture = f.capture_begin
        f.handle(b'SYNC refresh', 201000)
        f.handle(b'PING', 202000)
        self.assertEqual(f.session, 'abc')
        self.assertEqual(f.capture_begin, capture)
        self.assertEqual(f.last_command, 202000)
        self.assertEqual(f.laser.value(), 1)

    def test_partial_ccd_frame_has_capture_interval(self):
        f = self.firmware
        f.handle(b'START abc 20 8 fffe', 0)
        f.poll()
        self.tick = 200000
        f.poll()
        self.tick = 202000
        f.ccd.rx.extend(b'\xff\xfe\x03')
        f.poll()
        self.tick = 204000
        f.ccd.rx.extend(b'\x20')
        f.poll()
        self.assertIn(b'PIX abc 1 200000 204000 800\r\n', f.lora.writes)

    def test_timeout_stops_instead_of_matching_next_response(self):
        f = self.firmware
        f.handle(b'START abc 20 8 fffe', 0)
        f.poll()
        self.tick = 200000
        f.poll()
        self.tick = 450001
        f.poll()
        self.assertEqual(f.session, '')
        self.assertIn(b'ERROR CCD_TIMEOUT\r\n', f.lora.writes)

    def test_late_complete_frame_cannot_bypass_capture_timeout(self):
        f = self.firmware
        f.handle(b'START abc 20 8 fffe', 0)
        f.poll()
        self.tick = 200000
        f.poll()
        self.tick = 450001
        f.ccd.rx.extend(b'\xff\xfe\x03\x20')
        f.poll()
        self.assertEqual(f.session, '')
        self.assertFalse(any(packet.startswith(b'PIX ') for packet in f.lora.writes))
        self.assertIn(b'ERROR CCD_TIMEOUT\r\n', f.lora.writes)

    def test_calibration_takes_one_frame_without_motor(self):
        f = self.firmware
        f.handle(b'CAL 8 raw2', 0)
        f.poll()
        self.tick = 200000
        f.poll()
        self.tick = 205000
        f.ccd.rx.extend(b'\x03\x20')
        f.poll()
        self.assertEqual(f.session, '')
        self.assertIn(b'PIX CAL 1 200000 205000 800\r\n', f.lora.writes)

    def test_pc_named_calibration_round_trip_through_firmware(self):
        import queue
        from radar_app import RadarApp
        from radar_core import MotorLineParser
        app = object.__new__(RadarApp)
        app.calibration_pending = False
        app.calibration_session = ''
        app.calibration_parser = MotorLineParser()
        app.scanning = False
        app.root = Mock()
        app.current_pixel_var = Mock()
        app.events = queue.Queue()
        app._collect_config = lambda: {'ccd_parser': 'raw2', 'exposure_index': 8}
        app.measure_endpoint = Mock(is_open=True)
        app.measure_endpoint.write_line.return_value = True
        with patch('radar_app.time.perf_counter', return_value=0):
            app.read_calibration_pixel()
        command = app.measure_endpoint.write_line.call_args.args[0]
        f = self.firmware
        f.lora.rx.extend((command + '\n').encode())
        f.poll()
        self.tick = 200000
        f.poll()
        self.tick = 205000
        f.ccd.rx.extend(b'\x03\x20')
        f.poll()
        self.tick = 1200000
        f.poll()
        self.assertEqual(f.session, '')
        self.assertEqual(f.laser.value(), 0)
        self.assertEqual(f.ccd.writes.count(b'@c0081#@'), 1)
        # A radio packet can be fragmented at any byte boundary.
        for byte in b''.join(f.lora.writes):
            app._measurement_data(bytes([byte]), 1.3)
        self.assertEqual(app.events.get_nowait()[:2], ('calibration_pixel', (app.calibration_session, 800)))
        self.assertTrue(app.events.empty())

    def test_late_raw2_tail_is_not_forwarded_after_timeout(self):
        f = self.firmware
        f.handle(b'START abc 20 8 raw2', 0)
        f.poll()
        self.tick = 200000
        f.poll()
        self.tick = 205000
        f.ccd.rx.extend(b'\x03')
        f.poll()
        self.tick = 450001
        f.poll()
        self.tick = 460000
        f.ccd.rx.extend(b'\x20')
        f.poll()
        self.assertEqual(f.lora.writes[-1], b'ERROR CCD_TIMEOUT\r\n')

    def test_legacy_ccd_command_still_forwards_binary_response(self):
        f = self.firmware
        f.handle(b'@c0081#@', 0)
        f.ccd.rx.extend(b'\x03\x20')
        f.poll()
        self.assertIn(b'\x03\x20', f.lora.writes)

    def test_extended_clock_survives_counter_wrap(self):
        f = self.firmware
        self.tick = (1 << 30) - 100
        f.clock.raw = self.tick
        f.clock.total = self.tick
        self.tick += 200
        self.assertEqual(f.clock.now(), self.tick)


if __name__ == '__main__':
    unittest.main()
