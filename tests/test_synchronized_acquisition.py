import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
import json
from unittest.mock import patch

from data_fusion import parse_trigger
from radar_core import CalibrationModel, RotationTracker
from serial_backend import _PySerialTransport
from synchronized_acquisition import ClockEstimate, SynchronizedAcquisition, TimedSweepBuilder
from navigation_core import NavigationEngine, ScanPoint


class Endpoint:
    def __init__(self):
        self.messages = []
        self.now = 0.0

    def write_line(self, line, on_sent=None):
        self.messages.append(line)
        if on_sent:
            on_sent(self.now)
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
        self.assertAlmostEqual(clock.map(1_000_000, 10)[1], 0.006)

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
        self.assertIn('时间配准误差', builder.reason)

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

    def test_densification_does_not_invent_large_missing_sector(self):
        points = [ScanPoint(0, 1), ScanPoint(math.pi, 1)]
        self.assertEqual(NavigationEngine._densify_for_mapping(points), points)


class AcquisitionTests(unittest.TestCase):
    def make(self):
        endpoints = [Endpoint(), Endpoint()]
        output = []
        acquisition = SynchronizedAcquisition(*endpoints, CalibrationModel(p0=700, k=100),
            {'clock_drift_bound_ppm': 0, 'irq_timestamp_uncertainty_ms': 0},
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
        for i in range(8):
            now = 0.2 + i * 0.1
            for endpoint in endpoints:
                endpoint.now = now
            a.poll(now)
            for source in a.endpoints:
                token = a.outstanding[source][0]
                a.feed(source, f'SYNC {token} {round((now+0.002)*1e6)} {round((now+0.003)*1e6)}\n'.encode(), now+0.005)
            a.poll(now+0.005)
        self.assertEqual(a.state, 'running')
        self.assertTrue(endpoints[0].messages[-1].startswith('START '))
        self.assertTrue(endpoints[1].messages[-1].startswith('ROT '))

    def test_rotation_waits_for_delayed_measurements(self):
        a, _, output = self.running()
        a.last_arrival['measurement'] = 0.2
        s = a.session
        a.feed('rotation', f'TRIG {s} 1 1000000\nTRIG {s} 2 2000000\n'.encode(), 2.01)
        a.poll(2.01)
        self.assertIsNone(a.builder.anchor)
        a.feed('measurement', f'PIX {s} 1 1499000 1501000 800\nPIX {s} 2 2099000 2101000 800\n'.encode(), 2.1)
        a.poll(2.1)
        self.assertEqual(a.builder.anchor[2], 2)
        self.assertFalse(any(event[0] == 'sync_sweep' for event in output))

    def test_old_session_ignored(self):
        a, _, _ = self.running()
        a.feed('rotation', b'TRIG old-session 1 1000000\n', 1)
        a.poll(1)
        self.assertEqual(a.pending, [])

    def test_sequence_gap_discards_partial_sweep(self):
        a, _, _ = self.running()
        s = a.session
        a.builder.trigger(0, 0, 1)
        a.builder.sample(0.2, 0, 800, 1)
        a._line('measurement', f'PIX {s} 1 499000 501000 800', 0.51)
        a._line('measurement', f'PIX {s} 3 699000 701000 800', 0.71)
        self.assertIsNone(a.builder.anchor)
        self.assertEqual(a.builder.samples, [])

    def test_restart_aborts_scan(self):
        a, _, output = self.running()
        a.feed('rotation', b'READY ROTATION_SYNC_V1\n', 1)
        a.poll(1)
        self.assertEqual(a.state, 'stopped')
        self.assertTrue(any(item[0] == 'sync_error' for item in output))

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
                expected = math.tau * ((point.timestamp - 1) % 2) / 2
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
        self.assertEqual(config['exposure_index'], 4)


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

    def test_extended_clock_survives_counter_wrap(self):
        f = self.firmware
        self.tick = (1 << 30) - 100
        f.clock.raw = self.tick
        f.clock.total = self.tick
        self.tick += 200
        self.assertEqual(f.clock.now(), self.tick)


if __name__ == '__main__':
    unittest.main()
