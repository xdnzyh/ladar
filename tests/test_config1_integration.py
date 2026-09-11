from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from chassis_config1 import DEFAULT_PROFILE, ROOT, crc, decode, encode, load_profile, values_crc
from chassis_controller import ChassisController, ChassisMotionAdapter, ChassisState
from navigation_core import VelocityCommand
from runtime_config import resolve_runtime_config, RuntimeConfigError


class FirmwareEndpoint:
    is_open = True

    def __init__(self):
        self.values, _ = load_profile(DEFAULT_PROFILE)
        self.staged = None
        self.owner = None
        self.writes = []
        self.replies = []
        self.drop = None
        self.corrupt = None
        self.drop_once = None
        self.time = 0.0

    def write_ticket(self, data, on_sent=None, on_written=None, **kwargs):
        self.writes.append(data)
        if on_sent:
            on_sent(self.time)
        line = data.strip().decode('ascii')
        body = None
        if line.startswith('@CFG,'):
            request, checksum = line.rsplit(',', 1)
            assert crc(request.encode()) == int(checksum, 16)
            _, op, tag, *numbers = request.split(',')
            args = tuple(map(int, numbers))
            if op == 'I':
                body = f'@CFG,I,{tag},1,95,{values_crc(self.values)}'
            elif op == 'G':
                i, = args
                body = f'@CFG,G,{tag},{i}' + ''.join(f',{x}' for x in self.values[i:i+8])
            elif op == 'B':
                self.staged = self.values.copy()
                self.owner = tag
                body = f'@CFG,B,{tag},{values_crc(self.values)}'
            elif op == 'S':
                assert self.owner == tag
                i, value = args
                self.staged[i] = value
                body = f'@CFG,S,{tag},{i},{value}'
            elif op == 'C':
                assert self.owner == tag
                assert values_crc(self.staged) == args[0]
                self.values = self.staged
                self.staged = None
                body = f'@CFG,C,{tag},{values_crc(self.values)}'
            elif op == 'A':
                self.staged = None
                body = f'@CFG,A,{tag},{values_crc(self.values)}'
            if op == self.drop:
                body = None
            if op == self.drop_once:
                self.drop_once = None
                body = None
            if body:
                reply = encode(body)
                if op == self.corrupt:
                    reply = reply[:-6] + b'0000\r\n'
                self.replies.append(reply)
        elif line.startswith('@PING,'):
            self.replies.append(line.replace('@PING,', '@PONG,').encode()+b'\r\n')
        if on_written:
            on_written(self.time)
        return object()

    def cancel_write(self, ticket):
        return 'started'


class Config1IntegrationTests(unittest.TestCase):
    def test_navigation_loads_terminal_profile_for_distance_control(self):
        from navigation_app import load_configuration

        profile = 'control/参数工具/底盘参数.json'
        values, coefficients = load_profile(profile)
        config = load_configuration('hardware', 'navigation')
        self.assertEqual(config['chassis_config1_file'], profile)
        self.assertEqual(config['chassis_config1_values'], values)
        self.assertEqual(config['chassis_config1_crc'], values_crc(values))
        self.assertEqual(config['chassis_preferred_translation_unit'], 'CNT')
        self.assertEqual(config['chassis_tx_padding_spaces'], 32)
        self.assertTrue(config['chassis_result_recovery'])
        self.assertTrue(config['chassis_idle_preflight'])
        for index, mode in enumerate('WSADQEZC'):
            entry = config['chassis_translation_capabilities'][mode]
            self.assertEqual(entry['counts_per_mm'], coefficients[mode])
            self.assertEqual(entry['coefficient_source'], profile)
            self.assertEqual(entry['brake_min_counts'], values[44 + 4*index])
            self.assertEqual(entry['brake_max_counts'], values[45 + 4*index])
            self.assertFalse(entry['motion_range_validated'])
        self.assertFalse(config['chassis_speed_validated'])
        self.assertFalse(config['chassis_braking_validated'])
        self.assertTrue(config['chassis_distance_control'])
        self.assertTrue(ChassisMotionAdapter(config).readiness()[0])

    def make_controller(self):
        self.endpoint = FirmwareEndpoint()
        self.events = []
        config = resolve_runtime_config('hardware', 'navigation', {
            'chassis_config1_file': DEFAULT_PROFILE,
            'chassis_tx_padding_spaces': 32,
            'chassis_firmware_confirmed': True,
            'chassis_capability_mode': 'mm_ping_v1',
        })

        def emit(kind, value, stamp):
            self.events.append((kind, value))
            if kind == 'chassis_frame':
                self.controller.handle_frame(*value, stamp)

        self.controller = ChassisController(self.endpoint, emit, config,
            clock=lambda: self.endpoint.time, nonce_factory=lambda: '12345678')
        self.generation = self.controller.begin_connection()
        self.controller.feed_data(b'IDLE X=0 Y=0 R=0 S=0\r\n', 0, self.generation)
        return self.controller

    def step(self, count=1):
        for _ in range(count):
            self.endpoint.time += 0.6
            self.controller.poll()
            replies, self.endpoint.replies = self.endpoint.replies, []
            for reply in replies:
                self.controller.feed_data(reply, self.endpoint.time, self.generation)

    def drain(self):
        for _ in range(250):
            self.step()
            if self.controller.config_exchange is None:
                return
        self.fail('CONFIG1 did not finish')

    def moves(self):
        return [data for data in self.endpoint.writes if data.lstrip().startswith(b'@MOVE,')]

    def test_read_default_apply_and_complete_readback(self):
        controller = self.make_controller()
        self.assertEqual(crc(b'123456789'), 0x29B1)
        self.assertEqual(values_crc(self.endpoint.values), controller.config['chassis_config1_crc'])
        self.assertTrue(controller.allow_automatic())
        self.assertTrue(controller.request_config_sync(apply=False))
        self.assertFalse(controller.request_communication_check())
        self.assertFalse(controller.request_status())
        self.assertFalse(controller.request_move('D', 100, operator_authorized=True))
        self.drain()
        self.assertTrue(controller.config_verified)
        self.assertTrue(controller.allow_automatic())
        candidate = deepcopy(controller.config)
        candidate['chassis_config1_values'][19] = 550  # LAT_RAMP_MS
        self.assertTrue(controller.update_config(candidate))
        self.assertTrue(controller.request_config_sync())
        self.drain()
        self.assertEqual(self.endpoint.values[19], 550)
        self.assertTrue(controller.config_verified)
        self.assertFalse(self.moves())
        self.assertTrue(all(data.startswith(b' '*32) for data in self.endpoint.writes))
        self.assertTrue(all(len(data.strip()) <= 47 for data in self.endpoint.writes))

    def test_matching_preflight_sends_once_and_done_uses_host_coefficient(self):
        controller = self.make_controller()
        controller.config['chassis_translation_capabilities']['D']['uncertainty_m'] = 0.01
        adapter = ChassisMotionAdapter(controller.config)
        request = adapter.request_for_manual('D', 100, 'MM')
        self.assertEqual((request.unit, request.request_value), ('CNT', 739))
        self.assertTrue(controller.request_config_sync(apply=False))
        self.drain()
        self.endpoint.writes.clear()
        self.assertTrue(controller.request_move(request, operator_authorized=True))
        self.step(12)
        self.assertEqual(self.moves(), [b' '*32+b'@MOVE,D,739,CNT\r\n'])
        done = b'@DONE,D,TARGET,REQ=739,UNIT=CNT,BRAKE=700,ENC=748,DX=0,DY=-748,DR=0,DS=0,Q1=-748,Q2=748,Q3=-748,Q4=748\r\n'
        controller.feed_data(done, self.endpoint.time, self.generation)
        self.assertEqual(controller.state, ChassisState.SETTLING)
        report = [value[2] for kind, value in self.events if kind == 'chassis_done'][-1]
        self.assertAlmostEqual(adapter.execution_from_report(report).local_x_m, 748/7.3864/1000)
        self.assertTrue(controller.complete_settle(resume_auto=False))
        self.assertTrue(controller.request_move(request, operator_authorized=True))
        self.step(4)
        self.assertEqual(len(self.moves()), 2)
        self.assertFalse(any(b'@CFG,' in wire for wire in self.endpoint.writes))

    def test_failed_sync_is_not_remembered_and_does_not_lock_motion(self):
        for failure in ('mismatch', 'corrupt', 'drop'):
            with self.subTest(failure=failure):
                controller = self.make_controller()
                if failure == 'mismatch':
                    self.endpoint.values[19] = 550
                else:
                    setattr(self.endpoint, failure, 'I')
                request = ChassisMotionAdapter(controller.config).request_for_manual('Q', 100, 'MM')
                self.assertTrue(controller.request_config_sync(apply=False))
                self.drain()
                self.assertFalse(self.moves())
                self.assertIsNone(controller.pending)
                self.assertFalse(controller.config_verified)
                self.assertFalse(controller.initial_sync_completed)
                self.assertTrue(controller.request_move(request, operator_authorized=True))

    def test_reload_same_execution_values_keeps_cache_but_changes_invalidate(self):
        controller = self.make_controller()
        self.assertTrue(controller.request_config_sync(apply=False))
        self.drain()
        changed = deepcopy(controller.config)
        changed['chassis_translation_capabilities']['D']['counts_per_mm'] = 8.0
        self.assertTrue(controller.update_config(changed))
        self.assertTrue(controller.config_verified)
        changed = deepcopy(controller.config)
        changed['chassis_config1_values'][19] = 700
        self.assertTrue(controller.update_config(changed))
        self.assertFalse(controller.config_verified)

    def test_restart_during_ping_cancels_move_without_querying_parameters(self):
        controller = self.make_controller()
        self.assertTrue(controller.request_config_sync(apply=False))
        self.drain()
        self.endpoint.writes.clear()
        request = ChassisMotionAdapter(controller.config).request_for_manual('D', 100, 'MM')
        self.assertTrue(controller.request_move(request, operator_authorized=True))
        self.step()
        controller.feed_data(b'MECANUM UNIVERSAL V6.3 COMM READY\r\n', self.endpoint.time, self.generation)
        self.step(10)
        self.assertFalse(self.moves())
        self.assertFalse(controller.config_verified)
        self.assertFalse(any(b'@CFG,' in wire for wire in self.endpoint.writes))

    def test_lost_commit_reply_reads_back_without_repeating_commit_or_moving(self):
        controller = self.make_controller()
        controller.config['chassis_config1_values'][19] = 550
        self.endpoint.drop = 'C'
        self.assertTrue(controller.request_config_sync())
        self.drain()
        self.assertEqual(self.endpoint.values[19], 550)
        self.assertTrue(controller.config_verified)
        self.assertEqual(sum(b'@CFG,C,' in data for data in self.endpoint.writes), 1)
        self.assertFalse(self.moves())
        self.step()  # consume the best-effort abort reply
        self.endpoint.drop = None
        self.assertTrue(controller.request_config_sync(apply=False))
        self.drain()
        self.assertTrue(controller.config_verified)

    def test_single_lost_read_or_staging_reply_recovers_with_correct_tags(self):
        for op in ('I', 'G', 'B', 'S'):
            with self.subTest(op=op):
                controller = self.make_controller()
                controller.config['chassis_config1_values'][19] = 550
                self.endpoint.drop_once = op
                self.assertTrue(controller.request_config_sync())
                self.drain()
                self.assertTrue(controller.config_verified)
                requests = [wire.strip().decode().split(',') for wire in self.endpoint.writes
                            if wire.lstrip().startswith(('@CFG,' + op + ',').encode())]
                self.assertGreaterEqual(len(requests), 2)
                if op in ('I', 'G'):
                    self.assertNotEqual(requests[0][2], requests[1][2])
                else:
                    self.assertEqual(requests[0], requests[1])
                self.assertEqual(sum(b'@CFG,C,' in wire for wire in self.endpoint.writes), 1)
                self.assertFalse(self.moves())

    def test_read_retries_are_bounded_and_rejection_is_final(self):
        controller = self.make_controller()
        self.endpoint.drop = 'I'
        self.assertTrue(controller.request_config_sync())
        self.drain()
        self.assertEqual(sum(b'@CFG,I,' in wire for wire in self.endpoint.writes), 3)
        self.assertFalse(controller.config_verified)
        controller = self.make_controller()
        self.endpoint.drop = 'I'
        self.assertTrue(controller.request_config_sync())
        self.step()
        tag = controller.config_exchange.request_tag
        controller.feed_data(encode(f'@CFG,E,{tag},1'), self.endpoint.time, self.generation)
        self.assertIsNone(controller.config_exchange)
        self.assertFalse(controller.config_verified)
        self.assertEqual(sum(b'@CFG,I,' in wire for wire in self.endpoint.writes), 1)

    def test_distance_control_planner_protected_result_and_scan_gate(self):
        from navigation_app import load_configuration
        from test_result_recovery import result
        from runtime_config import build_navigation_engine
        from navigation_core import ScanPoint
        import math

        controller = self.make_controller()
        config = load_configuration('hardware', 'navigation')
        self.assertIsNone(config['safety_speed_upper_bound_mps'])
        self.assertIsNone(config['safety_stop_distance_m'])
        self.assertTrue(controller.update_config(config))
        self.assertTrue(controller.allow_automatic())
        self.assertTrue(controller.request_config_sync())
        self.drain()
        navigator = build_navigation_engine(config)
        navigator.set_auto(True)
        navigator.grid.log_odds[:] = [-6.0] * len(navigator.grid.log_odds)
        navigator.match_score = 0.95
        navigator.latest_scan = [ScanPoint(math.tau*i/48, 1.0) for i in range(48)]
        start = navigator.grid.world_to_cell(0.0, 0.0)
        navigator.path_cells = [(start[0], start[1]-i) for i in range(8)]
        adapter = ChassisMotionAdapter(config)
        request = adapter.request_for_command(navigator._command_along_path())
        self.assertEqual(request.unit, 'CNT')
        self.assertTrue(controller.allow_automatic())
        self.assertTrue(controller.request_move(request, source='auto'))
        self.step(2)
        self.assertEqual(len(self.moves()), 1)
        self.assertIn(b',12345678,', self.moves()[0])
        wire = result('12345678', request.mode, value=request.request_value,
                      enc=request.request_value)
        controller.feed_data(wire, self.endpoint.time, self.generation)
        controller.feed_data(wire, self.endpoint.time, self.generation)
        reports = [value[2] for kind, value in self.events if kind == 'chassis_done']
        self.assertEqual(len(reports), 1)
        estimate = adapter.execution_from_report(reports[0])
        self.assertEqual(estimate.uncertainty_m, config['map_resolution_m'])
        navigator.apply_execution_delta(estimate.local_x_m, estimate.local_y_m,
            estimate.yaw_rad, estimate.uncertainty_m, estimate.uncertainty_rad)
        self.assertGreater(navigator.pose.y, 0)
        self.assertTrue(controller.complete_settle(resume_auto=True))
        self.assertEqual(controller.state, ChassisState.WAITING_SCAN)
        self.assertFalse(controller.request_move(request, source='auto'))
        navigator.process_scan([ScanPoint(math.tau*i/72, 0.65) for i in range(72)])
        self.assertTrue(controller.mark_scan_ready())
        self.assertEqual(controller.state, ChassisState.IDLE)
        self.assertEqual(len(self.moves()), 1)

    def test_distance_control_directions_limits_and_navigation_preflight(self):
        from navigation_app import NavigationApp, load_configuration
        from chassis_controller import MotionConversionError
        from runtime_config import build_navigation_engine
        from unittest.mock import Mock

        config = load_configuration('hardware', 'navigation')
        adapter = ChassisMotionAdapter(config)
        navigator = build_navigation_engine(config)
        for mode in 'WSADQEZC':
            with self.subTest(mode=mode):
                self.assertTrue(adapter.readiness(mode)[0])
                self.assertTrue(navigator.translation_capabilities[mode]['enabled'])
                request = adapter.request_for_manual(mode, 100, 'MM')
                command = NavigationApp._command_for_chassis_request(request)
                auto_request = adapter.request_for_command(command)
                self.assertEqual(auto_request.mode, mode)
                self.assertEqual(auto_request.unit, 'CNT')
                self.assertLessEqual(abs(auto_request.target - 0.1), 0.001)
        for mode in 'RF':
            self.assertFalse(adapter.readiness(mode)[0])
        with self.assertRaises(MotionConversionError):
            adapter.request_for_command(VelocityCommand(forward_mps=1, duration_s=1))
        app = NavigationApp.__new__(NavigationApp)
        app.source = 'hardware'
        app.config = config
        app.connected = True
        app.calibration = Mock(ready=True)
        app.chassis_adapter = adapter
        app.chassis_endpoint = Mock(is_open=True)
        app.chassis_controller = Mock(confirmed=True, config_verified=True)
        app.chassis_controller.allow_automatic.return_value = True
        self.assertTrue(app._navigation_preflight())
        app.chassis_controller.config_verified = False
        self.assertTrue(app._navigation_preflight())
        app.chassis_controller.config_verified = True
        app.chassis_controller.allow_automatic.return_value = False
        self.assertFalse(app._navigation_preflight())

    def test_parameter_reload_preserves_current_map_and_snapshot(self):
        from navigation_app import NavigationApp
        from runtime_config import build_navigation_engine
        from unittest.mock import Mock
        import threading

        controller = self.make_controller()
        app = NavigationApp.__new__(NavigationApp)
        app.config = controller.config
        app.chassis_controller = controller
        app.chassis_adapter = ChassisMotionAdapter(app.config)
        app.motion_safety = Mock()
        app.sync = Mock()
        app.navigator = build_navigation_engine(app.config)
        app.grid = app.navigator.grid
        app.grid.log_odds[0] = 4.25
        grid = app.grid
        app.mapping_snapshot = snapshot = object()
        app.mapping_generation = 12
        app.mapping_lock = threading.RLock()
        app._clear_mapping_tasks = Mock()
        app.manual_unit_var = Mock()
        app.chassis_capability_var = Mock()
        app._chassis_capability_summary = Mock(return_value='CONFIG1')
        app._activate_chassis_config(app.config, reset_mapping=False)
        self.assertIs(app.grid, grid)
        self.assertEqual(grid.log_odds[0], 4.25)
        self.assertIs(app.mapping_snapshot, snapshot)
        self.assertEqual(app.mapping_generation, 12)
        app._clear_mapping_tasks.assert_not_called()

    def test_restart_stop_disconnect_clear_config_permission(self):
        controller = self.make_controller()
        controller.config_verified = True
        controller.feed_data(b'MECANUM UNIVERSAL V6.3 COMM READY\r\n', 0, self.generation)
        self.assertFalse(controller.config_verified)
        controller.feed_data(b'IDLE X=0 Y=0 R=0 S=0\r\n', 0, self.generation)
        self.assertTrue(controller.request_config_sync())
        self.step(2)
        self.assertTrue(controller.request_stop())
        self.assertIsNone(controller.config_exchange)
        self.step(10)
        self.assertFalse(self.moves())
        controller.disconnect()
        self.assertFalse(controller.config_verified)

    def test_idle_stop_keeps_verified_parameters_until_transport_failure(self):
        controller = self.make_controller()
        self.assertTrue(controller.request_config_sync(apply=False))
        self.drain()
        self.assertTrue(controller.config_verified)
        self.assertTrue(controller.request_stop())
        self.assertTrue(controller.request_stop())
        self.assertTrue(controller.config_verified)
        self.assertFalse(controller.allow_automatic())
        controller.feed_data(b'IDLE X=0 Y=0 R=0 S=0\r\n', self.endpoint.time, self.generation)
        self.assertTrue(controller.config_verified)
        self.assertTrue(controller.allow_automatic())
        controller.handle_transport_error(self.generation, 'connection lost', self.endpoint.time)
        self.assertFalse(controller.config_verified)

    def test_app_syncs_once_then_skips_reconnection_and_board_restart(self):
        from navigation_app import NavigationApp
        from unittest.mock import Mock

        controller = self.make_controller()
        app = NavigationApp.__new__(NavigationApp)
        app.config = controller.config
        app.chassis_controller = controller
        original = controller.request_config_sync
        controller.request_config_sync = Mock(wraps=original)
        app._verify_chassis_on_idle(self.generation - 1, ChassisState.IDLE)
        controller.request_config_sync.assert_not_called()
        app._verify_chassis_on_idle(self.generation, ChassisState.IDLE)
        self.drain()
        self.assertTrue(controller.config_verified)
        app._verify_chassis_on_idle(self.generation, ChassisState.IDLE)
        controller.request_config_sync.assert_called_once_with(apply=True)
        self.assertTrue(controller.initial_sync_completed)
        controller.feed_data(b'MECANUM UNIVERSAL V6.3 COMM READY\r\n', self.endpoint.time, self.generation)
        app._verify_chassis_on_idle(self.generation, ChassisState.CONNECTED_WAITING)
        controller.feed_data(b'IDLE X=0 Y=0 R=0 S=0\r\n', self.endpoint.time, self.generation)
        self.endpoint.values[19] = 550
        app._verify_chassis_on_idle(self.generation, ChassisState.IDLE)
        self.drain()
        self.assertFalse(controller.config_verified)
        app._verify_chassis_on_idle(self.generation, ChassisState.IDLE)
        self.assertEqual(controller.request_config_sync.call_count, 1)
        controller.disconnect()
        self.generation = controller.begin_connection()
        controller.feed_data(b'IDLE X=0 Y=0 R=0 S=0\r\n', self.endpoint.time, self.generation)
        app._verify_chassis_on_idle(self.generation, ChassisState.IDLE)
        self.assertEqual(controller.request_config_sync.call_count, 1)
        self.assertTrue(controller.allow_automatic())
        self.assertFalse(self.moves())

    def test_external_profile_changes_manual_auto_and_timeout(self):
        values, coefficients = load_profile(DEFAULT_PROFILE)
        coefficients['D'] = 8.0
        values[76] = 15000
        with patch('chassis_config1.load_profile', return_value=(values, coefficients)):
            config = resolve_runtime_config('hardware', 'navigation', {
                'chassis_config1_file': DEFAULT_PROFILE,
                'chassis_firmware_confirmed': True,
                'chassis_capability_mode': 'mm_ping_v1',
            })
        adapter = ChassisMotionAdapter(config)
        self.assertEqual(adapter.request_for_manual('D', 100, 'MM').request_value, 800)
        request = adapter.request_for_command(VelocityCommand(0, 0.1, 0, 1), automatic=False)
        self.assertEqual((request.unit, request.request_value), ('CNT', 800))
        self.assertEqual(config['chassis_total_timeout_s'], 21)

    def test_invalid_profile_does_not_silently_use_defaults(self):
        with self.assertRaises(RuntimeConfigError):
            resolve_runtime_config('hardware', 'navigation', {'chassis_config1_file': 'missing.json'})
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            path = Path(directory)/'parameters.json'
            data = json.loads((ROOT/DEFAULT_PROFILE).read_text(encoding='utf-8-sig'))
            data['parameters']['LAT_RAMP_MS'] = '500'
            path.write_text(json.dumps(data), encoding='utf-8')
            with self.assertRaises(ValueError):
                load_profile(str(path.relative_to(ROOT)))


if __name__ == '__main__':
    unittest.main()
