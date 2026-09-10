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
        self.assertEqual(values_crc(self.endpoint.values), 5809)
        self.assertFalse(controller.allow_automatic())
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
        self.assertFalse(controller.request_move(request, operator_authorized=True))
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

    def test_mismatch_damaged_reply_and_silence_each_prevent_move(self):
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
                self.assertFalse(controller.request_move(request, operator_authorized=True))
                self.assertFalse(self.moves())
                self.assertIsNone(controller.pending)
                self.assertFalse(controller.config_verified)

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

    def test_lost_commit_reply_requires_new_read_and_never_moves(self):
        controller = self.make_controller()
        controller.config['chassis_config1_values'][19] = 550
        self.endpoint.drop = 'C'
        self.assertTrue(controller.request_config_sync())
        self.drain()
        self.assertEqual(self.endpoint.values[19], 550)
        self.assertFalse(controller.config_verified)
        self.assertFalse(self.moves())
        self.step()  # consume the best-effort abort reply
        self.endpoint.drop = None
        self.assertTrue(controller.request_config_sync(apply=False))
        self.drain()
        self.assertTrue(controller.config_verified)

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
