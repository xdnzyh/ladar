import math
import unittest

from chassis_controller import ChassisMotionAdapter, ChassisController, ChassisState, MotionConversionError
from chassis_protocol import (
    ChassisStreamParser,
    ChassisProtocolError,
    STOP_SEQUENCE,
    encode_move,
    parse_protocol_line,
    validate_done_kinematics,
)
from navigation_core import VelocityCommand


DONE_D = (
    "@DONE,D,TARGET,REQ=1400,UNIT=CNT,BRAKE=1402.50,ENC=1481.75,"
    "DX=-28.75,DY=-1481.75,DR=-37.25,DS=3.25,Q1=-1470,Q2=1419,Q3=-1551,Q4=1487"
)


class FakeEndpoint:
    is_open = True

    def __init__(self):
        self.writes = []
        self.cancelled = 0

    def write(self, data, on_sent=None, on_written=None, priority=False):
        self.writes.append(bytes(data))
        if on_sent:
            on_sent(1.0)
        if on_written:
            on_written(1.0)
        return True

    def cancel_pending(self):
        self.cancelled += 1
        return 1


def ready_config():
    entry = {
        "status": "validated",
        "counts_per_m": 1000.0,
        "counts_per_rad": 1000.0,
        "uncertainty_m": 0.01,
        "uncertainty_rad": 0.02,
    }
    return {
        "chassis_speed_validated": True,
        "chassis_braking_validated": True,
        "safety_speed_upper_bound_mps": 0.2,
        "safety_stop_distance_m": 0.04,
        "chassis_calibration": {mode: dict(entry) for mode in "WSADQEZCRF"},
    }


class ChassisProtocolTests(unittest.TestCase):
    def test_move_encoding_is_fixed_current_firmware_format(self):
        self.assertEqual(encode_move("d", 1400), b"@MOVE,D,1400,CNT\r\n")
        with self.assertRaises(ChassisProtocolError):
            encode_move("W", 0)

    def test_split_lines_and_debug_text_do_not_confuse_protocol_frames(self):
        parser = ChassisStreamParser()
        first = parser.feed(b"START MODE=D\r\n@ACK,D,1400,C")
        second = parser.feed(b"NT\r\n" + DONE_D.encode("ascii") + b"\r\n")
        self.assertEqual([frame.kind for frame in first], ["diagnostic"])
        self.assertEqual([frame.kind for frame in second], ["ack", "done"])
        self.assertEqual(second[0].value.counts, 1400)
        self.assertTrue(validate_done_kinematics(second[1].value)[0])

    def test_malformed_done_is_bounded_and_rejected(self):
        parser = ChassisStreamParser(max_line_bytes=64)
        frames = parser.feed(("@DONE,D,TARGET," + "X" * 100 + "\r\n").encode("ascii"))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].kind, "invalid")
        self.assertIn("长度", frames[0].error)
        bad = parse_protocol_line("@DONE,D,TARGET,REQ=1,UNIT=CNT,REQ=2")
        self.assertEqual(bad.kind, "invalid")

    def test_stop_sequence_does_not_complete_a_truncated_move(self):
        valid = b"@MOVE,W,1400,CNT\r\n"

        def firmware_parse(data):
            receiving = False
            line = bytearray()
            started = 0
            emergency = 0
            overflow = 0
            for byte in data:
                char = chr(byte)
                if receiving:
                    if char in "\r\n":
                        if line.startswith(b"@MOVE,") and line.count(b",") == 3 and not line.endswith(b"!"):
                            started += 1
                        receiving = False
                        line.clear()
                    elif len(line) >= 47:
                        receiving = False
                        line.clear()
                        overflow += 1
                    else:
                        line.append(byte)
                    continue
                if char == "@":
                    receiving = True
                    line = bytearray(b"@")
                elif char.lower() == "x":
                    emergency += 1
            return started, emergency, overflow

        self.assertEqual(firmware_parse(STOP_SEQUENCE), (0, 1, 0))
        for cut in range(1, len(valid) - 1):
            self.assertEqual(firmware_parse(valid[:cut] + STOP_SEQUENCE), (0, 1, 0), cut)
        for buffered_length in range(1, 48):
            prefix = b"@" + b"A" * (buffered_length - 1)
            self.assertEqual(firmware_parse(prefix + STOP_SEQUENCE)[:2], (0, 1), buffered_length)
        self.assertEqual(firmware_parse(b"@" + b"A" * 64 + STOP_SEQUENCE)[:2], (0, 1))


class ChassisControllerTests(unittest.TestCase):
    def setUp(self):
        self.endpoint = FakeEndpoint()
        self.events = []
        self.now = 0.0
        self.controller = ChassisController(
            self.endpoint,
            lambda kind, value, stamp: self.events.append((kind, value, stamp)),
            ready_config(),
            clock=lambda: self.now,
        )
        self.generation = self.controller.begin_connection()
        self.controller.handle_frame(self.generation, parse_protocol_line("MECANUM UNIVERSAL V6.3 COMM READY"), self.now)
        self.controller.allow_automatic()

    def test_ack_done_lifecycle_applies_one_action_only(self):
        self.assertTrue(self.controller.request_move("W", 100, source="auto", now=0.0))
        self.assertEqual(self.controller.state, ChassisState.WAITING_ACK)
        self.controller.handle_frame(self.generation, parse_protocol_line("@ACK,W,100,CNT"), 0.1)
        self.assertEqual(self.controller.state, ChassisState.WAITING_DONE)
        done = "@DONE,W,TARGET,REQ=100,UNIT=CNT,BRAKE=100,ENC=100,DX=100,DY=0,DR=0,DS=0,Q1=100,Q2=100,Q3=100,Q4=100"
        frame = parse_protocol_line(done)
        self.controller.handle_frame(self.generation, frame, 0.2)
        self.assertEqual(self.controller.state, ChassisState.SETTLING)
        self.assertEqual(len([item for item in self.events if item[0] == "chassis_done"]), 1)
        self.assertTrue(self.controller.complete_settle(resume_auto=True, now=0.4))
        self.assertEqual(self.controller.state, ChassisState.WAITING_SCAN)
        self.assertTrue(self.controller.mark_scan_ready(1.0))
        self.assertEqual(self.controller.state, ChassisState.IDLE)
        self.controller.handle_frame(self.generation, frame, 1.1)
        self.assertEqual(len([item for item in self.events if item[0] == "chassis_done"]), 1)

    def test_done_before_ack_locks_control(self):
        self.assertTrue(self.controller.request_move("W", 100, source="auto", now=0.0))
        frame = parse_protocol_line("@DONE,W,TARGET,REQ=100,UNIT=CNT,BRAKE=100,ENC=100,DX=100,DY=0,DR=0,DS=0,Q1=100,Q2=100,Q3=100,Q4=100")
        self.controller.handle_frame(self.generation, frame, 0.1)
        self.assertEqual(self.controller.state, ChassisState.UNKNOWN)
        self.assertFalse(self.controller.automatic_ready)

    def test_timeout_sends_stop_and_never_repeats_move(self):
        self.assertTrue(self.controller.request_move("W", 100, source="auto", now=0.0))
        self.now = 3.0
        self.controller.poll()
        self.assertEqual(self.endpoint.writes[0], b"@MOVE,W,100,CNT\r\n")
        self.assertEqual(self.endpoint.writes[-1], STOP_SEQUENCE)
        self.assertEqual(self.endpoint.writes.count(b"@MOVE,W,100,CNT\r\n"), 1)
        self.assertEqual(self.controller.state, ChassisState.STOPPING)

    def test_stop_keeps_action_until_done(self):
        self.assertTrue(self.controller.request_move("D", 100, source="manual", now=0.0))
        self.controller.request_stop(reason="用户停止", now=0.2)
        self.assertEqual(self.controller.state, ChassisState.STOPPING)
        self.assertIsNotNone(self.controller.pending)
        done = "@DONE,D,EMERGENCY,REQ=100,UNIT=CNT,BRAKE=20,ENC=25,DX=-1,DY=-25,DR=-1,DS=1,Q1=-25,Q2=24,Q3=-26,Q4=24"
        self.controller.handle_frame(self.generation, parse_protocol_line(done), 0.5)
        self.assertEqual(self.controller.state, ChassisState.SETTLING)


class ChassisMotionAdapterTests(unittest.TestCase):
    def test_cardinal_diagonal_and_rotation_conversion(self):
        adapter = ChassisMotionAdapter(ready_config())
        self.assertEqual(adapter.request_for_command(VelocityCommand(forward_mps=0.1, duration_s=1)).mode, "W")
        diagonal = VelocityCommand(forward_mps=0.1 / math.sqrt(2), right_mps=-0.1 / math.sqrt(2), duration_s=1)
        self.assertEqual(adapter.request_for_command(diagonal).mode, "Q")
        self.assertEqual(adapter.request_for_command(VelocityCommand(yaw_rps=-0.1, duration_s=1)).mode, "F")
        with self.assertRaises(MotionConversionError):
            adapter.request_for_command(VelocityCommand(forward_mps=0.1, right_mps=0.02, duration_s=1))

    def test_report_conversion_uses_bottom_left_sign_once(self):
        adapter = ChassisMotionAdapter(ready_config())
        report = parse_protocol_line(DONE_D).value
        estimate = adapter.execution_from_report(report)
        self.assertAlmostEqual(estimate.local_x_m, 1.48175)
        self.assertAlmostEqual(estimate.local_y_m, 0.0)

    def test_unvalidated_physical_data_blocks_automatic_only(self):
        config = ready_config()
        config["chassis_calibration"]["W"] = {"status": "uncalibrated"}
        adapter = ChassisMotionAdapter(config)
        command = VelocityCommand(forward_mps=0.1, duration_s=1)
        with self.assertRaises(MotionConversionError):
            adapter.request_for_command(command, automatic=True)


if __name__ == "__main__":
    unittest.main()
