import math
from pathlib import Path
import unittest

from chassis_controller import (
    ChassisController,
    ChassisMotionAdapter,
    ChassisMoveRequest,
    ChassisState,
    MotionConversionError,
)
from chassis_protocol import (
    ChassisProtocolError,
    ChassisStreamParser,
    EXPECTED_STARTUP_MARKERS,
    STOP_SEQUENCE,
    encode_move,
    firmware_mm_to_counts,
    parse_protocol_line,
    validate_done_kinematics,
)
from navigation_core import VelocityCommand
from runtime_config import resolve_runtime_config


COUNTS_100_MM = {
    "W": 718, "S": 713, "A": 724, "D": 739,
    "Q": 1018, "E": 912, "Z": 965, "C": 1019,
}
CLOCK = [0.0]


class FakeTicket:
    def __init__(self, number):
        self.write_id = number


class FakeEndpoint:
    is_open = True

    def __init__(self, *, auto_complete=True):
        self.writes = []
        self.records = []
        self.cancelled = []
        self.auto_complete = auto_complete
        self.callbacks = []

    def write_ticket(
        self, data, on_sent=None, on_written=None, priority=False,
        on_failed=None, on_cancelled=None, discard_pending=False,
    ):
        payload = bytes(data)
        ticket = FakeTicket(len(self.writes) + 1)
        self.writes.append(payload)
        self.records.append((payload, priority, discard_pending, ticket))
        self.callbacks.append((on_sent, on_written, on_failed, on_cancelled))
        if self.auto_complete:
            if on_sent:
                on_sent(CLOCK[0])
            if on_written:
                on_written(CLOCK[0])
        return ticket

    def cancel_write(self, ticket):
        self.cancelled.append(ticket)
        return "cancelled"

    def flush(self, _timeout):
        return True


def ready_config(*, unit="MM", capability="mm_ping_v1"):
    config = resolve_runtime_config("hardware", "navigation", {})
    config["chassis_firmware_confirmed"] = True
    config["chassis_capability_mode"] = capability
    config["chassis_preferred_translation_unit"] = unit
    config["chassis_speed_validated"] = True
    config["chassis_braking_validated"] = True
    config["safety_speed_upper_bound_mps"] = 0.30
    config["safety_stop_distance_m"] = 0.05
    for entry in config["chassis_translation_capabilities"].values():
        entry["motion_range_validated"] = True
        entry["validated_min_mm"] = 20.0
        entry["validated_max_mm"] = 200.0
        entry["uncertainty_m"] = 0.015
    return resolve_runtime_config("hardware", "navigation", config)


def done_line(mode="W", value=100, unit="MM", reason="TARGET", enc=None):
    target = COUNTS_100_MM.get(mode, value) if unit == "MM" else None
    if enc is None:
        enc = target if target is not None else value
    patterns = {
        "W": (enc, enc, enc, enc), "S": (-enc, -enc, -enc, -enc),
        "A": (enc, -enc, enc, -enc), "D": (-enc, enc, -enc, enc),
        "Q": (enc, 0, enc, 0), "E": (0, enc, 0, enc),
        "Z": (0, -enc, 0, -enc), "C": (-enc, 0, -enc, 0),
        "R": (-enc, enc, enc, -enc), "F": (enc, -enc, -enc, enc),
    }
    q1, q2, q3, q4 = patterns[mode]
    dx = (q1 + q2 + q3 + q4) / 4
    dy = (q1 - q2 + q3 - q4) / 4
    dr = (-q1 + q2 + q3 - q4) / 4
    ds = (q1 + q2 - q3 - q4) / 4
    fields = [f"REQ={value}", f"UNIT={unit}"]
    if target is not None:
        fields.append(f"TARGET_CNT={target}")
    fields.extend([
        f"BRAKE={float(enc):.2f}", f"ENC={float(enc):.2f}",
        f"DX={dx:.2f}", f"DY={dy:.2f}", f"DR={dr:.2f}", f"DS={ds:.2f}",
        f"Q1={q1}", f"Q2={q2}", f"Q3={q3}", f"Q4={q4}",
    ])
    return f"@DONE,{mode},{reason}," + ",".join(fields)


def request_payload(request):
    return encode_move(request.mode, request.request_value, request.unit)


class ChassisProtocolTests(unittest.TestCase):
    def test_cnt_and_mm_move_encoding(self):
        self.assertEqual(encode_move("d", 1400), b"@MOVE,D,1400,CNT\r\n")
        self.assertEqual(encode_move("D", 100, "MM"), b"@MOVE,D,100,MM\r\n")
        for bad in (("W", 0, "MM"), ("R", 10, "MM"), ("K", 10, "CNT")):
            with self.assertRaises(ChassisProtocolError):
                encode_move(*bad)

    def test_firmware_fixed_point_examples_for_all_translation_modes(self):
        config = ready_config()
        for mode, expected in COUNTS_100_MM.items():
            entry = config["chassis_translation_capabilities"][mode]
            self.assertEqual(
                firmware_mm_to_counts(100, fixed_counts_per_mm=entry["fixed_counts_per_mm"]),
                expected, mode,
            )

    def test_mm_ack_done_keep_request_target_and_encoder_separate(self):
        ack = parse_protocol_line("@ACK,D,100,MM")
        report = parse_protocol_line(done_line("D"))
        self.assertEqual(ack.kind, "ack")
        self.assertEqual((ack.value.request_value, ack.value.unit), (100, "MM"))
        self.assertEqual(report.kind, "done")
        self.assertEqual(report.value.request_value, 100)
        self.assertEqual(report.value.target_counts, 739)
        self.assertEqual(report.value.enc, 739.0)
        self.assertTrue(validate_done_kinematics(report.value)[0])

    def test_invalid_numeric_and_field_sets_are_rejected(self):
        invalid = [
            "@ACK,W,1.5,MM",
            "@ACK,R,10,MM",
            "@DONE,W,TARGET,REQ=100,UNIT=MM,BRAKE=1,ENC=1,DX=1,DY=0,DR=0,DS=0,Q1=1,Q2=1,Q3=1,Q4=1",
            "@DONE,W,TARGET,REQ=100,UNIT=MM,TARGET_CNT=718,BRAKE=NaN,ENC=718,DX=718,DY=0,DR=0,DS=0,Q1=718,Q2=718,Q3=718,Q4=718",
            "@DONE,W,TARGET,REQ=100,REQ=100,UNIT=MM,TARGET_CNT=718,BRAKE=718,ENC=718,DX=718,DY=0,DR=0,DS=0,Q1=718,Q2=718,Q3=718,Q4=718",
            "@DONE,W,TARGET,REQ=999999999999,UNIT=MM,TARGET_CNT=718,BRAKE=718,ENC=718,DX=718,DY=0,DR=0,DS=0,Q1=718,Q2=718,Q3=718,Q4=718",
        ]
        for line in invalid:
            self.assertEqual(parse_protocol_line(line).kind, "invalid", line)

    def test_split_joined_diagnostics_and_oversize_recovery(self):
        parser = ChassisStreamParser(max_line_bytes=128)
        self.assertEqual(parser.feed(b"note\r\n@ACK,D,100,M")[0].kind, "diagnostic")
        frames = parser.feed(b"M\r\n" + done_line("D").encode("ascii") + b"\r\n")
        self.assertEqual([frame.kind for frame in frames], ["ack", "invalid"])
        parser = ChassisStreamParser(max_line_bytes=128)
        frames = parser.feed(b"@" + b"X" * 200 + b"\r\n@PONG,12AB34CD\n")
        self.assertEqual([frame.kind for frame in frames], ["invalid", "pong"])

    def test_corrupt_original_calibration_log_rows_stay_rejected(self):
        path = Path(__file__).resolve().parents[1] / "control" / "标定" / "标定输出记录.txt"
        lines = path.read_text(encoding="utf-8").splitlines()
        for line_number in (122, 186, 248, 323, 356, 417, 478):
            self.assertEqual(parse_protocol_line(lines[line_number - 1]).kind, "invalid", line_number)

    def test_stop_sequence_terminates_partial_frame(self):
        parser = ChassisStreamParser()
        frames = parser.feed(b"@MOVE,W,100,MM" + STOP_SEQUENCE)
        self.assertEqual(frames[0].kind, "diagnostic")
        self.assertEqual(STOP_SEQUENCE, b"!\r\nX\r\n")

    def test_complete_done_after_field_diagnostic_fragment(self):
        done = ("@DONE,W,TARGET,REQ=718,UNIT=CNT,BRAKE=634.50,ENC=739.25,"
                "DX=739.25,DY=-4.25,DR=-1.25,DS=-3.25,"
                "Q1=733,Q2=739,Q3=737,Q4=748")
        wire = ("FREE_RAM_START=84" + done + "\r\n").encode("ascii")
        for split in range(len(wire) + 1):
            parser = ChassisStreamParser()
            frames = parser.feed(wire[:split]) + parser.feed(wire[split:])
            self.assertEqual([f.kind for f in frames], ["diagnostic", "done"])
            self.assertEqual(frames[1].raw, done)
            self.assertTrue(validate_done_kinematics(frames[1].value)[0])
        for bad in (done[:-4] + "\r\n", done, done + "junk\r\n",
                    done[20:] + "\r\n", "@DONE,W,TARGET,REQ=" + done + "\r\n"):
            parser = ChassisStreamParser()
            frames = parser.feed(("FREE_RAM_START=84" + bad).encode("ascii"))
            frames += parser.finalize()
            self.assertNotIn("done", [f.kind for f in frames], bad)

    def test_startup_before_resynchronized_frame_remains_visible(self):
        from chassis_protocol import PROTOCOL_BANNER
        frames = ChassisStreamParser().feed(
            (PROTOCOL_BANNER + "@ACK,W,718,CNT\r\n").encode("ascii"))
        self.assertEqual([f.kind for f in frames], ["banner", "ack"])


class ChassisControllerTests(unittest.TestCase):
    def setUp(self):
        CLOCK[0] = 0.0
        self.endpoint = FakeEndpoint()
        self.events = []
        nonces = iter([f"{number:08X}" for number in range(1, 100)])
        self.controller = ChassisController(
            self.endpoint,
            lambda kind, value, stamp: self.events.append((kind, value, stamp)),
            ready_config(),
            clock=lambda: CLOCK[0],
            nonce_factory=lambda: next(nonces),
        )
        self.generation = self.controller.begin_connection()
        self.receive("IDLE X=0 Y=0 R=0 S=0", 0.0)

    def receive(self, line, stamp):
        CLOCK[0] = stamp
        self.controller._last_rx_at = stamp
        self.controller.handle_frame(self.generation, parse_protocol_line(line), stamp)

    def start_move(self, request=None, *, source="auto", nonce="00000001"):
        request = request or ChassisMoveRequest("W", 100, "MM", 718, 0.1, "m")
        if source == "auto":
            self.assertTrue(self.controller.allow_automatic())
        self.assertTrue(self.controller.request_move(
            request, source=source, operator_authorized=source == "manual", now=CLOCK[0],
        ))
        CLOCK[0] += 0.5
        self.controller.poll(CLOCK[0])
        self.assertEqual(self.endpoint.writes[-1], f"@PING,{nonce}\r\n".encode())
        self.receive(f"@PONG,{nonce}", CLOCK[0] + 0.05)
        CLOCK[0] += 0.30
        self.controller.poll(CLOCK[0])
        self.assertEqual(self.endpoint.writes[-1], request_payload(request))
        self.assertEqual(self.controller.state, ChassisState.WAITING_ACK)
        return self.controller.pending

    def test_banner_alone_does_not_confirm_mm_but_full_markers_do(self):
        controller = ChassisController(self.endpoint, lambda *_: None, ready_config(), clock=lambda: 0.0)
        generation = controller.begin_connection()
        controller.handle_frame(generation, parse_protocol_line("MECANUM UNIVERSAL V6.3 COMM READY"), 0.0)
        self.assertFalse(controller.confirmed)
        for marker in EXPECTED_STARTUP_MARKERS:
            controller.handle_frame(generation, parse_protocol_line(marker), 0.1)
        self.assertTrue(controller.confirmed)
        self.assertEqual(controller.observed_capability_mode, "mm_ping_v1")

    def test_motion_waits_for_silence_matching_pong_and_post_silence(self):
        self.controller.allow_automatic()
        request = ChassisMoveRequest("D", 100, "MM", 739, 0.1, "m")
        self.assertTrue(self.controller.request_move(request, source="auto", now=0.0))
        self.controller.poll(0.49)
        self.assertEqual(self.endpoint.writes, [])
        self.controller.poll(0.50)
        self.receive("@PONG,FFFFFFFF", 0.55)
        self.controller.poll(0.80)
        self.assertNotIn(request_payload(request), self.endpoint.writes)
        self.receive("@PONG,00000001", 0.81)
        self.controller.poll(1.05)
        self.assertNotIn(request_payload(request), self.endpoint.writes)
        self.controller.poll(1.06)
        self.assertEqual(self.endpoint.writes.count(request_payload(request)), 1)

    def test_wrong_pong_nonce_times_out_without_sending_move(self):
        self.controller.allow_automatic()
        request = ChassisMoveRequest("D", 100, "MM", 739, 0.1, "m")
        self.assertTrue(self.controller.request_move(request, source="auto", now=0.0))
        self.controller.poll(0.5)
        self.receive("@PONG,FFFFFFFF", 0.6)
        self.controller.poll(2.51)
        self.assertNotIn(request_payload(request), self.endpoint.writes)
        self.assertEqual(self.controller.state, ChassisState.IDLE)
        self.assertIsNone(self.controller.pending)

    def test_continuous_rx_never_sends_move(self):
        self.controller.allow_automatic()
        request = ChassisMoveRequest("W", 100, "MM", 718, 0.1, "m")
        self.controller.request_move(request, source="auto", now=0.0)
        for index in range(1, 9):
            stamp = index * 0.49
            self.controller.feed_data(b"x", stamp, self.generation)
            self.controller.poll(stamp)
        self.controller.poll(4.01)
        self.assertNotIn(request_payload(request), self.endpoint.writes)
        self.assertEqual(self.controller.state, ChassisState.IDLE)

    def test_ack_missing_valid_done_completes_without_move_retry(self):
        action = self.start_move()
        self.controller.poll(float(action.send_started_at) + 2.01)
        self.assertTrue(action.ack_missing)
        self.receive(done_line("W"), float(action.send_started_at) + 2.10)
        self.assertEqual(self.controller.state, ChassisState.SETTLING)
        self.assertEqual(self.endpoint.writes.count(action.payload), 1)
        self.assertEqual(len([event for event in self.events if event[0] == "chassis_done"]), 1)

    def test_busy_after_move_started_stops_without_retry(self):
        action = self.start_move()
        self.receive("@ERR,BUSY", 1.0)
        self.assertEqual(self.endpoint.writes.count(action.payload), 1)
        self.assertEqual(self.endpoint.writes[-1], STOP_SEQUENCE)
        self.assertEqual(self.controller.state, ChassisState.STOPPING)

    def test_busy_during_preflight_aborts_without_stop_or_move(self):
        self.controller.allow_automatic()
        request = ChassisMoveRequest("W", 100, "MM", 718, 0.1, "m")
        self.assertTrue(self.controller.request_move(request, source="auto", now=0.0))
        self.controller.poll(0.5)
        self.receive("@ERR,BUSY", 0.6)
        self.assertNotIn(request_payload(request), self.endpoint.writes)
        self.assertNotIn(STOP_SEQUENCE, self.endpoint.writes)
        self.assertEqual(self.controller.state, ChassisState.IDLE)

    def test_wrong_ack_and_wrong_mm_target_each_stop_transaction(self):
        action = self.start_move()
        self.receive("@ACK,D,100,MM", 0.9)
        self.assertEqual(self.endpoint.writes.count(action.payload), 1)
        self.assertEqual(self.endpoint.writes[-1], STOP_SEQUENCE)

        self.receive("IDLE X=0 Y=0 R=0 S=0", 1.0)
        CLOCK[0] = 1.0
        action = self.start_move(source="manual", nonce="00000002")
        wrong_target = done_line("W").replace("TARGET_CNT=718", "TARGET_CNT=719")
        self.receive(wrong_target, 2.0)
        self.assertEqual(self.endpoint.writes.count(action.payload), 2)
        self.assertEqual(self.endpoint.writes[-1], STOP_SEQUENCE)
        self.assertEqual(self.controller.state, ChassisState.STOPPING)

    def test_missing_mm_target_is_protocol_fault_after_move_started(self):
        action = self.start_move()
        missing_target = done_line("W").replace("TARGET_CNT=718,", "")
        self.receive(missing_target, 1.0)
        self.assertEqual(self.endpoint.writes.count(action.payload), 1)
        self.assertEqual(self.endpoint.writes[-1], STOP_SEQUENCE)
        self.assertEqual(self.controller.state, ChassisState.STOPPING)

    def test_duplicate_done_during_settle_is_ignored(self):
        self.start_move()
        self.receive("@ACK,W,100,MM", 0.9)
        report = done_line("W")
        self.receive(report, 1.0)
        self.receive(report, 1.1)
        self.assertEqual(self.controller.state, ChassisState.SETTLING)
        self.assertEqual(len([event for event in self.events if event[0] == "chassis_done"]), 1)

    def test_consecutive_same_request_is_not_globally_deduplicated(self):
        self.start_move(source="manual")
        self.receive(done_line("W"), 1.0)
        self.controller.complete_settle(resume_auto=False, now=1.2)
        CLOCK[0] = 1.2
        self.start_move(source="manual", nonce="00000002")
        self.receive(done_line("W"), 2.1)
        self.assertEqual(len([event for event in self.events if event[0] == "chassis_done"]), 2)

    def test_late_done_before_new_move_is_written_locks_control(self):
        self.start_move(source="manual")
        report = done_line("W")
        self.receive(report, 1.0)
        self.controller.complete_settle(resume_auto=False, now=1.2)
        request = ChassisMoveRequest("W", 100, "MM", 718, 0.1, "m")
        self.controller.request_move(request, source="manual", operator_authorized=True, now=1.2)
        self.receive(report, 1.3)
        self.assertEqual(self.controller.state, ChassisState.UNKNOWN)
        self.assertTrue(self.controller.in_flight)

    def test_total_timeout_stops_and_move_is_never_retried(self):
        action = self.start_move()
        self.controller.poll(float(action.send_started_at) + 12.01)
        self.assertEqual(self.endpoint.writes.count(action.payload), 1)
        self.assertEqual(self.endpoint.writes[-1], STOP_SEQUENCE)
        self.assertIn(self.controller.state, {ChassisState.STOPPING, ChassisState.STOPPING_IDLE})

    def test_stop_feedback_timeout_stays_unknown_until_idle(self):
        action = self.start_move()
        self.assertTrue(self.controller.request_stop(reason="test", now=1.0))
        self.controller.poll(4.01)
        self.assertEqual(self.endpoint.writes.count(action.payload), 1)
        self.assertEqual(self.controller.state, ChassisState.UNKNOWN)
        self.assertTrue(self.controller.in_flight)
        self.receive("IDLE X=0 Y=0 R=0 S=0", 4.1)
        self.assertEqual(self.controller.state, ChassisState.IDLE)
        self.assertFalse(self.controller.in_flight)

    def test_partial_move_write_failure_stops_and_never_retries(self):
        self.endpoint.auto_complete = False
        self.controller.allow_automatic()
        request = ChassisMoveRequest("W", 100, "MM", 718, 0.1, "m")
        self.assertTrue(self.controller.request_move(request, source="auto", now=0.0))
        self.controller.poll(0.5)
        ping_callbacks = self.endpoint.callbacks[-1]
        ping_callbacks[0](0.5)
        self.receive("@PONG,00000001", 0.55)
        self.controller.poll(0.80)
        move_callbacks = self.endpoint.callbacks[-1]
        move_callbacks[0](0.80)
        move_callbacks[2](0.81, 5, len(request_payload(request)), "partial")
        self.assertEqual(self.endpoint.writes.count(request_payload(request)), 1)
        self.assertEqual(self.endpoint.writes[-1], STOP_SEQUENCE)
        self.assertEqual(self.controller.state, ChassisState.STOPPING)

    def test_idle_query_and_idle_stop_are_normal(self):
        self.assertTrue(self.controller.request_status(0.1))
        self.receive("IDLE X=1 Y=2 R=3 S=4", 0.2)
        self.assertEqual(self.controller.state, ChassisState.IDLE)
        self.controller.request_stop(now=0.3)
        self.assertEqual(self.controller.state, ChassisState.STOPPING_IDLE)
        self.receive("@ERR,BAD_CMD", 0.31)
        self.receive("IDLE X=1 Y=2 R=3 S=4", 0.7)
        self.assertEqual(self.controller.state, ChassisState.IDLE)
        self.assertFalse(self.controller.in_flight)

    def test_runtime_banner_during_action_stops_and_invalidates_capability(self):
        self.start_move()
        self.receive("MECANUM UNIVERSAL V6.3 COMM READY", 1.0)
        self.assertIn(STOP_SEQUENCE, self.endpoint.writes)
        self.assertFalse(self.controller.confirmed)
        self.assertEqual(self.controller.observed_capability_mode, "unknown")
        self.assertTrue(self.controller.in_flight)

    def test_stale_connection_frame_cannot_complete_current_action(self):
        self.start_move()
        current = self.controller.state
        self.controller.handle_frame(self.generation - 1, parse_protocol_line(done_line("W")), 1.0)
        self.assertEqual(self.controller.state, current)

    def test_communication_check_separates_first_and_retry_success(self):
        self.assertTrue(self.controller.request_communication_check(2, now=0.0))
        self.controller.poll(0.5)
        self.receive("@PONG,00000001", 0.55)
        self.controller.poll(1.05)
        self.controller.poll(3.06)
        self.controller.poll(3.56)
        self.receive("@PONG,00000003", 3.60)
        result = [event for event in self.events if event[0] == "chassis_communication_result"][-1][1][1]
        self.assertEqual(result["first_successes"], 1)
        self.assertEqual(result["retry_successes"], 1)
        self.assertEqual(result["failures"], 0)


class ChassisMotionAdapterTests(unittest.TestCase):
    def test_mm_path_uses_millimetres_once_and_never_rounds_up(self):
        adapter = ChassisMotionAdapter(ready_config())
        request = adapter.request_for_command(VelocityCommand(forward_mps=0.1239, duration_s=1.0))
        self.assertEqual((request.mode, request.request_value, request.unit), ("W", 123, "MM"))
        self.assertEqual(request.target_counts, firmware_mm_to_counts(123, fixed_counts_per_mm=71775))
        self.assertLessEqual(request.target, 0.1239)

    def test_cnt_compatibility_converts_exactly_once(self):
        adapter = ChassisMotionAdapter(ready_config(unit="CNT", capability="cnt_only"))
        request = adapter.request_for_command(VelocityCommand(right_mps=0.1, duration_s=1.0))
        self.assertEqual((request.mode, request.unit), ("D", "CNT"))
        self.assertEqual(request.request_value, 738)
        self.assertLessEqual(request.target, 0.1)

    def test_all_eight_directions_and_diagonal_total_length(self):
        adapter = ChassisMotionAdapter(ready_config())
        commands = {
            "W": VelocityCommand(forward_mps=0.1, duration_s=1),
            "S": VelocityCommand(forward_mps=-0.1, duration_s=1),
            "A": VelocityCommand(right_mps=-0.1, duration_s=1),
            "D": VelocityCommand(right_mps=0.1, duration_s=1),
            "Q": VelocityCommand(forward_mps=0.1 / math.sqrt(2), right_mps=-0.1 / math.sqrt(2), duration_s=1),
            "E": VelocityCommand(forward_mps=0.1 / math.sqrt(2), right_mps=0.1 / math.sqrt(2), duration_s=1),
            "Z": VelocityCommand(forward_mps=-0.1 / math.sqrt(2), right_mps=-0.1 / math.sqrt(2), duration_s=1),
            "C": VelocityCommand(forward_mps=-0.1 / math.sqrt(2), right_mps=0.1 / math.sqrt(2), duration_s=1),
        }
        for mode, command in commands.items():
            request = adapter.request_for_command(command)
            self.assertEqual((request.mode, request.request_value), (mode, 100))
        with self.assertRaises(MotionConversionError):
            adapter.request_for_command(VelocityCommand(forward_mps=0.1, right_mps=0.02, duration_s=1))

    def test_unvalidated_rotation_does_not_block_translation(self):
        adapter = ChassisMotionAdapter(ready_config())
        self.assertTrue(adapter.readiness("W")[0])
        with self.assertRaises(MotionConversionError):
            adapter.request_for_command(VelocityCommand(yaw_rps=0.1, duration_s=1))

    def test_disabled_unused_directions_do_not_block_enabled_mode(self):
        config = ready_config()
        for mode in "SADQEZC":
            config["chassis_translation_capabilities"][mode]["enabled"] = False
        adapter = ChassisMotionAdapter(config)
        self.assertTrue(adapter.readiness("W")[0])
        self.assertTrue(adapter.readiness()[0])

    def test_manual_requires_version_but_not_automatic_range_validation(self):
        config = resolve_runtime_config("hardware", "navigation", {})
        with self.assertRaises(MotionConversionError):
            ChassisMotionAdapter(config).request_for_manual("D", "100", "MM")
        config["chassis_firmware_confirmed"] = True
        config["chassis_capability_mode"] = "mm_ping_v1"
        request = ChassisMotionAdapter(config).request_for_manual("D", "100", "MM")
        self.assertEqual(request.target_counts, 739)

    def test_execution_prior_uses_enc_and_separates_units(self):
        adapter = ChassisMotionAdapter(ready_config())
        report = parse_protocol_line(done_line("Q", enc=1018)).value
        estimate = adapter.execution_from_report(report)
        distance = 1018 / 10.1756 / 1000
        self.assertAlmostEqual(math.hypot(estimate.local_x_m, estimate.local_y_m), distance)
        self.assertAlmostEqual(estimate.uncertainty_m, 0.015)
        self.assertEqual(estimate.uncertainty_rad, 0.0)


if __name__ == "__main__":
    unittest.main()
