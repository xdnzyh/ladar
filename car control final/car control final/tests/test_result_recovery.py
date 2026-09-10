import json
from pathlib import Path
from types import SimpleNamespace
import unittest

from chassis_controller import ChassisController, ChassisMotionAdapter, ChassisMoveRequest, ChassisState
from chassis_protocol import (
    ChassisProtocolError, ChassisStreamParser, PROTOCOL_BANNER, STOP_SEQUENCE,
    encode_move, encode_result_query, firmware_mm_to_counts, parse_protocol_line,
    validate_done_kinematics,
)
from runtime_config import RuntimeConfigError, resolve_runtime_config


def checked(body):
    # Independent CCITT-FALSE reference, including the leading @ in the CRC.
    crc = 0xFFFF
    for byte in body.encode("ascii"):
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return f"{body},{crc:04X}\r\n".encode("ascii")


WHEELS = {
    "W": (1, 1, 1, 1), "S": (-1, -1, -1, -1),
    "A": (1, -1, 1, -1), "D": (-1, 1, -1, 1),
    "Q": (1, 0, 1, 0), "E": (0, 1, 0, 1),
    "Z": (0, -1, 0, -1), "C": (-1, 0, -1, 0),
    "R": (-1, 1, 1, -1), "F": (1, -1, -1, 1),
}


def result(nonce="00000001", mode="W", reason=0, value=100, unit="CNT", enc=100):
    wheels = ",".join(str(sign * enc) for sign in WHEELS[mode])
    return checked(f"@RESULT,{nonce},{mode},{reason},{value},{unit},{enc:.2f},{enc:.2f},{wheels}")


class Endpoint:
    is_open = True

    def __init__(self, clock):
        self.clock = clock
        self.writes = []
        self.tickets = []
        self.defer_results = False

    def write_ticket(self, data, **callbacks):
        ticket = SimpleNamespace(data=data, callbacks=callbacks, state="queued")
        if callbacks.get("discard_pending"):
            for old in self.tickets:
                self.cancel_write(old)
        self.tickets.append(ticket)
        if not (self.defer_results and data.lstrip().startswith(b"@RESULT,")):
            self.complete(ticket)
        return ticket

    def complete(self, ticket):
        if ticket.state != "queued":
            return
        ticket.state = "written"
        self.writes.append((self.clock(), ticket.data))
        for name in ("on_sent", "on_written"):
            callback = ticket.callbacks.get(name)
            if callback:
                callback(self.clock())

    def cancel_write(self, ticket):
        if ticket.state != "queued":
            return "started"
        ticket.state = "cancelled"
        callback = ticket.callbacks.get("on_cancelled")
        if callback:
            callback(self.clock())
        return "cancelled"


class ResultProtocolTests(unittest.TestCase):
    def test_confirmed_firmware_wire_vectors(self):
        wire = b"@RESULT,1234ABCD,W,0,718,CNT,639.00,639.00,639,639,639,639,7EF0\r\n"
        frame = parse_protocol_line(wire.decode())
        self.assertEqual(frame.kind, "result")
        self.assertEqual((frame.value.nonce, frame.value.report.request_value, frame.value.report.enc),
                         ("1234ABCD", 718, 639.0))
        self.assertEqual(encode_result_query("1234ABCD"), b"@RESULT,1234ABCD,8CE5\r\n")
        for status, crc in (("B", "D873"), ("N", "19FF")):
            self.assertEqual(parse_protocol_line(f"@RESULT,1234ABCD,{status},{crc}").value.status, status)

    def test_crc_vectors_and_protected_move_cover_entire_body(self):
        self.assertEqual(checked("123456789"), b"123456789,29B1\r\n")
        self.assertEqual(encode_move("W", 100), b"@MOVE,W,100,CNT\r\n")
        self.assertEqual(encode_move("W", 100, nonce="A1B2C3D4"), checked("@MOVE,W,100,CNT,A1B2C3D4"))
        self.assertEqual(encode_result_query("A1B2C3D4"), checked("@RESULT,A1B2C3D4"))
        self.assertLessEqual(len(encode_move("D", 2147483647, nonce="FFFFFFFF")) - 2, 47)
        for nonce in ("abcdef12", "123", "FFFFFFFFF", "G0000000"):
            with self.subTest(nonce=nonce), self.assertRaises(ChassisProtocolError):
                encode_result_query(nonce)
        with self.assertRaises(ChassisProtocolError):
            encode_move("W", 100, nonce="00000001", max_command_bytes=20)

    def test_reasons_signed_wheels_and_all_kinematics(self):
        for mode in WHEELS:
            for number, reason in enumerate(("TARGET", "EMERGENCY", "TIMEOUT", "WRONG_DIRECTION")):
                with self.subTest(mode=mode, reason=reason):
                    frame = parse_protocol_line(result(mode=mode, reason=number).decode())
                    self.assertEqual(frame.kind, "result")
                    self.assertEqual(frame.value.report.reason, reason)
                    self.assertTrue(validate_done_kinematics(frame.value.report)[0])
        frame = parse_protocol_line(result(reason=3, enc=-100).decode())
        self.assertTrue(validate_done_kinematics(frame.value.report)[0])

    def test_crc_and_entire_field_set_are_strict(self):
        fields = result().decode().strip().rsplit(",", 1)[0].split(",")
        mutations = {
            0: ["@RESULTX"], 1: ["00000002X", "abcdefgh", "", " 00000001"],
            2: ["w", "WW", "K"], 3: ["4", "-1", "00", "TARGET", "0.0"],
            4: ["0", "-100", "100.5", "2147483648", "100x"],
            5: ["cnt", "CM", "CNT "], 6: ["nan", "inf", "1e999", "1.0x"],
            7: ["NaN", "-2147483649", "", " 100"],
            8: ["100.0", "2147483648"], 9: ["1x"], 10: [""], 11: ["100x"],
        }
        for index, values in mutations.items():
            for value in values:
                row = fields.copy()
                row[index] = value
                with self.subTest(index=index, value=value):
                    self.assertEqual(parse_protocol_line(checked(",".join(row)).decode()).kind, "invalid")
        for body in (",".join(fields[:-1]), ",".join(fields) + ",0",
                     "@RESULT,00000001,X", "@RESULT,00000001,B,0",
                     "@RESULT,00000001,R,0,100,MM,100,100,-100,100,100,-100"):
            self.assertEqual(parse_protocol_line(checked(body).decode()).kind, "invalid")
        good = result().decode().strip()
        for bad in (good[:-1], good + ",EXTRA", good + " ", good.replace(",100,CNT", ",101,CNT")):
            self.assertEqual(parse_protocol_line(bad).kind, "invalid")
        body = "@RESULT,ABCDEF01,B"
        # A CRC over text excluding @ is not the wire contract.
        self.assertEqual(parse_protocol_line("@" + checked(body[1:]).decode()).kind, "invalid")

    def test_statuses_marker_fragmentation_and_existing_resync(self):
        for status in ("B", "N"):
            frame = parse_protocol_line(checked(f"@RESULT,00000001,{status}").decode())
            self.assertEqual((frame.value.status, frame.value.report), (status, None))
        marker = "RESULT=1 CRC=CCITT QUERY=1"
        self.assertEqual(parse_protocol_line(marker).kind, "diagnostic")
        wire = result()
        for split in range(len(wire)):
            parser = ChassisStreamParser()
            frames = parser.feed(wire[:split]) + parser.feed(wire[split:])
            self.assertEqual([f.kind for f in frames], ["result"])
        frames = ChassisStreamParser().feed(b"diagnostic tail" + wire)
        self.assertEqual([f.kind for f in frames], ["diagnostic", "result"])
        frames = ChassisStreamParser().feed(b"@RESULT,damaged" + wire)
        self.assertEqual([f.kind for f in frames], ["invalid"])
        frames = ChassisStreamParser().feed(PROTOCOL_BANNER.encode() + wire)
        self.assertEqual([f.kind for f in frames], ["banner", "result"])


class ResultRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.events = []
        self.endpoint = Endpoint(lambda: self.now)
        self.config = resolve_runtime_config("hardware", "navigation", {
            "chassis_result_recovery": True, "chassis_firmware_confirmed": True,
            "chassis_capability_mode": "mm_ping_v1",
        })
        nonces = iter(f"{i:08X}" for i in range(1, 100))
        self.controller = ChassisController(
            self.endpoint, self.emit, self.config, clock=lambda: self.now,
            nonce_factory=lambda: next(nonces),
        )
        self.generation = self.controller.begin_connection()
        self.receive(b"IDLE X=0 Y=0 R=0 S=0\r\n")

    def emit(self, kind, value, stamp):
        self.events.append((kind, value, stamp))
        if kind == "chassis_frame":
            self.controller.handle_frame(*value, stamp)

    def receive(self, payload, stamp=None, generation=None):
        if stamp is not None:
            self.now = stamp
        self.controller.feed_data(payload, self.now, self.generation if generation is None else generation)

    def poll(self, stamp):
        self.now = stamp
        self.controller.poll()

    def writes(self, prefix):
        return [(stamp, data) for stamp, data in self.endpoint.writes if data.lstrip().startswith(prefix)]

    def start(self, mode="W", unit="CNT", request=None):
        self.assertTrue(self.controller.request_move(
            request if request is not None else mode, 100, unit,
            source="manual", operator_authorized=True,
        ))
        self.poll(self.now + 0.51)
        action = self.controller.pending
        self.assertEqual(self.writes(b"@PING,")[-1][1].lstrip(), f"@PING,{action.nonce}\r\n".encode())
        self.receive(f"@PONG,{action.nonce}\r\n".encode(), self.now + 0.05)
        self.poll(self.now + 0.26)
        self.assertEqual(self.controller.state, ChassisState.WAITING_ACK)
        return action

    def assert_no_motion_retry(self):
        self.assertEqual(len(self.writes(b"@MOVE,")), 1)
        self.assertEqual(len(self.writes(b"@PING,")), 1)

    def test_auto_result_completes_immediately_without_ack_or_query(self):
        action = self.start()
        self.assertEqual(action.payload, checked("@MOVE,W,100,CNT,00000001"))
        self.receive(result(), self.now + 0.15)
        self.assertEqual(self.controller.state, ChassisState.SETTLING)
        self.assertEqual(action.done_at, self.now)
        self.receive(result(), self.now + 0.01)
        self.poll(10.0)
        self.assertEqual(len([e for e in self.events if e[0] == "chassis_done"]), 1)
        self.assertFalse(self.writes(b"@RESULT,"))
        self.assert_no_motion_retry()

    def test_lost_or_corrupt_completion_recovers_by_read_only_query(self):
        for corruption in (b"", b"@DONE,W,TARGET,broken\r\n",
                           result().replace(b",100,CNT", b",101,CNT"),
                           b"@RESULT,broken\xff\r\n", b"@RESULT,truncated",
                           b"x" * 1100 + b"\r\n"):
            with self.subTest(corruption=corruption[:30]):
                self.setUp()
                action = self.start()
                self.receive(corruption, self.now + 0.3)
                self.assertNotIn(self.controller.state, {ChassisState.STOPPING, ChassisState.UNKNOWN})
                self.poll(action.result_due_at + 0.01)
                self.assertEqual(self.writes(b"@RESULT,")[-1][1], checked("@RESULT,00000001"))
                # Terminate a truncated line; no damaged protocol frame is spliced.
                self.receive(b"\r\n" + result(), self.now + 0.15)
                self.assertEqual(self.controller.state, ChassisState.SETTLING)
                self.assertFalse(self.writes(b"!"))
                self.assert_no_motion_retry()

    def test_unchecked_legacy_done_never_completes_protected_move(self):
        action = self.start()
        legacy = (b"@DONE,W,TARGET,REQ=100,UNIT=CNT,BRAKE=100,ENC=100,"
                  b"DX=100,DY=0,DR=0,DS=0,Q1=100,Q2=100,Q3=100,Q4=100\r\n")
        self.receive(legacy, self.now + 0.1)
        self.assertIsNone(action.done_at)
        self.poll(action.result_due_at + 0.01)
        self.receive(result())
        self.assertEqual(self.controller.state, ChassisState.SETTLING)
        self.assert_no_motion_retry()

    def test_missing_line_end_recovers_on_first_query_and_retains_raw_fragment(self):
        action = self.start()
        fragment = result().rstrip(b"\r\n")
        self.receive(fragment, self.now + 0.2)
        self.assertIsNone(action.done_at)
        self.poll(action.result_due_at + 0.01)
        errors = [e for e in self.events if e[0] == "chassis_protocol_error"]
        self.assertEqual(errors[-1][1][1], fragment.decode())
        self.assertEqual(action.result_queries, 1)
        self.assertIsNone(action.done_at)
        # No extra newline before the recovered reply.
        self.receive(result(), self.now + 0.15)
        self.assertEqual(self.controller.state, ChassisState.SETTLING)
        self.assertFalse(self.writes(b"!"))
        self.assert_no_motion_retry()

    def test_each_query_discards_only_quiet_partial_rx_and_unknown_keeps_pending(self):
        action = self.start()
        for _ in range(2):
            due = action.result_due_at
            self.receive(b"@RESULT,00000001,N,partial", due - 0.1)
            self.poll(due)
            self.assertTrue(self.controller.parser._buffer)
            self.poll(due + 0.151)
            self.assertFalse(self.controller.parser._buffer)
            self.receive(checked("@RESULT,00000001,N"), self.now + 0.01)
            self.assertIs(self.controller.pending, action)
            self.assertIsNone(action.done_at)
            self.assertFalse(self.controller.can_scan)
        self.assertEqual(action.result_queries, 2)
        self.receive(result(), self.now + 0.1)
        self.assertEqual(self.controller.state, ChassisState.SETTLING)
        self.assert_no_motion_retry()

    def test_wrong_id_request_crc_or_encoder_does_not_complete_or_stop(self):
        action = self.start()
        bad_reports = [
            result(nonce="00000002"), result(mode="S"), result(value=101), result(unit="MM"),
            result().replace(b",100,CNT", b",101,CNT"),
            checked("@RESULT,00000001,W,0,100,CNT,100,90,100,100,100,100"),
            checked("@RESULT,00000001,W,0,100,CNT,100,100,100,100,100,100,0"),
        ]
        for wire in bad_reports:
            self.receive(wire, self.now + 0.03)
            self.assertIsNone(action.done_at)
            self.assertFalse(self.writes(b"!"))
        self.poll(action.result_due_at + 0.01)
        self.receive(result())
        self.assertEqual(self.controller.state, ChassisState.SETTLING)
        self.assert_no_motion_retry()

    def test_initial_inactivity_and_actual_partial_rx_quiet_gate_queries(self):
        action = self.start()
        self.receive(b"@ACK,W,100,CNT\r\n", self.now + 0.2)
        due = self.now + 1.8
        self.assertAlmostEqual(action.result_due_at, due)
        self.poll(due - 0.01)
        self.assertFalse(self.writes(b"@RESULT,"))
        self.receive(b"diagnostic fragment", due - 0.1)
        self.poll(due)
        self.assertFalse(self.writes(b"@RESULT,"))
        self.poll(due + 0.151)
        self.assertEqual(len(self.writes(b"@RESULT,")), 1)
        self.assert_no_motion_retry()

    def test_busy_unknown_and_loss_share_five_query_bound_and_total_deadline(self):
        action = self.start()
        deadline = self.controller._total_deadline
        for i in range(5):
            self.poll(action.result_due_at + 0.001)
            self.assertEqual(len(self.writes(b"@RESULT,")), i + 1)
            if i in (0, 2, 4):
                self.receive(checked("@RESULT,00000001,B"), self.now + 0.1)
                self.assertEqual(self.controller.state, ChassisState.WAITING_DONE)
            elif i == 1:
                self.receive(checked("@RESULT,00000001,N"), self.now + 0.1)
                self.assertIsNone(action.done_at)
            self.poll(action.result_due_at - 0.001)
            self.assertEqual(len(self.writes(b"@RESULT,")), i + 1)
            self.assertEqual(self.controller._total_deadline, deadline)
        self.poll(deadline - 0.01)
        self.assertEqual(len(self.writes(b"@RESULT,")), 5)
        times = [stamp for stamp, _ in self.writes(b"@RESULT,")]
        self.assertTrue(all(b - a >= 1.2 for a, b in zip(times, times[1:])))
        self.poll(deadline)
        self.assertEqual(self.controller.state, ChassisState.STOPPING)
        self.assertEqual(self.writes(b"!")[-1][1], STOP_SEQUENCE)
        self.poll(deadline + self.controller._stop_timeout() + 0.1)
        self.assertEqual(self.controller.state, ChassisState.UNKNOWN)
        self.assertEqual(len(self.writes(b"@RESULT,")), 5)
        self.assert_no_motion_retry()

    def test_continuous_rx_cannot_extend_total_deadline(self):
        action = self.start()
        deadline = self.controller._total_deadline
        while self.now < deadline:
            self.receive(b"x", min(deadline, self.now + 0.2))
            self.poll(self.now)
        self.assertEqual(self.controller.state, ChassisState.STOPPING)
        self.assertEqual(action.result_queries, 0)
        self.assert_no_motion_retry()

    def test_long_move_lost_auto_result_recovers_with_reserved_fifth_query(self):
        action = self.start()
        deadline = self.controller._total_deadline
        stopped_at = action.send_started_at + 8.0
        for _ in range(4):
            self.poll(action.result_due_at + 0.001)
            self.assertLess(self.now, stopped_at)
            self.receive(checked("@RESULT,00000001,B"), self.now + 0.05)
        self.assertEqual(len(self.writes(b"@RESULT,")), 4)
        self.poll(stopped_at)
        self.assertEqual(len(self.writes(b"@RESULT,")), 4)
        self.assertIsNone(action.done_at)
        self.poll(deadline - 1.501)
        self.assertEqual(len(self.writes(b"@RESULT,")), 4)
        self.poll(deadline - 1.5)
        self.assertEqual(len(self.writes(b"@RESULT,")), 5)
        self.assertEqual(self.controller._total_deadline, deadline)
        self.receive(result(), self.now + 0.15)
        self.assertEqual(self.controller.state, ChassisState.SETTLING)
        self.assertFalse(self.writes(b"!"))
        self.assert_no_motion_retry()

    def test_reserved_query_requires_spacing_and_never_crosses_total_deadline(self):
        action = self.start()
        deadline = self.controller._total_deadline
        # Late polling puts query four near the reserved window.
        for offset in (6.0, 7.3, 8.6, 9.9):
            self.poll(action.send_started_at + offset)
        self.assertEqual(action.result_queries, 4)
        self.poll(deadline - 1.5)
        self.assertEqual(action.result_queries, 4)
        self.poll(deadline - 0.8)
        self.assertEqual(action.result_queries, 5)
        self.assertGreaterEqual(self.writes(b"@RESULT,")[-1][0] - self.writes(b"@RESULT,")[-2][0], 1.2)
        self.poll(deadline)
        self.assertEqual(self.controller.state, ChassisState.STOPPING)
        self.assertEqual(action.result_queries, 5)
        self.assert_no_motion_retry()

    def test_short_deadline_stops_even_when_no_query_window_remains(self):
        self.config["chassis_total_timeout_s"] = 1.0
        self.start()
        self.poll(self.controller._total_deadline)
        self.assertEqual(self.controller.state, ChassisState.STOPPING)
        self.assertFalse(self.writes(b"@RESULT,"))
        self.assert_no_motion_retry()

    def test_stale_snapshot_cannot_complete_identical_next_request(self):
        first = self.start()
        self.receive(result(), self.now + 0.1)
        self.controller.complete_settle(resume_auto=False)
        second = self.start()
        self.assertNotEqual(first.nonce, second.nonce)
        self.receive(result(), self.now + 0.1)
        self.assertIsNone(second.done_at)
        self.receive(result(nonce=second.nonce))
        self.assertEqual(self.controller.state, ChassisState.SETTLING)
        self.assertEqual(len(self.writes(b"@MOVE,")), 2)

    def test_preflight_result_does_not_complete_or_send_move(self):
        self.assertTrue(self.controller.request_move("W", 100, operator_authorized=True))
        self.poll(0.51)
        self.receive(result(), 0.6)
        self.assertEqual(self.controller.state, ChassisState.WAITING_PONG)
        self.poll(3.0)
        self.assertFalse(self.writes(b"@MOVE,"))
        self.assertFalse(self.writes(b"@RESULT,"))

    def test_mm_result_uses_adapter_or_controller_target_counts(self):
        for use_adapter in (False, True):
            self.setUp()
            request = (ChassisMotionAdapter(self.config).request_for_manual("W", 100, "MM")
                       if use_adapter else None)
            action = self.start(unit="MM", request=request)
            expected = firmware_mm_to_counts(100, fixed_counts_per_mm=71775)
            self.receive(result(unit="MM", enc=expected), self.now + 0.1)
            self.assertEqual(self.controller.state, ChassisState.SETTLING)
            report = [e[1][2] for e in self.events if e[0] == "chassis_done"][-1]
            self.assertEqual(report.target_counts, expected)
            self.assertEqual(report.dx, expected)
            self.assertEqual(report.raw, result(unit="MM", enc=expected).decode().strip())
            self.assertEqual(action.request.unit, "MM")

    def test_inconsistent_mm_request_target_is_refused(self):
        action = self.start(request=ChassisMoveRequest("W", 100, "MM", 999, 0.1, "m"))
        self.receive(result(unit="MM", enc=718))
        self.assertIsNone(action.done_at)
        self.assertFalse(self.writes(b"!"))

    def test_all_stop_reasons_reach_existing_done_path(self):
        for reason, name in enumerate(("TARGET", "EMERGENCY", "TIMEOUT", "WRONG_DIRECTION")):
            with self.subTest(reason=reason):
                self.setUp()
                action = self.start()
                self.receive(result(reason=reason), self.now + 0.1)
                done = [e for e in self.events if e[0] == "chassis_done"]
                self.assertEqual(len(done), 1)
                self.assertEqual(done[0][1][2].reason, name)
                self.assertIsNotNone(action.done_at)

    def test_stop_keeps_priority_deadline_and_checked_confirmation(self):
        action = self.start()
        self.controller.request_stop()
        deadline = self.controller._stop_deadline
        self.poll(self.now + 0.1)
        self.assertFalse(self.writes(b"@RESULT,"))
        self.assertEqual(self.controller._stop_deadline, deadline)
        self.receive(result(reason=1))
        self.assertTrue(action.stop_requested)
        self.assertEqual(self.controller.state, ChassisState.SETTLING)
        self.assert_no_motion_retry()

    def test_old_firmware_rejection_never_falls_back_or_retries_move(self):
        self.start()
        self.receive(b"@ERR,BAD_CMD\r\n", self.now + 0.1)
        self.assertEqual(self.controller.state, ChassisState.STOPPING)
        self.poll(20.0)
        self.assertFalse(self.writes(b"@RESULT,"))
        self.assert_no_motion_retry()

    def test_padding_and_query_tags_use_current_action_and_connection(self):
        self.config["chassis_tx_padding_spaces"] = 32
        action = self.start()
        self.poll(action.result_due_at + 0.01)
        wire = self.writes(b"@RESULT,")[-1][1]
        self.assertEqual(wire, b" " * 32 + checked("@RESULT,00000001"))
        event = [e for e in self.events if e[0] == "chassis_raw_tx"][-1]
        self.assertEqual(event[1], (self.generation, action.action_id, wire))

    def test_pending_query_cancels_on_done_stop_startup_disconnect_or_reconnect(self):
        for operation in ("done", "stop", "startup", "disconnect", "reconnect"):
            with self.subTest(operation=operation):
                self.setUp()
                action = self.start()
                self.endpoint.defer_results = True
                self.poll(action.result_due_at + 0.01)
                ticket = self.endpoint.tickets[-1]
                self.assertTrue(action.result_query_pending)
                if operation == "done":
                    self.receive(result())
                elif operation == "stop":
                    self.controller.request_stop()
                elif operation == "startup":
                    self.receive(PROTOCOL_BANNER.encode() + b"\r\n")
                elif operation == "disconnect":
                    self.controller.disconnect()
                else:
                    self.controller.begin_connection()
                self.assertEqual(ticket.state, "cancelled")
                before = (action.result_due_at, action.done_at, self.controller.state)
                for callback in ("on_sent", "on_written"):
                    ticket.callbacks[callback](self.now + 20)
                ticket.callbacks["on_failed"](self.now + 20, 0, len(ticket.data), "stale")
                self.assertEqual(before, (action.result_due_at, action.done_at, self.controller.state))
                self.endpoint.complete(ticket)
                self.assertFalse(self.writes(b"@RESULT,"))
                if operation in {"startup", "disconnect", "reconnect"}:
                    self.receive(result())
                    self.assertIsNone(action.done_at)
                    self.assertTrue(action.result_invalidated)

    def test_reset_then_new_connection_rejects_old_generation_and_snapshot(self):
        first = self.start()
        old_generation = self.generation
        self.controller.disconnect()
        self.generation = self.controller.begin_connection()
        self.receive(b"IDLE X=0 Y=0 R=0 S=0\r\n")
        second = self.start()
        self.receive(result(nonce=second.nonce), generation=old_generation)
        self.receive(result(nonce=first.nonce))
        self.assertIsNone(second.done_at)
        self.receive(result(nonce=second.nonce))
        self.assertIsNotNone(second.done_at)
        self.assertEqual(len(self.writes(b"@MOVE,")), 2)

    def test_deferred_query_spacing_starts_from_actual_write_and_is_bounded(self):
        action = self.start()
        self.endpoint.defer_results = True
        self.poll(action.result_due_at + 0.01)
        ticket = self.endpoint.tickets[-1]
        self.poll(self.now + 2.0)
        self.assertEqual(action.result_queries, 1)
        self.endpoint.complete(ticket)
        completed = self.now
        self.poll(completed + 1.19)
        self.assertEqual(action.result_queries, 1)
        self.poll(completed + 1.21)
        self.assertEqual(action.result_queries, 2)
        self.assert_no_motion_retry()

    def test_query_write_failures_are_bounded_without_move_retry(self):
        action = self.start()
        self.endpoint.defer_results = True
        for attempt in range(5):
            self.poll(action.result_due_at + 0.01)
            ticket = self.endpoint.tickets[-1]
            ticket.state = "failed"
            ticket.callbacks["on_failed"](self.now, 3, len(ticket.data), "partial")
            self.assertFalse(action.result_query_pending)
            self.assertEqual(action.result_queries, attempt + 1)
        self.poll(action.result_due_at + 0.01)
        self.assertEqual(action.result_queries, 5)
        self.assertFalse(self.writes(b"!"))
        self.assert_no_motion_retry()


class ResultConfigTests(unittest.TestCase):
    def test_legacy_default_false_actual_navigation_true_and_strict_bool(self):
        self.assertFalse(resolve_runtime_config("hardware", "navigation")["chassis_result_recovery"])
        root = Path(__file__).resolve().parents[1]
        actual = json.loads((root / "navigation_config.json").read_text(encoding="utf-8"))
        self.assertIs(actual["chassis_result_recovery"], True)
        self.assertIs(resolve_runtime_config("hardware", "navigation", actual)["chassis_result_recovery"], True)
        for bad in (1, "true", None):
            with self.assertRaises(RuntimeConfigError):
                resolve_runtime_config("hardware", "navigation", {"chassis_result_recovery": bad})


if __name__ == "__main__":
    unittest.main()
