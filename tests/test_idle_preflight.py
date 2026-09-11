import json
from pathlib import Path
from types import SimpleNamespace
import unittest

from chassis_config1 import DEFAULT_PROFILE
from chassis_controller import ChassisController, ChassisState
from chassis_protocol import PROTOCOL_BANNER
from runtime_config import RuntimeConfigError, resolve_runtime_config
from test_result_recovery import Endpoint, checked, result


class PreflightEndpoint(Endpoint):
    """Deferred writes plus firmware's latest-PING, consume-once contract."""

    def __init__(self, clock):
        super().__init__(clock)
        self.defer_ping = False
        self.armed = None

    def write_ticket(self, data, **callbacks):
        ticket = SimpleNamespace(data=data, callbacks=callbacks, state="queued")
        if callbacks.get("discard_pending"):
            for old in self.tickets:
                self.cancel_write(old)
        self.tickets.append(ticket)
        if not (self.defer_ping and data.lstrip().startswith(b"@PING,")):
            self.complete(ticket)
        return ticket

    def complete(self, ticket):
        if ticket.state != "queued":
            return
        data = ticket.data.strip()
        if data.startswith(b"@PING,"):
            self.armed = data.decode().split(",")[1]
        elif data.startswith(b"@MOVE,"):
            body = data.decode().rsplit(",", 1)[0]
            nonce = body.split(",")[-1]
            assert nonce == self.armed, (nonce, self.armed)
            assert ticket.data.lstrip() == checked(body)
            self.armed = None
        super().complete(ticket)


class IdlePreflightTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.events = []
        self.endpoint = PreflightEndpoint(lambda: self.now)
        self.config = resolve_runtime_config("hardware", "navigation", {
            "chassis_idle_preflight": True, "chassis_result_recovery": True,
            "chassis_firmware_confirmed": True, "chassis_capability_mode": "mm_ping_v1",
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

    def receive(self, data, stamp=None):
        if stamp is not None:
            self.now = stamp
        self.controller.feed_data(data, self.now, self.generation)

    def poll(self, stamp):
        self.now = stamp
        self.controller.poll()

    def writes(self, prefix):
        return [(t, data) for t, data in self.endpoint.writes if data.lstrip().startswith(prefix)]

    def request(self):
        return self.controller.request_move("W", 100, operator_authorized=True)

    def prepare(self):
        self.poll(self.now + 0.5)
        nonce = self.endpoint.armed
        self.receive(f"@PONG,{nonce}\r\n".encode(), self.now + 0.125)
        self.poll(self.now + 0.25)
        self.assertTrue(self.controller.idle_preflight_ready)
        return nonce

    def test_opt_in_is_strict_default_false_and_enabled_in_actual_config(self):
        config = resolve_runtime_config("hardware", "navigation", {})
        self.assertIs(config["chassis_idle_preflight"], False)
        for value in (1, "true", None):
            with self.subTest(value=value), self.assertRaises(RuntimeConfigError):
                resolve_runtime_config("hardware", "navigation", {"chassis_idle_preflight": value})
        path = Path(__file__).resolve().parents[1] / "navigation_config.json"
        self.assertIs(json.loads(path.read_text(encoding="utf-8"))["chassis_idle_preflight"], True)
        self.controller.update_config({**self.config, "chassis_idle_preflight": False})
        self.poll(10)
        self.assertFalse(self.endpoint.writes)

    def test_idle_sends_one_ping_only_after_actual_rx_quiet(self):
        self.poll(0.49)
        self.assertFalse(self.endpoint.writes)
        self.receive(b"partial diagnostic", 0.49)
        self.poll(0.98)
        self.assertFalse(self.endpoint.writes)
        self.poll(0.99)
        self.assertEqual(len(self.writes(b"@PING,")), 1)
        self.receive(b"\r\n@PONG,FFFFFFFF\r\n", 1)
        self.assertFalse(self.controller.idle_preflight_ready)
        self.receive(b"@PONG,00000001\r\n", 1.125)
        self.poll(1.374)
        self.assertFalse(self.controller.idle_preflight_ready)
        self.poll(1.375)
        self.assertTrue(self.controller.idle_preflight_ready)
        for stamp in (2, 4, 30, 100):
            self.poll(stamp)
        self.assertEqual(len(self.endpoint.writes), 1)
        self.assertIsNone(self.controller.pending)
        statuses = [v[1] for k, v, _ in self.events if k == "chassis_status"]
        self.assertEqual(statuses.count("IDLE_PREFLIGHT=1"), 1)

    def test_prepared_request_sends_immediately_with_exact_cached_nonce(self):
        nonce = self.prepare()
        stamp = self.now
        self.assertTrue(self.request())
        self.assertEqual(self.writes(b"@MOVE,"), [(stamp, checked(f"@MOVE,W,100,CNT,{nonce}"))])
        self.assertEqual(len(self.writes(b"@PING,")), 1)
        self.assertFalse(self.controller.idle_preflight_ready)
        self.assertEqual(self.controller.state, ChassisState.WAITING_ACK)

    def test_prepared_request_waits_only_for_remaining_actual_rx_silence(self):
        self.prepare()
        self.receive(b"noise", 1)
        self.assertFalse(self.controller.idle_preflight_ready)
        self.poll(1.125)
        self.assertTrue(self.request())
        self.poll(1.249)
        self.assertFalse(self.writes(b"@MOVE,"))
        self.poll(1.25)
        self.assertEqual(self.writes(b"@MOVE,")[0][0], 1.25)
        self.assertEqual(len(self.writes(b"@PING,")), 1)

    def test_inflight_ping_is_adopted_before_pong(self):
        self.poll(0.5)
        self.assertTrue(self.request())
        self.assertEqual(self.controller.pending.nonce, "00000001")
        self.assertEqual(self.controller.state, ChassisState.WAITING_PONG)
        self.receive(b"@PONG,00000001\r\n", 0.625)
        self.poll(0.875)
        self.assertEqual(len(self.writes(b"@PING,")), 1)
        self.assertEqual(len(self.writes(b"@MOVE,")), 1)

    def test_queued_ping_adoption_keeps_callbacks_and_waits_for_write(self):
        self.endpoint.defer_ping = True
        self.poll(0.5)
        ticket = self.endpoint.tickets[-1]
        self.assertTrue(self.request())
        # A stale PONG must not prepare a PING which hasn't started writing.
        self.receive(b"@PONG,00000001\r\n", 0.625)
        self.poll(0.875)
        self.assertFalse(self.writes(b"@MOVE,"))
        self.now = 1
        self.endpoint.complete(ticket)
        self.assertEqual(self.controller.pending.ping_sent_at, 1)
        self.receive(b"@PONG,00000001\r\n", 1.125)
        self.poll(1.375)
        self.assertEqual(len(self.writes(b"@PING,")), 1)
        self.assertEqual(len(self.writes(b"@MOVE,")), 1)

    def test_completed_pong_waits_for_write_completion_callback(self):
        self.endpoint.defer_ping = True
        self.poll(0.5)
        ticket = self.endpoint.tickets[-1]
        ticket.callbacks["on_sent"](self.now)
        self.receive(b"@PONG,00000001\r\n", 0.625)
        self.assertTrue(self.request())
        self.poll(0.875)
        self.assertFalse(self.writes(b"@MOVE,"))
        self.endpoint.complete(ticket)
        self.poll(0.875)
        self.assertEqual(len(self.writes(b"@MOVE,")), 1)

    def test_background_timeout_stops_polling_and_request_uses_bounded_fallback(self):
        self.poll(0.5)
        self.poll(2.5)
        self.poll(10)
        self.assertEqual(len(self.writes(b"@PING,")), 1)
        self.assertTrue(self.request())
        self.poll(10)
        self.assertEqual(self.controller.pending.nonce, "00000002")
        self.receive(b"@PONG,00000001\r\n", 10.125)
        self.assertEqual(self.controller.state, ChassisState.WAITING_PONG)
        self.poll(12)
        self.assertIsNone(self.controller.pending)
        self.assertFalse(self.writes(b"@MOVE,"))

    def test_adopted_timeout_cancels_old_ticket_before_foreground_fallback(self):
        self.endpoint.defer_ping = True
        self.poll(0.5)
        old = self.endpoint.tickets[-1]
        self.assertTrue(self.request())
        self.poll(2.5)
        self.assertEqual(old.state, "cancelled")
        self.endpoint.defer_ping = False
        self.poll(2.5)
        self.receive(b"@PONG,00000002\r\n", 2.625)
        self.poll(2.875)
        action = self.controller.pending
        self.endpoint.complete(old)
        for name in ("on_sent", "on_written", "on_cancelled"):
            old.callbacks[name](3)
        old.callbacks["on_failed"](3, 0, 16, "late failure")
        self.assertIs(self.controller.pending, action)
        self.assertEqual(action.nonce, "00000002")
        self.assertEqual(self.controller.state, ChassisState.WAITING_ACK)
        self.assertEqual(len(self.writes(b"@MOVE,")), 1)
        self.assertEqual(len(self.writes(b"@PING,")), 1)

    def test_restart_disconnect_stop_update_and_uncertainty_cancel_preflight(self):
        for operation in ("restart", "disconnect", "begin", "stop", "update", "fault"):
            with self.subTest(operation=operation):
                self.setUp()
                self.endpoint.defer_ping = True
                self.poll(0.5)
                old = self.endpoint.tickets[-1]
                if operation == "restart":
                    self.receive(PROTOCOL_BANNER.encode() + b"\r\n")
                elif operation == "disconnect":
                    self.controller.disconnect()
                elif operation == "begin":
                    self.controller.begin_connection()
                elif operation == "stop":
                    self.controller.request_stop()
                elif operation == "update":
                    self.controller.update_config(dict(self.config))
                else:
                    self.controller.mark_execution_failed("uncertain")
                self.assertEqual(old.state, "cancelled")
                old.callbacks["on_written"](self.now)
                self.assertIsNone(self.controller._idle_preflight)
                self.assertFalse(self.controller.idle_preflight_ready)
                self.assertFalse(self.writes(b"@MOVE,"))

    def test_restart_invalidates_prepared_nonce_and_requires_fresh_ping(self):
        self.prepare()
        self.receive(PROTOCOL_BANNER.encode() + b"\r\n", 1)
        self.assertFalse(self.controller.idle_preflight_ready)
        self.assertFalse(self.request())
        self.receive(b"IDLE X=0 Y=0 R=0 S=0\r\n", 1.125)
        self.poll(1.625)
        self.assertEqual(self.endpoint.armed, "00000002")

    def test_manual_status_blocks_preflight_until_idle_reply(self):
        self.prepare()
        self.assertTrue(self.controller.request_status())
        self.assertFalse(self.controller.idle_preflight_ready)
        self.poll(5)
        self.assertEqual(len(self.writes(b"@PING,")), 1)
        self.assertFalse(self.request())
        self.assertFalse(self.controller.request_communication_check(1))
        self.receive(b"IDLE X=0 Y=0 R=0 S=0\r\n", 5)
        self.poll(5.5)
        self.assertEqual(len(self.writes(b"@PING,")), 2)

    def test_check_cancels_queued_preflight_and_never_adopts_its_pong(self):
        self.endpoint.defer_ping = True
        self.poll(0.5)
        old = self.endpoint.tickets[-1]
        self.assertTrue(self.controller.request_communication_check(1))
        self.assertEqual(old.state, "cancelled")
        self.endpoint.defer_ping = False
        self.poll(0.5)
        self.receive(b"@PONG,00000001\r\n", 0.625)
        self.assertEqual(self.controller.communication_check.completed, 0)
        self.receive(b"@PONG,00000002\r\n", 0.75)
        self.assertIsNone(self.controller.communication_check)
        self.assertFalse(self.controller.idle_preflight_ready)

    def test_unverified_config_allows_preflight_but_sync_cancels_it(self):
        config = resolve_runtime_config("hardware", "navigation", {
            **self.config, "chassis_config1_file": DEFAULT_PROFILE,
        })
        self.controller.update_config(config)
        self.assertFalse(self.controller.config_verified)
        self.endpoint.defer_ping = True
        self.poll(0.5)
        old = self.endpoint.tickets[-1]
        self.assertTrue(old.data.lstrip().startswith(b"@PING,"))
        self.assertTrue(self.controller.request_config_sync(apply=False))
        self.assertEqual(old.state, "cancelled")
        self.poll(0.5)
        self.assertTrue(self.writes(b"@CFG,"))
        self.assertFalse(self.writes(b"@PING,"))
        self.assertFalse(self.request())
        self.assertFalse(self.controller.request_status())
        self.assertFalse(self.controller.request_communication_check(1))

    def test_sent_ping_drains_before_read_check_or_manual_status(self):
        for operation in ("read", "check", "status"):
            with self.subTest(operation=operation):
                self.setUp()
                if operation == "read":
                    self.controller.update_config(resolve_runtime_config("hardware", "navigation", {
                        **self.config, "chassis_config1_file": DEFAULT_PROFILE,
                    }))
                    self.controller.config_verified = True
                self.poll(0.5)
                self.assertEqual(len(self.writes(b"@PING,")), 1)
                if operation == "read":
                    self.assertTrue(self.controller.request_config_sync(apply=False))
                    prefix = b"@CFG,"
                elif operation == "check":
                    self.assertTrue(self.controller.request_communication_check(1))
                    prefix = b"@PING,"
                else:
                    self.assertTrue(self.controller.request_status())
                    prefix = b"P\r\n"
                before = len(self.writes(prefix))
                self.poll(1)
                self.receive(b"@PONG,00000001\r\n", 2.25)
                self.poll(2.5)  # Original PING deadline; still need quiet.
                self.poll(2.999)
                self.assertEqual(len(self.writes(prefix)), before)
                self.receive(b"late diagnostic\r\n", 2.999)
                self.poll(3.498)
                self.assertEqual(len(self.writes(prefix)), before)
                self.poll(3.5)
                self.assertEqual(len(self.writes(prefix)), before + 1)
                self.assertFalse(self.writes(b"@MOVE,"))
                self.assertFalse(self.controller.idle_preflight_ready)

    def test_started_ping_cancel_race_reserves_response_window(self):
        self.endpoint.defer_ping = True
        self.poll(0.5)
        ticket = self.endpoint.tickets[-1]
        # The writer selected the ticket before its on_sent callback took the lock.
        ticket.state = "started"
        self.assertTrue(self.controller.request_communication_check(1))
        ticket.callbacks["on_sent"](0.5)
        ticket.callbacks["on_written"](0.625)
        self.endpoint.defer_ping = False
        self.poll(2.999)
        self.assertFalse(self.writes(b"@PING,"))
        self.poll(3)
        self.assertEqual(self.writes(b"@PING,")[0][1], b"@PING,00000002\r\n")

    def test_write_failure_and_cancellation_fall_back_without_reusing_nonce(self):
        for callback in ("on_failed", "on_cancelled"):
            with self.subTest(callback=callback):
                self.setUp()
                self.endpoint.defer_ping = True
                self.poll(0.5)
                old = self.endpoint.tickets[-1]
                self.assertTrue(self.request())
                args = (self.now, 0, 16, "write failed") if callback == "on_failed" else (self.now,)
                old.callbacks[callback](*args)
                self.assertEqual(old.state, "cancelled")
                self.endpoint.defer_ping = False
                self.poll(0.5)
                self.assertEqual(self.controller.pending.nonce, "00000002")
                self.receive(b"@PONG,00000002\r\n", 0.625)
                self.poll(0.875)
                self.assertEqual(len(self.writes(b"@MOVE,")), 1)

    def test_preflight_is_limited_to_confirmed_idle(self):
        for state in vars(ChassisState).values():
            if not isinstance(state, str) or state == ChassisState.IDLE:
                continue
            with self.subTest(state=state):
                self.setUp()
                self.controller.state = state
                self.poll(1)
                self.assertFalse(self.writes(b"@PING,"))
        for attr in ("_motion_unconfirmed", "confirmed"):
            self.setUp()
            setattr(self.controller, attr, attr == "_motion_unconfirmed")
            self.poll(1)
            self.assertFalse(self.writes(b"@PING,"))

    def test_repeated_moves_use_unique_consumed_nonces_and_preserve_result_recovery(self):
        nonces = []
        for _ in range(3):
            nonce = self.prepare()
            nonces.append(nonce)
            self.assertTrue(self.request())
            action = self.controller.pending
            self.receive(b"@RESULT,damaged\r\n", self.now + 0.125)
            self.poll(action.result_due_at)
            self.assertEqual(self.writes(b"@RESULT,")[-1][1], checked(f"@RESULT,{nonce}"))
            self.receive(result(nonce=nonce), self.now + 0.125)
            self.assertEqual(self.controller.state, ChassisState.SETTLING)
            self.poll(self.now + 1)
            self.assertEqual(len(self.writes(b"@PING,")), len(nonces))
            self.assertTrue(self.controller.complete_settle(resume_auto=False))
        self.assertEqual(len(set(nonces)), 3)
        self.assertEqual(len(self.writes(b"@MOVE,")), 3)
        self.assertEqual(len(self.writes(b"@PING,")), 3)


if __name__ == "__main__":
    unittest.main()
