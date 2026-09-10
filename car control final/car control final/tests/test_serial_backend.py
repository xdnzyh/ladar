import threading
import time
import unittest
from unittest.mock import patch

from serial_backend import SerialEndpoint


class FakeTransport:
    def __init__(self, *, block_first=False, partial=False):
        self.is_open = True
        self.block_first = block_first
        self.partial = partial
        self.started = threading.Event()
        self.release = threading.Event()
        self.writes = []

    def read(self, _size):
        time.sleep(0.002)
        return b""

    def write(self, data):
        payload = bytes(data)
        self.writes.append(payload)
        if self.block_first and len(self.writes) == 1:
            self.started.set()
            self.release.wait(1.0)
        if self.partial:
            return max(0, len(payload) - 1)
        return len(payload)

    def close(self):
        self.is_open = False
        self.release.set()


class SerialEndpointQueueTests(unittest.TestCase):
    def endpoint(self, transport):
        endpoint = SerialEndpoint("test", lambda *_: None, lambda *_: None)
        with patch("serial_backend._open_transport", return_value=transport):
            endpoint.open("COM_TEST", 9600)
        self.addCleanup(endpoint.close)
        return endpoint

    def test_partial_write_reports_exact_count_and_faults_session(self):
        transport = FakeTransport(partial=True)
        endpoint = self.endpoint(transport)
        failed = threading.Event()
        details = []
        endpoint.write(
            b"@MOVE,W,100,MM\r\n",
            on_failed=lambda stamp, written, total, detail: (
                details.append((stamp, written, total, detail)), failed.set()
            ),
        )
        self.assertTrue(failed.wait(1.0))
        self.assertEqual(details[0][1:3], (len(transport.writes[0]) - 1, len(transport.writes[0])))
        self.assertFalse(endpoint.is_open)

    def test_queued_write_can_be_cancelled_but_active_write_cannot(self):
        transport = FakeTransport(block_first=True)
        endpoint = self.endpoint(transport)
        first = endpoint.write_ticket(b"FIRST")
        self.assertTrue(transport.started.wait(1.0))
        cancelled = threading.Event()
        second = endpoint.write_ticket(b"SECOND", on_cancelled=lambda _stamp: cancelled.set())
        self.assertEqual(endpoint.cancel_write(first), "started")
        self.assertEqual(endpoint.cancel_write(second), "cancelled")
        self.assertTrue(cancelled.wait(1.0))
        transport.release.set()
        self.assertTrue(endpoint.flush(1.0))
        self.assertEqual(transport.writes, [b"FIRST"])

    def test_priority_stop_discards_unsent_move_without_reordering_active_bytes(self):
        transport = FakeTransport(block_first=True)
        endpoint = self.endpoint(transport)
        endpoint.write_ticket(b"ACTIVE")
        self.assertTrue(transport.started.wait(1.0))
        cancelled = threading.Event()
        endpoint.write_ticket(b"UNSENT_MOVE", on_cancelled=lambda _stamp: cancelled.set())
        endpoint.write_ticket(b"!\r\nX\r\n", priority=True, discard_pending=True)
        self.assertTrue(cancelled.wait(1.0))
        transport.release.set()
        self.assertTrue(endpoint.flush(1.0))
        self.assertEqual(transport.writes, [b"ACTIVE", b"!\r\nX\r\n"])

    def test_close_suppresses_completion_from_retired_session(self):
        transport = FakeTransport(block_first=True)
        endpoint = self.endpoint(transport)
        written = threading.Event()
        endpoint.write_ticket(b"ACTIVE", on_written=lambda _stamp: written.set())
        self.assertTrue(transport.started.wait(1.0))
        endpoint.close()
        self.assertFalse(written.is_set())

        replacement = FakeTransport()
        with patch("serial_backend._open_transport", return_value=replacement):
            endpoint.open("COM_REOPEN", 9600)
        current_written = threading.Event()
        endpoint.write_ticket(b"CURRENT", on_written=lambda _stamp: current_written.set())
        self.assertTrue(current_written.wait(1.0))
        self.assertEqual(replacement.writes, [b"CURRENT"])


if __name__ == "__main__":
    unittest.main()
