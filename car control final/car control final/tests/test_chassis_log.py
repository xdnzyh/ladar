import json
from pathlib import Path
import unittest
from unittest.mock import mock_open, patch

from chassis_log import ChassisTrafficLogger


class ChassisTrafficLoggerTests(unittest.TestCase):
    def test_raw_payload_and_host_timestamp_are_preserved(self):
        path = Path("traffic.jsonl")
        stream = mock_open()
        logger = ChassisTrafficLogger(path)
        with patch.object(Path, "mkdir"), patch.object(Path, "open", stream):
            logger.record("RX", b"@ACK,W,100,MM\r\n\x00", 12.3456789,
                          connection_generation=3, action_id=8)
        item = json.loads(stream().write.call_args.args[0])
        self.assertEqual(item["direction"], "RX")
        self.assertEqual(item["connection_generation"], 3)
        self.assertEqual(item["action_id"], 8)
        self.assertEqual(item["length"], 16)
        self.assertTrue(item["hex"].endswith("00"))
        self.assertEqual(item["host_monotonic_s"], 12.3456789)

    def test_disabled_logger_does_not_create_a_file(self):
        path = Path("traffic.jsonl")
        with patch.object(Path, "open") as open_file:
            ChassisTrafficLogger(path, enabled=False).record(
                "TX", b"P\r\n", 1.0, connection_generation=1,
            )
        open_file.assert_not_called()


if __name__ == "__main__":
    unittest.main()
