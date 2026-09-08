import json
import unittest
from unittest.mock import Mock

from app_utils import load_json_config, save_json


class JsonConfigTests(unittest.TestCase):
    def test_load_merges_file_without_mutating_nested_defaults(self):
        path = Mock()
        path.exists.return_value = True
        path.read_text.return_value = '{"port": "COM3"}'
        defaults = {"port": "", "calibration": {"points": []}}

        loaded = load_json_config(path, defaults)
        loaded["calibration"]["points"].append([1, 2])

        self.assertEqual(loaded["port"], "COM3")
        self.assertEqual(defaults["calibration"]["points"], [])

    def test_invalid_json_keeps_defaults(self):
        path = Mock()
        path.exists.return_value = True
        path.read_text.return_value = "not json"

        self.assertEqual(load_json_config(path, {"port": ""}), {"port": ""})

    def test_save_replaces_file_with_utf8_json(self):
        path = Mock(suffix=".json")
        temporary = path.with_suffix.return_value

        save_json(path, {"status": "正常"}, trailing_newline=True)

        written = temporary.write_text.call_args.args[0]
        self.assertEqual(json.loads(written), {"status": "正常"})
        self.assertTrue(written.endswith("\n"))
        path.with_suffix.assert_called_once_with(".json.tmp")
        temporary.replace.assert_called_once_with(path)


if __name__ == "__main__":
    unittest.main()
