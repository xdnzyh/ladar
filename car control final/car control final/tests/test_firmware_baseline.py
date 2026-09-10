import hashlib
import json
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class CurrentFirmwareBaselineTests(unittest.TestCase):
    def test_manifest_matches_user_supplied_files(self):
        manifest_path = ROOT / "firmware_baseline" / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.assertEqual(manifest["status"], "user_supplied_burned_baseline")
        for name in ("rotation", "measurement"):
            entry = manifest[name]
            path = ROOT / "firmware_baseline" / entry["file"]
            digest = hashlib.sha256(path.read_bytes()).hexdigest().upper()
            self.assertEqual(digest, entry["sha256"])

    def test_measurement_baseline_exposes_fixed_hardware_limits(self):
        manifest = json.loads((ROOT / "firmware_baseline" / "manifest.json").read_text(encoding="utf-8"))
        path = ROOT / "firmware_baseline" / manifest["measurement"]["file"]
        text = path.read_text(encoding="utf-8")
        self.assertIn('VERSION = "CCD-PEAK-RAW-CAL-3.0"', text)
        self.assertIn("DEFAULT_EXPOSURE = 5", text)
        self.assertIn("COORDINATE_MAX = 1500", text)
        self.assertNotIn('elif cmd.startswith("START ")', text)

    def test_synchronized_candidate_uses_current_calibration_exposure(self):
        path = ROOT / "firmware_candidates" / "measurement_main_MEASUREMENT_SYNC_CAL_V3_candidate.py"
        text = path.read_text(encoding="utf-8")
        self.assertIn("EXPOSURE_INDEX = 5", text)
        self.assertIn('command = b"@c0071#@" if mode == "fffe"', text)

    def test_rotation_baseline_is_micropython_session_protocol(self):
        path = ROOT / "firmware_baseline" / "rotation_main_ROTATION_SYNC_MP_V2.py"
        text = path.read_text(encoding="utf-8")
        self.assertIn('VERSION = "ROTATION_SYNC_MP_V2"', text)
        self.assertRegex(text, re.compile(r'"TRIG \{\} \{\} \{\}"'))
