from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

from runtime_config import (
    CHASSIS_FIXED_COUNTS_PER_MM,
    RUNTIME_DEFAULTS,
    RuntimeConfigError,
    resolve_runtime_config,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class ChassisRuntimeConfigTests(unittest.TestCase):
    def test_defaults_preserve_radar_settings_and_chassis_evidence_state(self):
        config = resolve_runtime_config("hardware", "navigation")

        self.assertEqual(config["hardware_sample_rate_hz"], 100.0)
        self.assertEqual(config["sample_rate_hz"], 100.0)
        self.assertEqual(config["exposure_index"], 5)
        self.assertEqual(config["actual_exposure_index"], 5)
        self.assertEqual(config["chassis_baudrate"], 9600)
        self.assertEqual(config["chassis_capability_mode"], "unknown")
        self.assertFalse(config["chassis_firmware_confirmed"])
        self.assertEqual(config["chassis_preferred_translation_unit"], "MM")
        self.assertTrue(config["chassis_raw_log_enabled"])
        self.assertEqual(config["chassis_raw_log_file"], "logs/chassis_serial.jsonl")
        self.assertEqual(config["chassis_tx_max_command_bytes"], 47)
        self.assertEqual(config["chassis_max_line_bytes"], 1024)
        self.assertEqual(config["chassis_rx_silence_before_ping_s"], 0.5)
        self.assertEqual(config["chassis_rx_silence_after_pong_s"], 0.25)
        self.assertEqual(config["chassis_silence_wait_timeout_s"], 4.0)
        self.assertEqual(config["chassis_ping_timeout_s"], 2.0)
        self.assertEqual(config["chassis_total_timeout_s"], 12.0)

        capabilities = config["chassis_translation_capabilities"]
        self.assertEqual(set(capabilities), set("WSADQEZC"))
        for mode, fixed in CHASSIS_FIXED_COUNTS_PER_MM.items():
            entry = capabilities[mode]
            self.assertEqual(entry["fixed_counts_per_mm"], fixed)
            self.assertEqual(entry["coefficient_status"], "initial")
            self.assertFalse(entry["motion_range_validated"])
            self.assertIsNone(entry["validated_min_mm"])
            self.assertIsNone(entry["validated_max_mm"])
            self.assertIsNone(entry["uncertainty_m"])
            self.assertTrue(entry["enabled"])

        rotations = config["chassis_rotation_capabilities"]
        self.assertEqual(set(rotations), {"R", "F"})
        for entry in rotations.values():
            self.assertEqual(entry["unit"], "CNT")
            self.assertEqual(entry["status"], "unvalidated")
            self.assertFalse(entry["enabled"])
            self.assertIsNone(entry["counts_per_rad"])
            self.assertIsNone(entry["uncertainty_rad"])

    def test_checked_in_navigation_config_resolves(self):
        with (PROJECT_ROOT / "navigation_config.json").open(encoding="utf-8") as stream:
            saved = json.load(stream)
        config = resolve_runtime_config("hardware", "navigation", saved)
        self.assertEqual(config["hardware_sample_rate_hz"], 100.0)
        self.assertEqual(config["sample_rate_hz"], 100.0)
        self.assertEqual(config["exposure_index"], 5)
        self.assertEqual(config["chassis_translation_capabilities"]["W"]["counts_per_mm"], 7.1775)

    def test_firmware_fixed_point_values_match_all_100_mm_examples(self):
        config = resolve_runtime_config("hardware", "navigation")
        actual = {
            mode: (100 * entry["fixed_counts_per_mm"] + 5000) // 10000
            for mode, entry in config["chassis_translation_capabilities"].items()
        }
        self.assertEqual(
            actual,
            {"W": 718, "S": 713, "A": 724, "D": 739,
             "Q": 1018, "E": 912, "Z": 965, "C": 1019},
        )

    def test_legacy_and_total_timeout_names_stay_consistent(self):
        legacy = resolve_runtime_config(
            "hardware", "navigation", {"chassis_action_timeout_s": 13.0}
        )
        self.assertEqual(legacy["chassis_total_timeout_s"], 13.0)
        modern = resolve_runtime_config(
            "hardware", "navigation", {"chassis_total_timeout_s": 14.0}
        )
        self.assertEqual(modern["chassis_action_timeout_s"], 14.0)
        with self.assertRaises(RuntimeConfigError):
            resolve_runtime_config(
                "hardware", "navigation",
                {"chassis_action_timeout_s": 12.0, "chassis_total_timeout_s": 13.0},
            )

    def test_validated_translation_range_requires_complete_finite_evidence(self):
        table = deepcopy(RUNTIME_DEFAULTS["chassis_translation_capabilities"])
        table["W"].update({
            "motion_range_validated": True,
            "validated_min_mm": 95.0,
            "validated_max_mm": 190.0,
            "uncertainty_m": 0.02,
        })
        config = resolve_runtime_config(
            "hardware", "navigation", {"chassis_translation_capabilities": table}
        )
        self.assertEqual(config["chassis_translation_capabilities"]["W"]["validated_max_mm"], 190.0)

        for bad_update in (
            {"validated_min_mm": 100.0},
            {"motion_range_validated": True, "validated_min_mm": 200.0,
             "validated_max_mm": 100.0, "uncertainty_m": 0.01},
            {"motion_range_validated": True, "validated_min_mm": 10.0,
             "validated_max_mm": 100.0, "uncertainty_m": math.nan},
        ):
            with self.subTest(bad_update=bad_update):
                invalid = deepcopy(RUNTIME_DEFAULTS["chassis_translation_capabilities"])
                invalid["W"].update(bad_update)
                with self.assertRaises(RuntimeConfigError):
                    resolve_runtime_config(
                        "hardware", "navigation", {"chassis_translation_capabilities": invalid}
                    )

    def test_invalid_capability_units_directions_and_numeric_ranges_are_rejected(self):
        cases = [
            {"chassis_firmware_confirmed": "false"},
            {"chassis_raw_log_enabled": "true"},
            {"chassis_raw_log_file": ""},
            {"chassis_raw_log_file": "../outside.jsonl"},
            {"chassis_raw_log_file": str(PROJECT_ROOT / "logs" / "outside.jsonl")},
            {"clockwise": "false"},
            {"chassis_capability_mode": "MM"},
            {"chassis_capability_mode": "unknown", "chassis_firmware_confirmed": True},
            {"chassis_capability_mode": "cnt_only", "chassis_preferred_translation_unit": "MM"},
            {"chassis_preferred_translation_unit": "mm"},
            {"chassis_tx_max_command_bytes": 48},
            {"chassis_tx_max_command_bytes": "47"},
            {"chassis_max_line_bytes": 63},
            {"chassis_ping_timeout_s": math.inf},
            {"safety_speed_upper_bound_mps": math.nan},
            {"safety_stop_distance_m": -0.01},
            {"chassis_speed_validated": True},
            {"chassis_braking_validated": True},
            {"robot_radius_m": 0.0},
            {"measurement_port": "COM3", "rotation_port": "com3"},
        ]
        for overrides in cases:
            with self.subTest(overrides=overrides), self.assertRaises(RuntimeConfigError):
                resolve_runtime_config("hardware", "navigation", overrides)

        wrong_fixed = deepcopy(RUNTIME_DEFAULTS["chassis_translation_capabilities"])
        wrong_fixed["D"]["fixed_counts_per_mm"] = 73863
        missing_direction = deepcopy(RUNTIME_DEFAULTS["chassis_translation_capabilities"])
        missing_direction.pop("C")
        unknown_direction = deepcopy(RUNTIME_DEFAULTS["chassis_translation_capabilities"])
        unknown_direction["R"] = unknown_direction.pop("C")
        reversed_brake = deepcopy(RUNTIME_DEFAULTS["chassis_translation_capabilities"])
        reversed_brake["W"]["brake_min_counts"] = 1401
        nested_string_bool = deepcopy(RUNTIME_DEFAULTS["chassis_translation_capabilities"])
        nested_string_bool["W"]["enabled"] = "true"
        for table in (wrong_fixed, missing_direction, unknown_direction, reversed_brake, nested_string_bool):
            with self.subTest(table=table), self.assertRaises(RuntimeConfigError):
                resolve_runtime_config(
                    "hardware", "navigation", {"chassis_translation_capabilities": table}
                )

        rotations = deepcopy(RUNTIME_DEFAULTS["chassis_rotation_capabilities"])
        rotations["R"]["unit"] = "DEG"
        with self.assertRaises(RuntimeConfigError):
            resolve_runtime_config(
                "hardware", "navigation", {"chassis_rotation_capabilities": rotations}
            )

        rotations = deepcopy(RUNTIME_DEFAULTS["chassis_rotation_capabilities"])
        rotations["R"]["enabled"] = True
        with self.assertRaises(RuntimeConfigError):
            resolve_runtime_config(
                "hardware", "navigation", {"chassis_rotation_capabilities": rotations}
            )


if __name__ == "__main__":
    unittest.main()
