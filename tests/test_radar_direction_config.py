import unittest
from unittest.mock import patch

from navigation_app import load_configuration
from radar_app import load_config


class RadarDirectionConfigTests(unittest.TestCase):
    def test_hardware_entries_use_the_same_saved_scan_direction(self):
        radar_clockwise = load_config()["clockwise"]

        for view in ("radar", "navigation"):
            with self.subTest(view=view):
                self.assertEqual(
                    load_configuration("hardware", view)["clockwise"],
                    radar_clockwise,
                )

    def test_radar_scan_direction_wins_over_stale_navigation_copy(self):
        radar_config = {"clockwise": False}
        stale_navigation_config = {"clockwise": True}

        for view in ("radar", "navigation"):
            with self.subTest(view=view):
                with patch(
                    "navigation_app.load_json_config",
                    side_effect=[radar_config, stale_navigation_config],
                ):
                    config = load_configuration("hardware", view)
                self.assertFalse(config["clockwise"])


if __name__ == "__main__":
    unittest.main()
