import math
import queue
import threading
import unittest
from unittest.mock import Mock, patch

import mapping_runtime
from mapping_runtime import MappingRuntime
from navigation_app import NavigationApp
from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint


class Value:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class NavigationModeSwitchTests(unittest.TestCase):
    def make_app(self):
        app = object.__new__(NavigationApp)
        # Radiobutton sets its variable before invoking the command.
        app.view_mode = Value("navigation")
        app.nav_state_var = Value()
        app.nav_detail_var = Value()
        app.running, app.moving = True, False
        app.mapping_generation = 0
        app.mapping_lock = threading.RLock()
        app.mapping_tasks = queue.Queue()
        app.mapping_results = queue.Queue()
        app.mapping_snapshot = None
        app.navigator = NavigationEngine(OccupancyGrid(120, 120, 0.02), max_range_m=1)
        app._navigation_preflight = Mock(return_value=True)
        app._stop_motion_for_mode_switch = Mock()
        return app

    def test_old_radar_scan_cannot_undo_navigation_selection(self):
        app = self.make_app()
        entered, release = threading.Event(), threading.Event()
        original = mapping_runtime.complete_open_scan

        def blocked_scan(*args, **kwargs):
            entered.set()
            if not release.wait(2):
                raise TimeoutError("UI did not release scan")
            return original(*args, **kwargs)

        runtime = app.mapping_runtime = MappingRuntime(
            app.navigator, lock=app.mapping_lock, on_result=app.mapping_results.put)
        points = [ScanPoint(math.atan2(x, 0.45), math.hypot(x, 0.45), 1, True)
                  for x in [-0.25 + i * 0.025 for i in range(21)]]
        try:
            with patch.object(mapping_runtime, "complete_open_scan", blocked_scan):
                runtime.submit("session", 1, points, mode="local")
                self.assertTrue(entered.wait(2))
                app._set_view("navigation")
                release.set()
                runtime.submit("session", 2, points, mode="navigation")
                while True:
                    result = app.mapping_results.get(timeout=2)
                    self.assertIsNone(result.error)
                    if result.request.scan_sequence == 2:
                        break
                self.assertEqual(app.view_mode.get(), "navigation")
                self.assertTrue(app.navigator.auto_enabled,
                                "Old radar result disabled selected automatic navigation")
                self.assertEqual(result.snapshot.local_map_updates, 0)
                runtime.submit("session", 3, points, mode="navigation")
                result = app.mapping_results.get(timeout=2)
                self.assertIsNone(result.error)
                self.assertGreater(result.snapshot.completed_scans, 0)
                self.assertGreater(result.snapshot.grid.known_area_m2(), 0)
                self.assertTrue(app.navigator.auto_enabled)
        finally:
            release.set()
            runtime.stop()

    def test_failed_preflight_keeps_selection_and_pauses(self):
        app = self.make_app()
        app._navigation_preflight.return_value = False

        def stop():
            app.running = False
            app.navigator.set_auto(False)

        app.stop = Mock(side_effect=stop)
        app._set_view("navigation")
        self.assertEqual(app.view_mode.get(), "navigation")
        app.stop.assert_called_once()
        self.assertFalse(app.navigator.auto_enabled)
        self.assertNotEqual(app.nav_state_var.get(), "仅雷达")

    def test_switch_to_radar_stops_motion_and_disables_auto(self):
        app = self.make_app()
        app.navigator.set_auto(True)
        app.moving = True
        app._set_view("radar")
        app._stop_motion_for_mode_switch.assert_called_once()
        self.assertEqual(app.view_mode.get(), "radar")
        self.assertFalse(app.navigator.auto_enabled)


if __name__ == "__main__":
    unittest.main()
