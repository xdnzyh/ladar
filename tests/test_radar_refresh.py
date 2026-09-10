from copy import deepcopy
import math
import queue
import threading
import unittest
from unittest.mock import Mock, patch

from mapping_runtime import MappingRuntime
from navigation_app import NavigationApp
from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint


class RadarRefreshTests(unittest.TestCase):
    def test_mapping_commit_keeps_live_engine_readable_during_state_copy(self):
        engine = NavigationEngine(OccupancyGrid(80, 80, 0.04))
        output = queue.Queue()
        copying, release = threading.Event(), threading.Event()

        def delayed_copy(value, *args, **kwargs):
            if isinstance(value, dict) and "match_score" in value:
                copying.set()
                if not release.wait(5):
                    raise TimeoutError("commit copy barrier")
            return deepcopy(value, *args, **kwargs)

        runtime = MappingRuntime(engine, on_result=output.put)
        try:
            with patch("mapping_runtime.deepcopy", side_effect=delayed_copy):
                runtime.submit("session", 1, [], mode="local")
                self.assertTrue(copying.wait(5))
                before = dict(engine.__dict__)
                release.set()
                result = output.get(timeout=5)
            self.assertIsNone(result.error)
            self.assertIn("match_score", before)
            self.assertIn("grid", before)
            self.assertIn("pose", before)
            self.assertIs(runtime.navigator, engine)
        finally:
            release.set()
            runtime.stop()

    def test_one_draw_failure_does_not_cancel_future_frames(self):
        app = object.__new__(NavigationApp)
        app._closed = False
        app.root = Mock()
        app._draw_frame = Mock(side_effect=[RuntimeError("frame failure"), None])
        with self.assertRaisesRegex(RuntimeError, "frame failure"):
            app._draw()
        app.root.after.assert_called_once_with(50, app._draw)
        app.root.after.call_args.args[1]()
        self.assertEqual(app._draw_frame.call_count, 2)
        self.assertEqual(app.root.after.call_count, 2)

    def test_closed_window_does_not_schedule_another_frame(self):
        app = object.__new__(NavigationApp)
        app._closed = False
        app.root = Mock()
        app._draw_frame = Mock(side_effect=lambda: setattr(app, "_closed", True))
        app._draw()
        app._draw()
        app.root.after.assert_not_called()
        app._draw_frame.assert_called_once()

    def test_clear_during_mapping_discards_old_frame_and_rebuilds(self):
        app = object.__new__(NavigationApp)
        app.source = "hardware"
        app.running, app.moving = True, False
        app.chassis_controller = None
        app.mapping_generation = 0
        app.mapping_lock = threading.RLock()
        app.mapping_tasks, app.mapping_results = queue.Queue(), queue.Queue()
        app.mapping_snapshot = None
        app.view_mode = Mock(get=lambda: "radar")
        app._log = Mock()
        app.navigator = NavigationEngine(OccupancyGrid(80, 80, 0.04))
        app.grid = app.navigator.grid
        app.grid._add(*app.grid.world_to_cell(0, 0.4), 6)
        runtime = app.mapping_runtime = MappingRuntime(
            app.navigator, lock=app.mapping_lock, on_result=app.mapping_results.put)
        entered, release = threading.Event(), threading.Event()
        from mapping_policy import process_radar_debug_scan

        def delayed_scan(*args, **kwargs):
            entered.set()
            if not release.wait(5):
                raise TimeoutError("mapping barrier")
            return process_radar_debug_scan(*args, **kwargs)

        points = [ScanPoint(math.atan2(x, 0.45), math.hypot(x, 0.45), is_echo=True)
                  for x in (-0.25 + i * 0.025 for i in range(21))]
        try:
            with patch("mapping_runtime.process_radar_debug_scan", side_effect=delayed_scan):
                runtime.submit("session", 1, points, mode="local")
                self.assertTrue(entered.wait(5))
                app.clear_local_map()
                self.assertFalse(app.grid.occupied_cells())
                self.assertIsNone(app.mapping_snapshot)
                release.set()
                runtime.submit("session", 2, points, mode="local")
                first = app.mapping_results.get(timeout=5)
                self.assertEqual(first.request.scan_sequence, 2)
                runtime.submit("session", 3, points, mode="local")
                second = app.mapping_results.get(timeout=5)
                self.assertIsNone(second.error)
                app.mapping_results.put(first)
                app.mapping_results.put(second)
                app._handle_mapping_results()
                self.assertTrue(app.grid.occupied_cells())
                self.assertEqual(app.mapping_snapshot.generation, app.mapping_generation)
        finally:
            release.set()
            runtime.stop()


if __name__ == "__main__":
    unittest.main()
