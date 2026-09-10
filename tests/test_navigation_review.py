import math
import queue
import threading
import unittest
from unittest.mock import patch

from mapping_policy import prepare_free_space_points
from mapping_runtime import MappingRuntime
from navigation_core import NavigationEngine, OccupancyGrid, Pose2D, ScanPoint, VelocityCommand
from runtime_config import build_navigation_engine, resolve_runtime_config


def box_scan():
    return [ScanPoint(a, 0.5 / max(abs(math.sin(a)), abs(math.cos(a))), is_echo=True)
            for a in (math.tau * i / 72 for i in range(72))]


class NavigationReviewTests(unittest.TestCase):
    def test_motion_prior_survives_inflight_mapping_for_both_sources(self):
        for hardware in (False, True):
            with self.subTest(hardware=hardware):
                engine = NavigationEngine(OccupancyGrid(80, 80, 0.04))
                engine.set_auto(True)
                output = queue.Queue()
                entered, release = threading.Event(), threading.Event()
                original = NavigationEngine.process_scan

                def blocked(working, *args, **kwargs):
                    entered.set()
                    if not release.wait(5):
                        raise TimeoutError("mapping barrier")
                    return original(working, *args, **kwargs)

                runtime = MappingRuntime(engine, on_result=output.put)
                try:
                    with patch.object(NavigationEngine, "process_scan", blocked):
                        runtime.submit("test", 1, box_scan())
                        self.assertIsNone(output.get(timeout=5).error)
                        runtime.submit("test", 2, box_scan())
                        self.assertTrue(entered.wait(5))
                        version = runtime.state_version
                        if hardware:
                            runtime.apply_execution_delta(0, 0.1)
                        else:
                            runtime.predict_motion(VelocityCommand(forward_mps=0.1, duration_s=1))
                        self.assertGreater(runtime.state_version, version)
                        release.set()
                        runtime.submit("test", 3, box_scan())
                        result = output.get(timeout=5)
                        self.assertIsNone(result.error)
                        self.assertEqual(result.request.scan_sequence, 3)
                        self.assertAlmostEqual(engine.pose.y, 0.1)
                        self.assertTrue(engine._translation_since_last_scan)
                finally:
                    release.set()
                    runtime.stop()

    def test_rejected_scan_preserves_motion_until_localization_succeeds(self):
        engine = NavigationEngine(OccupancyGrid(80, 80, 0.04))
        engine.set_auto(True)
        engine.predict_motion(VelocityCommand(forward_mps=0.1, right_mps=0.1, duration_s=1))
        engine.grid.update_scan(Pose2D(), box_scan(), 3)
        count = engine.completed_scans
        for points in ([], box_scan()):
            with patch.object(engine.matcher, "match", return_value=(engine._sensor_pose(), 0.1)):
                engine.process_scan(points)
            self.assertTrue(engine._translation_since_last_scan)
            self.assertTrue(engine._motion_since_last_scan)
            self.assertTrue(engine._diagonal_motion_since_last_scan)
            self.assertEqual(engine.completed_scans, count)
        with patch.object(engine.matcher, "match", return_value=(engine._sensor_pose(), 1.0)):
            engine.process_scan(box_scan())
        self.assertEqual(engine.completed_scans, count + 1)
        self.assertFalse(engine._motion_since_last_scan)
        self.assertTrue(engine._last_scan_had_translation)

    def terminal_engine(self, distances, incoming=math.pi):
        engine = NavigationEngine(OccupancyGrid())
        engine.match_score = 0.9
        engine._last_scan_had_translation = True
        engine.trajectory = [(0.2 * math.sin(incoming), 0.2 * math.cos(incoming)), (0, 0)]
        engine.latest_scan = [ScanPoint(incoming + angle + delta, distance, is_echo=True)
                              for angle, distance in zip((math.pi, math.pi / 2, -math.pi / 2, 0), distances)
                              for delta in (-0.45, 0, 0.45)]
        return engine

    def test_side_opening_unknown_or_assumption_cannot_confirm_parking(self):
        for distances in ((0.25, 0.25, 1, 1), (0.25, 1, 0.25, 1)):
            self.assertFalse(self.terminal_engine(distances)._terminal_geometry_confirmed())
        engine = self.terminal_engine((0.25, 0.25, 0.6, 1))
        engine.latest_scan = engine.latest_scan[:6] + engine.latest_scan[9:]
        self.assertFalse(engine._terminal_geometry_confirmed())
        engine = self.terminal_engine((0.25, 0.25, 0.6, 1))
        engine.latest_scan[-3:] = [ScanPoint(math.pi + d, 1, is_echo=False, source="assumed_open")
                                  for d in (-0.1, 0, 0.1)]
        self.assertFalse(engine._terminal_geometry_confirmed())

    def test_strafed_dead_end_uses_actual_approach(self):
        engine = self.terminal_engine((0.25, 0.25, 0.6, 1), -math.pi / 2)
        self.assertTrue(engine._terminal_geometry_confirmed())
        engine.trajectory = [(0, -0.2), (0, 0)]
        engine._terminal_signature = None
        self.assertFalse(engine._terminal_geometry_confirmed())

    def test_local_repair_restores_unknown_without_moving_pose_or_erasing_other_walls(self):
        grid = OccupancyGrid()
        near, far = grid.world_to_cell(0.4, 0), grid.world_to_cell(1, 0)
        grid._add(*near, 6)
        grid._add(*far, 6)
        grid.inflated_obstacles(0.17)
        grid.clear_region(0.4, 0, 0.3)
        self.assertEqual(grid.state(*near), grid.UNKNOWN)
        self.assertEqual(grid.state(*far), grid.OCCUPIED)
        self.assertNotIn(near, grid.occupied_cells())
        self.assertEqual(grid._known_count, 1)

    def test_assumed_free_provenance_survives_preparation_and_is_replaced_by_measurement(self):
        grid = OccupancyGrid()
        rays = prepare_free_space_points([ScanPoint(0, 1, is_echo=False, source="assumed_open")], 1, 0.15, 0.04)
        cell = grid.world_to_cell(0, 0.4)
        for _ in range(3):
            grid.update_scan(Pose2D(), rays, 1, add_only=True)
        self.assertEqual(grid.state(*cell), grid.FREE)
        self.assertIn(cell, grid.assumed_free_cells)
        grid.update_scan(Pose2D(), [ScanPoint(0, 1, is_echo=False)], 1, add_only=True)
        self.assertNotIn(cell, grid.assumed_free_cells)

    def test_hardware_navigation_map_covers_reference_route_independently_of_display_radius(self):
        engine = build_navigation_engine(resolve_runtime_config("hardware", "navigation", {"display_radius_m": 1.1}))
        self.assertTrue(engine.grid.in_bounds(*engine.grid.world_to_cell(0, 4.5)))
        self.assertGreater(engine.grid.cell_to_world(0, 0)[1], 4.5 + engine.robot_radius_m)


if __name__ == "__main__":
    unittest.main()
