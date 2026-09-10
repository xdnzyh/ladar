import math
import unittest
from unittest.mock import patch

from navigation_core import CorrelativeScanMatcher, NavigationEngine, OccupancyGrid, Pose2D, ScanPoint


def room_fixture():
    grid = OccupancyGrid()
    for col in range(grid.width):
        for row in range(grid.height):
            x, y = grid.cell_to_world(col, row)
            if ((-1.2 <= x <= 1.6 and (abs(y + 0.8) < 0.001 or abs(y - 2.0) < 0.001))
                    or (-0.8 <= y <= 2.0 and (abs(x + 1.2) < 0.001 or abs(x - 1.6) < 0.001))):
                grid._add(col, row, 10)
    grid.update_count = 3
    points = []
    for index in range(60):
        angle = math.tau * index / 60
        sine, cosine = math.sin(angle), math.cos(angle)
        distances = []
        if abs(sine) > 1e-10:
            distances.append((1.6 if sine > 0 else -1.2) / sine)
        if abs(cosine) > 1e-10:
            distances.append((2.0 if cosine > 0 else -0.8) / cosine)
        points.append(ScanPoint(angle, min(distances)))
    return grid, points


class MappingTests(unittest.TestCase):
    def test_low_confidence_scan_does_not_update_map(self):
        grid, points = room_fixture()
        navigator = NavigationEngine(grid)
        navigator.set_auto(True)
        navigator.pose = Pose2D(0.02, 0.01, 0.005)
        before = list(grid.log_odds), grid.update_count, vars(navigator.pose).copy()
        with patch.object(navigator.matcher, "match", return_value=(Pose2D(1, 1, 1), 0.1)):
            for _ in range(3):
                self.assertTrue(navigator.process_scan(points).stopped)
        self.assertEqual((grid.log_odds, grid.update_count, vars(navigator.pose)), before)
        self.assertEqual(navigator.state, "定位丢失")
        self.assertEqual(navigator.rejected_scans, 3)

    def test_recovery_clears_failure_count(self):
        grid, points = room_fixture()
        navigator = NavigationEngine(grid)
        navigator.set_auto(True)
        with patch.object(navigator.matcher, "match", return_value=(Pose2D(), 0.01)):
            navigator.process_scan(points)
        with patch.object(navigator.matcher, "match", return_value=(Pose2D(), 0.9)):
            navigator.process_scan(points)
        self.assertEqual(navigator.match_failures, 0)
        self.assertEqual(grid.update_count, 4)

    def test_single_outlier_and_duplicates_do_not_create_obstacle(self):
        grid = OccupancyGrid()
        point = ScanPoint(0, 1)
        grid.update_scan(Pose2D(), [point] * 100, 3)
        endpoint = grid.world_to_cell(0, 1)
        self.assertEqual(grid.state(*endpoint), grid.UNKNOWN)
        self.assertLessEqual(grid.value(*endpoint), 2)
        for _ in range(5):
            grid.update_scan(Pose2D(), [ScanPoint(0, 3)], 3)
        self.assertEqual(grid.state(*endpoint), grid.FREE)

    def test_low_quality_interpolated_endpoint_is_ignored(self):
        grid = OccupancyGrid()
        for _ in range(20):
            grid.update_scan(Pose2D(), [ScanPoint(0, 1, 0.1)], 3)
        self.assertEqual(grid.update_count, 0)
        self.assertTrue(all(value == 0 for value in grid.log_odds))

    def test_quality_and_confidence_reduce_hit_weight(self):
        values = []
        for quality, confidence in ((1, 1), (0.5, 1), (1, 0.6)):
            grid = OccupancyGrid()
            grid.update_scan(Pose2D(), [ScanPoint(0, 1, quality)], 3, scan_confidence=confidence)
            values.append(grid.value(*grid.world_to_cell(0, 1)))
        self.assertAlmostEqual(values[1], values[0] * 0.5)
        self.assertAlmostEqual(values[2], values[0] * 0.6)
        grid.update_scan(Pose2D(), [ScanPoint(0, 1)], 3, scan_confidence=0.1)
        self.assertEqual(grid.update_count, 1)

    def test_max_range_clears_endpoint_without_obstacle(self):
        grid = OccupancyGrid()
        for _ in range(3):
            grid.update_scan(Pose2D(), [ScanPoint(0, 3)], 3)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 3)), grid.FREE)
        self.assertFalse(grid.occupied_cells())

    def test_distance_field_matches_exact_euclidean_distance_and_invalidates(self):
        grid = OccupancyGrid(20, 20, 0.04)
        grid._add(5, 5, 5)
        field = grid.likelihood_field(0.2)
        self.assertAlmostEqual(field[9 * 20 + 8], math.exp(-0.2 ** 2 / (2 * 0.2 ** 2)))
        self.assertIs(field, grid.likelihood_field(0.2))
        grid._add(8, 9, 5)
        self.assertEqual(grid.likelihood_field(0.2)[9 * 20 + 8], 1)
        grid.clear()
        self.assertTrue(all(value == 0 for value in grid.likelihood_field(0.2)))

    def test_distance_field_with_multiple_obstacles(self):
        grid = OccupancyGrid(12, 10, 0.04)
        obstacles = [(1, 1), (8, 2), (3, 8), (10, 9)]
        for cell in obstacles:
            grid._add(*cell, 5)
        field = grid.likelihood_field(0.08)
        for row in range(grid.height):
            for col in range(grid.width):
                squared = min((col - x) ** 2 + (row - y) ** 2 for x, y in obstacles) * 0.04 ** 2
                self.assertAlmostEqual(field[row * grid.width + col], math.exp(-squared / (2 * 0.08 ** 2)))

    def test_low_information_after_initialization_counts_as_failure(self):
        grid, _ = room_fixture()
        navigator = NavigationEngine(grid)
        navigator.set_auto(True)
        before = list(grid.log_odds)
        for _ in range(3):
            self.assertTrue(navigator.process_scan([]).stopped)
        self.assertEqual(navigator.state, "定位丢失")
        self.assertEqual(navigator.mapping_attempts, 3)
        self.assertEqual(grid.log_odds, before)

    def test_matcher_recovers_subcell_pose_error(self):
        grid, points = room_fixture()
        predicted = Pose2D(0.02, -0.02, math.radians(0.7))
        pose, confidence = CorrelativeScanMatcher().match(grid, predicted, points)
        self.assertGreater(confidence, 0.8)
        self.assertLess(math.hypot(pose.x, pose.y), 0.01)
        self.assertLess(abs(math.degrees(pose.yaw)), 0.3)

    def test_matcher_recovers_larger_motion_error(self):
        grid, points = room_fixture()
        pose, confidence = CorrelativeScanMatcher().match(grid, Pose2D(0.15, -0.10, math.radians(5)), points)
        self.assertGreater(confidence, 0.8)
        self.assertLess(math.hypot(pose.x, pose.y), 0.02)
        self.assertLess(abs(math.degrees(pose.yaw)), 0.5)

    def test_unrelated_scan_is_rejected_by_real_matcher(self):
        grid, _ = room_fixture()
        points = [ScanPoint(math.tau * i / 30, 0.3) for i in range(30)]
        navigator = NavigationEngine(grid)
        navigator.set_auto(True)
        before = list(grid.log_odds)
        self.assertTrue(navigator.process_scan(points).stopped)
        self.assertEqual(grid.log_odds, before)
        self.assertEqual(navigator.state, "定位不可信")

    def test_mapping_does_not_add_interpolated_endpoints(self):
        grid, dense = room_fixture()
        points = dense[::2]
        navigator = NavigationEngine(grid)
        navigator.set_auto(True)
        with patch.object(navigator.matcher, "match", return_value=(Pose2D(), 0.9)), patch.object(grid, "update_scan") as update:
            navigator.process_scan(points)
        self.assertEqual(update.call_args.args[1], points)

    def test_initialization_waits_for_repeat_scans(self):
        _, points = room_fixture()
        navigator = NavigationEngine()
        navigator.set_auto(True)
        self.assertTrue(navigator.process_scan(points).stopped)
        self.assertFalse(navigator.grid.occupied_cells())
        self.assertEqual(navigator.state, "建图初始化")
        self.assertTrue(navigator.process_scan(points).stopped)
        navigator.process_scan(points)
        self.assertTrue(navigator.grid.occupied_cells())

    def test_invalid_angles_and_quality_do_not_write_map(self):
        navigator = NavigationEngine()
        navigator.set_auto(True)
        points = [ScanPoint(float("nan"), 1), ScanPoint(0, 1, float("nan")), ScanPoint(1, float("inf"))] * 10
        self.assertTrue(navigator.process_scan(points).stopped)
        self.assertEqual(navigator.grid.update_count, 0)

    def test_all_max_range_scan_cannot_initialize_localization(self):
        navigator = NavigationEngine()
        navigator.set_auto(True)
        points = [ScanPoint(math.tau * index / 30, 3) for index in range(30)]
        self.assertTrue(navigator.process_scan(points).stopped)
        self.assertEqual(navigator.grid.update_count, 0)

    def test_low_quality_initialization_does_not_start_motion_prematurely(self):
        _, points = room_fixture()
        points = [ScanPoint(p.angle_rad, p.distance_m, 0.3) for p in points]
        navigator = NavigationEngine()
        navigator.set_auto(True)
        for _ in range(3):
            self.assertTrue(navigator.process_scan(points).stopped)
        self.assertFalse(navigator._map_initialized)
        self.assertEqual(navigator.state, "建图初始化")

    def test_search_window_expands_after_strafe_and_failure(self):
        grid, points = room_fixture()
        navigator = NavigationEngine(grid)
        navigator.set_auto(True)
        with patch.object(navigator.matcher, "match", return_value=(Pose2D(), 0.1)) as match:
            navigator.process_scan(points)
            first = match.call_args.kwargs["translation_window_scale"]
            from navigation_core import VelocityCommand
            navigator.predict_motion(VelocityCommand(right_mps=0.2, duration_s=1))
            navigator.process_scan(points)
            second = match.call_args.kwargs["translation_window_scale"]
        self.assertGreater(second, first)
        self.assertLessEqual(second, 1.5)

    def test_search_window_expands_for_diagonal_motion_uncertainty(self):
        grid, points = room_fixture()
        navigator = NavigationEngine(grid)
        navigator.set_auto(True)
        with patch.object(navigator.matcher, "match", return_value=(Pose2D(), 0.1)) as match:
            navigator.process_scan(points)
            first = match.call_args.kwargs["translation_window_scale"]
            from navigation_core import VelocityCommand
            navigator.predict_motion(VelocityCommand(forward_mps=0.08, right_mps=0.08, duration_s=0.5))
            navigator.process_scan(points)
            second = match.call_args.kwargs["translation_window_scale"]
        self.assertGreater(second, first)
        self.assertLessEqual(second, 1.5)

    def test_execution_uncertainties_expand_only_their_matching_axis(self):
        def scales(uncertainty_m, uncertainty_rad):
            grid, points = room_fixture()
            navigator = NavigationEngine(grid)
            navigator.set_auto(True)
            navigator.match_score = 0.95
            navigator.apply_execution_delta(
                0.0,
                0.02,
                yaw_rad=0.01,
                uncertainty_m=uncertainty_m,
                uncertainty_rad=uncertainty_rad,
            )

            def accept(_grid, predicted, _points, **_kwargs):
                return predicted, 0.9

            with patch.object(navigator.matcher, "match", side_effect=accept) as match:
                navigator.process_scan(points)
            return (
                match.call_args.kwargs["translation_window_scale"],
                match.call_args.kwargs["rotation_window_scale"],
            )

        baseline_translation, baseline_rotation = scales(0.0, 0.0)
        uncertain_translation, unchanged_rotation = scales(0.04, 0.0)
        unchanged_translation, uncertain_rotation = scales(0.0, 0.04)
        self.assertGreater(uncertain_translation, baseline_translation)
        self.assertAlmostEqual(unchanged_rotation, baseline_rotation)
        self.assertAlmostEqual(unchanged_translation, baseline_translation)
        self.assertGreater(uncertain_rotation, baseline_rotation)


if __name__ == "__main__":
    unittest.main()
