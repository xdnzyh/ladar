import math
import unittest

from mapping_policy import prepare_mapping_points, TwoSweepWallEvidence, process_radar_debug_scan
from navigation_core import CorrelativeScanMatcher, NavigationEngine, OccupancyGrid, Pose2D, ScanPoint
from scan_geometry import line_supported_indices, range_tolerance_scale


def wall(distance, jitter=0.0):
    return [ScanPoint(math.atan2(x, y) % math.tau, math.hypot(x, y), is_echo=True)
            for i in range(17)
            for x, y in [(-0.48 + i * 0.06, -distance + jitter * (-1) ** i)]]


class RangeAdaptiveGeometryTests(unittest.TestCase):
    def test_tolerance_is_continuous_monotonic_and_bounded(self):
        scales = [range_tolerance_scale(i / 100) for i in range(1001)]
        self.assertEqual(scales[:51], [1.0] * 51)
        self.assertEqual(scales, sorted(scales))
        self.assertEqual(scales[-1], 3.0)
        self.assertLessEqual(max(b - a for a, b in zip(scales, scales[1:])), 0.020001)

    def test_distant_noisy_wall_survives_preselection(self):
        self.assertGreaterEqual(len(line_supported_indices(wall(0.95, 0.021))), 15)

    def test_noisy_distant_wall_is_visible_after_two_sweeps(self):
        grid = OccupancyGrid(160, 160, 0.02)
        navigator = NavigationEngine(grid, max_range_m=2)
        evidence = TwoSweepWallEvidence(0.02)
        for sequence, offset in enumerate((-0.03, 0.03), 1):
            selection = prepare_mapping_points(wall(0.95 + offset, 0.03), 2, 0.1, 0.02)
            self.assertGreaterEqual(selection.supported_echoes, 15)
            process_radar_debug_scan(navigator, evidence.update('test', sequence, selection), 0.1)
        self.assertGreaterEqual(len(grid.occupied_cells()), 30)

    def test_near_wall_accepts_three_centimetre_range_jitter(self):
        self.assertGreaterEqual(prepare_mapping_points(wall(0.30, 0.03), 2, 0.1, 0.02).supported_echoes, 15)

    def test_sparse_distant_wall_is_kept_without_bridging_opening(self):
        def points(xs):
            return [ScanPoint(math.atan2(x, 1.5), math.hypot(x, 1.5), is_echo=True) for x in xs]
        sparse = points([-0.6 + i * 0.12 for i in range(11)])
        selection = prepare_mapping_points(sparse, 3, 0.1, 0.02)
        self.assertEqual(selection.supported_echoes, 11)
        opening = points([sign * (0.10 + i * 0.06) for sign in (-1, 1) for i in range(8)])
        selection = prepare_mapping_points(opening, 3, 0.1, 0.02)
        self.assertEqual(selection.fitted_segments, 2)
        self.assertFalse(any(abs(p.x) < 0.08 for p in selection.points))

    def test_matching_tolerates_more_scatter_at_long_range(self):
        scores = []
        for distance in (0.35, 1.5):
            grid = OccupancyGrid(240, 240, 0.02)
            points = []
            for i in range(24):
                angle = i * math.tau / 24
                grid._add(*grid.world_to_cell(distance * math.sin(angle), distance * math.cos(angle)), 10)
                points.append(ScanPoint(angle, distance + 0.08, is_echo=True))
            result = CorrelativeScanMatcher(0, 0).match(grid, Pose2D(), points)
            scores.append(result.data_score)
        self.assertGreater(scores[1], 0.55)
        self.assertGreater(scores[1], scores[0] + 0.15)


if __name__ == '__main__':
    unittest.main()
