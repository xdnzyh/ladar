import math
import unittest

from mapping_policy import prepare_mapping_points, process_radar_debug_scan
from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint
from scan_acquisition import scan_complete
from scan_geometry import line_supported_indices


def wall_points(y=0.45, x0=-0.28, x1=0.28, count=13):
    result = []
    for index in range(count):
        x = x0 + (x1 - x0) * index / (count - 1)
        distance = math.hypot(x, y)
        angle = math.atan2(x, y) % math.tau
        result.append(ScanPoint(angle, distance, 1.0, True))
    return result


def scattered_points():
    return [
        ScanPoint(math.radians(105), 0.72, 1.0, True),
        ScanPoint(math.radians(125), 0.58, 1.0, True),
        ScanPoint(math.radians(150), 0.81, 1.0, True),
        ScanPoint(math.radians(198), 0.69, 1.0, True),
    ]


class LineMappingPolicyTests(unittest.TestCase):
    def test_short_straight_wall_is_supported_but_scatter_is_not(self):
        wall = wall_points()
        noise = scattered_points()
        points = wall + noise
        supported = line_supported_indices(points)
        self.assertGreaterEqual(len(supported), len(wall) - 2)
        noise_indexes = set(range(len(wall), len(points)))
        self.assertFalse(supported & noise_indexes)

    def test_mapping_selection_keeps_wall_and_drops_isolated_echoes(self):
        wall = wall_points()
        noise = scattered_points()
        selection = prepare_mapping_points(wall + noise, 1.0, 0.15)
        self.assertGreaterEqual(selection.supported_echoes, len(wall) - 2)
        self.assertGreaterEqual(selection.noise_echoes, len(noise))
        self.assertLess(len(selection.points), len(wall) + len(noise))

    def test_large_angular_gap_is_accepted_when_short_wall_geometry_is_strong(self):
        class PolarLike:
            def __init__(self, point):
                self.angle_rad = point.angle_rad
                self.distance_m = point.distance_m
                self.is_echo = True
            @property
            def x(self):
                return self.distance_m * math.sin(self.angle_rad)
            @property
            def y(self):
                return self.distance_m * math.cos(self.angle_rad)

        points = [PolarLike(point) for point in wall_points()]
        valid, reason = scan_complete(points, {})
        self.assertTrue(valid)
        self.assertIn("短直线", reason)

    def test_radar_only_mode_builds_map_without_issuing_motion(self):
        navigator = NavigationEngine(
            OccupancyGrid(120, 120, 0.02),
            max_range_m=1.0,
            min_range_m=0.15,
        )
        navigator.set_auto(False)
        selection = prepare_mapping_points(wall_points() + scattered_points(), 1.0, 0.15)
        command = process_radar_debug_scan(navigator, selection, 0.15)
        self.assertTrue(command.stopped)
        self.assertFalse(navigator.auto_enabled)
        self.assertEqual(navigator.state, "仅雷达建图")
        self.assertGreater(navigator.grid.update_count, 0)
        self.assertGreater(navigator.local_map_updates, 0)
        self.assertIn("忽略孤立噪声", navigator.detail)

    def test_curved_ring_with_sector_gap_is_not_mistaken_for_wall(self):
        points = [
            ScanPoint(math.tau * index / 40, 1.0, 1.0, True)
            for index in range(40)
            if index not in range(10, 16)
        ]
        self.assertLess(len(line_supported_indices(points)), 8)


if __name__ == "__main__":
    unittest.main()
