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


def rectangular_enclosure_points(jitter_phase=0.0):
    walls = {"top": 0.55, "bottom": -0.42, "right": 0.36, "left": -0.38}
    result = []
    for degrees in range(0, 360, 7):
        angle = math.radians(degrees)
        direction_x = math.sin(angle)
        direction_y = math.cos(angle)
        candidates = []
        if abs(direction_y) > 1e-9:
            for y in (walls["top"], walls["bottom"]):
                distance = y / direction_y
                if distance > 0:
                    candidates.append(distance)
        if abs(direction_x) > 1e-9:
            for x in (walls["right"], walls["left"]):
                distance = x / direction_x
                if distance > 0:
                    candidates.append(distance)
        distance = min(candidates) + 0.004 * math.sin(3.0 * angle + jitter_phase)
        result.append(ScanPoint(angle, distance, 1.0, True))
    return result


def open_room_with_noisy_doorway_points(jitter_phase=0.0):
    def segment(x0, y0, x1, y1, count, phase):
        result = []
        for index in range(count):
            fraction = index / (count - 1)
            x = x0 + (x1 - x0) * fraction
            y = y0 + (y1 - y0) * fraction
            jitter = 0.004 * math.sin(index * 1.7 + phase)
            x += 0.5 * jitter
            y += jitter
            result.append(ScanPoint(math.atan2(x, y) % math.tau, math.hypot(x, y), 1.0, True))
        return result

    points = []
    points += segment(-0.32, 0.36, 0.10, 0.36, 14, jitter_phase)
    points += segment(-0.36, 0.32, -0.36, -0.12, 13, jitter_phase + 1.0)
    points += segment(-0.32, -0.22, 0.08, -0.22, 12, jitter_phase + 2.0)
    points += segment(0.12, -0.18, 0.22, -0.08, 5, jitter_phase + 3.0)
    doorway_noise = [
        (0.22, 0.03),
        (0.28, 0.08),
        (0.38, 0.16),
        (0.46, 0.23),
        (0.32, -0.02),
        (0.50, -0.15),
        (0.18, 0.18),
        (0.42, -0.05),
    ]
    for index, (x, y) in enumerate(doorway_noise):
        x += 0.006 * math.sin(jitter_phase + index)
        y += 0.006 * math.cos(jitter_phase + index * 0.5)
        points.append(ScanPoint(math.atan2(x, y) % math.tau, math.hypot(x, y), 1.0, True))
    return points


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
        selection = prepare_mapping_points(wall + noise, 1.0, 0.15, 0.02)
        self.assertGreaterEqual(selection.supported_echoes, len(wall) - 2)
        self.assertGreaterEqual(selection.noise_echoes, len(noise))
        self.assertGreaterEqual(selection.fitted_segments, 1)
        self.assertGreater(len(selection.points), len(wall))

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
        selection = prepare_mapping_points(wall_points() + scattered_points(), 1.0, 0.15, navigator.grid.resolution_m)
        command = process_radar_debug_scan(navigator, selection, 0.15)
        self.assertTrue(command.stopped)
        self.assertFalse(navigator.auto_enabled)
        self.assertEqual(navigator.state, "仅雷达建图")
        self.assertGreater(navigator.grid.update_count, 0)
        self.assertGreater(navigator.local_map_updates, 0)
        self.assertIn("忽略孤立噪声", navigator.detail)

    def test_fitted_wall_becomes_visible_after_two_scans_without_thickening(self):
        grid = OccupancyGrid(80, 80, 0.02)
        navigator = NavigationEngine(grid, max_range_m=1.0, min_range_m=0.15)
        navigator.set_auto(False)
        jittered = []
        for scan in range(2):
            offset = 0.006 if scan else -0.006
            jittered.append(wall_points(y=0.45 + offset, count=9))
        for points in jittered:
            selection = prepare_mapping_points(points, 1.0, 0.15, grid.resolution_m)
            process_radar_debug_scan(navigator, selection, 0.15)
        occupied = grid.occupied_cells()
        self.assertGreaterEqual(len(occupied), 8)
        rows = {row for _, row in occupied}
        self.assertLessEqual(max(rows) - min(rows), 1)

    def test_enclosed_room_fits_four_thin_walls_across_zero_angle(self):
        grid = OccupancyGrid(100, 100, 0.02)
        navigator = NavigationEngine(grid, max_range_m=1.2, min_range_m=0.10)
        navigator.set_auto(False)
        for phase in (0.0, math.pi):
            selection = prepare_mapping_points(rectangular_enclosure_points(phase), 1.2, 0.10, grid.resolution_m)
            self.assertEqual(selection.fitted_segments, 4)
            process_radar_debug_scan(navigator, selection, 0.10)
        occupied = grid.occupied_cells()
        self.assertGreaterEqual(len(occupied), 60)
        for col, row in occupied:
            x, y = grid.cell_to_world(col, row)
            distance_to_wall = min(abs(y - 0.55), abs(y + 0.42), abs(x - 0.36), abs(x + 0.38))
            self.assertLessEqual(distance_to_wall, 0.035)

    def test_noisy_opening_is_not_closed_by_fitted_wall(self):
        grid = OccupancyGrid(100, 100, 0.02)
        navigator = NavigationEngine(grid, max_range_m=1.0, min_range_m=0.10)
        navigator.set_auto(False)
        for phase in (0.0, math.pi):
            selection = prepare_mapping_points(open_room_with_noisy_doorway_points(phase), 1.0, 0.10, grid.resolution_m)
            self.assertLessEqual(selection.fitted_segments, 4)
            self.assertGreaterEqual(selection.noise_echoes, 6)
            process_radar_debug_scan(navigator, selection, 0.10)
        occupied_in_opening = []
        for col, row in grid.occupied_cells():
            x, y = grid.cell_to_world(col, row)
            if 0.26 <= x <= 0.52 and -0.03 <= y <= 0.28:
                occupied_in_opening.append((x, y))
        self.assertEqual(occupied_in_opening, [])

    def test_curved_ring_with_sector_gap_is_not_mistaken_for_wall(self):
        points = [
            ScanPoint(math.tau * index / 40, 1.0, 1.0, True)
            for index in range(40)
            if index not in range(10, 16)
        ]
        self.assertLess(len(line_supported_indices(points)), 8)


if __name__ == "__main__":
    unittest.main()
