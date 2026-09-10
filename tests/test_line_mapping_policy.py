import math
import unittest

from mapping_policy import TwoSweepWallEvidence, prepare_mapping_points, process_radar_debug_scan
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
        evidence = TwoSweepWallEvidence(navigator.grid.resolution_m)
        selection = prepare_mapping_points(wall_points() + scattered_points(), 1.0, 0.15, navigator.grid.resolution_m)
        process_radar_debug_scan(navigator, evidence.update("session", 1, selection), 0.15)
        command = process_radar_debug_scan(navigator, evidence.update("session", 2, selection), 0.15)
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
        evidence = TwoSweepWallEvidence(grid.resolution_m)
        jittered = []
        for scan in range(2):
            offset = 0.006 if scan else -0.006
            jittered.append(wall_points(y=0.45 + offset, count=9))
        for sequence, points in enumerate(jittered, 1):
            selection = prepare_mapping_points(points, 1.0, 0.15, grid.resolution_m)
            process_radar_debug_scan(navigator, evidence.update("session", sequence, selection), 0.15)
        occupied = grid.occupied_cells()
        self.assertGreaterEqual(len(occupied), 8)
        rows = {row for _, row in occupied}
        self.assertLessEqual(max(rows) - min(rows), 1)

    def test_enclosed_room_fits_four_thin_walls_across_zero_angle(self):
        grid = OccupancyGrid(100, 100, 0.02)
        navigator = NavigationEngine(grid, max_range_m=1.2, min_range_m=0.10)
        navigator.set_auto(False)
        evidence = TwoSweepWallEvidence(grid.resolution_m)
        for sequence, phase in enumerate((0.0, math.pi), 1):
            selection = prepare_mapping_points(rectangular_enclosure_points(phase), 1.2, 0.10, grid.resolution_m)
            self.assertEqual(selection.fitted_segments, 4)
            process_radar_debug_scan(navigator, evidence.update("session", sequence, selection), 0.10)
        occupied = grid.occupied_cells()
        self.assertGreaterEqual(len(occupied), 50)
        for col, row in occupied:
            x, y = grid.cell_to_world(col, row)
            distance_to_wall = min(abs(y - 0.55), abs(y + 0.42), abs(x - 0.36), abs(x + 0.38))
            self.assertLessEqual(distance_to_wall, 0.035)

    def test_noisy_opening_is_not_closed_by_fitted_wall(self):
        grid = OccupancyGrid(100, 100, 0.02)
        navigator = NavigationEngine(grid, max_range_m=1.0, min_range_m=0.10)
        navigator.set_auto(False)
        evidence = TwoSweepWallEvidence(grid.resolution_m)
        for sequence, phase in enumerate((0.0, math.pi), 1):
            selection = prepare_mapping_points(open_room_with_noisy_doorway_points(phase), 1.0, 0.10, grid.resolution_m)
            self.assertLessEqual(selection.fitted_segments, 4)
            self.assertGreaterEqual(selection.noise_echoes, 6)
            process_radar_debug_scan(navigator, evidence.update("session", sequence, selection), 0.10)
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

    def test_fitted_points_keep_measurement_uncertainty_and_provenance(self):
        points = [
            ScanPoint(
                point.angle_rad,
                point.distance_m,
                1.0,
                True,
                timestamp_s=1.0 + index * 0.01,
                time_error_s=0.02,
                angle_error_rad=math.radians(15),
                distance_error_m=0.003,
                pixel=900 + index,
                calibration_version="cal-v3",
                source="range",
                session="session-a",
            )
            for index, point in enumerate(wall_points(y=0.30, count=21))
        ]

        selection = prepare_mapping_points(points, 1.0, 0.15, 0.02)

        self.assertTrue(selection.points)
        self.assertTrue(all(point.timestamp_s is not None for point in selection.points))
        self.assertTrue(all(point.angle_error_rad == math.radians(15) for point in selection.points))
        self.assertTrue(all(point.distance_error_m == 0.003 for point in selection.points))
        self.assertTrue(all(point.pixel is not None for point in selection.points))
        self.assertTrue(all(point.calibration_version == "cal-v3" for point in selection.points))
        self.assertTrue(all(point.source == "thin_wall:range" for point in selection.points))
        self.assertTrue(all(point.session == "session-a" for point in selection.points))
        self.assertLess(max(point.evidence_weight(0.02) for point in selection.points), 0.2)

    def test_isolated_points_have_no_mapping_fallback(self):
        selection = prepare_mapping_points(scattered_points(), 1.0, 0.15, 0.02)

        self.assertEqual(selection.supported_echoes, 0)
        self.assertEqual(selection.fitted_segments, 0)
        self.assertEqual(selection.points, ())

        navigator = NavigationEngine(OccupancyGrid(80, 80, 0.02), max_range_m=1.0)
        for _ in range(12):
            navigator.process_local_scan(selection.points, min_range_m=0.15)
        self.assertEqual(navigator.grid.occupied_cells(), [])

    def test_two_consecutive_sweeps_are_required_for_wall_mapping(self):
        evidence = TwoSweepWallEvidence(0.02)
        first = prepare_mapping_points(wall_points(y=0.45), 1.0, 0.15, 0.02)
        second = prepare_mapping_points(wall_points(y=0.454), 1.0, 0.15, 0.02)

        waiting = evidence.update("session-a", 7, first)
        stable = evidence.update("session-a", 8, second)

        self.assertEqual(waiting.points, ())
        self.assertEqual(waiting.mapping_passes, 0)
        self.assertEqual(stable.confirmed_scans, 2)
        self.assertEqual(stable.mapping_passes, 2)
        self.assertGreaterEqual(stable.supported_echoes, 4)
        self.assertTrue(all(point.source.startswith("stable_wall") for point in stable.points))

    def test_sequence_gap_and_new_session_invalidate_two_sweep_evidence(self):
        evidence = TwoSweepWallEvidence(0.02)
        selection = prepare_mapping_points(wall_points(), 1.0, 0.15, 0.02)
        evidence.update("session-a", 1, selection)
        evidence.update("session-a", 2, selection)

        gap = evidence.update("session-a", 4, selection)
        resumed = evidence.update("session-a", 5, selection)
        new_session = evidence.update("session-b", 1, selection)

        self.assertTrue(gap.reset_required)
        self.assertEqual(gap.points, ())
        self.assertEqual(resumed.confirmed_scans, 2)
        self.assertTrue(new_session.reset_required)
        self.assertEqual(new_session.points, ())

    def test_twenty_centimeter_collinear_opening_is_not_bridged(self):
        left = wall_points(y=0.35, x0=-0.35, x1=-0.10, count=9)
        right = wall_points(y=0.35, x0=0.10, x1=0.35, count=9)

        selection = prepare_mapping_points(left + right, 1.0, 0.15, 0.02)

        self.assertEqual(selection.fitted_segments, 2)
        opening_points = [point for point in selection.points if abs(point.x) < 0.08]
        self.assertEqual(opening_points, [])

    def test_two_uncertain_sweeps_confirm_wall_without_claiming_free_space(self):
        def uncertain(scan_time, offset):
            return [
                ScanPoint(
                    point.angle_rad,
                    point.distance_m,
                    1.0,
                    True,
                    timestamp_s=scan_time + index * 0.01,
                    time_error_s=0.02,
                    angle_error_rad=math.radians(15),
                    distance_error_m=0.003,
                    pixel=900 + index,
                    calibration_version="cal-v3",
                    source="range",
                    session="session-a",
                )
                for index, point in enumerate(wall_points(y=0.40 + offset, count=21))
            ]

        evidence = TwoSweepWallEvidence(0.02)
        first = prepare_mapping_points(uncertain(1.0, -0.002), 1.0, 0.15, 0.02)
        second = prepare_mapping_points(uncertain(2.0, 0.002), 1.0, 0.15, 0.02)
        evidence.update("session-a", 1, first)
        stable = evidence.update("session-a", 2, second)
        navigator = NavigationEngine(OccupancyGrid(80, 80, 0.02), max_range_m=1.0)

        process_radar_debug_scan(navigator, stable, 0.15)

        self.assertGreater(len(navigator.grid.occupied_cells()), 8)
        self.assertEqual(len(stable.mapping_layers), 2)
        self.assertLess(max(point.timestamp_s for point in stable.mapping_layers[0]), 2.0)
        self.assertGreaterEqual(min(point.timestamp_s for point in stable.mapping_layers[1]), 2.0)
        free_cells = [
            (col, row)
            for row in range(navigator.grid.height)
            for col in range(navigator.grid.width)
            if navigator.grid.state(col, row) == navigator.grid.FREE
        ]
        self.assertEqual(free_cells, [])

    def test_unconfirmed_current_window_preserves_previous_fixed_pose_map(self):
        evidence = TwoSweepWallEvidence(0.02)
        wall = prepare_mapping_points(wall_points(), 1.0, 0.15, 0.02)
        process_radar_debug_scan(navigator := NavigationEngine(
            OccupancyGrid(80, 80, 0.02), max_range_m=1.0,
        ), evidence.update("session-a", 1, wall), 0.15)
        process_radar_debug_scan(navigator, evidence.update("session-a", 2, wall), 0.15)
        previous = set(navigator.grid.occupied_cells())
        self.assertTrue(previous)

        sparse = prepare_mapping_points(scattered_points(), 1.0, 0.15, 0.02)
        process_radar_debug_scan(navigator, evidence.update("session-a", 3, sparse), 0.15)

        self.assertEqual(set(navigator.grid.occupied_cells()), previous)
        self.assertEqual(navigator.latest_scan, [])

    def test_confirmed_new_wall_only_adds_to_fixed_pose_map(self):
        evidence = TwoSweepWallEvidence(0.02)
        navigator = NavigationEngine(OccupancyGrid(80, 80, 0.02), max_range_m=1.0)
        far = prepare_mapping_points(wall_points(y=0.45), 1.0, 0.15, 0.02)
        process_radar_debug_scan(navigator, evidence.update("session-a", 1, far), 0.15)
        process_radar_debug_scan(navigator, evidence.update("session-a", 2, far), 0.15)
        first_map = set(navigator.grid.occupied_cells())

        near = prepare_mapping_points(wall_points(y=0.33), 1.0, 0.15, 0.02)
        process_radar_debug_scan(navigator, evidence.update("session-a", 3, near), 0.15)
        process_radar_debug_scan(navigator, evidence.update("session-a", 4, near), 0.15)
        final_map = set(navigator.grid.occupied_cells())

        self.assertTrue(first_map < final_map)


if __name__ == "__main__":
    unittest.main()
