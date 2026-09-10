import json
import math
from pathlib import Path
import unittest

from navigation_core import CorrelativeScanMatcher, NavigationEngine, OccupancyGrid, Pose2D, ScanPoint, VelocityCommand


class LocalizationAmbiguityTests(unittest.TestCase):
    def fixture(self):
        data = json.loads((Path(__file__).parent / 'fixtures' / 'localization_ambiguity.json').read_text(encoding='utf-8'))
        spec = data['grid']
        grid = OccupancyGrid(spec['width'], spec['height'], spec['resolution_m'])
        for index, value in spec['evidence']:
            grid._add(index % grid.width, index // grid.width, value)
        grid.update_count = 5
        return data, grid, Pose2D(**data['predicted']), [ScanPoint(**p) for p in data['points']]

    def test_neighboring_candidates_in_same_peak_are_not_lost_localization(self):
        data, grid, predicted, points = self.fixture()
        result = CorrelativeScanMatcher().match(grid, predicted, points, **data['kwargs'])
        self.assertGreater(result.data_score, 0.8)
        expected = data['expected_pose']
        self.assertLess(math.hypot(result.corrected_sensor_pose.x - expected['x'],
                                   result.corrected_sensor_pose.y - expected['y']), 0.03)
        self.assertFalse(result.degenerate)

    def test_navigation_accepts_the_valid_correction_instead_of_repeated_rejection(self):
        data, grid, predicted, points = self.fixture()
        navigator = NavigationEngine(grid)
        navigator.pose = predicted
        navigator.set_auto(True)
        match = navigator.matcher.match
        navigator.matcher.match = lambda grid, pose, points, **kwargs: match(grid, pose, points, **data['kwargs'])
        navigator._plan_next_command = lambda: VelocityCommand()
        before = grid.update_count
        for _ in range(3):
            navigator.process_scan(points)
        self.assertEqual(navigator.rejected_scans, 0, navigator.detail)
        self.assertEqual(grid.update_count, before + 3)

    def test_parallel_corridor_still_reports_unconstrained_axis(self):
        grid = OccupancyGrid(180, 180, 0.04)
        points = [ScanPoint(math.atan2(x, y), math.hypot(x, y), is_echo=True)
                  for x in (-0.6, 0.6) for y in (i * 0.04 for i in range(-60, 61))]
        for _ in range(4):
            grid.update_scan(Pose2D(), points, 3)
        result = CorrelativeScanMatcher().match(grid, Pose2D(0, 0.1), points)
        self.assertGreater(result.data_score, 0.75)
        self.assertTrue(result.degenerate)

    def test_insufficient_map_evidence_is_not_accepted(self):
        _, _, predicted, points = self.fixture()
        result = CorrelativeScanMatcher().match(OccupancyGrid(), predicted, points)
        self.assertLess(result.data_score, NavigationEngine.MAP_UPDATE_MIN_CONFIDENCE)

    def test_two_separate_matching_locations_remain_ambiguous(self):
        grid = OccupancyGrid(160, 160, 0.02)
        points = [ScanPoint(i * math.tau / 24, 0.7 + 0.12 * math.sin(i * 1.7), is_echo=True)
                  for i in range(24)]
        for shift in (-0.15, 0.15):
            for point in points:
                grid._add(*grid.world_to_cell(point.x + shift, point.y), 10)
        result = CorrelativeScanMatcher().match(grid, Pose2D(), points)
        self.assertGreater(result.data_score, 0.75)
        self.assertTrue(result.degenerate)
