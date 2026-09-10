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
