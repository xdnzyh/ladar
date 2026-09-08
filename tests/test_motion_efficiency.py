import unittest

from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint


class MotionSegmentTests(unittest.TestCase):
    def navigator(self):
        grid = OccupancyGrid()
        grid.log_odds[:] = [-6.0] * len(grid.log_odds)
        navigator = NavigationEngine(grid)
        navigator.match_score = 0.95
        start = grid.world_to_cell(0, 0)
        navigator.path_cells = [(start[0], start[1] - index) for index in range(8)]
        navigator.latest_scan = [ScanPoint(0, 2)]
        return navigator

    def test_clear_known_path_allows_longer_segment(self):
        navigator = self.navigator()
        command = navigator._command_along_path()
        self.assertAlmostEqual(command.forward_mps * command.duration_s, 0.16)
        self.assertEqual(command.forward_mps, 0.14)

    def test_low_confidence_retains_short_segment(self):
        navigator = self.navigator()
        navigator.match_score = 0.6
        self.assertAlmostEqual(navigator._command_along_path().duration_s, 0.38)

    def test_unknown_gap_does_not_allow_long_segment(self):
        navigator = self.navigator()
        cell = navigator.grid.world_to_cell(0, 0.08)
        navigator.grid.log_odds[navigator.grid._index(*cell)] = 0
        self.assertAlmostEqual(navigator._command_along_path().duration_s, 0.38)

    def test_inflated_obstacle_prevents_long_segment(self):
        navigator = self.navigator()
        navigator.grid._add(*navigator.grid.world_to_cell(0.12, 0.16), 20)
        self.assertAlmostEqual(navigator._command_along_path().duration_s, 0.38)

    def test_fresh_obstacle_stops_motion(self):
        navigator = self.navigator()
        navigator.latest_scan = [ScanPoint(0, 0.24)]
        self.assertTrue(navigator._command_along_path().stopped)


if __name__ == "__main__":
    unittest.main()
