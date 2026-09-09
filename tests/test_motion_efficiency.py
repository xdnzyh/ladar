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
        self.assertAlmostEqual(command.forward_mps * command.duration_s, 0.24)
        self.assertEqual(command.forward_mps, 0.14)

    def test_near_diagonal_path_uses_conservative_45_degree_motion(self):
        navigator = self.navigator()
        start = navigator.grid.world_to_cell(0, 0)
        navigator.path_cells = [start, (start[0] + 4, start[1] - 4)]
        command = navigator._command_along_path()
        self.assertEqual(command.yaw_rps, 0.0)
        self.assertNotEqual(command.forward_mps, 0.0)
        self.assertNotEqual(command.right_mps, 0.0)
        self.assertAlmostEqual(abs(command.forward_mps), abs(command.right_mps), places=7)
        self.assertLessEqual(
            (command.forward_mps ** 2 + command.right_mps ** 2) ** 0.5 * command.duration_s,
            navigator.MAX_DIAGONAL_SEGMENT_M,
        )

    def test_non_diagonal_path_keeps_cardinal_priority(self):
        navigator = self.navigator()
        start = navigator.grid.world_to_cell(0, 0)
        navigator.path_cells = [start, (start[0] + 2, start[1] - 6)]
        command = navigator._command_along_path()
        self.assertTrue(command.forward_mps == 0.0 or command.right_mps == 0.0)

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
        self.assertTrue(navigator._command_along_path().stopped)

    def test_fresh_obstacle_stops_motion(self):
        navigator = self.navigator()
        navigator.latest_scan = [ScanPoint(0, 0.24)]
        self.assertTrue(navigator._command_along_path().stopped)

    def test_turn_penalty_prefers_one_long_run_before_the_bend(self):
        grid = OccupancyGrid(20, 20, 0.05)
        grid.log_odds[:] = [-6.0] * len(grid.log_odds)
        path = grid.astar(
            (3, 3),
            (10, 6),
            0.0,
            blocked=set(),
            turn_penalty=0.75,
            initial_step=(1, 0),
        )
        directions = [
            (later[0] - earlier[0], later[1] - earlier[1])
            for earlier, later in zip(path, path[1:])
        ]
        direction_changes = sum(
            later != earlier for earlier, later in zip(directions, directions[1:])
        )
        self.assertEqual(directions[:4], [(1, 0)] * 4)
        self.assertEqual(direction_changes, 1)

    def test_first_straight_run_stops_before_a_planned_corner(self):
        navigator = self.navigator()
        start = navigator.grid.world_to_cell(0, 0)
        navigator.path_cells = [
            start,
            (start[0], start[1] - 1),
            (start[0], start[1] - 2),
            (start[0] + 1, start[1] - 2),
            (start[0] + 2, start[1] - 2),
            (start[0] + 3, start[1] - 2),
        ]
        command = navigator._command_along_path()
        self.assertGreater(command.forward_mps, 0.0)
        self.assertEqual(command.right_mps, 0.0)
        self.assertLessEqual(
            command.forward_mps * command.duration_s,
            2 * navigator.grid.resolution_m + 1e-9,
        )

    def test_heading_aware_path_still_avoids_blocked_cells(self):
        grid = OccupancyGrid(20, 20, 0.05)
        grid.log_odds[:] = [-6.0] * len(grid.log_odds)
        blocked = {(6, 3), (6, 4)}
        path = grid.astar(
            (3, 3),
            (10, 3),
            0.0,
            blocked=blocked,
            turn_penalty=0.75,
            initial_step=(1, 0),
        )
        self.assertTrue(path)
        self.assertTrue(blocked.isdisjoint(path))


if __name__ == "__main__":
    unittest.main()
