import math
import unittest

from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint, VelocityCommand


class MotionSegmentTests(unittest.TestCase):
    @staticmethod
    def capabilities(**overrides):
        table = {
            mode: {"enabled": True, "min_m": 0.0, "max_m": 0.20}
            for mode in NavigationEngine.TRANSLATION_MODES
        }
        for mode, values in overrides.items():
            table[mode].update(values)
        return table

    def navigator(self, capabilities=None):
        grid = OccupancyGrid()
        grid.log_odds[:] = [-6.0] * len(grid.log_odds)
        navigator = NavigationEngine(grid, translation_capabilities=capabilities)
        navigator.match_score = 0.95
        start = grid.world_to_cell(0, 0)
        navigator.path_cells = [(start[0], start[1] - index) for index in range(8)]
        navigator.latest_scan = [ScanPoint(math.tau * index / 24, 2) for index in range(24)]
        return navigator

    def test_clear_known_path_allows_longer_segment(self):
        navigator = self.navigator()
        command = navigator._command_along_path()
        self.assertAlmostEqual(command.forward_mps * command.duration_s, 0.20)
        self.assertEqual(command.forward_mps, 0.14)

    def test_direction_cap_limits_normal_path(self):
        navigator = self.navigator(self.capabilities(W={"max_m": 0.12}))
        command = navigator._command_along_path()
        self.assertEqual(navigator._translation_mode(command), "W")
        self.assertAlmostEqual(navigator._command_distance(command), 0.12)

    def test_too_short_direction_step_stops_with_specific_detail(self):
        navigator = self.navigator(self.capabilities(W={"min_m": 0.08}))
        start = navigator.grid.world_to_cell(0, 0)
        navigator.path_cells = [start, (start[0], start[1] - 1)]
        self.assertTrue(navigator._command_along_path().stopped)
        self.assertIn("W 方向目标", navigator.detail)
        self.assertIn("0.080 m", navigator.detail)

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

    def test_unknown_gap_blocks_even_a_short_segment(self):
        navigator = self.navigator()
        cell = navigator.grid.world_to_cell(0, 0.08)
        navigator.grid.log_odds[navigator.grid._index(*cell)] = 0
        self.assertTrue(navigator._command_along_path().stopped)
        self.assertIn("未知空间", navigator.detail)

    def test_inflated_obstacle_prevents_long_segment(self):
        navigator = self.navigator()
        navigator.grid._add(*navigator.grid.world_to_cell(0.12, 0.16), 20)
        self.assertTrue(navigator._command_along_path().stopped)

    def test_fresh_obstacle_stops_motion(self):
        navigator = self.navigator()
        navigator.latest_scan = [ScanPoint(0, 0.24)]
        self.assertTrue(navigator._command_along_path().stopped)

    def test_corridor_probe_does_not_enter_unknown_space(self):
        navigator = NavigationEngine()
        navigator.match_score = 0.95
        navigator.latest_scan = [ScanPoint(math.tau * index / 24, 1.0) for index in range(24)]
        self.assertTrue(navigator._corridor_probe_command().stopped)
        self.assertIn("未知空间", navigator._last_motion_rejection_reason)

    def test_diagonal_recovery_does_not_bypass_unknown_space(self):
        navigator = self.navigator()
        start = navigator.grid.world_to_cell(0, 0)
        navigator.path_cells = [start, (start[0] + 4, start[1] - 4)]
        for cell in (
            navigator.grid.world_to_cell(0.0, 0.04),
            navigator.grid.world_to_cell(0.04, 0.04),
        ):
            navigator.grid.log_odds[navigator.grid._index(*cell)] = 0.0
        self.assertTrue(navigator._command_along_path().stopped)
        self.assertIn("未知空间", navigator.detail)

    def test_parking_search_does_not_enter_unknown_space(self):
        navigator = NavigationEngine()
        navigator.match_score = 0.95
        navigator.latest_scan = [ScanPoint(math.tau * index / 24, 0.5) for index in range(24)]
        self.assertTrue(navigator._parking_search_command().stopped)
        self.assertIn("未知空间", navigator._last_motion_rejection_reason)

    def test_disabled_diagonal_falls_back_without_emitting_that_mode(self):
        navigator = self.navigator(self.capabilities(E={"enabled": False, "max_m": 0.0}))
        start = navigator.grid.world_to_cell(0, 0)
        navigator.path_cells = [start, (start[0] + 4, start[1] - 4)]
        command = navigator._command_along_path()
        self.assertFalse(command.stopped)
        self.assertEqual(navigator._translation_mode(command), "W")

    def test_one_way_guard_rejects_return_toward_start(self):
        navigator = self.navigator()
        navigator.pose.y = 0.20
        command = VelocityCommand(forward_mps=-0.10, duration_s=1.2)
        guarded = navigator._collision_guard(command)
        self.assertTrue(guarded.stopped)
        self.assertIn("单向路线", navigator.detail)

    def test_rotation_modes_are_not_navigation_actions(self):
        navigator = self.navigator()
        guarded = navigator._collision_guard(VelocityCommand(yaw_rps=0.2, duration_s=1.0))
        self.assertTrue(guarded.stopped)
        self.assertIn("未开放旋转", navigator.detail)

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
