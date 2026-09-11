import math
import queue
import unittest
from unittest.mock import Mock

from mapping_runtime import MappingRuntime
from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint, VelocityCommand
from motion_safety import MotionSafetyGuard
from types import SimpleNamespace


class ExplorationPriorityTests(unittest.TestCase):
    def test_two_scan_action_uses_fresh_scan_when_global_grid_is_unknown(self):
        engine = self.engine()
        engine.grid = OccupancyGrid(120, 120, .02)
        engine.local_probe_after_two_scans = True
        engine._stationary_scan_attempts = 2
        command = engine._plan_next_command()
        self.assertFalse(command.stopped)
        self.assertAlmostEqual(command.forward_mps * command.duration_s, .1)
        self.assertEqual(len(engine.grid.occupied_cells()), 0)

    def test_two_scan_priority_is_disabled_after_manual_stop(self):
        engine = self.engine()
        engine.local_probe_after_two_scans = True
        engine._stationary_scan_attempts = 2
        engine.set_auto(False)
        self.assertFalse(engine._two_scan_due())

    def test_second_scan_preempts_gap_and_parking_planning(self):
        engine = self.engine()
        engine.local_probe_after_two_scans = True
        engine._stationary_scan_attempts = 2
        engine._unexplored_gap_candidates = Mock(side_effect=AssertionError('ordinary planner ran'))
        command = engine._plan_next_command()
        self.assertAlmostEqual(command.forward_mps * command.duration_s, .1)

    def test_blocked_recovery_does_not_skip_other_two_scan_actions(self):
        engine = self.engine()
        engine.recovery_requested = True
        engine._recovery_command = Mock(return_value=VelocityCommand())
        command = engine._two_scan_action()
        self.assertFalse(command.stopped)

    def test_second_scan_rejected_localization_uses_action_budget(self):
        engine = self.engine()
        engine.local_probe_after_two_scans = True
        engine._stationary_scan_attempts = 2
        command = engine._reject_scan(0, 'ambiguous localization')
        self.assertFalse(command.stopped)
        self.assertEqual(engine.state, '两圈短步探索')
        self.assertAlmostEqual(command.forward_mps * command.duration_s, .1)

    def test_worker_exits_stationary_planning_on_second_scan(self):
        engine = self.room(opening=0)
        engine.local_probe_after_two_scans = True
        engine._plan_next_command = Mock(return_value=VelocityCommand())
        engine._fresh_local_probe = Mock(return_value=VelocityCommand())
        output = queue.Queue()
        runtime = MappingRuntime(engine, on_result=output.put)
        try:
            for sequence in (1, 2):
                runtime.submit('budget', sequence, engine.latest_scan)
                result = output.get(timeout=5)
                self.assertIsNone(result.error)
                self.assertEqual(result.command.stopped, sequence == 1)
            self.assertTrue(result.snapshot.scan_accepted)
        finally:
            runtime.stop()

    def test_two_scan_budget_selects_ten_cm_in_forward_half_plane(self):
        engine = self.engine()
        engine.latest_scan.append(ScanPoint(0, .24, is_echo=True))
        command = engine._two_scan_action()
        self.assertFalse(command.stopped)
        self.assertGreaterEqual(command.forward_mps, 0)
        self.assertAlmostEqual(math.hypot(command.forward_mps, command.right_mps) * command.duration_s, .1)

    def test_two_scan_budget_does_not_cross_obstacles(self):
        engine = self.engine()
        engine.latest_scan = [ScanPoint(math.radians(d), .15, is_echo=True) for d in range(0,360,3)]
        self.assertTrue(engine._two_scan_action().stopped)

    def test_gap_width_jitter_keeps_same_turn_target(self):
        engine = self.engine()
        engine._unexplored_gap_candidates = Mock(return_value=[(.7, .8, .5), (.6, -.8, .5)])
        self.assertGreater(engine._plan_next_command().yaw_rps, 0)
        engine._unexplored_gap_candidates = Mock(return_value=[(.72, -.8, .5), (.69, .8, .5)])
        self.assertGreater(engine._plan_next_command().yaw_rps, 0)

    def test_blocked_gap_tries_clear_side_translation(self):
        engine = self.engine()
        engine._unexplored_gap_candidates = Mock(return_value=[(.6, 1., .4)])
        engine._command_to_gap = Mock(return_value=VelocityCommand())
        engine.latest_scan.append(ScanPoint(-math.pi/2, .24, is_echo=True))
        command = engine._plan_next_command()
        self.assertGreater(command.right_mps, 0)
        self.assertEqual(command.yaw_rps, 0)

    def test_recovery_does_not_allow_actual_body_overlap(self):
        engine = self.engine()
        engine.robot_radius_m = .15
        engine.latest_scan.append(ScanPoint(math.radians(310), .19, is_echo=True,
                                            distance_error_m=.05))
        engine.recovery_requested = True
        self.assertTrue(engine._recovery_command().stopped)

    def test_live_guard_allows_escape_but_blocks_approach(self):
        guard = MotionSafetyGuard({'runtime_source': 'hardware', 'chassis_distance_control': True,
                                   'robot_radius_m': .15})
        packet = SimpleNamespace(source='range', status='ok', distance=.214,
                                 uncertainty=0, device_timestamp=10.1)
        guard.start(VelocityCommand(right_mps=.1, duration_s=.5, recovery_translation=True), 10)
        self.assertIsNone(guard.observe(packet, (math.radians(310), .20), 10.1))
        guard.start(VelocityCommand(right_mps=-.1, duration_s=.5, recovery_translation=True), 10)
        self.assertIsNotNone(guard.observe(packet, (math.radians(310), .20), 10.1))

    def test_recovery_can_leave_clearance_buffer_toward_right(self):
        engine = self.engine()
        engine.robot_radius_m = .15
        engine.latest_scan.append(ScanPoint(math.radians(310), .214, is_echo=True,
                                            distance_error_m=.05))
        engine.recovery_requested = True
        command = engine._recovery_command()
        self.assertFalse(command.stopped)
        self.assertGreater(command.right_mps, 0)

    def test_obvious_wall_overrides_old_free_cells_in_one_scan(self):
        from mapping_policy import prepare_mapping_points
        from navigation_core import Pose2D
        raw = [ScanPoint(math.atan2(x, .5), math.hypot(x, .5), is_echo=True,
                         distance_error_m=.05) for x in [-.3+i*.02 for i in range(31)]]
        fitted = prepare_mapping_points(raw, 1, .02, .02).points
        grid = OccupancyGrid(120, 120, .02)
        for p in fitted:
            grid._add(*grid.world_to_cell(p.x, p.y), -20)
        grid.update_scan(Pose2D(), fitted, 1, scan_confidence=.7)
        self.assertGreater(len(grid.occupied_cells()), 10)

    def test_actual_front_opening_accepts_heading_jitter_without_yaw(self):
        for degrees in (-14, 14):
            engine = self.room()
            engine.pose.yaw = -math.pi / 2 + math.radians(degrees)
            engine.latest_scan = [ScanPoint(p.angle_rad - engine.pose.yaw, p.distance_m,
                                           is_echo=True) for p in engine.latest_scan]
            command = engine._plan_next_command()
            self.assertEqual(command.yaw_rps, 0)
            self.assertAlmostEqual(command.forward_mps * command.duration_s, .20)

    def test_local_fallback_recovers_away_instead_of_exploring(self):
        engine = self.engine()
        engine.recovery_requested = True
        engine.latest_scan.append(ScanPoint(0, .24, is_echo=True))
        command = engine._fresh_local_probe()
        self.assertLess(command.forward_mps, 0)
        self.assertEqual(command.yaw_rps, 0)
        self.assertFalse(engine.recovery_requested)

    def test_measured_wall_replaces_accumulated_assumed_free(self):
        from mapping_policy import prepare_mapping_points
        from navigation_core import Pose2D
        raw = [ScanPoint(math.atan2(x, .5), math.hypot(x, .5), is_echo=True)
               for x in [-.3 + i*.02 for i in range(31)]]
        fitted = prepare_mapping_points(raw, 1, .02, .02).points
        grid = OccupancyGrid(120, 120, .02)
        for p in fitted:
            cell = grid.world_to_cell(p.x, p.y)
            grid._add(*cell, -20)
            grid.assumed_free_cells.add(cell)
        for _ in range(2):
            grid.update_scan(Pose2D(), fitted, 1, add_only=True)
        self.assertGreater(len(grid.occupied_cells()), 10)

    def test_front_gap_jitter_advances_twenty_cm_without_turning(self):
        engine = self.engine()
        for degrees in (14, -13, 18, -16, 0):
            command = engine._command_to_gap((.60, math.radians(degrees), .50))
            self.assertEqual(command.yaw_rps, 0)
            self.assertAlmostEqual(command.forward_mps * command.duration_s, .20)

    def test_front_gap_obstacle_still_blocks_twenty_cm_step(self):
        engine = self.engine()
        engine.latest_scan.append(ScanPoint(0, .30, is_echo=True))
        command = engine._command_to_gap((.60, 0, .50))
        self.assertLess(command.forward_mps * command.duration_s, .20)

    def test_fitted_wall_with_range_variation_is_written_promptly(self):
        from mapping_policy import prepare_mapping_points
        from navigation_core import Pose2D
        points = [ScanPoint(math.atan2(x, .50 + .025 * (-1)**i),
                            math.hypot(x, .50 + .025 * (-1)**i),
                            is_echo=True, distance_error_m=.05)
                  for i, x in enumerate([-.30 + .02*i for i in range(31)])]
        selection = prepare_mapping_points(points, 1, .02, .02)
        self.assertGreater(selection.fitted_segments, 0)
        grid = OccupancyGrid(120, 120, .02)
        grid.update_scan(Pose2D(), selection.points, 1)
        grid.update_scan(Pose2D(), selection.points, 1)
        self.assertGreater(len(grid.occupied_cells()), 10)

    def engine(self):
        engine = NavigationEngine(OccupancyGrid(120, 120, .02), max_range_m=1,
                                  forward_only=True, prefer_forward_exploration=True,
                                  rotation_enabled=True, unobserved_clear_range_m=1)
        engine.immediate_navigation = True
        engine.distance_controlled_motion = True
        engine.prioritize_unexplored_gaps = True
        engine.grid.log_odds[:] = [-6.] * len(engine.grid.log_odds)
        engine.latest_scan = [ScanPoint(math.radians(d), 1, is_echo=False) for d in range(0, 360, 2)]
        engine.set_auto(True)
        engine._map_initialized = True
        engine.match_score = .95
        return engine

    def room(self, opening=.55):
        engine = self.engine()
        points = []
        for i in range(41):
            x = -.65 + 1.3 * i / 40
            for y in (-.65, .65):
                points.append(ScanPoint(math.atan2(x, y), math.hypot(x, y), is_echo=True))
            y = x
            points.append(ScanPoint(math.atan2(.65, y), math.hypot(.65, y), is_echo=True))
            if abs(y) >= opening / 2:
                points.append(ScanPoint(math.atan2(-.65, y), math.hypot(-.65, y), is_echo=True))
        engine.latest_scan = points
        for p in points:
            engine.grid._add(*engine.grid.world_to_cell(p.x, p.y), 20)
        # The opening leads to cells that have never been observed.
        for row in range(engine.grid.height):
            for col in range(engine.grid.width):
                x, y = engine.grid.cell_to_world(col, row)
                if x < -.7 and abs(y) < opening / 2:
                    engine.grid._add(col, row, 6)
        return engine

    def test_wide_unexplored_gap_wins_over_parking_and_straight_motion(self):
        engine = self.room()
        engine.course_model = Mock()
        engine._terminal_geometry_confirmed = Mock(return_value=True)
        engine._terminal_evidence_scans = 10
        command = engine._plan_next_command()
        self.assertFalse(command.stopped)
        self.assertLess(command.yaw_rps, 0)
        self.assertNotEqual(engine.state, '泊车完成')

    def test_actual_swept_circle_allows_short_step_before_35cm_wall(self):
        engine = self.engine()
        engine.robot_radius_m = .15
        engine.safety_clearance_m = .12
        engine.safety_max_observation_age_s = .5
        engine.latest_scan.append(ScanPoint(0, .35, is_echo=True))
        self.assertTrue(engine._command_has_clearance(VelocityCommand(forward_mps=.1, duration_s=1)))

    def test_initialized_map_uses_first_post_motion_scan(self):
        engine = self.engine()
        engine.predict_motion(VelocityCommand(forward_mps=.1, duration_s=1))
        output = queue.Queue()
        runtime = MappingRuntime(engine, on_result=output.put)
        try:
            runtime.submit('new-pose', 1, engine.latest_scan)
            result = output.get(timeout=5)
            self.assertIsNone(result.error)
            self.assertFalse(result.command.stopped)
            self.assertTrue(result.snapshot.scan_accepted)
        finally:
            runtime.stop()

    def test_rotation_precedes_side_translation_when_forward_is_blocked(self):
        engine = self.engine()
        engine._forward_exploration_command = Mock(return_value=VelocityCommand())
        turn = VelocityCommand(yaw_rps=.25, duration_s=1)
        engine._start_detour = Mock(return_value=turn)
        engine._course_parking_command = Mock(return_value=VelocityCommand(right_mps=.1, duration_s=1))
        self.assertEqual(engine._plan_next_command(), turn)

    def test_sub_40cm_gap_is_not_top_priority(self):
        engine = self.room(opening=.30)
        self.assertFalse(engine._unexplored_gap_candidates())

    def test_known_space_beyond_gap_is_not_unexplored(self):
        engine = self.room()
        for row in range(engine.grid.height):
            for col in range(engine.grid.width):
                x, y = engine.grid.cell_to_world(col, row)
                if x < -.7:
                    engine.grid._add(col, row, -10)
        self.assertFalse(engine._unexplored_gap_candidates())

    def test_blocked_gap_cannot_be_declared_parking_complete(self):
        engine = self.room()
        engine._safe_rotation = Mock(return_value=VelocityCommand())
        engine._translation_guard = Mock(return_value=(VelocityCommand(), 'blocked'))
        for _ in range(5):
            self.assertTrue(engine._plan_next_command().stopped)
            self.assertEqual(engine.state, '缺口待通行')

    def test_gap_turn_tracks_heading_across_multiple_fresh_scans(self):
        engine = self.room()
        original = list(engine.latest_scan)
        total = 0
        for _ in range(3):
            command = engine._plan_next_command()
            self.assertLess(command.yaw_rps, 0)
            total += command.yaw_rps * command.duration_s
            engine.predict_motion(command)
            engine.latest_scan = [ScanPoint(p.angle_rad - total, p.distance_m, is_echo=True) for p in original]
        command = engine._plan_next_command()
        self.assertEqual(command.yaw_rps, 0)
        self.assertGreater(command.forward_mps, 0)

    def test_live_guard_agrees_with_distance_planner(self):
        config = {'runtime_source': 'hardware', 'chassis_distance_control': True,
                  'robot_radius_m': .15, 'safety_clearance_m': .12}
        guard = MotionSafetyGuard(config)
        command = VelocityCommand(forward_mps=.1, duration_s=1)
        guard.start(command, 10)
        packet = SimpleNamespace(source='range', status='ok', distance=.35,
                                 uncertainty=0, device_timestamp=10.1)
        self.assertIsNone(guard.observe(packet, (0, 0), 10.1))
        packet.distance = .25
        self.assertIsNotNone(guard.observe(packet, (0, 0), 10.1))
        engine = self.engine()
        engine.robot_radius_m = .15
        engine.latest_scan = [ScanPoint(0, .25, is_echo=True)]
        self.assertFalse(engine._command_has_clearance(command))

    def test_config_enables_new_priorities(self):
        from navigation_app import load_configuration
        from runtime_config import build_navigation_engine
        engine = build_navigation_engine(load_configuration('hardware', 'navigation'))
        self.assertTrue(engine.immediate_navigation)
        self.assertTrue(engine.prioritize_unexplored_gaps)
        self.assertTrue(engine.rotation_enabled)
        self.assertTrue(engine.distance_controlled_motion)

    def test_path_to_side_rotates_instead_of_strafing(self):
        engine = self.engine()
        start = engine.grid.world_to_cell(0, 0)
        engine.path_cells = [start, (start[0] + 8, start[1])]
        command = engine._command_along_path()
        self.assertGreater(command.yaw_rps, 0)
        self.assertEqual(command.right_mps, 0)

    def test_real_map_and_pose_update_do_not_restart_two_sweep_wait(self):
        engine = self.room(opening=0)
        raw = list(engine.latest_scan)
        engine.grid = OccupancyGrid(120, 120, .02)
        engine._map_initialized = False
        output = queue.Queue()
        runtime = MappingRuntime(engine, on_result=output.put)
        try:
            for sequence in range(2):
                runtime.submit('room', sequence, raw)
                result = output.get(timeout=5)
                self.assertIsNone(result.error)
            self.assertTrue(engine._map_initialized)
            self.assertGreater(len(engine.grid.occupied_cells()), 20)
            runtime.apply_execution_delta(0, .1)
            points = [ScanPoint(math.atan2(p.x, p.y - .1), math.hypot(p.x, p.y - .1), is_echo=True) for p in raw]
            runtime.submit('room', 2, points)
            result = output.get(timeout=5)
            self.assertIsNone(result.error)
            self.assertTrue(result.snapshot.scan_accepted)
            self.assertFalse(result.command.stopped)
            self.assertNotEqual(result.snapshot.state, '两圈墙面确认')
        finally:
            runtime.stop()
