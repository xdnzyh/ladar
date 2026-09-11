import json
import math
import queue
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from course_model import BoardCourseModel
from mapping_policy import TwoSweepWallEvidence, prepare_mapping_points
from mapping_runtime import MappingRuntime
from navigation_core import NavigationEngine, OccupancyGrid, Pose2D, ScanPoint, VelocityCommand
from runtime_config import build_navigation_engine, resolve_runtime_config
import test_shared_acquisition


def echoes(coordinates):
    return [ScanPoint(math.atan2(x, y), math.hypot(x, y), is_echo=True) for x, y in coordinates]


def start_scan():
    coordinates = [(side, -0.4 + i * 0.05) for side in (-0.825, 0.825) for i in range(19)]
    coordinates += [(-0.7 + i * 0.05, -0.45) for i in range(29)]
    return echoes(coordinates)


class BoardCourseTests(unittest.TestCase):
    def model(self):
        return BoardCourseModel(0.55, 3, 23)

    def test_alignment_requires_two_agreeing_start_scans(self):
        model = self.model()
        model.fuse(Pose2D(), start_scan(), Pose2D(), (0, 0, 0), 1)
        self.assertIsNone(model.left_x)
        model.fuse(Pose2D(), start_scan(), Pose2D(), (0, 0, 0), 1)
        self.assertAlmostEqual(model.left_x, -0.825, delta=0.025)
        self.assertAlmostEqual(model.rear_y, -0.45, delta=0.025)
        self.assertEqual(len(model.parking_centers()), 2)
        self.assertFalse(model.parking_allowed(Pose2D(0, 4), Pose2D()))
        self.assertTrue(model.parking_allowed(Pose2D(0.55, 12), Pose2D()))

    def test_prior_corrects_lateral_drift_but_not_longitudinal_alias(self):
        model = self.model()
        for _ in range(2):
            model.fuse(Pose2D(), start_scan(), Pose2D(), (0, 0, 0), 1)
        pose = Pose2D(0.04, 3.7)
        corrected = model.fuse(pose, start_scan(), Pose2D(), (0, 0, 0), 1)
        self.assertLess(corrected.x, pose.x)
        self.assertLessEqual(abs(corrected.x - pose.x), 0.02)
        self.assertEqual(corrected.y, pose.y)

    def test_single_wall_narrow_box_or_late_scan_cannot_anchor(self):
        cases = (echoes([(-0.825, i * 0.04) for i in range(14)]),
                 echoes([(x, i * 0.04) for x in (-0.275, 0.275) for i in range(14)]))
        for points in cases:
            model = self.model()
            for _ in range(3):
                model.fuse(Pose2D(), points, Pose2D(), (0, 0, 0), 1)
            self.assertIsNone(model.left_x)
        model = self.model()
        for _ in range(3):
            model.fuse(Pose2D(0, 2), start_scan(), Pose2D(), (0, 0, 0), 1)
        self.assertIsNone(model.left_x)

    def test_reset_discards_old_course_alignment(self):
        model = self.model()
        for _ in range(2):
            model.fuse(Pose2D(), start_scan(), Pose2D(), (0, 0, 0), 1)
        model.reset()
        self.assertIsNone(model.left_x)
        self.assertIsNone(model.rear_y)

    def test_mapping_worker_preserves_measured_course_alignment(self):
        engine = NavigationEngine(OccupancyGrid(100, 120, 0.02), max_range_m=1,
                                  course_model=self.model(), forward_only=True)
        engine.set_auto(True)
        output = queue.Queue()
        runtime = MappingRuntime(engine, on_result=output.put)
        try:
            for sequence in range(5):
                runtime.submit('course-start', sequence, start_scan())
                result = output.get(timeout=5)
                self.assertIsNone(result.error)
            self.assertIsNotNone(engine.course_model.left_x, engine.detail)
            self.assertAlmostEqual(engine.course_model.rear_y, -0.45, delta=0.025)
        finally:
            runtime.stop()

    def test_hardware_map_fits_whole_course_without_painting_prior_as_free(self):
        config = json.loads((Path(__file__).resolve().parents[1] / 'navigation_config.json').read_text('utf-8'))
        engine = build_navigation_engine(resolve_runtime_config('hardware', 'navigation', config))
        self.assertTrue(engine.forward_only)
        self.assertAlmostEqual(engine.course_model.length_m, 12.65)
        for x, y in ((-1.65, -1), (1.65, 13)):
            self.assertTrue(engine.grid.in_bounds(*engine.grid.world_to_cell(x, y)))
        self.assertEqual(engine.grid.known_area_m2(), 0)
        sim = build_navigation_engine(resolve_runtime_config('simulation', 'navigation', config))
        self.assertIsNone(sim.course_model)
        self.assertFalse(sim.forward_only)

    def test_four_sparse_wall_returns_survive_two_sweeps(self):
        evidence = TwoSweepWallEvidence(0.02)
        selections = [prepare_mapping_points(echoes([(x, y) for x in (-0.12, -0.04, 0.04, 0.12)]),
                                              1, 0.15, 0.02) for y in (0.45, 0.454)]
        self.assertGreater(selections[0].fitted_segments, 0)
        self.assertFalse(evidence.update('course', 1, selections[0]).points)
        self.assertTrue(evidence.update('course', 2, selections[1]).points)


class ForwardOnlyTests(unittest.TestCase):
    def scene(self):
        grid = OccupancyGrid(70, 70, 0.04)
        grid.log_odds[:] = [-6] * len(grid.log_odds)
        engine = NavigationEngine(grid, max_range_m=1, forward_only=True)
        engine.latest_scan = [ScanPoint(math.radians(i), 1, is_echo=False) for i in range(0, 360, 5)]
        engine.match_score = 1
        engine.set_auto(True)
        return engine

    def test_reverse_is_blocked_even_if_it_increases_distance_from_start(self):
        engine = self.scene()
        engine.pose.y = -0.2
        for forward, right in ((-0.1, 0), (-0.1, 0.1), (-0.1, -0.1)):
            command, reason = engine._translation_guard(VelocityCommand(forward, right, duration_s=0.3))
            self.assertTrue(command.stopped)
            self.assertIn('仅允许', reason)

    def test_rotating_heading_does_not_allow_course_backtracking(self):
        engine = self.scene()
        engine.pose.yaw = math.pi
        self.assertTrue(engine._translation_guard(VelocityCommand(0.1, duration_s=0.3))[0].stopped)

    def test_grid_search_excludes_backward_edges_in_both_algorithms(self):
        engine = self.scene()
        start, goal = (20, 20), (25, 15)
        for penalty in (0, 0.75):
            path = engine.grid.astar(start, goal, 0, turn_penalty=penalty, allowed_steps=engine._planning_steps())
            self.assertTrue(path)
            self.assertTrue(all(b[1] <= a[1] for a, b in zip(path, path[1:])))
            self.assertEqual(engine.grid.astar(start, (20, 21), 0, turn_penalty=penalty,
                                               allowed_steps=engine._planning_steps()), [])

    def test_failed_planning_uses_larger_front_side_gap_and_shows_path(self):
        engine = self.scene()
        engine.grid.frontier_clusters = lambda: []
        engine.latest_scan = [ScanPoint(math.radians(i), 0.4 if i > 180 else 1, is_echo=i > 180)
                              for i in range(0, 360, 5)]
        engine.latest_scan += echoes([(x, 0.24) for x in (-0.2, -0.1, 0, 0.1, 0.2)])
        command = engine._plan_next_command()
        self.assertFalse(command.stopped, engine.detail)
        self.assertGreater(command.right_mps, 0)
        self.assertGreaterEqual(command.forward_mps, -1e-9)
        self.assertLessEqual(engine._command_distance(command), 0.1 + 1e-9)
        self.assertEqual(engine.state, '空隙探索')
        self.assertTrue(engine.path_cells)

    def test_gap_does_not_need_connectivity_to_start_or_enter_unknown(self):
        engine = self.scene()
        engine.start_pose.y = -5
        self.assertFalse(engine._largest_gap_command().stopped)
        engine.grid.clear()
        self.assertTrue(engine._largest_gap_command().stopped)

    def test_recovery_requires_obstacle_event_and_stays_within_five_cm(self):
        engine = self.scene()
        engine.latest_scan += [ScanPoint(0, 0.24)]
        self.assertTrue(engine._recovery_command().stopped)
        engine.recovery_requested = True
        command = engine._recovery_command()
        self.assertFalse(command.stopped, engine.detail)
        self.assertLess(command.forward_mps, 0)
        self.assertLessEqual(engine._command_distance(command), 0.05 + 1e-9)
        self.assertTrue(engine._recovery_command().stopped)

    def test_blocked_plan_without_collision_event_cannot_trigger_reverse(self):
        engine = self.scene()
        engine.latest_scan += [ScanPoint(0, 0.24)]
        for mode, capability in engine.translation_capabilities.items():
            capability['enabled'] = mode in {'W', 'S'}
        start = engine.grid.world_to_cell(0, 0)
        engine.path_cells = [start, (start[0], start[1] - 5)]
        self.assertTrue(engine._command_along_path().stopped)
        self.assertFalse(engine.recovery_requested)

    def test_recovery_does_not_enlarge_minimum_step_or_cross_unknown(self):
        engine = self.scene()
        engine.recovery_requested = True
        engine.latest_scan += [ScanPoint(0, 0.24)]
        for capability in engine.translation_capabilities.values():
            capability['min_m'] = 0.08
        self.assertTrue(engine._recovery_command().stopped)

    def test_at_either_bay_center_does_not_switch_to_other_bay(self):
        engine = self.scene()
        engine.course_model = BoardCourseModel(0.55, 3, 23)
        engine.course_model.left_x = -0.825
        engine.course_model.rear_y = -12.375
        for x in (-0.55, 0.55):
            engine.pose = Pose2D(x, 0)
            self.assertTrue(engine._course_parking_command().stopped)

    def test_three_stationary_parking_confirmations_stop_despite_open_frontiers(self):
        engine = self.scene()
        engine.course_model = BoardCourseModel(0.55, 3, 23)
        engine._terminal_geometry_confirmed = lambda: True
        engine._course_parking_command = Mock()
        for _ in range(3):
            self.assertTrue(engine._plan_next_command().stopped)
        self.assertEqual(engine.state, '泊车完成')
        engine._course_parking_command.assert_not_called()


class HardwareObstacleRecoveryTests(unittest.TestCase):
    def app(self):
        app = test_shared_acquisition.SharedAcquisitionTests().app()
        app.navigator = NavigationEngine()
        app._send_chassis_stop = Mock(return_value=True)
        app._stop_radar_only = Mock()
        app._clear_mapping_tasks = Mock()
        app.stop = Mock()
        app.chassis_controller.pending = SimpleNamespace(action_id=4, source='auto', stop_requested=False)
        app.chassis_adapter.execution_from_report.return_value = SimpleNamespace(
            local_x_m=0, local_y_m=0.04, yaw_rad=0, uncertainty_m=0.03, uncertainty_rad=0, trusted=False)
        return app

    def report(self, reason='EMERGENCY'):
        return SimpleNamespace(mode='W', reason=reason, request_value=100, unit='MM', target_counts=718,
                               brake=40, enc=287, dx=0, dy=287, dr=0, ds=287, wheels=(287,) * 4)

    def test_obstacle_stop_waits_for_matching_report_and_new_scan(self):
        app = self.app()
        app._safety_stop('运动方向出现近距离障碍，紧急停车')
        app.stop.assert_not_called()
        self.assertFalse(app.navigator.recovery_requested)
        action = app.chassis_controller.pending
        action.stop_requested = True
        app._handle_chassis_done(1, action, self.report(), 10)
        self.assertTrue(app.navigator.recovery_requested)
        self.assertTrue(action.obstacle_recovery_confirmed)
        self.assertTrue(app.running)
        app._restart_hardware_scan = Mock()
        app.root.after.call_args.args[1]()
        app.chassis_controller.complete_settle.assert_called_once_with(resume_auto=True)
        app._restart_hardware_scan.assert_called_once()

    def test_fault_report_or_stale_event_never_auto_resumes(self):
        for reason, stale in (('TIMEOUT', False), ('WRONG_DIRECTION', False), ('EMERGENCY', True)):
            app = self.app()
            app._safety_stop('运动方向出现近距离障碍，紧急停车')
            if stale:
                app.motion_generation += 1
            action = app.chassis_controller.pending
            action.stop_requested = True
            app._handle_chassis_done(1, action, self.report(reason), 10)
            self.assertFalse(app.navigator.recovery_requested)
            self.assertFalse(app.running)
