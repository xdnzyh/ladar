import math
import unittest
import queue
from types import SimpleNamespace
from unittest.mock import Mock

from navigation_app import NavigationApp
from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint
from navigation_core import VelocityCommand
from mapping_runtime import MappingRuntime
from chassis_controller import ChassisMotionAdapter
from rotation_calibration import goal_counts, angle_from_encoder
from scan_acquisition import TimedSweepBuilder


class ResponsivenessTests(unittest.TestCase):
    def scene(self):
        engine = NavigationEngine(OccupancyGrid(100, 100, 0.02), max_range_m=1,
                                  forward_only=True, prefer_forward_exploration=True,
                                  unobserved_clear_range_m=1)
        engine.grid.log_odds[:] = [-6.0] * len(engine.grid.log_odds)
        engine.latest_scan = [ScanPoint(math.radians(d), 1, is_echo=False)
                              for d in range(0, 360, 5)]
        engine.match_score = 0.95
        engine.set_auto(True)
        return engine

    def test_acquisition_progress_does_not_overwrite_navigation_reason(self):
        app = SimpleNamespace(running=True, sync=SimpleNamespace(
            generation=1, state='running', recovery_started_at=None),
            navigator=SimpleNamespace(state='无可执行安全动作', detail='路径被墙挡住'),
            connection_label=Mock(), _log=Mock())
        NavigationApp._handle_event(app, 'sync_status', (1, '扫描中，等待下一真实零位确认'), 1)
        self.assertEqual(app.navigator.state, '无可执行安全动作')

    def test_gap_without_returns_uses_mapped_free_side(self):
        engine = self.scene()
        # A front wall with an observed/free right side, but no side echoes.
        engine.latest_scan = [ScanPoint(math.atan2(i * .02, .24),
                                        math.hypot(i * .02, .24)) for i in range(-12, 13)]
        command = engine._largest_gap_command()
        self.assertFalse(command.stopped)
        self.assertNotEqual(command.right_mps, 0)

    def test_two_failed_matches_can_probe_fresh_local_free_space(self):
        engine = self.scene()
        engine.local_probe_after_two_scans = True
        self.assertTrue(engine._reject_scan(0, '匹配不唯一').stopped)
        command = engine._reject_scan(0, '匹配不唯一')
        self.assertFalse(command.stopped)
        self.assertAlmostEqual(engine._command_distance(command), .10)

    def test_no_scan_still_stops_after_two_failures(self):
        engine = self.scene()
        engine.local_probe_after_two_scans = True
        engine.latest_scan = []
        engine._reject_scan(0, '无回波')
        self.assertTrue(engine._reject_scan(0, '无回波').stopped)

    def test_rotation_forward_and_return_use_fresh_geometry(self):
        engine = self.scene()
        engine.rotation_enabled = True
        engine.translation_capabilities['W']['min_m'] = .08
        for i in range(-30, -14):
            x, y = i * .01, .30
            engine.latest_scan.append(ScanPoint(math.atan2(x, y), math.hypot(x, y)))
            engine.grid._add(*engine.grid.world_to_cell(x, y), 20)
        original = list(engine.latest_scan)
        command = engine._plan_next_command()
        self.assertGreater(command.yaw_rps, 0)
        turn = command.yaw_rps * command.duration_s
        engine.predict_motion(command)
        engine.latest_scan = [ScanPoint(p.angle_rad - turn, p.distance_m, is_echo=p.is_echo) for p in original]
        command = engine._plan_next_command()
        self.assertGreater(command.forward_mps, 0)
        engine.predict_motion(command)
        # The next stationary scan is expressed in the translated sensor frame.
        moved = []
        for p in engine.latest_scan:
            x, y = p.x, p.y - .10
            moved.append(ScanPoint(math.atan2(x, y), math.hypot(x, y), is_echo=p.is_echo))
        engine.latest_scan = moved
        command = engine._plan_next_command()
        self.assertLess(command.yaw_rps, 0)

    def test_rotation_rejects_unknown_and_near_obstacle(self):
        engine = self.scene()
        engine.rotation_enabled = True
        engine.latest_scan.append(ScanPoint(0, .17))
        self.assertTrue(engine._safe_rotation(math.radians(15)).stopped)
        engine.latest_scan.pop()
        engine.grid = OccupancyGrid(100, 100, .02)
        self.assertTrue(engine._safe_rotation(math.radians(15)).stopped)

    def test_detour_chooses_left_for_mirrored_obstacle(self):
        engine = self.scene()
        engine.rotation_enabled = True
        engine.translation_capabilities['W']['min_m'] = .08
        for i in range(15, 31):
            x, y = i * .01, .30
            engine.latest_scan.append(ScanPoint(math.atan2(x, y), math.hypot(x, y)))
            engine.grid._add(*engine.grid.world_to_cell(x, y), 20)
        self.assertLess(engine._plan_next_command().yaw_rps, 0)

    def test_probe_shrinks_before_wall_and_never_writes_global_map(self):
        engine = self.scene()
        engine.local_probe_after_two_scans = True
        # All directions are bounded; 10 cm cannot fit, but a smaller step can.
        engine.latest_scan = [ScanPoint(math.radians(d), .32) for d in range(0, 360, 2)]
        before = list(engine.grid.log_odds)
        engine._reject_scan(0, '匹配不唯一')
        command = engine._reject_scan(0, '匹配不唯一')
        self.assertFalse(command.stopped)
        self.assertLess(engine._command_distance(command), .1)
        self.assertEqual(engine.grid.log_odds, before)

    def test_rotation_wire_compensation_and_encoder_inverse(self):
        from navigation_app import load_configuration
        adapter = ChassisMotionAdapter(load_configuration('hardware', 'navigation'))
        for mode, sign in (('R', 1), ('F', -1)):
            command = VelocityCommand(yaw_rps=sign * math.radians(20), duration_s=1)
            request = adapter.request_for_command(command, automatic=True)
            self.assertEqual((request.mode, request.request_value, request.target_counts), (mode, 355, 355))
            self.assertAlmostEqual(request.target, math.radians(20))
            self.assertAlmostEqual(angle_from_encoder(adapter.rotation_curve, mode, 500), math.radians(20))
            self.assertEqual(goal_counts(adapter.rotation_curve, mode, math.radians(15)), (375, 230))
            estimate = adapter.execution_from_report(SimpleNamespace(mode=mode, enc=500, reason='TARGET'))
            self.assertAlmostEqual(estimate.yaw_rad, sign * math.radians(20))

    def test_configured_scan_accepts_24_good_points(self):
        from navigation_app import load_configuration
        from scan_acquisition import scan_complete
        config = load_configuration('hardware', 'navigation')
        points = [ScanPoint(math.tau * i / 24, .6) for i in range(24)]
        self.assertTrue(scan_complete(points, config)[0])
        self.assertFalse(scan_complete(points[:11], config)[0])

    def test_runtime_open_room_moves_by_second_scan_without_wall_fits(self):
        engine = self.scene()
        engine.grid = OccupancyGrid(100, 100, .02)
        engine.local_probe_after_two_scans = True
        output = queue.Queue()
        runtime = MappingRuntime(engine, on_result=output.put)
        try:
            for sequence in range(2):
                runtime.submit('open', sequence, engine.latest_scan or self.scene().latest_scan)
                result = output.get(timeout=5)
                self.assertIsNone(result.error)
            self.assertFalse(result.command.stopped)
            self.assertTrue(result.snapshot.scan_accepted)
        finally:
            runtime.stop()

    def test_stop_reuses_rotation_timing_and_first_full_stationary_scan(self):
        builder = TimedSweepBuilder({'min_scan_points': 40})
        for count in range(3):
            builder.trigger(count * 1.5, .001, count)
        builder.begin_after(3.2)
        builder.trigger(4.5, .001, 3)
        for i in range(100):
            builder.sample(4.5 + (i + .5) * .015, .001, 800, 1)
        self.assertTrue(builder.trigger(6, .001, 4))

    def test_short_six_centimetre_cluster_is_not_a_wall(self):
        from mapping_policy import prepare_mapping_points
        points = [ScanPoint(math.atan2(x, .45), math.hypot(x, .45))
                  for x in (-.03, -.02, -.01, 0, .01, .02, .03)]
        self.assertEqual(prepare_mapping_points(points, 1, .15).fitted_segments, 0)

    def test_55cm_board_with_range_jitter_is_recognized(self):
        from mapping_policy import prepare_mapping_points
        from navigation_app import load_configuration
        from scan_acquisition import scan_complete
        points = []
        for i in range(31):
            x, y = -.275 + .55 * i / 30, .45
            angle = math.atan2(x, y)
            distance = math.hypot(x, y) + .045 * math.sin(i * 2.1)
            points.append(ScanPoint(angle, distance, is_echo=True))
        selection = prepare_mapping_points(points, 1, .15)
        self.assertGreaterEqual(selection.supported_echoes, 24)
        self.assertTrue(scan_complete(points, load_configuration('hardware', 'navigation'))[0])

    def test_navigation_walls_are_visible_after_two_scans(self):
        engine = NavigationEngine(OccupancyGrid(100, 100, .02), max_range_m=1)
        engine.set_auto(True)
        points = []
        for d in range(0, 360, 4):
            a = math.radians(d)
            distance = min(.5 / max(abs(math.sin(a)), 1e-9), .55 / max(abs(math.cos(a)), 1e-9))
            points.append(ScanPoint(a, distance))
        output = queue.Queue()
        runtime = MappingRuntime(engine, on_result=output.put)
        try:
            for sequence in range(2):
                runtime.submit('room', sequence, points)
                result = output.get(timeout=5)
                self.assertIsNone(result.error)
            self.assertGreater(len(engine.grid.occupied_cells()), 30)
        finally:
            runtime.stop()

    def test_isolated_side_and_back_walls_fit_despite_large_individual_fluctuations(self):
        from mapping_policy import prepare_mapping_points
        from navigation_app import load_configuration
        from scan_acquisition import scan_complete
        points = []
        for side in (True, False):
            for i in range(31):
                jitter = .04 * math.sin(i * 2.1) + (.06 if i == 13 else 0)
                x, y = ((.85 + jitter, -.55 * i / 30) if side
                        else (-.275 + .55 * i / 30, -.82 + jitter))
                points.append(ScanPoint(math.atan2(x, y), math.hypot(x, y), is_echo=True))
        selection = prepare_mapping_points(points, 1.1, .15)
        self.assertGreaterEqual(selection.supported_echoes, 50)
        self.assertLessEqual(selection.fitted_segments, 4)
        self.assertTrue(scan_complete(points, load_configuration('hardware', 'navigation'))[0])

    def test_adjacent_wall_sweeps_tolerate_opposite_four_centimetre_bias(self):
        from mapping_policy import prepare_mapping_points, TwoSweepWallEvidence
        evidence = TwoSweepWallEvidence(.02)
        for sequence, bias in enumerate((-.04, .04)):
            points = [ScanPoint(math.atan2(-.275 + .55 * i / 30, .6 + bias),
                                math.hypot(-.275 + .55 * i / 30, .6 + bias), is_echo=True)
                      for i in range(31)]
            selection = evidence.update('jitter', sequence, prepare_mapping_points(points, 1, .15))
        self.assertEqual(selection.confirmed_scans, 2)
