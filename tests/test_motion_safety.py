import math
import unittest
from dataclasses import replace
from unittest.mock import Mock
import queue
import threading
from types import SimpleNamespace

from motion_safety import MotionSafetyGuard
from navigation_core import VelocityCommand, ScanPoint, Pose2D
from runtime_config import build_navigation_engine, resolve_runtime_config
from navigation_app import NavigationApp
from scan_acquisition import HardwareObservation


class MotionSafetyTests(unittest.TestCase):
    def test_near_obstacle_recovery_chooses_translation_away_after_scan(self):
        config = resolve_runtime_config('simulation', 'navigation', {}, prefer_mode_defaults=True)
        navigator = build_navigation_engine(config)
        points = [ScanPoint(math.radians(degree), 3, is_echo=False) for degree in range(0, 360, 3)]
        points[0] = ScanPoint(0, 0.24, is_echo=True)
        for _ in range(4):
            navigator.grid.update_scan(Pose2D(), points, 3)
        navigator.latest_scan = points
        navigator.recovery_requested = True
        command = navigator._plan_next_command()
        self.assertFalse(command.stopped, navigator.detail)
        self.assertLessEqual(command.forward_mps, 0)
        self.assertEqual(command.yaw_rps, 0)
        self.assertFalse(navigator.recovery_requested)

    def test_recovery_waits_for_known_free_space(self):
        navigator = build_navigation_engine(resolve_runtime_config('simulation', 'navigation', {}, prefer_mode_defaults=True))
        navigator.latest_scan = [ScanPoint(math.radians(degree), 0.24, is_echo=True) for degree in range(0, 360, 3)]
        navigator.recovery_requested = True
        self.assertTrue(navigator._plan_next_command().stopped)
        self.assertTrue(navigator.recovery_requested)
        navigator.reset()
        self.assertFalse(navigator.recovery_requested)

    def test_simulation_obstacle_stop_keeps_scanning_and_corrects_partial_pose(self):
        app = self.simulation_app(10.5)
        app.motion_generation = app.mapping_generation = 0
        app.mapping_lock = threading.RLock()
        app._clear_mapping_tasks = Mock()
        app._log = Mock()
        app.stop = Mock()
        app.simulation.hardware.stop = Mock()
        app.navigator = build_navigation_engine(resolve_runtime_config('simulation', 'navigation', {}, prefer_mode_defaults=True))
        app.navigator.predict_motion(app.motion_safety.command)
        NavigationApp._safety_stop(app, "运动方向出现近距离障碍，紧急停车")
        app.stop.assert_not_called()
        app.simulation.hardware.stop.assert_called_once()
        self.assertTrue(app.running)
        self.assertFalse(app.moving)
        self.assertTrue(app.accept_samples)
        self.assertTrue(app.navigator.recovery_requested)
        self.assertAlmostEqual(app.navigator.pose.y, 0.05)
        self.assertIsNone(app.motion_safety.command)

    def test_planner_rejects_command_that_enters_live_stop_distance(self):
        config = resolve_runtime_config('simulation', 'navigation', {}, prefer_mode_defaults=True)
        navigator = build_navigation_engine(config)
        navigator.latest_scan = [ScanPoint(0, 0.45, is_echo=True)]
        command = VelocityCommand(forward_mps=0.14, duration_s=1.5)
        guard = MotionSafetyGuard(config)
        guard.start(command, 10)
        self.assertIsNotNone(guard.observe(self.point(0.45 - 0.14 * 1.5, 11.5), (0, 0), 11.5))
        self.assertFalse(navigator._command_has_clearance(command))

    def simulation_app(self, simulation_time=11.0):
        app = object.__new__(NavigationApp)
        app._closed = False
        app.source = "simulation"
        app.running = app.moving = True
        app.config = {}
        app.events = queue.Queue()
        app.pending_events = []
        app.event_counter = 0
        app.root = Mock()
        app._handle_mapping_results = Mock()
        app._runtime_fault = Mock()
        app._safety_stop = Mock()
        app.simulation = SimpleNamespace(lock=threading.Lock(), generation=1,
                                         hardware=SimpleNamespace(time=simulation_time))
        app.motion_safety = self.guard(forward_mps=0.1)
        return app

    def test_simulation_backlog_is_consumed_before_blind_timeout(self):
        app = self.simulation_app()
        for index in range(1, 101):
            stamp = 10 + index / 100
            app.events.put(("simulation_observation", (1, self.point(2, stamp), (0, 0), stamp), 0))
        app._poll()
        app._runtime_fault.assert_not_called()
        app._safety_stop.assert_not_called()
        self.assertAlmostEqual(app.motion_safety.last_observation, 11)

    def test_simulation_missing_measurements_still_stops(self):
        app = self.simulation_app()
        app._poll()
        app._runtime_fault.assert_not_called()
        app._safety_stop.assert_called_once_with("运动期间测距更新中断，紧急停车")

    def test_simulation_timeout_uses_captured_clock_not_later_producer_time(self):
        app = self.simulation_app(10.1)
        app.events.put(("simulation_observation", (1, self.point(2), (0, 0), 10.1), 0))
        original = app._handle_event
        def handle(kind, value, stamp):
            original(kind, value, stamp)
            app.simulation.hardware.time = 12
        app._handle_event = handle
        app._poll()
        app._runtime_fault.assert_not_called()
        app._safety_stop.assert_not_called()

    def test_simulation_backlog_keeps_near_obstacle_protection(self):
        app = self.simulation_app(10.2)
        for index in range(1, 11):
            stamp = 10 + index / 100
            distance = 0.2 if index == 10 else 2
            app.events.put(("simulation_observation", (1, self.point(distance, stamp), (0, 0), stamp), 0))
        app._poll()
        app._runtime_fault.assert_not_called()
        app._safety_stop.assert_called_once_with("运动方向出现近距离障碍，紧急停车")

    def test_simulation_completion_is_consumed_before_blind_timeout(self):
        app = self.simulation_app()
        for index in range(10):
            stamp = 10.01 + index / 100
            app.events.put(("simulation_observation", (1, self.point(2, stamp), (0, 0), stamp), 0))
        app.events.put(("simulation_motion_stopped", 1, 0))
        app._poll()
        app._runtime_fault.assert_not_called()
        app._safety_stop.assert_not_called()
        self.assertFalse(app.moving)

    def guard(self, **command):
        guard = MotionSafetyGuard({})
        guard.start(VelocityCommand(duration_s=1, **command), 10)
        return guard

    def point(self, distance=0.2, stamp=10.1):
        return HardwareObservation("range", 1, stamp, distance)

    def test_hardware_speed_bound_does_not_change_motion_direction(self):
        guard = MotionSafetyGuard({
            "runtime_source": "hardware",
            "safety_speed_upper_bound_mps": 0.30,
            "safety_stop_distance_m": 0.04,
        })
        guard.start(VelocityCommand(forward_mps=0.10, duration_s=1), 10)
        self.assertIsNone(guard.observe(self.point(distance=0.30), (math.pi / 2, 0), 10.1))
        self.assertIsNotNone(guard.observe(self.point(distance=0.30), (0, 0), 10.1))

    def test_missing_hardware_stop_distance_fails_closed(self):
        guard = MotionSafetyGuard({
            "runtime_source": "hardware",
            "safety_speed_upper_bound_mps": 0.30,
            "safety_stop_distance_m": None,
        })
        guard.start(VelocityCommand(forward_mps=0.10, duration_s=1), 10)
        self.assertEqual(
            guard.observe(self.point(distance=2.0), (0, 0), 10.1),
            "底盘制动距离未标定，停止自动运动",
        )

    def test_forward_near_obstacle_stops_without_a_complete_sweep(self):
        guard = self.guard(forward_mps=0.14)
        self.assertIsNotNone(guard.observe(self.point(), (0, 0.001), 10.11))

    def test_reverse_and_strafe_use_command_direction(self):
        for command, angle in [({"forward_mps": -0.1}, math.pi), ({"right_mps": 0.1}, math.pi / 2)]:
            guard = self.guard(**command)
            self.assertIsNotNone(guard.observe(self.point(), (angle, 0), 10.11))
            self.assertIsNone(guard.observe(self.point(), ((angle + math.pi) % math.tau, 0), 10.11))

    def test_turning_near_obstacle_and_unknown_angle_fail_safe(self):
        self.assertIsNotNone(self.guard(yaw_rps=0.1).observe(self.point(), (math.pi, 0), 10.11))
        self.assertIsNotNone(self.guard(forward_mps=0.1).observe(self.point(), None, 10.11))

    def test_far_side_and_old_returns_do_not_trigger_false_stop(self):
        guard = self.guard(forward_mps=0.1)
        self.assertIsNone(guard.observe(self.point(distance=2), (0, 0), 10.11))
        self.assertIsNone(guard.observe(self.point(), (math.pi / 2, 0), 10.11))
        self.assertIsNone(guard.observe(self.point(stamp=9.9), (0, 0), 10.11))
        self.assertIsNone(guard.observe(self.point(), (0, 0), 11))

    def test_no_return_and_stale_replay_do_not_reset_blind_timeout(self):
        guard = self.guard(forward_mps=0.1)
        guard.observe(replace(self.point(), status="no_return", distance=None), (0, 0), 10.2)
        guard.observe(self.point(stamp=9.9), (0, 0), 10.3)
        self.assertIsNotNone(guard.poll(10.76))
        guard.clear()
        self.assertIsNone(guard.poll(20))

    def test_live_observation_reaches_stop_without_navigation_processing(self):
        app = object.__new__(NavigationApp)
        app.running = app.moving = True
        app.motion_safety = self.guard(forward_mps=0.1)
        app.sync = Mock(session="session")
        app.sync.receiver.estimate_angle.return_value = (0, 0)
        app.safety_events = queue.Queue()
        app.safety_events.put(("sync_observation", ("session", self.point()), 10.11))
        app._safety_stop = Mock()
        app.navigator = Mock()
        app._check_motion_safety(10.11)
        app._safety_stop.assert_called_once()
        app.navigator.process_scan.assert_not_called()

    def test_safety_stop_keeps_navigation_view(self):
        app = object.__new__(NavigationApp)
        app._send_chassis_stop = Mock()
        app.stop = Mock()
        app.navigator = Mock()
        app._log = Mock()
        app._safety_stop("测距中断")
        app._send_chassis_stop.assert_called_once_with()
        app.stop.assert_called_once_with()
        self.assertEqual(app.navigator.state, "安全停车")
        self.assertEqual(app.navigator.detail, "测距中断")

    def test_settle_delay_is_configurable_and_rejects_invalid_values(self):
        app = object.__new__(NavigationApp)
        for value in (0, 0.2, 0.7):
            app.config = {"hardware_settle_s": value}
            self.assertEqual(app._settle_duration(), value)
        for value in (-1, math.nan, math.inf):
            app.config = {"hardware_settle_s": value}
            with self.assertRaises(ValueError):
                app._settle_duration()


if __name__ == "__main__":
    unittest.main()
