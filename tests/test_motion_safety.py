import math
import unittest
from dataclasses import replace
from unittest.mock import Mock
import queue

from motion_safety import MotionSafetyGuard
from navigation_core import VelocityCommand
from navigation_app import NavigationApp
from scan_acquisition import HardwareObservation


class MotionSafetyTests(unittest.TestCase):
    def guard(self, **command):
        guard = MotionSafetyGuard({})
        guard.start(VelocityCommand(duration_s=1, **command), 10)
        return guard

    def point(self, distance=0.2, stamp=10.1):
        return HardwareObservation("range", 1, stamp, distance)

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
