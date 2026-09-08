import math
import queue
import unittest
from unittest.mock import Mock

from motion_safety import MotionSafetyGuard
from navigation_app import NavigationApp
from navigation_core import VelocityCommand
from radar_core import CalibrationModel
from scan_acquisition import DistanceObservationReceiver, HardwareObservation, ReceivedObservation
from synchronized_acquisition import ClockEstimate, SynchronizedAcquisition


class Endpoint:
    def __init__(self):
        self.messages = []

    def write_line(self, line, on_sent=None):
        self.messages.append(line)
        if on_sent:
            on_sent(10.0)
        return True


class SharedAcquisitionTests(unittest.TestCase):
    def replay(self, uncertainty=0.0, move=False):
        config = {"arbitrary_phase_scans": True, "clock_drift_bound_ppm": 0,
                  "irq_timestamp_uncertainty_ms": 0, "sync_max_age_s": 60}
        output = []
        acquisition = SynchronizedAcquisition(Endpoint(), Endpoint(), CalibrationModel(p0=700, k=100),
                                              config, lambda *event: output.append(event))
        acquisition.start(0)
        acquisition.state = "running"
        acquisition.last_arrival = {source: 0 for source in acquisition.endpoints}
        acquisition.clocks = {"measurement": ClockEstimate(100, uncertainty, 0, 0),
                              "rotation": ClockEstimate(250, uncertainty, 0, 0)}
        receiver = DistanceObservationReceiver(config)
        packets = []
        for sequence in range(1, 9):
            stamp = sequence * 1.5
            packets.append((stamp + 0.08, "rotation", sequence, stamp))
        for sequence in range(1, 241):
            stamp = sequence * 0.05 + 0.025
            packets.append((stamp + (0.15 if sequence % 7 == 0 else 0.02), "measurement", sequence, stamp))
        expected = []
        moved = False
        for arrival, source, sequence, stamp in sorted(packets):
            if move and not moved and arrival >= 6.2:
                receiver.reset(7.1)
                acquisition.begin_after(7.1)
                moved = True
            if source == "rotation":
                line = f"TRIG {acquisition.session} {sequence} {round((stamp+250)*1e6)}"
            else:
                line = f"PIX {acquisition.session} {sequence} {round((stamp+100)*1e6)} {round((stamp+100)*1e6)} 800"
            packet = HardwareObservation("rotation" if source == "rotation" else "range", sequence,
                                         stamp, None if source == "rotation" else 1.0,
                                         uncertainty=uncertainty, pixel=0 if source == "rotation" else 800)
            for _ in range(2):
                acquisition.feed(source, (line + "\n").encode(), arrival)
                receiver.feed(ReceivedObservation(packet, arrival))
            acquisition.poll(arrival)
            expected.extend(receiver.poll(arrival))
        acquisition.poll(12.6)
        expected.extend(receiver.poll(12.6))
        actual = [(value[1], value[2], value[3]) for kind, value, stamp in output if kind == "sync_sweep"]
        return actual, expected, acquisition

    def test_serial_and_simulation_observations_produce_same_sweeps(self):
        actual, expected, acquisition = self.replay()
        self.assertGreater(len(actual), 2)
        self.assertEqual(len(actual), len(expected))
        for (seq, points, period), (expected_seq, expected_points, expected_period) in zip(actual, expected):
            self.assertEqual(seq, expected_seq)
            self.assertAlmostEqual(period, expected_period)
            self.assertEqual(len(points), len(expected_points))
            for point, other in zip(points, expected_points):
                self.assertAlmostEqual(point.angle_rad, other.angle_rad)
                self.assertAlmostEqual(point.timestamp, other.timestamp)
                self.assertEqual(point.distance_m, other.distance_m)
        self.assertGreater(acquisition.receiver.duplicates, 0)

    def test_motion_window_excludes_moving_observations_and_keeps_phase(self):
        actual, expected, acquisition = self.replay(move=True)
        self.assertTrue(actual)
        self.assertEqual(len(actual), len(expected))
        for _, points, _ in actual:
            self.assertTrue(points[-1].timestamp < 6.2 or points[0].timestamp >= 7.1)
        resumed = next(points for _, points, _ in actual if points[0].timestamp >= 7.1)
        self.assertGreaterEqual(resumed[0].timestamp, 7.5)
        self.assertLess(resumed[0].timestamp, 7.56)
        self.assertEqual(acquisition.endpoints["rotation"].messages.count("OFF"), 1)

    def test_real_clock_uncertainty_rejects_unreliable_sweeps(self):
        actual, expected, _ = self.replay(uncertainty=0.03)
        self.assertEqual(actual, [])
        self.assertEqual(expected, [])

    def app(self):
        app = object.__new__(NavigationApp)
        app.source = "hardware"
        app.running = True
        app.moving = False
        app.motion_generation = 0
        app.config = {"synchronized_acquisition": True}
        app.motion_safety = MotionSafetyGuard(app.config)
        app.protocol_var = Mock(get=lambda: "MOVE {fl} {fr} {rl} {rr} {duration_ms}")
        app.stop_command_var = Mock(get=lambda: "STOP")
        app.chassis_endpoint = Endpoint()
        app.rotation_endpoint = Endpoint()
        app.sync = Mock(state="running")
        app.events = queue.Queue()
        app.navigator = Mock()
        app.root = Mock()
        return app

    def test_movement_keeps_radar_running_and_waits_for_send_event(self):
        app = self.app()
        command = VelocityCommand(forward_mps=0.1, duration_s=0.4)
        app._execute_navigation_command(command)
        self.assertTrue(app.moving)
        self.assertEqual(app.rotation_endpoint.messages, [])
        app.sync.stop.assert_not_called()
        app.sync.begin_after.assert_called_once_with(math.inf)
        app.navigator.predict_motion.assert_not_called()
        app._handle_event(*app.events.get_nowait())
        app.navigator.predict_motion.assert_called_once_with(command)
        app._finish_hardware_motion(app.motion_generation)
        app._handle_event(*app.events.get_nowait())
        self.assertAlmostEqual(app.scan_collect_after, 10.2)
        app._restart_hardware_scan(app.motion_generation)
        self.assertFalse(app.moving)
        app.sync.start.assert_not_called()

    def test_old_motion_callbacks_cannot_stop_or_resume_new_run(self):
        app = self.app()
        app.moving = True
        app.motion_generation = 2
        app._finish_hardware_motion(1)
        app._restart_hardware_scan(1)
        self.assertEqual(app.chassis_endpoint.messages, [])
        self.assertTrue(app.moving)
        app.root.after.assert_not_called()


if __name__ == "__main__":
    unittest.main()

