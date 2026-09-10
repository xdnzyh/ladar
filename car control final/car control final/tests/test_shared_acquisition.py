import math
import queue
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from chassis_controller import ChassisMoveRequest, ChassisState
from mapping_runtime import MappingRequest, MappingResult, MappingSnapshot
from motion_safety import MotionSafetyGuard
from navigation_app import NavigationApp
from navigation_core import OccupancyGrid, Pose2D, VelocityCommand
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
        self.assertGreaterEqual(resumed[0].timestamp, 7.1)
        self.assertLess(resumed[0].timestamp, 7.16)
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
        app.chassis_endpoint = Endpoint()
        app.rotation_endpoint = Endpoint()
        app.sync = Mock(state="running", receiver=Mock(), session="session")
        app.events = queue.Queue()
        app.navigator = Mock()
        app.root = Mock()
        app._log = Mock()
        app.mapping_generation = 0
        app.mapping_tasks = queue.Queue()
        app.mapping_results = queue.Queue()
        app.mapping_runtime = None
        app.mapping_snapshot = None
        app.disconnect_requested = False
        app.mapping_lock = __import__("threading").RLock()
        app._action_commands = {}
        request = ChassisMoveRequest("W", 100, "MM", 718, 0.1, "m")
        action = SimpleNamespace(action_id=1, request=request, mode="W", request_value=100,
                                 unit="MM", stop_requested=False, source="auto")
        controller = Mock(
            in_flight=False,
            pending=None,
            state=ChassisState.IDLE,
            connection_generation=1,
        )

        def request_move(*_args, **_kwargs):
            controller.pending = action
            controller.in_flight = True
            return True

        controller.request_move.side_effect = request_move
        app.chassis_controller = controller
        app.chassis_adapter = Mock(request_for_command=Mock(return_value=request))
        app.manual_motion = False
        return app

    def test_motion_is_registered_before_send_and_pose_waits_for_done(self):
        app = self.app()
        command = VelocityCommand(forward_mps=0.1, duration_s=0.4)
        app._execute_navigation_command(command)
        self.assertTrue(app.moving)
        self.assertEqual(app.rotation_endpoint.messages, [])
        app.sync.stop.assert_not_called()
        app.sync.begin_after.assert_called_once_with(math.inf)
        app.navigator.predict_motion.assert_not_called()
        action = app.chassis_controller.pending
        app._handle_event("chassis_move_sent", (1, action), 10.0)
        self.assertIs(app.motion_safety.command, command)
        app.navigator.predict_motion.assert_not_called()

    def test_old_connection_send_callback_cannot_start_safety(self):
        app = self.app()
        command = VelocityCommand(forward_mps=0.1, duration_s=0.4)
        app._execute_navigation_command(command)
        action = app.chassis_controller.pending
        app._handle_event("chassis_move_sent", (0, action), 10.0)
        self.assertIsNone(app.motion_safety.command)

    def test_sweep_crossing_settle_boundary_is_not_submitted(self):
        app = self.app()
        app.scan_collect_after = 10.0
        app.rotation = SimpleNamespace(period_s=0.0, period_history=[])
        app._handle_sweep = Mock()
        points = [
            SimpleNamespace(timestamp=9.99),
            SimpleNamespace(timestamp=10.01),
        ]

        app._handle_event("sync_sweep", ("session", 8, points, 1.2), 10.02)

        app._handle_sweep.assert_not_called()
        self.assertEqual(app.rotation.period_history, [])

    def test_full_sweep_does_not_unlock_until_mapping_accepts_it(self):
        app = self.app()
        app.running = True
        app.moving = False
        app.mapping_generation = 2
        app.scan_collect_after = 10.0
        app._post_motion_map_revision = 7
        app._post_motion_scan_count = 4
        app.view_mode = Mock(get=lambda: "navigation")
        action = SimpleNamespace(action_id=9)
        app.chassis_controller.pending = action
        app.chassis_controller.state = ChassisState.WAITING_SCAN
        app.chassis_controller.mark_scan_ready.return_value = True
        app._action_commands = {9: VelocityCommand(forward_mps=0.1, duration_s=1)}
        grid = OccupancyGrid()
        request = MappingRequest(2, "session", 8, 10.1, 11.4, "navigation", 3, ())
        snapshot = MappingSnapshot(
            2, 4, 8, "session", 8, "navigation", grid, Pose2D(), (), None,
            VelocityCommand(), "等待规划", "", 5, 0, 0, 0, 0,
        )
        app.mapping_results.put(MappingResult(request, snapshot, VelocityCommand()))
        app._handle_mapping_results()
        app.chassis_controller.mark_scan_ready.assert_called_once()
        self.assertNotIn(9, app._action_commands)

    def test_old_or_rejected_mapping_result_cannot_unlock_motion(self):
        app = self.app()
        app.running = True
        app.moving = False
        app.mapping_generation = 2
        app.scan_collect_after = 10.0
        app._post_motion_map_revision = 7
        app._post_motion_scan_count = 4
        app.view_mode = Mock(get=lambda: "navigation")
        app.chassis_controller.pending = SimpleNamespace(action_id=9)
        app.chassis_controller.state = ChassisState.WAITING_SCAN
        app.chassis_controller.mark_scan_ready.return_value = True
        grid = OccupancyGrid()
        request = MappingRequest(2, "session", 8, 9.9, 11.4, "navigation", 3, ())
        snapshot = MappingSnapshot(
            2, 4, 7, "session", 8, "navigation", grid, Pose2D(), (), None,
            VelocityCommand(), "定位拒绝", "", 4, 0, 1, 0, 0,
        )
        app.mapping_results.put(MappingResult(request, snapshot, VelocityCommand()))
        app._handle_mapping_results()
        app.chassis_controller.mark_scan_ready.assert_not_called()

    def test_waiting_scan_requires_both_boundaries_and_both_map_counters(self):
        cases = (
            ("other", 10.1, 11.4, 8, 5),
            ("session", 10.1, 9.9, 8, 5),
            ("session", 10.1, 11.4, 7, 5),
            ("session", 10.1, 11.4, 8, 4),
        )
        for session, scan_start, scan_end, map_version, completed_scans in cases:
            with self.subTest(
                session=session,
                scan_start=scan_start,
                scan_end=scan_end,
                map_version=map_version,
                completed_scans=completed_scans,
            ):
                app = self.app()
                app.mapping_generation = 2
                app.scan_collect_after = 10.0
                app._post_motion_map_revision = 7
                app._post_motion_scan_count = 4
                app.view_mode = Mock(get=lambda: "navigation")
                app.chassis_controller.pending = SimpleNamespace(action_id=9)
                app.chassis_controller.state = ChassisState.WAITING_SCAN
                app.chassis_controller.mark_scan_ready.return_value = True
                request = MappingRequest(2, session, 8, scan_start, scan_end, "navigation", 3, ())
                snapshot = MappingSnapshot(
                    2, 4, map_version, session, 8, "navigation", OccupancyGrid(), Pose2D(), (), None,
                    VelocityCommand(), "定位检查", "", completed_scans, 0, 0, 0, 0,
                )
                app.mapping_results.put(MappingResult(request, snapshot, VelocityCommand()))

                app._handle_mapping_results()

                app.chassis_controller.mark_scan_ready.assert_not_called()

    def test_target_done_applies_report_prior_once_without_command_prediction(self):
        app = self.app()
        action = app.chassis_controller.pending = SimpleNamespace(
            action_id=3,
            source="auto",
            stop_requested=False,
        )
        app._action_commands[3] = VelocityCommand(forward_mps=0.1, duration_s=1.0)
        app.mapping_snapshot = SimpleNamespace(map_version=12, completed_scans=6)
        estimate = SimpleNamespace(
            local_x_m=0.01,
            local_y_m=0.09,
            yaw_rad=0.02,
            uncertainty_m=0.015,
            uncertainty_rad=0.01,
            trusted=True,
        )
        app.chassis_adapter.execution_from_report.return_value = estimate
        report = SimpleNamespace(
            mode="W", reason="TARGET", request_value=100, unit="MM", target_counts=718,
            brake=44.0, enc=646.0, dx=0.0, dy=646.0, dr=0.0, ds=646.0,
            wheels=(646, 646, 646, 646),
        )

        app._handle_chassis_done(1, action, report, 10.0)

        app.navigator.apply_execution_delta.assert_called_once_with(0.01, 0.09, 0.02, 0.015, 0.01)
        app.navigator.predict_motion.assert_not_called()
        self.assertTrue(app.running)
        self.assertEqual(app._post_motion_map_revision, 12)
        self.assertEqual(app._post_motion_scan_count, 6)

    def test_abnormal_done_applies_report_prior_but_never_resumes_automatic_motion(self):
        app = self.app()
        action = app.chassis_controller.pending = SimpleNamespace(
            action_id=4,
            source="auto",
            stop_requested=False,
        )
        estimate = SimpleNamespace(
            local_x_m=0.0,
            local_y_m=0.04,
            yaw_rad=0.0,
            uncertainty_m=0.03,
            uncertainty_rad=0.0,
            trusted=False,
        )
        app.chassis_adapter.execution_from_report.return_value = estimate
        app._stop_radar_only = Mock()
        report = SimpleNamespace(
            mode="W", reason="EMERGENCY", request_value=100, unit="MM", target_counts=718,
            brake=40.0, enc=287.0, dx=0.0, dy=287.0, dr=0.0, ds=287.0,
            wheels=(287, 287, 287, 287),
        )

        app._handle_chassis_done(1, action, report, 10.0)

        app.navigator.apply_execution_delta.assert_called_once_with(0.0, 0.04, 0.0, 0.03, 0.0)
        app.navigator.predict_motion.assert_not_called()
        self.assertFalse(app.running)
        self.assertIn("禁止自动续航", app.navigator.detail)
        app._stop_radar_only.assert_called_once()
        scheduled = app.root.after.call_args.args[1]
        app.chassis_controller.complete_settle.return_value = True
        scheduled()
        app.chassis_controller.complete_settle.assert_called_once_with(resume_auto=False)

    def test_in_flight_blocks_reset_clear_disconnect_and_window_close(self):
        app = self.app()
        app.chassis_controller.in_flight = True
        app.chassis_controller.request_stop.return_value = True
        app.latest_points = []
        app.current_pixel = None
        app.latest_bias = 0.0
        app.connected = True
        app.simulation = None
        app.mapping_snapshot = object()
        app._complete_disconnect = Mock()
        app.stop = Mock()
        app.chassis_endpoint = Mock(is_open=True)
        app.measure_endpoint = Mock()
        app.rotation_endpoint = Mock()
        app.connection_label = Mock()
        app._closed = False
        app._close_requested = False
        app._close_deadline = None
        app.config["chassis_stop_timeout_s"] = 0.5
        app._finalize_close = Mock()

        app.reset()
        app.clear_local_map()
        app.disconnect()
        app.on_close()
        app._close_deadline = 0.0
        app._poll_close()

        self.assertEqual(app.chassis_controller.request_stop.call_count, 2)
        app.navigator.reset.assert_not_called()
        app._complete_disconnect.assert_not_called()
        app.chassis_endpoint.close.assert_not_called()
        app._finalize_close.assert_not_called()
        self.assertTrue(app._close_requested)
        self.assertGreaterEqual(app.root.after.call_count, 2)
        app.connection_label.configure.assert_any_call(
            text="●  停止未确认，保持底盘连接", fg="#ff6b6b"
        )

    def test_parking_completion_stops_radar_and_preserves_success_state(self):
        app = self.app()
        app.mapping_generation = 0
        app.mapping_tasks = queue.Queue()
        app.mapping_results = queue.Queue()
        app.simulation = None
        app.start_button = Mock()
        app.connection_label = Mock()
        app.connected = True
        app._send_chassis_stop = Mock(return_value=True)
        app._log = Mock()

        app._finish_parking()

        self.assertFalse(app.running)
        self.assertFalse(app.accept_samples)
        self.assertFalse(app.moving)
        self.assertEqual(app.navigator.state, "泊车完成")
        self.assertIn("雷达已停止", app.navigator.detail)
        app._send_chassis_stop.assert_called_once_with(wait=True)
        app.sync.stop.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()

