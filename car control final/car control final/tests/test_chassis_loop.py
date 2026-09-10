import math
import unittest

from chassis_controller import ChassisController, ChassisMotionAdapter, ChassisState
from chassis_protocol import firmware_mm_to_counts
from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint
from runtime_config import build_navigation_engine, resolve_runtime_config


class ImmediateEndpoint:
    is_open = True

    def __init__(self):
        self.writes = []

    def write_ticket(self, data, on_sent=None, on_written=None, **_kwargs):
        self.writes.append(bytes(data))
        if on_sent:
            on_sent(NOW[0])
        if on_written:
            on_written(NOW[0])
        return object()

    def cancel_write(self, _ticket):
        return "started"


NOW = [0.0]


def integrated_config():
    config = resolve_runtime_config("hardware", "navigation", {})
    config["chassis_firmware_confirmed"] = True
    config["chassis_capability_mode"] = "mm_ping_v1"
    config["chassis_speed_validated"] = True
    config["chassis_braking_validated"] = True
    config["safety_speed_upper_bound_mps"] = 0.30
    config["safety_stop_distance_m"] = 0.05
    for entry in config["chassis_translation_capabilities"].values():
        entry["motion_range_validated"] = True
        entry["validated_min_mm"] = 20.0
        entry["validated_max_mm"] = 200.0
        entry["uncertainty_m"] = 0.015
    return resolve_runtime_config("hardware", "navigation", config)


class ChassisLoopIntegrationTests(unittest.TestCase):
    def test_planner_adapter_fake_firmware_done_prior_and_new_scan(self):
        config = integrated_config()
        navigator = build_navigation_engine(config)
        navigator.set_auto(True)
        navigator.grid.log_odds[:] = [-6.0] * len(navigator.grid.log_odds)
        navigator.match_score = 0.95
        navigator.latest_scan = [ScanPoint(math.tau * index / 48, 1.0) for index in range(48)]
        start = navigator.grid.world_to_cell(0.0, 0.0)
        navigator.path_cells = [(start[0], start[1] - index) for index in range(8)]
        command = navigator._command_along_path()
        self.assertFalse(command.stopped)
        self.assertLessEqual(navigator._command_distance(command), NavigationEngine.MAX_MOTION_SEGMENT_M)

        adapter = ChassisMotionAdapter(config)
        request = adapter.request_for_command(command)
        self.assertEqual((request.mode, request.unit), ("W", "MM"))

        endpoint = ImmediateEndpoint()
        events = []
        controller = None

        def emit(kind, value, stamp):
            events.append((kind, value, stamp))
            if kind == "chassis_frame":
                generation, frame = value
                controller.handle_frame(generation, frame, stamp)

        controller = ChassisController(
            endpoint, emit, config, clock=lambda: NOW[0], nonce_factory=lambda: "12AB34CD",
        )
        generation = controller.begin_connection()
        controller.feed_data(b"IDLE X=0 Y=0 R=0 S=0\r\n", 0.0, generation)
        self.assertTrue(controller.allow_automatic())
        self.assertTrue(controller.request_move(request, source="auto", now=0.0))
        NOW[0] = 0.5
        controller.poll()
        controller.feed_data(b"@PONG,12AB34CD\r\n", 0.55, generation)
        NOW[0] = 0.80
        controller.poll()
        self.assertEqual(endpoint.writes[-1], f"@MOVE,W,{request.request_value},MM\r\n".encode())

        target_counts = firmware_mm_to_counts(
            request.request_value,
            fixed_counts_per_mm=config["chassis_translation_capabilities"]["W"]["fixed_counts_per_mm"],
        )
        done = (
            f"@DONE,W,TARGET,REQ={request.request_value},UNIT=MM,TARGET_CNT={target_counts},"
            f"BRAKE={target_counts},ENC={target_counts},DX={target_counts},DY=0,DR=0,DS=0,"
            f"Q1={target_counts},Q2={target_counts},Q3={target_counts},Q4={target_counts}\r\n"
        )
        controller.feed_data(done.encode(), 1.5, generation)
        self.assertEqual(controller.state, ChassisState.SETTLING)
        report = [event for event in events if event[0] == "chassis_done"][-1][1][2]
        estimate = adapter.execution_from_report(report)
        before_y = navigator.pose.y
        navigator.apply_execution_delta(
            estimate.local_x_m,
            estimate.local_y_m,
            estimate.yaw_rad,
            estimate.uncertainty_m,
            estimate.uncertainty_rad,
        )
        self.assertAlmostEqual(
            navigator.pose.y - before_y,
            target_counts / config["chassis_translation_capabilities"]["W"]["counts_per_mm"] / 1000,
        )
        self.assertTrue(controller.complete_settle(resume_auto=True, now=1.7))
        self.assertEqual(controller.state, ChassisState.WAITING_SCAN)

        previous_revision = navigator.grid._revision
        previous_scans = navigator.completed_scans
        new_scan = [ScanPoint(math.tau * index / 72, 0.65) for index in range(72)]
        navigator.process_scan(new_scan)
        self.assertGreater(navigator.grid._revision, previous_revision)
        self.assertGreater(navigator.completed_scans, previous_scans)
        self.assertTrue(controller.mark_scan_ready(now=3.2))
        self.assertEqual(controller.state, ChassisState.IDLE)
        self.assertEqual(endpoint.writes.count(
            f"@MOVE,W,{request.request_value},MM\r\n".encode()
        ), 1)


if __name__ == "__main__":
    unittest.main()
