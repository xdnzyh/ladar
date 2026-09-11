import math
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch

from mapping_runtime import MappingRuntime
from navigation_core import HiddenWorld, NavigationEngine, OccupancyGrid, Pose2D, ScanPoint, VelocityCommand
from radar_core import CalibrationModel, PolarPoint, analyze_repeated_calibration
from runtime_config import RuntimeConfigError, build_navigation_engine, resolve_runtime_config
from scan_acquisition import DistanceObservationReceiver, HardwareObservation, ReceivedObservation, polar_to_scan_point
from synchronized_acquisition import ClockEstimate


class RuntimeConfigContractTests(unittest.TestCase):
    def test_mode_defaults_are_centered_and_share_engine_builder(self):
        hardware = resolve_runtime_config("hardware", "radar", {}, prefer_mode_defaults=True)
        simulation = resolve_runtime_config("simulation", "navigation", {}, prefer_mode_defaults=True)
        self.assertEqual((hardware["map_width_cells"], hardware["map_height_cells"]), (260, 760))
        self.assertEqual(hardware["map_resolution_m"], 0.02)
        self.assertEqual(hardware["hardware_sample_rate_hz"], 100.0)
        self.assertEqual((hardware["min_range_m"], hardware["max_range_m"]), (0.15, 1.0))
        self.assertEqual(hardware["actual_exposure_index"], 5)
        self.assertEqual((simulation["map_width_cells"], simulation["map_height_cells"]), (180, 280))
        self.assertEqual(simulation["map_resolution_m"], 0.04)
        self.assertEqual(simulation["simulation_sample_rate_hz"], 100.0)
        self.assertEqual(build_navigation_engine(simulation).path_turn_penalty, 0.75)
        self.assertEqual(build_navigation_engine(hardware).grid.origin_row, 699)
        self.assertEqual(build_navigation_engine(simulation).grid.origin_row, 140)
        custom = resolve_runtime_config(
            "simulation",
            "navigation",
            {"map_width_cells": 220, "map_height_cells": 160, "map_resolution_m": 0.03},
            prefer_mode_defaults=True,
        )
        self.assertEqual((custom["map_width_cells"], custom["map_height_cells"]), (220, 160))
        self.assertEqual(custom["map_resolution_m"], 0.03)

        custom_planner = resolve_runtime_config(
            "simulation",
            "navigation",
            {"path_turn_penalty": 1.25},
            prefer_mode_defaults=True,
        )
        self.assertEqual(build_navigation_engine(custom_planner).path_turn_penalty, 1.25)

    def test_nonfinite_configuration_is_rejected(self):
        with self.assertRaises(RuntimeConfigError):
            resolve_runtime_config("hardware", overrides={"map_resolution_m": math.nan})


class MappingContractTests(unittest.TestCase):
    def test_endpoint_only_evidence_does_not_expand_to_gaussian_neighbors(self):
        grid = OccupancyGrid(100, 100, 0.02)
        point = ScanPoint(0.0, 0.50, 1.0, True)
        for _ in range(3):
            grid.update_scan(Pose2D(), [point], 0.50)
        endpoint = grid.world_to_cell(0.0, 0.50)
        self.assertEqual(grid.state(*endpoint), grid.OCCUPIED)
        self.assertEqual(grid.state(endpoint[0] + 1, endpoint[1]), grid.UNKNOWN)
        self.assertEqual(grid.state(endpoint[0] - 1, endpoint[1]), grid.UNKNOWN)

    def test_same_scan_hit_overrides_free_evidence(self):
        grid = OccupancyGrid(100, 100, 0.02)
        points = [ScanPoint(0.0, 0.50, 1.0, False), ScanPoint(0.0, 0.50, 1.0, True)]
        for _ in range(3):
            grid.update_scan(Pose2D(), points, 0.50)
        self.assertEqual(grid.state(*grid.world_to_cell(0.0, 0.50)), grid.OCCUPIED)

    def test_sensor_pose_matching_is_inverted_to_body_pose(self):
        navigator = NavigationEngine(
            OccupancyGrid(100, 100, 0.02),
            sensor_offset_x_m=0.10,
            sensor_offset_y_m=0.20,
            sensor_offset_yaw_rad=math.pi / 2,
        )
        self.assertAlmostEqual(navigator._sensor_to_body(0.0, 1.0)[0], 1.10, places=6)
        self.assertAlmostEqual(navigator._sensor_to_body(0.0, 1.0)[1], 0.20, places=6)

    def test_no_echo_remains_at_max_range_without_noise(self):
        world = HiddenWorld(
            seed=4,
            map_data={
                "bounds": {"min_x": -10, "max_x": 10, "min_y": -10, "max_y": 10},
                "start": {"x": 0, "y": 0},
                "finish": {"x": 1, "y": 1},
                "obstacles": [],
            },
        )
        points = world.scan(sample_count=1, max_range_m=3.0, point_noise_m=1.0, drift_step_m=1.0)
        self.assertFalse(points[0].is_echo)
        self.assertEqual(points[0].distance_m, 3.0)

    def test_metadata_is_preserved_by_central_scan_point_conversion(self):
        polar = PolarPoint(
            0.3, 1.0, 2.0, 942, True,
            time_error_s=0.002,
            angle_error_rad=0.01,
            distance_error_m=0.003,
            calibration_version="table-v1",
            source="range",
            session="s1",
        )
        point = polar_to_scan_point(polar)
        self.assertEqual(point.pixel, 942)
        self.assertEqual(point.calibration_version, "table-v1")
        self.assertEqual(point.session, "s1")
        self.assertAlmostEqual(point.angle_error_rad, 0.01)

    def test_repeated_calibration_reports_uncertainty_without_refitting(self):
        model = CalibrationModel.from_table([(15, 1203), (20, 1099), (25, 1033)])
        report = analyze_repeated_calibration({0.20: [1099, 1100, 1098]}, model)
        self.assertFalse(report["refit"])
        self.assertEqual(report["samples"][0]["sample_count"], 3)
        self.assertIsNotNone(report["samples"][0]["quantization_error_m"])


class AcquisitionContractTests(unittest.TestCase):
    def test_clock_model_update_does_not_make_mapped_timestamp_rollback(self):
        receiver = DistanceObservationReceiver({"min_range_m": 0.1, "max_range_m": 0.5})
        first = HardwareObservation("range", 1, 1.0, 0.2, raw_timestamp_us=100000, clock_model_version=0)
        updated = HardwareObservation("range", 2, 0.5, 0.2, raw_timestamp_us=110000, clock_model_version=1)
        self.assertTrue(receiver.feed(ReceivedObservation(first, 1.0)))
        self.assertTrue(receiver.feed(ReceivedObservation(updated, 1.1)))
        self.assertEqual(receiver.progress["range"][0], 2)

    def test_raw_timestamp_rollback_is_rejected_even_after_model_update(self):
        receiver = DistanceObservationReceiver({"min_range_m": 0.1, "max_range_m": 0.5})
        first = HardwareObservation("range", 1, 1.0, 0.2, raw_timestamp_us=100000, clock_model_version=0)
        rollback = HardwareObservation("range", 2, 1.1, 0.2, raw_timestamp_us=90000, clock_model_version=1)
        self.assertTrue(receiver.feed(ReceivedObservation(first, 1.0)))
        self.assertFalse(receiver.feed(ReceivedObservation(rollback, 1.1)))


class MappingRuntimeContractTests(unittest.TestCase):
    def test_reset_barrier_drops_an_old_inflight_request(self):
        engine = build_navigation_engine(resolve_navigation_config())
        started = threading.Event()
        release = threading.Event()
        results = []

        def blocked(_self, _points, min_range_m=0.08, scan_confidence=0.7, **kwargs):
            started.set()
            release.wait(1.0)
            return VelocityCommand()

        runtime = MappingRuntime(engine, queue_size=1, on_result=results.append)
        try:
            with patch("mapping_runtime.process_radar_debug_scan", side_effect=blocked):
                request = runtime.submit("session", 1, [ScanPoint(0.0, 0.3)], 1.0, mode="local")
                self.assertIsNotNone(request)
                self.assertTrue(started.wait(1.0))
                runtime.invalidate()
                release.set()
                deadline = time.time() + 1.0
                while time.time() < deadline and runtime.queue_depth:
                    time.sleep(0.01)
            self.assertFalse(any(item.request.mode == "local" for item in results))
        finally:
            release.set()
            runtime.stop()


def resolve_navigation_config():
    return resolve_runtime_config("simulation", "navigation", {
        "simulation_profile": "IDEAL",
        "simulation_min_range_m": 0.08,
        "simulation_max_range_m": 3.0,
    }, prefer_mode_defaults=True)


if __name__ == "__main__":
    unittest.main()
