import io
import unittest
from pathlib import Path
from unittest.mock import patch

from navigation_core import NavigationEngine, OccupancyGrid, Pose2D, ScanPoint
from measurement_protocol import parse_observation
from radar_core import CalibrationModel
from scan_acquisition import TimedSweepBuilder


class TableCalibrationTests(unittest.TestCase):
    def test_bom_csv_uses_piecewise_interpolation_without_extrapolation(self):
        csv_text = "\ufeffdistance_cm,ccd_x\n15,1203\n20,1099\n25,1033\n30,989\n35,947\n40,922\n45,901\n50,885\n55,873\n60,863\n65,853\n70,844\n75,838\n80,831\n85,827\n90,823\n95,818\n100,814\n"
        opener = lambda *args, **kwargs: io.TextIOWrapper(io.BytesIO(csv_text.encode("utf-8")), encoding="utf-8-sig")
        with patch("builtins.open", side_effect=opener):
            model = CalibrationModel.from_csv(Path("calibration.csv"))
        self.assertTrue(model.ready)
        self.assertEqual(model.distance_range_m, (0.15, 1.0))
        self.assertEqual(model.coordinate_range, (814.0, 1203.0))
        self.assertAlmostEqual(model.distance(910), 0.4285714286)
        self.assertIsNone(model.distance(813))
        self.assertIsNone(model.distance(1204))

    def test_all_current_calibration_points_round_trip_exactly(self):
        path = Path(__file__).resolve().parents[1] / "CCD_Distance_App_v1_2" / "calibration.csv"
        model = CalibrationModel.from_csv(path)
        for distance_cm, pixel in model.table_points:
            self.assertAlmostEqual(model.distance(pixel), distance_cm / 100.0)
        self.assertTrue(model.identifier.startswith("table-"))
        self.assertAlmostEqual(model.pixel_quantization_error_m(815), 0.00625)
        self.assertAlmostEqual(model.pixel_quantization_error_m(818), 0.00625)

    def test_observation_records_exact_calibration_fingerprint(self):
        path = Path(__file__).resolve().parents[1] / "CCD_Distance_App_v1_2" / "calibration.csv"
        model = CalibrationModel.from_csv(path)
        observation = parse_observation(
            "measurement",
            "PIX session-1 7 100000 102000 910",
            "session-1",
            model,
            {"min_range_m": 0.15, "max_range_m": 1.0},
        )
        self.assertIsNotNone(observation)
        self.assertEqual(observation.calibration_version, model.identifier)
        self.assertAlmostEqual(observation.distance, 0.4285714286)


class LocalSweepTests(unittest.TestCase):
    def test_stable_adjacent_zeroes_publish_sparse_closed_points(self):
        builder = TimedSweepBuilder({"min_scan_points": 12})
        builder.trigger(0.0, 0.001, 1)
        builder.trigger(1.0, 0.001, 2)
        builder.sample(1.2, 0.001, 900, 0.30, True)
        self.assertEqual(builder.trigger(2.0, 0.001, 3), [])
        self.assertEqual(len(builder.last_closed_points), 1)
        self.assertTrue(builder.last_closed_points[0].is_echo)

    def test_explicit_max_range_echo_is_an_obstacle_endpoint(self):
        grid = OccupancyGrid(40, 40, 0.02)
        navigator = NavigationEngine(grid, max_range_m=0.50)
        point = ScanPoint(0.0, 0.50, 1.0, True)
        navigator.process_local_scan([point], min_range_m=0.10)
        endpoint = grid.world_to_cell(0.0, 0.50)
        self.assertEqual(grid.state(*endpoint), grid.UNKNOWN)
        navigator.process_local_scan([point], min_range_m=0.10)
        navigator.process_local_scan([point], min_range_m=0.10)
        navigator.process_local_scan([point], min_range_m=0.10)
        self.assertEqual(grid.state(*endpoint), grid.OCCUPIED)

    def test_default_max_range_point_keeps_simulation_free_space_semantics(self):
        grid = OccupancyGrid(40, 40, 0.02)
        for _ in range(3):
            grid.update_scan(Pose2D(), [ScanPoint(0.0, 0.50)], 0.50)
        endpoint = grid.world_to_cell(0.0, 0.50)
        self.assertEqual(grid.state(*endpoint), grid.FREE)


if __name__ == "__main__":
    unittest.main()
