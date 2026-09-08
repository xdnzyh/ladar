import io
import unittest
from pathlib import Path
from unittest.mock import patch

from navigation_core import NavigationEngine, OccupancyGrid, Pose2D, ScanPoint
from radar_core import CalibrationModel
from scan_acquisition import TimedSweepBuilder


class TableCalibrationTests(unittest.TestCase):
    def test_bom_csv_uses_piecewise_interpolation_without_extrapolation(self):
        csv_text = "\ufeffdistance_cm,ccd_x\n8,1211\n13,1100\n18,1025\n23,982\n28,942\n33,921\n38,901\n43,886\n48,871\n53,862\n58,855\n"
        opener = lambda *args, **kwargs: io.TextIOWrapper(io.BytesIO(csv_text.encode("utf-8")), encoding="utf-8-sig")
        with patch("builtins.open", side_effect=opener):
            model = CalibrationModel.from_csv(Path("calibration.csv"))
        self.assertTrue(model.ready)
        self.assertAlmostEqual(model.distance(910), 0.3575)
        self.assertIsNone(model.distance(850))
        self.assertIsNone(model.distance(1300))


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
