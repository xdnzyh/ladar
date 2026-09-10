import math
import queue
import unittest
from unittest.mock import Mock

from mapping_policy import complete_open_scan, prepare_free_space_points
from mapping_runtime import MappingRuntime
from measurement_protocol import parse_observation
from navigation_core import NavigationEngine, OccupancyGrid, Pose2D, ScanPoint
from scan_acquisition import TimedSweepBuilder


class HardwareFreeSpaceTests(unittest.TestCase):
    def test_assumed_open_space_preserves_raw_wall_shadow_and_one_metre_limit(self):
        raw = [ScanPoint(math.radians(angle), 0.5, is_echo=True)
               for angle in range(-12, 13, 2)]
        points = complete_open_scan(raw, 1, 1, 0.15, 0.02)
        grid = OccupancyGrid(160, 160, 0.02)
        for _ in range(4):
            grid.update_scan(Pose2D(), points, 1, add_only=True)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 0.5)), grid.OCCUPIED)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 0.8)), grid.UNKNOWN)
        self.assertEqual(grid.state(*grid.world_to_cell(0.8, 0)), grid.FREE)
        self.assertEqual(grid.state(*grid.world_to_cell(1.1, 0)), grid.UNKNOWN)

    def test_open_space_assumption_requires_scan_and_is_disabled_for_simulation(self):
        self.assertEqual(complete_open_scan([], 1, 1, 0.15, 0.02), ())
        raw = tuple(ScanPoint(math.radians(a), 0.5, is_echo=True) for a in range(12))
        self.assertEqual(complete_open_scan(raw, 0, 3, 0.08, 0.04), raw)

    def test_valid_no_echo_and_beyond_range_are_free_but_invalid_measurements_are_not(self):
        calibration = Mock()
        calibration.distance.return_value = 2.0
        config = {'min_range_m': 0.1, 'max_range_m': 1.0}
        for pixel in (-1, 800):
            packet = parse_observation('measurement', f'PIX s 1 1000 2000 {pixel}', 's', calibration, config)
            self.assertEqual((packet.distance, packet.status, packet.is_echo), (1.0, 'over_range', False))
        packet = parse_observation('measurement', 'PIX s 1 1000 2000 -2', 's', calibration, config)
        self.assertEqual(packet.status, 'invalid_pixel')
        self.assertIsNone(packet.distance)
        calibration.distance.return_value = None
        packet = parse_observation('measurement', 'PIX s 1 1000 2000 800', 's', calibration, config)
        self.assertEqual(packet.status, 'calibration_outside')
        self.assertIsNone(packet.distance)

    def test_free_rays_preserve_wall_and_do_not_clear_unseen_or_occluded_cells(self):
        grid = OccupancyGrid(120, 120, 0.02)
        wall = grid.world_to_cell(0, 0.4)
        grid._add(*wall, 6)
        for _ in range(5):
            grid.update_scan(Pose2D(), [ScanPoint(0, 1, is_echo=False)], 1, add_only=True)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 0.2)), grid.FREE)
        self.assertEqual(grid.value(*wall), 6)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 0.6)), grid.UNKNOWN)
        self.assertEqual(grid.state(*grid.world_to_cell(0.3, 0.3)), grid.UNKNOWN)

    def test_empty_space_matches_simulation_ray_carving(self):
        hardware = OccupancyGrid(120, 120, 0.02)
        simulation = OccupancyGrid(120, 120, 0.02)
        rays = [ScanPoint(math.radians(angle), 1, is_echo=False) for angle in range(0, 90, 3)]
        for _ in range(3):
            hardware.update_scan(Pose2D(), rays, 1, add_only=True)
            simulation.update_scan(Pose2D(), rays, 1)
        self.assertEqual(hardware.log_odds, simulation.log_odds)

    def test_structure_filter_keeps_timed_no_echo_samples(self):
        builder = TimedSweepBuilder({'runtime_source': 'hardware', 'runtime_view': 'radar',
                                'min_scan_points': 12})
        builder.trigger(0, 0, 0)
        builder.trigger(1.5, 0, 1)
        for i in range(72):
            builder.sample(1.5 + (i + 0.5)/48, 0, -1, 1, False)
        points = builder.trigger(3, 0, 2)
        self.assertEqual(len(points), 72)
        self.assertTrue(all(not p.is_echo for p in points))

    def test_mapper_records_open_space_without_waiting_for_wall_fits(self):
        for mode in ('local', 'navigation'):
            with self.subTest(mode=mode):
                engine = NavigationEngine(OccupancyGrid(120, 120, 0.02), max_range_m=1)
                engine.set_auto(mode == 'navigation')
                output = queue.Queue()
                runtime = MappingRuntime(engine, on_result=output.put)
                try:
                    for sequence in range(4):
                        runtime.submit('s', sequence, [ScanPoint(0, 1, is_echo=False)], mode=mode)
                        result = output.get(timeout=2)
                        self.assertIsNone(result.error)
                    self.assertEqual(engine.grid.state(*engine.grid.world_to_cell(0, 0.5)), engine.grid.FREE)
                    self.assertEqual(engine.grid.state(*engine.grid.world_to_cell(0.5, 0)), engine.grid.UNKNOWN)
                finally:
                    runtime.stop()

    def test_rejected_wall_endpoint_only_clears_before_the_return(self):
        rays = prepare_free_space_points([ScanPoint(0, 0.5, is_echo=True)], 1, 0.1, 0.02)
        self.assertEqual(len(rays), 1)
        self.assertFalse(rays[0].is_echo)
        self.assertLess(rays[0].distance_m, 0.5)
