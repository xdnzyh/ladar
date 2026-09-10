import math
import threading
import unittest

from mapping_runtime import MappingRuntime
from navigation_core import HiddenWorld, NavigationEngine, OccupancyGrid, Pose2D, ScanPoint


class NavigationFreeSpaceTests(unittest.TestCase):
    def test_hardware_opening_without_return_samples_allows_forward_motion(self):
        from runtime_config import build_navigation_engine, resolve_runtime_config
        import json
        from pathlib import Path
        config = json.loads((Path(__file__).resolve().parents[1] / 'navigation_config.json').read_text(encoding='utf-8'))
        navigator = build_navigation_engine(resolve_runtime_config('hardware', 'navigation', config))
        navigator.set_auto(True)
        world = HiddenWorld(map_data={
            'bounds': {'min_x': -5, 'max_x': 5, 'min_y': -5, 'max_y': 5},
            'start': {'x': 0, 'y': 0, 'yaw_deg': 0},
            'finish': {'x': 0, 'y': 4},
            'obstacles': [
                {'x1': -0.6, 'y1': -0.5, 'x2': -0.6, 'y2': 4},
                {'x1': 0.6, 'y1': -0.5, 'x2': 0.6, 'y2': 4},
                {'x1': -0.6, 'y1': -0.5, 'x2': 0.6, 'y2': -0.5},
            ],
        })
        points = []
        for degree in range(0, 360, 2):
            angle = math.radians(degree)
            distance = world.ray_distance(angle, 1.0)
            if distance < 1.0:
                points.append(ScanPoint(angle, distance, is_echo=True))
        import queue
        output = queue.Queue()
        runtime = MappingRuntime(navigator, on_result=output.put)
        try:
            commands = []
            for sequence in range(6):
                runtime.submit('opening', sequence, points)
                result = output.get(timeout=10)
                self.assertIsNone(result.error)
                commands.append(result.command)
            self.assertEqual(navigator.grid.state(*navigator.grid.world_to_cell(0, 0.8)), OccupancyGrid.FREE)
            self.assertEqual(navigator.grid.state(*navigator.grid.world_to_cell(0, 1.1)), OccupancyGrid.UNKNOWN)
            self.assertTrue(any(c and c.forward_mps > 0 for c in commands), (navigator.state, navigator.detail))
        finally:
            runtime.stop()

    def test_runtime_preserves_open_forward_sector(self):
        world = HiddenWorld(map_data={
            'bounds': {'min_x': -5, 'max_x': 5, 'min_y': -5, 'max_y': 5},
            'start': {'x': 0, 'y': 0, 'yaw_deg': 0},
            'finish': {'x': 0, 'y': 4},
            'obstacles': [
                {'x1': -0.6, 'y1': -0.5, 'x2': -0.6, 'y2': 4},
                {'x1': 0.6, 'y1': -0.5, 'x2': 0.6, 'y2': 4},
                {'x1': -0.6, 'y1': -0.5, 'x2': 0.6, 'y2': -0.5},
            ],
        })
        self.assertFalse(world.map_error)
        points = []
        for degree in range(0, 360, 2):
            angle = math.radians(degree)
            distance = world.ray_distance(angle, 3.0)
            points.append(ScanPoint(angle, distance, is_echo=distance < 3.0))
        navigator = NavigationEngine(OccupancyGrid())
        navigator.set_auto(True)
        done = threading.Event()
        results = []
        def received(result):
            results.append(result)
            done.set()
        runtime = MappingRuntime(navigator, on_result=received)
        try:
            for sequence in range(6):
                done.clear()
                runtime.submit('free-space', sequence, points)
                self.assertTrue(done.wait(10))
                self.assertIsNone(results[-1].error)
            self.assertEqual(navigator.grid.state(*navigator.grid.world_to_cell(0, 2.0)), OccupancyGrid.FREE,
                             (navigator.state, navigator.detail, navigator.grid.update_count))
            self.assertTrue(any(result.command and not result.command.stopped for result in results),
                            (navigator.state, navigator.detail))
        finally:
            runtime.stop()

    def test_adjacent_no_echo_rays_fill_scan_spacing(self):
        grid = OccupancyGrid(200, 200, 0.04)
        points = [ScanPoint(math.radians(d), 3, is_echo=False) for d in range(0, 360, 3)]
        for _ in range(3):
            grid.update_scan(Pose2D(), points, 3)
        self.assertEqual(grid.state(*grid.world_to_cell(0.06, 2)), grid.FREE)

    def test_missing_sector_and_wall_shadow_stay_unknown(self):
        grid = OccupancyGrid(200, 200, 0.04)
        points = [ScanPoint(math.radians(d), 1, is_echo=True) for d in (-3, 0, 3)]
        for _ in range(3):
            grid.update_scan(Pose2D(), points, 3)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 1.5)), grid.UNKNOWN)
        self.assertEqual(grid.state(*grid.world_to_cell(1, 0)), grid.UNKNOWN)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 1)), grid.OCCUPIED)

    def test_wraparound_open_sector_stops_at_sensor_range(self):
        grid = OccupancyGrid(200, 200, 0.04)
        points = [ScanPoint(math.radians(d), 3, is_echo=False) for d in (358, 2)]
        for _ in range(3):
            grid.update_scan(Pose2D(), points, 3)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 2)), grid.FREE)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 3.2)), grid.UNKNOWN)
        self.assertFalse(grid.occupied_cells())

    def test_depth_discontinuity_does_not_clear_behind_nearer_wall(self):
        grid = OccupancyGrid(200, 200, 0.04)
        points = [ScanPoint(math.radians(-2), 0.8, is_echo=True),
                  ScanPoint(math.radians(2), 3, is_echo=False)]
        for _ in range(3):
            grid.update_scan(Pose2D(), points, 3)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 0.5)), grid.FREE)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 2)), grid.UNKNOWN)
