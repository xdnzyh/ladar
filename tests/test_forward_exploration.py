import math
import queue
import unittest

from mapping_runtime import MappingRuntime
from navigation_core import HiddenWorld, NavigationEngine, OccupancyGrid, ScanPoint


class ForwardExplorationTests(unittest.TestCase):
    def scene(self, yaw=0):
        grid = OccupancyGrid(120, 120, 0.02)
        engine = NavigationEngine(grid, max_range_m=1)
        engine.prefer_forward_exploration = True
        engine.set_auto(True)
        engine.pose.yaw = yaw
        engine.start_pose.yaw = yaw
        engine.match_score = 1
        # A broad observed opening: side frontiers must not pull the car aside.
        for row in range(grid.height):
            for col in range(grid.width):
                x, y = grid.cell_to_world(col, row)
                if math.hypot(x, y) < 0.95:
                    grid._add(col, row, -10)
        engine.latest_scan = [ScanPoint(math.radians(d), 1, is_echo=False)
                              for d in range(0, 360, 2)]
        return engine

    def test_broad_opening_prefers_body_forward_at_different_headings(self):
        for yaw in (0, math.radians(35), math.pi / 2):
            with self.subTest(yaw=yaw):
                engine = self.scene(yaw)
                command = engine._plan_next_command()
                self.assertGreater(command.forward_mps, 0)
                self.assertEqual(command.right_mps, 0)
                self.assertEqual(command.yaw_rps, 0)
                self.assertLessEqual(engine._command_distance(command), 0.2)

    def test_front_obstacle_uses_side_opening(self):
        engine = self.scene()
        engine.translation_capabilities['W']['min_m'] = 0.08
        for i in range(-12, 13):
            x, y = i * 0.02, 0.24
            engine.grid._add(*engine.grid.world_to_cell(x, y), 20)
            engine.latest_scan.append(ScanPoint(math.atan2(x, y), math.hypot(x, y), is_echo=True))
        command = engine._plan_next_command()
        self.assertFalse(command.stopped)
        self.assertNotEqual(command.right_mps, 0)
        self.assertFalse(engine._translation_guard(command)[0].stopped)

    def test_forward_step_shortens_before_a_wall(self):
        engine = self.scene()
        for i in range(-12, 13):
            x, y = i * 0.02, 0.30
            engine.grid._add(*engine.grid.world_to_cell(x, y), 20)
            engine.latest_scan.append(ScanPoint(math.atan2(x, y), math.hypot(x, y), is_echo=True))
        command = engine._plan_next_command()
        self.assertGreater(command.forward_mps, 0)
        self.assertEqual(command.right_mps, 0)
        self.assertLess(engine._command_distance(command), 0.1)
        self.assertFalse(engine._translation_guard(command)[0].stopped)

    def test_unknown_space_does_not_allow_motion(self):
        engine = self.scene()
        engine.grid = OccupancyGrid(120, 120, 0.02)
        self.assertTrue(engine._plan_next_command().stopped)

    def test_forward_step_respects_capability_and_localization_confidence(self):
        for maximum, score, expected in ((0.12, 1, 0.12), (0.2, 0.6, 0.14 * 0.38)):
            with self.subTest(maximum=maximum, score=score):
                engine = self.scene()
                engine.translation_capabilities['W']['max_m'] = maximum
                engine.match_score = score
                command = engine._plan_next_command()
                self.assertGreater(command.forward_mps, 0)
                self.assertEqual(command.right_mps, 0)
                self.assertLessEqual(engine._command_distance(command), expected + 1e-9)

    def test_mapping_runtime_prioritizes_forward_in_wide_open_corridor(self):
        engine = NavigationEngine(OccupancyGrid(120, 120, 0.02), max_range_m=1,
                                  unobserved_clear_range_m=1, prefer_forward_exploration=True)
        engine.set_auto(True)
        world = HiddenWorld(map_data={
            'bounds': {'min_x': -5, 'max_x': 5, 'min_y': -5, 'max_y': 5},
            'start': {'x': 0, 'y': 0, 'yaw_deg': 0},
            'finish': {'x': 0, 'y': 4},
            'obstacles': [
                {'x1': -0.8, 'y1': -0.5, 'x2': -0.8, 'y2': 4},
                {'x1': 0.8, 'y1': -0.5, 'x2': 0.8, 'y2': 4},
                {'x1': -0.8, 'y1': -0.5, 'x2': 0.8, 'y2': -0.5},
            ],
        })
        points = []
        for degree in range(0, 360, 2):
            angle = math.radians(degree)
            distance = world.ray_distance(angle, 1)
            if distance < 1:
                points.append(ScanPoint(angle, distance, is_echo=True))
        output = queue.Queue()
        runtime = MappingRuntime(engine, on_result=output.put)
        try:
            commands = []
            for sequence in range(6):
                runtime.submit('wide-corridor', sequence, points)
                result = output.get(timeout=5)
                self.assertIsNone(result.error)
                if not result.command.stopped:
                    commands.append(result.command)
            self.assertTrue(commands, engine.detail)
            self.assertTrue(all(c.forward_mps > 0 and c.right_mps == 0 and c.yaw_rps == 0
                                for c in commands))
        finally:
            runtime.stop()

    def test_missing_scan_does_not_allow_forward_motion(self):
        engine = self.scene()
        engine.latest_scan = []
        self.assertTrue(engine._plan_next_command().stopped)
