import math
import unittest

from data_fusion import DeviceClock, SequenceMonitor, parse_chassis_state, parse_timestamped_distance, parse_trigger
from navigation_core import HiddenWorld, NavigationEngine, OccupancyGrid, Pose2D, ScanPoint, VelocityCommand, mecanum_mix


class OccupancyGridTests(unittest.TestCase):
    def test_scan_marks_free_space_and_obstacle(self):
        grid = OccupancyGrid(80, 80, 0.05)
        points = [ScanPoint(0.0, 1.0) for _ in range(3)]
        grid.update_scan(Pose2D(), points, 3.0)
        start = grid.world_to_cell(0.0, 0.0)
        middle = grid.world_to_cell(0.0, 0.5)
        end = grid.world_to_cell(0.0, 1.0)
        self.assertEqual(grid.state(*start), grid.FREE)
        self.assertEqual(grid.state(*middle), grid.FREE)
        self.assertEqual(grid.state(*end), grid.OCCUPIED)

    def test_frontier_and_path_exist_after_scan(self):
        grid = OccupancyGrid(100, 100, 0.05)
        points = [ScanPoint(math.tau * index / 72, 1.2) for index in range(72)]
        for _ in range(3):
            grid.update_scan(Pose2D(), points, 3.0)
        clusters = grid.frontier_clusters()
        self.assertTrue(clusters)
        goal = grid.nearest_cell_to_centroid(clusters[0])
        path = grid.astar(grid.world_to_cell(0.0, 0.0), goal, 0.10)
        self.assertGreater(len(path), 1)


class SimulatorTests(unittest.TestCase):
    def test_line_map_is_loaded_and_blocks_lidar_ray(self):
        world = HiddenWorld(
            seed=3,
            map_data={
                "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -2.0, "max_y": 2.0},
                "wall_thickness_m": 0.06,
                "start": {"x": 0.0, "y": -1.0, "yaw_deg": 0.0},
                "finish": {"x": 0.0, "y": 1.0},
                "obstacles": [{"x1": -1.0, "y1": 0.0, "x2": 1.0, "y2": 0.0}],
            },
        )
        self.assertTrue(world.map_loaded)
        self.assertEqual(len(world.wall_segments), 1)
        self.assertAlmostEqual(world.ray_distance(0.0, 3.0), 0.98, delta=0.03)

    def test_hidden_world_has_random_walk_distance_drift(self):
        world = HiddenWorld(seed=7)
        scans = [world.scan(sample_count=24) for _ in range(8)]
        self.assertTrue(all(len(scan) == 24 for scan in scans))
        self.assertLessEqual(abs(world.distance_bias_m), 0.035)
        self.assertNotEqual(world.distance_bias_m, 0.0)

    def test_blocked_simulation_motion_reports_no_translation(self):
        world = HiddenWorld(
            seed=4,
            map_data={
                "bounds": {"min_x": -1.0, "max_x": 1.0, "min_y": -1.0, "max_y": 1.0},
                "start": {"x": 0.0, "y": -0.3, "yaw_deg": 0.0},
                "finish": {"x": 0.0, "y": 0.7},
                "obstacles": [{"x1": -0.8, "y1": 0.0, "x2": 0.8, "y2": 0.0}],
            },
        )
        applied = world.apply(VelocityCommand(forward_mps=0.5, duration_s=1.0))
        self.assertTrue(applied.stopped)
        self.assertAlmostEqual(world.pose.y, -0.3)

    def test_swept_robot_footprint_rejects_corner_collision(self):
        navigator = NavigationEngine(OccupancyGrid(), robot_radius_m=0.15)
        navigator.latest_scan = [ScanPoint(math.pi / 2, 0.20)]
        command = VelocityCommand(right_mps=0.13, duration_s=0.42)
        self.assertFalse(navigator._command_has_clearance(command))

    def test_navigation_builds_map_without_access_to_hidden_map(self):
        world = HiddenWorld(seed=20260903)
        navigator = NavigationEngine(
            OccupancyGrid(180, 180, 0.04),
            max_range_m=3.0,
            robot_radius_m=0.15,
        )
        navigator.set_auto(True)
        for _ in range(150):
            command = navigator.process_scan(world.scan(sample_count=30))
            applied = world.apply(command)
            navigator.predict_motion(applied)
            if navigator.state == "泊车完成":
                break
        finish_distance = math.hypot(world.pose.x - world.finish[0], world.pose.y - world.finish[1])
        self.assertEqual(navigator.state, "泊车完成")
        self.assertLess(finish_distance, 0.50)
        self.assertGreater(navigator.grid.known_area_m2(), 8.0)


class MecanumTests(unittest.TestCase):
    def test_forward_and_strafe_mix(self):
        self.assertEqual(mecanum_mix(VelocityCommand(forward_mps=1.0)), (1.0, 1.0, 1.0, 1.0))
        self.assertEqual(mecanum_mix(VelocityCommand(right_mps=1.0)), (-1.0, 1.0, 1.0, -1.0))


class FusionTests(unittest.TestCase):
    def test_device_clock_maps_microseconds(self):
        clock = DeviceClock()
        mapped = clock.observe(2_000_000, 14.0)
        self.assertAlmostEqual(mapped, 14.0, places=6)
        for index in range(1, 12):
            clock.observe(2_000_000 + index * 100_000, 14.0 + index * 0.1 + (0.001 if index % 3 else 0.0))
        self.assertAlmostEqual(clock.to_host(3_000_000), 15.0, delta=0.01)

    def test_protocol_parsers(self):
        self.assertEqual(parse_trigger("TRIG 12 1234567"), (12, 1234567))
        self.assertEqual(parse_timestamped_distance("DIST,9,1234567,0.82,744"), (9, 1234567, 0.82, 744))
        self.assertEqual(parse_chassis_state("CHASSIS,7,1234600,10,-10,10,-10"), (7, 1234600, (10.0, -10.0, 10.0, -10.0)))
        monitor = SequenceMonitor()
        self.assertEqual(monitor.add("distance", 10), 0)
        self.assertEqual(monitor.add("distance", 13), 2)


if __name__ == "__main__":
    unittest.main()
