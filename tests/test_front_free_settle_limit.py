import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from mapping_policy import complete_open_scan, prepare_free_space_points
from navigation_core import OccupancyGrid, Pose2D, ScanPoint
from navigation_app import NavigationApp
from chassis_controller import ChassisState


class FrontFreeAndSettleTests(unittest.TestCase):
    def test_one_scan_marks_front_one_metre_known_free(self):
        raw = [ScanPoint(math.radians(d), 1, is_echo=False) for d in range(0, 360, 10)]
        rays = complete_open_scan(raw, 1, 1, .08, .02)
        grid = OccupancyGrid(140, 140, .02)
        grid.update_scan(Pose2D(), prepare_free_space_points(rays, 1, .08, .02), 1, add_only=True)
        for x, y in ((0, .95), (.5, .5), (-.5, .5), (.8, .3)):
            self.assertEqual(grid.state(*grid.world_to_cell(x, y)), grid.FREE)
        self.assertEqual(grid.state(*grid.world_to_cell(0, 1.1)), grid.UNKNOWN)
        for row in range(grid.height):
            for col in range(grid.width):
                x, y = grid.cell_to_world(col, row)
                if y > 0 and math.hypot(x, y) < .98:
                    self.assertEqual(grid.state(col, row), grid.FREE, (x, y))

    def test_empty_scan_does_not_claim_known_free_space(self):
        self.assertFalse(complete_open_scan([], 1, 1, .08, .02))

    def test_front_wall_and_its_shadow_are_not_cleared(self):
        raw = [ScanPoint(math.radians(d), 1, is_echo=False) for d in range(0, 360, 10)]
        raw.append(ScanPoint(0, .4, is_echo=True))
        rays = complete_open_scan(raw, 1, 1, .08, .02)
        grid = OccupancyGrid(140, 140, .02)
        wall = grid.world_to_cell(0, .4)
        grid._add(*wall, 6)
        grid.update_scan(Pose2D(), prepare_free_space_points(rays, 1, .08, .02), 1, add_only=True)
        self.assertEqual(grid.state(*wall), grid.OCCUPIED)
        self.assertEqual(grid.state(*grid.world_to_cell(0, .7)), grid.UNKNOWN)

    def test_sensor_yaw_rotates_front_free_sector(self):
        raw = [ScanPoint(math.radians(d), 1, is_echo=False) for d in range(0, 360, 10)]
        rays = complete_open_scan(raw, 1, 1, .08, .02, front_angle_rad=-math.pi/2)
        self.assertTrue(all(math.sin(p.angle_rad) <= 1e-8 for p in rays if p.immediate_free))

    def app(self):
        app = NavigationApp.__new__(NavigationApp)
        app.config = {'hardware_settle_s': 10, 'synchronized_acquisition': True}
        app.root = Mock()
        app.sync = Mock()
        app.motion_generation = 7
        app.chassis_controller = SimpleNamespace(connection_generation=2, state=ChassisState.SETTLING,
                                                pending=SimpleNamespace(action_id=4, done_at=10))
        app._complete_chassis_settle = Mock()
        return app

    def test_settle_callback_is_capped_at_two_seconds(self):
        app = self.app()
        with patch('navigation_app.time.perf_counter', return_value=10):
            app._finish_chassis_action_after_settle(app.chassis_controller.pending, 2, 10, resume_auto=True)
        self.assertLessEqual(app.root.after.call_args.args[0], 2000)
        self.assertLessEqual(app.scan_collect_after, 12)

    def test_poll_releases_settle_if_normal_callback_was_missed(self):
        app = self.app()
        with patch('navigation_app.time.perf_counter', return_value=10):
            app._finish_chassis_action_after_settle(app.chassis_controller.pending, 2, 10, resume_auto=True)
        app._check_settle_timeout(11.9)
        app._complete_chassis_settle.assert_not_called()
        app._check_settle_timeout(12.01)
        app._complete_chassis_settle.assert_called_once_with(2, 4, 7, True)

    def test_timeout_does_not_release_unconfirmed_stop(self):
        app = self.app()
        with patch('navigation_app.time.perf_counter', return_value=10):
            app._finish_chassis_action_after_settle(app.chassis_controller.pending, 2, 10, resume_auto=True)
        app.chassis_controller.state = ChassisState.STOPPING
        app._check_settle_timeout(14)
        app._complete_chassis_settle.assert_not_called()

    def test_old_timer_does_not_release_new_action(self):
        app = self.app()
        with patch('navigation_app.time.perf_counter', return_value=10):
            app._finish_chassis_action_after_settle(app.chassis_controller.pending, 2, 10, resume_auto=True)
        app.chassis_controller.pending.action_id = 5
        app._check_settle_timeout(14)
        app._complete_chassis_settle.assert_not_called()


if __name__ == '__main__':
    unittest.main()
