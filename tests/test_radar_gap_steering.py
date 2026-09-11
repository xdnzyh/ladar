import math
import unittest

from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint, VelocityCommand


class RadarGapSteeringTests(unittest.TestCase):
    def engine(self):
        engine = NavigationEngine(
            OccupancyGrid(120, 120, .02),
            max_range_m=1,
            forward_only=True,
            rotation_enabled=True,
            radar_gap_steering=True,
            distance_controlled_motion=True,
        )
        engine.set_auto(True)
        return engine

    @staticmethod
    def full_open_scan():
        return [ScanPoint(math.radians(degrees), 1, is_echo=False)
                for degrees in range(-180, 180, 5)]

    def test_fresh_radar_gap_moves_without_a_free_global_map(self):
        engine = self.engine()
        engine.latest_scan = self.full_open_scan()
        engine.match_score = .95

        command = engine._plan_next_command()

        self.assertGreater(command.forward_mps, 0)
        self.assertAlmostEqual(engine._command_distance(command), .20)
        self.assertEqual(engine.state, "沿雷达缺口前进")

    def test_low_map_confidence_does_not_shorten_a_clear_radar_step(self):
        engine = self.engine()
        engine.latest_scan = self.full_open_scan()
        engine.match_score = 0

        command = engine._plan_next_command()

        self.assertGreater(command.forward_mps, 0)
        self.assertAlmostEqual(engine._command_distance(command), .20)
        self.assertIn("偏低", engine.detail)

    def test_front_wall_shortens_the_normal_twenty_centimetre_step(self):
        engine = self.engine()
        engine.latest_scan = [
            ScanPoint(math.radians(degrees), .38 if abs(degrees) <= 5 else 1,
                      is_echo=abs(degrees) <= 5)
            for degrees in range(-180, 180, 5)
        ]
        engine.match_score = .95

        command = engine._plan_next_command()

        self.assertGreater(command.forward_mps, 0)
        self.assertLess(engine._command_distance(command), .20)
        self.assertGreater(engine._command_distance(command), 0)

    def test_first_complete_circle_acts_without_waiting_for_map_bootstrap(self):
        engine = self.engine()

        command = engine.process_scan(self.full_open_scan())

        self.assertFalse(command.stopped)
        self.assertEqual(engine.completed_scans, 1)
        self.assertEqual(engine.grid.update_count, 0)

    def test_locked_gap_is_not_replaced_by_a_wider_opposite_gap(self):
        engine = self.engine()
        # A clear right-hand opening amid nearer front walls.
        engine.latest_scan = [
            ScanPoint(math.radians(degrees), 1 if 40 <= degrees <= 75 else .25,
                      is_echo=not (40 <= degrees <= 75))
            for degrees in range(-180, 180, 5)
        ]
        first = engine._plan_next_command()
        self.assertGreater(first.yaw_rps, 0)

        # The left opening becomes wider, but the original gap is still seen.
        engine.latest_scan = [
            ScanPoint(math.radians(degrees), 1 if (40 <= degrees <= 75 or -80 <= degrees <= -30) else .25,
                      is_echo=not (40 <= degrees <= 75 or -80 <= degrees <= -30))
            for degrees in range(-180, 180, 5)
        ]
        second = engine._plan_next_command()
        self.assertGreater(second.yaw_rps, 0)

    def test_absolute_forward_gap_beats_a_wider_side_gap_after_turning(self):
        engine = self.engine()
        # The robot has previously turned right.  A relative-forward opening
        # is therefore map-right, while the course's absolute forward opening
        # is at the relative left edge of this scan.
        engine.pose.yaw = math.radians(90)
        engine.latest_scan = [
            ScanPoint(math.radians(degrees),
                      1 if (-90 <= degrees <= -45 or -30 <= degrees <= 30) else .25,
                      is_echo=not (-90 <= degrees <= -45 or -30 <= degrees <= 30))
            for degrees in range(-180, 180, 5)
        ]

        command = engine._plan_next_command()

        # The initial vehicle heading defines map-forward (0 degrees), so the
        # controller must turn left toward that gap rather than drive sideward.
        self.assertLess(command.yaw_rps, 0)

    def test_front_wall_cancels_the_apparent_gap_before_translation(self):
        engine = self.engine()
        engine.rotation_enabled = False
        # Only the centre would otherwise be a gap; an actual return in that
        # predicted sweep must prevent the car from moving through it.
        engine.latest_scan = [
            ScanPoint(math.radians(degrees), .25, is_echo=True)
            for degrees in range(-180, 180, 5)
        ]
        engine.latest_scan += [
            ScanPoint(math.radians(degrees), 1, is_echo=False)
            for degrees in range(-25, 30, 5)
        ]
        engine.latest_scan.append(ScanPoint(0, .25, is_echo=True))

        command = engine._plan_next_command()

        self.assertTrue(command.stopped)
        self.assertIn(engine.state, {"前方无可信缺口", "雷达缺口受阻"})

    def test_hardware_configuration_enables_direct_radar_gap_control(self):
        from navigation_app import load_configuration
        from runtime_config import build_navigation_engine

        engine = build_navigation_engine(load_configuration("hardware", "navigation"))

        self.assertTrue(engine.radar_gap_steering)

    def test_normal_navigation_rejects_strafe_and_diagonal_but_recovery_may_strafe(self):
        engine = self.engine()
        engine.forward_turn_only = True
        engine.grid.log_odds[:] = [-6.] * len(engine.grid.log_odds)
        engine.latest_scan = self.full_open_scan()

        side, _ = engine._translation_guard(
            VelocityCommand(right_mps=.1, duration_s=1),
        )
        diagonal, _ = engine._translation_guard(
            VelocityCommand(forward_mps=.1, right_mps=.1, duration_s=1),
        )
        self.assertTrue(side.stopped)
        self.assertTrue(diagonal.stopped)

        engine.latest_scan.append(ScanPoint(-math.pi / 2, .4, is_echo=True))
        engine.recovery_requested = True
        escape = engine._recovery_command()
        self.assertFalse(escape.stopped)
        self.assertEqual(escape.forward_mps, 0)
        self.assertNotEqual(escape.right_mps, 0)

    def test_hardware_adapter_enforces_forward_turn_only_at_the_send_boundary(self):
        from chassis_controller import ChassisMotionAdapter, MotionConversionError
        from navigation_app import load_configuration

        adapter = ChassisMotionAdapter(load_configuration("hardware", "navigation"))
        with self.assertRaises(MotionConversionError):
            adapter.request_for_command(VelocityCommand(right_mps=.1, duration_s=1), automatic=False)
        with self.assertRaises(MotionConversionError):
            adapter.request_for_command(
                VelocityCommand(forward_mps=.1, right_mps=.1, duration_s=1), automatic=False,
            )
        with self.assertRaises(MotionConversionError):
            adapter.request_for_manual("D", 100, "MM")
        request = adapter.request_for_command(
            VelocityCommand(right_mps=.1, duration_s=1, recovery_translation=True), automatic=False,
        )
        self.assertEqual(request.mode, "D")


if __name__ == "__main__":
    unittest.main()
