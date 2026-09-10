import json
from pathlib import Path
import unittest

from radar_core import PolarPoint
from scan_acquisition import TimedSweepBuilder
from scan_acquisition import scan_points_from_polar
from mapping_policy import prepare_mapping_points


class CapturedWallScanTests(unittest.TestCase):
    def replay(self, view='radar', uncertainty_scale=1):
        fixture = json.loads((Path(__file__).parent / 'fixtures/radar_wall_scan_20260910.json').read_text())
        start, start_error, count = fixture['start']
        end, end_error, next_count = fixture['end']
        points = [PolarPoint(**p) for p in fixture['points']]
        # This historical capture was recorded with clockwise rotation.
        builder = TimedSweepBuilder({'clockwise': True, 'runtime_source': 'hardware', 'runtime_view': view, 'min_scan_points': 40})
        builder.trigger(start - (end-start), start_error * uncertainty_scale, count-1)
        builder.trigger(start, start_error * uncertainty_scale, count)
        for p in points:
            builder.sample(p.timestamp, p.time_error_s * uncertainty_scale, p.pixel, p.distance_m,
                           p.is_echo, p.distance_error_m, p.calibration_version, p.session)
        result = builder.trigger(end, end_error * uncertainty_scale, next_count)
        return builder, points, result

    def test_acquisition_preserves_walls_and_raw_isolated_echo(self):
        builder, original, selected = self.replay()
        selected_times = {p.timestamp for p in selected}
        front = [p for p in original if p.y > .30 and abs(p.x) < .15]
        left = [p for p in original if p.x < -.38 and -.08 < p.y < .15]
        noise = [p for p in original if -.24 < p.x < -.20 and .07 < p.y < .12]
        self.assertGreaterEqual(len(front), 8)
        self.assertGreaterEqual(len(left), 5)
        self.assertEqual(len(noise), 1)
        self.assertTrue(all(p.timestamp in selected_times for p in front + left))
        self.assertIn(noise[0].timestamp, selected_times)
        self.assertEqual(len(selected), 71)
        self.assertEqual(len(builder.last_display_points), 71)
        # Geometry support must not erase the original clock uncertainty.
        self.assertTrue(all(p.angle_error_rad > .3 for p in selected))

    def test_navigation_keeps_strict_time_gate(self):
        _, _, selected = self.replay(view='navigation')
        self.assertEqual(len(selected), 48)

    def test_wall_segments_reach_actual_mapping_preparation(self):
        _, _, selected = self.replay()
        mapping = prepare_mapping_points(scan_points_from_polar(selected), 1.0, .15, .02)
        self.assertGreaterEqual(sum(p.x < -.38 and -.08 < p.y < .15 for p in mapping.points), 5)
        self.assertGreaterEqual(sum(p.y > .30 and abs(p.x) < .15 for p in mapping.points), 8)
        self.assertFalse(any(-.24 < p.x < -.20 and .07 < p.y < .12 for p in mapping.points))

    def test_structure_cannot_rescue_unbounded_angular_error(self):
        builder, _, selected = self.replay(uncertainty_scale=3)
        self.assertFalse(selected)
        self.assertEqual(len(builder.last_display_points), 71)


if __name__ == '__main__':
    unittest.main()
