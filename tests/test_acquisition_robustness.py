import math
import unittest

from measurement_protocol import DeviceObservation, parse_observation
from radar_core import CalibrationModel
from scan_acquisition import TimedSweepBuilder, DistanceObservationReceiver, HardwareObservation, ReceivedObservation
from synchronized_acquisition import ClockEstimate


class RobustSweepTests(unittest.TestCase):
    def builder(self, **config):
        builder = TimedSweepBuilder(config)
        for n in range(4):
            builder.trigger(n * 1.5, 0, n + 1)
        return builder

    def circle(self, bad=(), missing=(), period=1.5, **config):
        builder = self.builder(**config)
        for n in range(30):
            if n not in missing:
                builder.sample(4.5 + (n + 0.5) * period / 30, 0.05 if n in bad else 0, n, 1)
        points = builder.trigger(4.5 + period, 0, 5)
        return builder, points

    def test_one_uncertain_point_does_not_discard_sweep(self):
        builder, points = self.circle(bad=(10,))
        self.assertEqual(len(points), 29)
        self.assertEqual(builder.rejected_points, 1)
        self.assertNotIn(10, [p.pixel for p in points])

    def test_adaptive_gap_accepts_isolated_missing_point_with_jitter(self):
        builder = self.builder()
        for n in range(30):
            if n != 10:
                builder.sample(4.5 + (n + 0.5) * 0.05 + (0.01 if n == 11 else 0), 0, n, 1)
        self.assertEqual(len(builder.trigger(6, 0, 5)), 29)

    def test_two_consecutive_missing_points_are_not_a_small_hole(self):
        builder, points = self.circle(missing=(10, 11))
        self.assertEqual(points, [])
        self.assertIn("角度空缺", builder.reason)

    def test_consecutive_uncertain_points_fail_final_coverage_check(self):
        builder, points = self.circle(bad=range(8, 14))
        self.assertEqual(builder.rejected_points, 6)
        self.assertEqual(points, [])
        self.assertIn("角度空缺", builder.reason)

    def test_real_neighbor_period_overrides_historical_mean(self):
        builder, points = self.circle(period=1.53)
        self.assertEqual(len(points), 30)
        self.assertAlmostEqual(builder.period_s, 1.53)
        for n, point in enumerate(points):
            self.assertAlmostEqual(point.angle_rad, math.tau * (n + 0.5) / 30)

    def test_hard_gap_cap_rejects_sparse_large_hole(self):
        builder = self.builder(scan_gap_factor=10)
        for n in range(30):
            if n not in range(10, 15):
                builder.sample(4.5 + (n + 0.5) * 0.05, 0, n, 1)
        self.assertEqual(builder.trigger(6, 0, 5), [])

    def test_configured_minimum_rejects_sparse_closed_sweep(self):
        builder = self.builder(min_scan_points=40)
        for n in range(20):
            builder.sample(4.5 + (n + 0.5) * 1.5 / 20, 0, n, 1)
        self.assertEqual(builder.trigger(6, 0, 5), [])
        self.assertIn("有效测距不足", builder.reason)

    def test_missing_zero_and_severe_period_change_reject_current_scan(self):
        for end, count in [(6, 6), (6.9, 5), (4.4, 5)]:
            builder = self.builder()
            builder.sample(4.55, 0, 1, 1)
            self.assertEqual(builder.trigger(end, 0, count), [])
            self.assertEqual(builder.stable_periods, 0)
            self.assertFalse(builder.samples)

    def test_stop_gate_preserves_rotation_but_requires_whole_stationary_circle(self):
        builder = self.builder()
        builder.begin_after(5.0)
        for n in range(20):
            builder.sample(5.01 + n * 0.04, 0, n, 1)
        self.assertFalse(builder.trigger(6, 0, 5))
        self.assertGreaterEqual(builder.stable_periods, 2)
        for n in range(30):
            builder.sample(6.025 + n * 0.05, 0, n, 1)
        self.assertEqual(len(builder.trigger(7.5, 0, 6)), 30)

    def test_zero_loss_waits_for_next_real_trigger_instead_of_poll_timeout(self):
        receiver = DistanceObservationReceiver({"observation_reorder_s": 0})
        for n in range(4):
            receiver.feed(ReceivedObservation(HardwareObservation("rotation", n + 1, n * 1.5), n * 1.5))
            receiver.poll(n * 1.5)
        for n in range(30):
            t = 4.525 + n * 0.05
            receiver.feed(ReceivedObservation(HardwareObservation("range", n + 1, t, 1), t))
            self.assertFalse(receiver.poll(t))
        anchor = receiver.builder.anchor
        sample_count = len(receiver.builder.samples)
        self.assertFalse(receiver.poll(7))
        self.assertEqual(receiver.builder.anchor, anchor)
        self.assertEqual(len(receiver.builder.samples), sample_count)
        self.assertIn("下一真实零位", receiver.builder.reason)
        receiver.feed(ReceivedObservation(HardwareObservation("rotation", 6, 7.5), 7.5))
        receiver.poll(7.5)
        self.assertEqual(receiver.builder.anchor[2], 6)
        self.assertFalse(receiver.builder.samples)
        self.assertIn("零位不连续", receiver.builder.reason)

    def test_range_timestamp_rollback_drops_only_bad_packet(self):
        receiver = DistanceObservationReceiver({})
        first = HardwareObservation("range", 1, 1.2, 1, raw_timestamp_us=1_200_000)
        bad = HardwareObservation("range", 2, 1.1, 1, raw_timestamp_us=1_100_000)
        self.assertTrue(receiver.feed(ReceivedObservation(first, 1.2)))
        pending_before = list(receiver.pending)
        self.assertFalse(receiver.feed(ReceivedObservation(bad, 1.3)))
        self.assertEqual(receiver.pending, pending_before)
        self.assertEqual(receiver.timestamp_conflicts["range"], 1)
        self.assertNotIn("倒退", receiver.builder.reason)

    def test_rotation_timestamp_rollback_invalidates_angle_baseline(self):
        receiver = DistanceObservationReceiver({})
        receiver.builder.trigger(1, 0, 1)
        receiver.builder.sample(1.1, 0, 800, 1)
        first = HardwareObservation("rotation", 1, 1.2, raw_timestamp_us=1_200_000)
        bad = HardwareObservation("rotation", 2, 1.1, raw_timestamp_us=1_100_000)
        self.assertTrue(receiver.feed(ReceivedObservation(first, 1.2)))
        self.assertFalse(receiver.feed(ReceivedObservation(bad, 1.3)))
        self.assertFalse(receiver.pending)
        self.assertIsNone(receiver.builder.anchor)
        self.assertEqual(receiver.timestamp_conflicts["rotation"], 1)
        self.assertIn("等待新的真实零位", receiver.builder.reason)

    def test_late_range_does_not_reset_angle_baseline(self):
        receiver = DistanceObservationReceiver({})
        receiver.builder.trigger(1, 0, 1)
        receiver.poll(1.5)
        anchor = receiver.builder.anchor
        receiver.feed(ReceivedObservation(HardwareObservation("range", 1, 1.1, 1), 1.6))
        self.assertEqual(receiver.builder.anchor, anchor)
        self.assertEqual(receiver.late, 1)

    def test_timed_preview_updates_during_the_open_sweep(self):
        receiver = DistanceObservationReceiver({"observation_reorder_s": 0})
        for n in range(4):
            receiver.feed(ReceivedObservation(HardwareObservation("rotation", n + 1, n * 1.5), n * 1.5))
            receiver.poll(n * 1.5)
        for n in range(30):
            t = 4.5 + (n + 0.5) * 1.5 / 30
            receiver.feed(ReceivedObservation(HardwareObservation("range", 100 + n, t, 1, pixel=n), t))
            receiver.poll(t)
        self.assertEqual(len(receiver.preview_points), 30)
        receiver.feed(ReceivedObservation(HardwareObservation("rotation", 5, 6.0), 6.0))
        receiver.poll(6.0)
        self.assertEqual(len(receiver.preview_points), 30)
        for n in range(10):
            t = 6.0 + (n + 0.5) * 1.53 / 30
            receiver.feed(ReceivedObservation(HardwareObservation("range", 200 + n, t, 1, pixel=100 + n), t))
            receiver.poll(t)
        self.assertEqual(len(receiver.preview_points), 10)
        self.assertEqual([point.pixel for point in receiver.preview_points], list(range(100, 110)))

    def test_direct_distance_and_pixel_adapter_have_same_standard_output(self):
        clock = ClockEstimate(100, 0.001, 2, 0)
        direct = DeviceObservation("range", 1, 102000000, 0.001, distance=1.0, pixel=800)
        pixel = parse_observation("measurement", "PIX session 1 101999000 102001000 800", "session",
                                  CalibrationModel(p0=700, k=100), {})
        self.assertEqual(direct.normalize(clock, 2.01), pixel.normalize(clock, 2.01))

    def test_invalid_pixel_is_no_return_not_free_space(self):
        raw = parse_observation("measurement", "PIX session 1 1000 2000 -1", "session",
                                CalibrationModel(p0=700, k=100), {})
        self.assertIsNone(raw.distance)
        self.assertEqual(raw.status, "no_return")


if __name__ == "__main__":
    unittest.main()
