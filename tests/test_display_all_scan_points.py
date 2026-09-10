import unittest

from scan_acquisition import DistanceObservationReceiver, HardwareObservation, ReceivedObservation, TimedSweepBuilder


class DisplayAllScanPointsTests(unittest.TestCase):
    def test_sparse_scan_reports_distance_dependent_timing_rejections(self):
        builder = TimedSweepBuilder({"min_scan_points": 40})
        builder.trigger(0, .02, 1)
        builder.trigger(.8, .02, 2)
        for n in range(80):
            builder.sample(.8 + (n + .5) * .01, .02, 1000, .25 if n % 2 else .5)
        builder.trigger(1.6, .02, 3)
        counts = builder.last_quality_counts
        self.assertEqual(counts["received"], 80)
        self.assertGreater(counts["timing_position"], 0)
        self.assertGreater(counts["boundary"], 0)
        self.assertLess(counts["kept"], 50)
        self.assertEqual(sum(counts[k] for k in ("invalid", "boundary", "timing_position", "kept")), 80)
        self.assertEqual(len(builder.last_display_points), 80)

    def test_builder_keeps_low_confidence_points_for_display_only(self):
        builder = TimedSweepBuilder({"max_timing_position_error_m": 0.04})
        for n in range(4):
            builder.trigger(n * 1.5, 0.0, n + 1)

        for n in range(30):
            t = 4.5 + (n + 0.5) * 1.5 / 30
            builder.sample(t, 0.05, n, 1.0)

        formal = builder.trigger(6.0, 0.0, 5)

        self.assertEqual(formal, [])
        self.assertEqual(len(builder.last_display_points), 30)
        self.assertLess(len(builder.last_closed_points), len(builder.last_display_points))
        self.assertEqual([point.pixel for point in builder.last_display_points], list(range(30)))

    def test_receiver_preview_shows_rejected_closed_sweep(self):
        receiver = DistanceObservationReceiver({
            "observation_reorder_s": 0.0,
            "max_timing_position_error_m": 0.04,
        })

        for n in range(4):
            t = n * 1.5
            receiver.feed(ReceivedObservation(
                HardwareObservation("rotation", n + 1, t), t
            ))
            receiver.poll(t)

        for n in range(30):
            t = 4.5 + (n + 0.5) * 1.5 / 30
            receiver.feed(ReceivedObservation(
                HardwareObservation(
                    "range",
                    100 + n,
                    t,
                    1.0,
                    uncertainty=0.05,
                    pixel=n,
                ),
                t,
            ))
            receiver.poll(t)

        self.assertEqual(len(receiver.preview_points), 30)

        receiver.feed(ReceivedObservation(
            HardwareObservation("rotation", 5, 6.0), 6.0
        ))
        result = receiver.poll(6.0)

        self.assertFalse(result.formal_scans)
        self.assertEqual(len(receiver.preview_points), 30)
        self.assertEqual([point.pixel for point in receiver.preview_points], list(range(30)))


if __name__ == "__main__":
    unittest.main()
