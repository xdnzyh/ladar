import math
import unittest

from navigation_core import HiddenWorld, VelocityCommand
from synchronized_acquisition import EstimatedSweepBuilder
from virtual_hardware import HardwareSimulation, DistanceObservationReceiver, HardwareObservation, ReceivedObservation


class EstimatedSweepTests(unittest.TestCase):
    def test_live_preview_updates_before_complete_scan(self):
        simulation = HardwareSimulation(HiddenWorld(), {"simulation_profile": "IDEAL"})
        self.assertFalse(simulation.advance(6.5))
        first = simulation.receiver.preview_points
        self.assertTrue(first)
        self.assertFalse(simulation.advance(0.15))
        second = simulation.receiver.preview_points
        self.assertGreater(second[-1][0], first[-1][0])
        self.assertNotEqual(second[-1][1], first[-1][1])
        self.assertEqual(simulation.receiver.accepted, 0)

    def test_preview_updates_during_motion_without_mapping_scan(self):
        simulation = HardwareSimulation(HiddenWorld(), {"simulation_profile": "IDEAL"})
        simulation.advance(8)
        first = simulation.receiver.preview_points[-1][0]
        simulation.execute(VelocityCommand(forward_mps=0.1, duration_s=1))
        self.assertFalse(simulation.advance(0.8))
        self.assertGreater(simulation.receiver.preview_points[-1][0], first)

    def test_preview_expires_old_revolutions(self):
        simulation = HardwareSimulation(HiddenWorld(), {"simulation_profile": "IDEAL"})
        simulation.advance(12)
        preview = simulation.receiver.preview_points
        self.assertLessEqual(len(preview), 31)
        self.assertLessEqual(preview[-1][0] - preview[0][0], 1.5 + 1e-9)

    def builder(self, **config):
        builder = EstimatedSweepBuilder(config)
        for count in range(4):
            builder.trigger(count * 1.5, 0, count)
        return builder

    def test_scan_starts_between_zero_events_and_finishes_without_next_zero(self):
        builder = self.builder()
        builder.begin_after(4.7)
        for index in range(30):
            builder.sample(4.75 + index * 0.05, 0, 0, 1)
        self.assertIsNone(builder.poll(6.249))
        _, points, period = builder.poll(6.25)
        self.assertEqual(len(points), 30)
        self.assertAlmostEqual(period, 1.5)
        self.assertAlmostEqual(points[0].angle_rad, math.pi / 3)

    def test_waiting_discards_ranges_but_preserves_period_history(self):
        builder = self.builder()
        builder.begin_after(6.2)
        builder.sample(5, 0, 0, 1)
        builder.trigger(6, 0, 4)
        self.assertFalse(builder.samples)
        self.assertEqual(len(builder.periods), 4)
        builder.sample(6.21, 0, 0, 1)
        self.assertAlmostEqual(builder.window[0], 6.21)

    def test_rotation_discontinuity_discards_current_window(self):
        builder = self.builder()
        builder.sample(4.75, 0, 0, 1)
        builder.trigger(6, 0, 5)
        self.assertIsNone(builder.window)
        self.assertEqual(builder.invalidated, 1)
        self.assertEqual(builder.stable_periods, 0)

    def test_direction_and_offset_use_observed_phase(self):
        builder = self.builder(clockwise=False, angle_offset_deg=15)
        for index in range(30):
            builder.sample(4.75 + index * 0.05, 0, 0, 1)
        points = builder.poll(6.25)[1]
        self.assertAlmostEqual(math.degrees(points[0].angle_rad), 315)

    def test_missing_sector_is_not_accepted_as_complete(self):
        builder = self.builder()
        for index in range(30):
            if not 8 <= index <= 14:
                builder.sample(4.75 + index * 0.05, 0, 0, 1)
        self.assertEqual(builder.poll(6.25)[1], [])

    def test_burst_arrivals_preserve_estimated_angles(self):
        packets = []
        for cycle in range(6):
            packets.append(HardwareObservation("rotation", cycle, float(cycle)))
            for index in range(20):
                packets.append(HardwareObservation("range", cycle * 20 + index, cycle + (index + 0.5) / 20, 1))
        def receive(burst):
            receiver = DistanceObservationReceiver({"arbitrary_phase_scans": True})
            received = [ReceivedObservation(p, math.ceil(p.device_timestamp / 0.2) * 0.2
                        + (0.02 if p.source == "rotation" else 0.04) if burst else p.device_timestamp) for p in packets]
            result = []
            for packet in sorted(received, key=lambda p: p.arrival_time):
                receiver.feed(packet)
                result.extend(receiver.poll(packet.arrival_time))
            result.extend(receiver.poll(7))
            return result
        normal, burst = receive(False), receive(True)
        self.assertTrue(normal)
        self.assertEqual(normal, burst)

    def test_hardware_sampling_resumes_200ms_after_command_end(self):
        simulation = HardwareSimulation(HiddenWorld(), {"simulation_profile": "IDEAL"})
        simulation.advance(8.1)
        simulation.execute(VelocityCommand(forward_mps=0.1, duration_s=0.4))
        resume_at = simulation.resume_at
        self.assertAlmostEqual(resume_at, 8.7)
        sweeps = simulation.advance(2.5)
        self.assertTrue(sweeps)
        first_stamp = sweeps[0][1][0].timestamp
        self.assertGreaterEqual(first_stamp, resume_at - 1e-9)
        self.assertLess(first_stamp, resume_at + 0.051)
        self.assertLess(first_stamp, 9.0)


if __name__ == "__main__":
    unittest.main()

