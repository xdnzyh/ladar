import math
import unittest
from dataclasses import replace

from navigation_core import HiddenWorld, VelocityCommand
from runtime_config import build_navigation_engine, resolve_runtime_config
from scan_acquisition import scan_points_from_polar
from virtual_hardware import (
    PRESETS, HardwareObservation, ReceivedObservation, DistanceObservationReceiver,
    HardwareSimulation, VirtualChassis, VirtualCommunicationLink,
)


class HardwareSimulationTests(unittest.TestCase):
    def test_safety_observations_are_only_emitted_while_moving(self):
        simulation = HardwareSimulation(HiddenWorld(), {"simulation_profile": "IDEAL"})
        simulation.advance(1.0)
        self.assertEqual(simulation.last_safety_observations, [])
        simulation.execute(VelocityCommand(forward_mps=0.1, duration_s=0.4))
        simulation.advance(0.1)
        self.assertTrue(simulation.last_safety_observations)

    def test_navigation_recovers_from_ambiguous_corridor_match(self):
        config = resolve_runtime_config(
            "simulation",
            "navigation",
            {"simulation_profile": "IDEAL", "simulation_seed": 20260907},
            prefer_mode_defaults=True,
        )
        simulation = HardwareSimulation(HiddenWorld(seed=20260907), config)
        navigator = build_navigation_engine(config)
        navigator.set_auto(True)
        while simulation.time < 18.0:
            for _, polar_points, _ in simulation.advance(0.05):
                if simulation.time < simulation.resume_at:
                    continue
                command = navigator.process_scan(scan_points_from_polar(polar_points))
                if not command.stopped:
                    simulation.execute(command)
                    navigator.predict_motion(command)
        self.assertGreaterEqual(navigator.grid.update_count, 4)
        self.assertEqual(navigator.rejected_scans, 0)

    def test_ground_truth_distance_is_submillimetre(self):
        world = HiddenWorld()
        self.assertAlmostEqual(world.ray_distance(math.pi / 2, 3), 0.55, delta=0.0001)
        origin = (world.pose.x + 0.013, world.pose.y)
        self.assertAlmostEqual(world.ray_distance(math.pi / 2, 3, origin), 0.537, delta=0.0001)

    def test_bursts_and_reordering_preserve_device_time_angles(self):
        packets = []
        for cycle in range(5):
            packets.append(HardwareObservation("rotation", cycle, float(cycle)))
            for n in range(20):
                stamp = cycle + (n + 0.5) / 20
                packets.append(HardwareObservation("range", cycle * 20 + n, stamp, 1.0))
        def run(burst):
            receiver = DistanceObservationReceiver({"simulation_reorder_s": 0.3})
            received = [ReceivedObservation(p, math.ceil(p.device_timestamp / 0.2) * 0.2 +
                        (0.06 if p.source == "range" else 0.01) if burst else p.device_timestamp)
                        for p in packets]
            output = []
            for item in sorted(received, key=lambda r: r.arrival_time):
                receiver.feed(item)
                if burst:
                    receiver.feed(item)
                output.extend(receiver.poll(item.arrival_time))
            output.extend(receiver.poll(6))
            return output
        ideal, burst = run(False), run(True)
        self.assertTrue(ideal)
        self.assertEqual(ideal, burst)

    def test_old_packet_does_not_modify_closed_sweep(self):
        receiver = DistanceObservationReceiver({})
        receiver.poll(5)
        receiver.feed(ReceivedObservation(HardwareObservation("rotation", 1, 1), 5))
        self.assertEqual(receiver.late, 1)
        self.assertFalse(receiver.pending)

    def test_missing_zero_and_bad_sector_are_rejected(self):
        receiver = DistanceObservationReceiver({"simulation_reorder_s": 0})
        output = []
        for cycle in range(7):
            count = cycle if cycle < 4 else cycle + 1
            receiver.feed(ReceivedObservation(HardwareObservation("rotation", count, cycle), cycle))
            output.extend(receiver.poll(cycle))
            for n in range(20):
                if cycle == 5 and 4 <= n <= 9:
                    continue
                stamp = cycle + (n + 0.5) / 20
                receiver.feed(ReceivedObservation(HardwareObservation("range", cycle * 20 + n, stamp, 1), stamp))
                output.extend(receiver.poll(stamp))
        self.assertGreater(receiver.discarded, 0)
        self.assertFalse(any(count in (5, 7) for count, _, _ in output))

    def test_link_does_not_mutate_measurement_timestamp(self):
        parameters = replace(PRESETS["IDEAL"], range_delay_s=0.2, duplicate_probability=1)
        link = VirtualCommunicationLink(parameters, 1)
        packet = HardwareObservation("range", 1, 0.1, 1.2)
        link.send(packet, 0.1)
        self.assertFalse(list(link.receive(0.29)))
        received = list(link.receive(1))
        self.assertEqual(len(received), 2)
        self.assertTrue(all(r.observation == packet and r.arrival_time > 0.29 for r in received))

    def test_chassis_has_residual_motion_and_fixed_gains(self):
        world = HiddenWorld()
        chassis = VirtualChassis(world, PRESETS["STRESS"], 2)
        gains = chassis.p.wheel_gains
        chassis.execute(VelocityCommand(0.1, duration_s=0.5), 0)
        for n in range(250):
            chassis.step((n + 0.5) * 0.002, 0.002)
        before = world.pose.y
        for n in range(100):
            chassis.step(0.5 + (n + 0.5) * 0.002, 0.002)
        self.assertGreater(world.pose.y, before)
        self.assertEqual(chassis.p.wheel_gains, gains)

    def test_ideal_command_does_not_return_ground_truth(self):
        simulation = HardwareSimulation(HiddenWorld(), {"simulation_profile": "IDEAL"})
        before = simulation._world.pose.y
        self.assertIsNone(simulation.execute(VelocityCommand(0.1, duration_s=0.5)))
        self.assertEqual(simulation._world.pose.y, before)
        simulation.advance(0.5)
        self.assertAlmostEqual(simulation._world.pose.y - before, 0.05, delta=0.0003)

    def test_profiles_are_reproducible_and_use_hardware_path(self):
        for profile in PRESETS:
            config = {"simulation_profile": profile, "simulation_seed": 42}
            a, b = HardwareSimulation(HiddenWorld(), config), HardwareSimulation(HiddenWorld(), config)
            a._world.scan = lambda *args, **kwargs: self.fail("direct scan path")
            first, second = a.advance(12), b.advance(12)
            if profile != "STRESS":
                self.assertTrue(first, profile)
            self.assertGreater(a.sensor.sequence, 100)
            self.assertEqual(first, second)
            self.assertEqual(a.diagnostics(), b.diagnostics())


if __name__ == "__main__":
    unittest.main()
