from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from chassis_controller import ChassisState
from mapping_runtime import MappingRequest, MappingResult, MappingRuntime
from motion_safety import MotionSafetyGuard
from navigation_core import NavigationEngine, OccupancyGrid, VelocityCommand
from runtime_config import resolve_runtime_config
from simulation_chassis import SimulationChassis


class SimulationChassisTests(unittest.TestCase):
    def test_full_transaction_and_lost_feedback_apply_one_prior_and_wait_for_scan(self):
        for drop_ack, drop_result in ((False, False), (True, False), (True, True)):
            with self.subTest(drop_ack=drop_ack, drop_result=drop_result):
                config = resolve_runtime_config("simulation", "navigation", {})
                engine = NavigationEngine(OccupancyGrid(80, 80, 0.04))
                runtime = MappingRuntime(engine)
                simulation = SimpleNamespace(time=0.0, execute=Mock(), stop=Mock(), receiver=Mock())
                try:
                    chassis = SimulationChassis(simulation, config, runtime, MotionSafetyGuard(config),
                                                drop_ack=drop_ack, drop_result=drop_result)
                    chassis.execute(VelocityCommand(forward_mps=0.1, duration_s=1))
                    for i in range(1, 301):
                        simulation.time = i * 0.05
                        chassis.poll()
                        if chassis.controller.state == ChassisState.WAITING_SCAN:
                            break
                    self.assertIsNone(chassis.failure)
                    self.assertEqual(chassis.controller.state, ChassisState.WAITING_SCAN)
                    self.assertEqual(chassis.priors_applied, 1)
                    self.assertAlmostEqual(engine.pose.y, chassis.command.forward_mps * chassis.command.duration_s)
                    self.assertEqual(chassis.diagnostics()["chassis_moves"], 1)
                    simulation.execute.assert_called_once()
                    if drop_result:
                        self.assertGreater(chassis.diagnostics()["chassis_result_queries"], 0)
                    snapshot = runtime._snapshot_locked("simulation", 1, "navigation", VelocityCommand(), True)
                    request = MappingRequest(runtime.generation, "simulation", 1,
                                             chassis.scan_after, chassis.scan_after + 1, "navigation", 0, ())
                    mapped = MappingResult(request, snapshot, VelocityCommand())
                    self.assertFalse(chassis.accept_scan(replace(mapped, snapshot=replace(snapshot, scan_accepted=False))))
                    self.assertFalse(chassis.accept_scan(replace(mapped, request=replace(request, scan_start_s=0))))
                    self.assertTrue(chassis.accept_scan(mapped))
                    self.assertEqual(chassis.controller.state, ChassisState.IDLE)
                    chassis.controller.feed_data(chassis.cached_result, simulation.time, chassis.generation)
                    self.assertEqual(chassis.priors_applied, 1)
                finally:
                    runtime.stop()


if __name__ == "__main__":
    unittest.main()
