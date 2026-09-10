import unittest

from radar_core import CalibrationModel
from sync_resilience import ResilientSynchronizedAcquisition
from synchronized_acquisition import ClockEstimate


class Endpoint:
    def __init__(self):
        self.now = 0.0
        self.messages = []

    def write_line(self, line, on_sent=None, **_kwargs):
        self.messages.append((self.now, line))
        if on_sent is not None:
            on_sent(self.now)
        return True


class SyncResilienceTests(unittest.TestCase):
    def running(self):
        measurement = Endpoint()
        rotation = Endpoint()
        output = []
        acquisition = ResilientSynchronizedAcquisition(
            measurement,
            rotation,
            CalibrationModel(p0=700, k=300),
            {
                "sync_max_age_s": 8.0,
                "sync_interval_s": 0.5,
                "keepalive_interval_s": 5.0,
                "clock_drift_bound_ppm": 500.0,
                "irq_timestamp_uncertainty_ms": 2.0,
            },
            lambda *event: output.append(event),
        )
        acquisition.start(0.0)
        acquisition.state = "running"
        acquisition.clocks = {
            "measurement": ClockEstimate(100.0, 0.0002, 0.0),
            "rotation": ClockEstimate(250.0, 0.0002, 0.0),
        }
        acquisition.last_arrival = {"measurement": 9.0, "rotation": 9.0}
        acquisition.last_valid_range = 9.0
        acquisition.last_valid_rotation = 9.0
        return acquisition, measurement, rotation, output

    def test_stale_maintenance_sync_does_not_stop_live_session(self):
        acquisition, measurement, rotation, output = self.running()
        measurement.now = rotation.now = 9.0
        acquisition.poll(9.0)
        self.assertEqual(acquisition.state, "running")
        self.assertTrue(any(line.startswith("SYNC ") for _, line in measurement.messages))
        self.assertTrue(any(line.startswith("SYNC ") for _, line in rotation.messages))
        self.assertTrue(any(
            kind == "sync_diagnostic" and "继续采集" in str(value)
            for kind, value, _ in output
        ))
        self.assertFalse(any(kind == "sync_error" for kind, _, _ in output))

    def test_incompatible_refresh_rebases_clock_and_discards_only_current_revolution(self):
        acquisition, _measurement, _rotation, output = self.running()
        acquisition.builder.trigger(0.2, 0.0001, 1)
        acquisition.builder.sample(0.3, 0.0001, 800, 0.5)
        token = "runtime-jump"
        acquisition.outstanding["measurement"] = (token, 1.0)
        acquisition.sent_times[("measurement", token)] = 1.0

        acquisition._line(
            "measurement",
            f"SYNC {token} 200000000 200000100",
            1.001,
        )

        self.assertEqual(acquisition.state, "running")
        self.assertGreater(acquisition.clocks["measurement"].version, 0)
        self.assertIsNone(acquisition.builder.anchor)
        self.assertIn("重建时钟基准", acquisition.builder.reason)
        self.assertTrue(any(
            kind == "sync_diagnostic" and "重建时钟基准" in str(value)
            for kind, value, _ in output
        ))
        self.assertFalse(any(kind == "sync_error" for kind, _, _ in output))


if __name__ == "__main__":
    unittest.main()
