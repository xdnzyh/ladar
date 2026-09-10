import unittest

from radar_core import CalibrationModel
from scan_acquisition import DistanceObservationReceiver, TimedSweepBuilder
from synchronized_acquisition import ClockEstimate, SynchronizedAcquisition


class Endpoint:
    def __init__(self):
        self.messages = []

    def write_line(self, line, on_sent=None):
        self.messages.append(line)
        if on_sent:
            on_sent(0.0)
        return True


class RuntimeDiagnosticTests(unittest.TestCase):
    def acquisition(self):
        output = []
        acquisition = SynchronizedAcquisition(
            Endpoint(),
            Endpoint(),
            CalibrationModel(p0=700, k=100),
            {"clock_drift_bound_ppm": 0, "irq_timestamp_uncertainty_ms": 0},
            lambda *event: output.append(event),
        )
        acquisition.start(0.0)
        acquisition.state = "running"
        acquisition.clocks = {
            "measurement": ClockEstimate(0, 0.0001, 0, 0),
            "rotation": ClockEstimate(0, 0.0001, 0, 0),
        }
        acquisition.last_arrival = {"measurement": 10.0, "rotation": 9.2}
        acquisition.last_valid_rotation = 9.2
        return acquisition, output

    def test_runtime_status_exposes_trigger_age_counts_and_last_failure(self):
        acquisition, _ = self.acquisition()
        acquisition.raw_progress["rotation"] = (7, 9_000_000)
        acquisition.last_sequence["rotation"] = 7
        acquisition.builder.anchor = (9.0, 0.001, 7)
        acquisition.builder.period_s = 0.75
        acquisition.builder.samples = [object(), object(), object()]
        acquisition.builder.reason = "扫描中，等待下一真实零位确认"
        acquisition.builder.last_failure_reason = "零位不连续，本圈丢弃"
        acquisition.receiver.accepted = 4
        acquisition.receiver.discarded = 2
        acquisition.receiver.warmup = 1
        acquisition.sync_stats["measurement"]["invalid_timestamp"] = 1
        acquisition.sync_stats["rotation"]["invalid_timestamp"] = 2

        text = acquisition._runtime_status(10.0)

        self.assertIn("TRIG 收/有效/锚点 7/7/7", text)
        self.assertIn("距有效TRIG 0.8s", text)
        self.assertIn("周期 0.750s", text)
        self.assertIn("本圈 3 点", text)
        self.assertIn("完整/丢弃/预热 4/2/1", text)
        self.assertIn("时间异常 测距1 零位2", text)
        self.assertIn("最近失败：零位不连续，本圈丢弃", text)

    def test_runtime_status_without_any_trigger_marks_age_unavailable(self):
        acquisition, _ = self.acquisition()
        acquisition.last_sequence.clear()
        acquisition.raw_progress.clear()
        acquisition.builder.anchor = None
        acquisition.builder.period_s = None

        text = acquisition._runtime_status(10.0)

        self.assertIn("TRIG 收/有效/锚点 —/—/—", text)
        self.assertIn("距有效TRIG —", text)
        self.assertIn("周期 —", text)

    def test_timing_failure_survives_following_range_samples_for_diagnosis(self):
        builder = TimedSweepBuilder({})
        builder.trigger(0.0, 0.0, 1)
        builder.sample(0.2, 0.0, 800, 1.0)
        builder.trigger(1.0, 0.0, 3)

        self.assertEqual(builder.last_failure_reason, "零位不连续，本圈丢弃")

        builder.sample(1.1, 0.0, 800, 1.0)
        self.assertEqual(builder.reason, "扫描中，等待下一真实零位确认")
        self.assertEqual(builder.last_failure_reason, "零位不连续，本圈丢弃")

    def test_receiver_invalidation_keeps_explicit_zero_failure_reason(self):
        receiver = DistanceObservationReceiver({})
        receiver.builder.trigger(0.0, 0.0, 1)
        receiver.builder.sample(0.2, 0.0, 800, 1.0)
        receiver.invalidate("零位超过重排窗口，当前扫描失效")

        self.assertEqual(receiver.builder.reason, "零位超过重排窗口，当前扫描失效")
        self.assertEqual(receiver.builder.last_failure_reason, "零位超过重排窗口，当前扫描失效")


if __name__ == "__main__":
    unittest.main()
