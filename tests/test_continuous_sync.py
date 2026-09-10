import heapq
import math
import unittest

from radar_core import CalibrationModel
from synchronized_acquisition import ClockEstimate, SynchronizedAcquisition


class Endpoint:
    def __init__(self):
        self.now = 0
        self.messages = []

    def write_line(self, line, on_sent=None):
        self.messages.append((self.now, line))
        if on_sent:
            on_sent(self.now)
        return True


class ContinuousSyncTests(unittest.TestCase):
    def test_refresh_contains_true_offset_with_drift_and_asymmetric_delay(self):
        clock = ClockEstimate(100, 0.002, 0, 500)
        for n in range(1, 200):
            t1 = n * 0.5
            t2 = 100 + (t1 + 0.0002) * 1.0004
            t3 = t2 + 0.0001 * 1.0004
            t4 = t1 + 0.0013
            new = ClockEstimate.exchange(t1, round(t2 * 1e6), round(t3 * 1e6), t4)
            clock = clock.updated(new)
            sample_host = t4 + 0.4
            mapped, error = clock.map((100 + sample_host * 1.0004) * 1e6, sample_host + 0.1)
            self.assertLessEqual(abs(mapped - sample_host), error)
            self.assertLess(error, 0.003)

    def test_bad_clock_jump_is_not_hidden_by_widening_threshold(self):
        with self.assertRaises(ValueError):
            ClockEstimate(0, 0.001, 0).updated(ClockEstimate(1, 0.001, 1))

    def running(self):
        endpoints = {source: Endpoint() for source in ("measurement", "rotation")}
        output = []
        acquisition = SynchronizedAcquisition(endpoints["measurement"], endpoints["rotation"],
                                              CalibrationModel(p0=700, k=300),
                                              {"irq_timestamp_uncertainty_ms": 2},
                                              lambda *event: output.append(event))
        acquisition.start(0)
        acquisition.state = "running"
        acquisition.clocks = {"measurement": ClockEstimate(100, 0.0002, 0),
                              "rotation": ClockEstimate(250, 0.0002, 0)}
        acquisition.last_arrival = {source: 0 for source in endpoints}
        return acquisition, endpoints, output

    def test_90_seconds_online_sync_keeps_rotation_and_bounded_error(self):
        acquisition, endpoints, output = self.running()
        rates = {"measurement": 1.0004, "rotation": 0.9997}
        offsets = {"measurement": 100, "rotation": 250}
        positions = {source: len(endpoint.messages) for source, endpoint in endpoints.items()}
        pending = []
        sequence = 0
        for step in range(1, 3601):
            now = step * 0.025
            for endpoint in endpoints.values():
                endpoint.now = now
            if step % 2 == 1:
                sequence += 1
                begin = round((offsets["measurement"] + (now - 0.0001) * rates["measurement"]) * 1e6)
                end = round((offsets["measurement"] + (now + 0.0001) * rates["measurement"]) * 1e6)
                heapq.heappush(pending, (now + 0.004, "measurement", f"PIX {acquisition.session} {sequence} {begin} {end} 800"))
            if step % 60 == 0:
                tick = round((offsets["rotation"] + now * rates["rotation"]) * 1e6)
                heapq.heappush(pending, (now + 0.003, "rotation", f"TRIG {acquisition.session} {step // 60} {tick}"))
            while pending and pending[0][0] <= now:
                arrival, source, line = heapq.heappop(pending)
                acquisition.feed(source, (line + "\n").encode(), arrival)
            acquisition.poll(now)
            self.assertEqual(acquisition.state, "running", output[-3:])
            for source, endpoint in endpoints.items():
                for sent, line in endpoint.messages[positions[source]:]:
                    if line.startswith("SYNC "):
                        token = line.split()[1]
                        t2 = round((offsets[source] + (sent + 0.0002) * rates[source]) * 1e6)
                        t3 = round((offsets[source] + (sent + 0.0003) * rates[source]) * 1e6)
                        heapq.heappush(pending, (sent + 0.0005, source, f"SYNC {token} {t2} {t3}"))
                positions[source] = len(endpoint.messages)
        sweeps = [event for event in output if event[0] == "sync_sweep"]
        self.assertGreater(len(sweeps), 45, (acquisition.builder.reason, acquisition.builder.rejected_points, acquisition.receiver.discarded, acquisition.receiver.late, acquisition.clocks, acquisition.builder.anchor, acquisition.builder.periods, len(acquisition.builder.samples), output[-3:]))
        self.assertGreater(sweeps[-1][2], 85)
        self.assertLess(acquisition.receiver.discarded, 5)
        for source, endpoint in endpoints.items():
            self.assertTrue(any(line == "PING" for _, line in endpoint.messages))
            self.assertFalse(any(line in {"OFF", "STOP"} and sent > 0 for sent, line in endpoint.messages))
            self.assertLess(acquisition.clocks[source].map((offsets[source] + 90 * rates[source]) * 1e6, 90)[1], 0.002)
        self.assertFalse(any(kind == "sync_expired" for kind, _, _ in output))

    def test_missing_refresh_fails_safe_without_accepting_stale_clock_forever(self):
        acquisition, endpoints, output = self.running()
        acquisition.last_arrival = {source: 9 for source in endpoints}
        acquisition.poll(9)
        self.assertEqual(acquisition.state, "running")
        acquisition.poll(19)
        self.assertEqual(acquisition.state, "stopped")
        self.assertTrue(any(kind == "sync_error" and "校时超时" in value for kind, value, _ in output))

    def test_late_but_fresh_range_still_reaches_safety_layer(self):
        acquisition, endpoints, output = self.running()
        acquisition.receiver.poll(1.0)
        acquisition.feed("measurement", f"PIX {acquisition.session} 1 100600000 100600000 800\n".encode(), 1.0)
        acquisition.poll(1.0)
        self.assertEqual(acquisition.receiver.late, 1)
        self.assertTrue(any(kind == "sync_observation" for kind, _, _ in output))

    def test_raw_measurement_timestamp_rollback_drops_one_packet_and_recovers(self):
        acquisition, endpoints, output = self.running()
        acquisition.last_arrival["measurement"] = 0.6
        acquisition.feed("measurement", f"PIX {acquisition.session} 1 100500000 100500000 800\n".encode(), 0.6)
        acquisition.poll(0.6)
        acquisition.feed("measurement", f"PIX {acquisition.session} 2 100400000 100400000 800\n".encode(), 0.61)
        acquisition.poll(0.61)
        self.assertEqual(acquisition.state, "running")
        self.assertFalse(any(kind == "sync_error" and "倒退" in value for kind, value, _ in output))
        self.assertEqual(acquisition.raw_progress["measurement"], (1, 100500000.0))
        self.assertGreaterEqual(acquisition.sync_stats["measurement"]["invalid_timestamp"], 1)
        acquisition.feed("measurement", f"PIX {acquisition.session} 3 100700000 100700000 800\n".encode(), 0.7)
        acquisition.poll(0.7)
        self.assertEqual(acquisition.raw_progress["measurement"], (3, 100700000.0))

    def test_raw_rotation_timestamp_rollback_resets_scan_but_keeps_session(self):
        acquisition, endpoints, output = self.running()
        acquisition.last_arrival["measurement"] = 0.6
        acquisition.feed("rotation", f"TRIG {acquisition.session} 1 250500000\n".encode(), 0.6)
        acquisition.poll(0.6)
        acquisition.feed("rotation", f"TRIG {acquisition.session} 2 250400000\n".encode(), 0.61)
        acquisition.poll(0.61)
        self.assertEqual(acquisition.state, "running")
        self.assertFalse(any(kind == "sync_error" and "倒退" in value for kind, value, _ in output))
        self.assertIsNone(acquisition.builder.anchor)
        self.assertIn("等待新的真实零位", acquisition.builder.reason)

    def test_observations_remain_immutable_after_online_model_update(self):
        acquisition, endpoints, output = self.running()
        acquisition.feed("measurement", f"PIX {acquisition.session} 1 100500000 100500000 800\n".encode(), 0.51)
        acquisition.poll(0.51)
        packet = next(value[1] for kind, value, _ in output if kind == "sync_observation")
        stamp, error = packet.device_timestamp, packet.uncertainty
        acquisition.clocks["measurement"] = acquisition.clocks["measurement"].updated(
            ClockEstimate(100.0002, 0.0002, 0.6))
        self.assertEqual(packet.device_timestamp, stamp)
        self.assertEqual(packet.uncertainty, error)

    def test_malformed_range_and_bad_range_sequence_drop_only_that_packet(self):
        acquisition, endpoints, output = self.running()
        acquisition.builder.trigger(0, 0, 1)
        acquisition.builder.sample(0.2, 0, 800, 1)
        for payload in ("PIX {session} 0 100300000 100300000 800", "PIX {session} 2 100400000 100300000 800"):
            acquisition.feed("measurement", (payload.format(session=acquisition.session) + "\n").encode(), 0.4)
        acquisition.poll(0.4)
        self.assertEqual(len(acquisition.builder.samples), 1)
        self.assertEqual(acquisition.builder.anchor[2], 1)
        self.assertEqual(acquisition.state, "running")

    def test_invalid_clock_budget_is_rejected(self):
        for drift in (-1, math.nan, math.inf, 1000000):
            with self.assertRaises(ValueError):
                ClockEstimate(0, 0, 0, drift)

    def test_old_probe_response_does_not_replace_clock(self):
        acquisition, endpoints, output = self.running()
        old = acquisition.clocks["measurement"]
        acquisition.feed("measurement", b"SYNC stale-session 100001000 100002000\n", 0.01)
        acquisition.poll(0.02)
        self.assertEqual(acquisition.clocks["measurement"], old)


if __name__ == "__main__":
    unittest.main()
