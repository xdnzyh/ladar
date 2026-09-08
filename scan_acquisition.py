<<<<<<< HEAD
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
import math

from radar_core import PolarPoint


class TimedSweepBuilder:
    def __init__(self, config: dict):
        self.config = config
        self.reset()

    def reset(self):
        self.anchor = None
        self.samples = []
        self.previous_period = None
        self.stable_periods = 0
        self.reason = "等待光电零位"
        self.period_s = None

    def sample(self, timestamp: float, uncertainty: float, pixel: int, distance: float):
        if self.anchor is not None and timestamp >= self.anchor[0]:
            self.samples.append((timestamp, uncertainty, pixel, distance))

    def trigger(self, timestamp: float, uncertainty: float, count: int) -> list[PolarPoint]:
        anchor, samples = self.anchor, self.samples
        self.anchor = (timestamp, uncertainty, count)
        self.samples = []
        if anchor is None:
            return []
        start, start_error, previous_count = anchor
        period = timestamp - start
        if count != previous_count + 1 or not 0.1 <= period <= 60:
            self.previous_period = None
            self.stable_periods = 0
            self.reason = "零位不连续，本圈丢弃"
            return []
        self.period_s = period
        previous_period, self.previous_period = self.previous_period, period
        if previous_period is None or abs(period / previous_period - 1) > float(self.config.get("period_tolerance", 0.05)):
            self.stable_periods = 0
        else:
            self.stable_periods += 1
        if self.stable_periods < 2:
            self.reason = "等待连续稳定转动"
            return []
        lower_period = period - start_error - uncertainty
        if lower_period <= 0:
            self.reason = "校时误差超过旋转周期"
            return []
        result = []
        direction = 1 if self.config.get("clockwise", True) else -1
        offset = math.radians(float(self.config.get("angle_offset_deg", 0)))
        error_limit = float(self.config.get("max_timing_position_error_m", 0.04))
        for sample_time, sample_error, pixel, distance in samples:
            if sample_time - sample_error < start + start_error or sample_time + sample_error >= timestamp - uncertainty:
                continue
            phase = (sample_time - start) / period
            angular_error = math.tau * (sample_error + start_error + uncertainty) / lower_period
            if distance * angular_error > error_limit:
                self.reason = f"时间配准误差超过 {error_limit * 100:g} cm，本圈丢弃"
                return []
            result.append(PolarPoint(distance, offset + direction * math.tau * phase, sample_time, pixel))
        if len(result) < 12:
            self.reason = "本圈有效测距不足 12 点"
            return []
        angles = sorted(point.angle_rad % math.tau for point in result)
        gaps = [b - a for a, b in zip(angles, angles[1:] + [angles[0] + math.tau])]
        if max(gaps) > math.radians(float(self.config.get("max_scan_gap_deg", 25))):
            self.reason = "扫描存在过大的角度空缺，本圈丢弃"
            return []
        self.reason = "完整扫描"
        return result


class EstimatedSweepBuilder:
    def __init__(self, config: dict):
        self.config = config
        self.anchor = None
        self.periods = deque(maxlen=5)
        self.period_s = None
        self.previous_period = None
        self.stable_periods = 0
        self.reason = "等待转速估计"
        self.samples = []
        self.window = None
        self.collect_after = 0.0
        self.sequence = 0
        self.invalidated = 0

    def begin_after(self, timestamp: float):
        self.samples = []
        self.window = None
        self.collect_after = timestamp
        self.reason = "等待停车稳定"

    def trigger(self, timestamp: float, uncertainty: float, count: int):
        previous = self.anchor
        self.anchor = (timestamp, uncertainty, count)
        if previous is None:
            return
        period = timestamp - previous[0]
        valid = count == previous[2] + 1 and 0.1 <= period <= 60
        if valid and self.periods:
            valid = abs(period / (sum(self.periods) / len(self.periods)) - 1) <= float(self.config.get("period_tolerance", 0.05))
        if not valid:
            self.invalidated += self.window is not None
            self.window = None
            self.samples = []
            self.periods.clear()
            self.period_s = None
            self.stable_periods = 0
            self.reason = "零位或转速异常，重新估计"
            return
        self.periods.append(period)
        self.previous_period = period
        self.period_s = sum(self.periods) / len(self.periods)
        self.stable_periods = max(0, len(self.periods) - 1)

    def sample(self, timestamp: float, uncertainty: float, pixel: int, distance: float):
        if timestamp < self.collect_after or self.anchor is None or self.stable_periods < 2:
            return
        if timestamp - self.anchor[0] > self.period_s * 1.5:
            self.reason = "零位更新超时"
            return
        if self.window is None:
            self.window = (timestamp, timestamp + self.period_s, self.anchor, self.period_s,
                           max(self.periods) - min(self.periods))
            self.samples = []
        self.samples.append((timestamp, uncertainty, pixel, distance))
        self.reason = "扫描中"

    def poll(self, timestamp: float):
        if self.window is None or timestamp < self.window[1]:
            return None
        start, end, anchor, period, spread = self.window
        samples, self.samples = self.samples, []
        self.window = None
        self.sequence += 1
        points = []
        direction = 1 if self.config.get("clockwise", True) else -1
        offset = math.radians(float(self.config.get("angle_offset_deg", 0)))
        for stamp, error, pixel, distance in samples:
            if not start <= stamp < end:
                continue
            phase = (stamp - anchor[0]) / period
            angular_error = math.tau * ((error + anchor[1]) / period + abs(phase) * spread / period)
            if distance * angular_error > float(self.config.get("max_timing_position_error_m", 0.04)):
                self.reason = "角度估计误差过大，本圈丢弃"
                return self.sequence, [], period
            points.append(PolarPoint(distance, (offset + direction * math.tau * phase) % math.tau, stamp, pixel))
        if len(points) < 12:
            self.reason = "本圈有效测距不足 12 点"
            return self.sequence, [], period
        angles = sorted(p.angle_rad for p in points)
        gaps = [b - a for a, b in zip(angles, angles[1:] + [angles[0] + math.tau])]
        if max(gaps) > math.radians(float(self.config.get("max_scan_gap_deg", 25))):
            self.reason = "扫描存在过大的角度空缺，本圈丢弃"
            return self.sequence, [], period
        self.reason = "完整扫描"
        return self.sequence, points, period


@dataclass(frozen=True)
class HardwareObservation:
    source: str
    sequence: int
    device_timestamp: float
    distance: float | None = None
    status: str = "ok"
    uncertainty: float = 0.0
    pixel: int = 0


@dataclass(frozen=True)
class ReceivedObservation:
    observation: HardwareObservation
    arrival_time: float


class DistanceObservationReceiver:
    def __init__(self, config: dict):
        self.config = config
        self.builder = EstimatedSweepBuilder(config) if config.get("arbitrary_phase_scans", False) else TimedSweepBuilder(config)
        self.reorder_s = float(config.get("observation_reorder_s", config.get("simulation_reorder_s", 0.3)))
        if not math.isfinite(self.reorder_s) or self.reorder_s < 0:
            raise ValueError("重排等待时间必须是非负有限数")
        self.pending = []
        self.seen = {}
        self.counter = 0
        self.watermark = -math.inf
        self.accepted = self.discarded = self.late = self.duplicates = 0
        self.warmup = 0
        self._preview = deque(maxlen=512)
        self.preview_points = ()

    def reset(self, now: float):
        if isinstance(self.builder, EstimatedSweepBuilder):
            self.builder.begin_after(now)
            return
        previous_period = self.builder.previous_period
        stable_periods = self.builder.stable_periods
        self.builder.reset()
        self.builder.previous_period = previous_period
        self.builder.stable_periods = stable_periods
        self.pending.clear()
        self.seen.clear()
        self.watermark = now

    def feed(self, received: ReceivedObservation):
        packet = received.observation
        timestamp = packet.device_timestamp
        key = packet.source, packet.sequence
        if key in self.seen:
            self.duplicates += 1
            return
        if not math.isfinite(timestamp) or timestamp <= self.watermark:
            self.late += 1
            return
        self.seen[key] = timestamp
        self.counter += 1
        heapq.heappush(self.pending, (timestamp, 0 if packet.source == "rotation" else 1, self.counter, packet))

    def poll(self, now: float):
        cutoff = max(self.watermark, now - self.reorder_s)
        results = []
        def finish_window(timestamp):
            if isinstance(self.builder, EstimatedSweepBuilder):
                result = self.builder.poll(timestamp)
                if result is not None:
                    if result[1]:
                        self.accepted += 1
                        results.append(result)
                    else:
                        self.discarded += 1
        while self.pending and self.pending[0][0] <= cutoff:
            timestamp, _, _, packet = heapq.heappop(self.pending)
            finish_window(timestamp)
            if packet.source == "range":
                distance = packet.distance
                if packet.status in {"ok", "over_range"} and distance is not None and math.isfinite(distance):
                    if float(self.config.get("min_range_m", 0.08)) <= distance <= float(self.config.get("max_range_m", 3.0)):
                        self.builder.sample(timestamp, packet.uncertainty, packet.pixel, distance)
                        if (isinstance(self.builder, EstimatedSweepBuilder) and self.builder.anchor is not None
                                and self.builder.stable_periods >= 2 and self.builder.period_s is not None
                                and 0 <= timestamp - self.builder.anchor[0] <= self.builder.period_s * 1.5):
                            direction = 1 if self.config.get("clockwise", True) else -1
                            angle = math.radians(float(self.config.get("angle_offset_deg", 0))) + direction * math.tau * (timestamp - self.builder.anchor[0]) / self.builder.period_s
                            self._preview.append((timestamp, angle % math.tau, distance))
            elif packet.source == "rotation":
                if isinstance(self.builder, EstimatedSweepBuilder):
                    before = self.builder.invalidated
                    self.builder.trigger(timestamp, packet.uncertainty, packet.sequence)
                    self.discarded += self.builder.invalidated - before
                    if self.builder.stable_periods < 2:
                        self.warmup += 1
                    continue
                had_anchor = self.builder.anchor is not None
                points = self.builder.trigger(timestamp, packet.uncertainty, packet.sequence)
                if points:
                    self.accepted += 1
                    results.append((packet.sequence, points, self.builder.period_s))
                elif had_anchor:
                    if self.builder.reason == "等待连续稳定转动":
                        self.warmup += 1
                    else:
                        self.discarded += 1
        finish_window(cutoff)
        period = self.builder.period_s or float(self.config.get("radar_period_s", 1.5))
        while self._preview and self._preview[0][0] < cutoff - period:
            self._preview.popleft()
        self.preview_points = tuple(self._preview)
        self.watermark = cutoff
        self.seen = {key: timestamp for key, timestamp in self.seen.items() if timestamp > cutoff}
        return results


=======
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
import math

from radar_core import PolarPoint


class TimedSweepBuilder:
    def __init__(self, config: dict):
        self.config = config
        self.reset()

    def reset(self):
        self.anchor = None
        self.samples = []
        self.previous_period = None
        self.stable_periods = 0
        self.reason = "等待光电零位"
        self.period_s = None

    def sample(self, timestamp: float, uncertainty: float, pixel: int, distance: float):
        if self.anchor is not None and timestamp >= self.anchor[0]:
            self.samples.append((timestamp, uncertainty, pixel, distance))

    def trigger(self, timestamp: float, uncertainty: float, count: int) -> list[PolarPoint]:
        anchor, samples = self.anchor, self.samples
        self.anchor = (timestamp, uncertainty, count)
        self.samples = []
        if anchor is None:
            return []
        start, start_error, previous_count = anchor
        period = timestamp - start
        if count != previous_count + 1 or not 0.1 <= period <= 60:
            self.previous_period = None
            self.stable_periods = 0
            self.reason = "零位不连续，本圈丢弃"
            return []
        self.period_s = period
        previous_period, self.previous_period = self.previous_period, period
        if previous_period is None or abs(period / previous_period - 1) > float(self.config.get("period_tolerance", 0.05)):
            self.stable_periods = 0
        else:
            self.stable_periods += 1
        if self.stable_periods < 2:
            self.reason = "等待连续稳定转动"
            return []
        lower_period = period - start_error - uncertainty
        if lower_period <= 0:
            self.reason = "校时误差超过旋转周期"
            return []
        result = []
        direction = 1 if self.config.get("clockwise", True) else -1
        offset = math.radians(float(self.config.get("angle_offset_deg", 0)))
        error_limit = float(self.config.get("max_timing_position_error_m", 0.04))
        for sample_time, sample_error, pixel, distance in samples:
            if sample_time - sample_error < start + start_error or sample_time + sample_error >= timestamp - uncertainty:
                continue
            phase = (sample_time - start) / period
            angular_error = math.tau * (sample_error + start_error + uncertainty) / lower_period
            if distance * angular_error > error_limit:
                self.reason = f"时间配准误差超过 {error_limit * 100:g} cm，本圈丢弃"
                return []
            result.append(PolarPoint(distance, offset + direction * math.tau * phase, sample_time, pixel))
        if len(result) < 12:
            self.reason = "本圈有效测距不足 12 点"
            return []
        angles = sorted(point.angle_rad % math.tau for point in result)
        gaps = [b - a for a, b in zip(angles, angles[1:] + [angles[0] + math.tau])]
        if max(gaps) > math.radians(float(self.config.get("max_scan_gap_deg", 25))):
            self.reason = "扫描存在过大的角度空缺，本圈丢弃"
            return []
        self.reason = "完整扫描"
        return result


class EstimatedSweepBuilder:
    def __init__(self, config: dict):
        self.config = config
        self.anchor = None
        self.periods = deque(maxlen=5)
        self.period_s = None
        self.previous_period = None
        self.stable_periods = 0
        self.reason = "等待转速估计"
        self.samples = []
        self.window = None
        self.collect_after = 0.0
        self.sequence = 0
        self.invalidated = 0

    def begin_after(self, timestamp: float):
        self.samples = []
        self.window = None
        self.collect_after = timestamp
        self.reason = "等待停车稳定"

    def trigger(self, timestamp: float, uncertainty: float, count: int):
        previous = self.anchor
        self.anchor = (timestamp, uncertainty, count)
        if previous is None:
            return
        period = timestamp - previous[0]
        valid = count == previous[2] + 1 and 0.1 <= period <= 60
        if valid and self.periods:
            valid = abs(period / (sum(self.periods) / len(self.periods)) - 1) <= float(self.config.get("period_tolerance", 0.05))
        if not valid:
            self.invalidated += self.window is not None
            self.window = None
            self.samples = []
            self.periods.clear()
            self.period_s = None
            self.stable_periods = 0
            self.reason = "零位或转速异常，重新估计"
            return
        self.periods.append(period)
        self.previous_period = period
        self.period_s = sum(self.periods) / len(self.periods)
        self.stable_periods = max(0, len(self.periods) - 1)

    def sample(self, timestamp: float, uncertainty: float, pixel: int, distance: float):
        if timestamp < self.collect_after or self.anchor is None or self.stable_periods < 2:
            return
        if timestamp - self.anchor[0] > self.period_s * 1.5:
            self.reason = "零位更新超时"
            return
        if self.window is None:
            self.window = (timestamp, timestamp + self.period_s, self.anchor, self.period_s,
                           max(self.periods) - min(self.periods))
            self.samples = []
        self.samples.append((timestamp, uncertainty, pixel, distance))
        self.reason = "扫描中"

    def poll(self, timestamp: float):
        if self.window is None or timestamp < self.window[1]:
            return None
        start, end, anchor, period, spread = self.window
        samples, self.samples = self.samples, []
        self.window = None
        self.sequence += 1
        points = []
        direction = 1 if self.config.get("clockwise", True) else -1
        offset = math.radians(float(self.config.get("angle_offset_deg", 0)))
        for stamp, error, pixel, distance in samples:
            if not start <= stamp < end:
                continue
            phase = (stamp - anchor[0]) / period
            angular_error = math.tau * ((error + anchor[1]) / period + abs(phase) * spread / period)
            if distance * angular_error > float(self.config.get("max_timing_position_error_m", 0.04)):
                self.reason = "角度估计误差过大，本圈丢弃"
                return self.sequence, [], period
            points.append(PolarPoint(distance, (offset + direction * math.tau * phase) % math.tau, stamp, pixel))
        if len(points) < 12:
            self.reason = "本圈有效测距不足 12 点"
            return self.sequence, [], period
        angles = sorted(p.angle_rad for p in points)
        gaps = [b - a for a, b in zip(angles, angles[1:] + [angles[0] + math.tau])]
        if max(gaps) > math.radians(float(self.config.get("max_scan_gap_deg", 25))):
            self.reason = "扫描存在过大的角度空缺，本圈丢弃"
            return self.sequence, [], period
        self.reason = "完整扫描"
        return self.sequence, points, period


@dataclass(frozen=True)
class HardwareObservation:
    source: str
    sequence: int
    device_timestamp: float
    distance: float | None = None
    status: str = "ok"
    uncertainty: float = 0.0
    pixel: int = 0


@dataclass(frozen=True)
class ReceivedObservation:
    observation: HardwareObservation
    arrival_time: float


class DistanceObservationReceiver:
    def __init__(self, config: dict):
        self.config = config
        self.builder = EstimatedSweepBuilder(config) if config.get("arbitrary_phase_scans", False) else TimedSweepBuilder(config)
        self.reorder_s = float(config.get("observation_reorder_s", config.get("simulation_reorder_s", 0.3)))
        if not math.isfinite(self.reorder_s) or self.reorder_s < 0:
            raise ValueError("重排等待时间必须是非负有限数")
        self.pending = []
        self.seen = {}
        self.counter = 0
        self.watermark = -math.inf
        self.accepted = self.discarded = self.late = self.duplicates = 0
        self.warmup = 0
        self._preview = deque(maxlen=512)
        self.preview_points = ()

    def reset(self, now: float):
        if isinstance(self.builder, EstimatedSweepBuilder):
            self.builder.begin_after(now)
            return
        previous_period = self.builder.previous_period
        stable_periods = self.builder.stable_periods
        self.builder.reset()
        self.builder.previous_period = previous_period
        self.builder.stable_periods = stable_periods
        self.pending.clear()
        self.seen.clear()
        self.watermark = now

    def feed(self, received: ReceivedObservation):
        packet = received.observation
        timestamp = packet.device_timestamp
        key = packet.source, packet.sequence
        if key in self.seen:
            self.duplicates += 1
            return
        if not math.isfinite(timestamp) or timestamp <= self.watermark:
            self.late += 1
            return
        self.seen[key] = timestamp
        self.counter += 1
        heapq.heappush(self.pending, (timestamp, 0 if packet.source == "rotation" else 1, self.counter, packet))

    def poll(self, now: float):
        cutoff = max(self.watermark, now - self.reorder_s)
        results = []
        def finish_window(timestamp):
            if isinstance(self.builder, EstimatedSweepBuilder):
                result = self.builder.poll(timestamp)
                if result is not None:
                    if result[1]:
                        self.accepted += 1
                        results.append(result)
                    else:
                        self.discarded += 1
        while self.pending and self.pending[0][0] <= cutoff:
            timestamp, _, _, packet = heapq.heappop(self.pending)
            finish_window(timestamp)
            if packet.source == "range":
                distance = packet.distance
                if packet.status in {"ok", "over_range"} and distance is not None and math.isfinite(distance):
                    if float(self.config.get("min_range_m", 0.08)) <= distance <= float(self.config.get("max_range_m", 3.0)):
                        self.builder.sample(timestamp, packet.uncertainty, packet.pixel, distance)
                        if (isinstance(self.builder, EstimatedSweepBuilder) and self.builder.anchor is not None
                                and self.builder.stable_periods >= 2 and self.builder.period_s is not None
                                and 0 <= timestamp - self.builder.anchor[0] <= self.builder.period_s * 1.5):
                            direction = 1 if self.config.get("clockwise", True) else -1
                            angle = math.radians(float(self.config.get("angle_offset_deg", 0))) + direction * math.tau * (timestamp - self.builder.anchor[0]) / self.builder.period_s
                            self._preview.append((timestamp, angle % math.tau, distance))
            elif packet.source == "rotation":
                if isinstance(self.builder, EstimatedSweepBuilder):
                    before = self.builder.invalidated
                    self.builder.trigger(timestamp, packet.uncertainty, packet.sequence)
                    self.discarded += self.builder.invalidated - before
                    if self.builder.stable_periods < 2:
                        self.warmup += 1
                    continue
                had_anchor = self.builder.anchor is not None
                points = self.builder.trigger(timestamp, packet.uncertainty, packet.sequence)
                if points:
                    self.accepted += 1
                    results.append((packet.sequence, points, self.builder.period_s))
                elif had_anchor:
                    if self.builder.reason == "等待连续稳定转动":
                        self.warmup += 1
                    else:
                        self.discarded += 1
        finish_window(cutoff)
        period = self.builder.period_s or float(self.config.get("radar_period_s", 1.5))
        while self._preview and self._preview[0][0] < cutoff - period:
            self._preview.popleft()
        self.preview_points = tuple(self._preview)
        self.watermark = cutoff
        self.seen = {key: timestamp for key, timestamp in self.seen.items() if timestamp > cutoff}
        return results



>>>>>>> 2e7b899d23a3036e88a61b6bd655737df23da0c2
