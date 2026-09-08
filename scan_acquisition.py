from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
import math
from statistics import median

from radar_core import PolarPoint


def scan_complete(points, config):
    if len(points) < int(config.get("min_scan_points", 12)):
        return False, "本圈有效测距不足"
    angles = sorted({p.angle_rad % math.tau for p in points})
    if len(angles) < int(config.get("min_scan_points", 12)):
        return False, "本圈有效方位不足"
    gaps = [b - a for a, b in zip(angles, angles[1:] + [angles[0] + math.tau])]
    normal = median(gaps)
    limit = min(math.radians(float(config.get("scan_gap_hard_limit_deg", 45))),
                max(math.radians(float(config.get("max_scan_gap_deg", 25))),
                    normal * float(config.get("scan_gap_factor", 2.5))))
    if max(gaps) > limit + 1e-9:
        return False, "扫描存在过大的角度空缺，本圈丢弃"
    return True, "完整扫描"


class TimedSweepBuilder:
    def __init__(self, config: dict):
        self.config = config
        self.collect_after = -math.inf
        self.reset()

    def reset(self):
        self.anchor = None
        self.samples = []
        self.previous_period = None
        self.periods = deque(maxlen=5)
        self.stable_periods = 0
        self.reason = "等待光电零位"
        self.period_s = None
        self.rejected_points = 0
        self.last_closed_points = []
        self.last_closed_period = None

    def begin_after(self, timestamp):
        self.samples.clear()
        self.collect_after = timestamp
        self.reason = "等待停车后完整零位圈"

    def sample(self, timestamp, uncertainty, pixel, distance, is_echo=True):
        if (self.anchor is not None and self.anchor[0] - self.anchor[1] >= self.collect_after
                and timestamp >= self.anchor[0]):
            self.samples.append((timestamp, uncertainty, pixel, distance, bool(is_echo)))

    def trigger(self, timestamp, uncertainty, count):
        self.last_closed_points = []
        self.last_closed_period = None
        anchor, samples = self.anchor, self.samples
        self.anchor = (timestamp, uncertainty, count)
        self.samples = []
        if anchor is None:
            return []
        start, start_error, previous_count = anchor
        period = timestamp - start
        if count != previous_count + 1 or not 0.1 <= period <= 60:
            self.reset()
            self.anchor = (timestamp, uncertainty, count)
            self.reason = "零位不连续，本圈丢弃"
            return []
        previous = self.previous_period
        self.previous_period = self.period_s = period
        if previous is not None and abs(period / previous - 1) > float(self.config.get("period_tolerance", 0.05)):
            self.periods.clear()
            self.stable_periods = 0
            self.reason = "零位或转速异常，重新估计"
            return []
        self.periods.append(period)
        self.stable_periods = self.stable_periods + 1 if previous is not None else 0
        if self.stable_periods < 1:
            self.reason = "等待连续稳定转动"
            return []
        if start - start_error < self.collect_after:
            self.reason = "等待停车后完整零位圈"
            return []
        lower_period = period - start_error - uncertainty
        if lower_period <= 0:
            self.reason = "校时误差超过旋转周期"
            return []
        result = []
        direction = 1 if self.config.get("clockwise", True) else -1
        offset = math.radians(float(self.config.get("angle_offset_deg", 0)))
        error_limit = float(self.config.get("max_timing_position_error_m", 0.04))
        for stamp, error, pixel, distance, is_echo in samples:
            if (not all(math.isfinite(v) for v in (stamp, error, distance)) or error < 0
                    or stamp - error < start + start_error or stamp + error >= timestamp - uncertainty):
                self.rejected_points += 1
                continue
            phase = (stamp - start) / period
            angular_error = math.tau * (error + (1 - phase) * start_error + phase * uncertainty) / lower_period
            if distance * angular_error > error_limit:
                self.rejected_points += 1
                continue
            result.append(PolarPoint(distance, (offset + direction * math.tau * phase) % math.tau,
                                     stamp, pixel, bool(is_echo)))
        self.last_closed_points = result
        self.last_closed_period = period
        if self.stable_periods < 2:
            self.reason = "转速稳定中"
            return []
        valid, self.reason = scan_complete(result, self.config)
        return result if valid else []


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

    def sample(self, timestamp: float, uncertainty: float, pixel: int, distance: float, is_echo=True):
        if timestamp < self.collect_after or self.anchor is None or self.stable_periods < 2:
            return
        if timestamp - self.anchor[0] > self.period_s * 1.5:
            self.reason = "零位更新超时"
            return
        if self.window is None:
            self.window = (timestamp, timestamp + self.period_s, self.anchor, self.period_s,
                           max(self.periods) - min(self.periods))
            self.samples = []
        self.samples.append((timestamp, uncertainty, pixel, distance, bool(is_echo)))
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
        for stamp, error, pixel, distance, is_echo in samples:
            if not start <= stamp < end or not math.isfinite(error) or error < 0:
                continue
            phase = (stamp - anchor[0]) / period
            angular_error = math.tau * ((error + anchor[1]) / period + abs(phase) * spread / period)
            if distance * angular_error > float(self.config.get("max_timing_position_error_m", 0.04)):
                continue
            points.append(PolarPoint(distance, (offset + direction * math.tau * phase) % math.tau,
                                     stamp, pixel, bool(is_echo)))
        valid, self.reason = scan_complete(points, self.config)
        return self.sequence, points if valid else [], period



@dataclass(frozen=True)
class HardwareObservation:
    source: str
    sequence: int
    device_timestamp: float
    distance: float | None = None
    status: str = "ok"
    uncertainty: float = 0.0
    pixel: int = 0
    is_echo: bool = True


@dataclass(frozen=True)
class ReceivedObservation:
    observation: HardwareObservation
    arrival_time: float


class DistanceObservationReceiver:
    def __init__(self, config: dict):
        self.config = config
        self.builder = TimedSweepBuilder(config)
        self.reorder_s = float(config.get("observation_reorder_s", config.get("simulation_reorder_s", 0.3)))
        if not math.isfinite(self.reorder_s) or self.reorder_s < 0:
            raise ValueError("重排等待时间必须是非负有限数")
        self.pending = []
        self.seen = {}
        self.progress = {}
        self.counter = 0
        self.watermark = -math.inf
        self.accepted = self.discarded = self.late = self.duplicates = 0
        self.warmup = 0
        self._preview = deque(maxlen=512)
        self.preview_points = ()
        self.local_results = []

    def reset(self, now: float):
        self.builder.begin_after(now)
        self.local_results.clear()
        self._preview.clear()
        self.preview_points = ()

    def invalidate(self, reason):
        self.discarded += bool(self.builder.samples)
        self.builder.reset()
        self.builder.reason = reason
        self.pending.clear()
        self._preview.clear()
        self.preview_points = ()
        self.local_results.clear()

    def estimate_angle(self, packet):
        anchor = self.builder.anchor
        periods = self.builder.periods
        if anchor is None or not periods or self.builder.stable_periods < 1:
            return None
        period = sum(periods) / len(periods)
        age = packet.device_timestamp - anchor[0]
        if not 0 <= age <= period * 1.5:
            return None
        direction = 1 if self.config.get("clockwise", True) else -1
        angle = math.radians(float(self.config.get("angle_offset_deg", 0))) + direction * math.tau * age / period
        error = math.tau * ((packet.uncertainty + anchor[1]) / period
                           + abs(age / period) * (max(periods) - min(periods)) / period)
        return angle % math.tau, error

    def feed(self, received: ReceivedObservation):
        packet = received.observation
        timestamp = packet.device_timestamp
        if (packet.source not in {"range", "rotation"} or not math.isfinite(timestamp)
                or not math.isfinite(packet.uncertainty) or packet.uncertainty < 0):
            return False
        key = packet.source, packet.sequence
        if key in self.seen:
            self.duplicates += 1
            return False
        previous = self.progress.get(packet.source)
        if previous is not None:
            seq, stamp = previous
            if (packet.sequence > seq and timestamp < stamp) or (packet.sequence < seq and timestamp > stamp):
                self.invalidate("设备时间倒退，等待新的角度基准")
                return False
        if timestamp <= self.watermark:
            self.late += 1
            if packet.source == "rotation" and (previous is None or packet.sequence > previous[0]):
                self.invalidate("零位超过重排窗口，当前扫描失效")
            return False
        if previous is None or packet.sequence > previous[0]:
            self.progress[packet.source] = packet.sequence, timestamp
        self.seen[key] = timestamp
        self.counter += 1
        heapq.heappush(self.pending, (timestamp, 0 if packet.source == "rotation" else 1, self.counter, packet))
        return True

    def poll(self, now: float):
        cutoff = max(self.watermark, now - self.reorder_s)
        results = []
        while self.pending and self.pending[0][0] <= cutoff:
            timestamp, _, _, packet = heapq.heappop(self.pending)
            if packet.source == "range":
                distance = packet.distance
                if (packet.status in {"ok", "over_range"} and distance is not None and math.isfinite(distance)
                        and float(self.config.get("min_range_m", 0.08)) <= distance <= float(self.config.get("max_range_m", 3.0))):
                    is_echo = bool(packet.is_echo and packet.status == "ok")
                    self.builder.sample(timestamp, packet.uncertainty, packet.pixel, distance, is_echo)
                    estimate = self.estimate_angle(packet)
                    if estimate is not None and is_echo:
                        self._preview.append((timestamp, estimate[0], distance))
            else:
                had_anchor = self.builder.anchor is not None
                points = self.builder.trigger(timestamp, packet.uncertainty, packet.sequence)
                if self.builder.last_closed_points:
                    self.local_results.append((packet.sequence - 1, list(self.builder.last_closed_points),
                                                self.builder.last_closed_period))
                if points:
                    self.accepted += 1
                    results.append((packet.sequence - 1, points, self.builder.period_s))
                elif had_anchor:
                    if self.builder.reason in {"等待连续稳定转动", "等待停车后完整零位圈"}:
                        self.warmup += 1
                    else:
                        self.discarded += 1
        period = self.builder.period_s or float(self.config.get("radar_period_s", 1.5))
        if self.builder.anchor is not None and cutoff - self.builder.anchor[0] > period * 1.5:
            self.invalidate("零位更新超时，当前扫描失效")
        while self._preview and self._preview[0][0] < cutoff - period:
            self._preview.popleft()
        self.preview_points = tuple(self._preview)
        self.watermark = cutoff
        self.seen = {key: timestamp for key, timestamp in self.seen.items() if timestamp > cutoff}
        return results
