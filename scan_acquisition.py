from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
import math
from statistics import median

from radar_core import PolarPoint
from navigation_core import ScanPoint
from scan_geometry import line_supported_indices


def polar_to_scan_point(point: PolarPoint, quality: float | None = None) -> ScanPoint:
    if quality is None:
        quality = 1.0
    return ScanPoint(
        point.angle_rad,
        point.distance_m,
        float(quality),
        bool(point.is_echo),
        timestamp_s=point.timestamp,
        time_error_s=point.time_error_s,
        angle_error_rad=point.angle_error_rad,
        distance_error_m=point.distance_error_m,
        pixel=point.pixel,
        calibration_version=point.calibration_version,
        source=point.source,
        session=point.session,
    )


def scan_points_from_polar(points) -> list[ScanPoint]:
    return [polar_to_scan_point(point) for point in points]


def scan_complete(points, config):
    minimum = int(config.get("min_scan_points", 12))
    if len(points) < minimum:
        return False, "本圈有效测距不足"
    angles = sorted({p.angle_rad % math.tau for p in points})
    if len(angles) < minimum:
        return False, "本圈有效方位不足"
    gaps = [b - a for a, b in zip(angles, angles[1:] + [angles[0] + math.tau])]
    normal = median(gaps)
    limit = min(math.radians(float(config.get("scan_gap_hard_limit_deg", 45))),
                max(math.radians(float(config.get("max_scan_gap_deg", 25))),
                    normal * float(config.get("scan_gap_factor", 2.5))))
    if max(gaps) <= limit + 1e-9:
        return True, "完整扫描"

    # A small indoor scene often contains several short straight wall fragments
    # separated by large angular gaps.  Global 360-degree coverage is not a
    # requirement for mapping.  When enough locally straight echo points are
    # present, accept the revolution and let the mapping layer reject isolated
    # clutter instead of discarding the whole scan.
    echoes = [point for point in points if bool(getattr(point, "is_echo", True))]
    supported = line_supported_indices(
        echoes,
        min_window_points=int(config.get("line_support_window_points", 4)),
        max_angle_gap_deg=float(config.get("line_support_max_angle_gap_deg", 20.0)),
        max_neighbor_gap_m=float(config.get("line_support_max_neighbor_gap_m", 0.18)),
        max_rms_m=float(config.get("line_support_max_rms_m", 0.020)),
        min_span_m=float(config.get("line_support_min_span_m", 0.050)),
    )
    minimum_supported = int(config.get("min_line_supported_points", 8))
    minimum_ratio = float(config.get("min_line_support_ratio", 0.35))
    if (len(supported) >= minimum_supported
            and len(supported) / max(1, len(echoes)) >= minimum_ratio):
        return True, "短直线结构有效扫描"
    return False, "扫描存在过大的角度空缺，本圈丢弃"


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
        self.last_display_points = []
        self.last_closed_period = None
        self.last_outcome = "waiting_for_zero"
        self.last_failure_reason = ""

    def begin_after(self, timestamp):
        self.samples.clear()
        self.collect_after = timestamp
        self.reason = "等待停车后完整零位圈"
        self.last_display_points = []
        self.last_failure_reason = ""

    def sample(self, timestamp, uncertainty, pixel, distance, is_echo=True,
               distance_error_m=None, calibration_version=None, source_session=None):
        if (self.anchor is not None and self.anchor[0] - self.anchor[1] >= self.collect_after
                and timestamp >= self.anchor[0]):
            self.samples.append((timestamp, uncertainty, pixel, distance, bool(is_echo),
                                 distance_error_m, calibration_version, source_session))
            self.reason = "扫描中，等待下一真实零位确认"
            self.last_outcome = "collecting"

    def trigger(self, timestamp, uncertainty, count):
        self.last_closed_points = []
        self.last_display_points = []
        self.last_closed_period = None
        anchor, samples = self.anchor, self.samples
        self.anchor = (timestamp, uncertainty, count)
        self.samples = []
        if anchor is None:
            self.last_outcome = "waiting_for_zero"
            return []
        start, start_error, previous_count = anchor
        period = timestamp - start
        if count != previous_count + 1 or not 0.1 <= period <= 60:
            self.reset()
            self.anchor = (timestamp, uncertainty, count)
            self.reason = "零位不连续，本圈丢弃"
            self.last_failure_reason = self.reason
            self.last_outcome = "timing_failure"
            return []

        # Display is deliberately independent from mapping confidence.  Once a
        # revolution is bounded by two consecutive real TRIG events, every
        # finite range sample in that physical revolution remains visible.
        direction = 1 if self.config.get("clockwise", True) else -1
        offset = math.radians(float(self.config.get("angle_offset_deg", 0)))
        display_lower_period = period - start_error - uncertainty
        display_points = []
        for stamp, error, pixel, distance, is_echo, distance_error_m, calibration_version, source_session in samples:
            if (not math.isfinite(stamp) or not math.isfinite(distance)
                    or not start <= stamp < timestamp):
                continue
            phase = (stamp - start) / period
            angle_error = None
            if math.isfinite(error) and error >= 0 and display_lower_period > 0:
                angle_error = math.tau * (
                    error + (1 - phase) * start_error + phase * uncertainty
                ) / display_lower_period
            display_points.append(PolarPoint(
                distance,
                (offset + direction * math.tau * phase) % math.tau,
                stamp,
                pixel,
                bool(is_echo),
                time_error_s=error if math.isfinite(error) and error >= 0 else None,
                angle_error_rad=angle_error,
                distance_error_m=distance_error_m,
                calibration_version=calibration_version,
                source="range",
                session=source_session,
            ))
        self.last_display_points = display_points
        self.last_closed_period = period

        previous = self.previous_period
        self.previous_period = self.period_s = period
        if previous is not None and abs(period / previous - 1) > float(self.config.get("period_tolerance", 0.05)):
            self.periods.clear()
            self.stable_periods = 0
            self.reason = "零位或转速异常，重新估计"
            self.last_failure_reason = self.reason
            self.last_outcome = "timing_failure"
            return []
        self.periods.append(period)
        self.stable_periods = self.stable_periods + 1 if previous is not None else 0
        if self.stable_periods < 1:
            self.reason = "等待连续稳定转动"
            self.last_outcome = "warmup"
            return []
        if start - start_error < self.collect_after:
            self.reason = "等待停车后完整零位圈"
            self.last_outcome = "warmup"
            return []
        lower_period = period - start_error - uncertainty
        if lower_period <= 0:
            self.reason = "校时误差超过旋转周期"
            self.last_failure_reason = self.reason
            self.last_outcome = "timing_failure"
            return []
        result = []
        # 4 cm remains the scale used for downstream confidence weighting, but
        # it is too strict as a hard acquisition cutoff.  Only observations
        # whose timing uncertainty implies an extreme (>12 cm by default)
        # endpoint ambiguity are removed here; moderate uncertainty is carried
        # forward in angle_error_rad and down-weighted by the mapper.
        error_limit = max(
            float(self.config.get("max_timing_position_error_m", 0.04)),
            float(self.config.get("hard_timing_position_error_m", 0.12)),
        )
        for stamp, error, pixel, distance, is_echo, distance_error_m, calibration_version, source_session in samples:
            if (not all(math.isfinite(v) for v in (stamp, error, distance)) or error < 0
                    or stamp - error < start + start_error or stamp + error >= timestamp - uncertainty):
                self.rejected_points += 1
                continue
            phase = (stamp - start) / period
            angular_error = math.tau * (error + (1 - phase) * start_error + phase * uncertainty) / lower_period
            if distance * angular_error > error_limit:
                self.rejected_points += 1
                continue
            result.append(PolarPoint(
                distance,
                (offset + direction * math.tau * phase) % math.tau,
                stamp,
                pixel,
                bool(is_echo),
                time_error_s=error,
                angle_error_rad=angular_error,
                distance_error_m=distance_error_m,
                calibration_version=calibration_version,
                source="range",
                session=source_session,
            ))
        self.last_closed_points = result
        required_stable_periods = max(1, int(self.config.get("formal_scan_stable_periods", 1)))
        if self.stable_periods < required_stable_periods:
            self.reason = "转速稳定中"
            self.last_outcome = "warmup"
            return []
        valid, self.reason = scan_complete(result, self.config)
        self.last_outcome = "formal" if valid else "coverage_failure"
        if valid:
            self.last_failure_reason = ""
        else:
            self.last_failure_reason = self.reason
        return result if valid else []


class EstimatedSweepBuilder:
    def __init__(self, config: dict):
        self.config = config
        self.reset()

    def reset(self):
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
        self.last_closed_points = []
        self.last_closed_period = None
        self.last_outcome = "waiting_for_zero"
        self.last_failure_reason = ""

    def begin_after(self, timestamp: float):
        self.samples = []
        self.window = None
        self.collect_after = timestamp
        self.reason = "等待停车稳定"
        self.last_closed_points = []
        self.last_closed_period = None
        self.last_outcome = "warmup"
        self.last_failure_reason = ""

    def trigger(self, timestamp: float, uncertainty: float, count: int):
        self.last_closed_points = []
        self.last_closed_period = None
        previous = self.anchor
        self.anchor = (timestamp, uncertainty, count)
        if previous is None:
            self.last_outcome = "waiting_for_zero"
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
            self.last_failure_reason = self.reason
            self.last_outcome = "timing_failure"
            return
        self.periods.append(period)
        self.previous_period = period
        self.period_s = sum(self.periods) / len(self.periods)
        self.stable_periods = max(0, len(self.periods) - 1)
        self.last_outcome = "warmup" if self.stable_periods < 2 else "collecting"

    def sample(self, timestamp: float, uncertainty: float, pixel: int, distance: float, is_echo=True,
               distance_error_m=None, calibration_version=None, source_session=None):
        if timestamp < self.collect_after or self.anchor is None or self.stable_periods < 2:
            return
        if timestamp - self.anchor[0] > self.period_s * 1.5:
            self.reason = "零位更新超时"
            self.last_failure_reason = self.reason
            return
        if self.window is None:
            self.window = (timestamp, timestamp + self.period_s, self.anchor, self.period_s,
                           max(self.periods) - min(self.periods))
            self.samples = []
        self.samples.append((timestamp, uncertainty, pixel, distance, bool(is_echo),
                             distance_error_m, calibration_version, source_session))
        self.reason = "扫描中"
        self.last_outcome = "collecting"

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
        error_limit = max(
            float(self.config.get("max_timing_position_error_m", 0.04)),
            float(self.config.get("hard_timing_position_error_m", 0.12)),
        )
        for stamp, error, pixel, distance, is_echo, distance_error_m, calibration_version, source_session in samples:
            if not start <= stamp < end or not math.isfinite(error) or error < 0:
                continue
            phase = (stamp - anchor[0]) / period
            angular_error = math.tau * ((error + anchor[1]) / period + abs(phase) * spread / period)
            if distance * angular_error > error_limit:
                continue
            points.append(PolarPoint(
                distance,
                (offset + direction * math.tau * phase) % math.tau,
                stamp,
                pixel,
                bool(is_echo),
                time_error_s=error,
                angle_error_rad=angular_error,
                distance_error_m=distance_error_m,
                calibration_version=calibration_version,
                source="range",
                session=source_session,
            ))
        self.last_closed_points = points
        self.last_closed_period = period
        valid, self.reason = scan_complete(points, self.config)
        self.last_outcome = "formal" if valid else "coverage_failure"
        if valid:
            self.last_failure_reason = ""
        else:
            self.last_failure_reason = self.reason
        return self.sequence, points if valid else [], period


@dataclass(frozen=True, eq=False)
class HardwareObservation:
    source: str
    sequence: int
    device_timestamp: float
    distance: float | None = None
    status: str = "ok"
    uncertainty: float = 0.0
    pixel: int = 0
    is_echo: bool = True
    raw_timestamp_us: float | None = None
    clock_model_version: int | None = None
    source_session: str | None = None
    calibration_version: str | None = None
    distance_error_m: float | None = None

    def __eq__(self, other):
        if not isinstance(other, HardwareObservation):
            return NotImplemented
        return (
            self.source, self.sequence, self.device_timestamp, self.distance,
            self.status, self.uncertainty, self.pixel, self.is_echo,
        ) == (
            other.source, other.sequence, other.device_timestamp, other.distance,
            other.status, other.uncertainty, other.pixel, other.is_echo,
        )


@dataclass(frozen=True)
class ReceivedObservation:
    observation: HardwareObservation
    arrival_time: float


@dataclass(frozen=True)
class PreviewObservation:
    timestamp: float
    angle_rad: float
    distance_m: float
    pixel: int = 0
    quality: float = 1.0
    time_error_s: float | None = None
    angle_error_rad: float | None = None

    def __iter__(self):
        yield self.timestamp
        yield self.angle_rad
        yield self.distance_m

    def __getitem__(self, index):
        return (self.timestamp, self.angle_rad, self.distance_m)[index]


@dataclass(frozen=True)
class ReceiverPollResult:
    formal_scans: tuple[tuple[int, list[PolarPoint], float | None], ...] = ()
    local_scans: tuple[tuple[int, list[PolarPoint], float | None], ...] = ()
    diagnostics: tuple[str, ...] = ()

    def __iter__(self):
        return iter(self.formal_scans)

    def __len__(self):
        return len(self.formal_scans)

    def __bool__(self):
        return bool(self.formal_scans)

    def __getitem__(self, index):
        return self.formal_scans[index]


class DistanceObservationReceiver:
    def __init__(self, config: dict):
        self.config = config
        self.builder = (EstimatedSweepBuilder(config)
                        if config.get("arbitrary_phase_scans", False)
                        else TimedSweepBuilder(config))
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
        self.timestamp_conflicts = {"range": 0, "rotation": 0}
        self._preview = deque(maxlen=512)
        self.preview_points = ()
        self._preview_anchor_count = None
        self.local_results = []
        self.raw_progress = {}
        self.max_pending = 0

    def reset(self, now: float):
        self.builder.begin_after(now)
        self.local_results.clear()
        self._preview.clear()
        self.preview_points = ()
        self._preview_anchor_count = None

    def invalidate(self, reason):
        self.discarded += bool(self.builder.samples)
        self.builder.reset()
        self.builder.reason = reason
        self.builder.last_failure_reason = reason
        self.pending.clear()
        self.seen.clear()
        self.progress.clear()
        self.raw_progress.clear()
        self.watermark = -math.inf
        self._preview.clear()
        self.preview_points = ()
        self._preview_anchor_count = None
        self.local_results.clear()

    def estimate_angle(self, packet, *, require_stable: bool = True):
        anchor = self.builder.anchor
        periods = self.builder.periods
        if anchor is None or not periods or (require_stable and self.builder.stable_periods < 1):
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
            seq, stamp, model_version = previous
            raw_previous = self.raw_progress.get(packet.source)
            raw_current = packet.raw_timestamp_us
            raw_conflict = (
                raw_previous is not None and raw_current is not None
                and ((packet.sequence > seq and raw_current < raw_previous)
                     or (packet.sequence < seq and raw_current > raw_previous))
            )
            same_model = model_version == packet.clock_model_version
            mapped_conflict = same_model and (
                (packet.sequence > seq and timestamp < stamp)
                or (packet.sequence < seq and timestamp > stamp)
            )
            if raw_conflict or mapped_conflict:
                self.timestamp_conflicts[packet.source] += 1
                if packet.source == "rotation":
                    self.invalidate("零位设备时间异常，当前扫描失效，等待新的真实零位")
                return False
        if timestamp <= self.watermark:
            self.late += 1
            if packet.source == "rotation" and (previous is None or packet.sequence > previous[0]):
                self.invalidate("零位超过重排窗口，当前扫描失效")
            return False
        if previous is None or packet.sequence > previous[0]:
            self.progress[packet.source] = packet.sequence, timestamp, packet.clock_model_version
            if packet.raw_timestamp_us is not None:
                self.raw_progress[packet.source] = packet.raw_timestamp_us
        self.seen[key] = timestamp
        self.counter += 1
        heapq.heappush(self.pending, (timestamp, 0 if packet.source == "rotation" else 1, self.counter, packet))
        self.max_pending = max(self.max_pending, len(self.pending))
        return True

    def poll(self, now: float):
        cutoff = max(self.watermark, now - self.reorder_s)
        results = []
        local_results = []
        diagnostics = []
        estimated = isinstance(self.builder, EstimatedSweepBuilder)

        def finish_estimated_window(timestamp):
            if not estimated:
                return
            result = self.builder.poll(timestamp)
            if result is None:
                return
            sequence, points, period = result
            if self.builder.last_closed_points and not points:
                local_results.append((sequence, list(self.builder.last_closed_points), period))
            if points:
                self.accepted += 1
                results.append((sequence, points, period))
            else:
                self.discarded += 1

        while self.pending and self.pending[0][0] <= cutoff:
            timestamp, _, _, packet = heapq.heappop(self.pending)
            finish_estimated_window(timestamp)
            if packet.source == "range":
                distance = packet.distance
                if (packet.status in {"ok", "over_range"} and distance is not None and math.isfinite(distance)
                        and float(self.config.get("min_range_m", 0.08)) <= distance <= float(self.config.get("max_range_m", 3.0))):
                    is_echo = bool(packet.is_echo and packet.status == "ok")
                    self.builder.sample(
                        timestamp,
                        packet.uncertainty,
                        packet.pixel,
                        distance,
                        is_echo,
                        packet.distance_error_m,
                        packet.calibration_version,
                        packet.source_session,
                    )
                    estimate = self.estimate_angle(packet, require_stable=False)
                    if estimate is not None and is_echo:
                        if not estimated:
                            anchor_count = self.builder.anchor[2] if self.builder.anchor is not None else None
                            if anchor_count != self._preview_anchor_count:
                                self._preview.clear()
                                self._preview_anchor_count = anchor_count
                        self._preview.append(PreviewObservation(
                            timestamp,
                            estimate[0],
                            distance,
                            packet.pixel,
                            1.0,
                            packet.uncertainty,
                            estimate[1],
                        ))
            else:
                had_anchor = self.builder.anchor is not None
                before_invalidated = getattr(self.builder, "invalidated", 0)
                points = self.builder.trigger(timestamp, packet.uncertainty, packet.sequence)
                if estimated:
                    self.discarded += max(0, self.builder.invalidated - before_invalidated)
                    if self.builder.stable_periods < 2 and had_anchor:
                        self.warmup += 1
                    continue

                if self.builder.last_closed_period is not None:
                    self._preview.clear()
                    self._preview_anchor_count = None
                    for point in self.builder.last_display_points:
                        self._preview.append(PreviewObservation(
                            point.timestamp,
                            point.angle_rad,
                            point.distance_m,
                            point.pixel,
                            1.0,
                            point.time_error_s,
                            point.angle_error_rad,
                        ))

                # A formal scan and its local fallback must not both write the
                # same physical revolution.  Local mapping is retained only for
                # a revolution that failed the formal completeness gate.
                if self.builder.last_closed_points and not points:
                    local_results.append((packet.sequence - 1, list(self.builder.last_closed_points),
                                          self.builder.last_closed_period))
                if points:
                    self.accepted += 1
                    results.append((packet.sequence - 1, points, self.builder.period_s))
                elif had_anchor:
                    if self.builder.last_outcome == "warmup":
                        self.warmup += 1
                    else:
                        self.discarded += 1
        finish_estimated_window(cutoff)
        if estimated:
            period = self.builder.period_s or float(self.config.get("radar_period_s", 1.5))
            while self._preview and self._preview[0][0] < cutoff - period:
                self._preview.popleft()
        self.preview_points = tuple(self._preview)
        self.watermark = cutoff
        self.seen = {key: timestamp for key, timestamp in self.seen.items() if timestamp > cutoff}
        if self.builder.last_outcome in {"coverage_failure", "timing_failure"}:
            diagnostics.append(self.builder.last_failure_reason or self.builder.reason or self.builder.last_outcome)
        self.local_results.clear()
        return ReceiverPollResult(tuple(results), tuple(local_results), tuple(diagnostics))
