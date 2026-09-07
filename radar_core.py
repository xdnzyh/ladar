from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
import re
import time
from typing import Deque, Iterable


@dataclass
class CalibrationModel:
    """Inverse triangulation model: pixel = p0 + k / distance."""

    p0: float | None = None
    k: float | None = None
    points: list[tuple[float, float]] = field(default_factory=list)
    rmse: float | None = None

    @property
    def ready(self) -> bool:
        return self.p0 is not None and self.k is not None and abs(self.k) > 1e-9

    def add_point(self, pixel: float, distance_m: float) -> None:
        if not (0 <= pixel <= 4095):
            raise ValueError("像素坐标超出有效范围")
        if not (0.02 <= distance_m <= 100):
            raise ValueError("标定距离应在 0.02～100 m 之间")
        self.points.append((float(pixel), float(distance_m)))

    def clear(self) -> None:
        self.points.clear()
        self.p0 = None
        self.k = None
        self.rmse = None

    def fit(self) -> tuple[float, float, float]:
        if len(self.points) < 2:
            raise ValueError("至少需要两个不同距离的标定点")

        xs = [1.0 / distance for _, distance in self.points]
        ys = [pixel for pixel, _ in self.points]
        n = len(xs)
        sx = sum(xs)
        sy = sum(ys)
        sxx = sum(x * x for x in xs)
        sxy = sum(x * y for x, y in zip(xs, ys))
        denominator = n * sxx - sx * sx
        if abs(denominator) < 1e-12:
            raise ValueError("标定距离不能全部相同")

        k = (n * sxy - sx * sy) / denominator
        p0 = (sy - k * sx) / n
        if abs(k) < 1e-9:
            raise ValueError("标定结果无效，请扩大标定距离范围")

        residuals = [pixel - (p0 + k / distance) for pixel, distance in self.points]
        rmse = math.sqrt(sum(value * value for value in residuals) / n)
        self.p0 = p0
        self.k = k
        self.rmse = rmse
        return p0, k, rmse

    def distance(self, pixel: float) -> float | None:
        if not self.ready:
            return None
        denominator = float(pixel) - float(self.p0)
        if abs(denominator) < 1e-9:
            return None
        result = float(self.k) / denominator
        if not math.isfinite(result) or result <= 0:
            return None
        return result

    def to_dict(self) -> dict:
        return {
            "p0": self.p0,
            "k": self.k,
            "rmse": self.rmse,
            "points": [[pixel, distance] for pixel, distance in self.points],
        }

    @classmethod
    def from_dict(cls, data: dict | None) -> "CalibrationModel":
        data = data or {}
        points = []
        for item in data.get("points", []):
            if isinstance(item, (list, tuple)) and len(item) == 2:
                points.append((float(item[0]), float(item[1])))
        return cls(
            p0=_optional_float(data.get("p0")),
            k=_optional_float(data.get("k")),
            points=points,
            rmse=_optional_float(data.get("rmse")),
        )


def _optional_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


class CCDFrameParser:
    """Streaming parser for the center-coordinate responses in the supplied files."""

    MODES = {
        "fffe": "FF FE + 2字节",
        "raw2": "原始2字节",
        "ascii": "ASCII状态",
    }

    _CENTER_RE = re.compile(rb"(?:center|\xe4\xb8\xad\xe5\xbf\x83)\s*[:=]\s*(\d+)", re.I)

    def __init__(self, mode: str = "fffe") -> None:
        self.mode = mode
        self.buffer = bytearray()

    def reset(self) -> None:
        self.buffer.clear()

    def feed(self, data: bytes) -> list[int]:
        if not data:
            return []
        self.buffer.extend(data)
        if self.mode == "raw2":
            return self._parse_raw2()
        if self.mode == "ascii":
            return self._parse_ascii()
        return self._parse_fffe()

    def _parse_fffe(self) -> list[int]:
        values: list[int] = []
        while True:
            index = self.buffer.find(b"\xff\xfe")
            if index < 0:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                break
            if index:
                del self.buffer[:index]
            if len(self.buffer) < 4:
                break
            value = (self.buffer[2] << 8) | self.buffer[3]
            del self.buffer[:4]
            if 0 <= value <= 4095:
                values.append(value)
        return values

    def _parse_raw2(self) -> list[int]:
        values: list[int] = []
        while len(self.buffer) >= 2:
            value = (self.buffer[0] << 8) | self.buffer[1]
            del self.buffer[:2]
            if 0 <= value <= 4095:
                values.append(value)
        return values

    def _parse_ascii(self) -> list[int]:
        values: list[int] = []
        while True:
            positions = [p for p in (self.buffer.find(b"\r"), self.buffer.find(b"\n")) if p >= 0]
            if not positions:
                if len(self.buffer) > 2048:
                    del self.buffer[:-1024]
                break
            position = min(positions)
            line = bytes(self.buffer[:position])
            del self.buffer[: position + 1]
            line = line.strip()
            if not line:
                continue
            match = self._CENTER_RE.search(line)
            if match:
                values.append(int(match.group(1)))
                continue
            numbers = re.findall(rb"\d+", line)
            if numbers:
                candidate = int(numbers[-1])
                if 0 <= candidate <= 4095:
                    values.append(candidate)
        return values


class MotorLineParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def reset(self) -> None:
        self.buffer.clear()

    def feed(self, data: bytes) -> list[str]:
        if not data:
            return []
        self.buffer.extend(data)
        lines: list[str] = []
        while True:
            positions = [p for p in (self.buffer.find(b"\r"), self.buffer.find(b"\n")) if p >= 0]
            if not positions:
                if len(self.buffer) > 4096:
                    del self.buffer[:-2048]
                break
            position = min(positions)
            raw = bytes(self.buffer[:position])
            del self.buffer[: position + 1]
            while self.buffer and self.buffer[0] in (10, 13):
                del self.buffer[0]
            text = raw.decode("utf-8", "ignore").strip()
            if text:
                lines.append(text)
        return lines


@dataclass
class PolarPoint:
    distance_m: float
    angle_rad: float
    timestamp: float
    pixel: int

    @property
    def x(self) -> float:
        return self.distance_m * math.sin(self.angle_rad)

    @property
    def y(self) -> float:
        return self.distance_m * math.cos(self.angle_rad)


@dataclass
class PendingSample:
    timestamp: float
    distance_m: float
    pixel: int


class RotationTracker:
    """Uses one optical trigger per revolution and corrects the completed sweep."""

    def __init__(
        self,
        angle_offset_deg: float = 0.0,
        clockwise: bool = True,
        initial_period_s: float = 2.5,
        keep_revolutions: int = 3,
    ) -> None:
        self.angle_offset_deg = angle_offset_deg
        self.clockwise = clockwise
        self.period_s = initial_period_s
        self.keep_revolutions = keep_revolutions
        self.last_trigger: float | None = None
        self.trigger_count = 0
        self.pending: list[PendingSample] = []
        self.completed: Deque[list[PolarPoint]] = deque(maxlen=max(1, keep_revolutions))
        self.period_history: Deque[float] = deque(maxlen=5)

    def reset(self) -> None:
        self.last_trigger = None
        self.trigger_count = 0
        self.pending.clear()
        self.completed.clear()
        self.period_history.clear()

    def configure(self, angle_offset_deg: float, clockwise: bool, keep_revolutions: int) -> None:
        self.angle_offset_deg = angle_offset_deg
        self.clockwise = clockwise
        if keep_revolutions != self.keep_revolutions:
            old = list(self.completed)[-max(1, keep_revolutions) :]
            self.keep_revolutions = max(1, keep_revolutions)
            self.completed = deque(old, maxlen=self.keep_revolutions)

    def trigger(self, timestamp: float | None = None, count: int | None = None) -> list[PolarPoint]:
        timestamp = time.perf_counter() if timestamp is None else timestamp
        completed: list[PolarPoint] = []
        if self.last_trigger is not None:
            period = timestamp - self.last_trigger
            if 0.1 <= period <= 60:
                self.period_history.append(period)
                ordered = sorted(self.period_history)
                self.period_s = ordered[len(ordered) // 2]
                completed = self._convert_samples(self.pending, self.last_trigger, period)
                if completed:
                    self.completed.append(completed)
        self.pending.clear()
        self.last_trigger = timestamp
        self.trigger_count = int(count) if count is not None else self.trigger_count + 1
        return completed

    def add_sample(self, distance_m: float, pixel: int, timestamp: float | None = None) -> PolarPoint | None:
        timestamp = time.perf_counter() if timestamp is None else timestamp
        if self.last_trigger is None:
            return None
        sample = PendingSample(timestamp, distance_m, pixel)
        self.pending.append(sample)
        phase = max(0.0, min(1.0, (timestamp - self.last_trigger) / max(self.period_s, 1e-3)))
        return self._to_point(sample, phase)

    def active_points(self) -> list[PolarPoint]:
        if self.last_trigger is None:
            return []
        return [
            self._to_point(sample, max(0.0, min(1.0, (sample.timestamp - self.last_trigger) / max(self.period_s, 1e-3))))
            for sample in self.pending
        ]

    def all_points(self) -> Iterable[tuple[PolarPoint, float]]:
        revolutions = list(self.completed)
        total = max(1, len(revolutions))
        for index, revolution in enumerate(revolutions):
            alpha = 0.2 + 0.65 * (index + 1) / total
            for point in revolution:
                yield point, alpha
        for point in self.active_points():
            yield point, 1.0

    @property
    def rpm(self) -> float:
        return 60.0 / self.period_s if self.period_s > 0 else 0.0

    def _convert_samples(self, samples: list[PendingSample], start: float, period: float) -> list[PolarPoint]:
        points: list[PolarPoint] = []
        for sample in samples:
            phase = (sample.timestamp - start) / period
            if 0.0 <= phase < 1.05:
                points.append(self._to_point(sample, min(1.0, phase)))
        return points

    def _to_point(self, sample: PendingSample, phase: float) -> PolarPoint:
        direction = 1.0 if self.clockwise else -1.0
        angle = math.radians(self.angle_offset_deg) + direction * math.tau * phase
        return PolarPoint(sample.distance_m, angle, sample.timestamp, sample.pixel)


class SlidingRate:
    def __init__(self, window_s: float = 2.0) -> None:
        self.window_s = window_s
        self.events: Deque[float] = deque()

    def add(self, timestamp: float | None = None) -> None:
        timestamp = time.perf_counter() if timestamp is None else timestamp
        self.events.append(timestamp)
        self._trim(timestamp)

    def value(self, timestamp: float | None = None) -> float:
        timestamp = time.perf_counter() if timestamp is None else timestamp
        self._trim(timestamp)
        if len(self.events) < 2:
            return 0.0
        elapsed = self.events[-1] - self.events[0]
        return (len(self.events) - 1) / elapsed if elapsed > 0 else 0.0

    def _trim(self, now: float) -> None:
        while self.events and now - self.events[0] > self.window_s:
            self.events.popleft()
