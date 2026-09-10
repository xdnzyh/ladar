from __future__ import annotations

from collections import deque
import csv
from dataclasses import dataclass, field
import hashlib
import math
import re
from statistics import median, quantiles
import time
from typing import Deque, Iterable


CCD_PIXEL_MIN = 0
CCD_PIXEL_MAX = 1500


@dataclass
class CalibrationModel:
    """Inverse triangulation model: pixel = p0 + k / distance."""

    p0: float | None = None
    k: float | None = None
    points: list[tuple[float, float]] = field(default_factory=list)
    rmse: float | None = None
    model: str = "inverse"
    table_points: list[tuple[float, float]] = field(default_factory=list)

    @property
    def ready(self) -> bool:
        if self.model == "table":
            return len(self.table_points) >= 2
        return self.p0 is not None and self.k is not None and abs(self.k) > 1e-9

    @property
    def identifier(self) -> str:
        if self.model == "table":
            payload = repr(tuple(self.table_points))
        else:
            payload = repr((self.p0, self.k, self.rmse, tuple(self.points)))
        digest = hashlib.sha256(payload.encode("ascii")).hexdigest()[:12]
        return f"{self.model}-{digest}"

    @property
    def distance_range_m(self) -> tuple[float, float] | None:
        if self.model == "table" and self.table_points:
            return self.table_points[0][0] / 100.0, self.table_points[-1][0] / 100.0
        return None

    @property
    def coordinate_range(self) -> tuple[float, float] | None:
        if self.model == "table" and self.table_points:
            return self.table_points[-1][1], self.table_points[0][1]
        return None

    def add_point(self, pixel: float, distance_m: float) -> None:
        if not (CCD_PIXEL_MIN <= pixel <= CCD_PIXEL_MAX):
            raise ValueError(f"像素坐标超出有效范围（{CCD_PIXEL_MIN}～{CCD_PIXEL_MAX}）")
        if not (0.02 <= distance_m <= 100):
            raise ValueError("标定距离应在 0.02～100 m 之间")
        self.points.append((float(pixel), float(distance_m)))
        self.p0 = None
        self.k = None
        self.rmse = None

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
        try:
            pixel = float(pixel)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(pixel):
            return None
        if self.model == "table":
            if not self.table_points:
                return None
            first_pixel = self.table_points[0][1]
            last_pixel = self.table_points[-1][1]
            if not last_pixel <= pixel <= first_pixel:
                return None
            for (d1, x1), (d2, x2) in zip(self.table_points, self.table_points[1:]):
                if x2 <= pixel <= x1:
                    distance_cm = d1 + (x1 - pixel) / (x1 - x2) * (d2 - d1)
                    distance_m = distance_cm / 100.0
                    return distance_m if math.isfinite(distance_m) and distance_m > 0 else None
            return None
        if not self.ready:
            return None
        denominator = float(pixel) - float(self.p0)
        if abs(denominator) < 1e-9:
            return None
        result = float(self.k) / denominator
        if not math.isfinite(result) or result <= 0:
            return None
        return result

    def distance_slope_m_per_pixel(self, pixel: float) -> float | None:
        try:
            pixel = float(pixel)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(pixel):
            return None
        if self.model == "table":
            for (d1, x1), (d2, x2) in zip(self.table_points, self.table_points[1:]):
                if x2 <= pixel <= x1:
                    return abs(d2 - d1) / (100.0 * abs(x2 - x1))
            return None
        if not self.ready:
            return None
        denominator = float(pixel) - float(self.p0)
        if abs(denominator) < 1e-9:
            return None
        slope = abs(float(self.k) / (denominator * denominator))
        return slope if math.isfinite(slope) else None

    def pixel_quantization_error_m(self, pixel: float) -> float | None:
        if self.model == "table":
            center = self.distance(pixel)
            if center is None:
                return None
            neighbors = [self.distance(float(pixel) - 0.5), self.distance(float(pixel) + 0.5)]
            deviations = [abs(value - center) for value in neighbors if value is not None]
            return max(deviations) if deviations else None
        slope = self.distance_slope_m_per_pixel(pixel)
        return None if slope is None else 0.5 * slope

    def to_dict(self) -> dict:
        return {
            "model": self.model,
            "p0": self.p0,
            "k": self.k,
            "rmse": self.rmse,
            "points": [[pixel, distance] for pixel, distance in self.points],
            "table_points": [[distance_cm, pixel] for distance_cm, pixel in self.table_points],
        }

    @classmethod
    def from_dict(cls, data: dict | None) -> "CalibrationModel":
        data = data or {}
        points = []
        for item in data.get("points", []):
            if isinstance(item, (list, tuple)) and len(item) == 2:
                try:
                    pixel, distance = float(item[0]), float(item[1])
                except (TypeError, ValueError):
                    continue
                if (math.isfinite(pixel) and math.isfinite(distance)
                        and CCD_PIXEL_MIN <= pixel <= CCD_PIXEL_MAX
                        and 0.02 <= distance <= 100):
                    points.append((pixel, distance))
        table_points = []
        for item in data.get("table_points", []):
            if isinstance(item, (list, tuple)) and len(item) == 2:
                distance_cm, pixel = _optional_float(item[0]), _optional_float(item[1])
                if distance_cm is not None and pixel is not None:
                    table_points.append((distance_cm, pixel))
        try:
            table_points = _validate_table_points(table_points)
        except ValueError:
            table_points = []
        model = str(data.get("model", "inverse"))
        if model == "table" and not table_points:
            model = "inverse"
        return cls(
            p0=_optional_float(data.get("p0")),
            k=_optional_float(data.get("k")),
            points=points,
            rmse=_optional_float(data.get("rmse")),
            model=model,
            table_points=table_points,
        )

    @classmethod
    def from_csv(cls, path) -> "CalibrationModel":
        with open(path, "r", encoding="utf-8-sig", newline="") as file:
            reader = csv.DictReader(file)
            fields = set(reader.fieldnames or ())
            if not {"distance_cm", "ccd_x"}.issubset(fields):
                raise ValueError("标定表表头必须包含 distance_cm,ccd_x")
            rows = []
            for index, row in enumerate(reader, start=2):
                if index > 1001:
                    raise ValueError("标定表最多支持 1000 个点")
                try:
                    distance_cm = float(str(row["distance_cm"]).strip())
                    pixel = float(str(row["ccd_x"]).strip())
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"标定表第 {index} 行不是有限数值") from exc
                rows.append((distance_cm, pixel))
        return cls(model="table", table_points=_validate_table_points(rows))

    @classmethod
    def from_table(cls, points: list[tuple[float, float]]) -> "CalibrationModel":
        return cls(model="table", table_points=_validate_table_points(points))


def _validate_table_points(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    rows = [(float(distance_cm), float(pixel)) for distance_cm, pixel in points]
    if len(rows) < 2:
        raise ValueError("标定表至少需要两个点")
    if any(not math.isfinite(value) for row in rows for value in row):
        raise ValueError("标定值必须是有限数值")
    if any(distance_cm <= 0 or not CCD_PIXEL_MIN <= pixel <= CCD_PIXEL_MAX for distance_cm, pixel in rows):
        raise ValueError(f"标定距离必须大于 0，像素必须在 {CCD_PIXEL_MIN}～{CCD_PIXEL_MAX} 内")
    if any(d1 >= d2 or x1 <= x2 for (d1, x1), (d2, x2) in zip(rows, rows[1:])):
        raise ValueError("标定距离必须严格递增、像素必须严格递减，不能有重复值")
    return rows


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
            if CCD_PIXEL_MIN <= value <= CCD_PIXEL_MAX:
                values.append(value)
        return values

    def _parse_raw2(self) -> list[int]:
        values: list[int] = []
        while len(self.buffer) >= 2:
            value = (self.buffer[0] << 8) | self.buffer[1]
            del self.buffer[:2]
            if CCD_PIXEL_MIN <= value <= CCD_PIXEL_MAX:
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
                if CCD_PIXEL_MIN <= candidate <= CCD_PIXEL_MAX:
                    values.append(candidate)
        return values


class MotorLineParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.invalid_frames = 0

    def reset(self) -> None:
        self.buffer.clear()
        self.invalid_frames = 0

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
                    self.invalid_frames += 1
                break
            position = min(positions)
            raw = bytes(self.buffer[:position])
            del self.buffer[: position + 1]
            while self.buffer and self.buffer[0] in (10, 13):
                del self.buffer[0]
            try:
                text = raw.decode("ascii").strip()
            except UnicodeDecodeError:
                self.invalid_frames += 1
                continue
            if text:
                lines.append(text)
        return lines


@dataclass
class PolarPoint:
    distance_m: float
    angle_rad: float
    timestamp: float
    pixel: int
    is_echo: bool = True
    time_error_s: float | None = None
    angle_error_rad: float | None = None
    distance_error_m: float | None = None
    calibration_version: str | None = None
    source: str | None = None
    session: str | None = None

    @property
    def x(self) -> float:
        return self.distance_m * math.sin(self.angle_rad)

    @property
    def y(self) -> float:
        return self.distance_m * math.cos(self.angle_rad)


def analyze_repeated_calibration(samples, model: CalibrationModel | None = None) -> dict:
    grouped: dict[float, list[float]] = {}
    if isinstance(samples, dict):
        iterator = samples.items()
        for distance, pixels in iterator:
            grouped[float(distance)] = [float(pixel) for pixel in pixels]
    else:
        for distance, pixel in samples:
            grouped.setdefault(float(distance), []).append(float(pixel))
    result = []
    for distance, pixels in sorted(grouped.items()):
        pixels = [pixel for pixel in pixels if math.isfinite(pixel)]
        if not pixels:
            continue
        center = median(pixels)
        deviations = [abs(pixel - center) for pixel in pixels]
        entry = {
            "distance_m": distance,
            "pixel_median": center,
            "pixel_mad": median(deviations),
            "sample_count": len(pixels),
            "pixel_q05": None,
            "pixel_q95": None,
            "residual_median_m": None,
            "residual_mad_m": None,
            "quantization_error_m": None,
            "uncertainty_status": "未标定重复误差",
        }
        if len(pixels) >= 2:
            entry["pixel_q05"] = quantiles(pixels, n=20, method="inclusive")[0]
            entry["pixel_q95"] = quantiles(pixels, n=20, method="inclusive")[-1]
        if model is not None:
            predicted = [model.distance(pixel) for pixel in pixels]
            residuals = [value - distance for value in predicted if value is not None and math.isfinite(value)]
            if residuals:
                residual_center = median(residuals)
                entry["residual_median_m"] = residual_center
                entry["residual_mad_m"] = median(abs(value - residual_center) for value in residuals)
                entry["quantization_error_m"] = model.pixel_quantization_error_m(center)
                entry["uncertainty_status"] = "已计算量化与重复残差，系统项未标定"
        result.append(entry)
    return {"samples": result, "model_used": bool(model), "refit": False}


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
        clockwise: bool = False,
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
            if timestamp <= self.last_trigger or (count is not None and count == self.trigger_count):
                return []
            consecutive = count is None or count == self.trigger_count + 1
            if 0.1 <= period <= 60 and consecutive:
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
            if 0.0 <= phase < 1.0:
                points.append(self._to_point(sample, phase))
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
