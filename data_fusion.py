from __future__ import annotations

from collections import deque
import math
import re


class DeviceClock:
    """Maps one ESP32 microsecond counter onto the PC monotonic clock."""

    def __init__(self, keep: int = 80) -> None:
        self.samples: deque[tuple[float, float]] = deque(maxlen=max(8, keep))
        self.slope = 1.0
        self.offset = 0.0
        self.ready = False

    def observe(self, device_time_us: int, host_arrival_s: float) -> float:
        device_s = float(device_time_us) * 1e-6
        self.samples.append((device_s, host_arrival_s))
        if len(self.samples) == 1:
            self.offset = host_arrival_s - device_s
            self.ready = True
        elif len(self.samples) >= 6:
            self._fit_lower_envelope()
        return self.to_host(device_time_us)

    def _fit_lower_envelope(self) -> None:
        samples = list(self.samples)
        offsets = sorted(host - device for device, host in samples)
        threshold = offsets[max(0, len(offsets) // 4)]
        selected = [(device, host) for device, host in samples if host - device <= threshold + 0.002]
        if len(selected) < 2:
            return
        mean_x = sum(item[0] for item in selected) / len(selected)
        mean_y = sum(item[1] for item in selected) / len(selected)
        variance = sum((item[0] - mean_x) ** 2 for item in selected)
        if variance <= 1e-12:
            return
        slope = sum((x - mean_x) * (y - mean_y) for x, y in selected) / variance
        if 0.9995 <= slope <= 1.0005:
            self.slope = slope
            self.offset = mean_y - slope * mean_x
            self.ready = True

    def to_host(self, device_time_us: int) -> float:
        return self.slope * (float(device_time_us) * 1e-6) + self.offset


class SequenceMonitor:
    def __init__(self) -> None:
        self.last: dict[str, int] = {}
        self.dropped: dict[str, int] = {}

    def add(self, source: str, sequence: int | None) -> int:
        if sequence is None:
            return 0
        previous = self.last.get(source)
        self.last[source] = sequence
        if previous is None or sequence <= previous:
            return 0
        missing = max(0, sequence - previous - 1)
        self.dropped[source] = self.dropped.get(source, 0) + missing
        return missing


_TRIGGER_RE = re.compile(r"^TRIG(?:[ ,]+(\d+))?(?:[ ,]+(?:TICK_US=)?(\d+))?$", re.I)
def parse_trigger(line: str) -> tuple[int | None, int | None] | None:
    match = _TRIGGER_RE.match(line.strip())
    if not match:
        return None
    count = int(match.group(1)) if match.group(1) else None
    device_time_us = int(match.group(2)) if match.group(2) else None
    return count, device_time_us


def parse_timestamped_distance(line: str) -> tuple[int, int, float, int | None] | None:
    parts = [part.strip() for part in line.split(",")]
    if len(parts) < 4 or parts[0].upper() != "DIST":
        return None
    try:
        sequence = int(parts[1])
        device_time_us = int(parts[2])
        distance_m = float(parts[3])
        pixel = int(parts[4]) if len(parts) >= 5 and parts[4] else None
    except ValueError:
        return None
    if not math.isfinite(distance_m):
        return None
    return sequence, device_time_us, distance_m, pixel


def parse_chassis_state(line: str) -> tuple[int, int, tuple[float, ...]] | None:
    parts = [part for part in re.split(r"[ ,]+", line.strip()) if part]
    if len(parts) < 3 or parts[0].upper() not in {"STATE", "CHASSIS", "ODOM"}:
        return None
    try:
        sequence = int(parts[1])
        device_time_us = int(parts[2])
        values = tuple(float(part) for part in parts[3:])
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return sequence, device_time_us, values
