from dataclasses import dataclass
import math

from scan_acquisition import HardwareObservation


@dataclass(frozen=True)
class DeviceObservation:
    source: str
    sequence: int
    timestamp_us: float
    uncertainty_s: float = 0.0
    distance: float | None = None
    pixel: int = 0
    status: str = "ok"

    def normalize(self, clock, arrival):
        timestamp, error = clock.map(self.timestamp_us, arrival)
        return HardwareObservation(self.source, self.sequence, timestamp, self.distance,
                                   self.status, error + self.uncertainty_s, self.pixel)


def parse_observation(source, line, session, calibration, config):
    parts = line.split()
    if len(parts) < 2 or parts[1] != session:
        return None
    if source == "measurement" and parts[0] == "PIX" and len(parts) == 6:
        sequence, begin, end, pixel = map(int, parts[2:])
        if begin < 0 or end < begin or end - begin > 250000:
            raise ValueError("无效采集区间")
        distance = calibration.distance(pixel) if 0 <= pixel <= 1499 else None
        if distance is not None and not math.isfinite(distance):
            distance = None
        return DeviceObservation("range", sequence, (begin + end) / 2, (end - begin) * 0.5e-6,
                                 distance, pixel, "ok" if distance is not None else "no_return")
    if source == "rotation" and parts[0] == "TRIG" and len(parts) == 4:
        sequence, tick = map(int, parts[2:])
        if tick < 0:
            raise ValueError("无效零位时间")
        return DeviceObservation("rotation", sequence, tick,
                                 float(config.get("irq_timestamp_uncertainty_ms", 2)) / 1000)
    return None
