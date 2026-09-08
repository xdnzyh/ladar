from dataclasses import dataclass
import math

from scan_acquisition import HardwareObservation
from radar_core import CCD_PIXEL_MAX, CCD_PIXEL_MIN


@dataclass(frozen=True)
class DeviceObservation:
    source: str
    sequence: int
    timestamp_us: float
    uncertainty_s: float = 0.0
    distance: float | None = None
    pixel: int = 0
    status: str = "ok"
    is_echo: bool = True

    def normalize(self, clock, arrival):
        timestamp, error = clock.map(self.timestamp_us, arrival)
        return HardwareObservation(self.source, self.sequence, timestamp, self.distance,
                                   self.status, error + self.uncertainty_s, self.pixel, self.is_echo)


def parse_observation(source, line, session, calibration, config):
    parts = line.split()
    if len(parts) < 2 or parts[1] != session:
        return None
    if source == "measurement" and parts[0] == "PIX" and len(parts) == 6:
        sequence, begin, end, pixel = map(int, parts[2:])
        if begin < 0 or end < begin or end - begin > 250000:
            raise ValueError("无效采集区间")
        pixel_min = int(config.get("pixel_min", CCD_PIXEL_MIN))
        pixel_max = int(config.get("pixel_max", CCD_PIXEL_MAX))
        if pixel == -1:
            return DeviceObservation("range", sequence, (begin + end) / 2, (end - begin) * 0.5e-6,
                                     None, pixel, "no_return", False)
        if not pixel_min <= pixel <= pixel_max:
            return DeviceObservation("range", sequence, (begin + end) / 2, (end - begin) * 0.5e-6,
                                     None, pixel, "invalid_pixel", False)
        distance = calibration.distance(pixel)
        if distance is None or not math.isfinite(distance):
            return DeviceObservation("range", sequence, (begin + end) / 2, (end - begin) * 0.5e-6,
                                     None, pixel, "calibration_outside", False)
        minimum = float(config.get("min_range_m", 0.08))
        maximum = float(config.get("max_range_m", 3.0))
        status = "ok" if minimum <= distance <= maximum else "out_of_range"
        return DeviceObservation("range", sequence, (begin + end) / 2, (end - begin) * 0.5e-6,
                                 distance, pixel, status, status == "ok")
    if source == "rotation" and parts[0] == "TRIG" and len(parts) == 4:
        sequence, tick = map(int, parts[2:])
        if tick < 0:
            raise ValueError("无效零位时间")
        return DeviceObservation("rotation", sequence, tick,
                                 float(config.get("irq_timestamp_uncertainty_ms", 2)) / 1000)
    return None
