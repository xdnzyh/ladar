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
    session: str | None = None
    calibration_version: str | None = None
    distance_error_m: float | None = None

    def normalize(self, clock, arrival):
        timestamp, error = clock.map(self.timestamp_us, arrival)
        raw_time = float(self.timestamp_us)
        return HardwareObservation(
            self.source,
            self.sequence,
            timestamp,
            self.distance,
            self.status,
            error + self.uncertainty_s,
            self.pixel,
            self.is_echo,
            raw_timestamp_us=raw_time,
            clock_model_version=getattr(clock, "version", None),
            source_session=self.session,
            calibration_version=self.calibration_version,
            distance_error_m=self.distance_error_m,
        )


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
                                     None, pixel, "no_return", False, session=session)
        if not pixel_min <= pixel <= pixel_max:
            return DeviceObservation("range", sequence, (begin + end) / 2, (end - begin) * 0.5e-6,
                                     None, pixel, "invalid_pixel", False, session=session)
        distance = calibration.distance(pixel)
        if distance is None or not math.isfinite(distance):
            return DeviceObservation("range", sequence, (begin + end) / 2, (end - begin) * 0.5e-6,
                                     None, pixel, "calibration_outside", False, session=session)
        minimum = float(config.get("min_range_m", 0.08))
        maximum = float(config.get("max_range_m", 3.0))
        status = "ok" if minimum <= distance <= maximum else "out_of_range"
        distance_error = calibration.pixel_quantization_error_m(pixel)
        return DeviceObservation(
            "range", sequence, (begin + end) / 2, (end - begin) * 0.5e-6,
            distance, pixel, status, status == "ok", session,
            str(getattr(calibration, "identifier", getattr(calibration, "model", "unknown"))), distance_error,
        )
    if source == "rotation" and parts[0] == "TRIG" and len(parts) == 4:
        sequence, tick = map(int, parts[2:])
        if tick < 0:
            raise ValueError("无效零位时间")
        return DeviceObservation("rotation", sequence, tick,
                                 float(config.get("irq_timestamp_uncertainty_ms", 2)) / 1000,
                                 session=session)
    return None
