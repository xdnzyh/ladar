"""Calibration and record handling. No GUI or hardware dependencies."""
from dataclasses import dataclass
import csv
import hashlib
import math
from pathlib import Path
import re

# The original ruler readings used the CCD housing as the reference point.
# The final measurement reference is the laser spot, 2 cm closer to the wall.
DEFAULT_POINTS = ((8, 1211), (13, 1100), (18, 1025), (23, 982),
                  (28, 942), (33, 921), (38, 901), (43, 886),
                  (48, 871), (53, 862), (58, 855))
COORDINATE_PATTERN = re.compile(r"CCD MIN X:\s*(\d+)\s*\Z")


def parse_pix_line(line):
    """Parse the synchronized firmware's PIX session/sequence packet."""
    parts = line.strip().split()
    if len(parts) != 6 or parts[0] != "PIX":
        return None
    try:
        sample_id = int(parts[2])
        begin_us = int(parts[3])
        end_us = int(parts[4])
        pixel = int(parts[5])
    except ValueError as error:
        raise ValueError("PIX 字段不是整数") from error
    if sample_id < 0 or begin_us < 0 or end_us < begin_us:
        raise ValueError("PIX 时间或序号无效")
    if pixel != -1 and not 0 <= pixel <= 1500:
        raise ValueError("PIX 坐标超出 0～1500")
    return {"session_id": parts[1], "sample_id": sample_id,
            "device_begin_us": begin_us, "device_end_us": end_us,
            "ccd_x": None if pixel == -1 else pixel}


def parse_angle_line(line):
    """Parse a future transport packet: ANGLE/TRIG session sequence angle."""
    parts = line.strip().split()
    if len(parts) not in (4, 5) or parts[0] not in ("ANGLE", "TRIG"):
        return None
    try:
        sample_id = int(parts[2])
        angle = float(parts[3])
        trigger_id = parts[4] if len(parts) == 5 else ""
    except ValueError as error:
        raise ValueError("ANGLE/TRIG 字段无效") from error
    if sample_id < 0 or not math.isfinite(angle):
        raise ValueError("ANGLE/TRIG 序号或角度无效")
    return {"session_id": parts[1], "sample_id": sample_id,
            "angle_deg": angle, "trigger_id": trigger_id}


@dataclass(frozen=True)
class Calibration:
    points: tuple = DEFAULT_POINTS

    def __post_init__(self):
        rows = tuple(sorted((float(d), float(x)) for d, x in self.points))
        if len(rows) < 2:
            raise ValueError("标定表至少需要两个点")
        if any(not math.isfinite(v) for row in rows for v in row):
            raise ValueError("标定值必须是有限数值")
        if any(d <= 0 or not 0 <= x <= 1500 for d, x in rows):
            raise ValueError("距离必须大于 0，坐标必须在 0～1500 内")
        if any(d1 >= d2 or x1 <= x2 for (d1, x1), (d2, x2) in zip(rows, rows[1:])):
            raise ValueError("距离必须严格递增，坐标必须严格递减，不能有重复值")
        object.__setattr__(self, "points", rows)

    @property
    def identifier(self):
        return hashlib.sha256(repr(self.points).encode()).hexdigest()[:10]

    def convert(self, x):
        if not math.isfinite(x) or not 0 <= x <= 1500:
            return None, "无效坐标"
        if x > self.points[0][1]:
            return None, "超出标定范围（坐标偏大）"
        if x < self.points[-1][1]:
            return None, "超出标定范围（坐标偏小）"
        for (d1, x1), (d2, x2) in zip(self.points, self.points[1:]):
            if x2 <= x <= x1:
                return d1 + (x1 - x) / (x1 - x2) * (d2 - d1), "范围内"
        return None, "无效坐标"

    @classmethod
    def from_csv(cls, path):
        with open(path, encoding="utf-8-sig", newline="") as file:
            reader = csv.DictReader(file)
            if not {"distance_cm", "ccd_x"}.issubset(reader.fieldnames or []):
                raise ValueError("CSV 表头必须包含 distance_cm,ccd_x")
            rows = []
            for row in reader:
                if len(rows) >= 1000:
                    raise ValueError("标定表最多支持 1000 个点")
                rows.append((float(row["distance_cm"]), float(row["ccd_x"])))
        return cls(tuple(rows))

    def save(self, path):
        with open(path, "w", encoding="utf-8-sig", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(("distance_cm", "ccd_x"))
            writer.writerows(self.points)


def parse_coordinate_line(line):
    match = COORDINATE_PATTERN.fullmatch(line.strip())
    if not match:
        return None
    x = int(match.group(1))
    if not 0 <= x <= 1500:
        raise ValueError("CCD 返回坐标超出 0～1500")
    return x


def parse_reference(text):
    if not text.strip():
        return None
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("尺量距离应是大于 0 的有限数值，或留空")
    return value


FIELDS = ("sequence", "source", "port", "pc_request_time", "pc_receive_time",
          "round_trip_ms", "session_id", "sample_id", "device_begin_us",
          "device_end_us", "angle_deg", "trigger_id", "ccd_x", "distance_cm",
          "reference_cm", "error_cm", "status", "calibration_id",
          "calibration_points", "exposure_sent")


def write_records(path, records):
    with open(path, "w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS, extrasaction="ignore")
        writer.writeheader()
        for record in records:
            writer.writerow({key: record.get(key, "") for key in FIELDS})


class LineBuffer:
    """Preserve fragments across serial timeouts; bound malformed input."""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        lines = []
        for byte in data:
            if byte in (10, 13):
                if self.buffer:
                    try:
                        lines.append(self.buffer.decode("utf-8"))
                    finally:
                        self.buffer.clear()
            else:
                self.buffer.append(byte)
                if len(self.buffer) > 1024:
                    self.buffer.clear()
                    raise ValueError("接收行过长；请检查串口和固件")
        return lines
