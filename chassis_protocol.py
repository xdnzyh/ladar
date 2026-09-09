from __future__ import annotations

from dataclasses import dataclass
import math
import re


SUPPORTED_MODES = frozenset("WSADQEZCRF")
DONE_REASONS = frozenset({"TARGET", "EMERGENCY", "TIMEOUT", "WRONG_DIRECTION"})
PROTOCOL_BANNER = "MECANUM UNIVERSAL V6.3 COMM READY"
STOP_SEQUENCE = b"!\r\nX\r\n"

_INTEGER_RE = re.compile(r"[-+]?\d+\Z")
_POSITIVE_INTEGER_RE = re.compile(r"\d+\Z")
_FLOAT_RE = re.compile(
    r"[-+]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][-+]?\d+)?\Z"
)


class ChassisProtocolError(ValueError):
    def __init__(self, code: str, message: str, raw: str = "") -> None:
        super().__init__(message)
        self.code = code
        self.raw = raw


@dataclass(frozen=True)
class ChassisAck:
    mode: str
    counts: int
    raw: str


@dataclass(frozen=True)
class ChassisDone:
    mode: str
    reason: str
    requested_counts: int
    unit: str
    brake: float
    enc: float
    dx: float
    dy: float
    dr: float
    ds: float
    q1: int
    q2: int
    q3: int
    q4: int
    raw: str

    @property
    def wheels(self) -> tuple[int, int, int, int]:
        return self.q1, self.q2, self.q3, self.q4


@dataclass(frozen=True)
class ChassisError:
    code: str
    raw: str


@dataclass(frozen=True)
class IdleStatus:
    x: float
    y: float
    r: float
    s: float
    raw: str


@dataclass(frozen=True)
class ChassisFrame:
    kind: str
    value: object | None
    raw: str
    error: str | None = None


def encode_move(mode: str, counts: int) -> bytes:
    normalized_mode = _validate_mode(mode)
    normalized_counts = _parse_positive_integer(str(counts), "请求 CNT")
    return f"@MOVE,{normalized_mode},{normalized_counts},CNT\r\n".encode("ascii")


def _validate_mode(mode: str) -> str:
    if not isinstance(mode, str) or len(mode) != 1:
        raise ChassisProtocolError("BAD_MODE", "底盘动作模式必须是一个字符")
    normalized = mode.upper()
    if normalized not in SUPPORTED_MODES:
        raise ChassisProtocolError("BAD_MODE", f"不支持底盘动作模式：{mode}")
    return normalized


def _parse_positive_integer(value: str, label: str) -> int:
    if not _POSITIVE_INTEGER_RE.fullmatch(value):
        raise ChassisProtocolError("BAD_VALUE", f"{label}必须是正整数")
    parsed = int(value, 10)
    if parsed <= 0:
        raise ChassisProtocolError("BAD_VALUE", f"{label}必须大于 0")
    return parsed


def _parse_integer(value: str, label: str) -> int:
    if not _INTEGER_RE.fullmatch(value):
        raise ChassisProtocolError("BAD_VALUE", f"{label}必须是整数")
    return int(value, 10)


def _parse_float(value: str, label: str) -> float:
    if not _FLOAT_RE.fullmatch(value):
        raise ChassisProtocolError("BAD_VALUE", f"{label}必须是有限数值")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ChassisProtocolError("BAD_VALUE", f"{label}必须是有限数值")
    return parsed


def parse_ack(line: str) -> ChassisAck:
    parts = line.split(",")
    if len(parts) != 4 or parts[0] != "@ACK" or parts[3] != "CNT":
        raise ChassisProtocolError("BAD_ACK", "ACK 字段不完整或多余", line)
    mode = _validate_mode(parts[1])
    counts = _parse_positive_integer(parts[2], "ACK CNT")
    return ChassisAck(mode, counts, line)


def parse_done(line: str) -> ChassisDone:
    parts = line.split(",")
    if len(parts) < 4 or parts[0] != "@DONE":
        raise ChassisProtocolError("BAD_DONE", "DONE 前缀或字段不完整", line)
    mode = _validate_mode(parts[1])
    reason = parts[2]
    if reason not in DONE_REASONS:
        raise ChassisProtocolError("BAD_REASON", f"未知 DONE 原因：{reason}", line)
    fields: dict[str, str] = {}
    for item in parts[3:]:
        if "=" not in item:
            raise ChassisProtocolError("BAD_DONE", "DONE 存在无名字段", line)
        key, value = item.split("=", 1)
        if not key or key in fields:
            raise ChassisProtocolError("BAD_DONE", "DONE 字段重复或为空", line)
        fields[key] = value
    required = {
        "REQ", "UNIT", "BRAKE", "ENC", "DX", "DY", "DR", "DS",
        "Q1", "Q2", "Q3", "Q4",
    }
    if set(fields) != required:
        missing = ",".join(sorted(required - set(fields)))
        extra = ",".join(sorted(set(fields) - required))
        detail = f"缺少 {missing}" if missing else f"存在未知字段 {extra}"
        raise ChassisProtocolError("BAD_DONE", f"DONE 字段集合错误：{detail}", line)
    if fields["UNIT"] != "CNT":
        raise ChassisProtocolError("BAD_UNIT", "DONE 单位不是 CNT", line)
    requested_counts = _parse_positive_integer(fields["REQ"], "DONE REQ")
    return ChassisDone(
        mode=mode,
        reason=reason,
        requested_counts=requested_counts,
        unit="CNT",
        brake=_parse_float(fields["BRAKE"], "DONE BRAKE"),
        enc=_parse_float(fields["ENC"], "DONE ENC"),
        dx=_parse_float(fields["DX"], "DONE DX"),
        dy=_parse_float(fields["DY"], "DONE DY"),
        dr=_parse_float(fields["DR"], "DONE DR"),
        ds=_parse_float(fields["DS"], "DONE DS"),
        q1=_parse_integer(fields["Q1"], "DONE Q1"),
        q2=_parse_integer(fields["Q2"], "DONE Q2"),
        q3=_parse_integer(fields["Q3"], "DONE Q3"),
        q4=_parse_integer(fields["Q4"], "DONE Q4"),
        raw=line,
    )


def validate_done_kinematics(report: ChassisDone, tolerance: float = 0.75) -> tuple[bool, str]:
    q1, q2, q3, q4 = report.wheels
    expected = {
        "DX": (q1 + q2 + q3 + q4) / 4.0,
        "DY": (q1 - q2 + q3 - q4) / 4.0,
        "DR": (-q1 + q2 + q3 - q4) / 4.0,
        "DS": (q1 + q2 - q3 - q4) / 4.0,
    }
    actual = {"DX": report.dx, "DY": report.dy, "DR": report.dr, "DS": report.ds}
    for key, expected_value in expected.items():
        if abs(actual[key] - expected_value) > max(tolerance, abs(expected_value) * 0.002):
            return False, f"{key} 与 Q1~Q4 不一致"
    active_direction = {
        "W": (1, 1, 1, 1),
        "S": (-1, -1, -1, -1),
        "A": (1, -1, 1, -1),
        "D": (-1, 1, -1, 1),
        "Q": (1, 0, 1, 0),
        "E": (0, 1, 0, 1),
        "Z": (0, -1, 0, -1),
        "C": (-1, 0, -1, 0),
        "R": (-1, 1, 1, -1),
        "F": (1, -1, -1, 1),
    }[report.mode]
    active = [direction * wheel for direction, wheel in zip(active_direction, report.wheels) if direction]
    expected_enc = sum(active) / len(active)
    if abs(report.enc - expected_enc) > max(1.0, abs(expected_enc) * 0.003):
        return False, "ENC 与主动轮计数不一致"
    return True, ""


def parse_idle_status(line: str) -> IdleStatus | None:
    parts = line.split()
    if len(parts) != 5 or parts[0] != "IDLE":
        return None
    values: dict[str, float] = {}
    for item in parts[1:]:
        if "=" not in item:
            return None
        key, value = item.split("=", 1)
        if key in values:
            return None
        try:
            values[key] = _parse_float(value, key)
        except ChassisProtocolError:
            return None
    if set(values) != {"X", "Y", "R", "S"}:
        return None
    return IdleStatus(values["X"], values["Y"], values["R"], values["S"], line)


def parse_protocol_line(line: str) -> ChassisFrame:
    raw = line.rstrip("\r\n")
    if not raw:
        return ChassisFrame("empty", None, raw)
    if raw == PROTOCOL_BANNER:
        return ChassisFrame("banner", raw, raw)
    idle = parse_idle_status(raw)
    if idle is not None:
        return ChassisFrame("idle", idle, raw)
    if raw.startswith("@ACK"):
        try:
            return ChassisFrame("ack", parse_ack(raw), raw)
        except ChassisProtocolError as exc:
            return ChassisFrame("invalid", None, raw, str(exc))
    if raw.startswith("@DONE"):
        try:
            return ChassisFrame("done", parse_done(raw), raw)
        except ChassisProtocolError as exc:
            return ChassisFrame("invalid", None, raw, str(exc))
    if raw.startswith("@ERR"):
        parts = raw.split(",")
        if len(parts) == 2 and parts[0] == "@ERR" and parts[1]:
            return ChassisFrame("error", ChassisError(parts[1], raw), raw)
        return ChassisFrame("invalid", None, raw, "底盘错误回复格式无效")
    return ChassisFrame("diagnostic", raw, raw)


class ChassisStreamParser:
    def __init__(self, max_line_bytes: int = 1024) -> None:
        if max_line_bytes < 64:
            raise ValueError("底盘接收行上限过小")
        self.max_line_bytes = int(max_line_bytes)
        self._buffer = bytearray()
        self._discarding = False

    def reset(self) -> None:
        self._buffer.clear()
        self._discarding = False

    def feed(self, data: bytes | bytearray | memoryview) -> list[ChassisFrame]:
        frames: list[ChassisFrame] = []
        for byte in bytes(data):
            if byte in (10, 13):
                if self._discarding:
                    frames.append(ChassisFrame("invalid", None, "", "底盘接收行超过长度上限"))
                elif self._buffer:
                    try:
                        line = self._buffer.decode("ascii")
                    except UnicodeDecodeError:
                        frames.append(ChassisFrame("invalid", None, "", "底盘回复包含非 ASCII 字节"))
                    else:
                        frames.append(parse_protocol_line(line))
                self._buffer.clear()
                self._discarding = False
                continue
            if self._discarding:
                continue
            if len(self._buffer) >= self.max_line_bytes:
                self._discarding = True
                continue
            self._buffer.append(byte)
        return frames

