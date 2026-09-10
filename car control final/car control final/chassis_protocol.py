from __future__ import annotations

from dataclasses import dataclass
import binascii
import math
import re
import struct


TRANSLATION_MODES = frozenset("WSADQEZC")
ROTATION_MODES = frozenset("RF")
SUPPORTED_MODES = TRANSLATION_MODES | ROTATION_MODES
SUPPORTED_UNITS = frozenset({"CNT", "MM"})
DONE_REASONS = frozenset({"TARGET", "EMERGENCY", "TIMEOUT", "WRONG_DIRECTION"})
ERROR_CODES = frozenset(
    {"BAD_CMD", "BAD_MODE", "BAD_UNIT", "BAD_VALUE", "BUSY", "LINE_TOO_LONG"}
)
PROTOCOL_BANNER = "MECANUM UNIVERSAL V6.3 COMM READY"
EXPECTED_STARTUP_MARKERS = frozenset(
    {
        "RX_FRAME_GUARD=1 SERIAL=9600,8N1",
        "BRAKE_CAL=4 ALL8 SHORT-LONG",
        "DIST_CAL=1 MM=WSADQEZC",
        "COMM_GUARD=3 LOG=0 PING=1",
    }
)
STOP_SEQUENCE = b"!\r\nX\r\n"
FIRMWARE_MAX_COMMAND_BYTES = 47
INT32_MIN = -(2**31)
INT32_MAX = 2**31 - 1

_INTEGER_RE = re.compile(r"[-+]?[0-9]+\Z")
_POSITIVE_INTEGER_RE = re.compile(r"[0-9]+\Z")
_FLOAT_RE = re.compile(
    r"[-+]?(?:(?:[0-9]+(?:\.[0-9]*)?)|(?:\.[0-9]+))(?:[eE][-+]?[0-9]+)?\Z"
)
_NONCE_RE = re.compile(r"[0-9A-F]{8}\Z")
_CAPABILITY_PATTERNS = (
    ("rx", re.compile(r"RX_FRAME_GUARD=([0-9]+) SERIAL=([0-9]+),([0-9])([NEO])([0-9])\Z")),
    ("brake", re.compile(r"BRAKE_CAL=([0-9]+) ALL8 SHORT-LONG\Z")),
    ("distance", re.compile(r"DIST_CAL=([0-9]+) MM=([A-Z]+)\Z")),
    ("communication", re.compile(r"COMM_GUARD=([0-9]+) LOG=([01]) PING=([01])\Z")),
)


class ChassisProtocolError(ValueError):
    def __init__(self, code: str, message: str, raw: str = "") -> None:
        super().__init__(message)
        self.code = code
        self.raw = raw


@dataclass(frozen=True)
class ChassisAck:
    mode: str
    request_value: int
    unit: str
    raw: str

    @property
    def counts(self) -> int:
        return self.request_value

    @property
    def requested_counts(self) -> int:
        return self.request_value


@dataclass(frozen=True)
class ChassisDone:
    mode: str
    reason: str
    request_value: int
    unit: str
    target_counts: int | None
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
    def requested_counts(self) -> int:
        return self.request_value

    @property
    def wheels(self) -> tuple[int, int, int, int]:
        return self.q1, self.q2, self.q3, self.q4

    @property
    def request_identity(self) -> tuple[str, int, str]:
        return self.mode, self.request_value, self.unit


@dataclass(frozen=True)
class ChassisError:
    code: str
    raw: str


@dataclass(frozen=True)
class ChassisResult:
    nonce: str
    report: ChassisDone | None
    status: str | None
    raw: str


@dataclass(frozen=True)
class ChassisPong:
    nonce: str
    raw: str


@dataclass(frozen=True)
class ChassisLogStatus:
    enabled: bool
    raw: str


@dataclass(frozen=True)
class ChassisCapabilityMarker:
    name: str
    values: tuple[object, ...]
    raw: str

    @property
    def is_expected(self) -> bool:
        return self.raw in EXPECTED_STARTUP_MARKERS


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


def _validate_mode(mode: str) -> str:
    if not isinstance(mode, str) or len(mode) != 1:
        raise ChassisProtocolError("BAD_MODE", "底盘动作模式必须是一个字符")
    normalized = mode.upper()
    if normalized not in SUPPORTED_MODES:
        raise ChassisProtocolError("BAD_MODE", f"不支持底盘动作模式：{mode}")
    return normalized


def _validate_unit(unit: str, mode: str) -> str:
    if not isinstance(unit, str):
        raise ChassisProtocolError("BAD_UNIT", "底盘请求单位无效")
    normalized = unit.upper()
    if normalized not in SUPPORTED_UNITS:
        raise ChassisProtocolError("BAD_UNIT", f"不支持底盘请求单位：{unit}")
    if normalized == "MM" and mode not in TRANSLATION_MODES:
        raise ChassisProtocolError("BAD_UNIT", "R/F 旋转只支持 CNT")
    return normalized


def _parse_positive_integer(value: str, label: str, *, maximum: int = INT32_MAX) -> int:
    if not _POSITIVE_INTEGER_RE.fullmatch(value):
        raise ChassisProtocolError("BAD_VALUE", f"{label}必须是正整数")
    parsed = int(value, 10)
    if parsed <= 0 or parsed > maximum:
        raise ChassisProtocolError("BAD_VALUE", f"{label}超出有效范围")
    return parsed


def _parse_integer(value: str, label: str) -> int:
    if not _INTEGER_RE.fullmatch(value):
        raise ChassisProtocolError("BAD_VALUE", f"{label}必须是整数")
    parsed = int(value, 10)
    if parsed < INT32_MIN or parsed > INT32_MAX:
        raise ChassisProtocolError("BAD_VALUE", f"{label}超出有效范围")
    return parsed


def _parse_float(value: str, label: str) -> float:
    if not _FLOAT_RE.fullmatch(value):
        raise ChassisProtocolError("BAD_VALUE", f"{label}必须是有限数值")
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < INT32_MIN or parsed > INT32_MAX:
        raise ChassisProtocolError("BAD_VALUE", f"{label}超出有限编码器范围")
    return parsed


def _encode_command(text: str, max_command_bytes: int) -> bytes:
    try:
        payload = text.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ChassisProtocolError("BAD_CMD", "底盘命令必须为 ASCII") from exc
    if len(payload) > int(max_command_bytes):
        raise ChassisProtocolError("LINE_TOO_LONG", "底盘发送命令超过固件接收上限")
    return payload + b"\r\n"


def encode_move(
    mode: str,
    request_value: int,
    unit: str = "CNT",
    *,
    nonce: str | None = None,
    max_command_bytes: int = FIRMWARE_MAX_COMMAND_BYTES,
) -> bytes:
    normalized_mode = _validate_mode(mode)
    normalized_unit = _validate_unit(unit, normalized_mode)
    value = _parse_positive_integer(str(request_value), f"请求 {normalized_unit}")
    body = f"@MOVE,{normalized_mode},{value},{normalized_unit}"
    if nonce is not None:
        body = _checked_body(f"{body},{validate_nonce(nonce)}")
    return _encode_command(body, max_command_bytes)


def _checked_body(body: str) -> str:
    return f"{body},{binascii.crc_hqx(body.encode('ascii'), 0xFFFF):04X}"


def encode_result_query(
    nonce: str, *, max_command_bytes: int = FIRMWARE_MAX_COMMAND_BYTES
) -> bytes:
    return _encode_command(_checked_body(f"@RESULT,{validate_nonce(nonce)}"), max_command_bytes)


def validate_nonce(nonce: str) -> str:
    if not isinstance(nonce, str) or not _NONCE_RE.fullmatch(nonce):
        raise ChassisProtocolError("BAD_CMD", "PING 标记必须是 8 位大写十六进制")
    return nonce


def encode_ping(
    nonce: str, *, max_command_bytes: int = FIRMWARE_MAX_COMMAND_BYTES
) -> bytes:
    return _encode_command(f"@PING,{validate_nonce(nonce)}", max_command_bytes)


def encode_log(
    enabled: bool, *, max_command_bytes: int = FIRMWARE_MAX_COMMAND_BYTES
) -> bytes:
    return _encode_command(f"@LOG,{1 if enabled else 0}", max_command_bytes)


def _float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def firmware_fixed_counts_per_mm(counts_per_mm: float) -> int:
    try:
        value = _float32(float(counts_per_mm))
    except (TypeError, ValueError, OverflowError, struct.error) as exc:
        raise ChassisProtocolError("BAD_VALUE", "CNT/mm 系数必须是有限正数") from exc
    if not math.isfinite(value) or value < 0.0001 or value > 1000.0:
        raise ChassisProtocolError("BAD_VALUE", "CNT/mm 系数超出固件范围")
    scaled = _float32(value * _float32(10000.0))
    return int(_float32(scaled + _float32(0.5)))


def firmware_mm_to_counts(
    millimeters: int,
    counts_per_mm: float | None = None,
    *,
    fixed_counts_per_mm: int | None = None,
) -> int:
    mm = _parse_positive_integer(str(millimeters), "毫米请求")
    if fixed_counts_per_mm is None:
        if counts_per_mm is None:
            raise ChassisProtocolError("BAD_VALUE", "缺少 CNT/mm 系数")
        scale = firmware_fixed_counts_per_mm(counts_per_mm)
    else:
        scale = _parse_positive_integer(str(fixed_counts_per_mm), "定点 CNT/mm", maximum=10_000_000)
    counts = (mm * scale + 5000) // 10000
    if counts <= 0 or counts > INT32_MAX:
        raise ChassisProtocolError("BAD_VALUE", "毫米请求换算后的 CNT 超出固件范围")
    return counts


def parse_ack(line: str) -> ChassisAck:
    parts = line.split(",")
    if len(parts) != 4 or parts[0] != "@ACK":
        raise ChassisProtocolError("BAD_ACK", "ACK 字段不完整或多余", line)
    mode = _validate_mode(parts[1])
    unit = _validate_unit(parts[3], mode)
    request_value = _parse_positive_integer(parts[2], f"ACK {unit}")
    return ChassisAck(mode, request_value, unit, line)


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
    base_required = {
        "REQ", "UNIT", "BRAKE", "ENC", "DX", "DY", "DR", "DS",
        "Q1", "Q2", "Q3", "Q4",
    }
    if not base_required.issubset(fields):
        missing = ",".join(sorted(base_required - set(fields)))
        raise ChassisProtocolError("BAD_DONE", f"DONE 字段集合错误：缺少 {missing}", line)
    unit = _validate_unit(fields["UNIT"], mode)
    required = set(base_required)
    if unit == "MM":
        required.add("TARGET_CNT")
    if set(fields) != required:
        extra = ",".join(sorted(set(fields) - required))
        detail = f"存在未知字段 {extra}" if extra else "TARGET_CNT 仅允许用于 MM"
        raise ChassisProtocolError("BAD_DONE", f"DONE 字段集合错误：{detail}", line)
    request_value = _parse_positive_integer(fields["REQ"], "DONE REQ")
    target_counts = (
        _parse_positive_integer(fields["TARGET_CNT"], "DONE TARGET_CNT")
        if unit == "MM"
        else None
    )
    return ChassisDone(
        mode=mode,
        reason=reason,
        request_value=request_value,
        unit=unit,
        target_counts=target_counts,
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


def parse_result(line: str) -> ChassisResult:
    parts = line.split(",")
    if len(parts) not in {4, 13} or parts[0] != "@RESULT":
        raise ChassisProtocolError("BAD_RESULT", "RESULT 字段不完整或多余", line)
    body, check = line.rsplit(",", 1)
    try:
        valid_crc = bool(re.fullmatch(r"[0-9A-F]{4}", check)) and (
            binascii.crc_hqx(body.encode("ascii"), 0xFFFF) == int(check, 16)
        )
    except UnicodeEncodeError:
        valid_crc = False
    if not valid_crc:
        raise ChassisProtocolError("BAD_CRC", "RESULT CRC 校验失败", line)
    nonce = validate_nonce(parts[1])
    if len(parts) == 4:
        if parts[2] not in {"N", "B"}:
            raise ChassisProtocolError("BAD_RESULT", "RESULT 状态无效", line)
        return ChassisResult(nonce, None, parts[2], line)
    mode, reason, unit = parts[2], parts[3], parts[5]
    if mode not in SUPPORTED_MODES or unit not in SUPPORTED_UNITS:
        raise ChassisProtocolError("BAD_RESULT", "RESULT 模式或单位无效", line)
    _validate_unit(unit, mode)
    reasons = {"0": "TARGET", "1": "EMERGENCY", "2": "TIMEOUT", "3": "WRONG_DIRECTION"}
    if reason not in reasons:
        raise ChassisProtocolError("BAD_REASON", "RESULT 停止原因无效", line)
    request = _parse_positive_integer(parts[4], "RESULT REQ")
    brake = _parse_float(parts[6], "RESULT BRAKE")
    enc = _parse_float(parts[7], "RESULT ENC")
    q1, q2, q3, q4 = (_parse_integer(parts[8 + i], f"RESULT Q{i + 1}") for i in range(4))
    report = ChassisDone(
        mode, reasons[reason], request, unit, None, brake, enc,
        (q1 + q2 + q3 + q4) / 4.0, (q1 - q2 + q3 - q4) / 4.0,
        (-q1 + q2 + q3 - q4) / 4.0, (q1 + q2 - q3 - q4) / 4.0,
        q1, q2, q3, q4, line,
    )
    return ChassisResult(nonce, report, None, line)


def validate_mm_target_counts(
    report: ChassisDone,
    counts_per_mm: float | None = None,
    *,
    fixed_counts_per_mm: int | None = None,
) -> tuple[bool, str]:
    if report.unit != "MM" or report.target_counts is None:
        return False, "报告不是完整 MM DONE"
    try:
        expected = firmware_mm_to_counts(
            report.request_value,
            counts_per_mm,
            fixed_counts_per_mm=fixed_counts_per_mm,
        )
    except ChassisProtocolError as exc:
        return False, str(exc)
    if report.target_counts != expected:
        return False, f"TARGET_CNT 应为 {expected}，实际为 {report.target_counts}"
    return True, ""


def validate_done_kinematics(
    report: ChassisDone, tolerance: float = 0.75
) -> tuple[bool, str]:
    if not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError("运动学容差必须是非负有限数")
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


def _parse_capability_marker(raw: str) -> ChassisFrame | None:
    for name, pattern in _CAPABILITY_PATTERNS:
        match = pattern.fullmatch(raw)
        if match is not None:
            values: tuple[object, ...]
            if name == "rx":
                values = (
                    int(match.group(1)), int(match.group(2)), int(match.group(3)),
                    match.group(4), int(match.group(5)),
                )
            elif name == "brake":
                values = (int(match.group(1)),)
            elif name == "distance":
                values = (int(match.group(1)), frozenset(match.group(2)))
            else:
                values = (int(match.group(1)), bool(int(match.group(2))), bool(int(match.group(3))))
            marker = ChassisCapabilityMarker(name, values, raw)
            return ChassisFrame("capability", marker, raw)
    prefixes = ("RX_FRAME_GUARD=", "BRAKE_CAL=", "DIST_CAL=", "COMM_GUARD=")
    if raw.startswith(prefixes):
        return ChassisFrame("invalid", None, raw, "底盘能力标记格式无效")
    return None


def parse_protocol_line(line: str) -> ChassisFrame:
    raw = line.rstrip("\r\n")
    if not raw:
        return ChassisFrame("empty", None, raw)
    if raw.startswith("@CFG,"):
        return ChassisFrame("config1", None, raw)
    if raw == "CONFIG=1 RAM=1 CRC=CCITT":
        return ChassisFrame("diagnostic", raw, raw)
    if raw == "RESULT=1 CRC=CCITT QUERY=1":
        return ChassisFrame("diagnostic", raw, raw)
    if raw == PROTOCOL_BANNER:
        return ChassisFrame("banner", raw, raw)
    capability = _parse_capability_marker(raw)
    if capability is not None:
        return capability
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
    if raw.startswith("@RESULT"):
        try:
            return ChassisFrame("result", parse_result(raw), raw)
        except ChassisProtocolError as exc:
            return ChassisFrame("invalid", None, raw, str(exc))
    if raw.startswith("@PONG"):
        parts = raw.split(",")
        if len(parts) == 2 and parts[0] == "@PONG" and _NONCE_RE.fullmatch(parts[1]):
            return ChassisFrame("pong", ChassisPong(parts[1], raw), raw)
        return ChassisFrame("invalid", None, raw, "PONG 格式无效")
    if raw.startswith("@LOG"):
        parts = raw.split(",")
        if len(parts) == 2 and parts[0] == "@LOG" and parts[1] in {"0", "1"}:
            return ChassisFrame("log", ChassisLogStatus(parts[1] == "1", raw), raw)
        return ChassisFrame("invalid", None, raw, "LOG 回复格式无效")
    if raw.startswith("@ERR"):
        parts = raw.split(",")
        if len(parts) == 2 and parts[0] == "@ERR" and parts[1] in ERROR_CODES:
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

    def finalize(self) -> list[ChassisFrame]:
        if self._discarding:
            frame = ChassisFrame("invalid", None, "", "底盘接收行超过长度上限且未结束")
        elif self._buffer:
            raw = bytes(self._buffer).decode("ascii", "backslashreplace")
            frame = ChassisFrame("invalid", None, raw, "底盘回复在行尾前截断")
        else:
            return []
        self.reset()
        return [frame]

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
                        raw = bytes(self._buffer).decode("ascii", "backslashreplace")
                        frames.append(ChassisFrame("invalid", None, raw, "底盘回复包含非 ASCII 字节"))
                    else:
                        # A diagnostic may lose its terminator before a complete
                        # protocol frame. Keep the prefix and validate the frame
                        # normally; a damaged protocol line is never spliced.
                        frame_start = line.find("@")
                        if frame_start > 0:
                            frames.append(parse_protocol_line(line[:frame_start]))
                            line = line[frame_start:]
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
