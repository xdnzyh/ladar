"""CONFIG1 profiles and nonblocking transactions on the existing chassis connection."""
from __future__ import annotations

import binascii
from decimal import Decimal
import json
from pathlib import Path
import re
import secrets
import struct

ROOT = Path(__file__).resolve().parent
DEFAULT_PROFILE = "control/参数工具/底盘参数.json"


def schema():
    source = (ROOT / "control/runtime_parameters.h").read_text(encoding="utf-8-sig")
    rows = re.findall(r"CAR_PARAMETER\((\w+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\)", source)
    if len(rows) != 95:
        raise ValueError("CONFIG1 参数表版本不匹配")
    return [(name, *map(int, numbers)) for name, *numbers in rows]


def crc(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def values_crc(values) -> int:
    return crc(struct.pack("<95H", *values))


def encode(body: str) -> bytes:
    data = body.encode("ascii")
    return data + f",{crc(data):04X}\r\n".encode("ascii")


def decode(raw: str):
    body, check = raw.rsplit(",", 1)
    if not re.fullmatch(r"[0-9A-F]{4}", check) or crc(body.encode("ascii")) != int(check, 16):
        raise ValueError("CONFIG1 回复 CRC 错误")
    parts = body.split(",")
    if len(parts) < 4 or parts[0] != "@CFG" or not re.fullmatch(r"[0-9A-F]{8}", parts[2]):
        raise ValueError("CONFIG1 回复格式错误")
    if any(not re.fullmatch(r"[0-9]+", value) for value in parts[3:]):
        raise ValueError("CONFIG1 回复数值错误")
    return parts[1], parts[2], tuple(map(int, parts[3:]))


def validate(values):
    rows = schema()
    if len(values) != 95:
        raise ValueError("CONFIG1 必须包含 95 项参数")
    for value, (name, _, low, high, _) in zip(values, rows):
        if type(value) is not int or not low <= value <= high:
            raise ValueError(f"{name} 超出范围")
    for b in range(0, 44, 11):
        if not values[b] >= values[b+1] >= values[b+2] >= values[b+6]:
            raise ValueError("速度必须满足 FAST≥MID≥SLOW≥START")
        if values[b+9] > values[b+10] or any(
            not values[b+9] <= values[b+k] <= values[b+10] for k in (3, 4, 5, 7)
        ):
            raise ValueError("前馈必须处于 PWM 范围内")
    for b in range(44, 76, 4):
        if values[b] >= values[b+1] or values[b+2]*2 >= values[b] or values[b+3]*2 >= values[b+1]:
            raise ValueError("刹车目标区间或提前量无效")
    if values[80] >= values[79] or values[88] > values[89]:
        raise ValueError("降速区间或航向修正组合无效")


def load_profile(relative_path):
    path = (ROOT / relative_path).resolve()
    if not path.is_relative_to(ROOT) or path == ROOT:
        raise ValueError("底盘参数文件必须位于本项目目录内")
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("底盘参数 JSON 顶层必须是对象")
    if type(data.get("schema")) is not int or data["schema"] != 1:
        raise ValueError("底盘参数 JSON schema 必须为 1")
    parameters = data["parameters"]
    rows = schema()
    if not isinstance(parameters, dict) or set(parameters) != {row[0] for row in rows}:
        raise ValueError("底盘参数 JSON 的参数名称不完整或存在未知项")
    values = []
    for name, _, _, _, scale in rows:
        value = parameters[name]
        if type(value) not in (int, float):
            raise ValueError(f"{name} 必须是数值")
        scaled = Decimal(str(value)) * scale
        if not scaled.is_finite() or scaled != scaled.to_integral_value():
            raise ValueError(f"{name} 小数精度无效")
        values.append(int(scaled))
    validate(values)
    coefficients = data["counts_per_mm"]
    if not isinstance(coefficients, dict) or set(coefficients) != set("WSADQEZC"):
        raise ValueError("距离系数必须包含八个平移方向")
    for mode, value in coefficients.items():
        if type(value) not in (int, float):
            raise ValueError(f"{mode} 距离系数必须是数值")
        number = Decimal(str(value))
        if not number.is_finite() or not Decimal("0.0001") <= number <= 1000 or number*10000 != (number*10000).to_integral_value():
            raise ValueError(f"{mode} 距离系数必须在 0.0001～1000，最多四位小数")
    return values, coefficients


def _info():
    result = yield ("I", ())
    if len(result) != 3 or result[:2] != (1, 95):
        raise ValueError("设备不支持预期的 CONFIG1/95 参数")
    return result[2]


def _read():
    first = yield from _info()
    values = []
    for index in range(0, 95, 8):
        page = yield ("G", (index,))
        if len(page) != 1 + min(8, 95-index) or page[0] != index:
            raise ValueError("CONFIG1 分页回复不匹配")
        values.extend(page[1:])
    last = yield from _info()
    validate(values)
    if first != last or last != values_crc(values):
        raise ValueError("CONFIG1 读取期间参数变化或数据不完整")
    return values


def transaction(target, operation):
    target = list(target)
    validate(target)
    expected = values_crc(target)
    current = yield from _read()
    if operation == "apply" and current != target:
        if (yield ("B", ())) != (values_crc(current),):
            raise ValueError("CONFIG1 暂存开始时参数已变化")
        for index, (old, new) in enumerate(zip(current, target)):
            if old != new and (yield ("S", (index, new))) != (index, new):
                raise ValueError("CONFIG1 暂存回复不匹配")
        committed = yield ("C", (expected,))
        if committed is not None and committed != (expected,):
            raise ValueError("CONFIG1 提交回复不匹配，请重新读取")
        current = yield from _read()
        if current != target:
            raise ValueError("CONFIG1 提交后读回不匹配")
    return current


class ConfigExchange:
    """Bounded I/G/B/S recovery; commit once, then verify all actual values."""

    def __init__(self, target, operation, send, finish, now):
        self.tag = secrets.token_hex(4).upper()
        self.request_tag = self.tag
        self.retries = 0
        self.steps = transaction(target, operation)
        self.send = send
        self.finish = finish
        self.command = next(self.steps)
        self.waiting = False
        self.deadline = now + 4.0
        self.finished = False

    def fail(self, message):
        if not self.finished:
            self.finished = True
            self.finish(None, message)

    def poll(self, now, last_rx):
        if self.finished:
            return
        if now >= self.deadline:
            op, _ = self.command
            if self.waiting and op in ("I", "G", "B", "S") and self.retries < 2:
                self.retries += 1
                self.waiting = False
                self.deadline = now + 4.0
            elif self.waiting and op == "C":
                # An unconfirmed commit may already have changed RAM. Never resend it.
                self._advance(None, now)
            else:
                self.fail("CONFIG1 等待超时；未发送移动，提交结果请重新读取")
        elif not self.waiting and now - last_rx >= 0.5:
            op, arguments = self.command
            self.request_tag = secrets.token_hex(4).upper() if op in ("I", "G") else self.tag
            body = f"@CFG,{op},{self.request_tag}" + "".join(f",{arg}" for arg in arguments)
            self.waiting = True
            self.deadline = now + 4.0
            if not self.send(encode(body)):
                self.fail("CONFIG1 写入失败")

    def feed(self, raw, now):
        if self.finished:
            return
        try:
            op, tag, values = decode(raw)
            if tag != self.request_tag:
                return
            if not self.waiting:
                return
            if op == "E":
                raise ValueError(f"CONFIG1 设备错误 {values}")
            if op != self.command[0]:
                return
            # Old G/S replies from the same transaction must not satisfy a new index.
            if op in ("G", "S") and (not values or values[0] != self.command[1][0]):
                return
            self._advance(values, now)
        except (ValueError, UnicodeError, struct.error) as exc:
            # Damaged frames do not confirm a command; wait for its bounded timeout.
            # Explicit device rejection is definitive and must never be retried.
            if str(exc).startswith("CONFIG1 设备错误"):
                self.fail(str(exc))

    def _advance(self, values, now):
        try:
            self.command = self.steps.send(values)
            self.retries = 0
            self.waiting = False
            self.deadline = now + 4.0
        except StopIteration as done:
            self.finished = True
            self.finish(done.value, None)
        except (ValueError, UnicodeError, struct.error) as exc:
            self.fail(str(exc))
