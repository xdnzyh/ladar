from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, Mapping

from chassis_protocol import (
    ChassisAck,
    ChassisDone,
    ChassisFrame,
    ChassisStreamParser,
    ChassisProtocolError,
    STOP_SEQUENCE,
    validate_done_kinematics,
    encode_move,
)
from navigation_core import VelocityCommand


class MotionConversionError(ValueError):
    pass


@dataclass(frozen=True)
class ChassisMoveRequest:
    mode: str
    counts: int
    target: float
    target_unit: str


@dataclass(frozen=True)
class ExecutionEstimate:
    mode: str
    reason: str
    local_x_m: float
    local_y_m: float
    yaw_rad: float
    uncertainty_m: float
    trusted: bool
    report: ChassisDone


class ChassisMotionAdapter:
    """Converts one planner intention to one calibrated firmware action."""

    _MODE_SIGNS = {
        "W": (0.0, 1.0, 0.0),
        "S": (0.0, -1.0, 0.0),
        "A": (-1.0, 0.0, 0.0),
        "D": (1.0, 0.0, 0.0),
        "Q": (-1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0),
        "E": (1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0), 0.0),
        "Z": (-1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0), 0.0),
        "C": (1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0), 0.0),
        "R": (0.0, 0.0, 1.0),
        "F": (0.0, 0.0, -1.0),
    }

    def __init__(self, config: Mapping[str, object]) -> None:
        self.config = config

    @staticmethod
    def mode_for_command(command: VelocityCommand, ratio_tolerance: float = 0.04) -> str:
        values = (command.forward_mps, command.right_mps, command.yaw_rps, command.duration_s)
        if not all(math.isfinite(float(value)) for value in values):
            raise MotionConversionError("底盘动作包含非有限数值")
        if command.duration_s <= 0 or command.stopped:
            raise MotionConversionError("底盘动作没有有效持续时间")
        translation = math.hypot(command.forward_mps, command.right_mps)
        turning = abs(command.yaw_rps)
        if translation > 1e-9 and turning > 1e-9:
            raise MotionConversionError("当前底盘不支持平移与旋转混合动作")
        if turning > 1e-9:
            return "R" if command.yaw_rps > 0 else "F"
        if abs(command.forward_mps) <= 1e-9:
            return "D" if command.right_mps > 0 else "A"
        if abs(command.right_mps) <= 1e-9:
            return "W" if command.forward_mps > 0 else "S"
        dominant = max(abs(command.forward_mps), abs(command.right_mps))
        minor = min(abs(command.forward_mps), abs(command.right_mps))
        if minor / dominant < 1.0 - ratio_tolerance:
            raise MotionConversionError("斜移动作必须是等幅 45° 方向")
        if command.forward_mps > 0:
            return "E" if command.right_mps > 0 else "Q"
        return "C" if command.right_mps > 0 else "Z"

    def _calibration_entry(self, mode: str) -> Mapping[str, object]:
        table = self.config.get("chassis_calibration", {})
        if not isinstance(table, Mapping):
            return {}
        entry = table.get(mode, {})
        return entry if isinstance(entry, Mapping) else {}

    @staticmethod
    def _finite_value(value: object) -> float | None:
        try:
            parsed = float(value)
        except (TypeError, ValueError, OverflowError):
            return None
        return parsed if math.isfinite(parsed) else None

    def _factor(self, mode: str) -> float | None:
        entry = self._calibration_entry(mode)
        key = "counts_per_rad" if mode in {"R", "F"} else "counts_per_m"
        value = entry.get("counts_per_unit", entry.get(key))
        factor = self._finite_value(value)
        return factor if factor is not None and factor > 0 else None

    def readiness(self, mode: str | None = None) -> tuple[bool, tuple[str, ...]]:
        reasons: list[str] = []
        if not bool(self.config.get("chassis_speed_validated", False)):
            reasons.append("底盘实际速度上界尚未测得")
        if not bool(self.config.get("chassis_braking_validated", False)):
            reasons.append("底盘制动距离和超调尚未测得")
        speed_bound = self._finite_value(self.config.get("safety_speed_upper_bound_mps"))
        if speed_bound is None or speed_bound <= 0:
            reasons.append("缺少经测量的底盘速度上界")
        stop_distance = self._finite_value(self.config.get("safety_stop_distance_m"))
        if stop_distance is None or stop_distance < 0:
            reasons.append("缺少经测量的保守停止距离")
        modes = (mode,) if mode is not None else tuple(self._MODE_SIGNS)
        for item in modes:
            entry = self._calibration_entry(item)
            status = str(entry.get("status", "uncalibrated")).lower()
            if status not in {"validated", "ready", "confirmed"}:
                reasons.append(f"{item} 方向尚未标定")
                continue
            if self._factor(item) is None:
                reasons.append(f"{item} 方向缺少有效 CNT 换算")
            uncertainty = self._finite_value(entry.get("uncertainty_m"))
            if item in {"R", "F"}:
                uncertainty = self._finite_value(entry.get("uncertainty_rad"))
            if uncertainty is None or uncertainty < 0:
                reasons.append(f"{item} 方向缺少执行不确定度")
        return not reasons, tuple(reasons)

    def request_for_command(self, command: VelocityCommand, *, automatic: bool = True) -> ChassisMoveRequest:
        mode = self.mode_for_command(command)
        if automatic:
            ready, reasons = self.readiness(mode)
            if not ready:
                raise MotionConversionError("；".join(reasons))
        factor = self._factor(mode)
        if factor is None:
            raise MotionConversionError(f"{mode} 方向尚未完成 CNT 标定")
        if mode in {"R", "F"}:
            target = abs(command.yaw_rps * command.duration_s)
            target_unit = "rad"
            maximum = self._finite_value(self.config.get("chassis_max_rotation_rad"))
        else:
            target = math.hypot(
                command.forward_mps * command.duration_s,
                command.right_mps * command.duration_s,
            )
            target_unit = "m"
            maximum = self._finite_value(self.config.get("chassis_max_translation_m"))
        if not math.isfinite(target) or target <= 0:
            raise MotionConversionError("底盘动作目标量无效")
        if maximum is not None and maximum > 0 and target > maximum + 1e-9:
            raise MotionConversionError(f"动作长度超过当前底盘安全上限 {maximum:g} {target_unit}")
        entry = self._calibration_entry(mode)
        bias = self._finite_value(entry.get("request_bias_counts")) or 0.0
        counts = int(round(target * factor + bias))
        minimum = self._finite_value(entry.get("min_counts"))
        maximum_counts = self._finite_value(entry.get("max_counts"))
        if minimum is None:
            minimum = self._finite_value(self.config.get("chassis_min_counts")) or 1.0
        if maximum_counts is None:
            maximum_counts = self._finite_value(self.config.get("chassis_max_counts")) or 2000.0
        if counts < max(1, math.ceil(minimum)):
            raise MotionConversionError("目标小于当前方向的最小可重复 CNT")
        if counts > math.floor(maximum_counts):
            raise MotionConversionError("请求 CNT 超过当前方向的安全上限")
        return ChassisMoveRequest(mode, counts, target, target_unit)

    def execution_from_report(self, report: ChassisDone) -> ExecutionEstimate:
        factor = self._factor(report.mode)
        if factor is None:
            raise MotionConversionError(f"{report.mode} 方向缺少执行先验换算")
        entry = self._calibration_entry(report.mode)
        uncertainty_key = "uncertainty_rad" if report.mode in {"R", "F"} else "uncertainty_m"
        uncertainty = self._finite_value(entry.get(uncertainty_key))
        if uncertainty is None or uncertainty < 0:
            raise MotionConversionError(f"{report.mode} 方向缺少执行不确定度")
        amount = report.enc / factor
        right_sign, forward_sign, yaw_sign = self._MODE_SIGNS[report.mode]
        estimate = ExecutionEstimate(
            report.mode,
            report.reason,
            right_sign * amount,
            forward_sign * amount,
            yaw_sign * amount,
            uncertainty,
            report.reason == "TARGET",
            report,
        )
        return estimate


class ChassisState:
    DISCONNECTED = "disconnected"
    CONNECTED_WAITING = "connected_waiting"
    IDLE = "idle"
    WAITING_ACK = "waiting_ack"
    WAITING_DONE = "waiting_done"
    STOPPING = "stopping"
    STOPPING_IDLE = "stopping_idle"
    SETTLING = "settling"
    WAITING_SCAN = "waiting_scan"
    UNKNOWN = "unknown"


@dataclass
class ChassisAction:
    action_id: int
    connection_generation: int
    mode: str
    counts: int
    source: str
    created_at: float
    request: bytes
    ack_at: float | None = None
    done_at: float | None = None
    stop_requested: bool = False


EventEmitter = Callable[[str, object, float], None]


class ChassisController:
    def __init__(
        self,
        endpoint,
        emit: EventEmitter,
        config: Mapping[str, object] | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.endpoint = endpoint
        self.emit = emit
        self.config = config or {}
        self.clock = clock or time.perf_counter
        self.parser = ChassisStreamParser(int(self.config.get("chassis_max_line_bytes", 1024)))
        self.state = ChassisState.DISCONNECTED
        self.connection_generation = 0
        self.confirmed = False
        self.automatic_locked = True
        self.pending: ChassisAction | None = None
        self.last_done: tuple[str, int, str] | None = None
        self._action_counter = 0
        self._tx_epoch = 0
        self._ack_deadline: float | None = None
        self._done_deadline: float | None = None
        self._stop_deadline: float | None = None
        self._status_due: float | None = None
        self._status_sent = False
        self._fault_emitted = False

    @property
    def in_flight(self) -> bool:
        return self.pending is not None and self.state in {
            ChassisState.WAITING_ACK,
            ChassisState.WAITING_DONE,
            ChassisState.STOPPING,
            ChassisState.SETTLING,
            ChassisState.WAITING_SCAN,
        }

    @property
    def can_scan(self) -> bool:
        return self.state in {ChassisState.IDLE, ChassisState.CONNECTED_WAITING}

    def _emit_event(self, kind: str, value: object, timestamp: float | None = None) -> None:
        self.emit(kind, value, self.clock() if timestamp is None else timestamp)

    def begin_connection(self) -> int:
        self.connection_generation += 1
        self.parser.reset()
        self.state = ChassisState.CONNECTED_WAITING
        self.confirmed = False
        self.automatic_locked = True
        self.pending = None
        self.last_done = None
        self._tx_epoch += 1
        self._ack_deadline = self._done_deadline = self._stop_deadline = None
        self._status_due = None
        self._status_sent = False
        self._fault_emitted = False
        generation = self.connection_generation
        self._emit_event("chassis_state", (generation, self.state, "等待底盘协议反馈"))
        return generation

    def mark_connection_open(self) -> None:
        if self.state == ChassisState.DISCONNECTED:
            return
        self._emit_event("chassis_state", (self.connection_generation, self.state, "底盘串口已打开，尚未确认机械状态"))

    def disconnect(self) -> None:
        self.connection_generation += 1
        self.parser.reset()
        self.state = ChassisState.DISCONNECTED
        self.confirmed = False
        self.automatic_locked = True
        self.pending = None
        self._tx_epoch += 1
        self._ack_deadline = self._done_deadline = self._stop_deadline = None
        self._status_due = None
        self._emit_event("chassis_state", (self.connection_generation, self.state, "底盘已断开"))

    def feed_data(self, data: bytes, host_time: float, generation: int) -> None:
        if generation != self.connection_generation:
            return
        for frame in self.parser.feed(data):
            self._emit_event("chassis_frame", (generation, frame), host_time)

    def handle_frame(self, generation: int, frame: ChassisFrame, host_time: float) -> None:
        if generation != self.connection_generation:
            return
        if frame.kind in {"empty", "diagnostic"}:
            if frame.kind == "diagnostic":
                self._handle_diagnostic(str(frame.value), host_time)
            return
        if frame.kind == "banner":
            self.confirmed = True
            if self.state == ChassisState.CONNECTED_WAITING:
                self.state = ChassisState.IDLE
                self._emit_event("chassis_state", (generation, self.state, "底盘协议已确认"), host_time)
            return
        if frame.kind == "invalid":
            self._emit_event("chassis_protocol_error", (generation, frame.raw, frame.error), host_time)
            return
        if frame.kind == "idle":
            self.confirmed = True
            if self.state in {ChassisState.CONNECTED_WAITING, ChassisState.STOPPING_IDLE}:
                self.state = ChassisState.IDLE
                self._stop_deadline = self._status_due = None
                self._emit_event("chassis_stop_confirmed", (generation, frame.value), host_time)
                return
            if self.state == ChassisState.STOPPING and self.pending is None:
                self.state = ChassisState.IDLE
                self._stop_deadline = self._status_due = None
                self._emit_event("chassis_stop_confirmed", (generation, frame.value), host_time)
                return
            self._mark_unknown("运行期间收到空闲状态，底盘动作归属不明", host_time)
            return
        if frame.kind == "ack":
            self._handle_ack(generation, frame.value, host_time)
            return
        if frame.kind == "done":
            self._handle_done(generation, frame.value, host_time)
            return
        if frame.kind == "error":
            self._handle_error(generation, frame.value, host_time)

    def _handle_diagnostic(self, line: str, host_time: float) -> None:
        if line == "MECANUM UNIVERSAL V6.3 COMM READY":
            self.confirmed = True
            if self.state == ChassisState.CONNECTED_WAITING:
                self.state = ChassisState.IDLE
                self._emit_event("chassis_state", (self.connection_generation, self.state, "底盘协议已确认"), host_time)
        self._emit_event("chassis_status", (self.connection_generation, line), host_time)

    def _handle_ack(self, generation: int, ack: ChassisAck, host_time: float) -> None:
        self.confirmed = True
        action = self.pending
        if action is None:
            self._mark_unknown("收到无待处理动作的 ACK", host_time)
            return
        if (ack.mode, ack.counts) != (action.mode, action.counts):
            self._mark_unknown("ACK 与当前动作的模式或 CNT 不匹配", host_time)
            return
        if self.state == ChassisState.WAITING_ACK:
            action.ack_at = host_time
            self.state = ChassisState.WAITING_DONE
            self._ack_deadline = None
            self._done_deadline = host_time + self._duration_timeout()
            self._emit_event("chassis_ack", (generation, action, ack), host_time)
            return
        if self.state == ChassisState.STOPPING:
            action.ack_at = action.ack_at or host_time
            self._emit_event("chassis_ack", (generation, action, ack), host_time)
            return
        self._emit_event("chassis_status", (generation, f"重复 ACK：{ack.raw}"), host_time)

    def _handle_done(self, generation: int, report: ChassisDone, host_time: float) -> None:
        action = self.pending
        identity = (report.mode, report.requested_counts, report.raw)
        if self.last_done == identity and action is None:
            self._emit_event("chassis_status", (generation, "忽略重复 DONE"), host_time)
            return
        if action is None:
            self._mark_unknown("收到无待处理动作的 DONE，拒绝恢复控制", host_time)
            return
        if (report.mode, report.requested_counts) != (action.mode, action.counts):
            self._mark_unknown("DONE 与当前动作的模式或 REQ 不匹配", host_time)
            return
        coherent, detail = validate_done_kinematics(report)
        if not coherent:
            self._mark_unknown(f"DONE 执行报告异常：{detail}", host_time)
            return
        if self.state == ChassisState.WAITING_ACK:
            self._mark_unknown("DONE 先于可确认的 ACK 到达", host_time)
            self._emit_event("chassis_unmatched_done", (generation, action, report), host_time)
            return
        if self.state not in {ChassisState.WAITING_DONE, ChassisState.STOPPING}:
            self._mark_unknown("DONE 到达时动作状态不可完成", host_time)
            return
        action.done_at = host_time
        self.last_done = identity
        self._ack_deadline = self._done_deadline = self._stop_deadline = None
        self._status_due = None
        self.state = ChassisState.SETTLING
        self._emit_event("chassis_done", (generation, action, report), host_time)

    def _handle_error(self, generation: int, error, host_time: float) -> None:
        code = getattr(error, "code", "UNKNOWN")
        if self.state in {ChassisState.STOPPING, ChassisState.STOPPING_IDLE} and code == "BAD_CMD":
            self._emit_event("chassis_status", (generation, f"停止清理行返回 {code}"), host_time)
            return
        self._mark_unknown(f"底盘错误：{code}", host_time)

    def request_move(self, mode: str, counts: int, *, source: str = "manual", now: float | None = None) -> bool:
        if source not in {"manual", "auto"}:
            raise ValueError("底盘动作来源必须是 manual 或 auto")
        now = self.clock() if now is None else float(now)
        if not getattr(self.endpoint, "is_open", False):
            self._emit_event("chassis_rejected", (self.connection_generation, "底盘串口未打开"), now)
            return False
        if source == "auto" and not self.automatic_ready:
            self._emit_event("chassis_rejected", (self.connection_generation, "底盘尚未满足自动动作准入"), now)
            return False
        if source == "manual":
            allowed = self.state in {ChassisState.IDLE, ChassisState.CONNECTED_WAITING}
        else:
            allowed = self.state == ChassisState.IDLE
        if not allowed or self.pending is not None:
            self._emit_event("chassis_rejected", (self.connection_generation, f"底盘状态为 {self.state}，不能发送新动作"), now)
            return False
        try:
            payload = encode_move(mode, counts)
        except ChassisProtocolError as exc:
            self._emit_event("chassis_rejected", (self.connection_generation, str(exc)), now)
            return False
        self._action_counter += 1
        action = ChassisAction(
            self._action_counter,
            self.connection_generation,
            payload.decode("ascii").split(",")[1],
            int(counts),
            source,
            now,
            payload,
        )
        self.pending = action
        self.state = ChassisState.WAITING_ACK
        self._tx_epoch += 1
        self._ack_deadline = now + self._ack_timeout()
        self._done_deadline = None
        self._fault_emitted = False
        epoch = self._tx_epoch
        try:
            sent = self._write(
                payload,
                on_sent=lambda stamp: self._emit_tx("chassis_tx_started", action, epoch, stamp),
                on_written=lambda stamp: self._emit_tx("chassis_tx_completed", action, epoch, stamp),
            )
        except Exception as exc:
            sent = False
            self._mark_unknown(f"底盘动作写入失败：{exc}", now)
        if not sent:
            self._mark_unknown("底盘动作未能排队发送", now)
            return False
        self._emit_event("chassis_motion_started", (self.connection_generation, action), now)
        return True

    @property
    def automatic_ready(self) -> bool:
        return bool(
            getattr(self.endpoint, "is_open", False)
            and self.confirmed
            and not self.automatic_locked
            and self.state == ChassisState.IDLE
            and self.pending is None
        )

    def allow_automatic(self) -> None:
        if self.state == ChassisState.IDLE and self.confirmed:
            self.automatic_locked = False

    def request_stop(self, *, reason: str = "用户停止", now: float | None = None) -> bool:
        now = self.clock() if now is None else float(now)
        self.automatic_locked = True
        if self.state in {ChassisState.STOPPING, ChassisState.STOPPING_IDLE}:
            return True
        if not getattr(self.endpoint, "is_open", False):
            self._mark_unknown("底盘连接不可用，停止未确认", now)
            return False
        action = self.pending
        if action is not None:
            action.stop_requested = True
            self.state = ChassisState.STOPPING
        else:
            self.state = ChassisState.STOPPING_IDLE
        self._tx_epoch += 1
        epoch = self._tx_epoch
        cancel = getattr(self.endpoint, "cancel_pending", None)
        if callable(cancel):
            cancel()
        self._stop_deadline = now + self._stop_timeout()
        self._status_due = now + self._stop_status_delay()
        self._status_sent = False
        try:
            sent = self._write(
                STOP_SEQUENCE,
                priority=True,
                on_written=lambda stamp: self._emit_tx("chassis_stop_tx_completed", action, epoch, stamp),
            )
        except Exception as exc:
            sent = False
            self._mark_unknown(f"停止字节写入失败：{exc}", now)
        self._emit_event("chassis_stop_requested", (self.connection_generation, reason, action), now)
        if not sent:
            self._mark_unknown("停止字节未能排队发送", now)
        return bool(sent)

    def request_status(self, now: float | None = None) -> bool:
        now = self.clock() if now is None else float(now)
        if not getattr(self.endpoint, "is_open", False):
            return False
        if self.state not in {
            ChassisState.IDLE,
            ChassisState.CONNECTED_WAITING,
            ChassisState.STOPPING,
            ChassisState.STOPPING_IDLE,
        }:
            return False
        if self.state in {ChassisState.STOPPING, ChassisState.STOPPING_IDLE} and self._status_sent:
            return False
        sent = self._write(b"P\r\n", priority=False)
        if sent:
            self._status_sent = True
            self._emit_event("chassis_status_request", (self.connection_generation, "P"), now)
        return bool(sent)

    def complete_settle(self, *, resume_auto: bool, now: float | None = None) -> bool:
        now = self.clock() if now is None else float(now)
        if self.state != ChassisState.SETTLING or self.pending is None:
            return False
        action = self.pending
        if resume_auto and action.source == "auto" and not action.stop_requested:
            self.state = ChassisState.WAITING_SCAN
            self._emit_event("chassis_waiting_scan", (self.connection_generation, action), now)
            return True
        self.pending = None
        self.state = ChassisState.IDLE if self.confirmed else ChassisState.CONNECTED_WAITING
        self._emit_event("chassis_ready", (self.connection_generation, action, "动作已结束，地图参考点需重新确认"), now)
        return True

    def mark_scan_ready(self, now: float | None = None) -> bool:
        now = self.clock() if now is None else float(now)
        if self.state != ChassisState.WAITING_SCAN or self.pending is None:
            return False
        action = self.pending
        self.pending = None
        self.state = ChassisState.IDLE
        self._emit_event("chassis_ready", (self.connection_generation, action, "已收到停稳后的新完整扫描"), now)
        return True

    def mark_execution_failed(self, detail: str, now: float | None = None) -> None:
        self.automatic_locked = True
        self._mark_unknown(detail, self.clock() if now is None else now)

    def poll(self, now: float | None = None) -> None:
        now = self.clock() if now is None else float(now)
        if self.state == ChassisState.WAITING_ACK and self._ack_deadline is not None and now >= self._ack_deadline:
            self._emit_event("chassis_timeout", (self.connection_generation, "ACK 超时，禁止重发 MOVE"), now)
            self.request_stop(reason="ACK 超时", now=now)
        elif self.state == ChassisState.WAITING_DONE and self._done_deadline is not None and now >= self._done_deadline:
            self._emit_event("chassis_timeout", (self.connection_generation, "DONE 超时，停止结果不明"), now)
            self.request_stop(reason="DONE 超时", now=now)
        elif self.state in {ChassisState.STOPPING, ChassisState.STOPPING_IDLE}:
            if self._status_due is not None and not self._status_sent and now >= self._status_due:
                self.request_status(now)
            if self._stop_deadline is not None and now >= self._stop_deadline:
                self._mark_unknown("停止反馈超时，物理停止未确认", now)

    def handle_transport_error(self, generation: int, message: str, host_time: float | None = None) -> None:
        if generation != self.connection_generation:
            return
        self._mark_unknown(f"底盘串口异常：{message}", self.clock() if host_time is None else host_time)

    def _write(self, payload: bytes, *, priority: bool = False, on_sent=None, on_written=None) -> bool:
        writer = getattr(self.endpoint, "write", None)
        if not callable(writer):
            writer = getattr(self.endpoint, "write_line", None)
            if not callable(writer):
                return False
            text = payload.decode("ascii", "replace").rstrip("\r\n")
            try:
                return bool(writer(text, on_sent=on_sent, on_written=on_written, priority=priority))
            except TypeError:
                return bool(writer(text, on_sent=on_sent))
        try:
            return bool(writer(payload, on_sent=on_sent, on_written=on_written, priority=priority))
        except TypeError:
            try:
                return bool(writer(payload, on_sent=on_sent, priority=priority))
            except TypeError:
                return bool(writer(payload))

    def _emit_tx(self, kind: str, action: ChassisAction | None, epoch: int, stamp: float) -> None:
        if epoch != self._tx_epoch:
            return
        self._emit_event(kind, (self.connection_generation, action), stamp)

    def _mark_unknown(self, detail: str, now: float) -> None:
        self.automatic_locked = True
        self.state = ChassisState.UNKNOWN
        self._ack_deadline = self._done_deadline = self._stop_deadline = None
        if not self._fault_emitted:
            self._fault_emitted = True
            self._emit_event("chassis_fault", (self.connection_generation, detail), now)

    def _ack_timeout(self) -> float:
        return max(0.1, self._number("chassis_ack_timeout_s", 2.0))

    def _duration_timeout(self) -> float:
        return max(0.5, self._number("chassis_action_timeout_s", 12.0))

    def _stop_timeout(self) -> float:
        return max(0.5, self._number("chassis_stop_timeout_s", 3.0))

    def _stop_status_delay(self) -> float:
        return max(0.0, self._number("chassis_stop_status_delay_s", 0.35))

    def _number(self, key: str, fallback: float) -> float:
        try:
            value = float(self.config.get(key, fallback))
        except (TypeError, ValueError, OverflowError):
            return fallback
        return value if math.isfinite(value) else fallback
