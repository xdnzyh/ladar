from __future__ import annotations

from dataclasses import dataclass, field
import math
import secrets
import statistics
import threading
import time
from typing import Callable, Mapping

from chassis_protocol import (
    ChassisAck,
    ChassisCapabilityMarker,
    ChassisDone,
    ChassisFrame,
    ChassisProtocolError,
    ChassisStreamParser,
    EXPECTED_STARTUP_MARKERS,
    STOP_SEQUENCE,
    TRANSLATION_MODES,
    encode_move,
    encode_ping,
    firmware_mm_to_counts,
    validate_done_kinematics,
    validate_mm_target_counts,
)
from navigation_core import VelocityCommand


class MotionConversionError(ValueError):
    pass


@dataclass(frozen=True)
class ChassisMoveRequest:
    mode: str
    request_value: int
    unit: str
    target_counts: int | None
    target: float
    target_unit: str

    @property
    def counts(self) -> int:
        return self.request_value

    @property
    def request_identity(self) -> tuple[str, int, str]:
        return self.mode, self.request_value, self.unit


@dataclass(frozen=True)
class ExecutionEstimate:
    mode: str
    reason: str
    local_x_m: float
    local_y_m: float
    yaw_rad: float
    uncertainty_m: float
    uncertainty_rad: float
    trusted: bool
    report: ChassisDone


class ChassisMotionAdapter:
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

    def update_config(self, config: Mapping[str, object]) -> None:
        self.config = config

    @staticmethod
    def _finite(value: object) -> float | None:
        try:
            parsed = float(value)
        except (TypeError, ValueError, OverflowError):
            return None
        return parsed if math.isfinite(parsed) else None

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
        if dominant <= 0 or minor / dominant < 1.0 - ratio_tolerance:
            raise MotionConversionError("斜移动作必须是等幅 45° 方向")
        if command.forward_mps > 0:
            return "E" if command.right_mps > 0 else "Q"
        return "C" if command.right_mps > 0 else "Z"

    def _translation_entry(self, mode: str) -> Mapping[str, object]:
        table = self.config.get("chassis_translation_capabilities", {})
        if not isinstance(table, Mapping):
            return {}
        entry = table.get(mode, {})
        return entry if isinstance(entry, Mapping) else {}

    def _rotation_entry(self, mode: str) -> Mapping[str, object]:
        table = self.config.get("chassis_rotation_capabilities", {})
        if not isinstance(table, Mapping):
            return {}
        entry = table.get(mode, {})
        return entry if isinstance(entry, Mapping) else {}

    def readiness(self, mode: str | None = None) -> tuple[bool, tuple[str, ...]]:
        reasons: list[str] = []
        capability_mode = str(self.config.get("chassis_capability_mode", "unknown"))
        if not bool(self.config.get("chassis_firmware_confirmed", False)):
            reasons.append("未确认底盘当前烧录版本")
        if capability_mode == "unknown":
            reasons.append("未确认底盘协议能力")
        if not bool(self.config.get("chassis_speed_validated", False)):
            reasons.append("底盘实际速度上界尚未测得")
        if not bool(self.config.get("chassis_braking_validated", False)):
            reasons.append("底盘完整停止距离尚未测得")
        speed = self._finite(self.config.get("safety_speed_upper_bound_mps"))
        if speed is None or speed <= 0:
            reasons.append("缺少经测量的底盘速度上界")
        stop_distance = self._finite(self.config.get("safety_stop_distance_m"))
        if stop_distance is None or stop_distance < 0:
            reasons.append("缺少经测量的保守停止距离")

        if mode is None:
            modes = tuple(
                item for item in "WSADQEZC"
                if bool(self._translation_entry(item).get("enabled", False))
            )
            if not modes:
                reasons.append("没有启用任何平移方向")
        else:
            modes = (str(mode).upper(),)
        for item in modes:
            if item in TRANSLATION_MODES:
                entry = self._translation_entry(item)
                if not bool(entry.get("enabled", False)):
                    reasons.append(f"{item} 方向未启用")
                    continue
                if not bool(entry.get("motion_range_validated", False)):
                    reasons.append(f"{item} 方向执行范围尚未重复验证")
                    continue
                minimum = self._finite(entry.get("validated_min_mm"))
                maximum = self._finite(entry.get("validated_max_mm"))
                uncertainty = self._finite(entry.get("uncertainty_m"))
                if minimum is None or maximum is None or minimum <= 0 or maximum < minimum:
                    reasons.append(f"{item} 方向验证范围无效")
                if uncertainty is None or uncertainty < 0:
                    reasons.append(f"{item} 方向缺少执行不确定度")
                coefficient = self._finite(entry.get("counts_per_mm"))
                if coefficient is None or coefficient <= 0:
                    reasons.append(f"{item} 方向缺少有效 CNT/mm 初值")
            elif item in {"R", "F"}:
                entry = self._rotation_entry(item)
                if not bool(entry.get("enabled", False)) or entry.get("status") != "validated":
                    reasons.append(f"{item} 旋转尚未标定并启用")
            else:
                reasons.append(f"{item} 不是受支持的底盘方向")
        return not reasons, tuple(dict.fromkeys(reasons))

    def _validate_translation_range(
        self,
        mode: str,
        distance_m: float,
        *,
        automatic: bool,
    ) -> Mapping[str, object]:
        entry = self._translation_entry(mode)
        if not bool(entry.get("enabled", False)):
            raise MotionConversionError(f"{mode} 方向未启用")
        maximum_global = self._finite(self.config.get("chassis_max_translation_m"))
        if maximum_global is not None and maximum_global > 0 and distance_m > maximum_global + 1e-12:
            raise MotionConversionError(f"动作长度超过当前底盘上限 {maximum_global:g} m")
        if automatic or bool(entry.get("motion_range_validated", False)):
            if not bool(entry.get("motion_range_validated", False)):
                raise MotionConversionError(f"{mode} 方向执行范围尚未重复验证")
            minimum = self._finite(entry.get("validated_min_mm"))
            maximum = self._finite(entry.get("validated_max_mm"))
            if minimum is None or maximum is None:
                raise MotionConversionError(f"{mode} 方向执行范围不完整")
            minimum_m = minimum / 1000.0
            maximum_m = maximum / 1000.0
            if distance_m + 1e-12 < minimum_m:
                raise MotionConversionError(
                    f"{mode} 方向目标 {distance_m:.3f} m 小于已验证步长 {minimum_m:.3f} m"
                )
            if distance_m > maximum_m + 1e-12:
                raise MotionConversionError(
                    f"{mode} 方向目标 {distance_m:.3f} m 超过已验证上限 {maximum_m:.3f} m"
                )
        return entry

    def request_for_command(
        self,
        command: VelocityCommand,
        *,
        automatic: bool = True,
    ) -> ChassisMoveRequest:
        mode = self.mode_for_command(command)
        if automatic:
            ready, reasons = self.readiness(mode)
            if not ready:
                raise MotionConversionError("；".join(reasons))
        if mode in TRANSLATION_MODES:
            distance = math.hypot(
                command.forward_mps * command.duration_s,
                command.right_mps * command.duration_s,
            )
            entry = self._validate_translation_range(mode, distance, automatic=automatic)
            preferred = str(self.config.get("chassis_preferred_translation_unit", "MM"))
            capability = str(self.config.get("chassis_capability_mode", "unknown"))
            if preferred == "MM":
                if capability != "mm_ping_v1":
                    raise MotionConversionError("当前底盘能力未确认支持 MM")
                millimeters = int(math.floor(distance * 1000.0 + 1e-9))
                if millimeters <= 0:
                    raise MotionConversionError("量化后的 MM 请求为零")
                quantized_distance = millimeters / 1000.0
                self._validate_translation_range(mode, quantized_distance, automatic=automatic)
                fixed = entry.get("fixed_counts_per_mm")
                try:
                    target_counts = firmware_mm_to_counts(
                        millimeters,
                        fixed_counts_per_mm=int(fixed),
                    )
                except (ChassisProtocolError, TypeError, ValueError, OverflowError) as exc:
                    raise MotionConversionError(str(exc)) from exc
                return ChassisMoveRequest(
                    mode, millimeters, "MM", target_counts, quantized_distance, "m"
                )
            coefficient = self._finite(entry.get("counts_per_mm"))
            if coefficient is None or coefficient <= 0:
                raise MotionConversionError(f"{mode} 方向缺少有效 CNT/mm 初值")
            counts = int(math.floor(distance * 1000.0 * coefficient + 1e-9))
            if counts <= 0:
                raise MotionConversionError("量化后的 CNT 请求为零")
            quantized_distance = counts / coefficient / 1000.0
            self._validate_translation_range(mode, quantized_distance, automatic=automatic)
            maximum_counts = int(self.config.get("chassis_max_counts", 2_147_483_647))
            if counts > maximum_counts:
                raise MotionConversionError("CNT 请求超过当前发送上限")
            return ChassisMoveRequest(mode, counts, "CNT", counts, quantized_distance, "m")

        entry = self._rotation_entry(mode)
        if not bool(entry.get("enabled", False)) or entry.get("status") != "validated":
            raise MotionConversionError(f"{mode} 旋转尚未标定并启用")
        factor = self._finite(entry.get("counts_per_rad"))
        uncertainty = self._finite(entry.get("uncertainty_rad"))
        if factor is None or factor <= 0 or uncertainty is None or uncertainty < 0:
            raise MotionConversionError(f"{mode} 旋转换算不完整")
        angle = abs(command.yaw_rps * command.duration_s)
        counts = int(math.floor(angle * factor + 1e-9))
        minimum = int(entry.get("validated_min_counts"))
        maximum = int(entry.get("validated_max_counts"))
        if counts < minimum or counts > maximum:
            raise MotionConversionError(f"{mode} 旋转 CNT 不在已验证范围")
        return ChassisMoveRequest(mode, counts, "CNT", counts, counts / factor, "rad")

    def request_for_manual(self, mode: str, value: int, unit: str) -> ChassisMoveRequest:
        mode = str(mode).upper()
        unit = str(unit).upper()
        if mode not in self._MODE_SIGNS:
            raise MotionConversionError(f"不支持底盘方向：{mode}")
        if isinstance(value, bool):
            raise MotionConversionError("人工动作请求量必须是正整数")
        try:
            request_value = int(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise MotionConversionError("人工动作请求量必须是正整数") from exc
        if request_value <= 0 or str(request_value) != str(value).strip():
            raise MotionConversionError("人工动作请求量必须是正整数")
        if not bool(self.config.get("chassis_firmware_confirmed", False)):
            raise MotionConversionError("请先确认并保存底盘当前烧录版本")
        capability = str(self.config.get("chassis_capability_mode", "unknown"))
        if capability == "unknown":
            raise MotionConversionError("请先确认并保存底盘协议能力")
        if unit == "MM":
            if mode not in TRANSLATION_MODES:
                raise MotionConversionError("R/F 旋转只支持 CNT")
            if capability != "mm_ping_v1":
                raise MotionConversionError("当前底盘能力不支持 MM")
            distance = request_value / 1000.0
            entry = self._validate_translation_range(mode, distance, automatic=False)
            try:
                target_counts = firmware_mm_to_counts(
                    request_value,
                    fixed_counts_per_mm=int(entry.get("fixed_counts_per_mm")),
                )
            except (ChassisProtocolError, TypeError, ValueError, OverflowError) as exc:
                raise MotionConversionError(str(exc)) from exc
            return ChassisMoveRequest(mode, request_value, "MM", target_counts, distance, "m")
        if unit != "CNT":
            raise MotionConversionError("人工动作单位必须是 MM 或 CNT")
        minimum = int(self.config.get("chassis_min_counts", 1))
        maximum = int(self.config.get("chassis_max_counts", 2_147_483_647))
        if not minimum <= request_value <= maximum:
            raise MotionConversionError(f"CNT 请求必须在 {minimum}～{maximum} 之间")
        if mode in TRANSLATION_MODES:
            entry = self._translation_entry(mode)
            if not bool(entry.get("enabled", False)):
                raise MotionConversionError(f"{mode} 方向未启用")
            coefficient = self._finite(entry.get("counts_per_mm"))
            if coefficient is None or coefficient <= 0:
                raise MotionConversionError(f"{mode} 方向缺少有效 CNT/mm 初值")
            distance = request_value / coefficient / 1000.0
            self._validate_translation_range(mode, distance, automatic=False)
            return ChassisMoveRequest(mode, request_value, "CNT", request_value, distance, "m")
        entry = self._rotation_entry(mode)
        if not bool(entry.get("enabled", False)) or entry.get("status") != "validated":
            raise MotionConversionError(f"{mode} 旋转尚未标定并启用")
        factor = self._finite(entry.get("counts_per_rad"))
        if factor is None or factor <= 0:
            raise MotionConversionError(f"{mode} 旋转换算不完整")
        return ChassisMoveRequest(mode, request_value, "CNT", request_value,
                                  request_value / factor, "rad")

    def execution_from_report(self, report: ChassisDone) -> ExecutionEstimate:
        mode = report.mode
        if mode in TRANSLATION_MODES:
            entry = self._translation_entry(mode)
            coefficient = self._finite(entry.get("counts_per_mm"))
            if coefficient is None or coefficient <= 0:
                raise MotionConversionError(f"{mode} 方向缺少执行先验换算")
            uncertainty_m = self._finite(entry.get("uncertainty_m"))
            if uncertainty_m is None or uncertainty_m < 0:
                raise MotionConversionError(f"{mode} 方向缺少执行不确定度")
            amount = report.enc / coefficient / 1000.0
            uncertainty_rad = 0.0
        else:
            entry = self._rotation_entry(mode)
            factor = self._finite(entry.get("counts_per_rad"))
            uncertainty_rad = self._finite(entry.get("uncertainty_rad"))
            if factor is None or factor <= 0:
                raise MotionConversionError(f"{mode} 方向缺少旋转执行先验换算")
            if uncertainty_rad is None or uncertainty_rad < 0:
                raise MotionConversionError(f"{mode} 方向缺少旋转执行不确定度")
            amount = report.enc / factor
            uncertainty_m = 0.0
        right, forward, yaw = self._MODE_SIGNS[mode]
        return ExecutionEstimate(
            mode=mode,
            reason=report.reason,
            local_x_m=right * amount,
            local_y_m=forward * amount,
            yaw_rad=yaw * amount,
            uncertainty_m=float(uncertainty_m),
            uncertainty_rad=float(uncertainty_rad),
            trusted=report.reason == "TARGET",
            report=report,
        )


class ChassisState:
    DISCONNECTED = "disconnected"
    CONNECTED_WAITING = "connected_waiting"
    IDLE = "idle"
    WAITING_SILENCE = "waiting_silence"
    WAITING_PONG = "waiting_pong"
    WAITING_POST_PONG_SILENCE = "waiting_post_pong_silence"
    PENDING_SEND = "pending_send"
    WAITING_ACK = "waiting_ack"
    WAITING_DONE = "waiting_done"
    STOPPING = "stopping"
    STOPPING_IDLE = "stopping_idle"
    SETTLING = "settling"
    WAITING_SCAN = "waiting_scan"
    CHECKING = "checking"
    UNKNOWN = "unknown"


@dataclass
class ChassisAction:
    action_id: int
    connection_generation: int
    request: ChassisMoveRequest
    source: str
    created_at: float
    payload: bytes
    nonce: str | None = None
    ping_sent_at: float | None = None
    send_started_at: float | None = None
    write_completed_at: float | None = None
    ack_at: float | None = None
    done_at: float | None = None
    ack_missing: bool = False
    stop_requested: bool = False
    move_may_have_started: bool = False
    write_ticket: object | None = None
    handled_report_raw: str | None = None

    @property
    def mode(self) -> str:
        return self.request.mode

    @property
    def request_value(self) -> int:
        return self.request.request_value

    @property
    def unit(self) -> str:
        return self.request.unit

    @property
    def target_counts(self) -> int | None:
        return self.request.target_counts

    @property
    def counts(self) -> int:
        return self.request.request_value


@dataclass
class CommunicationCheck:
    requested: int
    completed: int = 0
    first_successes: int = 0
    retry_successes: int = 0
    failures: int = 0
    current_retry: int = 0
    phase: str = "waiting_silence"
    nonce: str | None = None
    wait_started_at: float = 0.0
    sent_at: float | None = None
    deadline: float | None = None
    latencies_ms: list[float] = field(default_factory=list)
    restarts: int = 0


EventEmitter = Callable[[str, object, float], None]


class ChassisController:
    def __init__(
        self,
        endpoint,
        emit: EventEmitter,
        config: Mapping[str, object] | None = None,
        clock: Callable[[], float] | None = None,
        nonce_factory: Callable[[], str] | None = None,
    ) -> None:
        self.endpoint = endpoint
        self.emit = emit
        self.config = config or {}
        self.clock = clock or time.perf_counter
        self.nonce_factory = nonce_factory or (lambda: secrets.token_hex(4).upper())
        self.parser = ChassisStreamParser(int(self.config.get("chassis_max_line_bytes", 1024)))
        self.state = ChassisState.DISCONNECTED
        self.connection_generation = 0
        self.confirmed = False
        self.automatic_locked = True
        self.pending: ChassisAction | None = None
        self.communication_check: CommunicationCheck | None = None
        self.observed_markers: set[str] = set()
        self.observed_capability_mode = "unknown"
        self.last_idle = None
        self.last_done = None
        self._action_counter = 0
        self._tx_epoch = 0
        self._last_rx_at = -math.inf
        self._phase_deadline: float | None = None
        self._ack_deadline: float | None = None
        self._total_deadline: float | None = None
        self._stop_deadline: float | None = None
        self._status_due: float | None = None
        self._status_sent = False
        self._fault_emitted = False
        self._motion_unconfirmed = False
        self._lock = threading.RLock()

    def update_config(self, config: Mapping[str, object]) -> bool:
        with self._lock:
            if self.in_flight or self.communication_check is not None:
                return False
            self.config = config
            self.parser.max_line_bytes = int(config.get("chassis_max_line_bytes", 1024))
            return True

    @property
    def in_flight(self) -> bool:
        return bool(
            self.pending is not None
            or self._motion_unconfirmed
            or self.state in {
                ChassisState.WAITING_SILENCE,
                ChassisState.WAITING_PONG,
                ChassisState.WAITING_POST_PONG_SILENCE,
                ChassisState.PENDING_SEND,
                ChassisState.WAITING_ACK,
                ChassisState.WAITING_DONE,
                ChassisState.STOPPING,
                ChassisState.STOPPING_IDLE,
                ChassisState.SETTLING,
                ChassisState.WAITING_SCAN,
                ChassisState.UNKNOWN,
            }
        )

    @property
    def move_may_have_started(self) -> bool:
        return bool(
            self._motion_unconfirmed
            or (self.pending is not None and self.pending.move_may_have_started)
        )

    @property
    def capability_ready(self) -> bool:
        configured = str(self.config.get("chassis_capability_mode", "unknown"))
        return bool(
            self.config.get("chassis_firmware_confirmed", False)
            and configured in {"cnt_only", "mm_ping_v1"}
        ) or self.observed_capability_mode == "mm_ping_v1"

    @property
    def automatic_ready(self) -> bool:
        return bool(
            getattr(self.endpoint, "is_open", False)
            and self.confirmed
            and self.capability_ready
            and not self.automatic_locked
            and not self._motion_unconfirmed
            and self.state == ChassisState.IDLE
            and self.pending is None
            and self.communication_check is None
        )

    @property
    def can_scan(self) -> bool:
        return self.state in {ChassisState.IDLE, ChassisState.CONNECTED_WAITING}

    def _now(self, value: float | None) -> float:
        return self.clock() if value is None else float(value)

    def _emit(self, kind: str, value: object, stamp: float | None = None) -> None:
        self.emit(kind, value, self.clock() if stamp is None else float(stamp))

    def _state(self, state: str, detail: str, stamp: float) -> None:
        self.state = state
        self._emit("chassis_state", (self.connection_generation, state, detail), stamp)

    def begin_connection(self) -> int:
        with self._lock:
            self.connection_generation += 1
            self.parser.reset()
            self.confirmed = False
            self.automatic_locked = True
            self.pending = None
            self.communication_check = None
            self.observed_markers.clear()
            self.observed_capability_mode = "unknown"
            self.last_idle = None
            self.last_done = None
            self._tx_epoch += 1
            self._clear_deadlines()
            self._fault_emitted = False
            self._last_rx_at = self.clock()
            self._state(ChassisState.CONNECTED_WAITING, "等待底盘空闲状态与协议能力证据", self.clock())
            return self.connection_generation

    def mark_connection_open(self) -> None:
        with self._lock:
            if self.state != ChassisState.DISCONNECTED:
                self._emit(
                    "chassis_state",
                    (self.connection_generation, self.state, "底盘串口已打开，机械状态尚未确认"),
                )

    def disconnect(self) -> None:
        with self._lock:
            if self.pending is not None and self.pending.move_may_have_started:
                self._motion_unconfirmed = True
            self.connection_generation += 1
            self.parser.reset()
            self.confirmed = False
            self.automatic_locked = True
            self.pending = None
            self.communication_check = None
            self.observed_markers.clear()
            self.observed_capability_mode = "unknown"
            self.last_done = None
            self._tx_epoch += 1
            self._clear_deadlines()
            self.state = ChassisState.DISCONNECTED
            detail = "底盘已断开；停止状态未确认" if self._motion_unconfirmed else "底盘已断开"
            self._emit("chassis_state", (self.connection_generation, self.state, detail))

    def feed_data(self, data: bytes, host_time: float, generation: int) -> None:
        with self._lock:
            if generation != self.connection_generation:
                return
            self._last_rx_at = float(host_time)
            action_id = self.pending.action_id if self.pending is not None else None
            self._emit("chassis_raw_rx", (generation, action_id, bytes(data)), host_time)
            frames = self.parser.feed(data)
        for frame in frames:
            self._emit("chassis_frame", (generation, frame), host_time)

    def handle_frame(self, generation: int, frame: ChassisFrame, host_time: float) -> None:
        with self._lock:
            if generation != self.connection_generation:
                return
            if frame.kind == "empty":
                return
            if frame.kind == "invalid":
                self._emit("chassis_protocol_error", (generation, frame.raw, frame.error), host_time)
                if self.pending is not None and self.pending.move_may_have_started:
                    self._fault_and_stop(f"底盘报告损坏：{frame.error}", host_time)
                return
            if frame.kind == "banner":
                self._handle_startup(frame.raw, host_time)
                return
            if frame.kind == "capability":
                self._handle_capability(frame.value, host_time)
                return
            if frame.kind == "diagnostic":
                self._emit("chassis_status", (generation, str(frame.value)), host_time)
                return
            if frame.kind == "idle":
                self._handle_idle(frame.value, host_time)
                return
            if frame.kind == "pong":
                self._handle_pong(frame.value, host_time)
                return
            if frame.kind == "ack":
                self._handle_ack(frame.value, host_time)
                return
            if frame.kind == "done":
                self._handle_done(frame.value, host_time)
                return
            if frame.kind == "error":
                self._handle_error(frame.value, host_time)
                return
            if frame.kind == "log":
                self._emit("chassis_status", (generation, frame.raw), host_time)

    def _handle_startup(self, raw: str, stamp: float) -> None:
        active = self.pending is not None or self._motion_unconfirmed or self.state not in {
            ChassisState.CONNECTED_WAITING,
            ChassisState.IDLE,
            ChassisState.CHECKING,
        }
        if self.communication_check is not None:
            self.communication_check.restarts += 1
            active = True
        self.confirmed = False
        self.automatic_locked = True
        self.observed_markers = {raw}
        self.observed_capability_mode = "unknown"
        self._emit("chassis_status", (self.connection_generation, raw), stamp)
        if active:
            self._fault_and_stop("底盘运行期间重启，原动作与位姿依据失效", stamp)
        else:
            self._state(ChassisState.CONNECTED_WAITING, "检测到底盘启动，等待完整能力标记", stamp)

    def _handle_capability(self, marker: ChassisCapabilityMarker, stamp: float) -> None:
        if not marker.is_expected:
            self._emit("chassis_protocol_error", (
                self.connection_generation, marker.raw, "底盘能力标记与受支持固件不一致"
            ), stamp)
            return
        self.observed_markers.add(marker.raw)
        self._emit("chassis_status", (self.connection_generation, marker.raw), stamp)
        if EXPECTED_STARTUP_MARKERS.issubset(self.observed_markers):
            self.observed_capability_mode = "mm_ping_v1"
            if self.state == ChassisState.CONNECTED_WAITING and not self._motion_unconfirmed:
                self.confirmed = True
                self._state(ChassisState.IDLE, "已识别 MM/PING 固件能力，底盘空闲", stamp)
            self._emit("chassis_capability", (
                self.connection_generation, self.observed_capability_mode, tuple(sorted(self.observed_markers))
            ), stamp)

    def _handle_idle(self, status, stamp: float) -> None:
        self.last_idle = status
        if self.state in {ChassisState.STOPPING, ChassisState.STOPPING_IDLE, ChassisState.UNKNOWN}:
            action = self.pending
            self.pending = None
            self._motion_unconfirmed = False
            self.confirmed = True
            self._clear_deadlines()
            self._state(ChassisState.IDLE, "底盘返回 IDLE，停止状态已确认", stamp)
            self._emit("chassis_stop_confirmed", (self.connection_generation, status, action), stamp)
            return
        if self.state in {ChassisState.CONNECTED_WAITING, ChassisState.IDLE}:
            self.confirmed = True
            self._motion_unconfirmed = False
            self._state(ChassisState.IDLE, "底盘返回 IDLE", stamp)
            self._emit("chassis_idle", (self.connection_generation, status), stamp)
            return
        if self.state == ChassisState.CHECKING:
            self._emit("chassis_status", (self.connection_generation, status.raw), stamp)
            return
        self._fault_and_stop("运动事务期间收到无归属的 IDLE", stamp)

    def _handle_pong(self, pong, stamp: float) -> None:
        if self.communication_check is not None and self.state == ChassisState.CHECKING:
            check = self.communication_check
            if check.phase == "waiting_pong" and pong.nonce == check.nonce:
                latency = max(0.0, stamp - float(check.sent_at or stamp)) * 1000.0
                check.latencies_ms.append(latency)
                if check.current_retry == 0:
                    check.first_successes += 1
                else:
                    check.retry_successes += 1
                check.completed += 1
                self._emit("chassis_communication_progress", self._check_snapshot(check), stamp)
                if check.completed >= check.requested:
                    self._finish_communication_check(stamp)
                else:
                    self._prepare_next_check_sample(check, stamp)
            else:
                self._emit("chassis_status", (self.connection_generation, f"忽略旧 PONG {pong.nonce}"), stamp)
            return
        action = self.pending
        if (
            action is not None
            and self.state == ChassisState.WAITING_PONG
            and action.nonce == pong.nonce
        ):
            self._phase_deadline = stamp + self._silence_wait_timeout()
            self._state(ChassisState.WAITING_POST_PONG_SILENCE, "PING 已匹配，等待发送前静默", stamp)
            return
        self._emit("chassis_status", (self.connection_generation, f"忽略无归属 PONG {pong.nonce}"), stamp)

    @staticmethod
    def _matches(action: ChassisAction, mode: str, value: int, unit: str) -> bool:
        return (mode, value, unit) == action.request.request_identity

    def _handle_ack(self, ack: ChassisAck, stamp: float) -> None:
        action = self.pending
        if action is None:
            self._fault_and_stop("收到无待处理动作的 ACK", stamp)
            return
        if not self._matches(action, ack.mode, ack.request_value, ack.unit):
            self._fault_and_stop("ACK 与当前动作的模式、请求量或单位不匹配", stamp)
            return
        if self.state in {ChassisState.WAITING_ACK, ChassisState.WAITING_DONE}:
            if action.ack_at is None:
                action.ack_at = stamp
                action.ack_missing = False
                self._ack_deadline = None
                self._state(ChassisState.WAITING_DONE, "底盘已接受动作，等待 DONE", stamp)
                self._emit("chassis_ack", (self.connection_generation, action, ack), stamp)
            else:
                self._emit("chassis_status", (self.connection_generation, "忽略重复 ACK"), stamp)
            return
        if self.state in {ChassisState.STOPPING, ChassisState.SETTLING, ChassisState.WAITING_SCAN}:
            self._emit("chassis_status", (self.connection_generation, "忽略迟到或重复 ACK"), stamp)
            return
        self._fault_and_stop("ACK 到达时事务尚未发送或已不可完成", stamp)

    def _handle_done(self, report: ChassisDone, stamp: float) -> None:
        action = self.pending
        if action is None:
            if self.last_done == report.raw:
                self._emit("chassis_status", (self.connection_generation, "忽略重复 DONE"), stamp)
            else:
                self._emit("chassis_unmatched_done", (self.connection_generation, None, report), stamp)
                self._fault_and_stop("收到无待处理动作的 DONE，归属不明", stamp)
            return
        if not self._matches(action, report.mode, report.request_value, report.unit):
            self._emit("chassis_unmatched_done", (self.connection_generation, action, report), stamp)
            self._fault_and_stop("DONE 与当前动作的模式、请求量或单位不匹配", stamp)
            return
        if action.handled_report_raw == report.raw or (
            self.state in {ChassisState.SETTLING, ChassisState.WAITING_SCAN}
            and self.last_done == report.raw
        ):
            self._emit("chassis_status", (self.connection_generation, "忽略重复 DONE"), stamp)
            return
        coherent, detail = validate_done_kinematics(report)
        if not coherent:
            self._fault_and_stop(f"DONE 编码器分量不一致：{detail}", stamp)
            return
        if report.unit == "MM":
            entry = ChassisMotionAdapter(self.config)._translation_entry(report.mode)
            valid, detail = validate_mm_target_counts(
                report,
                fixed_counts_per_mm=int(entry.get("fixed_counts_per_mm", 0)),
            )
            if not valid or (
                action.target_counts is not None and report.target_counts != action.target_counts
            ):
                self._fault_and_stop(f"DONE TARGET_CNT 校验失败：{detail or '与请求不一致'}", stamp)
                return
        allowed = {ChassisState.WAITING_ACK, ChassisState.WAITING_DONE, ChassisState.STOPPING}
        if self.state not in allowed:
            self._fault_and_stop("DONE 到达时事务尚未发送或已不可完成", stamp)
            return
        action.done_at = stamp
        action.handled_report_raw = report.raw
        self.last_done = report.raw
        self._clear_deadlines()
        self._motion_unconfirmed = False
        self._state(ChassisState.SETTLING, "已收到完整 DONE，等待机械停稳", stamp)
        self._emit("chassis_done", (self.connection_generation, action, report), stamp)

    def _handle_error(self, error, stamp: float) -> None:
        code = getattr(error, "code", "UNKNOWN")
        if self.state in {ChassisState.STOPPING, ChassisState.STOPPING_IDLE} and code == "BAD_CMD":
            self._emit("chassis_status", (self.connection_generation, "停止清理行返回预期 BAD_CMD"), stamp)
            return
        if self.communication_check is not None and self.state == ChassisState.CHECKING:
            if code == "BUSY":
                self._fail_check_attempt(stamp, "PING 返回 BUSY")
            else:
                self._abort_communication_check(stamp, f"通信检查返回 {code}")
            return
        action = self.pending
        if action is not None and not action.move_may_have_started:
            self._abort_before_move(f"底盘在 MOVE 前返回错误：{code}", stamp)
        else:
            self._fault_and_stop(f"底盘错误：{code}", stamp)

    def request_move(
        self,
        request_or_mode: ChassisMoveRequest | str,
        request_value: int | None = None,
        unit: str = "CNT",
        *,
        source: str = "manual",
        operator_authorized: bool = False,
        now: float | None = None,
    ) -> bool:
        stamp = self._now(now)
        with self._lock:
            if source not in {"manual", "auto"}:
                raise ValueError("底盘动作来源必须是 manual 或 auto")
            if source == "manual" and not operator_authorized:
                self._emit("chassis_rejected", (self.connection_generation, "人工动作需要现场单步授权"), stamp)
                return False
            if not getattr(self.endpoint, "is_open", False):
                self._emit("chassis_rejected", (self.connection_generation, "底盘串口未打开"), stamp)
                return False
            if source == "auto" and not self.automatic_ready:
                self._emit("chassis_rejected", (self.connection_generation, "底盘尚未满足自动动作准入"), stamp)
                return False
            if self.state != ChassisState.IDLE or self.pending is not None or self._motion_unconfirmed:
                self._emit("chassis_rejected", (
                    self.connection_generation, f"底盘状态为 {self.state}，不能发送新动作"
                ), stamp)
                return False
            if isinstance(request_or_mode, ChassisMoveRequest):
                request = request_or_mode
            else:
                try:
                    payload = encode_move(
                        request_or_mode,
                        int(request_value),
                        unit,
                        max_command_bytes=int(self.config.get("chassis_tx_max_command_bytes", 47)),
                    )
                except (ChassisProtocolError, TypeError, ValueError, OverflowError) as exc:
                    self._emit("chassis_rejected", (self.connection_generation, str(exc)), stamp)
                    return False
                mode = payload.decode("ascii").split(",")[1]
                parsed_unit = payload.decode("ascii").split(",")[3].strip()
                parsed_value = int(payload.decode("ascii").split(",")[2])
                request = ChassisMoveRequest(mode, parsed_value, parsed_unit,
                                             parsed_value if parsed_unit == "CNT" else None,
                                             float(parsed_value), parsed_unit.lower())
            try:
                payload = encode_move(
                    request.mode,
                    request.request_value,
                    request.unit,
                    max_command_bytes=int(self.config.get("chassis_tx_max_command_bytes", 47)),
                )
            except ChassisProtocolError as exc:
                self._emit("chassis_rejected", (self.connection_generation, str(exc)), stamp)
                return False
            if request.unit == "MM" and self._effective_capability_mode() != "mm_ping_v1":
                self._emit("chassis_rejected", (self.connection_generation, "底盘 MM/PING 能力尚未确认"), stamp)
                return False
            if self._effective_capability_mode() == "unknown":
                self._emit("chassis_rejected", (self.connection_generation, "底盘协议能力尚未确认"), stamp)
                return False
            self._action_counter += 1
            action = ChassisAction(
                self._action_counter,
                self.connection_generation,
                request,
                source,
                stamp,
                payload,
            )
            self.pending = action
            self.automatic_locked = source != "auto"
            self._fault_emitted = False
            self._phase_deadline = stamp + self._silence_wait_timeout()
            self._state(ChassisState.WAITING_SILENCE, "动作已登记，等待底盘 RX 静默", stamp)
            return True

    def allow_automatic(self) -> bool:
        with self._lock:
            if (
                self.state == ChassisState.IDLE
                and self.confirmed
                and self.capability_ready
                and not self._motion_unconfirmed
            ):
                self.automatic_locked = False
                return True
            return False

    def request_stop(self, *, reason: str = "用户停止", now: float | None = None) -> bool:
        stamp = self._now(now)
        with self._lock:
            self.automatic_locked = True
            if self.state in {ChassisState.STOPPING, ChassisState.STOPPING_IDLE}:
                return True
            if not getattr(self.endpoint, "is_open", False):
                self._motion_unconfirmed = self.move_may_have_started or self._motion_unconfirmed
                self._mark_unknown("底盘连接不可用，停止未确认", stamp)
                return False
            action = self.pending
            if action is not None:
                action.stop_requested = True
                ticket = action.write_ticket
                cancel_write = getattr(self.endpoint, "cancel_write", None)
                if ticket is not None and callable(cancel_write):
                    result = cancel_write(ticket)
                    if result == "started":
                        action.move_may_have_started = True
                if action.move_may_have_started:
                    self._motion_unconfirmed = True
            self._tx_epoch += 1
            epoch = self._tx_epoch
            self._clear_action_deadlines()
            self._stop_deadline = stamp + self._stop_timeout()
            self._status_due = stamp + self._stop_status_delay()
            self._status_sent = False
            state = ChassisState.STOPPING if self.move_may_have_started else ChassisState.STOPPING_IDLE
            self._state(state, "停止请求已登记，等待底盘状态确认", stamp)
            sent = self._queue_write(
                STOP_SEQUENCE,
                action_id=action.action_id if action is not None else None,
                epoch=epoch,
                priority=True,
                discard_pending=True,
                on_failed=lambda failed_at, written, total, detail: self._on_stop_write_failed(
                    epoch, failed_at, written, total, detail
                ),
            )
            self._emit("chassis_stop_requested", (self.connection_generation, reason, action), stamp)
            if not sent:
                self._mark_unknown("停止字节未能排队发送", stamp)
            return bool(sent)

    def request_status(self, now: float | None = None) -> bool:
        stamp = self._now(now)
        with self._lock:
            if not getattr(self.endpoint, "is_open", False):
                return False
            if self.state not in {
                ChassisState.IDLE,
                ChassisState.CONNECTED_WAITING,
                ChassisState.STOPPING,
                ChassisState.STOPPING_IDLE,
                ChassisState.UNKNOWN,
            }:
                return False
            if self.state in {ChassisState.STOPPING, ChassisState.STOPPING_IDLE} and self._status_sent:
                return False
            self._tx_epoch += 1
            epoch = self._tx_epoch
            sent = self._queue_write(b"P\r\n", action_id=None, epoch=epoch, priority=False)
            if sent:
                self._status_sent = True
                self._emit("chassis_status_request", (self.connection_generation, "P"), stamp)
            return bool(sent)

    def request_communication_check(self, count: int = 20, now: float | None = None) -> bool:
        stamp = self._now(now)
        with self._lock:
            if type(count) is not int or not 1 <= count <= 100:
                raise ValueError("通信检查次数必须是 1～100 的整数")
            if (
                not getattr(self.endpoint, "is_open", False)
                or self.state != ChassisState.IDLE
                or self.pending is not None
                or self.communication_check is not None
            ):
                return False
            if self._effective_capability_mode() != "mm_ping_v1":
                self._emit("chassis_rejected", (self.connection_generation, "通信检查需要已确认的 PING 能力"), stamp)
                return False
            self.automatic_locked = True
            self.communication_check = CommunicationCheck(
                requested=count,
                wait_started_at=stamp,
            )
            self._state(ChassisState.CHECKING, f"通信检查 0/{count}", stamp)
            return True

    def complete_settle(self, *, resume_auto: bool, now: float | None = None) -> bool:
        stamp = self._now(now)
        with self._lock:
            if self.state != ChassisState.SETTLING or self.pending is None:
                return False
            action = self.pending
            if resume_auto and action.source == "auto" and not action.stop_requested:
                self._state(ChassisState.WAITING_SCAN, "等待停稳后的新完整扫描", stamp)
                self._emit("chassis_waiting_scan", (self.connection_generation, action), stamp)
                return True
            self.pending = None
            self.automatic_locked = True
            target = ChassisState.IDLE if self.confirmed else ChassisState.CONNECTED_WAITING
            self._state(target, "动作已结束，地图参考点需要重新确认", stamp)
            self._emit("chassis_ready", (
                self.connection_generation, action, "动作已结束，地图参考点需要重新确认"
            ), stamp)
            return True

    def mark_scan_ready(self, now: float | None = None) -> bool:
        stamp = self._now(now)
        with self._lock:
            if self.state != ChassisState.WAITING_SCAN or self.pending is None:
                return False
            action = self.pending
            self.pending = None
            self._state(ChassisState.IDLE, "新完整扫描已通过定位与地图更新", stamp)
            self._emit("chassis_ready", (
                self.connection_generation, action, "新完整扫描已通过定位与地图更新"
            ), stamp)
            return True

    def mark_execution_failed(self, detail: str, now: float | None = None) -> None:
        with self._lock:
            self._fault_and_stop(detail, self._now(now))

    def poll(self, now: float | None = None) -> None:
        stamp = self._now(now)
        with self._lock:
            if self.communication_check is not None and self.state == ChassisState.CHECKING:
                self._poll_communication_check(stamp)
                return
            action = self.pending
            if action is not None and self.state == ChassisState.WAITING_SILENCE:
                if self._phase_deadline is not None and stamp >= self._phase_deadline:
                    self._abort_before_move("底盘持续有数据，静默等待超时，MOVE 未发送", stamp)
                elif stamp - self._last_rx_at >= self._silence_before_ping():
                    if self._effective_capability_mode() == "mm_ping_v1":
                        self._send_motion_ping(action, stamp)
                    else:
                        self._queue_move(action, stamp)
            elif action is not None and self.state == ChassisState.WAITING_PONG:
                if self._phase_deadline is not None and stamp >= self._phase_deadline:
                    self._abort_before_move("PING 超时，MOVE 未发送", stamp)
            elif action is not None and self.state == ChassisState.WAITING_POST_PONG_SILENCE:
                if self._phase_deadline is not None and stamp >= self._phase_deadline:
                    self._abort_before_move("PONG 后持续有数据，MOVE 未发送", stamp)
                elif stamp - self._last_rx_at >= self._silence_after_pong():
                    self._queue_move(action, stamp)
            elif action is not None and self.state == ChassisState.PENDING_SEND:
                if self._phase_deadline is not None and stamp >= self._phase_deadline:
                    self.request_stop(reason="MOVE 写入未开始", now=stamp)
            elif action is not None and self.state == ChassisState.WAITING_ACK:
                if self._ack_deadline is not None and stamp >= self._ack_deadline:
                    action.ack_missing = True
                    self._ack_deadline = None
                    self._state(ChassisState.WAITING_DONE, "ACK 缺失，继续等待匹配 DONE，禁止重发", stamp)
                    self._emit("chassis_ack_missing", (self.connection_generation, action), stamp)
            if (
                action is not None
                and action.move_may_have_started
                and self.state in {ChassisState.WAITING_ACK, ChassisState.WAITING_DONE}
                and self._total_deadline is not None
                and stamp >= self._total_deadline
            ):
                self._emit("chassis_timeout", (
                    self.connection_generation, action, "动作总时限到达，停止结果不明"
                ), stamp)
                self.request_stop(reason="动作总超时", now=stamp)
            if self.state in {ChassisState.STOPPING, ChassisState.STOPPING_IDLE}:
                if self._status_due is not None and not self._status_sent and stamp >= self._status_due:
                    self.request_status(stamp)
                if self._stop_deadline is not None and stamp >= self._stop_deadline:
                    self._mark_unknown("停止反馈超时，物理停止未确认", stamp)

    def handle_transport_error(
        self,
        generation: int,
        message: str,
        host_time: float | None = None,
    ) -> None:
        with self._lock:
            if generation != self.connection_generation:
                return
            stamp = self._now(host_time)
            if self.pending is not None and self.pending.move_may_have_started:
                self._motion_unconfirmed = True
            self._mark_unknown(f"底盘串口异常：{message}", stamp)

    def _effective_capability_mode(self) -> str:
        if self.observed_capability_mode == "mm_ping_v1":
            return "mm_ping_v1"
        if bool(self.config.get("chassis_firmware_confirmed", False)):
            configured = str(self.config.get("chassis_capability_mode", "unknown"))
            if configured in {"cnt_only", "mm_ping_v1"}:
                return configured
        return "unknown"

    def _send_motion_ping(self, action: ChassisAction, stamp: float) -> None:
        nonce = self.nonce_factory()
        try:
            payload = encode_ping(nonce, max_command_bytes=int(
                self.config.get("chassis_tx_max_command_bytes", 47)
            ))
        except ChassisProtocolError as exc:
            self._abort_before_move(str(exc), stamp)
            return
        action.nonce = nonce
        action.ping_sent_at = None
        self._tx_epoch += 1
        epoch = self._tx_epoch
        self._phase_deadline = stamp + self._ping_timeout()
        self._state(ChassisState.WAITING_PONG, f"等待 PONG {nonce}", stamp)
        sent = self._queue_write(
            payload,
            action_id=action.action_id,
            epoch=epoch,
            on_sent=lambda sent_at: self._set_ping_sent(action, epoch, sent_at),
            on_failed=lambda failed_at, written, total, detail: self._on_preflight_write_failed(
                action, epoch, failed_at, written, total, detail
            ),
        )
        if not sent:
            self._abort_before_move("PING 未能排队，MOVE 未发送", stamp)

    def _set_ping_sent(self, action: ChassisAction, epoch: int, stamp: float) -> None:
        with self._lock:
            if epoch == self._tx_epoch and self.pending is action:
                action.ping_sent_at = stamp
                self._phase_deadline = stamp + self._ping_timeout()

    def _queue_move(self, action: ChassisAction, stamp: float) -> None:
        if self.pending is not action:
            return
        self._tx_epoch += 1
        epoch = self._tx_epoch
        self._phase_deadline = stamp + self._silence_wait_timeout()
        self._state(ChassisState.PENDING_SEND, "MOVE 已入发送队列，尚未开始写入", stamp)
        ticket = self._queue_write(
            action.payload,
            action_id=action.action_id,
            epoch=epoch,
            return_ticket=True,
            on_sent=lambda sent_at: self._on_move_send_started(action, epoch, sent_at),
            on_written=lambda written_at: self._on_move_write_completed(action, epoch, written_at),
            on_failed=lambda failed_at, written, total, detail: self._on_move_write_failed(
                action, epoch, failed_at, written, total, detail
            ),
            on_cancelled=lambda cancelled_at: self._on_move_cancelled(action, epoch, cancelled_at),
        )
        if ticket is None:
            self._abort_before_move("MOVE 未能排队发送", stamp)
        else:
            action.write_ticket = ticket

    def _on_move_send_started(self, action: ChassisAction, epoch: int, stamp: float) -> None:
        with self._lock:
            if epoch != self._tx_epoch or self.pending is not action:
                return
            action.send_started_at = stamp
            action.move_may_have_started = True
            self._motion_unconfirmed = True
            self._phase_deadline = None
            self._ack_deadline = stamp + self._ack_timeout()
            self._total_deadline = stamp + self._total_timeout()
            self._state(ChassisState.WAITING_ACK, "MOVE 开始写入，等待 ACK 或完整 DONE", stamp)
            self._emit("chassis_move_sent", (self.connection_generation, action), stamp)

    def _on_move_write_completed(self, action: ChassisAction, epoch: int, stamp: float) -> None:
        with self._lock:
            if epoch != self._tx_epoch or self.pending is not action:
                return
            action.write_completed_at = stamp
            self._emit("chassis_move_written", (self.connection_generation, action), stamp)

    def _on_move_write_failed(
        self,
        action: ChassisAction,
        epoch: int,
        stamp: float,
        written: int,
        total: int,
        detail: str,
    ) -> None:
        with self._lock:
            if epoch != self._tx_epoch or self.pending is not action:
                return
            if written > 0:
                action.move_may_have_started = True
                self._motion_unconfirmed = True
            self._emit("chassis_tx_failed", (
                self.connection_generation, action.action_id, written, total, detail
            ), stamp)
            self._fault_and_stop(f"MOVE 写入失败 {written}/{total}：{detail}", stamp)

    def _on_move_cancelled(self, action: ChassisAction, epoch: int, stamp: float) -> None:
        with self._lock:
            if self.pending is action and not action.move_may_have_started:
                self._emit("chassis_status", (
                    self.connection_generation, "未开始写入的 MOVE 已取消"
                ), stamp)

    def _on_preflight_write_failed(
        self,
        action: ChassisAction,
        epoch: int,
        stamp: float,
        written: int,
        total: int,
        detail: str,
    ) -> None:
        with self._lock:
            if epoch == self._tx_epoch and self.pending is action:
                self._abort_before_move(f"PING 写入失败 {written}/{total}：{detail}", stamp)

    def _on_stop_write_failed(
        self,
        epoch: int,
        stamp: float,
        written: int,
        total: int,
        detail: str,
    ) -> None:
        with self._lock:
            if epoch == self._tx_epoch:
                self._motion_unconfirmed = self.move_may_have_started or self._motion_unconfirmed
                self._mark_unknown(f"停止字节写入失败 {written}/{total}：{detail}", stamp)

    def _queue_write(
        self,
        payload: bytes,
        *,
        action_id: int | None,
        epoch: int,
        priority: bool = False,
        discard_pending: bool = False,
        return_ticket: bool = False,
        on_sent=None,
        on_written=None,
        on_failed=None,
        on_cancelled=None,
    ):
        generation = self.connection_generation

        def sent(stamp):
            if generation == self.connection_generation and epoch == self._tx_epoch and on_sent:
                on_sent(stamp)

        def written(stamp):
            if generation != self.connection_generation or epoch != self._tx_epoch:
                return
            self._emit("chassis_raw_tx", (generation, action_id, bytes(payload)), stamp)
            if on_written:
                on_written(stamp)

        def failed(stamp, count, total, detail):
            if generation == self.connection_generation and epoch == self._tx_epoch and on_failed:
                on_failed(stamp, count, total, detail)

        def cancelled(stamp):
            if generation == self.connection_generation and epoch == self._tx_epoch and on_cancelled:
                on_cancelled(stamp)

        writer = getattr(self.endpoint, "write_ticket", None)
        if callable(writer):
            ticket = writer(
                payload,
                on_sent=sent,
                on_written=written,
                on_failed=failed,
                on_cancelled=cancelled,
                priority=priority,
                discard_pending=discard_pending,
            )
            return ticket if return_ticket else ticket is not None
        writer = getattr(self.endpoint, "write", None)
        if not callable(writer):
            return None if return_ticket else False
        try:
            ok = writer(
                payload,
                on_sent=sent,
                on_written=written,
                on_failed=failed,
                on_cancelled=cancelled,
                priority=priority,
                discard_pending=discard_pending,
            )
        except TypeError:
            try:
                ok = writer(payload, on_sent=sent, on_written=written, priority=priority)
            except TypeError:
                ok = writer(payload, on_sent=sent)
        if return_ticket:
            return object() if ok else None
        return bool(ok)

    def _fault_and_stop(self, detail: str, stamp: float) -> None:
        action = self.pending
        may_move = self._motion_unconfirmed or bool(action and action.move_may_have_started)
        self.automatic_locked = True
        if may_move and getattr(self.endpoint, "is_open", False):
            self._motion_unconfirmed = True
            self._emit_fault(detail, stamp)
            self.request_stop(reason=detail, now=stamp)
        else:
            self._mark_unknown(detail, stamp)

    def _abort_before_move(self, detail: str, stamp: float) -> None:
        action = self.pending
        if action is not None and action.move_may_have_started:
            self._fault_and_stop(detail, stamp)
            return
        self.pending = None
        self._clear_action_deadlines()
        self.automatic_locked = True
        target = ChassisState.IDLE if self.confirmed else ChassisState.CONNECTED_WAITING
        self._state(target, detail, stamp)
        self._emit("chassis_rejected", (self.connection_generation, detail), stamp)

    def _emit_fault(self, detail: str, stamp: float) -> None:
        if not self._fault_emitted:
            self._fault_emitted = True
            self._emit("chassis_fault", (self.connection_generation, detail), stamp)

    def _mark_unknown(self, detail: str, stamp: float) -> None:
        self.automatic_locked = True
        self._clear_deadlines()
        self.state = ChassisState.UNKNOWN
        self._emit_fault(detail, stamp)
        self._emit("chassis_state", (self.connection_generation, self.state, detail), stamp)

    def _send_check_ping(self, check: CommunicationCheck, stamp: float) -> None:
        nonce = self.nonce_factory()
        try:
            payload = encode_ping(nonce, max_command_bytes=int(
                self.config.get("chassis_tx_max_command_bytes", 47)
            ))
        except ChassisProtocolError as exc:
            self._abort_communication_check(stamp, str(exc))
            return
        check.nonce = nonce
        check.phase = "waiting_pong"
        check.sent_at = None
        self._tx_epoch += 1
        epoch = self._tx_epoch

        def on_sent(sent_at):
            with self._lock:
                if self.communication_check is check and epoch == self._tx_epoch:
                    check.sent_at = sent_at
                    check.deadline = sent_at + self._ping_timeout()

        def on_failed(failed_at, written, total, detail):
            with self._lock:
                if self.communication_check is check and epoch == self._tx_epoch:
                    self._fail_check_attempt(failed_at, f"PING 写入失败 {written}/{total}：{detail}")

        if not self._queue_write(
            payload,
            action_id=None,
            epoch=epoch,
            on_sent=on_sent,
            on_failed=on_failed,
        ):
            self._fail_check_attempt(stamp, "PING 未能排队")

    def _poll_communication_check(self, stamp: float) -> None:
        check = self.communication_check
        if check is None:
            return
        if check.phase == "waiting_silence":
            if stamp - check.wait_started_at >= self._silence_wait_timeout():
                self._fail_check_attempt(stamp, "持续 RX，PING 未发送")
            elif stamp - self._last_rx_at >= self._silence_before_ping():
                self._send_check_ping(check, stamp)
        elif check.phase == "waiting_pong" and check.deadline is not None and stamp >= check.deadline:
            self._fail_check_attempt(stamp, "PONG 超时")

    def _fail_check_attempt(self, stamp: float, detail: str) -> None:
        check = self.communication_check
        if check is None:
            return
        if check.current_retry == 0:
            check.current_retry = 1
            check.phase = "waiting_silence"
            check.wait_started_at = stamp
            check.deadline = None
            check.nonce = None
            check.sent_at = None
            self._emit("chassis_communication_progress", {
                **self._check_snapshot(check), "detail": f"首发失败，准备重试：{detail}"
            }, stamp)
            return
        check.failures += 1
        check.completed += 1
        self._emit("chassis_communication_progress", {
            **self._check_snapshot(check), "detail": f"本次检查失败：{detail}"
        }, stamp)
        if check.completed >= check.requested:
            self._finish_communication_check(stamp)
        else:
            self._prepare_next_check_sample(check, stamp)

    def _prepare_next_check_sample(self, check: CommunicationCheck, stamp: float) -> None:
        check.current_retry = 0
        check.phase = "waiting_silence"
        check.wait_started_at = stamp
        check.deadline = None
        check.nonce = None
        check.sent_at = None

    def _check_snapshot(self, check: CommunicationCheck) -> dict[str, object]:
        latencies = sorted(check.latencies_ms)
        p95 = None
        if latencies:
            p95 = latencies[min(len(latencies) - 1, math.ceil(len(latencies) * 0.95) - 1)]
        return {
            "requested": check.requested,
            "completed": check.completed,
            "first_successes": check.first_successes,
            "retry_successes": check.retry_successes,
            "failures": check.failures,
            "first_success_rate": check.first_successes / check.requested,
            "retry_recovery_rate": check.retry_successes / check.requested,
            "latency_min_ms": min(latencies) if latencies else None,
            "latency_median_ms": statistics.median(latencies) if latencies else None,
            "latency_p95_ms": p95,
            "latency_max_ms": max(latencies) if latencies else None,
            "restarts": check.restarts,
        }

    def _finish_communication_check(self, stamp: float) -> None:
        check = self.communication_check
        if check is None:
            return
        result = self._check_snapshot(check)
        self.communication_check = None
        self._state(ChassisState.IDLE, "通信检查完成", stamp)
        self._emit("chassis_communication_result", (self.connection_generation, result), stamp)

    def _abort_communication_check(self, stamp: float, detail: str) -> None:
        check = self.communication_check
        if check is None:
            return
        result = {**self._check_snapshot(check), "aborted": True, "detail": detail}
        self.communication_check = None
        self.automatic_locked = True
        self._state(ChassisState.CONNECTED_WAITING, detail, stamp)
        self._emit("chassis_communication_result", (self.connection_generation, result), stamp)

    def _clear_action_deadlines(self) -> None:
        self._phase_deadline = None
        self._ack_deadline = None
        self._total_deadline = None

    def _clear_deadlines(self) -> None:
        self._clear_action_deadlines()
        self._stop_deadline = None
        self._status_due = None
        self._status_sent = False

    def _number(self, key: str, fallback: float) -> float:
        try:
            value = float(self.config.get(key, fallback))
        except (TypeError, ValueError, OverflowError):
            return fallback
        return value if math.isfinite(value) else fallback

    def _silence_before_ping(self) -> float:
        return max(0.0, self._number("chassis_rx_silence_before_ping_s", 0.5))

    def _silence_after_pong(self) -> float:
        return max(0.0, self._number("chassis_rx_silence_after_pong_s", 0.25))

    def _silence_wait_timeout(self) -> float:
        return max(0.1, self._number("chassis_silence_wait_timeout_s", 4.0))

    def _ping_timeout(self) -> float:
        return max(0.1, self._number("chassis_ping_timeout_s", 2.0))

    def _ack_timeout(self) -> float:
        return max(0.1, self._number("chassis_ack_timeout_s", 2.0))

    def _total_timeout(self) -> float:
        return max(0.5, self._number("chassis_total_timeout_s", 12.0))

    def _stop_timeout(self) -> float:
        return max(0.5, self._number("chassis_stop_timeout_s", 3.0))

    def _stop_status_delay(self) -> float:
        return max(0.0, self._number("chassis_stop_status_delay_s", 0.35))
