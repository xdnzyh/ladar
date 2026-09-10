from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
import secrets
import statistics
import threading
import time
from chassis_config1 import ConfigExchange, encode as encode_config, values_crc
from typing import Callable, Mapping

from chassis_protocol import (
    ChassisAck,
    ChassisCapabilityMarker,
    ChassisDone,
    ChassisFrame,
    ChassisProtocolError,
    ChassisResult,
    ChassisStreamParser,
    EXPECTED_STARTUP_MARKERS,
    STOP_SEQUENCE,
    TRANSLATION_MODES,
    encode_move,
    encode_ping,
    encode_result_query,
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
            if preferred == "MM" and not self.config.get("chassis_config1_file"):
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
            if self.config.get("chassis_config1_file"):
                counts = int(math.floor(request_value * float(entry["counts_per_mm"]) + 0.5))
                minimum = int(self.config.get("chassis_min_counts", 1))
                maximum = int(self.config.get("chassis_max_counts", 2_147_483_647))
                if not minimum <= counts <= maximum:
                    raise MotionConversionError(f"换算后的 CNT 必须在 {minimum}～{maximum} 之间")
                return ChassisMoveRequest(mode, counts, "CNT", counts, distance, "m")
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
    ping_write_ticket: object | None = None
    handled_report_raw: str | None = None
    result_recovery: bool = False
    result_invalidated: bool = False
    result_due_at: float | None = None
    result_queries: int = 0
    result_query_pending: bool = False
    result_query_ticket: object | None = None

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
class IdlePreflight:
    nonce: str
    deadline: float
    sent_at: float | None = None
    written: bool = False
    pong_at: float | None = None
    ticket: object | None = None


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
        self.config_exchange = None
        self.config_verified = False
        self._idle_preflight: IdlePreflight | None = None
        self._idle_preflight_failed = False
        self._idle_preflight_announced = False
        self._idle_status_pending = False
        self._idle_status_deferred = False
        self._idle_preflight_drain_until = -math.inf

    def update_config(self, config: Mapping[str, object]) -> bool:
        with self._lock:
            if self.in_flight or self.communication_check is not None:
                return False
            self._invalidate_idle_preflight()
            same_values = (self.config.get("chassis_config1_file") == config.get("chassis_config1_file")
                           and self.config.get("chassis_config1_values") == config.get("chassis_config1_values"))
            self.config = config
            self.config_verified = self.config_verified and same_values
            self.parser.max_line_bytes = int(config.get("chassis_max_line_bytes", 1024))
            return True

    @property
    def in_flight(self) -> bool:
        return bool(
            self.config_exchange is not None
            or self.pending is not None
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
            and self.config_exchange is None and not self._idle_status_pending
            and (not self.config.get("chassis_config1_file") or self.config_verified)
        )

    @property
    def can_scan(self) -> bool:
        return self.state in {ChassisState.IDLE, ChassisState.CONNECTED_WAITING}

    def _idle_preflight_safe(self) -> bool:
        return bool(
            self.config.get("chassis_idle_preflight", False)
            and getattr(self.endpoint, "is_open", False)
            and self.state == ChassisState.IDLE and self.confirmed
            and self.pending is None and not self._motion_unconfirmed
            and self.config_exchange is None and self.communication_check is None
            and not self._idle_status_pending
            and (not self.config.get("chassis_config1_file") or self.config_verified)
            and self._effective_capability_mode() == "mm_ping_v1"
            and callable(getattr(self.endpoint, "write_ticket", None))
            and callable(getattr(self.endpoint, "cancel_write", None))
        )

    @property
    def idle_preflight_ready(self) -> bool:
        with self._lock:
            return self._idle_preflight_ready_at(self.clock())

    def _idle_preflight_ready_at(self, stamp: float) -> bool:
        prep = self._idle_preflight
        return bool(self._idle_preflight_safe() and prep is not None
                    and prep.pong_at is not None and prep.written
                    and stamp - self._last_rx_at >= self._silence_after_pong())

    def _report_idle_preflight(self, ready: bool, stamp: float) -> None:
        if ready != self._idle_preflight_announced:
            self._idle_preflight_announced = ready
            self._emit("chassis_status", (
                self.connection_generation, f"IDLE_PREFLIGHT={int(ready)}"
            ), stamp)

    def _invalidate_idle_preflight(self) -> None:
        prep = self._idle_preflight
        self._idle_preflight = None
        self._idle_preflight_failed = False
        if prep is not None:
            # Epochs suppress callbacks, but do not remove bytes from the queue.
            # Cancel the actual ticket before any replacement transaction.
            self._tx_epoch += 1
            cancelled = None
            if prep.ticket is not None:
                cancelled = self.endpoint.cancel_write(prep.ticket)
            if prep.pong_at is None and (prep.sent_at is not None or cancelled == "started"):
                deadline = prep.deadline
                if not prep.written:
                    deadline = max(deadline, self.clock() + self._ping_timeout())
                self._idle_preflight_drain_until = max(self._idle_preflight_drain_until, deadline)
        self._report_idle_preflight(False, self.clock())

    def _idle_preflight_draining(self, stamp: float) -> bool:
        if self._idle_preflight_drain_until == -math.inf:
            return False
        # A started PING cannot be unsent. Let its response window expire,
        # then leave a full quiet interval before another protocol transaction.
        if stamp - max(self._idle_preflight_drain_until, self._last_rx_at) < self._silence_before_ping():
            return True
        self._idle_preflight_drain_until = -math.inf
        return False

    def _fail_idle_preflight(self, stamp: float) -> None:
        prep = self._idle_preflight
        self._invalidate_idle_preflight()
        self._idle_preflight_failed = True
        action = self.pending
        if prep is not None and action is not None and action.nonce == prep.nonce:
            action.nonce = None
            action.ping_sent_at = None
            self._phase_deadline = stamp + self._silence_wait_timeout()
            self._state(ChassisState.WAITING_SILENCE, "空闲 PING 失效，执行常规握手", stamp)

    def _poll_idle_preflight(self, stamp: float) -> None:
        if self._idle_preflight_draining(stamp):
            return
        if not self._idle_preflight_safe():
            self._invalidate_idle_preflight()
            return
        prep = self._idle_preflight
        if prep is not None:
            ready = self._idle_preflight_ready_at(stamp)
            self._report_idle_preflight(ready, stamp)
            if ready:
                prep.deadline = math.inf
            if not ready and stamp >= prep.deadline:
                self._fail_idle_preflight(stamp)
            return
        if self._idle_preflight_failed or stamp - self._last_rx_at < self._silence_before_ping():
            return
        nonce = self.nonce_factory()
        try:
            payload = encode_ping(nonce, max_command_bytes=int(
                self.config.get("chassis_tx_max_command_bytes", 47)))
        except ChassisProtocolError:
            self._idle_preflight_failed = True
            return
        prep = IdlePreflight(nonce, stamp + self._ping_timeout())
        self._idle_preflight = prep
        self._tx_epoch += 1
        epoch = self._tx_epoch

        def sent(sent_at):
            with self._lock:
                if self._idle_preflight is prep:
                    prep.sent_at = sent_at
                    prep.deadline = sent_at + self._ping_timeout()
                    if self.pending is not None and self.pending.nonce == prep.nonce:
                        self.pending.ping_sent_at = sent_at
                        self._phase_deadline = prep.deadline

        def written(written_at):
            with self._lock:
                if self._idle_preflight is prep:
                    prep.written = True
                    prep.ticket = None

        def failed(failed_at, *_args):
            with self._lock:
                if self._idle_preflight is prep:
                    self._fail_idle_preflight(failed_at)

        ticket = self._queue_write(
            payload, action_id=None, epoch=epoch, return_ticket=True,
            on_sent=sent, on_written=written, on_failed=failed, on_cancelled=failed,
        )
        if ticket is None:
            failed(stamp)
        elif self._idle_preflight is prep and not prep.written:
            prep.ticket = ticket

    def _now(self, value: float | None) -> float:
        return self.clock() if value is None else float(value)

    def _emit(self, kind: str, value: object, stamp: float | None = None) -> None:
        self.emit(kind, value, self.clock() if stamp is None else float(stamp))

    def _state(self, state: str, detail: str, stamp: float) -> None:
        self.state = state
        self._emit("chassis_state", (self.connection_generation, state, detail), stamp)

    def begin_connection(self) -> int:
        with self._lock:
            self._cancel_motion_ping()
            self._invalidate_idle_preflight()
            self._idle_status_pending = False
            self._idle_status_deferred = False
            self._invalidate_result_recovery()
            self._cancel_config_exchange()
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
            self._cancel_motion_ping()
            self._invalidate_idle_preflight()
            self._idle_status_pending = False
            self._idle_status_deferred = False
            self._invalidate_result_recovery()
            self._cancel_config_exchange()
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
            self._report_idle_preflight(False, host_time)
            action_id = self.pending.action_id if self.pending is not None else None
            self._emit("chassis_raw_rx", (generation, action_id, bytes(data)), host_time)
            frames = self.parser.feed(data)
        for frame in frames:
            self._emit("chassis_frame", (generation, frame), host_time)

    def handle_frame(self, generation: int, frame: ChassisFrame, host_time: float) -> None:
        with self._lock:
            if generation != self.connection_generation:
                return
            if frame.kind == "config1":
                if self.config_exchange is not None:
                    self.config_exchange.feed(frame.raw, host_time)
                return
            if self.config_exchange is not None and frame.kind in {"invalid", "error"}:
                self.config_exchange.fail(f"配置通信失败：{frame.raw or frame.error}")
                return
            if frame.kind == "empty":
                return
            if frame.kind == "invalid":
                self._emit("chassis_protocol_error", (generation, frame.raw, frame.error), host_time)
                if (self.pending is not None and self.pending.move_may_have_started
                        and not self.pending.result_recovery):
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
                if ((self.pending is not None and self.pending.result_recovery)
                        or self.config.get("chassis_result_recovery", False)):
                    self._emit("chassis_status", (generation, "忽略未校验的旧版 DONE"), host_time)
                    return
                self._handle_done(frame.value, host_time)
                return
            if frame.kind == "result":
                self._handle_result(frame.value, host_time)
                return
            if frame.kind == "error":
                self._handle_error(frame.value, host_time)
                return
            if frame.kind == "log":
                self._emit("chassis_status", (generation, frame.raw), host_time)

    def _handle_startup(self, raw: str, stamp: float) -> None:
        self._cancel_motion_ping()
        self._invalidate_idle_preflight()
        self._idle_status_pending = False
        self._idle_status_deferred = False
        self._invalidate_result_recovery()
        self._cancel_config_exchange()
        self._tx_epoch += 1
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
        if not self._idle_status_deferred:
            self._idle_status_pending = False
        self.last_idle = status
        if self.state in {ChassisState.STOPPING, ChassisState.STOPPING_IDLE, ChassisState.UNKNOWN}:
            action = self.pending
            self._cancel_result_query()
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
        prep = self._idle_preflight
        if prep is not None and prep.nonce == pong.nonce and prep.sent_at is None:
            return
        if prep is not None and prep.sent_at is not None and pong.nonce == prep.nonce:
            if prep.pong_at is None:
                prep.pong_at = stamp
                prep.deadline = stamp + self._silence_wait_timeout()
            if self.pending is None:
                return
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
                if action.result_recovery and action.result_queries == 0:
                    action.result_due_at = stamp + 1.8
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

    def _handle_result(self, result: ChassisResult, stamp: float) -> None:
        action = self.pending
        if (action is None or not action.result_recovery or action.result_invalidated
                or action.connection_generation != self.connection_generation
                or result.nonce != action.nonce or not action.move_may_have_started
                or action.done_at is not None
                or self.state not in {ChassisState.WAITING_ACK, ChassisState.WAITING_DONE,
                                      ChassisState.STOPPING}):
            self._emit("chassis_status", (self.connection_generation, "忽略无归属或过期 RESULT"), stamp)
            return
        if result.report is None:
            if result.status == "B" and self.state in {ChassisState.WAITING_ACK, ChassisState.WAITING_DONE}:
                self._ack_deadline = None
                self._state(ChassisState.WAITING_DONE, "RESULT 确认动作仍在运行", stamp)
            self._emit("chassis_status", (self.connection_generation, result.raw), stamp)
            return
        report = result.report
        if not self._matches(action, report.mode, report.request_value, report.unit):
            self._emit("chassis_protocol_error", (
                self.connection_generation, result.raw, "RESULT 与当前请求不匹配"
            ), stamp)
            return
        coherent, detail = validate_done_kinematics(report)
        if not coherent:
            self._emit("chassis_protocol_error", (self.connection_generation, result.raw, detail), stamp)
            return
        if report.unit == "MM":
            entry = ChassisMotionAdapter(self.config)._translation_entry(report.mode)
            try:
                target = firmware_mm_to_counts(
                    report.request_value, fixed_counts_per_mm=int(entry.get("fixed_counts_per_mm", 0))
                )
            except (ChassisProtocolError, TypeError, ValueError, OverflowError) as exc:
                self._emit("chassis_protocol_error", (self.connection_generation, result.raw, str(exc)), stamp)
                return
            if action.target_counts is not None and action.target_counts != target:
                self._emit("chassis_protocol_error", (
                    self.connection_generation, result.raw, "RESULT MM 换算与请求不一致"
                ), stamp)
                return
            report = replace(report, target_counts=target)
        self._handle_done(report, stamp)

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
        if self._idle_preflight is not None and code != "BUSY":
            self._fail_idle_preflight(stamp)
            return
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
            if self.config.get("chassis_config1_file") and not self.config_verified:
                self._emit("chassis_rejected", (self.connection_generation,
                           "参数尚未确认，请先读取或同步一次；后续动作沿用确认状态"), stamp)
                return False
            if source == "auto" and not self.automatic_ready:
                self._emit("chassis_rejected", (self.connection_generation, "底盘尚未满足自动动作准入"), stamp)
                return False
            if self.config_exchange is not None or self.state != ChassisState.IDLE or self.pending is not None or self._motion_unconfirmed or self._idle_status_pending:
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
            if self.config.get("chassis_config1_file") and request.unit == "MM":
                self._emit("chassis_rejected", (self.connection_generation,
                           "启用电脑标定后，毫米请求须先经适配器换算为 CNT"), stamp)
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
                result_recovery=self.config.get("chassis_result_recovery", False) is True,
            )
            self.pending = action
            self.automatic_locked = source != "auto"
            self._fault_emitted = False
            prep = self._idle_preflight
            if prep is not None and stamp >= prep.deadline and prep.pong_at is None:
                self._invalidate_idle_preflight()
                prep = None
            if prep is not None:
                action.nonce = prep.nonce
                action.ping_sent_at = prep.sent_at
                self._report_idle_preflight(False, stamp)
                self._phase_deadline = (stamp + self._silence_wait_timeout()
                                        if prep.pong_at is not None else prep.deadline)
                state = (ChassisState.WAITING_POST_PONG_SILENCE if prep.pong_at is not None
                         else ChassisState.WAITING_PONG)
                self._state(state, "沿用空闲 PING，等待发送条件", stamp)
                if prep.pong_at is not None and prep.written and stamp - self._last_rx_at >= self._silence_after_pong():
                    self._queue_move(action, stamp)
                return True
            self._idle_preflight_failed = False
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
                and self.config_exchange is None
                and (not self.config.get("chassis_config1_file") or self.config_verified)
            ):
                self.automatic_locked = False
                return True
            return False

    def request_stop(self, *, reason: str = "用户停止", now: float | None = None) -> bool:
        stamp = self._now(now)
        with self._lock:
            self._idle_status_deferred = False
            self._cancel_motion_ping()
            self._invalidate_idle_preflight()
            self._cancel_config_exchange()
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
            if self.config_exchange is not None:
                return False
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
            self._invalidate_idle_preflight()
            if self.config.get("chassis_idle_preflight", False):
                self._idle_status_pending = True
            if self._idle_preflight_draining(stamp):
                self._idle_status_deferred = True
                return True
            self._idle_status_deferred = False
            self._tx_epoch += 1
            epoch = self._tx_epoch
            sent = self._queue_write(b"P\r\n", action_id=None, epoch=epoch, priority=False)
            if not sent:
                self._idle_status_pending = False
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
                or self.config_exchange is not None
                or self._idle_status_pending
            ):
                return False
            if self._effective_capability_mode() != "mm_ping_v1":
                self._emit("chassis_rejected", (self.connection_generation, "通信检查需要已确认的 PING 能力"), stamp)
                return False
            self._invalidate_idle_preflight()
            self.automatic_locked = True
            self.communication_check = CommunicationCheck(
                requested=count,
                wait_started_at=max(stamp, self._idle_preflight_drain_until),
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
            if self._idle_status_deferred and not self._idle_preflight_draining(stamp):
                self.request_status(stamp)
            if self.config_exchange is not None:
                if not self._idle_preflight_draining(stamp):
                    self.config_exchange.poll(stamp, self._last_rx_at)
                return
            if self.communication_check is not None and self.state == ChassisState.CHECKING:
                self._poll_communication_check(stamp)
                return
            if self.state == ChassisState.IDLE:
                self._poll_idle_preflight(stamp)
                return
            action = self.pending
            if action is not None and self.state == ChassisState.WAITING_SILENCE:
                if self._phase_deadline is not None and stamp >= self._phase_deadline:
                    self._abort_before_move("底盘持续有数据，静默等待超时，MOVE 未发送", stamp)
                elif not self._idle_preflight_draining(stamp) and stamp - self._last_rx_at >= self._silence_before_ping():
                    if action.result_recovery or self._effective_capability_mode() == "mm_ping_v1":
                        self._send_motion_ping(action, stamp)
                    else:
                        self._queue_move(action, stamp)
            elif action is not None and self.state == ChassisState.WAITING_PONG:
                if self._phase_deadline is not None and stamp >= self._phase_deadline:
                    if self._idle_preflight is not None:
                        self._fail_idle_preflight(stamp)
                    else:
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
            if (action is not None and self.pending is action
                    and self.state in {ChassisState.WAITING_ACK, ChassisState.WAITING_DONE}):
                self._poll_result_recovery(action, stamp)
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

    def _cancel_config_exchange(self):
        ticket = getattr(self, "_config_ticket", None)
        cancel = getattr(self.endpoint, "cancel_write", None)
        if ticket is not None and callable(cancel):
            cancel(ticket)
        self._config_ticket = None
        self.config_exchange = None
        self.config_verified = False

    def request_config_sync(self, *, apply: bool = True, now: float | None = None) -> bool:
        with self._lock:
            if (not self.config.get("chassis_config1_file") or self.in_flight
                    or self._idle_status_pending
                    or self.communication_check is not None or self.state != ChassisState.IDLE
                    or not getattr(self.endpoint, "is_open", False)):
                return False
            self.automatic_locked = True
            self._start_config_exchange("apply" if apply else "read", self._now(now))
            return True

    def _start_config_exchange(self, operation, stamp):
        self._invalidate_idle_preflight()
        target = self.config["chassis_config1_values"]
        self._tx_epoch += 1
        epoch = self._tx_epoch
        self.config_verified = False

        def failed(_stamp, _written, _total, detail):
            with self._lock:
                if epoch == self._tx_epoch and self.config_exchange is not None:
                    self.config_exchange.fail(detail)

        def send(payload):
            ticket = self._queue_write(
                payload, action_id=None, epoch=epoch,
                return_ticket=True,
                on_failed=failed,
            )
            self._config_ticket = ticket
            return ticket is not None

        def finish(actual, error):
            exchange = self.config_exchange
            self.config_exchange = None
            if error:
                self.automatic_locked = True
                # Drop queued config writes and abandon our staging transaction.
                self._tx_epoch += 1
                if exchange is not None:
                    self._queue_write(encode_config(f"@CFG,A,{exchange.tag}"),
                                      action_id=None, epoch=self._tx_epoch,
                                      priority=True, discard_pending=True)
                self._emit("chassis_status", (self.connection_generation, error), self.clock())
                return
            self.config_verified = actual == list(target)
            detail = (f"底盘参数 CRC={values_crc(actual)}；"
                      f"与电脑不同项 {sum(a != b for a, b in zip(actual, target))}")
            self._emit("chassis_status", (self.connection_generation, detail), self.clock())

        self.config_exchange = ConfigExchange(
            target, operation, send, finish, max(stamp, self._idle_preflight_drain_until))
        self._emit("chassis_status", (self.connection_generation,
                   "正在读取/同步底盘参数，请等待"), stamp)

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
        ticket = self._queue_write(
            payload,
            action_id=action.action_id,
            epoch=epoch,
            return_ticket=True,
            on_sent=lambda sent_at: self._set_ping_sent(action, epoch, sent_at),
            on_failed=lambda failed_at, written, total, detail: self._on_preflight_write_failed(
                action, epoch, failed_at, written, total, detail
            ),
        )
        if ticket is None:
            self._abort_before_move("PING 未能排队，MOVE 未发送", stamp)
        elif self.pending is action and self._tx_epoch == epoch:
            action.ping_write_ticket = ticket

    def _cancel_motion_ping(self) -> None:
        action = self.pending
        if action is not None and action.ping_write_ticket is not None:
            ticket = action.ping_write_ticket
            action.ping_write_ticket = None
            cancel = getattr(self.endpoint, "cancel_write", None)
            if callable(cancel):
                self._tx_epoch += 1
                cancel(ticket)

    def _set_ping_sent(self, action: ChassisAction, epoch: int, stamp: float) -> None:
        with self._lock:
            if epoch == self._tx_epoch and self.pending is action:
                action.ping_sent_at = stamp
                self._phase_deadline = stamp + self._ping_timeout()

    def _queue_move(self, action: ChassisAction, stamp: float) -> None:
        if self.pending is not action:
            return
        self._cancel_motion_ping()
        prep = self._idle_preflight
        if prep is not None:
            if not prep.written or prep.pong_at is None:
                return
            # Consume only after the PING write has completed. An active serial
            # write cannot be cancelled; the endpoint serializes it before MOVE.
            self._invalidate_idle_preflight()
        if self.config.get("chassis_config1_file") and not self.config_verified:
            self._abort_before_move("参数确认状态已失效，请重新读取或同步", stamp)
            return
        if action.result_recovery:
            try:
                # The successful idle PING arms this exact ID. Never fall back
                # to an unchecked MOVE if encoding or firmware acceptance fails.
                if action.nonce is None:
                    raise ChassisProtocolError("BAD_CMD", "受保护 MOVE 缺少 PING 标记")
                action.payload = encode_move(
                    action.mode, action.request_value, action.unit, nonce=action.nonce,
                    max_command_bytes=int(self.config.get("chassis_tx_max_command_bytes", 47)),
                )
            except ChassisProtocolError as exc:
                self._abort_before_move(str(exc), stamp)
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
            if action.result_recovery:
                action.result_due_at = stamp + 1.8
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
            if action.result_recovery and action.result_queries == 0 and action.done_at is None:
                action.result_due_at = max(action.result_due_at or stamp, stamp + 1.8)
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
        if payload.startswith((b"@PING,", b"@MOVE,", b"@CFG,", b"@RESULT,")):
            payload = b" " * int(self.config.get("chassis_tx_padding_spaces", 0)) + payload
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
        self._cancel_motion_ping()
        self._invalidate_idle_preflight()
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
        self._cancel_motion_ping()
        self._invalidate_idle_preflight()
        self._cancel_config_exchange()
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
        if self._idle_preflight_draining(stamp):
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

    def _cancel_result_query(self) -> None:
        action = self.pending
        if action is None:
            return
        ticket = action.result_query_ticket
        action.result_query_ticket = None
        action.result_query_pending = False
        action.result_due_at = None
        cancel = getattr(self.endpoint, "cancel_write", None)
        if ticket is not None and callable(cancel):
            cancel(ticket)

    def _invalidate_result_recovery(self) -> None:
        if self.pending is not None:
            self.pending.result_invalidated = True
        self._cancel_result_query()

    def _poll_result_recovery(self, action: ChassisAction, stamp: float) -> None:
        if (not action.result_recovery or action.result_invalidated
                or action.connection_generation != self.connection_generation
                or not action.move_may_have_started or action.done_at is not None
                or action.result_query_pending or action.result_queries >= 5
                or action.result_due_at is None or stamp < action.result_due_at
                or stamp - self._last_rx_at < 0.25
                or self._total_deadline is None or stamp >= self._total_deadline):
            return
        # Keep one snapshot query for a long-running move whose automatic
        # completion may be lost after the first four queries reported busy.
        if action.result_queries == 4 and stamp < self._total_deadline - 1.5:
            return
        payload = encode_result_query(
            action.nonce, max_command_bytes=int(self.config.get("chassis_tx_max_command_bytes", 47))
        )
        # Quiet RX may still contain a result whose terminator was lost. Log
        # the fragment as invalid and start the queried reply on a fresh line.
        for frame in self.parser.finalize():
            self._emit("chassis_frame", (self.connection_generation, frame), stamp)
        epoch = self._tx_epoch
        action.result_queries += 1
        attempt = action.result_queries
        action.result_query_pending = True
        action.result_due_at = stamp + 1.2

        def current():
            return (self.pending is action and epoch == self._tx_epoch
                    and action.connection_generation == self.connection_generation
                    and not action.result_invalidated and action.done_at is None
                    and action.result_query_pending and action.result_queries == attempt)

        def sent(sent_at):
            with self._lock:
                if current():
                    action.result_due_at = sent_at + 1.2

        def finished(finished_at):
            with self._lock:
                if current():
                    action.result_query_pending = False
                    action.result_query_ticket = None
                    action.result_due_at = finished_at + 1.2
                    if action.result_queries == 4 and self._total_deadline is not None:
                        action.result_due_at = max(action.result_due_at, self._total_deadline - 1.5)

        def failed(failed_at, written, total, detail):
            with self._lock:
                if current():
                    self._emit("chassis_protocol_error", (
                        self.connection_generation, payload.decode("ascii").strip(),
                        f"RESULT 查询写入失败 {written}/{total}：{detail}"
                    ), failed_at)
                    finished(failed_at)

        ticket = self._queue_write(
            payload, action_id=action.action_id, epoch=epoch, return_ticket=True,
            on_sent=sent, on_written=finished, on_failed=failed, on_cancelled=finished,
        )
        if ticket is None:
            finished(stamp)
        elif current():
            action.result_query_ticket = ticket

    def _clear_action_deadlines(self) -> None:
        self._cancel_result_query()
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
        values = self.config.get("chassis_config1_values", [])
        configured = self._number("chassis_total_timeout_s", 12.0)
        return max(configured, 14.0, values[76] / 1000.0 + 6.0) if values else max(0.5, configured)

    def _stop_timeout(self) -> float:
        return max(0.5, self._number("chassis_stop_timeout_s", 3.0))

    def _stop_status_delay(self) -> float:
        return max(0.0, self._number("chassis_stop_status_delay_s", 0.35))
