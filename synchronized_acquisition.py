from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import queue
import secrets
import time

from measurement_protocol import parse_observation
from radar_core import MotorLineParser
from scan_acquisition import DistanceObservationReceiver, ReceivedObservation


SYNC_REQUIRED_EXCHANGES = 8
SYNC_MAX_ATTEMPTS = 16
SYNC_RESPONSE_TIMEOUT_S = 1.0
SYNC_TOTAL_TIMEOUT_S = 20.0
SYNC_FATAL_ERRORS = {
    "CCD_TIMEOUT",
    "EXPOSURE_WRITE",
    "UART_WRITE",
    "TX_OVERFLOW",
    "RX_OVERFLOW",
    "IRQ_OVERFLOW",
    "WATCHDOG",
}
SYNC_START_ERRORS = {"START_ARGUMENTS"}


@dataclass(frozen=True)
class ClockEstimate:
    offset: float
    uncertainty: float
    observed_at: float
    drift_ppm: float = 500.0
    version: int = 0

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.offset, self.uncertainty, self.observed_at, self.drift_ppm))
                or self.uncertainty < 0 or not 0 <= self.drift_ppm < 1000000
                or not isinstance(self.version, int) or self.version < 0):
            raise ValueError("无效时钟误差预算")

    @classmethod
    def exchange(cls, t1: float, t2_us: int, t3_us: int, t4: float, drift_ppm: float = 500.0):
        t2, t3 = t2_us * 1e-6, t3_us * 1e-6
        if not all(math.isfinite(t) for t in (t1, t2, t3, t4)) or t4 < t1 or t3 < t2:
            raise ValueError("无效校时响应")
        network_time = (t4 - t1) - (t3 - t2)
        if network_time < -1e-6:
            raise ValueError("校时响应的设备处理时间超过往返时间")
        uncertainty = max(0.0, network_time) / 2 + (t4 - t1) * drift_ppm * 1e-6 + 1e-6
        return cls(((t2 - t1) + (t3 - t4)) / 2, uncertainty, t4, drift_ppm)

    def map(self, device_us: float, now: float) -> tuple[float, float]:
        timestamp = device_us * 1e-6 - self.offset
        drift = self.drift_ppm * 1e-6
        error = (self.uncertainty + abs(timestamp - self.observed_at) * drift) / (1 - drift)
        return timestamp, error

    def updated(self, measurement):
        age = abs(measurement.observed_at - self.observed_at)
        radius = self.uncertainty + age * self.drift_ppm * 1e-6
        low = max(self.offset - radius, measurement.offset - measurement.uncertainty)
        high = min(self.offset + radius, measurement.offset + measurement.uncertainty)
        if low > high:
            raise ValueError("连续校时区间不相容")
        offset = (low + high) / 2
        return ClockEstimate(offset, max(offset - low, high - offset),
                             measurement.observed_at, self.drift_ppm, self.version + 1)



class SynchronizedAcquisition:
    def __init__(self, measurement, rotation, calibration, config: dict, emit):
        self.endpoints = {"measurement": measurement, "rotation": rotation}
        self.calibration = calibration
        self.config = config
        self.emit = emit
        self.incoming = queue.Queue()
        self.parsers = {name: MotorLineParser() for name in self.endpoints}
        self.receiver = DistanceObservationReceiver(config)
        self.builder = self.receiver.builder
        self.state = "stopped"
        self.session = ""
        self.generation = 0
        self.sync_started_at = -math.inf
        self.pending = self.receiver.pending
        self.clocks = {}
        self.sent_times = {}
        self.last_report = ""
        self.sync_stats = self._new_sync_stats()
        self.received_bytes = {source: 0 for source in self.endpoints}
        self.ignored_counts = {source: {} for source in self.endpoints}
        self.recent_anomalies = deque(maxlen=8)
        self._last_diagnostic_at = -math.inf
        self._last_runtime_status_at = -math.inf
        self._last_runtime_signature = None
        self.stop_status = {source: {"written": False, "confirmed": False} for source in self.endpoints}
        self.stop_pending = set()
        self.stop_deadline = -math.inf
        self.last_arrival = {source: -math.inf for source in self.endpoints}
        self.last_communication = {source: -math.inf for source in self.endpoints}
        self.last_valid_range = -math.inf
        self.last_valid_rotation = -math.inf

    @staticmethod
    def _new_sync_stats():
        fields = ("sent", "success", "timeout", "token_mismatch", "format_error",
                  "missing_send_time", "invalid_timestamp", "ignored")
        return {source: {field: 0 for field in fields} for source in ("measurement", "rotation")}

    def feed(self, source: str, data: bytes, arrival: float):
        self.incoming.put((source, data, arrival))

    def start(self, now: float | None = None):
        self.stop(force=True)
        self._drain_incoming()
        now = time.perf_counter() if now is None else now
        self.generation += 1
        self.session = secrets.token_hex(6)
        self.state = "syncing"
        self.started_at = now
        self.sync_started_at = now
        self.receiver = DistanceObservationReceiver(self.config)
        self.builder = self.receiver.builder
        self.pending = self.receiver.pending
        self.raw_progress = {}
        self.last_keepalive = now
        self.clocks = {}
        self.exchanges = {source: [] for source in self.endpoints}
        self.outstanding = {}
        self.sent_times = {}
        self.attempts = {source: 0 for source in self.endpoints}
        self.next_probe = {source: now + 0.1 for source in self.endpoints}
        self.last_sequence = {}
        self.sync_stats = self._new_sync_stats()
        self.received_bytes = {source: 0 for source in self.endpoints}
        self.ignored_counts = {source: {} for source in self.endpoints}
        self.recent_anomalies.clear()
        self._last_diagnostic_at = -math.inf
        self._last_runtime_status_at = -math.inf
        self._last_runtime_signature = None
        self.stop_pending.clear()
        self.stop_deadline = -math.inf
        self.last_arrival = {source: now for source in self.endpoints}
        self.last_communication = {source: now for source in self.endpoints}
        self.last_valid_range = now
        self.last_valid_rotation = now
        self.last_report = ""
        for parser in self.parsers.values():
            parser.reset()
        self._status(self._sync_progress())

    def stop(self, force: bool = False):
        active = (self.state != "stopped" or bool(self.session)
                  or bool(getattr(self, "outstanding", {})))
        if not force and not active and all(item["written"] for item in self.stop_status.values()):
            self._drain_incoming()
            return {source: dict(status) for source, status in self.stop_status.items()}
        self._drain_incoming()
        self.generation += 1
        self.state = "stopped"
        self.session = ""
        self.pending.clear()
        self.receiver.reset(math.inf)
        self.sent_times.clear()
        self.stop_status = {}
        self.stop_pending = set()
        self.stop_deadline = time.perf_counter() + 0.5
        commands = {"measurement": ["STOP", "LASER 0"], "rotation": ["OFF"]}
        for source, lines in commands.items():
            written = self._stop_endpoint(self.endpoints[source], lines)
            self.stop_status[source] = {"written": written, "confirmed": False}
            if written:
                self.stop_pending.add(source)
            else:
                self._diagnostic(f"{self._endpoint_label(source)}停止命令未写入串口", force=True)
        for parser in self.parsers.values():
            parser.reset()
        return {source: dict(status) for source, status in self.stop_status.items()}

    def _drain_incoming(self):
        while True:
            try:
                self.incoming.get_nowait()
            except queue.Empty:
                return

    @staticmethod
    def _stop_endpoint(endpoint, lines, timeout: float = 0.5) -> bool:
        stop_and_flush = getattr(endpoint, "stop_and_flush", None)
        if callable(stop_and_flush):
            return bool(stop_and_flush(lines, timeout))
        cancel_pending = getattr(endpoint, "cancel_pending", None)
        if callable(cancel_pending):
            cancel_pending()
        sent = True
        for line in lines:
            try:
                written = endpoint.write_line(line, priority=True)
            except TypeError:
                written = endpoint.write_line(line)
            sent = bool(written) and sent
        flush = getattr(endpoint, "flush", None)
        if callable(flush):
            sent = bool(flush(timeout)) and sent
        return sent

    def begin_after(self, timestamp: float):
        self.receiver.reset(timestamp)

    def _status(self, message):
        if message != self.last_report:
            self.last_report = message
            self.emit("sync_status", message, time.perf_counter())

    @staticmethod
    def _age_text(now: float, timestamp: float, available: bool = True) -> str:
        if not available or not math.isfinite(timestamp):
            return "—"
        return f"{max(0.0, now - timestamp):.1f}s"

    def _runtime_status(self, now: float) -> str:
        raw_rotation = self.raw_progress.get("rotation")
        raw_seq = raw_rotation[0] if raw_rotation is not None else None
        valid_seq = self.last_sequence.get("rotation")
        anchor = getattr(self.builder, "anchor", None)
        anchor_seq = anchor[2] if anchor is not None and len(anchor) >= 3 else None
        period = getattr(self.builder, "period_s", None)
        period_text = "—" if period is None or not math.isfinite(period) else f"{period:.3f}s"
        valid_age = self._age_text(now, self.last_valid_rotation, valid_seq is not None)
        samples = len(getattr(self.builder, "samples", ()))
        conflicts = getattr(self.receiver, "timestamp_conflicts", {})
        measurement_time_bad = (
            int(self.sync_stats["measurement"].get("invalid_timestamp", 0))
            + int(conflicts.get("range", 0))
        )
        rotation_time_bad = (
            int(self.sync_stats["rotation"].get("invalid_timestamp", 0))
            + int(conflicts.get("rotation", 0))
        )
        rotation_ignored = sum(self.ignored_counts.get("rotation", {}).values())
        current = str(getattr(self.builder, "reason", "等待扫描"))
        last_failure = str(getattr(self.builder, "last_failure_reason", "") or "").strip()
        seq_text = "/".join("—" if value is None else str(value)
                            for value in (raw_seq, valid_seq, anchor_seq))
        lines = [
            current,
            f"TRIG 收/有效/锚点 {seq_text} ｜ 距有效TRIG {valid_age} ｜ 周期 {period_text}",
            (f"本圈 {samples} 点 ｜ 完整/丢弃/预热 {self.receiver.accepted}/"
             f"{self.receiver.discarded}/{self.receiver.warmup} ｜ 时间异常 测距{measurement_time_bad} "
             f"零位{rotation_time_bad} ｜ 旋转忽略 {rotation_ignored}"),
        ]
        if last_failure and last_failure != current:
            lines.append(f"最近失败：{last_failure}")
        return "\n".join(lines)

    def _publish_runtime_status(self, now: float) -> None:
        raw_rotation = self.raw_progress.get("rotation")
        raw_seq = raw_rotation[0] if raw_rotation is not None else None
        valid_seq = self.last_sequence.get("rotation")
        anchor = getattr(self.builder, "anchor", None)
        anchor_seq = anchor[2] if anchor is not None and len(anchor) >= 3 else None
        last_failure = str(getattr(self.builder, "last_failure_reason", "") or "")
        conflicts = getattr(self.receiver, "timestamp_conflicts", {})
        age_bucket = None
        if valid_seq is not None and math.isfinite(self.last_valid_rotation):
            age_bucket = int(max(0.0, now - self.last_valid_rotation))
        signature = (
            str(getattr(self.builder, "reason", "")),
            last_failure,
            raw_seq,
            valid_seq,
            anchor_seq,
            self.receiver.accepted,
            self.receiver.discarded,
            self.receiver.warmup,
            int(self.sync_stats["measurement"].get("invalid_timestamp", 0)),
            int(self.sync_stats["rotation"].get("invalid_timestamp", 0)),
            int(conflicts.get("range", 0)),
            int(conflicts.get("rotation", 0)),
            sum(self.ignored_counts.get("rotation", {}).values()),
            age_bucket,
        )
        if signature == self._last_runtime_signature and now - self._last_runtime_status_at < 1.0:
            return
        self._last_runtime_signature = signature
        self._last_runtime_status_at = now
        self._status(self._runtime_status(now))

    def _sync_progress(self):
        return (f"双端校时：测距 {len(self.exchanges.get('measurement', []))}/{SYNC_REQUIRED_EXCHANGES}，"
                f"旋转 {len(self.exchanges.get('rotation', []))}/{SYNC_REQUIRED_EXCHANGES}")

    def _endpoint_label(self, source):
        names = {"measurement": "测距端", "rotation": "旋转端"}
        port = str(getattr(self.endpoints[source], "port", "") or "").strip()
        return f"{names[source]}({port})" if port else names[source]

    def _phase_label(self):
        return {"syncing": "校时", "starting_measurement": "启动测距",
                "starting_rotation": "启动旋转", "running": "采集"}.get(self.state, "采集")

    def _diagnostic(self, message, force=False, timestamp=None):
        timestamp = time.perf_counter() if timestamp is None else timestamp
        if force or timestamp - self._last_diagnostic_at >= 1:
            self._last_diagnostic_at = timestamp
            self.emit("sync_diagnostic", message, timestamp)

    def _stat(self, source, field, amount=1):
        if source in self.sync_stats and field in self.sync_stats[source]:
            self.sync_stats[source][field] += amount

    def _record_ignored(self, source, line, reason, arrival, format_error=False, token_mismatch=False):
        self._stat(source, "ignored")
        if format_error:
            self._stat(source, "format_error")
        if token_mismatch:
            self._stat(source, "token_mismatch")
        counts = self.ignored_counts[source]
        counts[reason] = counts.get(reason, 0) + 1
        self.recent_anomalies.append((source, line[:200], reason, arrival))
        if arrival - self._last_diagnostic_at >= 1:
            total = sum(counts.values())
            self._diagnostic(
                f"{self._endpoint_label(source)}忽略{reason}：{line[:120]}（累计 {total} 条）",
                timestamp=arrival,
            )

    def _fail(self, message):
        if self.state == "stopped":
            return
        self.stop()
        self.emit("sync_error", message, time.perf_counter())

    def poll(self, now: float | None = None):
        now = time.perf_counter() if now is None else now
        if self.state == "stopped" and self.stop_pending and time.perf_counter() >= self.stop_deadline:
            pending = [self._endpoint_label(source) for source in self.stop_pending]
            self._diagnostic(f"停止确认超时：{','.join(pending)}已写入停止命令但未收到确认", force=True)
            self.stop_pending.clear()
        if self.incoming.qsize() > int(self.config.get("raw_input_queue_limit", 10000)):
            self._fail("采集失败：原始输入队列过载，当前会话已停止")
            return
        budget_ms = float(self.config.get("poll_budget_ms", 4.0))
        if not math.isfinite(budget_ms) or budget_ms <= 0:
            budget_ms = 4.0
        deadline = time.perf_counter() + budget_ms / 1000.0
        processed = 0
        while processed < 512 and time.perf_counter() < deadline:
            processed += 1
            try:
                source, data, arrival = self.incoming.get_nowait()
            except queue.Empty:
                break
            if source == "sent":
                try:
                    generation, endpoint_source, token = data
                except (TypeError, ValueError):
                    continue
                if generation != self.generation or endpoint_source not in self.endpoints:
                    continue
                self.sent_times[(endpoint_source, token)] = arrival
                continue
            parser = self.parsers.get(source)
            if parser is None:
                continue
            if arrival >= self.sync_started_at:
                self.received_bytes[source] = self.received_bytes.get(source, 0) + len(data)
            invalid_before = parser.invalid_frames
            for line in parser.feed(data):
                self._line(source, line, arrival)
            invalid_count = parser.invalid_frames - invalid_before
            if invalid_count > 0:
                self._record_ignored(source, f"<坏帧 hex:{bytes(data[:64]).hex(' ')}>", "格式错误", arrival,
                                     format_error=True)
                for _ in range(invalid_count - 1):
                    self._stat(source, "format_error")
                    self._stat(source, "ignored")
        if self.state == "syncing":
            self._sync_poll(now)
        elif self.state in {"starting_measurement", "starting_rotation"}:
            if now >= self.start_deadline:
                source = "measurement" if self.state == "starting_measurement" else "rotation"
                self._fail(f"启动失败：{self._endpoint_label(source)} OK确认超过 2 秒，请检查无线链路")
        elif self.state == "running":
            self._running_sync_poll(now)
            if self.state != "running":
                return
            poll_result = self.receiver.poll(now)
            for sequence, points, period in poll_result.formal_scans:
                self.emit("sync_sweep", (self.session, sequence, points, period), points[-1].timestamp)
            for sequence, points, period in poll_result.local_scans:
                if points:
                    self.emit("sync_local_sweep", (self.session, sequence, points, period), points[-1].timestamp)
            for diagnostic in poll_result.diagnostics:
                self._diagnostic(f"扫描诊断：{diagnostic}", timestamp=now)
            if self.builder.period_s is not None:
                self.emit("sync_period", (self.session, self.builder.period_s), now)
            self._publish_runtime_status(now)
            missing = []
            if now - self.last_arrival["measurement"] > 2:
                missing.append(self._endpoint_label("measurement"))
            if now - self.last_arrival["rotation"] > 10:
                missing.append(self._endpoint_label("rotation"))
            if len(self.pending) > 10000 or missing:
                detail = "、".join(missing) if missing else "观测队列"
                self._fail(f"采集失败：{detail}有效数据中断，请检查光电开关、CCD 和无线链路")

    def _running_sync_poll(self, now):
        if now - self.last_keepalive >= float(self.config.get("keepalive_interval_s", 5)):
            for source, endpoint in self.endpoints.items():
                if not endpoint.write_line("PING"):
                    self._fail(f"采集失败：{self._endpoint_label(source)}保活指令发送失败")
                    return
            self.last_keepalive = now
        for source, endpoint in self.endpoints.items():
            if now - self.clocks[source].observed_at > float(self.config.get("sync_max_age_s", 8)):
                self._fail(f"采集失败：{self._endpoint_label(source)}持续校时超时，停止采集与运动")
                return
            if source in self.outstanding:
                token, queued_at = self.outstanding[source]
                if now - queued_at < SYNC_RESPONSE_TIMEOUT_S:
                    continue
                self.sent_times.pop((source, token), None)
                del self.outstanding[source]
                self._stat(source, "timeout")
            if now < self.next_probe[source]:
                continue
            self.attempts[source] += 1
            token = f"{self.session}-{self.attempts[source]}"
            self.outstanding[source] = token, now
            self.next_probe[source] = now + float(self.config.get("sync_interval_s", 0.5))
            generation = self.generation
            if not endpoint.write_line(
                    f"SYNC {token}",
                    lambda t, g=generation, s=source, k=token: self.incoming.put(("sent", (g, s, k), t))):
                self._fail(f"持续校时失败：{self._endpoint_label(source)}串口发送失败")
                return
            self._stat(source, "sent")

    def _sync_failure_message(self, source):
        stats = self.sync_stats[source]
        received = self.received_bytes.get(source, 0)
        valid = len(self.exchanges[source])
        attempts = self.attempts[source]
        label = self._endpoint_label(source)
        if received == 0:
            return (f"校时失败：{label}未收到任何字节（已发送 {attempts} 次，"
                    f"有效回应 {valid}/{SYNC_REQUIRED_EXCHANGES}）；请检查设备供电、无线链路、"
                    "AUX/MD0/MD1 状态和实际固件")
        if stats["token_mismatch"]:
            return (f"校时失败：{label}收到 {received} 字节，但没有匹配本次 token 的回应（"
                    f"有效回应 {valid}/{SYNC_REQUIRED_EXCHANGES}，token 不匹配 {stats['token_mismatch']} 条）")
        if stats["format_error"] or stats["missing_send_time"] or stats["invalid_timestamp"]:
            return (f"校时失败：{label}收到 {received} 字节，但有效 SYNC 仅 "
                    f"{valid}/{SYNC_REQUIRED_EXCHANGES}（格式/时间字段无效 "
                    f"{stats['format_error'] + stats['missing_send_time'] + stats['invalid_timestamp']} 条）")
        if stats["ignored"]:
            return (f"校时失败：{label}收到 {received} 字节，但没有有效 SYNC 回应（"
                    f"有效回应 {valid}/{SYNC_REQUIRED_EXCHANGES}，无关回应 {stats['ignored']} 条）；"
                    "请检查固件是否支持当前同步协议")
        return (f"校时失败：{label}收到 {received} 字节，但没有有效 SYNC 回应（"
                f"有效回应 {valid}/{SYNC_REQUIRED_EXCHANGES}）；请检查固件同步协议")

    def _sync_poll(self, now):
        if now - self.sync_started_at > SYNC_TOTAL_TIMEOUT_S:
            pending = [self._endpoint_label(source) for source in self.endpoints
                       if len(self.exchanges[source]) < SYNC_REQUIRED_EXCHANGES]
            self._fail(f"校时失败：{','.join(pending)}超过 {SYNC_TOTAL_TIMEOUT_S:g} 秒未完成，请检查无线链路")
            return
        for source, endpoint in self.endpoints.items():
            if len(self.exchanges[source]) >= SYNC_REQUIRED_EXCHANGES:
                continue
            if source in self.outstanding:
                token, queued_at = self.outstanding[source]
                if now - queued_at < SYNC_RESPONSE_TIMEOUT_S:
                    continue
                self.sent_times.pop((source, token), None)
                del self.outstanding[source]
                self._stat(source, "timeout")
            if now < self.next_probe[source]:
                continue
            if self.attempts[source] >= SYNC_MAX_ATTEMPTS:
                self._fail(self._sync_failure_message(source))
                return
            self.attempts[source] += 1
            token = f"{self.session}-{self.attempts[source]}"
            self.outstanding[source] = token, now
            generation = self.generation
            if not endpoint.write_line(
                    f"SYNC {token}",
                    lambda t, g=generation, s=source, k=token: self.incoming.put(("sent", (g, s, k), t))):
                self._fail(f"校时失败：{self._endpoint_label(source)}串口发送失败")
                return
            self._stat(source, "sent")
        if all(len(items) >= SYNC_REQUIRED_EXCHANGES for items in self.exchanges.values()):
            self.clocks = {source: min(items, key=lambda item: item.uncertainty) for source, items in self.exchanges.items()}
            mode = str(self.config.get("measurement_mode", "fffe"))
            if mode not in {"fffe", "raw2"}:
                self._fail("同步采集支持 fffe 或 raw2 中心像素协议")
                return
            try:
                rate = float(self.config.get("hardware_sample_rate_hz", 100))
                exposure = int(self.config.get("actual_exposure_index", self.config.get("exposure_index", 5)))
                if not 1 <= rate <= 100 or not 0 <= exposure <= 13:
                    raise ValueError
            except (TypeError, ValueError, OverflowError):
                self._fail("采集参数无效：频率需为 1～100 Hz，曝光档位需为 0～13")
                return
            self.state = "starting_measurement"
            self.started_at = now
            self.start_deadline = now + 2
            self.last_arrival = {source: now for source in self.endpoints}
            self.last_communication = {source: now for source in self.endpoints}
            self.last_valid_range = now
            self.last_valid_rotation = now
            if not self.endpoints["measurement"].write_line(f"START {self.session} {rate:g} {exposure} {mode}"):
                self._fail("测距启动命令发送失败")
                return
            names = {"measurement": "测距", "rotation": "旋转"}
            errors = "，".join(f"{names[name]} ±{clock.uncertainty * 1000:.2f} ms" for name, clock in self.clocks.items())
            self._status(f"校时完成：{errors}；等待测距端启动确认")

    def _handle_error(self, source, parts, line, arrival):
        code = parts[1].upper() if len(parts) > 1 else ""
        startup_error = (self.state == "starting_measurement" and source == "measurement"
                         and code in SYNC_START_ERRORS)
        if code in SYNC_FATAL_ERRORS or startup_error:
            self._fail(f"{self._phase_label()}失败：{self._endpoint_label(source)}返回 {line}；当前协议未携带请求归属字段")
            return
        self._record_ignored(source, line, "无效或不相关错误回应", arrival,
                             format_error=parts != ["ERROR", "COMMAND"])

    def _handle_stop_confirmation(self, source, parts):
        expected = {"measurement": ["OK", "STOP"], "rotation": ["OK", "OFF"]}[source]
        if source in self.stop_pending and parts == expected:
            self.stop_status[source]["confirmed"] = True
            self.stop_pending.discard(source)
            self._diagnostic(f"{self._endpoint_label(source)}已收到停止确认", force=True)
            return True
        return False

    def _line(self, source, line, arrival):
        parts = line.split()
        if not parts:
            return
        if self.state == "stopped":
            self._handle_stop_confirmation(source, parts)
            return
        command = parts[0].upper()
        if command == "SYNC":
            if len(parts) != 4:
                self._record_ignored(source, line, "格式错误", arrival, format_error=True)
                return
            if self.state not in {"syncing", "running"}:
                self._record_ignored(source, line, "非当前阶段回应", arrival)
                return
            token = parts[1]
            if self.outstanding.get(source, (None,))[0] != token:
                self._record_ignored(source, line, "校时 token 不匹配", arrival, token_mismatch=True)
                return
            t1 = self.sent_times.pop((source, token), None)
            if t1 is None:
                self._stat(source, "missing_send_time")
                self._record_ignored(source, line, "缺失发送时间", arrival)
                return
            try:
                estimate = ClockEstimate.exchange(
                    t1, int(parts[2]), int(parts[3]), arrival,
                    float(self.config.get("clock_drift_bound_ppm", 500)),
                )
            except (ValueError, OverflowError):
                self._stat(source, "invalid_timestamp")
                self._record_ignored(source, line, "时间戳无效", arrival)
                return
            self.last_communication[source] = arrival
            if self.state == "running":
                try:
                    self.clocks[source] = self.clocks[source].updated(estimate)
                except ValueError:
                    self._fail(f"采集失败：{self._endpoint_label(source)}连续校时时钟跳变，请检查设备时钟与无线链路")
                    return
                self.next_probe[source] = arrival + float(self.config.get("sync_interval_s", 0.5))
            else:
                self.exchanges[source].append(estimate)
                self.next_probe[source] = arrival + 0.03
                self._status(self._sync_progress())
            self._stat(source, "success")
            del self.outstanding[source]
            return
        if command == "ERROR":
            self._handle_error(source, parts, line, arrival)
            return
        if command == "ERR" or command.startswith("ERR,"):
            self._record_ignored(source, line, "外来协议消息", arrival)
            return
        if self.state in {"starting_measurement", "starting_rotation"}:
            if arrival >= self.start_deadline:
                expected_source = "measurement" if self.state == "starting_measurement" else "rotation"
                self._fail(f"启动失败：{self._endpoint_label(expected_source)} OK确认超过 2 秒，请检查无线链路")
                return
            if source == "measurement" and parts == ["OK", "START", self.session] and self.state == "starting_measurement":
                self.state = "starting_rotation"
                self.start_deadline = arrival + 2
                if not self.endpoints["rotation"].write_line(f"ROT {self.session}"):
                    self._fail(f"启动失败：{self._endpoint_label('rotation')}串口发送失败")
                    return
                self._status("测距端已启动，等待旋转端启动确认")
                return
            if source == "rotation" and parts == ["OK", "ROT", self.session] and self.state == "starting_rotation":
                self.state = "running"
                self.last_arrival[source] = arrival
                self.last_valid_rotation = arrival
                self._status("两端已启动，等待稳定完整扫描")
                return
        if command in {"READY", "PONG", "STATUS", "SLIDE_REPORT"} or command == "OK":
            self._record_ignored(source, line, "无关或重复状态回应", arrival)
            return
        if command not in {"PIX", "TRIG"}:
            self._record_ignored(source, line, "格式错误", arrival, format_error=True)
            return
        if self.state not in {"running", "starting_rotation"} or len(parts) < 2:
            self._record_ignored(source, line, "无关数据帧", arrival)
            return
        if parts[1] != self.session:
            self._record_ignored(source, line, "旧 session 数据", arrival)
            return
        try:
            raw = parse_observation(source, line, self.session, self.calibration, self.config)
        except (ValueError, OverflowError) as exc:
            invalid_timestamp = "时间" in str(exc) or "采集区间" in str(exc)
            if invalid_timestamp:
                self._stat(source, "invalid_timestamp")
            self._record_ignored(source, line, "时间戳无效" if invalid_timestamp else "格式错误",
                                 arrival, format_error=not invalid_timestamp)
            return
        if raw is None:
            self._record_ignored(source, line, "格式错误", arrival, format_error=True)
            return
        if raw.sequence < 1:
            self._record_ignored(source, line, "格式错误", arrival, format_error=True)
            if source == "rotation":
                self.receiver.invalidate("零位帧序号无效")
            return
        previous = self.raw_progress.get(source)
        if previous is not None:
            seq, tick = previous
            if (raw.sequence > seq and raw.timestamp_us < tick or
                    raw.sequence < seq and raw.timestamp_us > tick):
                self._stat(source, "invalid_timestamp")
                if source == "rotation":
                    self.receiver.invalidate("零位设备时间异常，当前扫描失效，等待新的真实零位")
                    reason = "零位时间/序号异常，当前扫描丢弃"
                else:
                    reason = "测距时间/序号异常，单帧丢弃"
                self._record_ignored(source, line, reason, arrival)
                return
        if previous is None or raw.sequence > previous[0]:
            self.raw_progress[source] = raw.sequence, raw.timestamp_us
        packet = raw.normalize(self.clocks[source], arrival)
        is_new_progress = previous is None or raw.sequence > previous[0]
        accepted = self.receiver.feed(ReceivedObservation(packet, arrival))
        if packet.source == "range" and packet.status == "ok" and packet.distance is not None and is_new_progress:
            self.last_arrival["measurement"] = arrival
            self.last_valid_range = arrival
            self.emit("sync_range", (self.session, packet.pixel, packet.distance), arrival)
            self.emit("sync_observation", (self.session, packet), arrival)
        if not accepted:
            return
        if packet.source == "rotation":
            self.last_arrival["rotation"] = arrival
            self.last_valid_rotation = arrival
        self.last_sequence[source] = raw.sequence
