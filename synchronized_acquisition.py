from __future__ import annotations

from dataclasses import dataclass
import math
import queue
import secrets
import time

from measurement_protocol import parse_observation
from radar_core import MotorLineParser
from scan_acquisition import (TimedSweepBuilder, EstimatedSweepBuilder, DistanceObservationReceiver,
                              ReceivedObservation)


@dataclass(frozen=True)
class ClockEstimate:
    offset: float
    uncertainty: float
    observed_at: float
    drift_ppm: float = 500.0

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.offset, self.uncertainty, self.observed_at, self.drift_ppm))
                or self.uncertainty < 0 or not 0 <= self.drift_ppm < 1000000):
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
                             measurement.observed_at, self.drift_ppm)



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
        self.pending = self.receiver.pending
        self.clocks = {}
        self.sent_times = {}
        self.last_report = ""

    def feed(self, source: str, data: bytes, arrival: float):
        self.incoming.put((source, data, arrival))

    def start(self, now: float | None = None):
        self.stop()
        now = time.perf_counter() if now is None else now
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
        self.last_report = ""
        for parser in self.parsers.values():
            parser.reset()
        self._status("双向校时中")

    def stop(self):
        self.state = "stopped"
        self.session = ""
        self.pending.clear()
        self.endpoints["measurement"].write_line("STOP")
        self.endpoints["measurement"].write_line("LASER 0")
        self.endpoints["rotation"].write_line("OFF")

    def begin_after(self, timestamp: float):
        self.receiver.reset(timestamp)

    def _status(self, message):
        if message != self.last_report:
            self.last_report = message
            self.emit("sync_status", message, time.perf_counter())

    def _fail(self, message):
        self.stop()
        self.emit("sync_error", message, time.perf_counter())

    def poll(self, now: float | None = None):
        now = time.perf_counter() if now is None else now
        for _ in range(4096):
            try:
                source, data, arrival = self.incoming.get_nowait()
            except queue.Empty:
                break
            if source == "sent":
                self.sent_times[data] = arrival
                continue
            for line in self.parsers[source].feed(data):
                self._line(source, line, arrival)
        if self.state == "syncing":
            self._sync_poll(now)
        elif self.state in {"starting_measurement", "starting_rotation"}:
            if now >= self.start_deadline:
                self._fail("设备启动确认超时，请检查两端固件和无线链路")
        elif self.state == "running":
            self._running_sync_poll(now)
            if self.state != "running":
                return
            for sequence, points, period in self.receiver.poll(now):
                self.emit("sync_sweep", (self.session, sequence, points, period), points[-1].timestamp)
            if self.builder.period_s is not None:
                self.emit("sync_period", (self.session, self.builder.period_s), now)
            self._status(self.builder.reason)
            if len(self.pending) > 10000 or now - self.last_arrival["measurement"] > 2 or now - self.last_arrival["rotation"] > 10:
                self._fail("测距或零位数据中断，请检查光电开关、CCD 和无线链路")

    def _running_sync_poll(self, now):
        if now - self.last_keepalive >= float(self.config.get("keepalive_interval_s", 5)):
            for endpoint in self.endpoints.values():
                if not endpoint.write_line("PING"):
                    self._fail("保活指令发送失败")
                    return
            self.last_keepalive = now
        for source, endpoint in self.endpoints.items():
            if now - self.clocks[source].observed_at > float(self.config.get("sync_max_age_s", 8)):
                self._fail("持续校时超时，停止采集与运动")
                return
            if source in self.outstanding:
                token, queued_at = self.outstanding[source]
                if now - queued_at < 1:
                    continue
                self.sent_times.pop((source, token), None)
                del self.outstanding[source]
            if now < self.next_probe[source]:
                continue
            self.attempts[source] += 1
            token = f"{self.session}-{self.attempts[source]}"
            self.outstanding[source] = token, now
            self.next_probe[source] = now + float(self.config.get("sync_interval_s", 0.5))
            if not endpoint.write_line(f"SYNC {token}", lambda t, s=source, k=token: self.incoming.put(("sent", (s, k), t))):
                self._fail("持续校时发送失败")
                return

    def _sync_poll(self, now):
        if now - self.sync_started_at > 20:
            self._fail("双向校时失败，请确认两端已烧录配套同步固件")
            return
        for source, endpoint in self.endpoints.items():
            if len(self.exchanges[source]) >= 8:
                continue
            if source in self.outstanding:
                token, queued_at = self.outstanding[source]
                if now - queued_at < 1:
                    continue
                self.sent_times.pop((source, token), None)
                del self.outstanding[source]
            if now < self.next_probe[source]:
                continue
            if self.attempts[source] >= 16:
                self._fail("校时响应丢失过多，请检查无线链路")
                return
            self.attempts[source] += 1
            token = f"{self.session}-{self.attempts[source]}"
            self.outstanding[source] = token, now
            if not endpoint.write_line(f"SYNC {token}", lambda t, s=source, k=token: self.incoming.put(("sent", (s, k), t))):
                self._fail("串口发送失败")
                return
        if all(len(items) >= 8 for items in self.exchanges.values()):
            self.clocks = {source: min(items, key=lambda item: item.uncertainty) for source, items in self.exchanges.items()}
            mode = str(self.config.get("measurement_mode", "fffe"))
            if mode not in {"fffe", "raw2"}:
                self._fail("同步采集支持 fffe 或 raw2 中心像素协议")
                return
            try:
                rate = float(self.config.get("hardware_sample_rate_hz", 20))
                exposure = int(self.config.get("exposure_index", 8))
                if not 1 <= rate <= 100 or not 0 <= exposure <= 13:
                    raise ValueError
            except (TypeError, ValueError, OverflowError):
                self._fail("采集参数无效：频率需为 1～100 Hz，曝光档位需为 0～13")
                return
            self.state = "starting_measurement"
            self.started_at = now
            self.start_deadline = now + 2
            self.last_arrival = {source: now for source in self.endpoints}
            if not self.endpoints["measurement"].write_line(f"START {self.session} {rate:g} {exposure} {mode}"):
                self._fail("测距启动命令发送失败")
                return
            names = {"measurement": "测距", "rotation": "旋转"}
            errors = "，".join(f"{names[name]} ±{clock.uncertainty * 1000:.2f} ms" for name, clock in self.clocks.items())
            self._status(f"校时完成：{errors}；等待测距端启动确认")

    def _line(self, source, line, arrival):
        parts = line.split()
        if not parts or self.state == "stopped":
            return
        if parts[0] == "SYNC" and len(parts) == 4 and self.state in {"syncing", "running"}:
            token = parts[1]
            if self.outstanding.get(source, (None,))[0] != token:
                return
            t1 = self.sent_times.pop((source, token), None)
            if t1 is None:
                return
            try:
                estimate = ClockEstimate.exchange(t1, int(parts[2]), int(parts[3]), arrival, float(self.config.get("clock_drift_bound_ppm", 500)))
            except (ValueError, OverflowError):
                return
            if self.state == "running":
                try:
                    self.clocks[source] = self.clocks[source].updated(estimate)
                except ValueError:
                    self._fail("连续校时时钟跳变，请检查设备时钟与无线链路")
                    return
                self.next_probe[source] = arrival + float(self.config.get("sync_interval_s", 0.5))
            else:
                self.exchanges[source].append(estimate)
                self.next_probe[source] = arrival + 0.03
            del self.outstanding[source]
            return
        if parts[0] == "ERROR":
            self._fail(f"{source}: {line}")
            return
        if parts[0] == "READY" and self.state in {"starting_measurement", "starting_rotation", "running"}:
            self._fail(f"{source} 设备重启，请重新开始扫描")
            return
        if self.state in {"starting_measurement", "starting_rotation"}:
            if arrival >= self.start_deadline:
                self._fail("设备启动确认超时，请检查两端固件和无线链路")
                return
            if source == "measurement" and parts == ["OK", "START", self.session] and self.state == "starting_measurement":
                self.state = "starting_rotation"
                self.start_deadline = arrival + 2
                if not self.endpoints["rotation"].write_line(f"ROT {self.session}"):
                    self._fail("旋转启动命令发送失败")
                    return
                self._status("测距端已启动，等待旋转端启动确认")
                return
            if source == "rotation" and parts == ["OK", "ROT", self.session] and self.state == "starting_rotation":
                self.state = "running"
                self.last_arrival[source] = arrival
                self._status("两端已启动，等待稳定完整扫描")
                return
        if self.state not in {"running", "starting_rotation"} or len(parts) < 2 or parts[1] != self.session:
            return
        try:
            raw = parse_observation(source, line, self.session, self.calibration, self.config)
        except (ValueError, OverflowError):
            if source == "rotation":
                self.receiver.invalidate("零位时间数据无效")
            return
        if raw is None:
            return
        if raw.sequence < 1:
            if source == "rotation":
                self.receiver.invalidate("零位帧序号无效")
            return
        previous = self.raw_progress.get(source)
        if previous is not None:
            seq, tick = previous
            if (raw.sequence > seq and raw.timestamp_us < tick or
                    raw.sequence < seq and raw.timestamp_us > tick):
                self._fail(f"{source} 设备时间倒退")
                return
        if previous is None or raw.sequence > previous[0]:
            self.raw_progress[source] = raw.sequence, raw.timestamp_us
        packet = raw.normalize(self.clocks[source], arrival)
        self.receiver.feed(ReceivedObservation(packet, arrival))
        if packet.source == "range" and (previous is None or raw.sequence > previous[0]):
            self.emit("sync_observation", (self.session, packet), arrival)
        self.last_arrival[source] = arrival
        self.last_sequence[source] = raw.sequence
