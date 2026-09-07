from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
import queue
import secrets
import time

from radar_core import MotorLineParser, PolarPoint


@dataclass(frozen=True)
class ClockEstimate:
    offset: float
    uncertainty: float
    observed_at: float
    drift_ppm: float = 500.0

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
        error = self.uncertainty + abs(now - self.observed_at) * self.drift_ppm * 1e-6
        return device_us * 1e-6 - self.offset, error


class TimedSweepBuilder:
    def __init__(self, config: dict):
        self.config = config
        self.reset()

    def reset(self):
        self.anchor = None
        self.samples = []
        self.previous_period = None
        self.stable_periods = 0
        self.reason = "等待光电零位"
        self.period_s = None

    def sample(self, timestamp: float, uncertainty: float, pixel: int, distance: float):
        if self.anchor is not None and timestamp >= self.anchor[0]:
            self.samples.append((timestamp, uncertainty, pixel, distance))

    def trigger(self, timestamp: float, uncertainty: float, count: int) -> list[PolarPoint]:
        anchor, samples = self.anchor, self.samples
        self.anchor = (timestamp, uncertainty, count)
        self.samples = []
        if anchor is None:
            return []
        start, start_error, previous_count = anchor
        period = timestamp - start
        if count != previous_count + 1 or not 0.1 <= period <= 60:
            self.previous_period = None
            self.stable_periods = 0
            self.reason = "零位不连续，本圈丢弃"
            return []
        self.period_s = period
        previous_period, self.previous_period = self.previous_period, period
        if previous_period is None or abs(period / previous_period - 1) > float(self.config.get("period_tolerance", 0.05)):
            self.stable_periods = 0
        else:
            self.stable_periods += 1
        if self.stable_periods < 2:
            self.reason = "等待连续稳定转动"
            return []
        lower_period = period - start_error - uncertainty
        if lower_period <= 0:
            self.reason = "校时误差超过旋转周期"
            return []
        result = []
        direction = 1 if self.config.get("clockwise", True) else -1
        offset = math.radians(float(self.config.get("angle_offset_deg", 0)))
        error_limit = float(self.config.get("max_timing_position_error_m", 0.04))
        for sample_time, sample_error, pixel, distance in samples:
            if sample_time - sample_error < start + start_error or sample_time + sample_error >= timestamp - uncertainty:
                continue
            phase = (sample_time - start) / period
            angular_error = math.tau * (sample_error + start_error + uncertainty) / lower_period
            if distance * angular_error > error_limit:
                self.reason = f"时间配准误差超过 {error_limit * 100:g} cm，本圈丢弃"
                return []
            result.append(PolarPoint(distance, offset + direction * math.tau * phase, sample_time, pixel))
        if len(result) < 12:
            self.reason = "本圈有效测距不足 12 点"
            return []
        angles = sorted(point.angle_rad % math.tau for point in result)
        gaps = [b - a for a, b in zip(angles, angles[1:] + [angles[0] + math.tau])]
        if max(gaps) > math.radians(float(self.config.get("max_scan_gap_deg", 25))):
            self.reason = "扫描存在过大的角度空缺，本圈丢弃"
            return []
        self.reason = "完整扫描"
        return result


class SynchronizedAcquisition:
    def __init__(self, measurement, rotation, calibration, config: dict, emit):
        self.endpoints = {"measurement": measurement, "rotation": rotation}
        self.calibration = calibration
        self.config = config
        self.emit = emit
        self.incoming = queue.Queue()
        self.parsers = {name: MotorLineParser() for name in self.endpoints}
        self.builder = TimedSweepBuilder(config)
        self.state = "stopped"
        self.session = ""
        self.pending = []
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
        self.builder.reset()
        self.clocks = {}
        self.exchanges = {source: [] for source in self.endpoints}
        self.outstanding = {}
        self.sent_times = {}
        self.attempts = {source: 0 for source in self.endpoints}
        self.next_probe = {source: now + 0.1 for source in self.endpoints}
        self.latest = {source: -math.inf for source in self.endpoints}
        self.last_sequence = {}
        self.last_processed = -math.inf
        self.event_number = 0
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
            watermark = min(self.latest.values())
            while self.pending and self.pending[0][0] <= watermark:
                timestamp, _, kind, payload = heapq.heappop(self.pending)
                if timestamp < self.last_processed:
                    self.builder.reset()
                    self._status("收到迟到数据，本圈丢弃")
                    continue
                self.last_processed = timestamp
                if kind == "pixel":
                    uncertainty, pixel = payload
                    distance = self.calibration.distance(pixel) if 0 <= pixel <= 1499 else None
                    if distance is not None and float(self.config.get("min_range_m", 0.08)) <= distance <= float(self.config.get("max_range_m", 3)):
                        self.builder.sample(timestamp, uncertainty, pixel, distance)
                        self.emit("sync_range", (self.session, distance), timestamp)
                else:
                    uncertainty, count = payload
                    points = self.builder.trigger(timestamp, uncertainty, count)
                    if self.builder.period_s is not None:
                        self.emit("sync_period", (self.session, self.builder.period_s), timestamp)
                    self._status(self.builder.reason)
                    if points:
                        self.emit("sync_sweep", (self.session, count - 1, points, self.builder.period_s), timestamp)
            if now - self.started_at > float(self.config.get("sync_scan_duration_s", 30)):
                self.stop()
                self.emit("sync_expired", None, now)
            elif len(self.pending) > 10000 or now - self.last_arrival["measurement"] > 2 or now - self.last_arrival["rotation"] > 10:
                self._fail("测距或零位数据中断，请检查光电开关、CCD 和无线链路")

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
        if parts[0] == "SYNC" and len(parts) == 4 and self.state == "syncing":
            token = parts[1]
            if self.outstanding.get(source, (None,))[0] != token:
                return
            t1 = self.sent_times.pop((source, token), None)
            if t1 is None:
                return
            try:
                estimate = ClockEstimate.exchange(t1, int(parts[2]), int(parts[3]), arrival, float(self.config.get("clock_drift_bound_ppm", 500)))
            except ValueError:
                return
            self.exchanges[source].append(estimate)
            del self.outstanding[source]
            self.next_probe[source] = arrival + 0.03
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
            if source == "measurement" and parts[0] == "PIX" and len(parts) == 6:
                sequence, begin, end, pixel = map(int, parts[2:])
                if end < begin or begin < 0 or end - begin > 250000:
                    raise ValueError
                timestamp, uncertainty = self.clocks[source].map((begin + end) / 2, arrival)
                uncertainty += (end - begin) * 0.5e-6
                kind, payload = "pixel", (uncertainty, pixel)
            elif source == "rotation" and parts[0] == "TRIG" and len(parts) == 4:
                sequence, tick = map(int, parts[2:])
                if tick < 0:
                    raise ValueError
                timestamp, uncertainty = self.clocks[source].map(tick, arrival)
                uncertainty += float(self.config.get("irq_timestamp_uncertainty_ms", 2.0)) / 1000
                kind, payload = "trigger", (uncertainty, sequence)
            else:
                return
        except ValueError:
            self._fail(f"{source} 时间戳数据无效")
            return
        previous = self.last_sequence.get(source)
        if sequence < 1:
            self._fail(f"{source} 帧序号无效")
            return
        if previous is not None and sequence <= previous:
            return
        if previous is not None and sequence != previous + 1:
            self.pending.clear()
            self.builder.reset()
            self._status(f"{source} 数据丢帧，本圈丢弃")
        if timestamp < self.latest[source]:
            self._fail(f"{source} 设备时间倒退")
            return
        self.last_sequence[source] = sequence
        self.latest[source] = timestamp
        self.last_arrival[source] = arrival
        self.event_number += 1
        heapq.heappush(self.pending, (timestamp, self.event_number, kind, payload))
