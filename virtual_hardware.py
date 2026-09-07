from __future__ import annotations

from dataclasses import dataclass, replace
import heapq
import math
import random
from collections import deque

from navigation_core import HiddenWorld, VelocityCommand, mecanum_mix, wrap_angle
from synchronized_acquisition import TimedSweepBuilder, EstimatedSweepBuilder


@dataclass(frozen=True)
class SimulationParameters:
    range_bias_m: float = 0.0
    sigma0_m: float = 0.0
    sigma1: float = 0.0
    sigma2: float = 0.0
    drift_m: float = 0.0
    quantization_m: float = 0.0
    outlier_probability: float = 0.0
    failure_probability: float = 0.0
    bad_interval_s: float = 0.0
    sample_jitter: float = 0.0
    rotation_ripple: float = 0.0
    spinup_s: float = 0.0
    zero_jitter_s: float = 0.0
    zero_offset_rad: float = 0.0
    missing_zero_probability: float = 0.0
    double_zero_probability: float = 0.0
    range_delay_s: float = 0.0
    rotation_delay_s: float = 0.0
    delay_jitter_s: float = 0.0
    drop_probability: float = 0.0
    duplicate_probability: float = 0.0
    burst_s: float = 0.0
    tail_probability: float = 0.0
    tail_delay_s: float = 0.0
    wheel_gains: tuple = (1.0, 1.0, 1.0, 1.0)
    wheel_noise: float = 0.0
    lateral_slip: float = 0.0
    lateral_yaw: float = 0.0
    deadzone: float = 0.0
    start_delay_s: float = 0.0
    response_s: float = 0.0
    stop_response_s: float = 0.0
    battery_decay: float = 0.0
    lidar_to_body_x: float = 0.0
    lidar_to_body_y: float = 0.0
    lidar_to_body_yaw: float = 0.0
    eccentricity_m: float = 0.0
    wobble_rad: float = 0.0


PRESETS = {
    "IDEAL": SimulationParameters(),
    "NOMINAL": SimulationParameters(
        range_bias_m=0.002, sigma0_m=0.001, sigma1=0.001, sigma2=0.0003,
        drift_m=0.003, quantization_m=0.001, outlier_probability=0.002,
        failure_probability=0.002, sample_jitter=0.04, rotation_ripple=0.025,
        spinup_s=0.4, zero_jitter_s=0.0002, range_delay_s=0.025,
        rotation_delay_s=0.012, delay_jitter_s=0.008, drop_probability=0.001,
        duplicate_probability=0.003, burst_s=0.05,
        wheel_gains=(0.98, 1.02, 0.97, 1.0), wheel_noise=0.005,
        lateral_slip=0.04, lateral_yaw=0.01, deadzone=0.008,
        start_delay_s=0.025, response_s=0.035, stop_response_s=0.06,
        battery_decay=0.00001, eccentricity_m=0.001, wobble_rad=0.001),
    "STRESS": SimulationParameters(
        range_bias_m=0.008, sigma0_m=0.004, sigma1=0.004, sigma2=0.001,
        drift_m=0.012, quantization_m=0.002, outlier_probability=0.015,
        failure_probability=0.015, bad_interval_s=0.4, sample_jitter=0.20,
        rotation_ripple=0.16, spinup_s=1.0, zero_jitter_s=0.001,
        missing_zero_probability=0.025, double_zero_probability=0.02,
        range_delay_s=0.06, rotation_delay_s=0.015, delay_jitter_s=0.04,
        drop_probability=0.02, duplicate_probability=0.04, burst_s=0.20,
        tail_probability=0.025, tail_delay_s=0.7,
        wheel_gains=(0.94, 1.06, 0.91, 1.02), wheel_noise=0.025,
        lateral_slip=0.18, lateral_yaw=0.06, deadzone=0.02,
        start_delay_s=0.06, response_s=0.10, stop_response_s=0.45,
        battery_decay=0.00005, eccentricity_m=0.004, wobble_rad=0.008),
}


def simulation_parameters(config: dict) -> SimulationParameters:
    mode = str(config.get("simulation_profile", "NOMINAL")).upper()
    if mode not in PRESETS:
        raise ValueError(f"未知仿真模式：{mode}")
    parameters = replace(PRESETS[mode], **config.get("simulation_parameters", {}))
    for name, value in vars(parameters).items():
        values = value if name == "wheel_gains" else (value,)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError(f"无效仿真参数：{name}")
        if name.endswith("probability") and not 0 <= value <= 1:
            raise ValueError(f"概率必须在 0 到 1 之间：{name}")
        if name.endswith("_s") and value < 0:
            raise ValueError(f"时间参数不能为负：{name}")
    if len(parameters.wheel_gains) != 4 or min(parameters.wheel_gains) <= 0:
        raise ValueError("wheel_gains 必须包含四个正数")
    if not 0 <= parameters.sample_jitter < 1 or not 0 <= parameters.lateral_slip < 1:
        raise ValueError("采样抖动和横移滑移必须在 0 到 1 之间")
    return parameters


@dataclass(frozen=True)
class HardwareObservation:
    source: str
    sequence: int
    device_timestamp: float
    distance: float | None = None
    status: str = "ok"


@dataclass(frozen=True)
class ReceivedObservation:
    observation: HardwareObservation
    arrival_time: float


class VirtualCommunicationLink:
    def __init__(self, parameters: SimulationParameters, seed: int):
        self.p = parameters
        self.random = random.Random(seed)
        self.pending = []
        self.counter = 0
        self.dropped = 0
        self.duplicates = 0

    def send(self, packet: HardwareObservation, measurement_time: float):
        if self.random.random() < self.p.drop_probability:
            self.dropped += 1
            return
        delay = self.p.range_delay_s if packet.source == "range" else self.p.rotation_delay_s
        arrival = measurement_time + max(0, delay + self.random.uniform(-self.p.delay_jitter_s, self.p.delay_jitter_s))
        if self.p.burst_s:
            arrival = math.ceil(arrival / self.p.burst_s) * self.p.burst_s
        if self.random.random() < self.p.tail_probability:
            arrival += self.p.tail_delay_s
        copies = 2 if self.random.random() < self.p.duplicate_probability else 1
        self.duplicates += copies - 1
        for index in range(copies):
            self.counter += 1
            received = ReceivedObservation(packet, arrival + index * 0.001)
            heapq.heappush(self.pending, (received.arrival_time, self.counter, received))

    def receive(self, now: float):
        while self.pending and self.pending[0][0] <= now:
            yield heapq.heappop(self.pending)[2]


class DistanceObservationReceiver:
    def __init__(self, config: dict):
        self.config = config
        self.builder = EstimatedSweepBuilder(config) if config.get("arbitrary_phase_scans", False) else TimedSweepBuilder(config)
        self.reorder_s = float(config.get("simulation_reorder_s", 0.3))
        if not math.isfinite(self.reorder_s) or self.reorder_s < 0:
            raise ValueError("重排等待时间必须是非负有限数")
        self.pending = []
        self.seen = {}
        self.counter = 0
        self.watermark = -math.inf
        self.accepted = self.discarded = self.late = self.duplicates = 0
        self.warmup = 0
        self._preview = deque(maxlen=512)
        self.preview_points = ()

    def reset(self, now: float):
        if isinstance(self.builder, EstimatedSweepBuilder):
            self.builder.begin_after(now)
            return
        previous_period = self.builder.previous_period
        stable_periods = self.builder.stable_periods
        self.builder.reset()
        self.builder.previous_period = previous_period
        self.builder.stable_periods = stable_periods
        self.pending.clear()
        self.seen.clear()
        self.watermark = now

    def feed(self, received: ReceivedObservation):
        packet = received.observation
        timestamp = packet.device_timestamp
        key = packet.source, packet.sequence
        if key in self.seen:
            self.duplicates += 1
            return
        if not math.isfinite(timestamp) or timestamp <= self.watermark:
            self.late += 1
            return
        self.seen[key] = timestamp
        self.counter += 1
        heapq.heappush(self.pending, (timestamp, 0 if packet.source == "rotation" else 1, self.counter, packet))

    def poll(self, now: float):
        cutoff = max(self.watermark, now - self.reorder_s)
        results = []
        def finish_window(timestamp):
            if isinstance(self.builder, EstimatedSweepBuilder):
                result = self.builder.poll(timestamp)
                if result is not None:
                    if result[1]:
                        self.accepted += 1
                        results.append(result)
                    else:
                        self.discarded += 1
        while self.pending and self.pending[0][0] <= cutoff:
            timestamp, _, _, packet = heapq.heappop(self.pending)
            finish_window(timestamp)
            if packet.source == "range":
                distance = packet.distance
                if packet.status in {"ok", "over_range"} and distance is not None and math.isfinite(distance):
                    if float(self.config.get("min_range_m", 0.08)) <= distance <= float(self.config.get("max_range_m", 3.0)):
                        self.builder.sample(timestamp, 0.0, 0, distance)
                        if (isinstance(self.builder, EstimatedSweepBuilder) and self.builder.anchor is not None
                                and self.builder.stable_periods >= 2 and self.builder.period_s is not None
                                and 0 <= timestamp - self.builder.anchor[0] <= self.builder.period_s * 1.5):
                            direction = 1 if self.config.get("clockwise", True) else -1
                            angle = math.radians(float(self.config.get("angle_offset_deg", 0))) + direction * math.tau * (timestamp - self.builder.anchor[0]) / self.builder.period_s
                            self._preview.append((timestamp, angle % math.tau, distance))
            elif packet.source == "rotation":
                if isinstance(self.builder, EstimatedSweepBuilder):
                    before = self.builder.invalidated
                    self.builder.trigger(timestamp, 0.0, packet.sequence)
                    self.discarded += self.builder.invalidated - before
                    if self.builder.stable_periods < 2:
                        self.warmup += 1
                    continue
                had_anchor = self.builder.anchor is not None
                points = self.builder.trigger(timestamp, 0.0, packet.sequence)
                if points:
                    self.accepted += 1
                    results.append((packet.sequence, points, self.builder.period_s))
                elif had_anchor:
                    if self.builder.reason == "等待连续稳定转动":
                        self.warmup += 1
                    else:
                        self.discarded += 1
        finish_window(cutoff)
        period = self.builder.period_s or float(self.config.get("radar_period_s", 1.5))
        while self._preview and self._preview[0][0] < cutoff - period:
            self._preview.popleft()
        self.preview_points = tuple(self._preview)
        self.watermark = cutoff
        self.seen = {key: timestamp for key, timestamp in self.seen.items() if timestamp > cutoff}
        return results


class VirtualRotationHardware:
    def __init__(self, parameters: SimulationParameters, period: float, seed: int):
        self.p = parameters
        self.period = period
        self.random = random.Random(seed)
        self.angle = 0.0
        self.sequence = 0
        self.next_zero = math.tau + parameters.zero_offset_rad

    def step(self, now: float, dt: float):
        p = self.p
        acceleration = min(1.0, now / p.spinup_s) if p.spinup_s else 1.0
        ripple = p.rotation_ripple * (math.sin(self.angle) + 0.35 * math.sin(2 * self.angle + 0.7))
        speed = math.tau / self.period * acceleration * max(0.2, 1 + ripple - p.battery_decay * now)
        previous = self.angle
        self.angle += speed * dt
        packets = []
        while self.angle >= self.next_zero:
            crossing = now - dt + dt * (self.next_zero - previous) / max(self.angle - previous, 1e-12)
            self.sequence += 1
            if self.random.random() >= p.missing_zero_probability:
                stamp = crossing + self.random.uniform(-p.zero_jitter_s, p.zero_jitter_s)
                packets.append(HardwareObservation("rotation", self.sequence, stamp))
                if self.random.random() < p.double_zero_probability:
                    self.sequence += 1
                    packets.append(HardwareObservation("rotation", self.sequence, stamp + 0.002))
            self.next_zero += math.tau
        return packets


class VirtualRangeSensor:
    def __init__(self, parameters: SimulationParameters, rate: float, maximum: float, seed: int):
        self.p = parameters
        self.rate = rate
        self.maximum = maximum
        self.random = random.Random(seed)
        self.next_sample = 0.0
        self.sequence = 0

    def sample(self, now: float, world: HiddenWorld, angle: float):
        p = self.p
        self.sequence += 1
        self.next_sample = now + (1 + self.random.uniform(-p.sample_jitter, p.sample_jitter)) / self.rate
        ox = p.lidar_to_body_x + p.eccentricity_m * math.sin(angle)
        oy = p.lidar_to_body_y + p.eccentricity_m * math.cos(angle)
        origin = world.pose.local_to_world(ox, oy)
        distance = world.ray_distance(angle + p.lidar_to_body_yaw + p.wobble_rad * math.sin(2 * angle), self.maximum, origin)
        status = "over_range" if distance >= self.maximum else "ok"
        bad_interval = p.bad_interval_s > 0 and 3.0 <= now % 9.0 < 3.0 + p.bad_interval_s
        if self.random.random() < p.failure_probability or (bad_interval and self.sequence % 3 == 0):
            return HardwareObservation("range", self.sequence, now, None, "no_return")
        if status == "ok":
            sigma = max(0, p.sigma0_m + p.sigma1 * distance + p.sigma2 * distance * distance)
            distance += p.range_bias_m + p.drift_m * math.sin(now / 25) + self.random.gauss(0, sigma)
            if bad_interval or self.random.random() < p.outlier_probability:
                distance += 0.18
            if p.quantization_m > 0:
                distance = round(distance / p.quantization_m) * p.quantization_m
            distance = min(self.maximum, distance)
        return HardwareObservation("range", self.sequence, now, distance, status)


class VirtualChassis:
    def __init__(self, world: HiddenWorld, parameters: SimulationParameters, seed: int):
        self._world = world
        self.p = parameters
        self.random = random.Random(seed)
        self.velocity = [0.0] * 4
        self.target = [0.0] * 4
        self.start_at = self.stop_at = 0.0
        self.collisions = 0
        self.in_contact = False

    def execute(self, command: VelocityCommand, now: float):
        self.start_at = now + self.p.start_delay_s
        self.stop_at = now + max(0, command.duration_s)
        self.target = [v * gain * (1 + self.random.gauss(0, self.p.wheel_noise))
                       for v, gain in zip(mecanum_mix(command), self.p.wheel_gains)]

    def stop(self, now: float):
        self.stop_at = now
        self.target = [0.0] * 4

    def step(self, now: float, dt: float):
        active = self.start_at <= now < self.stop_at
        tau = self.p.response_s if active else self.p.stop_response_s
        alpha = 1 - math.exp(-dt / tau) if tau else 1.0
        battery = max(0.5, 1 - self.p.battery_decay * now)
        for index in range(4):
            target = self.target[index] * battery if active else 0.0
            if abs(target) < self.p.deadzone:
                target = 0.0
            self.velocity[index] += alpha * (target - self.velocity[index])
        fl, fr, rl, rr = self.velocity
        forward = (fl + fr + rl + rr) / 4
        right = (-fl + fr + rl - rr) / 4 * (1 - self.p.lateral_slip)
        yaw = (-fl + fr - rl + rr) / 4 + right * self.p.lateral_yaw
        world = self._world
        x, y = world.pose.local_to_world(right * dt, forward * dt)
        contact = world._occupied(x, y, world.robot_radius_m)
        if contact and math.hypot(forward, right) > 1e-5:
            if not self.in_contact:
                self.collisions += 1
        else:
            world.pose.x, world.pose.y = x, y
        self.in_contact = contact
        world.pose.yaw = wrap_angle(world.pose.yaw + yaw * dt)


class HardwareSimulation:
    def __init__(self, world: HiddenWorld, config: dict):
        self._world = world
        self.config = config
        self.parameters = simulation_parameters(config)
        seed = int(config.get("simulation_seed", 20260907))
        period = float(config.get("radar_period_s", 1.5))
        rate = float(config.get("simulation_sample_rate_hz", 20))
        maximum = float(config.get("max_range_m", 3))
        if not all(math.isfinite(v) and v > 0 for v in (period, rate, maximum)):
            raise ValueError("周期、采样率与量程必须是正有限数")
        self.rotation = VirtualRotationHardware(self.parameters, period, seed + 1)
        self.sensor = VirtualRangeSensor(self.parameters, rate, maximum, seed + 2)
        self.chassis = VirtualChassis(world, self.parameters, seed + 3)
        self.link = VirtualCommunicationLink(self.parameters, seed + 4)
        self.receiver = DistanceObservationReceiver({**config, "arbitrary_phase_scans": True})
        self.time = 0.0
        self.resume_at = 0.0
        self.was_moving = False

    def execute(self, command: VelocityCommand):
        self.chassis.execute(command, self.time)
        self.resume_at = self.time + command.duration_s + float(self.config.get("simulation_settle_s", 0.2))
        self.was_moving = True
        self.receiver.reset(self.resume_at)

    def stop(self):
        self.chassis.stop(self.time)
        self.receiver.reset(self.time + float(self.config.get("simulation_settle_s", 0.2)))

    def advance(self, duration: float):
        end = self.time + duration
        results = []
        while self.time < end - 1e-10:
            dt = min(0.002, end - self.time)
            if self.sensor.next_sample > self.time + 1e-10:
                dt = min(dt, self.sensor.next_sample - self.time)
            self.time += dt
            self.chassis.step(self.time, dt)
            for packet in self.rotation.step(self.time, dt):
                self.link.send(packet, self.time)
            if self.time >= self.sensor.next_sample - 1e-10:
                direction = 1 if self.config.get("clockwise", True) else -1
                packet = self.sensor.sample(self.time, self._world, direction * self.rotation.angle)
                self.link.send(packet, self.time)
            moving = self.time < self.resume_at
            if self.was_moving and not moving:
                self.was_moving = False
            for received in self.link.receive(self.time):
                self.receiver.feed(received)
            results.extend(self.receiver.poll(self.time))
        return results

    def diagnostics(self):
        receiver = self.receiver
        total = receiver.accepted + receiver.discarded
        return {"simulation_time_s": self.time, "accepted_scans": receiver.accepted,
                "warmup_scans": receiver.warmup,
                "discarded_scans": receiver.discarded,
                "scan_discard_rate": receiver.discarded / total if total else 0.0,
                "late_packets": receiver.late, "duplicate_packets": receiver.duplicates,
                "dropped_packets": self.link.dropped, "collisions": self.chassis.collisions}
