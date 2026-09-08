import math


class MotionSafetyGuard:
    def __init__(self, config):
        self.config = config
        self.command = None
        self.started_at = None
        self.last_observation = None

    def start(self, command, timestamp):
        self.command = command
        self.started_at = timestamp
        self.last_observation = timestamp

    def clear(self):
        self.command = None
        self.started_at = self.last_observation = None

    def observe(self, packet, estimate, now):
        if self.command is None or self.started_at is None:
            return None
        age = now - packet.device_timestamp
        maximum_age = float(self.config.get("safety_max_observation_age_s", 0.5))
        if (packet.source != "range" or packet.status not in {"ok", "over_range"}
                or packet.distance is None or not math.isfinite(packet.distance) or packet.distance <= 0
                or not math.isfinite(packet.uncertainty) or packet.uncertainty < 0
                or packet.device_timestamp < self.started_at or not 0 <= age <= maximum_age):
            return None
        self.last_observation = max(self.last_observation, packet.device_timestamp)
        radius = float(self.config.get("robot_radius_m", 0.15))
        clearance = float(self.config.get("safety_clearance_m", 0.12))
        speed = math.hypot(self.command.right_mps, self.command.forward_mps)
        limit = radius + clearance + speed * age
        if packet.distance > limit:
            return None
        if estimate is None or estimate[1] > math.radians(float(self.config.get("safety_max_angle_error_deg", 15))):
            return "近距离回波方位不确定，紧急停车"
        angle, error = estimate
        if abs(self.command.yaw_rps) > 1e-6 or speed < 1e-6:
            return "转向范围内存在近距离障碍，紧急停车"
        x, y = packet.distance * math.sin(angle), packet.distance * math.cos(angle)
        ux, uy = self.command.right_mps / speed, self.command.forward_mps / speed
        along = x * ux + y * uy
        lateral = abs(x * uy - y * ux)
        margin = packet.distance * math.sin(min(math.pi / 2, max(0, error)))
        if along + margin >= 0 and lateral <= radius + margin:
            return "运动方向出现近距离障碍，紧急停车"
        return None

    def poll(self, now):
        if (self.command is not None and self.last_observation is not None
                and now - self.last_observation > float(self.config.get("safety_blind_timeout_s", 0.75))):
            return "运动期间测距更新中断，紧急停车"
        return None
