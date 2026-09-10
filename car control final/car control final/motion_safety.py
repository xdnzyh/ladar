import math


class MotionSafetyGuard:
    def __init__(self, config):
        self.config = config
        self.command = None
        self.started_at = None
        self.last_observation = None
        self.speed_mps = None
        self.direction_speed_mps = None

    def start(self, command, timestamp):
        self.command = command
        self.started_at = timestamp
        self.last_observation = timestamp
        configured_speed = self.config.get("safety_speed_upper_bound_mps")
        try:
            configured_speed = float(configured_speed)
        except (TypeError, ValueError, OverflowError):
            configured_speed = None
        if configured_speed is not None and not math.isfinite(configured_speed):
            configured_speed = None
        self.direction_speed_mps = math.hypot(command.right_mps, command.forward_mps)
        if str(self.config.get("runtime_source", "")).lower() == "hardware":
            self.speed_mps = configured_speed if configured_speed is not None and configured_speed > 0 else None
        else:
            self.speed_mps = self.direction_speed_mps

    def clear(self):
        self.command = None
        self.started_at = self.last_observation = None
        self.speed_mps = None
        self.direction_speed_mps = None

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
        speed = self.speed_mps
        if speed is None:
            return "底盘实际速度上界未标定，停止自动运动"
        speed = max(0.0, speed)
        configured_stop_distance = self.config.get("safety_stop_distance_m")
        if configured_stop_distance is None:
            if str(self.config.get("runtime_source", "")).lower() == "hardware":
                return "底盘制动距离未标定，停止自动运动"
            configured_stop_distance = 0.0
        try:
            stop_distance = float(configured_stop_distance)
        except (TypeError, ValueError, OverflowError):
            return "底盘制动距离配置无效，停止自动运动"
        if not math.isfinite(stop_distance) or stop_distance < 0:
            return "底盘制动距离配置无效，停止自动运动"
        if estimate is None or estimate[1] > math.radians(float(self.config.get("safety_max_angle_error_deg", 15))):
            return "近距离回波方位不确定，紧急停车"
        angle, error = estimate
        sensor_x = packet.distance * math.sin(angle)
        sensor_y = packet.distance * math.cos(angle)
        sensor_yaw = math.radians(float(self.config.get("radar_offset_yaw_deg", 0.0)))
        cosine = math.cos(sensor_yaw)
        sine = math.sin(sensor_yaw)
        x = sensor_x * cosine + sensor_y * sine + float(self.config.get("radar_offset_x_m", 0.0))
        y = -sensor_x * sine + sensor_y * cosine + float(self.config.get("radar_offset_y_m", 0.0))
        point_distance = math.hypot(x, y)
        margin = packet.distance * math.sin(min(math.pi / 2, max(0, error)))
        limit = radius + clearance + speed * age + stop_distance
        if point_distance > limit + margin:
            return None
        direction_speed = self.direction_speed_mps or 0.0
        if abs(self.command.yaw_rps) > 1e-6 or direction_speed < 1e-6:
            if point_distance <= limit + margin:
                return "转向范围内存在近距离障碍，紧急停车"
            return None
        ux = self.command.right_mps / direction_speed
        uy = self.command.forward_mps / direction_speed
        along = x * ux + y * uy
        lateral = abs(x * uy - y * ux)
        if along + margin >= 0 and along <= speed * age + radius + clearance + stop_distance + margin and lateral <= radius + margin:
            return "运动方向出现近距离障碍，紧急停车"
        return None

    def poll(self, now):
        if (self.command is not None and self.last_observation is not None
                and now - self.last_observation > float(self.config.get("safety_blind_timeout_s", 0.75))):
            return "运动期间测距更新中断，紧急停车"
        return None
