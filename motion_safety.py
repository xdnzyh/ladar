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
        if (packet.source != "range" or packet.status != "ok"
                or packet.distance is None or not math.isfinite(packet.distance) or packet.distance <= 0
                or not math.isfinite(packet.uncertainty) or packet.uncertainty < 0
                or packet.device_timestamp < self.started_at or not 0 <= age <= maximum_age):
            return None
        self.last_observation = max(self.last_observation, packet.device_timestamp)
        radius = float(self.config.get("robot_radius_m", 0.15))
        clearance = float(self.config.get("safety_clearance_m", 0.12))
        speed = self.speed_mps
        distance_control = (str(self.config.get("runtime_source", "")).lower() == "hardware"
                            and bool(self.config.get("chassis_distance_control", False)))
        speed = max(0.0, speed or 0.0)
        configured_stop_distance = self.config.get("safety_stop_distance_m")
        try:
            stop_distance = float(configured_stop_distance or 0.0)
        except (TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(stop_distance) or stop_distance < 0:
            return None
        lookahead = speed * age + stop_distance
        if distance_control:
            lookahead = max(lookahead, (self.direction_speed_mps or 0.0) * self.command.duration_s)
        limit = radius + clearance + lookahead
        sensor_offset = math.hypot(float(self.config.get("radar_offset_x_m", 0.0)),
                                   float(self.config.get("radar_offset_y_m", 0.0)))
        if (estimate is None or not all(math.isfinite(value) for value in estimate)
                or estimate[1] < 0
                or estimate[1] > math.radians(float(self.config.get("safety_max_angle_error_deg", 15)))):
            if packet.distance <= limit + sensor_offset:
                return "雷达检测到近距离障碍，紧急停车"
            return None
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
        if along + margin >= 0 and along <= limit + margin and lateral <= radius + margin:
            return "运动方向出现近距离障碍，紧急停车"
        return None

    def poll(self, now):
        return None
