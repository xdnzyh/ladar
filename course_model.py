from dataclasses import replace
import math


class BoardCourseModel:
    def __init__(self, board_width_m, width_boards, length_boards):
        values = (board_width_m, width_boards, length_boards)
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               or not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError("赛道板宽和板数必须为正数")
        if width_boards != 3 or int(length_boards) != length_boards:
            raise ValueError("双车位赛道横向为三块板，纵向板数必须为整数")
        self.board_width_m = float(board_width_m)
        self.width_m = board_width_m * width_boards
        self.length_m = board_width_m * length_boards
        self.reset()

    def reset(self):
        self.left_x = None
        self.rear_y = None
        self._pending = None

    @staticmethod
    def _lines(points, axis, minimum_span):
        result = []
        for coordinate in sorted({round(p[axis] / 0.02) * 0.02 for p in points}):
            group = [p for p in points if abs(p[axis] - coordinate) <= 0.035]
            if len(group) < 6:
                continue
            along = [p[1 - axis] for p in group]
            if max(along) - min(along) < minimum_span:
                continue
            cross_mean = sum(p[axis] for p in group) / len(group)
            along_mean = sum(along) / len(group)
            variance = sum((v - along_mean) ** 2 for v in along)
            slope = sum((p[axis] - cross_mean) * (p[1 - axis] - along_mean)
                        for p in group) / max(variance, 1e-9)
            rms = math.sqrt(sum((p[axis] - cross_mean - slope * (p[1 - axis] - along_mean)) ** 2
                                for p in group) / len(group))
            if rms <= 0.015 and abs(slope) <= math.tan(math.radians(6)):
                result.append((cross_mean, slope, len(group), along_mean))
        return result

    def fuse(self, pose, points, start_pose, sensor_offset, max_range_m):
        world = []
        for point in points:
            if (not point.has_echo(max_range_m) or point.quality < 0.55
                    or not math.isfinite(point.angle_rad) or not math.isfinite(point.distance_m)
                    or not math.isfinite(point.quality) or not 0 < point.distance_m <= max_range_m
                    or point.evidence_weight(0.02) < 0.55):
                continue
            sx, sy, yaw = sensor_offset
            bx = sx + point.x * math.cos(yaw) + point.y * math.sin(yaw)
            by = sy - point.x * math.sin(yaw) + point.y * math.cos(yaw)
            world.append(start_pose.world_to_local(*pose.local_to_world(bx, by)))
        if not world:
            return pose
        px, py = start_pose.world_to_local(pose.x, pose.y)
        lines = self._lines(world, 0, 0.30)
        pairs = [(left, right) for left in lines for right in lines
                 if left[0] < px < right[0]
                 and abs(right[0] - left[0] - self.width_m) <= 0.06
                 and abs(left[1] - right[1]) <= 0.035]
        if not pairs:
            self._pending = None
            return pose
        left, right = max(pairs, key=lambda pair: pair[0][2] + pair[1][2])
        slope = (left[1] + right[1]) / 2
        observed_left = ((left[0] + slope * (py - left[3]))
                         + (right[0] + slope * (py - right[3]) - self.width_m)) / 2
        if self.left_x is None:
            if math.hypot(px, py) > 0.20:
                return pose
            rear = [line for line in self._lines(world, 1, self.board_width_m)
                    if -max_range_m <= line[0] < -0.15]
            rear_y = min(rear, key=lambda line: line[0])[0] if rear else None
            candidate = (observed_left, rear_y)
            if (self._pending is not None and rear_y is not None and self._pending[1] is not None
                    and abs(observed_left - self._pending[0]) <= 0.03
                    and abs(rear_y - self._pending[1]) <= 0.03):
                self.left_x = (observed_left + self._pending[0]) / 2
                self.rear_y = (rear_y + self._pending[1]) / 2
            self._pending = candidate
            return pose
        correction = self.left_x - observed_left
        if abs(correction) > 0.08:
            return pose
        dx = max(-0.02, min(0.02, correction * 0.25))
        x, y = start_pose.local_to_world(px + dx, py)
        dyaw = max(-math.radians(1), min(math.radians(1), -math.atan(slope) * 0.25))
        return replace(pose, x=x, y=y, yaw=pose.yaw + dyaw)

    def parking_allowed(self, pose, start_pose):
        if self.left_x is None or self.rear_y is None:
            return False
        x, y = start_pose.world_to_local(pose.x, pose.y)
        return (self.rear_y + self.length_m - 2 * self.board_width_m <= y
                <= self.rear_y + self.length_m
                and self.left_x <= x <= self.left_x + self.width_m)

    def parking_centers(self):
        if self.left_x is None or self.rear_y is None:
            return ()
        return tuple((self.left_x + self.board_width_m * column,
                      self.rear_y + self.length_m - self.board_width_m / 2)
                     for column in (0.5, 2.5))
