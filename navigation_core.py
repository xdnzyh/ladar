from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
import json
import math
from pathlib import Path
import random
from typing import Iterable, Mapping, Sequence


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % math.tau - math.pi


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def local_to_world(self, local_x: float, local_y: float) -> tuple[float, float]:
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return (
            self.x + local_x * cosine + local_y * sine,
            self.y - local_x * sine + local_y * cosine,
        )

    def world_to_local(self, world_x: float, world_y: float) -> tuple[float, float]:
        delta_x = world_x - self.x
        delta_y = world_y - self.y
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return (
            delta_x * cosine - delta_y * sine,
            delta_x * sine + delta_y * cosine,
        )


@dataclass(frozen=True)
class ScanPoint:
    angle_rad: float
    distance_m: float
    quality: float = 1.0

    @property
    def x(self) -> float:
        return self.distance_m * math.sin(self.angle_rad)

    @property
    def y(self) -> float:
        return self.distance_m * math.cos(self.angle_rad)


@dataclass(frozen=True)
class VelocityCommand:
    forward_mps: float = 0.0
    right_mps: float = 0.0
    yaw_rps: float = 0.0
    duration_s: float = 0.0

    @property
    def stopped(self) -> bool:
        return (
            abs(self.forward_mps) < 1e-9
            and abs(self.right_mps) < 1e-9
            and abs(self.yaw_rps) < 1e-9
        )


class OccupancyGrid:
    UNKNOWN = 0
    FREE = -1
    OCCUPIED = 1
    HIT_LOG_ODDS = 2.0
    FREE_LOG_ODDS = 1.0
    MIN_QUALITY = 0.25
    MIN_SCAN_CONFIDENCE = 0.55
    ENDPOINT_SIGMA_CELLS = 0.75

    def __init__(self, width: int = 180, height: int = 180, resolution_m: float = 0.04) -> None:
        if width <= 0 or height <= 0 or resolution_m <= 0:
            raise ValueError("地图尺寸和分辨率必须为正数")
        self.width = int(width)
        self.height = int(height)
        self.resolution_m = float(resolution_m)
        self.origin_col = self.width // 2
        self.origin_row = round(self.height * 0.84)
        self.log_odds = [0] * (self.width * self.height)
        self.update_count = 0
        self._revision = 0
        self._field_cache = {}

    def clear(self) -> None:
        self.log_odds[:] = [0] * len(self.log_odds)
        self.update_count = 0
        self._revision += 1
        self._field_cache.clear()

    def _index(self, col: int, row: int) -> int:
        return row * self.width + col

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = self.origin_col + round(x / self.resolution_m)
        row = self.origin_row - round(y / self.resolution_m)
        return col, row

    def cell_to_world(self, col: int, row: int) -> tuple[float, float]:
        return (
            (col - self.origin_col) * self.resolution_m,
            (self.origin_row - row) * self.resolution_m,
        )

    def value(self, col: int, row: int) -> float:
        if not self.in_bounds(col, row):
            return 20
        return self.log_odds[self._index(col, row)]

    def state(self, col: int, row: int) -> int:
        value = self.value(col, row)
        if value >= 4:
            return self.OCCUPIED
        if value <= -2:
            return self.FREE
        return self.UNKNOWN

    def _add(self, col: int, row: int, amount: float) -> None:
        if not self.in_bounds(col, row):
            return
        index = self._index(col, row)
        self.log_odds[index] = max(-20, min(20, self.log_odds[index] + amount))
        self._revision += 1

    @staticmethod
    def _line_cells(start: tuple[int, int], end: tuple[int, int]) -> list[tuple[int, int]]:
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx + dy
        result: list[tuple[int, int]] = []
        while True:
            result.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            twice = 2 * error
            if twice >= dy:
                error += dy
                x0 += sx
            if twice <= dx:
                error += dx
                y0 += sy
        return result

    def update_scan(
        self,
        pose: Pose2D,
        points: Sequence[ScanPoint],
        max_range_m: float,
        min_range_m: float = 0.08,
        scan_confidence: float = 1.0,
    ) -> None:
        if not math.isfinite(scan_confidence) or scan_confidence < self.MIN_SCAN_CONFIDENCE:
            return
        confidence = min(1.0, scan_confidence)
        start = self.world_to_cell(pose.x, pose.y)
        hits = {}
        frees = {}
        for point in points:
            if (not math.isfinite(point.angle_rad) or not math.isfinite(point.quality)
                    or point.quality < self.MIN_QUALITY
                    or not min_range_m <= point.distance_m <= max_range_m):
                continue
            weight = min(1.0, point.quality) * confidence / (1 + 0.1 * (point.distance_m / max_range_m) ** 2)
            endpoint = pose.local_to_world(point.x, point.y)
            end = self.world_to_cell(*endpoint)
            cells = self._line_cells(start, end)
            has_hit = point.distance_m < max_range_m - 1e-6
            for cell in cells[:-1] if has_hit else cells:
                frees[cell] = max(frees.get(cell, 0.0), weight)
            if has_hit:
                sigma = self.resolution_m * self.ENDPOINT_SIGMA_CELLS
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        cell = end[0] + dx, end[1] + dy
                        cx, cy = self.cell_to_world(*cell)
                        squared = (cx - endpoint[0]) ** 2 + (cy - endpoint[1]) ** 2
                        spatial_weight = math.exp(-squared / (2 * sigma ** 2))
                        hits[cell] = max(hits.get(cell, 0.0), weight * spatial_weight)
        if not hits and not frees:
            return
        for cell, weight in frees.items():
            if cell not in hits:
                self._add(*cell, -self.FREE_LOG_ODDS * weight)
        for cell, weight in hits.items():
            self._add(*cell, self.HIT_LOG_ODDS * weight)
        self.update_count += 1

    @staticmethod
    def _squared_distance_transform(values: Sequence[float]) -> list[float]:
        sites = [index for index, value in enumerate(values) if math.isfinite(value)]
        if not sites:
            return [math.inf] * len(values)
        vertices = [sites[0]]
        boundaries = [-math.inf, math.inf]
        for site in sites[1:]:
            previous = vertices[-1]
            crossing = ((values[site] + site * site) - (values[previous] + previous * previous)) / (2 * (site - previous))
            while crossing <= boundaries[-2]:
                vertices.pop()
                boundaries.pop(-2)
                previous = vertices[-1]
                crossing = ((values[site] + site * site) - (values[previous] + previous * previous)) / (2 * (site - previous))
            vertices.append(site)
            boundaries.insert(-1, crossing)
        result = []
        vertex = 0
        for index in range(len(values)):
            while boundaries[vertex + 1] < index:
                vertex += 1
            source = vertices[vertex]
            result.append((index - source) ** 2 + values[source])
        return result

    def likelihood_field(self, sigma_m: float, minimum_evidence: float = 4.0) -> list[float]:
        if not math.isfinite(sigma_m) or sigma_m <= 0 or not math.isfinite(minimum_evidence) or minimum_evidence <= 0:
            raise ValueError("距离场尺度和证据阈值必须为正有限数")
        key = (sigma_m, minimum_evidence)
        cached = self._field_cache.get(key)
        if cached is not None and cached[0] == self._revision:
            return cached[1]
        distance_key = ("distance", minimum_evidence)
        distances = self._field_cache.get(distance_key)
        if distances is None or distances[0] != self._revision:
            horizontal = []
            for row in range(self.height):
                horizontal.extend(self._squared_distance_transform([
                    0.0 if self.log_odds[row * self.width + col] >= minimum_evidence else math.inf
                    for col in range(self.width)]))
            squared = [math.inf] * len(self.log_odds)
            for col in range(self.width):
                column = self._squared_distance_transform([horizontal[row * self.width + col] for row in range(self.height)])
                for row, value in enumerate(column):
                    squared[row * self.width + col] = value
            self._field_cache.clear()
            self._field_cache[distance_key] = (self._revision, squared)
        else:
            squared = distances[1]
        coefficient = self.resolution_m ** 2 / (2 * sigma_m ** 2)
        field = [math.exp(-distance * coefficient) for distance in squared]
        self._field_cache[key] = (self._revision, field)
        return field

    def known_ratio(self) -> float:
        known = sum(1 for value in self.log_odds if abs(value) >= 2)
        return known / len(self.log_odds)

    def known_area_m2(self) -> float:
        known = sum(1 for value in self.log_odds if abs(value) >= 2)
        return known * self.resolution_m * self.resolution_m

    def occupied_cells(self) -> list[tuple[int, int]]:
        result = []
        for row in range(self.height):
            for col in range(self.width):
                if self.state(col, row) == self.OCCUPIED:
                    result.append((col, row))
        return result

    def inflated_obstacles(self, radius_m: float) -> set[tuple[int, int]]:
        radius = max(0, math.ceil(radius_m / self.resolution_m))
        offsets = [
            (dx, dy)
            for dy in range(-radius, radius + 1)
            for dx in range(-radius, radius + 1)
            if dx * dx + dy * dy <= radius * radius
        ]
        blocked: set[tuple[int, int]] = set()
        for col, row in self.occupied_cells():
            for dx, dy in offsets:
                candidate = col + dx, row + dy
                if self.in_bounds(*candidate):
                    blocked.add(candidate)
        return blocked

    def frontier_clusters(self, min_cells: int = 4) -> list[list[tuple[int, int]]]:
        frontier: set[tuple[int, int]] = set()
        for row in range(1, self.height - 1):
            for col in range(1, self.width - 1):
                if self.state(col, row) != self.FREE:
                    continue
                if any(
                    self.state(col + dx, row + dy) == self.UNKNOWN
                    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                ):
                    frontier.add((col, row))

        clusters: list[list[tuple[int, int]]] = []
        while frontier:
            seed = frontier.pop()
            cluster = [seed]
            pending = [seed]
            while pending:
                col, row = pending.pop()
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    neighbor = col + dx, row + dy
                    if neighbor in frontier:
                        frontier.remove(neighbor)
                        cluster.append(neighbor)
                        pending.append(neighbor)
            if len(cluster) >= min_cells:
                clusters.append(cluster)
        clusters.sort(key=len, reverse=True)
        return clusters

    def nearest_cell_to_centroid(self, cluster: Sequence[tuple[int, int]]) -> tuple[int, int]:
        mean_col = sum(cell[0] for cell in cluster) / len(cluster)
        mean_row = sum(cell[1] for cell in cluster) / len(cluster)
        return min(cluster, key=lambda cell: (cell[0] - mean_col) ** 2 + (cell[1] - mean_row) ** 2)

    def astar(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        clearance_m: float,
        blocked: set[tuple[int, int]] | None = None,
    ) -> list[tuple[int, int]]:
        if not self.in_bounds(*start) or not self.in_bounds(*goal):
            return []
        blocked = blocked if blocked is not None else self.inflated_obstacles(clearance_m)
        if goal in blocked:
            return []
        frontier: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        cost_so_far = {start: 0.0}
        directions = (
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (-1, -1, math.sqrt(2)),
        )
        while frontier:
            _, current_cost, current = heapq.heappop(frontier)
            if current == goal:
                break
            if current_cost > cost_so_far.get(current, math.inf) + 1e-9:
                continue
            for dx, dy, step_cost in directions:
                neighbor = current[0] + dx, current[1] + dy
                if not self.in_bounds(*neighbor) or neighbor in blocked:
                    continue
                if neighbor != goal and self.state(*neighbor) != self.FREE:
                    continue
                new_cost = current_cost + step_cost
                if new_cost >= cost_so_far.get(neighbor, math.inf):
                    continue
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                heuristic = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                heapq.heappush(frontier, (new_cost + heuristic, new_cost, neighbor))
        if goal not in came_from:
            return []
        path = []
        current: tuple[int, int] | None = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def reachable_tree(
        self,
        start: tuple[int, int],
        blocked: set[tuple[int, int]],
    ) -> tuple[dict[tuple[int, int], float], dict[tuple[int, int], tuple[int, int] | None]]:
        if not self.in_bounds(*start):
            return {}, {}
        pending: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
        distances = {start: 0.0}
        parents: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        directions = (
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
        )
        while pending:
            current_distance, current = heapq.heappop(pending)
            if current_distance > distances.get(current, math.inf) + 1e-9:
                continue
            for dx, dy, step in directions:
                neighbor = current[0] + dx, current[1] + dy
                if not self.in_bounds(*neighbor) or neighbor in blocked:
                    continue
                if self.state(*neighbor) != self.FREE:
                    continue
                new_distance = current_distance + step
                if new_distance >= distances.get(neighbor, math.inf):
                    continue
                distances[neighbor] = new_distance
                parents[neighbor] = current
                heapq.heappush(pending, (new_distance, neighbor))
        return distances, parents

    @staticmethod
    def path_from_tree(
        parents: dict[tuple[int, int], tuple[int, int] | None],
        goal: tuple[int, int],
    ) -> list[tuple[int, int]]:
        if goal not in parents:
            return []
        path = []
        current: tuple[int, int] | None = goal
        while current is not None:
            path.append(current)
            current = parents[current]
        path.reverse()
        return path


class CorrelativeScanMatcher:
    MIN_CONFIDENCE = 0.55
    MIN_HIT_POINTS = 8
    MAX_TRANSLATION_WINDOW_M = 0.30
    MAX_ROTATION_WINDOW_RAD = math.radians(15)
    LIKELIHOOD_SIGMA_M = 0.055
    MIN_SCORE_GAIN = 0.02

    def __init__(self, translation_window_m: float = 0.20, rotation_window_deg: float = 10.0) -> None:
        if not all(math.isfinite(v) and v >= 0 for v in (translation_window_m, rotation_window_deg)):
            raise ValueError("匹配搜索窗口必须为非负有限数")
        self.translation_window_m = translation_window_m
        self.rotation_window_rad = math.radians(rotation_window_deg)

    def match(
        self,
        grid: OccupancyGrid,
        predicted: Pose2D,
        points: Sequence[ScanPoint],
        *,
        window_scale: float = 1.0,
        minimum_evidence: float = 4.0,
    ) -> tuple[Pose2D, float]:
        if not math.isfinite(window_scale) or window_scale < 0:
            raise ValueError("搜索窗口倍率必须为非负有限数")
        valid = [p for p in points if math.isfinite(p.angle_rad) and math.isfinite(p.distance_m)
                 and math.isfinite(p.quality) and p.quality >= grid.MIN_QUALITY and p.distance_m > 0]
        if len(valid) < self.MIN_HIT_POINTS or sum(v >= minimum_evidence for v in grid.log_odds) < self.MIN_HIT_POINTS:
            return Pose2D(predicted.x, predicted.y, predicted.yaw), 0.0
        ordered = sorted(valid, key=lambda p: p.angle_rad % math.tau)
        if len(ordered) > 64:
            ordered = [ordered[index * len(ordered) // 64] for index in range(64)]
        sampled = [(p.x, p.y, min(1.0, p.quality)) for p in ordered]
        translation = min(self.MAX_TRANSLATION_WINDOW_M, self.translation_window_m * window_scale)
        rotation = min(self.MAX_ROTATION_WINDOW_RAD, self.rotation_window_rad * window_scale)
        levels = ((translation, rotation, 0.05, math.radians(2.5), 0.12),
                  (0.05, math.radians(2.5), 0.02, math.radians(1), 0.08),
                  (0.015, math.radians(0.8), 0.005, math.radians(0.2), self.LIKELIHOOD_SIGMA_M))
        centers = [Pose2D(predicted.x, predicted.y, predicted.yaw)]
        for level, (xy_window, yaw_window, xy_step, yaw_step, sigma) in enumerate(levels):
            field = grid.likelihood_field(sigma, minimum_evidence)
            candidates = []
            for center in centers:
                for dyaw in self._steps(yaw_window, yaw_step):
                    yaw = wrap_angle(center.yaw + dyaw)
                    if abs(wrap_angle(yaw - predicted.yaw)) > rotation + 1e-9:
                        continue
                    sine, cosine = math.sin(yaw), math.cos(yaw)
                    rotated = [(x * cosine + y * sine, -x * sine + y * cosine, weight) for x, y, weight in sampled]
                    for dx in self._steps(xy_window, xy_step):
                        x = center.x + dx
                        if abs(x - predicted.x) > translation + 1e-9:
                            continue
                        for dy in self._steps(xy_window, xy_step):
                            y = center.y + dy
                            if abs(y - predicted.y) > translation + 1e-9:
                                continue
                            score = self._field_score(grid, field, x, y, rotated)
                            penalty = (0.06 * (abs(x - predicted.x) + abs(y - predicted.y)) / max(translation, 1e-9)
                                       + 0.05 * abs(wrap_angle(yaw - predicted.yaw)) / max(rotation, 1e-9))
                            candidates.append((score - penalty, x, y, yaw))
            candidates.sort(reverse=True)
            centers = []
            for _, x, y, yaw in candidates:
                if not centers or all(math.hypot(x - p.x, y - p.y) >= 0.04
                                      or abs(wrap_angle(yaw - p.yaw)) >= math.radians(2) for p in centers):
                    centers.append(Pose2D(x, y, yaw))
                    if len(centers) >= (3 if level == 0 else 1):
                        break
        best = centers[0]
        field = grid.likelihood_field(self.LIKELIHOOD_SIGMA_M, minimum_evidence)
        def pose_score(pose):
            sine, cosine = math.sin(pose.yaw), math.cos(pose.yaw)
            endpoints = [(x * cosine + y * sine, -x * sine + y * cosine, weight) for x, y, weight in sampled]
            return self._field_score(grid, field, pose.x, pose.y, endpoints)
        if pose_score(best) - pose_score(predicted) < self.MIN_SCORE_GAIN:
            best = Pose2D(predicted.x, predicted.y, predicted.yaw)
        sine, cosine = math.sin(best.yaw), math.cos(best.yaw)
        endpoints = [(p.x * cosine + p.y * sine, -p.x * sine + p.y * cosine, min(1.0, p.quality)) for p in valid]
        weighted_score = observed_weight = total_weight = inlier_weight = 0.0
        inliers = 0
        sectors = set()
        for point, endpoint in zip(valid, endpoints):
            weight = endpoint[2]
            total_weight += weight
            likelihood = self._field_score(grid, field, best.x, best.y, [endpoint])
            col, row = grid.world_to_cell(best.x + endpoint[0], best.y + endpoint[1])
            if likelihood > 0.1 or grid.state(col, row) != grid.UNKNOWN:
                weighted_score += weight * likelihood
                observed_weight += weight
            if likelihood >= 0.4:
                inliers += 1
                inlier_weight += weight
                sectors.add(int((point.angle_rad % math.tau) / (math.tau / 8)))
        confidence = min(weighted_score / max(observed_weight, 1e-9),
                         inlier_weight / max(0.5 * total_weight, 1e-9), 1.0)
        if inliers < self.MIN_HIT_POINTS or len(sectors) < 3:
            confidence = 0.0
        return best, confidence

    @staticmethod
    def _steps(window: float, step: float) -> list[float]:
        if window <= 0:
            return [0.0]
        count = max(1, math.ceil(window / max(step, 1e-9)))
        return [window * index / count for index in range(-count, count + 1)]

    @staticmethod
    def _field_score(grid: OccupancyGrid, field: Sequence[float], x: float, y: float, endpoints) -> float:
        width, height = grid.width, grid.height
        inverse = 1 / grid.resolution_m
        origin_col = grid.origin_col + x * inverse
        origin_row = grid.origin_row - y * inverse
        total = weight_sum = 0.0
        for px, py, weight in endpoints:
            weight_sum += weight
            col, row = origin_col + px * inverse, origin_row - py * inverse
            if not (0 <= col < width - 1 and 0 <= row < height - 1):
                continue
            left, top = int(col), int(row)
            fx, fy = col - left, row - top
            index = top * width + left
            value = ((1 - fx) * (1 - fy) * field[index] + fx * (1 - fy) * field[index + 1]
                     + (1 - fx) * fy * field[index + width] + fx * fy * field[index + width + 1])
            total += weight * value
        return total / max(weight_sum, 1e-9)

    @staticmethod
    def _score(grid: OccupancyGrid, pose: Pose2D, points: Sequence[ScanPoint]) -> float:
        sine, cosine = math.sin(pose.yaw), math.cos(pose.yaw)
        endpoints = [(p.x * cosine + p.y * sine, -p.x * sine + p.y * cosine, min(1.0, p.quality))
                     for p in points if math.isfinite(p.quality) and p.quality >= grid.MIN_QUALITY]
        return CorrelativeScanMatcher._field_score(
            grid, grid.likelihood_field(CorrelativeScanMatcher.LIKELIHOOD_SIGMA_M), pose.x, pose.y, endpoints)


class NavigationEngine:
    MAP_UPDATE_MIN_CONFIDENCE = 0.55
    LOST_AFTER_FAILURES = 3
    BOOTSTRAP_SCANS = 3

    def __init__(
        self,
        grid: OccupancyGrid | None = None,
        max_range_m: float = 3.0,
        robot_radius_m: float = 0.16,
    ) -> None:
        self.grid = grid or OccupancyGrid()
        self.max_range_m = max_range_m
        self.robot_radius_m = robot_radius_m
        self.pose = Pose2D()
        self.start_pose = Pose2D()
        self.matcher = CorrelativeScanMatcher()
        self.auto_enabled = False
        self.state = "待机"
        self.detail = "等待完整扫描"
        self.latest_scan: list[ScanPoint] = []
        self.path_cells: list[tuple[int, int]] = []
        self.target_cell: tuple[int, int] | None = None
        self.frontier_count = 0
        self.reachable_frontier_count = 0
        self.completed_scans = 0
        self.match_score = 0.0
        self.match_failures = 0
        self.rejected_scans = 0
        self.mapping_attempts = 0
        self._predicted_travel_m = 0.0
        self._predicted_strafe_m = 0.0
        self._map_initialized = self.grid.update_count >= self.BOOTSTRAP_SCANS and len(self.grid.occupied_cells()) >= self.matcher.MIN_HIT_POINTS
        self._empty_frontier_scans = 0
        self._parking_goal: tuple[int, int] | None = None
        self.last_progress_angle_world = 0.0
        self.trajectory: list[tuple[float, float]] = [(0.0, 0.0)]

    def reset(self) -> None:
        self.grid.clear()
        self.pose = Pose2D()
        self.start_pose = Pose2D()
        self.state = "待机"
        self.detail = "等待完整扫描"
        self.latest_scan.clear()
        self.path_cells.clear()
        self.target_cell = None
        self.frontier_count = 0
        self.reachable_frontier_count = 0
        self.completed_scans = 0
        self.match_score = 0.0
        self.match_failures = 0
        self.rejected_scans = 0
        self.mapping_attempts = 0
        self._predicted_travel_m = 0.0
        self._predicted_strafe_m = 0.0
        self._map_initialized = False
        self._empty_frontier_scans = 0
        self._parking_goal = None
        self.last_progress_angle_world = 0.0
        self.trajectory = [(0.0, 0.0)]

    def set_auto(self, enabled: bool) -> None:
        self.auto_enabled = bool(enabled)
        if enabled:
            self.state = "准备探索"
            self.detail = "等待下一圈雷达数据"
        else:
            self.state = "仅雷达"
            self.detail = "自动导航未启用"
            self.path_cells.clear()
            self.target_cell = None

    def predict_motion(self, command: VelocityCommand) -> None:
        if command.stopped or command.duration_s <= 0:
            return
        local_x = command.right_mps * command.duration_s
        local_y = command.forward_mps * command.duration_s
        self._predicted_travel_m += math.hypot(local_x, local_y)
        self._predicted_strafe_m += abs(local_x)
        world_x, world_y = self.pose.local_to_world(local_x, local_y)
        self.pose.x = world_x
        self.pose.y = world_y
        self.pose.yaw = wrap_angle(self.pose.yaw + command.yaw_rps * command.duration_s)
        if math.hypot(self.pose.x - self.trajectory[-1][0], self.pose.y - self.trajectory[-1][1]) >= 0.04:
            self.trajectory.append((self.pose.x, self.pose.y))

    def process_scan(self, points: Sequence[ScanPoint]) -> VelocityCommand:
        if self.auto_enabled:
            self.mapping_attempts += 1
        valid = [
            point
            for point in points
            if math.isfinite(point.distance_m) and math.isfinite(point.angle_rad)
            and math.isfinite(point.quality) and point.quality >= self.grid.MIN_QUALITY
            and 0.08 <= point.distance_m <= self.max_range_m
        ]
        unique = {}
        for point in valid:
            key = round(point.angle_rad % math.tau, 6)
            previous = unique.get(key)
            if previous is None or (point.quality, -point.distance_m) > (previous.quality, -previous.distance_m):
                unique[key] = point
        valid = list(unique.values())
        self.latest_scan = valid
        if len(valid) < 12:
            if self.auto_enabled and self.grid.update_count:
                return self._reject_scan(0.0, f"有效点仅 {len(valid)} 个，停车重扫")
            self.state = "扫描不足"
            self.detail = f"本圈只有 {len(valid)} 个有效点"
            return VelocityCommand()

        self.completed_scans += 1
        if not self.auto_enabled:
            self.state = "仅雷达"
            self.detail = f"已接收 {self.completed_scans} 圈"
            return VelocityCommand()

        matching_points = [p for p in valid if p.distance_m < self.max_range_m - 1e-6]
        sectors = {int((p.angle_rad % math.tau) / (math.tau / 8)) for p in matching_points}
        if len(matching_points) < self.matcher.MIN_HIT_POINTS or len(sectors) < 3:
            return self._reject_scan(0.0, "有效障碍回波不足，等待重扫")
        initializing = not self._map_initialized
        if self.grid.update_count == 0:
            corrected, score = Pose2D(self.pose.x, self.pose.y, self.pose.yaw), 1.0
        else:
            scale = min(1.5, 0.6 + self._predicted_travel_m * 2 + self._predicted_strafe_m * 3
                        + max(0.0, 0.85 - self.match_score) + self.match_failures * 0.15)
            corrected, score = self.matcher.match(
                self.grid, self.pose, matching_points, window_scale=0.0 if initializing else scale,
                minimum_evidence=0.25 if initializing else 4.0)
            if not math.isfinite(score) or score < self.MAP_UPDATE_MIN_CONFIDENCE:
                return self._reject_scan(score, "本圈未写入地图，停车重扫")
        self.pose = corrected
        self.match_score = score
        self.match_failures = 0
        self._predicted_travel_m = self._predicted_strafe_m = 0.0
        self.grid.update_scan(self.pose, valid, self.max_range_m, scan_confidence=score)
        if initializing:
            self._map_initialized = (self.grid.update_count >= self.BOOTSTRAP_SCANS
                                     and len(self.grid.occupied_cells()) >= self.matcher.MIN_HIT_POINTS)
        if not self._map_initialized:
            self.state = "建图初始化"
            self.detail = "停车复测初始环境"
            return VelocityCommand()

        return self._plan_next_command()

    def _reject_scan(self, score: float, detail: str) -> VelocityCommand:
        self.match_score = score if math.isfinite(score) else 0.0
        self.match_failures += 1
        self.rejected_scans += 1
        self.state = "定位丢失" if self.match_failures >= self.LOST_AFTER_FAILURES else "定位不可信"
        self.detail = detail
        self.path_cells.clear()
        self.target_cell = None
        return VelocityCommand()

    @staticmethod
    def _densify_for_mapping(points: Sequence[ScanPoint]) -> list[ScanPoint]:
        if len(points) < 2:
            return list(points)
        ordered = sorted(points, key=lambda point: point.angle_rad % math.tau)
        dense: list[ScanPoint] = []
        for index, first in enumerate(ordered):
            second = ordered[(index + 1) % len(ordered)]
            first_angle = first.angle_rad % math.tau
            second_angle = second.angle_rad % math.tau
            if index == len(ordered) - 1:
                second_angle += math.tau
            dense.append(first)
            gap = second_angle - first_angle
            if gap <= math.radians(4) or gap > math.radians(15) or abs(second.distance_m - first.distance_m) > 0.28:
                continue
            subdivisions = min(5, max(1, math.ceil(gap / math.radians(4))))
            for step in range(1, subdivisions):
                fraction = step / subdivisions
                dense.append(
                    ScanPoint(
                        angle_rad=wrap_angle(first_angle + gap * fraction),
                        distance_m=first.distance_m + (second.distance_m - first.distance_m) * fraction,
                        quality=min(first.quality, second.quality) * 0.20,
                    )
                )
        return dense

    def _plan_next_command(self) -> VelocityCommand:
        start = self.grid.world_to_cell(self.pose.x, self.pose.y)
        clusters = self.grid.frontier_clusters()
        self.frontier_count = len(clusters)
        self.reachable_frontier_count = 0

        if clusters:
            best: tuple[float, list[tuple[int, int]], tuple[int, int]] | None = None
            clearance = self.robot_radius_m + 0.02
            blocked = self.grid.inflated_obstacles(clearance)
            route_start = self.grid.world_to_cell(self.start_pose.x, self.start_pose.y)
            reachable, parents = self.grid.reachable_tree(start, blocked)
            route_distances, _ = self.grid.reachable_tree(route_start, blocked)
            current_progress = route_distances.get(start, 0.0)
            for cluster in clusters[:8]:
                goal = self._reachable_frontier_goal(cluster, blocked, set(reachable))
                if goal is None:
                    continue
                path = self.grid.path_from_tree(parents, goal)
                if len(path) < 2:
                    continue
                if not self._is_forward_path(path, route_distances, current_progress):
                    continue
                self.reachable_frontier_count += 1
                progress = route_distances.get(goal, 0.0)
                score = progress * 1.5 + min(40, len(cluster)) * 0.25 - len(path) * 0.20
                if best is None or score > best[0]:
                    best = score, path, goal
            if best is not None:
                self._empty_frontier_scans = 0
                self._parking_goal = None
                _, self.path_cells, self.target_cell = best
                self.state = "探索中"
                self.detail = f"沿单向通道前进，可达前沿 {self.reachable_frontier_count} 个"
                return self._command_along_path()

        probe = self._corridor_probe_command()
        if not probe.stopped:
            self._empty_frontier_scans = 0
            self.path_cells.clear()
            self.target_cell = None
            self.state = "通过转弯"
            self.detail = "地图前沿暂时遮挡，沿雷达开口继续探索"
            return probe

        self._empty_frontier_scans += 1
        if self._empty_frontier_scans < 3:
            self.state = "终点确认"
            self.detail = "停车复测端墙与剩余可通行区域"
            return VelocityCommand()

        self._parking_goal = start
        self.path_cells = []
        self.target_cell = start
        self.state = "泊车完成"
        self.detail = "已确认到达单向通道另一端"
        return VelocityCommand()

    def _corridor_probe_command(self) -> VelocityCommand:
        if not self.latest_scan:
            return VelocityCommand()
        previous_local = wrap_angle(self.last_progress_angle_world - self.pose.yaw)
        route_start = self.grid.world_to_cell(self.start_pose.x, self.start_pose.y)
        current_cell = self.grid.world_to_cell(self.pose.x, self.pose.y)
        route_distances = self._free_distance_field(route_start)
        current_progress = route_distances.get(current_cell)
        candidates: list[tuple[float, float, float]] = []
        for index in range(-10, 11):
            angle = wrap_angle(previous_local + math.radians(index * 10))
            distances = sorted(
                point.distance_m
                for point in self.latest_scan
                if abs(wrap_angle(point.angle_rad - angle)) <= math.radians(20)
            )
            if len(distances) < 3:
                continue
            safety_distance = distances[0]
            if safety_distance < self.robot_radius_m + 0.14:
                continue
            clearance = distances[max(0, len(distances) // 3)]
            turn_cost = abs(wrap_angle(angle - previous_local)) / math.pi
            probe_distance = min(0.55, max(0.28, clearance * 0.55))
            local_x = math.sin(angle) * probe_distance
            local_y = math.cos(angle) * probe_distance
            probe_x, probe_y = self.pose.local_to_world(local_x, local_y)
            probe_cell = self.grid.world_to_cell(probe_x, probe_y)
            probe_progress = route_distances.get(probe_cell)
            if (
                current_progress is not None
                and probe_progress is not None
                and probe_progress < current_progress - 1
            ):
                continue
            novelty = min(
                math.hypot(probe_x - visited_x, probe_y - visited_y)
                for visited_x, visited_y in self.trajectory
            )
            if novelty < 0.18:
                continue
            score = clearance + 0.9 * min(0.55, novelty) - 0.28 * turn_cost
            candidates.append((score, clearance, angle))
        if not candidates:
            return VelocityCommand()
        speed = 0.13
        for _, clearance, angle in sorted(candidates, reverse=True):
            if clearance < max(0.55, self.robot_radius_m * 3.2):
                continue
            command = VelocityCommand(
                forward_mps=speed * math.cos(angle),
                right_mps=speed * math.sin(angle),
                duration_s=0.42,
            )
            if not self._command_has_clearance(command):
                continue
            chosen_world = wrap_angle(self.pose.yaw + angle)
            if abs(wrap_angle(chosen_world - self.last_progress_angle_world)) >= math.radians(60):
                self.last_progress_angle_world = chosen_world
            return command
        return VelocityCommand()

    @staticmethod
    def _is_forward_path(
        path: Sequence[tuple[int, int]],
        route_distances: Mapping[tuple[int, int], float],
        current_progress: float,
    ) -> bool:
        """Reject paths that revisit cells behind the current one-way progress."""
        minimum_progress = current_progress - 1.0
        progress_values = [route_distances.get(cell) for cell in path]
        if any(progress is None or progress < minimum_progress for progress in progress_values):
            return False
        return all(
            later + 1.0 >= earlier
            for earlier, later in zip(progress_values, progress_values[1:])
            if earlier is not None and later is not None
        )

    def _free_distance_field(self, start: tuple[int, int]) -> dict[tuple[int, int], int]:
        if not self.grid.in_bounds(*start):
            return {}
        pending = deque([start])
        distances = {start: 0}
        while pending:
            col, row = pending.popleft()
            distance = distances[(col, row)] + 1
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = col + dx, row + dy
                if neighbor in distances or not self.grid.in_bounds(*neighbor):
                    continue
                if self.grid.state(*neighbor) != self.grid.FREE:
                    continue
                distances[neighbor] = distance
                pending.append(neighbor)
        return distances

    def _reachable_frontier_goal(
        self,
        cluster: Sequence[tuple[int, int]],
        blocked: set[tuple[int, int]],
        reachable: set[tuple[int, int]],
    ) -> tuple[int, int] | None:
        centroid = self.grid.nearest_cell_to_centroid(cluster)
        candidates = sorted(
            (cell for cell in cluster if cell not in blocked and cell in reachable),
            key=lambda cell: (cell[0] - centroid[0]) ** 2 + (cell[1] - centroid[1]) ** 2,
        )
        stride = max(1, len(candidates) // 5)
        for candidate in candidates[::stride]:
            return candidate

        for radius in range(1, 9):
            nearby: set[tuple[int, int]] = set()
            for col, row in cluster[:: max(1, len(cluster) // 12)]:
                for dx in range(-radius, radius + 1):
                    for dy in (-radius, radius):
                        nearby.add((col + dx, row + dy))
                for dy in range(-radius + 1, radius):
                    for dx in (-radius, radius):
                        nearby.add((col + dx, row + dy))
            ordered = sorted(
                (
                    cell
                    for cell in nearby
                    if self.grid.in_bounds(*cell)
                    and cell not in blocked
                    and cell in reachable
                    and self.grid.state(*cell) == self.grid.FREE
                ),
                key=lambda cell: math.hypot(cell[0] - centroid[0], cell[1] - centroid[1]),
            )
            for candidate in ordered[:20]:
                return candidate
        return None

    def _command_along_path(self) -> VelocityCommand:
        if len(self.path_cells) < 2:
            return VelocityCommand()
        lookahead_index = min(len(self.path_cells) - 1, 4)
        world_x, world_y = self.grid.cell_to_world(*self.path_cells[lookahead_index])
        local_x, local_y = self.pose.world_to_local(world_x, world_y)
        distance = math.hypot(local_x, local_y)
        if distance < 1e-6:
            return VelocityCommand()
        speed = 0.14
        forward = speed * local_y / distance
        right = speed * local_x / distance
        command = VelocityCommand(forward, right, 0.0, 0.38)
        guarded = self._collision_guard(command)
        if not guarded.stopped:
            path_direction = wrap_angle(self.pose.yaw + math.atan2(local_x, local_y))
            if abs(wrap_angle(path_direction - self.last_progress_angle_world)) >= math.radians(60):
                self.last_progress_angle_world = path_direction
        return guarded

    def _command_has_clearance(self, command: VelocityCommand) -> bool:
        if command.stopped:
            return True
        speed = math.hypot(command.forward_mps, command.right_mps)
        if speed < 1e-9:
            return True
        travel_angle = math.atan2(command.right_mps, command.forward_mps)
        travel_distance = speed * command.duration_s
        footprint_radius = self.robot_radius_m + 0.035
        for point in self.latest_scan:
            difference = wrap_angle(point.angle_rad - travel_angle)
            along = point.distance_m * math.cos(difference)
            lateral = abs(point.distance_m * math.sin(difference))
            if 0.0 < along < travel_distance + footprint_radius + 0.035 and lateral < footprint_radius:
                return False
        return True

    def _collision_guard(self, command: VelocityCommand) -> VelocityCommand:
        if command.stopped:
            return command
        if not self._command_has_clearance(command):
            self.state = "避障重规划"
            self.detail = "车体扫过区域距离不足，停车更新障碍边界"
            return VelocityCommand()
        return command

    def _find_terminal_cell(self, current: tuple[int, int]) -> tuple[int, int] | None:
        start = self.grid.world_to_cell(self.start_pose.x, self.start_pose.y)
        blocked = self.grid.inflated_obstacles(self.robot_radius_m + 0.07)
        blocked.discard(start)
        pending = deque([start])
        distance = {start: 0}
        best: tuple[int, tuple[int, int]] | None = None
        while pending:
            cell = pending.popleft()
            cell_distance = distance[cell]
            if self.grid.state(*cell) == self.grid.FREE and cell not in blocked:
                if best is None or cell_distance > best[0]:
                    best = cell_distance, cell
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = cell[0] + dx, cell[1] + dy
                if neighbor in distance or neighbor in blocked or not self.grid.in_bounds(*neighbor):
                    continue
                if self.grid.state(*neighbor) != self.grid.FREE:
                    continue
                distance[neighbor] = cell_distance + 1
                pending.append(neighbor)
        return None if best is None else best[1]


class HiddenWorld:
    def __init__(
        self,
        seed: int = 20260903,
        map_path: str | Path | None = None,
        map_data: Mapping[str, object] | None = None,
    ) -> None:
        self.random = random.Random(seed)
        self.resolution_m = 0.02
        self.map_path = Path(map_path) if map_path is not None else None
        self.map_loaded = False
        self.map_error = ""
        self.wall_thickness_m = 0.06
        self.wall_segments: list[tuple[float, float, float, float]] = []
        self._load_builtin_map()
        try:
            if map_data is not None:
                self._load_line_map(map_data)
            elif self.map_path is not None and self.map_path.exists():
                loaded = json.loads(self.map_path.read_text(encoding="utf-8"))
                if not isinstance(loaded, dict):
                    raise ValueError("地图根节点必须是 JSON 对象")
                self._load_line_map(loaded)
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.map_error = str(exc)
            self._load_builtin_map()
        self.pose = Pose2D(self.start_pose.x, self.start_pose.y, self.start_pose.yaw)
        self.robot_radius_m = 0.15
        self.distance_bias_m = 0.0

    def _load_builtin_map(self) -> None:
        self.map_loaded = False
        self.min_x = -2.10
        self.max_x = 2.10
        self.min_y = -2.50
        self.max_y = 2.50
        self.free_regions = [
            (-0.55, -2.30, 0.55, -0.55),
            (-0.55, -1.20, 1.75, -0.25),
            (0.90, -1.20, 1.75, 1.20),
            (-1.65, 0.30, 1.75, 1.20),
            (-1.65, 0.30, -0.50, 2.30),
        ]
        self.obstacles = [
            (0.22, -1.20, 0.62, -0.74),
            (-0.20, 0.76, 0.28, 1.20),
        ]
        self.start_pose = Pose2D(0.0, -2.03, 0.0)
        self.finish = (-1.22, 2.02)
        self.wall_segments = []

    @staticmethod
    def _finite_number(value: object, name: str) -> float:
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f"{name} 必须是有限数值")
        return number

    def _load_line_map(self, data: Mapping[str, object]) -> None:
        bounds = data.get("bounds")
        start = data.get("start")
        finish = data.get("finish")
        obstacles = data.get("obstacles")
        if not isinstance(bounds, Mapping) or not isinstance(start, Mapping) or not isinstance(finish, Mapping):
            raise ValueError("地图缺少 bounds、start 或 finish")
        if not isinstance(obstacles, list):
            raise ValueError("obstacles 必须是线段列表")

        min_x = self._finite_number(bounds["min_x"], "min_x")
        max_x = self._finite_number(bounds["max_x"], "max_x")
        min_y = self._finite_number(bounds["min_y"], "min_y")
        max_y = self._finite_number(bounds["max_y"], "max_y")
        if max_x - min_x < 0.5 or max_y - min_y < 0.5:
            raise ValueError("场地宽度和高度必须至少为 0.5 m")

        start_x = self._finite_number(start["x"], "start.x")
        start_y = self._finite_number(start["y"], "start.y")
        yaw_deg = self._finite_number(start.get("yaw_deg", 0.0), "start.yaw_deg")
        finish_x = self._finite_number(finish["x"], "finish.x")
        finish_y = self._finite_number(finish["y"], "finish.y")
        for name, x, y in (("起点", start_x, start_y), ("终点", finish_x, finish_y)):
            if not (min_x < x < max_x and min_y < y < max_y):
                raise ValueError(f"{name}必须在场地边界内")

        segments: list[tuple[float, float, float, float]] = []
        for index, item in enumerate(obstacles):
            if not isinstance(item, Mapping):
                raise ValueError(f"第 {index + 1} 条障碍不是线段")
            segment = tuple(
                self._finite_number(item[key], f"obstacles[{index}].{key}")
                for key in ("x1", "y1", "x2", "y2")
            )
            if math.hypot(segment[2] - segment[0], segment[3] - segment[1]) < 0.02:
                continue
            segments.append(segment)

        thickness = self._finite_number(data.get("wall_thickness_m", 0.06), "wall_thickness_m")
        if not 0.01 <= thickness <= 0.30:
            raise ValueError("障碍线宽度必须在 0.01 m 到 0.30 m 之间")

        self.min_x, self.max_x = min_x, max_x
        self.min_y, self.max_y = min_y, max_y
        self.free_regions = []
        self.obstacles = []
        self.wall_segments = segments
        self.wall_thickness_m = thickness
        self.start_pose = Pose2D(start_x, start_y, math.radians(yaw_deg))
        self.finish = (finish_x, finish_y)
        self.map_loaded = True

    def reset(self) -> None:
        self.pose = Pose2D(self.start_pose.x, self.start_pose.y, self.start_pose.yaw)
        self.distance_bias_m = 0.0

    def _occupied(self, x: float, y: float, margin: float = 0.0) -> bool:
        if self.map_loaded:
            def point_is_free(check_x: float, check_y: float) -> bool:
                if not (self.min_x < check_x < self.max_x and self.min_y < check_y < self.max_y):
                    return False
                wall_radius = self.wall_thickness_m * 0.5
                return all(
                    self._point_segment_distance(check_x, check_y, *segment) > wall_radius
                    for segment in self.wall_segments
                )
        else:
            def point_is_free(check_x: float, check_y: float) -> bool:
                inside_passage = any(
                    left <= check_x <= right and bottom <= check_y <= top
                    for left, bottom, right, top in self.free_regions
                )
                inside_obstacle = any(
                    left <= check_x <= right and bottom <= check_y <= top
                    for left, bottom, right, top in self.obstacles
                )
                return inside_passage and not inside_obstacle

        if not point_is_free(x, y):
            return True
        if margin <= 0:
            return False
        for index in range(16):
            angle = math.tau * index / 16
            if not point_is_free(x + margin * math.cos(angle), y + margin * math.sin(angle)):
                return True
        return False

    @staticmethod
    def _point_segment_distance(
        x: float,
        y: float,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
    ) -> float:
        delta_x = x2 - x1
        delta_y = y2 - y1
        length_squared = delta_x * delta_x + delta_y * delta_y
        if length_squared <= 1e-12:
            return math.hypot(x - x1, y - y1)
        fraction = clamp(((x - x1) * delta_x + (y - y1) * delta_y) / length_squared, 0.0, 1.0)
        return math.hypot(x - (x1 + fraction * delta_x), y - (y1 + fraction * delta_y))

    def ray_distance(self, angle_rad: float, max_range_m: float, origin: tuple[float, float] | None = None) -> float:
        world_angle = self.pose.yaw + angle_rad
        ox, oy = origin if origin is not None else (self.pose.x, self.pose.y)
        sine, cosine = math.sin(world_angle), math.cos(world_angle)
        def occupied(distance):
            return self._occupied(ox + sine * distance, oy + cosine * distance)
        if occupied(0.0):
            return 0.0
        step = min(self.resolution_m, self.wall_thickness_m * 0.5)
        lower = 0.0
        while lower < max_range_m:
            upper = min(max_range_m, lower + step)
            if occupied(upper):
                while upper - lower > 0.0001:
                    middle = (lower + upper) * 0.5
                    if occupied(middle):
                        upper = middle
                    else:
                        lower = middle
                return upper
            lower = upper
        return max_range_m

    def scan(
        self,
        sample_count: int = 120,
        max_range_m: float = 3.0,
        point_noise_m: float = 0.007,
        drift_step_m: float = 0.0025,
        drift_limit_m: float = 0.035,
    ) -> list[ScanPoint]:
        self.distance_bias_m = clamp(
            self.distance_bias_m + self.random.gauss(0.0, drift_step_m),
            -drift_limit_m,
            drift_limit_m,
        )
        points = []
        for index in range(sample_count):
            angle = math.tau * index / sample_count
            true_distance = self.ray_distance(angle, max_range_m)
            distance = true_distance + self.distance_bias_m + self.random.gauss(0.0, point_noise_m)
            if self.random.random() < 0.012:
                distance += self.random.uniform(-0.12, 0.12)
            points.append(ScanPoint(angle, clamp(distance, 0.08, max_range_m), 1.0))
        return points

    def apply(self, command: VelocityCommand) -> VelocityCommand:
        if command.stopped or command.duration_s <= 0:
            return VelocityCommand()
        actuation_scale = self.random.uniform(0.96, 1.04)
        local_x = command.right_mps * command.duration_s * actuation_scale
        local_y = command.forward_mps * command.duration_s * actuation_scale
        candidate_x, candidate_y = self.pose.local_to_world(local_x, local_y)
        travel_distance = math.hypot(local_x, local_y)
        translation_applied = True
        steps = max(1, math.ceil(travel_distance / (self.resolution_m * 0.5)))
        for step in range(1, steps + 1):
            fraction = step / steps
            check_x, check_y = self.pose.local_to_world(local_x * fraction, local_y * fraction)
            if self._occupied(check_x, check_y, self.robot_radius_m):
                translation_applied = False
                break
        if translation_applied:
            self.pose.x = candidate_x
            self.pose.y = candidate_y
        yaw_scale = self.random.uniform(0.97, 1.03)
        self.pose.yaw = wrap_angle(
            self.pose.yaw + command.yaw_rps * command.duration_s * yaw_scale
        )
        return VelocityCommand(
            forward_mps=command.forward_mps * actuation_scale if translation_applied else 0.0,
            right_mps=command.right_mps * actuation_scale if translation_applied else 0.0,
            yaw_rps=command.yaw_rps * yaw_scale,
            duration_s=command.duration_s,
        )


def mecanum_mix(command: VelocityCommand) -> tuple[float, float, float, float]:
    forward = command.forward_mps
    right = command.right_mps
    rotation = command.yaw_rps
    values = [
        forward - right - rotation,
        forward + right + rotation,
        forward + right - rotation,
        forward - right + rotation,
    ]
    scale = max(1.0, *(abs(value) for value in values))
    return tuple(value / scale for value in values)  # type: ignore[return-value]
