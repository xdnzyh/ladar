from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import heapq
import json
import math
from pathlib import Path
import random
from typing import Mapping, Sequence
from scan_geometry import range_tolerance_scale


CARDINAL_STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1))
GRID_STEPS = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)),
    (-1, -1, math.sqrt(2)),
)


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
    is_echo: bool | None = None
    timestamp_s: float | None = None
    time_error_s: float | None = None
    angle_error_rad: float | None = None
    distance_error_m: float | None = None
    pixel: int | None = None
    calibration_version: str | None = None
    source: str | None = None
    session: str | None = None
    immediate_free: bool = False

    @property
    def x(self) -> float:
        return self.distance_m * math.sin(self.angle_rad)

    @property
    def y(self) -> float:
        return self.distance_m * math.cos(self.angle_rad)

    def has_echo(self, max_range_m: float) -> bool:
        if self.is_echo is not None:
            return bool(self.is_echo)
        return self.distance_m < max_range_m - 1e-6

    def evidence_weight(self, resolution_m: float) -> float:
        weight = min(1.0, max(0.0, self.quality))
        if self.angle_error_rad is not None and math.isfinite(self.angle_error_rad):
            scale = max(resolution_m, 1e-9)
            weight *= 1.0 / (1.0 + (self.distance_m * max(0.0, self.angle_error_rad) / scale) ** 2)
        if self.distance_error_m is not None and math.isfinite(self.distance_error_m):
            scale = max(resolution_m, 1e-9)
            weight *= 1.0 / (1.0 + (max(0.0, self.distance_error_m) / scale) ** 2)
        return weight


@dataclass(frozen=True)
class VelocityCommand:
    forward_mps: float = 0.0
    right_mps: float = 0.0
    yaw_rps: float = 0.0
    duration_s: float = 0.0
    recovery_translation: bool = False

    @property
    def stopped(self) -> bool:
        return (
            abs(self.forward_mps) < 1e-9
            and abs(self.right_mps) < 1e-9
            and abs(self.yaw_rps) < 1e-9
        )


@dataclass(frozen=True)
class MapUpdateSummary:
    changed_cells: tuple[tuple[int, int], ...] = ()
    state_changed_cells: tuple[tuple[int, int], ...] = ()
    out_of_bounds_points: int = 0
    known_cells: int = 0
    map_revision: int = 0


@dataclass
class ScanMatchResult:
    corrected_sensor_pose: Pose2D
    data_score: float
    prior_penalty: float = 0.0
    predicted_position_score: float = 0.0
    best_candidate_score: float = 0.0
    second_candidate_score: float | None = None
    inlier_count: int = 0
    valid_direction_count: int = 0
    known_overlap: float = 0.0
    touched_search_boundary: bool = False
    degenerate: bool = False
    rejection_reason: str = ""

    @property
    def confidence(self) -> float:
        return self.data_score

    def __iter__(self):
        yield self.corrected_sensor_pose
        yield self.data_score


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
        self.origin_row = self.height // 2 if self.height >= 80 else round(self.height * 0.84)
        self.log_odds = [0] * (self.width * self.height)
        self.update_count = 0
        self._revision = 0
        self._known_count = 0
        self._observed_cells: set[tuple[int, int]] = set()
        self.assumed_free_cells: set[tuple[int, int]] = set()
        self._measured_free_cells: set[tuple[int, int]] = set()
        self._occupied_cache: tuple[int, list[tuple[int, int]]] | None = None
        self._inflated_cache: dict[tuple[int, float], set[tuple[int, int]]] = {}
        self._field_cache = {}

    def clear(self) -> None:
        self.log_odds[:] = [0] * len(self.log_odds)
        self.update_count = 0
        self._revision += 1
        self._known_count = 0
        self._observed_cells.clear()
        self.assumed_free_cells.clear()
        self._measured_free_cells.clear()
        self._occupied_cache = None
        self._inflated_cache.clear()
        self._field_cache.clear()

    def _index(self, col: int, row: int) -> int:
        return row * self.width + col

    def clear_region(self, x: float, y: float, radius_m: float) -> None:
        if not all(math.isfinite(v) for v in (x, y, radius_m)) or radius_m <= 0:
            raise ValueError("局部修正范围无效")
        center = self.world_to_cell(x, y)
        radius = math.ceil(radius_m / self.resolution_m)
        for row in range(max(0, center[1] - radius), min(self.height, center[1] + radius + 1)):
            for col in range(max(0, center[0] - radius), min(self.width, center[0] + radius + 1)):
                cell = col, row
                if math.dist(self.cell_to_world(*cell), (x, y)) > radius_m:
                    continue
                self._add(*cell, -self.value(*cell))
                self._observed_cells.discard(cell)
                self.assumed_free_cells.discard(cell)
                self._measured_free_cells.discard(cell)

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

    def _add(self, col: int, row: int, amount: float) -> bool:
        if not self.in_bounds(col, row):
            return False
        index = self._index(col, row)
        before = self.log_odds[index]
        before_known = before <= -2 or before >= 4
        after = max(-20, min(20, before + amount))
        if after == before:
            if before != 0:
                self._observed_cells.add((col, row))
            return False
        self.log_odds[index] = after
        self._observed_cells.add((col, row))
        after_known = after <= -2 or after >= 4
        self._known_count += int(after_known) - int(before_known)
        self._revision += 1
        self._occupied_cache = None
        self._inflated_cache.clear()
        self._field_cache.clear()
        return True

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

    def _snap_thin_wall_hit(self, cell: tuple[int, int]) -> tuple[int, int]:
        col, row = cell
        if not self.in_bounds(col, row):
            return cell
        best = cell
        best_score = self.value(col, row)
        for delta_row in (-1, 0, 1):
            for delta_col in (-1, 0, 1):
                candidate = (col + delta_col, row + delta_row)
                if candidate == cell or not self.in_bounds(*candidate):
                    continue
                value = self.value(*candidate)
                if value <= 0:
                    continue
                distance_penalty = 0.1 * (abs(delta_col) + abs(delta_row))
                score = value - distance_penalty
                if score > best_score:
                    best = candidate
                    best_score = score
        return best

    def update_scan(
        self,
        pose: Pose2D,
        points: Sequence[ScanPoint],
        max_range_m: float,
        min_range_m: float = 0.08,
        scan_confidence: float = 1.0,
        add_only: bool = False,
    ) -> MapUpdateSummary:
        if not math.isfinite(scan_confidence) or scan_confidence < self.MIN_SCAN_CONFIDENCE:
            return MapUpdateSummary(known_cells=self._known_count, map_revision=self._revision)
        confidence = min(1.0, scan_confidence)
        start = self.world_to_cell(pose.x, pose.y)
        hits = {}
        frees = {}
        measured_frees = set()
        immediate_frees = set()
        out_of_bounds = 0
        valid_points = 0
        valid = [point for point in points
                 if math.isfinite(point.angle_rad) and math.isfinite(point.quality)
                 and point.quality >= self.MIN_QUALITY
                 and min_range_m <= point.distance_m <= max_range_m]
        rays = list(valid)
        if len(valid) > 1:
            ordered = sorted(valid, key=lambda point: point.angle_rad % math.tau)
            for left, right in zip(ordered, ordered[1:] + ordered[:1]):
                gap = (right.angle_rad - left.angle_rad) % math.tau
                # Only bridge ordinary scan spacing, never an unobserved sector.
                if not 0 < gap <= math.radians(5.0):
                    continue
                distance = min(left.distance_m, right.distance_m)
                if left.has_echo(max_range_m) or right.has_echo(max_range_m):
                    distance -= math.sqrt(2) * self.resolution_m
                if distance < min_range_m:
                    continue
                steps = max(1, math.ceil(gap * distance / (0.5 * self.resolution_m)))
                # Use the weaker ray's evidence; extra rays do not count as
                # repeated observations (frees stores the maximum per cell).
                quality = min(left.evidence_weight(self.resolution_m),
                              right.evidence_weight(self.resolution_m))
                for index in range(1, steps):
                    rays.append(ScanPoint(left.angle_rad + gap * index / steps,
                                          distance, quality, False,
                                          source="assumed_open" if "assumed_open" in (left.source, right.source) else "free_space",
                                          immediate_free=left.immediate_free and right.immediate_free))
        for point in rays:
            if (not math.isfinite(point.angle_rad) or not math.isfinite(point.quality)
                    or point.quality < self.MIN_QUALITY
                    or not min_range_m <= point.distance_m <= max_range_m):
                continue
            valid_points += 1
            evidence_weight = point.evidence_weight(self.resolution_m) * confidence / (1 + 0.1 * (point.distance_m / max_range_m) ** 2)
            if evidence_weight <= 0:
                continue
            endpoint = pose.local_to_world(point.x, point.y)
            end = self.world_to_cell(*endpoint)
            wall_model = bool(point.source and point.source.split(":", 1)[0] in {"thin_wall", "stable_wall"})
            if wall_model:
                end = self._snap_thin_wall_hit(end)
            cells = self._line_cells(start, end)
            has_hit = point.has_echo(max_range_m)
            for cell in cells[:-1] if has_hit else cells:
                if add_only and self.state(*cell) == self.OCCUPIED:
                    break
                frees[cell] = max(frees.get(cell, 0.0), evidence_weight)
                if point.immediate_free and not has_hit:
                    immediate_frees.add(cell)
                if point.source != "assumed_open":
                    measured_frees.add(cell)
            if has_hit:
                hit_log_odds = 2.25 if wall_model else self.HIT_LOG_ODDS
                hit_weight = evidence_weight
                if wall_model:
                    # These endpoints already passed line span, support and RMS
                    # checks. Raw range variance must not suppress that fitted
                    # wall again; retain it on rays and in the motion guard.
                    hit_weight = min(1.0, max(0.0, point.quality)) * confidence
                amount = hit_log_odds * hit_weight
                if wall_model and point.quality >= .9 and confidence >= .55:
                    # Coherent fitted walls are direct obstacle evidence. One
                    # high-quality observation supersedes old free-space odds.
                    amount = max(amount, 4.5 - self.value(*end))
                hits[end] = max(hits.get(end, 0.0), amount)
            if not self.in_bounds(*end):
                out_of_bounds += 1
        if immediate_frees:
            # Bresenham rays can leave isolated unknown cells even with dense
            # angular sampling. Fill only sectors bounded by two immediate
            # free rays, up to their shorter range, with old-wall occlusion.
            from bisect import bisect_right
            ordered_free = sorted(valid, key=lambda p: p.angle_rad % math.tau)
            angles = [p.angle_rad % math.tau for p in ordered_free]
            reach = min(1.0, max(p.distance_m for p in valid if p.immediate_free))
            radius_cells = math.ceil(reach / self.resolution_m) + 1
            for row in range(max(0, start[1]-radius_cells), min(self.height, start[1]+radius_cells+1)):
                for col in range(max(0, start[0]-radius_cells), min(self.width, start[0]+radius_cells+1)):
                    cell = (col, row)
                    if cell in immediate_frees or cell in hits or self.value(*cell) > 0:
                        continue
                    x, y = pose.world_to_local(*self.cell_to_world(col, row))
                    distance = math.hypot(x, y)
                    if distance > reach:
                        continue
                    angle = math.atan2(x, y) % math.tau
                    index = bisect_right(angles, angle)
                    left, right = ordered_free[index-1], ordered_free[index % len(angles)]
                    gap = (right.angle_rad-left.angle_rad) % math.tau
                    if (not left.immediate_free or not right.immediate_free
                            or not 0 < gap <= math.radians(5)
                            or distance > min(left.distance_m, right.distance_m)):
                        continue
                    if any(self.state(*c) == self.OCCUPIED for c in self._line_cells(start, cell)):
                        continue
                    frees[cell] = max(frees.get(cell, 0), confidence)
                    immediate_frees.add(cell)
        if not hits and not frees:
            return MapUpdateSummary(out_of_bounds_points=out_of_bounds, known_cells=self._known_count,
                                    map_revision=self._revision)
        changed = set()
        state_changed = set()
        for cell, weight in frees.items():
            if cell not in hits and not (add_only and self.value(*cell) > 0):
                was_assumed = cell in self.assumed_free_cells
                if cell in measured_frees:
                    self._measured_free_cells.add(cell)
                    self.assumed_free_cells.discard(cell)
                elif cell not in self._measured_free_cells and self.in_bounds(*cell):
                    self.assumed_free_cells.add(cell)
                if was_assumed != (cell in self.assumed_free_cells):
                    self._revision += 1
                before = self.state(*cell)
                amount = -self.FREE_LOG_ODDS * weight
                if cell in immediate_frees:
                    amount = min(amount, -2.0 - self.value(*cell))
                if self._add(*cell, amount):
                    changed.add(cell)
                    if self.state(*cell) != before:
                        state_changed.add(cell)
        for cell, amount in hits.items():
            before = self.state(*cell)
            # No-return extrapolation is not evidence against a newly measured
            # obstacle. Do not make a wall repay many scans of assumed vacancy.
            if cell in self.assumed_free_cells and self.value(*cell) < 0:
                self._add(*cell, -self.value(*cell))
            self.assumed_free_cells.discard(cell)
            self._measured_free_cells.discard(cell)
            if self._add(*cell, amount):
                changed.add(cell)
                if self.state(*cell) != before:
                    state_changed.add(cell)
        if valid_points:
            self.update_count += 1
        return MapUpdateSummary(
            changed_cells=tuple(sorted(changed)),
            state_changed_cells=tuple(sorted(state_changed)),
            out_of_bounds_points=out_of_bounds,
            known_cells=self._known_count,
            map_revision=self._revision,
        )

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

    def known_area_m2(self) -> float:
        return len(self._observed_cells) * self.resolution_m * self.resolution_m

    def occupied_cells(self) -> list[tuple[int, int]]:
        if self._occupied_cache is None or self._occupied_cache[0] != self._revision:
            result = []
            for row in range(self.height):
                for col in range(self.width):
                    if self.state(col, row) == self.OCCUPIED:
                        result.append((col, row))
            self._occupied_cache = (self._revision, result)
        return list(self._occupied_cache[1])

    def inflated_obstacles(self, radius_m: float) -> set[tuple[int, int]]:
        radius = max(0, math.ceil(radius_m / self.resolution_m))
        cache_key = (self._revision, float(radius_m))
        cached = self._inflated_cache.get(cache_key)
        if cached is not None:
            return set(cached)
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
        self._inflated_cache[cache_key] = set(blocked)
        return blocked

    def frontier_clusters(self, min_cells: int = 4) -> list[list[tuple[int, int]]]:
        frontier: set[tuple[int, int]] = set()
        for row in range(1, self.height - 1):
            for col in range(1, self.width - 1):
                if self.state(col, row) != self.FREE:
                    continue
                if any(
                    self.state(col + dx, row + dy) == self.UNKNOWN
                    for dx, dy in CARDINAL_STEPS
                ):
                    frontier.add((col, row))

        clusters: list[list[tuple[int, int]]] = []
        while frontier:
            seed = frontier.pop()
            cluster = [seed]
            pending = [seed]
            while pending:
                col, row = pending.pop()
                for dx, dy in CARDINAL_STEPS:
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
        turn_penalty: float = 0.0,
        initial_step: tuple[int, int] | None = None,
        allowed_steps: Sequence[tuple[int, int, float]] = GRID_STEPS,
    ) -> list[tuple[int, int]]:
        if not self.in_bounds(*start) or not self.in_bounds(*goal):
            return []
        blocked = blocked if blocked is not None else self.inflated_obstacles(clearance_m)
        if goal in blocked:
            return []
        if turn_penalty > 0:
            return self._heading_aware_astar(start, goal, blocked, turn_penalty, initial_step, allowed_steps)
        frontier: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        cost_so_far = {start: 0.0}
        while frontier:
            _, current_cost, current = heapq.heappop(frontier)
            if current == goal:
                break
            if current_cost > cost_so_far.get(current, math.inf) + 1e-9:
                continue
            for dx, dy, step_cost in allowed_steps:
                neighbor = current[0] + dx, current[1] + dy
                if not self._step_allowed(current, neighbor, blocked, goal):
                    continue
                new_cost = current_cost + step_cost
                if new_cost >= cost_so_far.get(neighbor, math.inf):
                    continue
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                heuristic = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                heapq.heappush(frontier, (new_cost + heuristic, new_cost, neighbor))
        return self.path_from_tree(came_from, goal)

    def _heading_aware_astar(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        blocked: set[tuple[int, int]],
        turn_penalty: float,
        initial_step: tuple[int, int] | None,
        allowed_steps: Sequence[tuple[int, int, float]] = GRID_STEPS,
    ) -> list[tuple[int, int]]:
        initial_direction = next(
            (index for index, (dx, dy, _) in enumerate(GRID_STEPS) if (dx, dy) == initial_step),
            -1,
        )
        start_state = start[0], start[1], initial_direction
        frontier = [(0.0, 0.0, start_state)]
        came_from: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start_state: None}
        cost_so_far = {start_state: 0.0}
        goal_state: tuple[int, int, int] | None = None
        while frontier:
            _, current_cost, current = heapq.heappop(frontier)
            if current_cost > cost_so_far.get(current, math.inf) + 1e-9:
                continue
            current_cell = current[0], current[1]
            if current_cell == goal:
                goal_state = current
                break
            for direction, (dx, dy, step_cost) in enumerate(GRID_STEPS):
                if (dx, dy, step_cost) not in allowed_steps:
                    continue
                neighbor = current[0] + dx, current[1] + dy
                if not self._step_allowed(current_cell, neighbor, blocked, goal):
                    continue
                bend = self._direction_change(current[2], direction)
                new_cost = current_cost + step_cost + turn_penalty * bend
                neighbor_state = neighbor[0], neighbor[1], direction
                if new_cost >= cost_so_far.get(neighbor_state, math.inf):
                    continue
                cost_so_far[neighbor_state] = new_cost
                came_from[neighbor_state] = current
                heuristic = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                heapq.heappush(frontier, (new_cost + heuristic, new_cost, neighbor_state))
        if goal_state is None:
            return []
        path = []
        current_state: tuple[int, int, int] | None = goal_state
        while current_state is not None:
            path.append((current_state[0], current_state[1]))
            current_state = came_from[current_state]
        path.reverse()
        return path

    @staticmethod
    def _direction_change(previous: int, current: int) -> float:
        if previous < 0 or previous == current:
            return 0.0
        previous_dx, previous_dy, previous_length = GRID_STEPS[previous]
        current_dx, current_dy, current_length = GRID_STEPS[current]
        cosine = clamp(
            (previous_dx * current_dx + previous_dy * current_dy) / (previous_length * current_length),
            -1.0,
            1.0,
        )
        return math.acos(cosine) / (math.pi / 4)

    def _step_allowed(
        self,
        current: tuple[int, int],
        neighbor: tuple[int, int],
        blocked: set[tuple[int, int]],
        goal: tuple[int, int] | None = None,
    ) -> bool:
        if not self.in_bounds(*neighbor) or neighbor in blocked:
            return False
        if neighbor != goal and self.state(*neighbor) != self.FREE:
            return False
        dx = neighbor[0] - current[0]
        dy = neighbor[1] - current[1]
        if abs(dx) == 1 and abs(dy) == 1:
            side_a = current[0] + dx, current[1]
            side_b = current[0], current[1] + dy
            for side in (side_a, side_b):
                if (not self.in_bounds(*side) or side in blocked
                        or self.state(*side) != self.FREE):
                    return False
        return True

    def reachable_tree(
        self,
        start: tuple[int, int],
        blocked: set[tuple[int, int]],
        allowed_steps: Sequence[tuple[int, int, float]] = GRID_STEPS,
    ) -> tuple[dict[tuple[int, int], float], dict[tuple[int, int], tuple[int, int] | None]]:
        if not self.in_bounds(*start):
            return {}, {}
        pending: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
        distances = {start: 0.0}
        parents: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        while pending:
            current_distance, current = heapq.heappop(pending)
            if current_distance > distances.get(current, math.inf) + 1e-9:
                continue
            for dx, dy, step in allowed_steps:
                neighbor = current[0] + dx, current[1] + dy
                if not self._step_allowed(current, neighbor, blocked):
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
        translation_window_scale: float | None = None,
        rotation_window_scale: float | None = None,
        minimum_evidence: float = 4.0,
    ) -> ScanMatchResult:
        translation_scale = window_scale if translation_window_scale is None else translation_window_scale
        rotation_scale = window_scale if rotation_window_scale is None else rotation_window_scale
        if not all(math.isfinite(value) and value >= 0 for value in (
                window_scale, translation_scale, rotation_scale)):
            raise ValueError("搜索窗口倍率必须为非负有限数")
        valid = [p for p in points if math.isfinite(p.angle_rad) and math.isfinite(p.distance_m)
                 and math.isfinite(p.quality) and p.quality >= grid.MIN_QUALITY and p.distance_m > 0]
        if len(valid) < self.MIN_HIT_POINTS or sum(v >= minimum_evidence for v in grid.log_odds) < self.MIN_HIT_POINTS:
            return ScanMatchResult(Pose2D(predicted.x, predicted.y, predicted.yaw), 0.0,
                                   rejection_reason="有效观测或地图证据不足")
        ordered = sorted(valid, key=lambda p: p.angle_rad % math.tau)
        if len(ordered) > 64:
            ordered = [ordered[index * len(ordered) // 64] for index in range(64)]
        sampled = [(p.x, p.y, p.evidence_weight(grid.resolution_m)) for p in ordered]
        translation = min(self.MAX_TRANSLATION_WINDOW_M, self.translation_window_m * translation_scale)
        rotation = min(self.MAX_ROTATION_WINDOW_RAD, self.rotation_window_rad * rotation_scale)
        levels = ((translation, rotation, 0.05, math.radians(2.5), 0.12),
                  (0.05, math.radians(2.5), 0.02, math.radians(1), 0.08),
                  (0.015, math.radians(0.8), 0.005, math.radians(0.2), self.LIKELIHOOD_SIGMA_M))
        centers = [Pose2D(predicted.x, predicted.y, predicted.yaw)]
        last_candidates = []
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
            last_candidates = candidates
            centers = []
            for _, x, y, yaw in candidates:
                if not centers or all(math.hypot(x - p.x, y - p.y) >= 0.04
                                      or abs(wrap_angle(yaw - p.yaw)) >= math.radians(2) for p in centers):
                    centers.append(Pose2D(x, y, yaw))
                    if len(centers) >= (5 if level < 2 else 3):
                        break
        best = centers[0]
        field = grid.likelihood_field(self.LIKELIHOOD_SIGMA_M, minimum_evidence)
        def pose_score(pose, *, range_adaptive=True):
            sine, cosine = math.sin(pose.yaw), math.cos(pose.yaw)
            endpoints = [(x * cosine + y * sine, -x * sine + y * cosine, weight) for x, y, weight in sampled]
            return self._field_score(grid, field, pose.x, pose.y, endpoints, range_adaptive=range_adaptive)
        best_raw_score = pose_score(best)
        predicted_score = pose_score(predicted)
        if best_raw_score - predicted_score < self.MIN_SCORE_GAIN:
            best = Pose2D(predicted.x, predicted.y, predicted.yaw)
            best_raw_score = predicted_score
        sine, cosine = math.sin(best.yaw), math.cos(best.yaw)
        endpoints = [(p.x * cosine + p.y * sine, -p.x * sine + p.y * cosine,
                     p.evidence_weight(grid.resolution_m)) for p in valid]
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
        candidate_scores = []
        ambiguous_candidates = []
        for candidate in centers:
            score = pose_score(candidate)
            separation = math.hypot(candidate.x - best.x, candidate.y - best.y)
            yaw_separation = abs(wrap_angle(candidate.yaw - best.yaw))
            if separation >= 2 * grid.resolution_m or yaw_separation >= math.radians(2):
                candidate_scores.append(score)
                if abs(best_raw_score - score) < 0.025:
                    # Nearby search samples can belong to one broad optimum.
                    # A second solution needs a valley between the poses, or
                    # a high-score ridge spanning a substantial pose interval.
                    bridge_scores = [pose_score(Pose2D(
                        best.x + fraction * (candidate.x - best.x),
                        best.y + fraction * (candidate.y - best.y),
                        wrap_angle(best.yaw + fraction * wrap_angle(candidate.yaw - best.yaw)),
                    )) for fraction in (0.25, 0.5, 0.75)]
                    valley = min(bridge_scores) < min(best_raw_score, score) - 0.015
                    wide_ridge = (separation >= max(4 * grid.resolution_m, 2 * self.LIKELIHOOD_SIGMA_M)
                                  or yaw_separation >= math.radians(5))
                    if valley or wide_ridge:
                        ambiguous_candidates.append(score)
        second_score = max(candidate_scores) if candidate_scores else None
        boundary = (
            abs(best.x - predicted.x) >= max(translation - grid.resolution_m * 0.5, 0.0)
            or abs(best.y - predicted.y) >= max(translation - grid.resolution_m * 0.5, 0.0)
            or abs(wrap_angle(best.yaw - predicted.yaw)) >= max(rotation - math.radians(0.3), 0.0)
        )
        separated_ambiguity = confidence >= 0.75 and bool(ambiguous_candidates)
        # Judge geometric observability at the original spatial scale; the
        # broader noise kernel alone must not turn a constrained wall into
        # an apparently flat corridor axis.
        geometric_score = pose_score(best, range_adaptive=False)
        local_probe_scores = [
            pose_score(Pose2D(best.x + dx * grid.resolution_m, best.y + dy * grid.resolution_m,
                              wrap_angle(best.yaw + yaw)), range_adaptive=False)
            for dx, dy, yaw in ((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0),
                                (0, 0, -math.radians(1)), (0, 0, math.radians(1)))
        ]
        # A corridor may constrain lateral position while leaving forward
        # position unobservable; test each axis, not the spread of all axes.
        flat_platform = confidence >= 0.75 and any(
            max(abs(left - geometric_score), abs(right - geometric_score)) < 0.008
            for left, right in zip(local_probe_scores[::2], local_probe_scores[1::2])
        )
        return ScanMatchResult(
            corrected_sensor_pose=best,
            data_score=confidence,
            prior_penalty=max(0.0, best_raw_score - confidence),
            predicted_position_score=predicted_score,
            best_candidate_score=best_raw_score,
            second_candidate_score=second_score,
            inlier_count=inliers,
            valid_direction_count=len(sectors),
            known_overlap=observed_weight / max(total_weight, 1e-9),
            touched_search_boundary=boundary,
            degenerate=separated_ambiguity or flat_platform,
            rejection_reason=("存在相互分离的近似等分位置" if separated_ambiguity
                              else "某个方向缺少可定位的几何约束" if flat_platform else ""),
        )

    @staticmethod
    def _steps(window: float, step: float) -> list[float]:
        if window <= 0:
            return [0.0]
        count = max(1, math.ceil(window / max(step, 1e-9)))
        return [window * index / count for index in range(-count, count + 1)]

    @staticmethod
    def _field_score(grid: OccupancyGrid, field: Sequence[float], x: float, y: float, endpoints,
                     *, range_adaptive: bool = True) -> float:
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
            # Scale Gaussian variance with range (sigma grows by sqrt(scale)).
            # Local endpoint radius is rotation invariant: each return gets
            # its own tolerance in search, confidence and ambiguity checks.
            scale = range_tolerance_scale(math.hypot(px, py)) if range_adaptive else 1.0
            value = value ** (1.0 / scale)
            total += weight * value
        return total / max(weight_sum, 1e-9)

class NavigationEngine:
    MAP_UPDATE_MIN_CONFIDENCE = 0.55
    LOST_AFTER_FAILURES = 3
    BOOTSTRAP_SCANS = 3
    TRANSLATION_MODES = tuple("WSADQEZC")
    MAX_MOTION_SEGMENT_M = 0.20
    DIAGONAL_MOTION_SPEED_MPS = 0.11
    MAX_DIAGONAL_SEGMENT_M = 0.10
    DIAGONAL_PROBE_SEGMENT_M = 0.08
    DIAGONAL_RECOVERY_SEGMENT_M = 0.02
    DIAGONAL_DIRECTION_RATIO = 0.72
    DIAGONAL_POSE_UNCERTAINTY_M = 0.04
    PATH_TURN_PENALTY = 0.75
    PARKING_SEARCH_STEP_M = 0.10
    PARKING_SEARCH_SPEED_MPS = 0.10
    PARKING_SEARCH_MAX_ATTEMPTS = 3
    RADAR_GAP_MIN_WIDTH_M = 0.40
    RADAR_GAP_LOCK_TOLERANCE_RAD = math.radians(30)
    RADAR_GAP_DIRECT_TOLERANCE_RAD = math.radians(25)

    def __init__(
        self,
        grid: OccupancyGrid | None = None,
        max_range_m: float = 3.0,
        robot_radius_m: float = 0.16,
        sensor_offset_x_m: float = 0.0,
        sensor_offset_y_m: float = 0.0,
        sensor_offset_yaw_rad: float = 0.0,
        min_range_m: float = 0.08,
        path_turn_penalty: float = PATH_TURN_PENALTY,
        translation_capabilities: Mapping[str, Mapping[str, object]] | None = None,
        safety_clearance_m: float = 0.035,
        safety_stop_distance_m: float = 0.0,
        safety_max_observation_age_s: float = 0.0,
        safety_speed_upper_bound_mps: float | None = None,
        unobserved_clear_range_m: float = 0.0,
        prefer_forward_exploration: bool = False,
        forward_only: bool = False,
        local_probe_after_two_scans: bool = False,
        rotation_enabled: bool = False,
        immediate_navigation: bool = False,
        prioritize_unexplored_gaps: bool = False,
        radar_gap_steering: bool = False,
        forward_turn_only: bool = False,
        distance_controlled_motion: bool = False,
        course_model=None,
    ) -> None:
        self.grid = grid or OccupancyGrid()
        self.max_range_m = max_range_m
        self.unobserved_clear_range_m = min(max_range_m, max(0.0, unobserved_clear_range_m))
        self.prefer_forward_exploration = bool(prefer_forward_exploration)
        self.forward_only = bool(forward_only)
        self.local_probe_after_two_scans = bool(local_probe_after_two_scans)
        self.rotation_enabled = bool(rotation_enabled)
        self.immediate_navigation = bool(immediate_navigation)
        self.prioritize_unexplored_gaps = bool(prioritize_unexplored_gaps)
        # In this hardware mode, a fresh complete radar circle chooses the
        # direction.  Mapping remains only a confidence aid, never a route.
        self.radar_gap_steering = bool(radar_gap_steering)
        self.forward_turn_only = bool(forward_turn_only)
        self.distance_controlled_motion = bool(distance_controlled_motion)
        self._detour_heading = None
        self._gap_world_heading = None
        self._gap_origin = None
        self._radar_gap_world_heading = None
        self._detour_origin = None
        self._stationary_scan_attempts = 0
        self.course_model = course_model
        self.min_range_m = min_range_m
        self.robot_radius_m = robot_radius_m
        self.safety_clearance_m = safety_clearance_m
        self.safety_stop_distance_m = safety_stop_distance_m
        self.safety_max_observation_age_s = safety_max_observation_age_s
        self.safety_speed_upper_bound_mps = safety_speed_upper_bound_mps
        self.recovery_requested = False
        self.sensor_offset_x_m = sensor_offset_x_m
        self.sensor_offset_y_m = sensor_offset_y_m
        self.sensor_offset_yaw_rad = sensor_offset_yaw_rad
        self.path_turn_penalty = max(0.0, float(path_turn_penalty))
        self.translation_capabilities = self._normalize_translation_capabilities(translation_capabilities)
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
        self.local_map_updates = 0
        self.match_score = 0.0
        self.last_match_result: ScanMatchResult | None = None
        self.match_failures = 0
        self.rejected_scans = 0
        self.rejection_reason_counts: dict[str, int] = {}
        self.mapping_attempts = 0
        self._predicted_travel_m = 0.0
        self._predicted_strafe_m = 0.0
        self._predicted_motion_uncertainty_m = 0.0
        self._predicted_rotation_rad = 0.0
        self._predicted_motion_uncertainty_rad = 0.0
        self._diagonal_motion_since_last_scan = False
        self._last_scan_had_diagonal_motion = False
        self._motion_since_last_scan = False
        self._last_scan_had_motion = False
        self._translation_since_last_scan = False
        self._last_scan_had_translation = False
        self._map_initialized = self.grid.update_count >= self.BOOTSTRAP_SCANS and len(self.grid.occupied_cells()) >= self.matcher.MIN_HIT_POINTS
        self._empty_frontier_scans = 0
        self._terminal_evidence_scans = 0
        self._terminal_signature: tuple[float, ...] | None = None
        self._no_route_turns = 0
        self._parking_search_attempts = 0
        self._parking_goal: tuple[int, int] | None = None
        self._last_motion_rejection_reason = ""
        self.last_progress_angle_world = 0.0
        self.trajectory: list[tuple[float, float]] = [(0.0, 0.0)]

    @classmethod
    def _normalize_translation_capabilities(
        cls,
        capabilities: Mapping[str, Mapping[str, object]] | None,
    ) -> dict[str, dict[str, float | bool]]:
        if capabilities is None:
            return {
                mode: {"enabled": True, "min_m": 0.0, "max_m": cls.MAX_MOTION_SEGMENT_M}
                for mode in cls.TRANSLATION_MODES
            }
        normalized: dict[str, dict[str, float | bool]] = {}
        for mode in cls.TRANSLATION_MODES:
            raw = capabilities.get(mode)
            if not isinstance(raw, Mapping):
                normalized[mode] = {"enabled": False, "min_m": 0.0, "max_m": 0.0}
                continue
            enabled = raw.get("enabled", False)
            if not isinstance(enabled, bool):
                raise ValueError(f"底盘 {mode} 方向 enabled 必须是布尔值")
            minimum = raw.get("min_m", raw.get("min_distance_m", 0.0))
            maximum = raw.get("max_m", raw.get("max_distance_m", cls.MAX_MOTION_SEGMENT_M))
            try:
                minimum = float(minimum)
                maximum = float(maximum)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError(f"底盘 {mode} 方向距离范围必须是有限数值") from exc
            if not all(math.isfinite(value) for value in (minimum, maximum)):
                raise ValueError(f"底盘 {mode} 方向距离范围必须是有限数值")
            if minimum < 0 or maximum < minimum or minimum > cls.MAX_MOTION_SEGMENT_M:
                raise ValueError(f"底盘 {mode} 方向距离范围无效")
            normalized[mode] = {
                "enabled": enabled,
                "min_m": minimum,
                "max_m": min(maximum, cls.MAX_MOTION_SEGMENT_M),
            }
        return normalized

    def reset(self) -> None:
        self.recovery_requested = False
        self._detour_heading = self._detour_origin = None
        self._gap_world_heading = self._gap_origin = None
        self._radar_gap_world_heading = None
        self._stationary_scan_attempts = 0
        if self.course_model is not None:
            self.course_model.reset()
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
        self.local_map_updates = 0
        self.match_score = 0.0
        self.last_match_result = None
        self.match_failures = 0
        self.rejected_scans = 0
        self.rejection_reason_counts = {}
        self.mapping_attempts = 0
        self._predicted_travel_m = 0.0
        self._predicted_strafe_m = 0.0
        self._predicted_motion_uncertainty_m = 0.0
        self._predicted_rotation_rad = 0.0
        self._predicted_motion_uncertainty_rad = 0.0
        self._diagonal_motion_since_last_scan = False
        self._last_scan_had_diagonal_motion = False
        self._motion_since_last_scan = False
        self._last_scan_had_motion = False
        self._translation_since_last_scan = False
        self._last_scan_had_translation = False
        self._map_initialized = False
        self._empty_frontier_scans = 0
        self._terminal_evidence_scans = 0
        self._terminal_signature = None
        self._no_route_turns = 0
        self._parking_search_attempts = 0
        self._parking_goal = None
        self._last_motion_rejection_reason = ""
        self.last_progress_angle_world = 0.0
        self.trajectory = [(0.0, 0.0)]

    def set_auto(self, enabled: bool) -> None:
        self.recovery_requested = False
        self._detour_heading = self._detour_origin = None
        self._gap_world_heading = self._gap_origin = None
        self._radar_gap_world_heading = None
        self._stationary_scan_attempts = 0
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
        self._motion_since_last_scan = True
        self._stationary_scan_attempts = 0
        local_x = command.right_mps * command.duration_s
        local_y = command.forward_mps * command.duration_s
        diagonal = abs(local_x) > 1e-6 and abs(local_y) > 1e-6
        self._diagonal_motion_since_last_scan = self._diagonal_motion_since_last_scan or diagonal
        if diagonal:
            self._predicted_motion_uncertainty_m += self.DIAGONAL_POSE_UNCERTAINTY_M
        self._translation_since_last_scan = self._translation_since_last_scan or math.hypot(local_x, local_y) > 1e-6
        self._predicted_travel_m += math.hypot(local_x, local_y)
        self._predicted_strafe_m += abs(local_x)
        self._predicted_rotation_rad += abs(command.yaw_rps * command.duration_s)
        world_x, world_y = self.pose.local_to_world(local_x, local_y)
        self.pose.x = world_x
        self.pose.y = world_y
        self.pose.yaw = wrap_angle(self.pose.yaw + command.yaw_rps * command.duration_s)
        if math.hypot(self.pose.x - self.trajectory[-1][0], self.pose.y - self.trajectory[-1][1]) >= 0.04:
            self.trajectory.append((self.pose.x, self.pose.y))

    def apply_execution_delta(
        self,
        local_x_m: float,
        local_y_m: float,
        yaw_rad: float = 0.0,
        uncertainty_m: float = 0.0,
        uncertainty_rad: float = 0.0,
    ) -> None:
        """Apply one calibrated hardware execution result as a scan-match prior."""
        values = (local_x_m, local_y_m, yaw_rad, uncertainty_m, uncertainty_rad)
        if (not all(math.isfinite(float(value)) for value in values)
                or uncertainty_m < 0 or uncertainty_rad < 0):
            raise ValueError("执行位姿先验必须是有限数值")
        translation = math.hypot(local_x_m, local_y_m)
        if translation <= 1e-9 and abs(yaw_rad) <= 1e-9:
            return
        self._motion_since_last_scan = True
        self._stationary_scan_attempts = 0
        diagonal = abs(local_x_m) > 1e-6 and abs(local_y_m) > 1e-6
        self._diagonal_motion_since_last_scan = self._diagonal_motion_since_last_scan or diagonal
        self._translation_since_last_scan = self._translation_since_last_scan or translation > 1e-6
        self._predicted_travel_m += translation
        self._predicted_strafe_m += abs(local_x_m)
        self._predicted_motion_uncertainty_m += max(0.0, uncertainty_m)
        self._predicted_rotation_rad += abs(yaw_rad)
        self._predicted_motion_uncertainty_rad += max(0.0, uncertainty_rad)
        world_x, world_y = self.pose.local_to_world(local_x_m, local_y_m)
        self.pose.x = world_x
        self.pose.y = world_y
        self.pose.yaw = wrap_angle(self.pose.yaw + yaw_rad)
        if math.hypot(self.pose.x - self.trajectory[-1][0], self.pose.y - self.trajectory[-1][1]) >= 0.04:
            self.trajectory.append((self.pose.x, self.pose.y))

    def process_scan(self, points: Sequence[ScanPoint], *,
                     free_space_points: Sequence[ScanPoint] = (),
                     obstacle_points: Sequence[ScanPoint] = (),
                     previous_wall_layer: Sequence[ScanPoint] = (),
                     add_only: bool = False) -> VelocityCommand:
        self._last_scan_had_motion = self._motion_since_last_scan
        self._last_scan_had_diagonal_motion = self._diagonal_motion_since_last_scan
        self._last_scan_had_translation = self._translation_since_last_scan
        if self.auto_enabled:
            self.mapping_attempts += 1
        valid = [
            point
            for point in points
            if math.isfinite(point.distance_m) and math.isfinite(point.angle_rad)
            and math.isfinite(point.quality) and point.quality >= self.grid.MIN_QUALITY
            and self.min_range_m <= point.distance_m <= self.max_range_m
        ]
        unique = {}
        for point in valid:
            key = round(point.angle_rad % math.tau, 6)
            previous = unique.get(key)
            if previous is None or (point.quality, -point.distance_m) > (previous.quality, -previous.distance_m):
                unique[key] = point
        valid = list(unique.values())
        self.latest_scan = valid + [point for point in obstacle_points
                                    if math.isfinite(point.distance_m) and math.isfinite(point.angle_rad)
                                    and math.isfinite(point.quality) and point.quality >= self.grid.MIN_QUALITY
                                    and self.min_range_m <= point.distance_m <= self.max_range_m
                                    and point.has_echo(self.max_range_m)]
        if len(valid) < 12:
            if self.auto_enabled and self.grid.update_count:
                return self._reject_scan(0.0, f"有效点仅 {len(valid)} 个，停车重扫")
            self.state = "扫描不足"
            self.detail = f"本圈只有 {len(valid)} 个有效点"
            return VelocityCommand()

        if not self.auto_enabled:
            self.completed_scans += 1
            self.state = "仅雷达"
            self.detail = f"已接收 {self.completed_scans} 圈"
            return VelocityCommand()

        matching_points = [p for p in valid if p.has_echo(self.max_range_m)]
        sectors = {int((p.angle_rad % math.tau) / (math.tau / 8)) for p in matching_points}
        if len(matching_points) < self.matcher.MIN_HIT_POINTS or len(sectors) < 3:
            return self._reject_scan(0.0, "有效障碍回波不足，等待重扫")
        initializing = not self._map_initialized
        if (self.grid.update_count == 0
                or (not self._map_initialized and not self.grid.occupied_cells()
                    and not self._last_scan_had_motion)):
            corrected_sensor = self._sensor_pose()
            match_result = ScanMatchResult(corrected_sensor, 1.0, inlier_count=len(matching_points),
                                           valid_direction_count=len(sectors), known_overlap=1.0)
            corrected, score = self._body_pose_from_sensor(corrected_sensor), 1.0
        else:
            predicted_sensor = self._sensor_pose()
            common_scale = 0.6 + max(0.0, 0.85 - self.match_score) + self.match_failures * 0.15
            translation_scale = min(
                1.5,
                common_scale + self._predicted_travel_m * 2 + self._predicted_strafe_m * 3
                + self._predicted_motion_uncertainty_m * 6,
            )
            rotation_scale = min(
                1.5,
                common_scale + self._predicted_rotation_rad / max(self.matcher.rotation_window_rad, 1e-9)
                + self._predicted_motion_uncertainty_rad / max(self.matcher.rotation_window_rad, 1e-9),
            )
            match_result = self.matcher.match(
                self.grid,
                predicted_sensor,
                matching_points,
                window_scale=0.0 if initializing else 1.0,
                translation_window_scale=0.0 if initializing else translation_scale,
                rotation_window_scale=0.0 if initializing else rotation_scale,
                minimum_evidence=0.25 if initializing else 4.0)
            if isinstance(match_result, ScanMatchResult):
                corrected_sensor, score = match_result.corrected_sensor_pose, match_result.data_score
            else:
                corrected_sensor, score = match_result
                match_result = ScanMatchResult(corrected_sensor, score)
            self.last_match_result = match_result
            if not math.isfinite(score) or score < self.MAP_UPDATE_MIN_CONFIDENCE:
                return self._reject_scan(score, "本圈未写入地图，停车重扫")
            correction_distance = math.hypot(
                corrected_sensor.x - predicted_sensor.x,
                corrected_sensor.y - predicted_sensor.y,
            )
            correction_yaw = abs(wrap_angle(corrected_sensor.yaw - predicted_sensor.yaw))
            prior_aligned = (
                correction_distance <= max(2.0 * self.grid.resolution_m, 0.05) + self._predicted_motion_uncertainty_m
                and correction_yaw <= math.radians(3.0) + self._predicted_motion_uncertainty_rad
            )
            if not initializing and match_result.degenerate and not prior_aligned:
                return self._reject_scan(score, match_result.rejection_reason or "本圈几何定位不充分，停车重扫")
            corrected = self._body_pose_from_sensor(corrected_sensor)
        if self.course_model is not None:
            corrected = self.course_model.fuse(
                corrected, obstacle_points or matching_points, self.start_pose,
                (self.sensor_offset_x_m, self.sensor_offset_y_m, self.sensor_offset_yaw_rad),
                self.max_range_m,
            )
        self.last_match_result = match_result
        self.completed_scans += 1
        self._motion_since_last_scan = False
        self._diagonal_motion_since_last_scan = False
        self._translation_since_last_scan = False
        self.pose = corrected
        self.match_score = score
        self.match_failures = 0
        self._predicted_travel_m = self._predicted_strafe_m = 0.0
        self._predicted_motion_uncertainty_m = 0.0
        self._predicted_rotation_rad = 0.0
        self._predicted_motion_uncertainty_rad = 0.0
        if previous_wall_layer:
            self.grid.update_scan(self._sensor_pose(), previous_wall_layer, self.max_range_m,
                                  scan_confidence=score, add_only=add_only)
        summary = self.grid.update_scan(self._sensor_pose(), valid + list(free_space_points),
                                        self.max_range_m, scan_confidence=score, add_only=add_only)
        if summary.out_of_bounds_points:
            self.detail = f"本圈有 {summary.out_of_bounds_points} 个回波超出地图范围"
        if initializing:
            self._map_initialized = (self.grid.update_count >= self.BOOTSTRAP_SCANS
                                     and len(self.grid.occupied_cells()) >= self.matcher.MIN_HIT_POINTS)
        if not self._map_initialized and not self.radar_gap_steering:
            self.state = "建图初始化"
            self.detail = "停车复测初始环境"
            return VelocityCommand()

        return self._plan_next_command()

    def process_local_scan(
        self,
        points: Sequence[ScanPoint],
        min_range_m: float = 0.08,
        scan_confidence: float = 0.7,
    ) -> VelocityCommand:
        valid = []
        unique = {}
        for point in points:
            if (math.isfinite(point.distance_m) and math.isfinite(point.angle_rad)
                    and math.isfinite(point.quality) and point.quality >= self.grid.MIN_QUALITY
                    and min_range_m <= point.distance_m <= self.max_range_m
                    and point.has_echo(self.max_range_m)):
                key = round(point.angle_rad % math.tau, 6)
                previous = unique.get(key)
                if previous is None or (point.quality, -point.distance_m) > (previous.quality, -previous.distance_m):
                    unique[key] = point
        valid = list(unique.values())
        self.latest_scan = valid
        if not valid:
            self.state = "本圈无有效回波"
            self.detail = "未收到可用于局部建图的回波"
            return VelocityCommand()
        before = self.grid.update_count
        sensor_pose = self._sensor_pose()
        summary = self.grid.update_scan(sensor_pose, valid, self.max_range_m, min_range_m=min_range_m,
                                       scan_confidence=scan_confidence)
        if self.grid.update_count > before:
            self.local_map_updates += 1
            self.state = "本圈局部回波"
            detail = f"已写入 {len(valid)} 个固定姿态回波"
            if summary.out_of_bounds_points:
                detail += f"，{summary.out_of_bounds_points} 个点超出地图"
            self.detail = detail
        return VelocityCommand()

    def _sensor_pose(self) -> Pose2D:
        x, y = self.pose.local_to_world(self.sensor_offset_x_m, self.sensor_offset_y_m)
        return Pose2D(x, y, wrap_angle(self.pose.yaw + self.sensor_offset_yaw_rad))

    def _body_pose_from_sensor(self, sensor_pose: Pose2D) -> Pose2D:
        body_yaw = wrap_angle(sensor_pose.yaw - self.sensor_offset_yaw_rad)
        cosine = math.cos(body_yaw)
        sine = math.sin(body_yaw)
        offset_x = self.sensor_offset_x_m * cosine + self.sensor_offset_y_m * sine
        offset_y = -self.sensor_offset_x_m * sine + self.sensor_offset_y_m * cosine
        return Pose2D(sensor_pose.x - offset_x, sensor_pose.y - offset_y, body_yaw)

    def _sensor_to_body(self, local_x: float, local_y: float) -> tuple[float, float]:
        yaw = self.sensor_offset_yaw_rad
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        body_x = local_x * cosine + local_y * sine
        body_y = -local_x * sine + local_y * cosine
        return body_x + self.sensor_offset_x_m, body_y + self.sensor_offset_y_m
    def _reject_scan(self, score: float, detail: str) -> VelocityCommand:
        self.match_score = score if math.isfinite(score) else 0.0
        self.match_failures += 1
        self.rejected_scans += 1
        self.rejection_reason_counts[detail] = self.rejection_reason_counts.get(detail, 0) + 1
        self.state = "定位丢失" if self.match_failures >= self.LOST_AFTER_FAILURES else "定位不可信"
        self.detail = detail
        self.path_cells.clear()
        self.target_cell = None
        if self.auto_enabled and self.radar_gap_steering:
            # A poor global match must not replace a fresh radar picture with
            # a stale map route.  It only reduces the translation length.
            command = self._radar_gap_command(map_confident=False)
            if not command.stopped:
                self.completed_scans += 1
                return command
        if self._two_scan_due():
            command = self._two_scan_action()
            if not command.stopped:
                self.completed_scans += 1
            return command
        if (self.auto_enabled and (
                (self.immediate_navigation and self._map_initialized)
                or (self.local_probe_after_two_scans
                    and max(self.match_failures, self._stationary_scan_attempts) >= 2))):
            command = self._fresh_local_probe()
            if not command.stopped:
                # A usable local scan does not imply a successful global pose match.
                self.completed_scans += 1
                self.match_failures = 0
                self.state = "局部短步探索"
                self.detail = "按当前完整扫描的空闲空间直接短步移动；本圈未写入全局地图"
                return command
        return VelocityCommand()

    def _fresh_local_probe(self, *, two_scan_action: bool = False) -> VelocityCommand:
        from mapping_policy import complete_open_scan
        if len(self.latest_scan) < 12:
            return VelocityCommand()
        size = max(80, math.ceil((self.max_range_m + self.robot_radius_m + .2) * 2 / self.grid.resolution_m))
        local = NavigationEngine(OccupancyGrid(size, size, self.grid.resolution_m),
                                 max_range_m=self.max_range_m, robot_radius_m=self.robot_radius_m,
                                 sensor_offset_x_m=self.sensor_offset_x_m, sensor_offset_y_m=self.sensor_offset_y_m,
                                 sensor_offset_yaw_rad=self.sensor_offset_yaw_rad,
                                 translation_capabilities=self.translation_capabilities,
                                 safety_clearance_m=self.safety_clearance_m,
                                 safety_stop_distance_m=self.safety_stop_distance_m,
                                 safety_max_observation_age_s=self.safety_max_observation_age_s,
                                 safety_speed_upper_bound_mps=self.safety_speed_upper_bound_mps,
                                 distance_controlled_motion=self.distance_controlled_motion,
                                 prioritize_unexplored_gaps=self.prioritize_unexplored_gaps,
                                 rotation_enabled=self.rotation_enabled,
                                 forward_only=True)
        local.start_pose.yaw = wrap_angle(self.start_pose.yaw - self.pose.yaw)
        local.latest_scan = list(complete_open_scan(self.latest_scan, self.unobserved_clear_range_m,
                                                   self.max_range_m, self.min_range_m, self.grid.resolution_m,
                                                   front_angle_rad=-self.sensor_offset_yaw_rad))
        # Local range geometry is fresh even if global localization is ambiguous.
        # This confidence controls probe length only; it is never committed.
        local.match_score = .8
        local.grid.update_scan(local._sensor_pose(), local.latest_scan, local.max_range_m, add_only=True)
        # This disposable grid represents current ray visibility, not accumulated
        # wall confidence. Unknown cells remain unknown; raw hits stay blocked.
        for row in range(size):
            for col in range(size):
                value = local.grid.value(col, row)
                if value <= -.25:
                    local.grid._add(col, row, -2)
        if two_scan_action:
            local.recovery_requested = self.recovery_requested
            if self._gap_world_heading is not None:
                local._gap_world_heading = wrap_angle(self._gap_world_heading - self.pose.yaw)
            command = local._two_scan_action(allow_local=False)
            self.state, self.detail = local.state, local.detail
            self.recovery_requested = local.recovery_requested
            return command
        if self.recovery_requested:
            local.recovery_requested = True
            command = local._recovery_command()
            if not command.stopped:
                self.recovery_requested = False
            return command
        gaps = local._unexplored_gap_candidates() if local.prioritize_unexplored_gaps else []
        if gaps:
            for gap in gaps:
                command = local._command_to_gap(gap)
                if not command.stopped:
                    return command
            return VelocityCommand()
        return local._largest_gap_command()

    def _two_scan_due(self) -> bool:
        return (self.auto_enabled and self.local_probe_after_two_scans
                and self._stationary_scan_attempts >= 2)

    def _two_scan_action(self, *, allow_local: bool = True) -> VelocityCommand:
        """Bound stationary deliberation; never bypass the physical sweep guard."""
        self._two_scan_action_attempted = True
        if self.recovery_requested:
            recovery = self._recovery_command()
            if not recovery.stopped:
                return recovery
        reasons = []
        for forward, right in ((1, 0), (1, 1), (1, -1), (0, 1), (0, -1)):
            norm = math.hypot(forward, right)
            command = VelocityCommand(.1 * forward / norm, .1 * right / norm, duration_s=1)
            guarded, reason = self._translation_guard(command, allow_backtracking=True)
            if not guarded.stopped:
                self.recovery_requested = False
                self.state = "两圈短步探索"
                self.detail = "已完成两圈扫描，向前方 180° 内可通行方向移动 10 cm"
                self.path_cells = self._motion_cells(guarded)
                self.target_cell = self.path_cells[-1]
                return guarded
            reasons.append(reason)
        sign = 1 if self._gap_world_heading is None or wrap_angle(self._gap_world_heading-self.pose.yaw) >= 0 else -1
        for direction in (sign, -sign):
            command = self._safe_rotation(direction * math.radians(20))
            if not command.stopped:
                self.recovery_requested = False
                self.state = "两圈转向探索"
                self.detail = "平移受限，执行已检查的 20° 旋转后重新扫描"
                return command
        self.recovery_requested = True
        command = self._recovery_command()
        if not command.stopped:
            return command
        self.recovery_requested = False
        self.state = "两圈动作均受阻"
        self.detail = "；".join(dict.fromkeys(reason for reason in reasons if reason))
        if allow_local:
            # Localization or accumulated grid uncertainty must not veto a
            # fully checked action in the current robot-relative scan.
            return self._fresh_local_probe(two_scan_action=True)
        return VelocityCommand()

    def _plan_next_command(self) -> VelocityCommand:
        if self.radar_gap_steering:
            # The legacy mapping/path planner remains for other runtime modes,
            # but cannot select a direction in direct radar-gap control.
            return self._radar_gap_command(
                map_confident=self.match_score >= self.MAP_UPDATE_MIN_CONFIDENCE,
            )
        if self._two_scan_due():
            return self._two_scan_action()
        if self.recovery_requested:
            return self._recovery_command()
        gaps = self._unexplored_gap_candidates() if self.prioritize_unexplored_gaps else []
        if gaps:
            self._terminal_evidence_scans = self._empty_frontier_scans = 0
            self._terminal_signature = self._parking_goal = None
            self._detour_heading = self._detour_origin = None
            if (self._gap_origin is not None
                    and math.dist(self._gap_origin, (self.pose.x, self.pose.y)) >= .15):
                self._gap_world_heading = self._gap_origin = None
            if self._gap_world_heading is not None:
                compatible = [gap for gap in gaps if abs(wrap_angle(
                    self.pose.yaw + gap[1] - self._gap_world_heading)) <= math.radians(35)]
                if compatible:
                    preferred = min(compatible, key=lambda gap: abs(wrap_angle(
                        self.pose.yaw + gap[1] - self._gap_world_heading)))
                    gaps = [preferred] + [gap for gap in gaps if gap != preferred]
            for gap in gaps:
                command = self._command_to_gap(gap)
                if not command.stopped:
                    if (self._gap_world_heading is None or abs(wrap_angle(
                            self.pose.yaw + gap[1] - self._gap_world_heading)) > math.radians(35)):
                        self._gap_world_heading = wrap_angle(self.pose.yaw + gap[1])
                        self._gap_origin = (self.pose.x, self.pose.y)
                    return command
            self.recovery_requested = True
            escape = self._recovery_command()
            if not escape.stopped:
                return escape
            self.recovery_requested = False
            self.state = "缺口待通行"
            self.detail = "发现大于 40 cm 的未探索缺口，当前动作空间不足；不判定停车完成"
            return VelocityCommand()
        self._gap_world_heading = self._gap_origin = None
        if self.rotation_enabled and self._detour_heading is not None:
            command = self._continue_detour()
            if not command.stopped:
                return command
        start = self.grid.world_to_cell(self.pose.x, self.pose.y)

        if self.course_model is not None and self._terminal_geometry_confirmed():
            self._terminal_evidence_scans += 1
            self.path_cells.clear()
            self.target_cell = start
            if self._terminal_evidence_scans >= 3:
                self._parking_goal = start
                self.state = "泊车完成"
                self.detail = "已确认到达终点车位"
            else:
                self.state = "终点确认"
                self.detail = "车位内固定姿态复测"
            return VelocityCommand()

        if self.prefer_forward_exploration:
            forward = self._forward_exploration_command()
            if not forward.stopped:
                self._empty_frontier_scans = 0
                self._terminal_evidence_scans = 0
                self._terminal_signature = None
                self._no_route_turns = 0
                self._parking_search_attempts = 0
                self._parking_goal = None
                self.path_cells = self._motion_cells(forward)
                self.target_cell = self.path_cells[-1]
                self.last_progress_angle_world = self.pose.yaw
                self.state = "直行探索"
                self.detail = f"前方可通行，优先直行 {self._command_distance(forward):.2f} m，停车后重扫"
                return forward

            turn = self._start_detour()
            if not turn.stopped:
                return turn

        parking = self._course_parking_command()
        if not parking.stopped:
            return parking

        clusters = self.grid.frontier_clusters()
        self.frontier_count = len(clusters)
        self.reachable_frontier_count = 0
        if clusters:
            candidates: list[tuple[float, list[tuple[int, int]], tuple[int, int]]] = []
            clearance = self.robot_radius_m + 0.02
            blocked = self.grid.inflated_obstacles(clearance)
            route_start = self.grid.world_to_cell(self.start_pose.x, self.start_pose.y)
            reachable, parents = self.grid.reachable_tree(start, blocked, self._planning_steps())
            route_distances, _ = self.grid.reachable_tree(route_start, blocked)
            current_progress = route_distances.get(start, 0.0)
            for cluster in clusters:
                goal = self._reachable_frontier_goal(cluster, blocked, set(reachable))
                if goal is None:
                    continue
                path = self.grid.path_from_tree(parents, goal)
                if len(path) < 2:
                    continue
                if not self.forward_only and not self._is_forward_path(path, route_distances, current_progress):
                    continue
                self.reachable_frontier_count += 1
                progress = route_distances.get(goal, 0.0)
                score = progress * 1.5 + min(40, len(cluster)) * 0.25 - len(path) * 0.20
                candidates.append((score, path, goal))
            for _, path, goal in sorted(candidates, key=lambda item: item[0], reverse=True):
                self._empty_frontier_scans = 0
                self._terminal_evidence_scans = 0
                self._terminal_signature = None
                self._no_route_turns = 0
                self._parking_search_attempts = 0
                self._parking_goal = None
                self.target_cell = goal
                straight_path = self.grid.astar(
                    start,
                    self.target_cell,
                    clearance,
                    blocked=blocked,
                    turn_penalty=self.path_turn_penalty,
                    initial_step=self._preferred_grid_step(),
                    allowed_steps=self._planning_steps(),
                )
                if straight_path and (self.forward_only or self._is_forward_path(straight_path, route_distances, current_progress)):
                    path = straight_path
                self.path_cells = self._compress_straight_runs(path)
                self.state = "探索中"
                self.detail = f"沿单向通道前进，可达前沿 {self.reachable_frontier_count} 个"
                command = self._command_along_path()
                if not command.stopped:
                    return command

        self.path_cells.clear()
        self.target_cell = None

        probe = self._corridor_probe_command()
        if not probe.stopped:
            self._empty_frontier_scans = 0
            self._terminal_evidence_scans = 0
            self._terminal_signature = None
            self._no_route_turns = 0
            self._parking_search_attempts = 0
            self.path_cells = self._motion_cells(probe)
            self.target_cell = self.path_cells[-1]
            self.state = "空隙探索"
            self.detail = "沿可通行空隙短距离移动，停车后重扫"
            return probe

        if self._parking_search_attempts < self.PARKING_SEARCH_MAX_ATTEMPTS:
            search = self._parking_search_command()
            if not search.stopped:
                self._parking_search_attempts += 1
                self._empty_frontier_scans = 0
                self._terminal_evidence_scans = 0
                self._terminal_signature = None
                self.path_cells.clear()
                self.target_cell = None
                self.state = "终点确认"
                self.detail = "向停车位内侧微移后复测"
                return search

        self._empty_frontier_scans += 1
        if self._terminal_geometry_confirmed():
            self._terminal_evidence_scans += 1
        else:
            self._terminal_evidence_scans = 0
            self._terminal_signature = None
        if self._empty_frontier_scans < 3 or self._terminal_evidence_scans < 3:
            if self._last_motion_rejection_reason:
                self.state = "无可执行安全动作"
                self.detail = self._last_motion_rejection_reason
            else:
                self.state = "终点确认"
                self.detail = "全向雷达固定姿态连续复测"
            return VelocityCommand()

        self._parking_goal = start
        self.path_cells = []
        self.target_cell = start
        self.state = "泊车完成"
        self.detail = "已确认到达单向通道另一端"
        return VelocityCommand()

    def _course_parking_command(self) -> VelocityCommand:
        if self.course_model is None or not self.course_model.parking_allowed(self.pose, self.start_pose):
            return VelocityCommand()
        start = self.grid.world_to_cell(self.pose.x, self.pose.y)
        blocked = self.grid.inflated_obstacles(self.robot_radius_m + 0.02)
        candidates = []
        centers = [self.start_pose.local_to_world(*center) for center in self.course_model.parking_centers()]
        if any(math.dist(world, (self.pose.x, self.pose.y)) < 0.10 for world in centers):
            return VelocityCommand()
        for world in centers:
            goal = self.grid.world_to_cell(*world)
            if self.grid.state(*goal) != self.grid.FREE:
                continue
            path = self.grid.astar(start, goal, self.robot_radius_m + 0.02, blocked=blocked,
                                   turn_penalty=self.path_turn_penalty,
                                   initial_step=self._preferred_grid_step(), allowed_steps=self._planning_steps())
            if len(path) >= 2:
                candidates.append((len(path), path, goal))
        for _, path, goal in sorted(candidates):
            self.path_cells = self._compress_straight_runs(path)
            self.target_cell = goal
            command = self._command_along_path()
            if not command.stopped:
                self.state = "驶入车位"
                self.detail = "沿已扫描通道驶入可达车位"
                return command
        self.path_cells.clear()
        self.target_cell = None
        return VelocityCommand()

    def _parking_search_command(self) -> VelocityCommand:
        if self.course_model is not None and not self.course_model.parking_allowed(self.pose, self.start_pose):
            return VelocityCommand()
        if self.match_score < 0.55 or len(self.latest_scan) < 12:
            return VelocityCommand()
        sectors = []
        for center in (0.0, math.pi / 2, -math.pi / 2, math.pi):
            distances = [
                point.distance_m
                for point in self.latest_scan
                if abs(wrap_angle(point.angle_rad - center)) <= math.radians(35)
            ]
            if len(distances) < 2:
                return VelocityCommand()
            sectors.append(min(distances))
        front, right, left, _ = sectors
        candidates = sorted(
            (distance, angle)
            for distance, angle in (
                (front, 0.0),
                (right, math.pi / 2),
                (left, -math.pi / 2),
            )
            if distance >= 0.30
        )
        if not candidates:
            return VelocityCommand()
        distance, angle = candidates[0]
        travel = min(self.PARKING_SEARCH_STEP_M, max(0.0, distance - 0.28))
        speed = self.PARKING_SEARCH_SPEED_MPS
        preview = VelocityCommand(
            forward_mps=speed * math.cos(angle),
            right_mps=speed * math.sin(angle),
            duration_s=1.0,
        )
        mode = self._translation_mode(preview)
        capability = self._capability(mode) if mode is not None else {"enabled": False, "min_m": 0.0, "max_m": 0.0}
        if not capability["enabled"]:
            self._last_motion_rejection_reason = f"{mode or '当前'} 方向尚未开放停车搜索"
            return VelocityCommand()
        travel = min(travel, float(capability["max_m"]))
        minimum = float(capability["min_m"])
        if travel < max(0.02, minimum) - 1e-9:
            self._last_motion_rejection_reason = (
                f"{mode} 方向停车搜索 {travel:.3f} m 小于已验证最小步长 {minimum:.3f} m"
            )
            return VelocityCommand()
        command = VelocityCommand(
            forward_mps=speed * math.cos(angle),
            right_mps=speed * math.sin(angle),
            duration_s=travel / speed,
        )
        return self._collision_guard(command)

    def _terminal_geometry_confirmed(self) -> bool:
        if self.prioritize_unexplored_gaps and self._unexplored_gap_candidates():
            return False
        if self.course_model is not None and not self.course_model.parking_allowed(self.pose, self.start_pose):
            return False
        if self._terminal_evidence_scans == 0 and not self._last_scan_had_translation:
            return False
        if self.match_score < 0.55 or len(self.latest_scan) < 12:
            return False
        approach = next((point for point in reversed(self.trajectory)
                         if math.dist(point, (self.pose.x, self.pose.y)) >= 0.15), None)
        if approach is None:
            return False
        dx, dy = self.pose.world_to_local(*approach)
        incoming = math.atan2(dx, dy)
        observed = [point for point in self.latest_scan
                    if point.source != "assumed_open"
                    and math.isfinite(point.angle_rad) and math.isfinite(point.distance_m)
                    and math.isfinite(point.quality) and point.quality >= self.grid.MIN_QUALITY
                    and self.min_range_m <= point.distance_m <= self.max_range_m]
        angles = sorted({point.angle_rad % math.tau for point in observed})
        if len(angles) < 12 or max((b - a) % math.tau for a, b in zip(angles, angles[1:] + angles[:1])) > math.radians(45):
            return False
        sectors = []
        for offset in (math.pi, math.pi / 2, -math.pi / 2, 0.0):
            center = incoming + offset
            points = [point for point in observed
                      if abs(wrap_angle(point.angle_rad + self.sensor_offset_yaw_rad - center)) <= math.radians(35)]
            if len(points) < 2:
                return False
            if offset != 0.0 and any(not point.has_echo(self.max_range_m) for point in points):
                return False
            sectors.append(max(point.distance_m for point in points) if offset != 0.0
                           else min(point.distance_m for point in points))
        front, right, left, rear = sectors
        close_side_count = sum(distance < 0.30 for distance in (front, right, left))
        open_limit = max(0.75, self.robot_radius_m + 0.45)
        if (close_side_count < 2 or rear < open_limit + 0.15
                or any(distance >= open_limit for distance in (front, right, left))):
            return False
        signature = tuple(sectors)
        if self._terminal_signature is not None and max(
                abs(a - b) for a, b in zip(signature, self._terminal_signature)) > 0.15:
            self._terminal_signature = signature
            self._terminal_evidence_scans = 0
            return False
        self._terminal_signature = signature
        return True

    def _radar_gap_candidates(self) -> list[tuple[float, float, float, float]]:
        """Return openings in the course-forward, absolute 180-degree half-plane.

        A missing bin is deliberately not treated as free: scan-completion
        validation happens upstream, and this additional rule makes a dropped
        group of measurements unable to create a fictitious opening.
        """
        observed = [
            point for point in self.latest_scan
            if (math.isfinite(point.angle_rad) and math.isfinite(point.distance_m)
                and math.isfinite(point.quality) and point.quality >= self.grid.MIN_QUALITY
                and self.min_range_m <= point.distance_m <= self.max_range_m)
        ]
        if len(observed) < 12:
            return []
        clearance = self.robot_radius_m + .035
        safe_distance = clearance + .10
        # ``start_pose.yaw`` is fixed when the vehicle is placed at the short
        # edge of the long rectangular course.  Do not centre the search on
        # ``pose.yaw``: that would redefine "forward" after every turn.
        bins: list[tuple[float, float] | None] = []
        for degrees in range(-90, 91, 5):
            absolute_offset = math.radians(degrees)
            target_world_heading = wrap_angle(self.start_pose.yaw + absolute_offset)
            target = wrap_angle(target_world_heading - self.pose.yaw)
            candidates = [
                point for point in observed
                if abs(wrap_angle(
                    math.atan2(*self._sensor_to_body(point.x, point.y)) - target,
                )) <= math.radians(13)
            ]
            if not candidates:
                bins.append(None)
                continue
            # Select the angularly closest return for this direction. A wall
            # just outside the bin is an opening edge, not evidence that the
            # bin centre itself is blocked.
            nearest = min(candidates, key=lambda point: (
                abs(wrap_angle(math.atan2(*self._sensor_to_body(point.x, point.y)) - target)),
                point.distance_m,
            ))
            body_x, body_y = self._sensor_to_body(nearest.x, nearest.y)
            heading = math.atan2(body_x, body_y)
            bins.append((absolute_offset, nearest.distance_m)
                        if nearest.distance_m > safe_distance else None)

        openings: list[tuple[float, float, float]] = []
        start = 0
        while start < len(bins):
            if bins[start] is None:
                start += 1
                continue
            end = start
            while end + 1 < len(bins) and bins[end + 1] is not None:
                end += 1
            run = [item for item in bins[start:end + 1] if item is not None]
            depth = min(item[1] for item in run)
            absolute_offset = sum(item[0] for item in run) / len(run)
            heading = wrap_angle(self.start_pose.yaw + absolute_offset - self.pose.yaw)
            nearest_forward_offset = min((item[0] for item in run), key=abs)
            # The opening width is measured at the nearest return, rather
            # than inferred from an old wall segment in the global map.
            span = math.radians(5 * (end - start + 1))
            width = 2 * depth * math.sin(min(math.pi / 2, span / 2))
            if width >= self.RADAR_GAP_MIN_WIDTH_M:
                openings.append((width, heading, depth, nearest_forward_offset))
            start = end + 1
        return openings

    def _radar_translation_guard(self, command: VelocityCommand) -> tuple[VelocityCommand, str | None]:
        """Check a proposed radar-guided move without consulting map routes."""
        if command.stopped:
            return command, None
        if self.forward_only and not self._normal_direction_allowed(command):
            return VelocityCommand(), "动作会朝起点方向回退"
        mode = self._translation_mode(command)
        if mode is None:
            return VelocityCommand(), "雷达缺口动作不是已标定的底盘方向"
        capability = self._capability(mode)
        distance = self._command_distance(command)
        if (not capability["enabled"]
                or distance + 1e-9 < float(capability["min_m"])
                or distance > float(capability["max_m"]) + 1e-9):
            return VelocityCommand(), f"{mode} 方向不支持该步长"
        swept = self._swept_body_cells(self._motion_cells(command))
        if any(not self.grid.in_bounds(*cell) for cell in swept):
            return VelocityCommand(), "雷达缺口动作会越出地图边界"
        if not self._command_has_clearance(command):
            return VelocityCommand(), "预测行程内有雷达墙体"
        return command, None

    def _safe_radar_rotation(self, angle: float) -> VelocityCommand:
        """Rotate from current radar clearance, without requiring mapped free cells."""
        if (not self.rotation_enabled
                or not math.radians(10) <= abs(angle) <= math.radians(25) + 1e-9
                or len(self.latest_scan) < 12):
            return VelocityCommand()
        clearance = self.robot_radius_m + (.035 if self.distance_controlled_motion else self.safety_clearance_m)
        for point in self.latest_scan:
            if not point.has_echo(self.max_range_m):
                continue
            body_x, body_y = self._sensor_to_body(point.x, point.y)
            margin = (point.distance_m * math.sin(min(math.pi / 2, max(0.0, point.angle_error_rad or 0.0)))
                      + max(0.0, point.distance_error_m or 0.0))
            if math.hypot(body_x, body_y) <= clearance + margin:
                return VelocityCommand()
        return VelocityCommand(yaw_rps=math.copysign(.25, angle), duration_s=abs(angle) / .25)

    def _radar_gap_command(self, *, map_confident: bool) -> VelocityCommand:
        """Commit to one front opening; abandon it only when this scan disproves it."""
        gaps = self._radar_gap_candidates()
        if not gaps:
            self._radar_gap_world_heading = None
            self.path_cells.clear()
            self.target_cell = None
            self.state = "前方无可信缺口"
            self.detail = "本圈雷达前方 180° 没有足够宽且预测无墙的缺口"
            return VelocityCommand()
        # Any gap covering the course centreline wins over a wider lateral
        # one.  Width only breaks ties within the same absolute-forward class.
        gaps = sorted(gaps, key=lambda gap: (
            abs(gap[3]) <= self.RADAR_GAP_DIRECT_TOLERANCE_RAD,
            gap[0], gap[2], -abs(gap[3]),
        ), reverse=True)

        if self._radar_gap_world_heading is not None:
            locked = [gap for gap in gaps if abs(wrap_angle(
                self.pose.yaw + gap[1] - self._radar_gap_world_heading,
            )) <= self.RADAR_GAP_LOCK_TOLERANCE_RAD]
            if locked:
                gaps = sorted(locked, key=lambda gap: abs(wrap_angle(
                    self.pose.yaw + gap[1] - self._radar_gap_world_heading,
                ))) + [gap for gap in gaps if gap not in locked]
            else:
                # The actual fresh circle no longer contains the chosen gap.
                self._radar_gap_world_heading = None

        for width, heading, depth, _ in gaps:
            world_heading = wrap_angle(self.pose.yaw + heading)
            turn_angle = wrap_angle(world_heading - self.pose.yaw)
            if abs(turn_angle) >= math.radians(10):
                turn = self._safe_radar_rotation(math.copysign(
                    min(abs(turn_angle), math.radians(25)), turn_angle,
                ))
                if turn.stopped:
                    continue
                self._radar_gap_world_heading = world_heading
                self.path_cells.clear()
                self.target_cell = None
                self.state = "对准雷达缺口"
                self.detail = f"锁定 {width:.2f} m 缺口，旋转 {abs(math.degrees(turn_angle)):.0f}° 后前进"
                return turn

            capability = self._capability("W")
            # A clear radar corridor takes the normal full 20 cm step.  Map
            # confidence is only a supporting signal; it must not turn a
            # physically clear path into needless short-step dithering.
            desired = .20
            distance = min(desired, float(capability["max_m"]))
            minimum = float(capability["min_m"])
            while distance + 1e-9 >= minimum:
                command = VelocityCommand(forward_mps=.10, duration_s=distance / .10)
                guarded, _ = self._radar_translation_guard(command)
                if not guarded.stopped:
                    self._radar_gap_world_heading = world_heading
                    self.path_cells = self._motion_cells(guarded)
                    self.target_cell = self.path_cells[-1]
                    confidence = "尚可" if map_confident else "偏低，仅作辅助判断"
                    self.state = "沿雷达缺口前进"
                    self.detail = (f"锁定 {width:.2f} m 前方缺口，预测路径无墙；"
                                   f"雷达图置信度{confidence}，前进 {distance:.2f} m")
                    return guarded
                if distance <= minimum + 1e-9:
                    break
                distance = max(minimum, distance - self.grid.resolution_m)

            # This candidate is physically contradicted by the new scan; try
            # another front gap instead of oscillating back to a stale target.
            if self._radar_gap_world_heading == world_heading:
                self._radar_gap_world_heading = None

        self._radar_gap_world_heading = None
        self.path_cells.clear()
        self.target_cell = None
        self.state = "雷达缺口受阻"
        self.detail = "候选缺口的预测行程有墙或底盘步长不支持，本圈不执行危险动作"
        return VelocityCommand()

    def _unexplored_gap_candidates(self):
        """Find >40 cm breaks between supported wall ends leading beyond known space."""
        from mapping_policy import prepare_mapping_points
        if len(self.latest_scan) < 12:
            return []
        fitted = prepare_mapping_points(self.latest_scan, self.max_range_m,
                                        self.min_range_m, self.grid.resolution_m).points
        points = sorted(fitted, key=lambda point: point.angle_rad % math.tau)
        if len(points) < 4:
            return []
        candidates = []
        for left, right in zip(points, points[1:] + points[:1]):
            arc = (right.angle_rad - left.angle_rad) % math.tau
            if arc < math.radians(12):
                continue
            width = math.hypot(right.x - left.x, right.y - left.y)
            if width <= .40 + 1e-9:
                continue
            angle = wrap_angle(left.angle_rad + arc / 2)
            depth = max(self.robot_radius_m + .10,
                        min(left.distance_m, right.distance_m) * max(.3, math.cos(min(math.pi / 2, arc / 2))))
            sx = math.sin(angle) * depth
            sy = math.cos(angle) * depth
            if arc < math.pi:
                # Aim at the physical opening midpoint: an angular bisector
                # skews toward the closer jamb when endpoint ranges differ.
                sx, sy = (left.x + right.x) / 2, (left.y + right.y) / 2
                depth = math.hypot(sx, sy)
            bx, by = self._sensor_to_body(sx, sy)
            heading = math.atan2(bx, by)
            world_heading = self.pose.yaw + heading
            unexplored = False
            blocked = False
            for extra in (.05, .15, .25, .35):
                distance = math.hypot(bx, by) + extra
                world = (self.pose.x + math.sin(world_heading) * distance,
                         self.pose.y + math.cos(world_heading) * distance)
                cell = self.grid.world_to_cell(*world)
                if not self.grid.in_bounds(*cell):
                    blocked = True
                    break
                state = self.grid.state(*cell)
                if state == self.grid.OCCUPIED:
                    blocked = True
                    break
                if state == self.grid.UNKNOWN or cell in self.grid.assumed_free_cells:
                    unexplored = True
            if unexplored and not blocked:
                candidates.append((width, heading, depth))
        return sorted(candidates, key=lambda item: (
            abs(item[1]) <= math.radians(25), item[0], -abs(item[1])), reverse=True)

    def _command_to_gap(self, gap) -> VelocityCommand:
        width, heading, _ = gap
        # A front opening need not be centred to sub-ten-degree precision.
        # Test the full forward sweep before spending another action on yaw;
        # the next scan will re-evaluate the approach after actual progress.
        if abs(heading) <= math.radians(25):
            capability = self._capability('W')
            distance = min(.20, float(capability['max_m']))
            if capability['enabled'] and distance >= max(.02, float(capability['min_m'])):
                straight = VelocityCommand(forward_mps=.1, duration_s=distance / .1)
                guarded, _ = self._translation_guard(straight, allow_backtracking=True)
                if not guarded.stopped:
                    self.state = "直行探索缺口"
                    self.detail = f"前方扫掠空间足够，向 {width:.2f} m 缺口直行 {distance:.2f} m"
                    self.path_cells = self._motion_cells(guarded)
                    self.target_cell = self.path_cells[-1]
                    return guarded
        if self.rotation_enabled and abs(heading) >= math.radians(10):
            # Re-evaluate the gap after every completed turn. Large heading
            # changes are made as several calibrated small rotations.
            angle = math.copysign(min(abs(heading), math.radians(25)), heading)
            turn = self._safe_rotation(angle)
            if not turn.stopped:
                self.state = "转向未探索缺口"
                self.detail = f"优先探索 {width:.2f} m 缺口，先{'右' if angle > 0 else '左'}转 {abs(math.degrees(angle)):.0f}°"
                self.path_cells.clear()
                self.target_cell = None
                return turn
        # Rotation is preferred; if unavailable/blocked, try a guarded
        # translation in the gap's direction instead of declaring parking.
        if abs(heading) < math.radians(10):
            forward, right = .1, 0.
        else:
            forward, right = self._cardinal_translation(math.cos(heading), math.sin(heading), .1)
        mode = self._translation_mode(VelocityCommand(forward, right, duration_s=1))
        capability = self._capability(mode)
        distance = min(.10, float(capability['max_m']))
        minimum = max(.02, float(capability['min_m']))
        while distance >= minimum - 1e-9:
            command = VelocityCommand(forward, right, duration_s=distance / .1)
            guarded, _ = self._translation_guard(command, allow_backtracking=True)
            if not guarded.stopped:
                self.state = "优先探索缺口"
                self.detail = f"沿 {width:.2f} m 未探索缺口前进 {distance:.2f} m"
                self.path_cells = self._motion_cells(command)
                self.target_cell = self.path_cells[-1]
                return command
            if distance <= minimum + 1e-9:
                break
            distance = max(minimum, distance - self.grid.resolution_m)
        return VelocityCommand()

    def _normal_direction_allowed(self, command: VelocityCommand) -> bool:
        angle = self.pose.yaw - self.start_pose.yaw
        progress = command.forward_mps * math.cos(angle) - command.right_mps * math.sin(angle)
        return command.forward_mps >= -1e-9 and progress >= -1e-9

    def _safe_rotation(self, angle: float) -> VelocityCommand:
        if not self.rotation_enabled or not math.radians(10) <= abs(angle) <= math.radians(25) + 1e-9:
            return VelocityCommand()
        if len(self.latest_scan) < 12:
            return VelocityCommand()
        center = self.grid.world_to_cell(self.pose.x, self.pose.y)
        cells = self._swept_body_cells([center])
        blocked = self.grid.inflated_obstacles(self.robot_radius_m + .035)
        if center in blocked or any(self.grid.state(*cell) != self.grid.FREE for cell in cells):
            return VelocityCommand()
        for point in self.latest_scan:
            if point.has_echo(self.max_range_m):
                x, y = self._sensor_to_body(point.x, point.y)
                margin = point.distance_m * math.sin(min(math.pi / 2, max(0, point.angle_error_rad or 0)))
                margin += max(0, point.distance_error_m or 0)
                clearance = .035 if self.distance_controlled_motion else max(.035, self.safety_clearance_m)
                if math.hypot(x, y) <= self.robot_radius_m + clearance + margin:
                    return VelocityCommand()
        return VelocityCommand(yaw_rps=math.copysign(.25, angle), duration_s=abs(angle) / .25)

    def _detour_forward(self) -> VelocityCommand:
        capability = self._capability('W')
        distance = min(.10, float(capability['max_m']))
        command = VelocityCommand(forward_mps=.1, duration_s=distance / .1)
        return self._translation_guard(command)[0]

    def _start_detour(self) -> VelocityCommand:
        if not self.rotation_enabled:
            return VelocityCommand()
        original_pose, original_scan = self.pose, self.latest_scan
        candidates = []
        for degrees in (15, -15, 20, -20, 25, -25):
            angle = math.radians(degrees)
            turn = self._safe_rotation(angle)
            if turn.stopped:
                continue
            # Check the forward leg in the candidate body frame, including the
            # sensor offset. Restore current geometry before returning a turn.
            try:
                self.pose = Pose2D(original_pose.x, original_pose.y, wrap_angle(original_pose.yaw + angle))
                rotated = []
                for point in original_scan:
                    bx, by = self._sensor_to_body(point.x, point.y)
                    x = bx * math.cos(angle) - by * math.sin(angle) - self.sensor_offset_x_m
                    y = bx * math.sin(angle) + by * math.cos(angle) - self.sensor_offset_y_m
                    rotated.append(replace(point, angle_rad=math.atan2(x, y) - self.sensor_offset_yaw_rad,
                                           distance_m=math.hypot(x, y)))
                self.latest_scan = rotated
                forward = self._detour_forward()
                distance = self._command_distance(forward)
                margin = min((math.hypot(p.x, p.y - min(distance, max(0, p.y)))
                              - max(0, p.distance_error_m or 0)
                              - p.distance_m * math.sin(min(math.pi / 2, max(0, p.angle_error_rad or 0)))
                              for p in rotated if p.has_echo(self.max_range_m) and p.y >= 0), default=self.max_range_m)
            finally:
                self.pose, self.latest_scan = original_pose, original_scan
            if not forward.stopped:
                candidates.append((margin, -abs(degrees), degrees, turn))
        if candidates:
            _, _, degrees, turn = max(candidates, key=lambda candidate: candidate[:3])
            self._detour_heading = original_pose.yaw
            self._detour_origin = (original_pose.x, original_pose.y)
            self.state = "小角度绕行"
            self.detail = f"向{'右' if degrees > 0 else '左'}转 {abs(degrees)}°，重扫后前进并回正"
            return turn
        return VelocityCommand()

    def _continue_detour(self) -> VelocityCommand:
        delta = wrap_angle(self._detour_heading - self.pose.yaw)
        traveled = math.dist(self._detour_origin, (self.pose.x, self.pose.y))
        if traveled < .05 and abs(delta) >= math.radians(8):
            forward = self._detour_forward()
            if not forward.stopped:
                self.state = "绕行前进"
                self.detail = "小角度转向后短步前进，随后回正"
                return forward
        if abs(delta) >= math.radians(10):
            turn = self._safe_rotation(math.copysign(min(abs(delta), math.radians(25)), delta))
            if not turn.stopped:
                self.state = "绕行回正"
                self.detail = "恢复绕行前朝向，停车后更新扫描"
                return turn
        self._detour_heading = self._detour_origin = None
        return VelocityCommand()

    def _planning_steps(self) -> tuple[tuple[int, int, float], ...]:
        if not self.forward_only:
            return GRID_STEPS
        return tuple(
            step for step in GRID_STEPS
            if step[0] * math.sin(self.start_pose.yaw) - step[1] * math.cos(self.start_pose.yaw) >= -1e-9
            and step[0] * math.sin(self.pose.yaw) - step[1] * math.cos(self.pose.yaw) >= -1e-9
        )

    def _largest_gap_command(self) -> VelocityCommand:
        observed = [
            point for point in self.latest_scan
            if math.isfinite(point.angle_rad) and math.isfinite(point.distance_m)
            and math.isfinite(point.quality) and point.quality >= self.grid.MIN_QUALITY
            and self.min_range_m <= point.distance_m <= self.max_range_m
        ]
        if len(observed) < 12:
            return VelocityCommand()
        ranges = []
        for degrees in range(-90, 91, 5):
            distances = [p.distance_m for p in observed
                         if abs(wrap_angle(p.angle_rad + self.sensor_offset_yaw_rad
                                           - math.radians(degrees))) <= math.radians(10)]
            # No echo is not a wall. The full grid/footprint guard below still
            # requires free space; an unobserved map cannot pass that guard.
            ranges.append(min(distances) if distances else self.max_range_m)
        threshold = (self.robot_radius_m + self.safety_clearance_m + .035 + .02
                     + self.safety_stop_distance_m
                     + max(.1, self.safety_speed_upper_bound_mps or 0) * self.safety_max_observation_age_s)
        candidates = []
        blocked = self.grid.inflated_obstacles(self.robot_radius_m + 0.02)
        for degrees in (-90, -45, 0, 45, 90):
            index = (degrees + 90) // 5
            if ranges[index] <= threshold:
                continue
            left = right = index
            while left > 0 and ranges[left - 1] > threshold:
                left -= 1
            while right + 1 < len(ranges) and ranges[right + 1] > threshold:
                right += 1
            depth = min(ranges[max(left, index - 3):min(right + 1, index + 4)])
            width = 2 * depth * math.sin(min(math.pi / 2, math.radians((right - left + 1) * 2.5)))
            angle = math.radians(degrees)
            velocity = VelocityCommand(0.1 * math.cos(angle), 0.1 * math.sin(angle), duration_s=1)
            if not self._normal_direction_allowed(velocity):
                continue
            capability = self._capability(self._translation_mode(velocity))
            maximum = min(0.10 if self.match_score >= 0.75 else 0.05, float(capability['max_m']))
            minimum = max(0.02, float(capability['min_m']))
            if not capability['enabled'] or maximum < minimum:
                continue
            distance = maximum
            while distance >= minimum - 1e-9:
                command = VelocityCommand(velocity.forward_mps, velocity.right_mps, duration_s=distance / 0.1)
                guarded, _ = self._translation_guard(command, blocked)
                if not guarded.stopped:
                    target = self.pose.local_to_world(command.right_mps * command.duration_s,
                                                      command.forward_mps * command.duration_s)
                    revisit = any(math.dist(target, point) < 0.06 for point in self.trajectory[:-1])
                    candidates.append((width - 0.5 * revisit, depth, math.cos(angle), command))
                    break
                if distance <= minimum + 1e-9:
                    break
                distance = max(minimum, distance - self.grid.resolution_m)
        if not candidates:
            return VelocityCommand()
        command = max(candidates, key=lambda item: item[:3])[3]
        heading = math.atan2(command.right_mps, command.forward_mps)
        if self.rotation_enabled and abs(heading) >= math.radians(10):
            turn = self._safe_rotation(math.copysign(min(abs(heading), math.radians(25)), heading))
            if not turn.stopped:
                self.state = "转向空隙"
                self.detail = "优先旋转对准可通行空隙，再向前移动"
                return turn
        return command

    def _forward_exploration_command(self) -> VelocityCommand:
        capability = self._capability("W")
        if not capability["enabled"]:
            return VelocityCommand()
        maximum = min(float(capability["max_m"]), self.MAX_MOTION_SEGMENT_M)
        if self.match_score < 0.75:
            maximum = min(maximum, 0.14 * 0.38)
        minimum = max(float(capability["min_m"]), min(0.05, maximum))
        if maximum <= 0 or maximum < minimum:
            return VelocityCommand()
        # Test only body-forward translations. A wider side opening must not
        # deflect a safe straight run. Every step keeps the normal full guard.
        blocked = self.grid.inflated_obstacles(self.robot_radius_m + 0.02)
        distance = maximum
        while True:
            command = VelocityCommand(forward_mps=0.14, duration_s=distance / 0.14)
            guarded, _ = self._translation_guard(command, blocked)
            if not guarded.stopped:
                return guarded
            if distance <= minimum + 1e-9:
                return VelocityCommand()
            distance = max(minimum, distance - self.grid.resolution_m)

    def _corridor_probe_command(self) -> VelocityCommand:
        if self.forward_only:
            return self._largest_gap_command()
        if not self.latest_scan:
            return VelocityCommand()
        self._last_motion_rejection_reason = ""
        previous_local = wrap_angle(self.last_progress_angle_world - self.pose.yaw)
        route_start = self.grid.world_to_cell(self.start_pose.x, self.start_pose.y)
        current_cell = self.grid.world_to_cell(self.pose.x, self.pose.y)
        blocked = self.grid.inflated_obstacles(self.robot_radius_m + 0.02)
        route_distances, _ = self.grid.reachable_tree(route_start, blocked)
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
            forward_hint = speed * math.cos(angle)
            right_hint = speed * math.sin(angle)
            commands: list[VelocityCommand] = []
            if self._is_diagonal_heading(forward_hint, right_hint):
                diagonal = self._safe_diagonal_command(
                    forward_hint,
                    right_hint,
                    blocked=blocked,
                    max_distance_m=self.DIAGONAL_PROBE_SEGMENT_M,
                )
                if diagonal is not None:
                    commands.append(diagonal)
            forward, right = self._cardinal_translation(forward_hint, right_hint, speed)
            mode = self._translation_mode(VelocityCommand(forward, right, 0.0, 1.0))
            capability = self._capability(mode) if mode is not None else {"max_m": 0.0}
            cardinal_distance = min(speed * 0.42, float(capability["max_m"]))
            if cardinal_distance > 1e-9:
                commands.append(VelocityCommand(forward, right, 0.0, cardinal_distance / speed))
            if not self._is_diagonal_heading(forward_hint, right_hint):
                diagonal = self._safe_diagonal_command(
                    forward_hint,
                    right_hint,
                    blocked=blocked,
                    max_distance_m=self.DIAGONAL_PROBE_SEGMENT_M,
                )
                if diagonal is not None:
                    commands.append(diagonal)
            for command in commands:
                guarded, _ = self._translation_guard(command, blocked)
                if guarded.stopped:
                    continue
                chosen_world = wrap_angle(
                    self.pose.yaw + math.atan2(guarded.right_mps, guarded.forward_mps)
                )
                if abs(wrap_angle(chosen_world - self.last_progress_angle_world)) >= math.radians(60):
                    self.last_progress_angle_world = chosen_world
                return guarded
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
            for dx, dy in CARDINAL_STEPS:
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
        self._last_motion_rejection_reason = ""
        lookahead_index = self._straight_run_end(self.path_cells)
        world_x, world_y = self.grid.cell_to_world(*self.path_cells[lookahead_index])
        local_x, local_y = self.pose.world_to_local(world_x, world_y)
        distance = math.hypot(local_x, local_y)
        if distance < 1e-6:
            return VelocityCommand()
        heading = math.atan2(local_x, local_y)
        if self.rotation_enabled and distance >= .06 and abs(heading) >= math.radians(10):
            turn = self._safe_rotation(math.copysign(min(abs(heading), math.radians(25)), heading))
            if not turn.stopped:
                self.state = "转向路径"
                self.detail = "优先旋转对准规划路径，随后前进"
                return turn
        speed = 0.14
        forward_hint = speed * local_y / distance
        right_hint = speed * local_x / distance
        blocked = self.grid.inflated_obstacles(self.robot_radius_m + 0.02)
        if self._is_diagonal_heading(forward_hint, right_hint):
            diagonal = self._safe_diagonal_command(
                forward_hint,
                right_hint,
                blocked=blocked,
                max_distance_m=distance,
            )
            if diagonal is not None:
                path_direction = wrap_angle(
                    self.pose.yaw + math.atan2(diagonal.right_mps, diagonal.forward_mps)
                )
                if abs(wrap_angle(path_direction - self.last_progress_angle_world)) >= math.radians(60):
                    self.last_progress_angle_world = path_direction
                return diagonal
        axis_distance = max(abs(local_y), abs(local_x))
        forward, right = self._cardinal_translation(forward_hint, right_hint, speed)
        mode = self._translation_mode(VelocityCommand(forward, right, 0.0, 1.0))
        if mode is None:
            return self._set_motion_blocked("规划结果无法分解为受支持的平移方向")
        capability = self._capability(mode)
        if not capability["enabled"]:
            return self._set_motion_blocked(f"{mode} 方向尚未开放")
        minimum = float(capability["min_m"])
        maximum = min(axis_distance, float(capability["max_m"]), self.MAX_MOTION_SEGMENT_M)
        short_travel = min(maximum, speed * 0.38)
        candidates: list[VelocityCommand] = []
        if self.match_score >= 0.75:
            travel = maximum
            while travel > short_travel + 0.01:
                if travel + 1e-9 >= minimum:
                    candidates.append(VelocityCommand(forward, right, 0.0, travel / speed))
                travel *= 0.7
        if short_travel + 1e-9 >= minimum and short_travel > 1e-9:
            candidates.append(VelocityCommand(forward, right, 0.0, short_travel / speed))
        elif self.match_score >= 0.75 and maximum + 1e-9 >= minimum and not candidates:
            candidates.append(VelocityCommand(forward, right, 0.0, maximum / speed))
        last_reason = None
        for candidate in candidates:
            guarded, last_reason = self._translation_guard(candidate, blocked)
            if not guarded.stopped:
                path_direction = wrap_angle(
                    self.pose.yaw + math.atan2(guarded.right_mps, guarded.forward_mps)
                )
                if abs(wrap_angle(path_direction - self.last_progress_angle_world)) >= math.radians(60):
                    self.last_progress_angle_world = path_direction
                return guarded
        if not candidates:
            last_reason = (
                f"{mode} 方向目标 {maximum:.3f} m 小于已验证最小步长 {minimum:.3f} m"
            )
            self._last_motion_rejection_reason = last_reason
        if candidates or self._is_diagonal_heading(forward_hint, right_hint):
            diagonal = self._safe_diagonal_command(forward_hint, right_hint, blocked=blocked)
            if diagonal is None:
                diagonal = self._safe_diagonal_command(
                    forward_hint,
                    right_hint,
                    blocked=blocked,
                    max_distance_m=self.DIAGONAL_RECOVERY_SEGMENT_M,
                )
            if diagonal is not None:
                self.state = "探索中"
                self.detail = "保守45°平移，下一圈重新定位"
                return diagonal
        if not self.forward_only and candidates and last_reason == "最新雷达回波显示车体扫过距离不足":
            self.recovery_requested = True
            return self._recovery_command()
        return self._set_motion_blocked(last_reason)

    @staticmethod
    def _compress_straight_runs(path: Sequence[tuple[int, int]]) -> list[tuple[int, int]]:
        if len(path) < 3:
            return list(path)
        result = [path[0]]
        previous_direction = (
            path[1][0] - path[0][0],
            path[1][1] - path[0][1],
        )
        for index in range(1, len(path) - 1):
            direction = (
                path[index + 1][0] - path[index][0],
                path[index + 1][1] - path[index][1],
            )
            if direction != previous_direction:
                result.append(path[index])
                previous_direction = direction
        result.append(path[-1])
        return result

    @staticmethod
    def _straight_run_end(path: Sequence[tuple[int, int]]) -> int:
        if len(path) < 2:
            return 0
        first_dx = path[1][0] - path[0][0]
        first_dy = path[1][1] - path[0][1]
        first_direction = (
            0 if first_dx == 0 else int(math.copysign(1, first_dx)),
            0 if first_dy == 0 else int(math.copysign(1, first_dy)),
        )
        for index in range(2, len(path)):
            dx = path[index][0] - path[index - 1][0]
            dy = path[index][1] - path[index - 1][1]
            direction = (
                0 if dx == 0 else int(math.copysign(1, dx)),
                0 if dy == 0 else int(math.copysign(1, dy)),
            )
            if direction != first_direction:
                return index - 1
        return len(path) - 1

    def _preferred_grid_step(self) -> tuple[int, int]:
        direction_x = math.sin(self.last_progress_angle_world)
        direction_y = -math.cos(self.last_progress_angle_world)
        dx, dy, _ = max(
            GRID_STEPS,
            key=lambda step: (step[0] * direction_x + step[1] * direction_y) / step[2],
        )
        return dx, dy

    @staticmethod
    def _cardinal_translation(forward_mps: float, right_mps: float, speed_mps: float) -> tuple[float, float]:
        if abs(forward_mps) < 1e-9 and abs(right_mps) < 1e-9:
            return 0.0, 0.0
        if abs(forward_mps) >= abs(right_mps):
            return math.copysign(speed_mps, forward_mps), 0.0
        return 0.0, math.copysign(speed_mps, right_mps)

    @staticmethod
    def _translation_mode(command: VelocityCommand) -> str | None:
        if command.stopped or abs(command.yaw_rps) > 1e-9:
            return None
        forward = command.forward_mps
        right = command.right_mps
        if abs(right) <= 1e-9:
            return "W" if forward > 0 else "S"
        if abs(forward) <= 1e-9:
            return "D" if right > 0 else "A"
        dominant = max(abs(forward), abs(right))
        if abs(abs(forward) - abs(right)) > dominant * 0.04:
            return None
        if forward > 0:
            return "E" if right > 0 else "Q"
        return "C" if right > 0 else "Z"

    def _capability(self, mode: str) -> Mapping[str, float | bool]:
        return self.translation_capabilities.get(
            mode,
            {"enabled": False, "min_m": 0.0, "max_m": 0.0},
        )

    @staticmethod
    def _command_distance(command: VelocityCommand) -> float:
        return math.hypot(command.forward_mps, command.right_mps) * command.duration_s

    def _motion_cells(self, command: VelocityCommand) -> list[tuple[int, int]]:
        target = self.pose.local_to_world(
            command.right_mps * command.duration_s,
            command.forward_mps * command.duration_s,
        )
        return self.grid._line_cells(
            self.grid.world_to_cell(self.pose.x, self.pose.y),
            self.grid.world_to_cell(*target),
        )

    def _swept_body_cells(self, center_cells: Sequence[tuple[int, int]]) -> set[tuple[int, int]]:
        radius_cells = max(1, math.ceil(self.robot_radius_m / self.grid.resolution_m))
        limit = self.robot_radius_m + self.grid.resolution_m * 0.5
        swept: set[tuple[int, int]] = set()
        for center_col, center_row in center_cells:
            for delta_row in range(-radius_cells, radius_cells + 1):
                for delta_col in range(-radius_cells, radius_cells + 1):
                    if math.hypot(delta_col, delta_row) * self.grid.resolution_m > limit:
                        continue
                    cell = center_col + delta_col, center_row + delta_row
                    swept.add(cell)
        return swept

    def _translation_guard(
        self,
        command: VelocityCommand,
        blocked: set[tuple[int, int]] | None = None,
        *, allow_backtracking: bool = False,
    ) -> tuple[VelocityCommand, str | None]:
        if command.stopped:
            return command, None
        if self.forward_only and not allow_backtracking and not self._normal_direction_allowed(command):
            reason = "正常导航仅允许向前或侧向移动"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        if abs(command.yaw_rps) > 1e-9:
            if abs(command.forward_mps) < 1e-9 and abs(command.right_mps) < 1e-9:
                turn = self._safe_rotation(command.yaw_rps * command.duration_s)
                if not turn.stopped:
                    return turn, None
            reason = "当前底盘能力未开放旋转动作"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        if (self.forward_turn_only and not command.recovery_translation
                and (command.forward_mps <= 1e-9 or abs(command.right_mps) > 1e-9)):
            reason = "正常导航仅允许前进；横移只允许用于紧急避障"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        mode = self._translation_mode(command)
        if mode is None:
            reason = "平移动作必须是轴向或等幅 45° 方向"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        capability = self._capability(mode)
        if not capability["enabled"]:
            reason = f"{mode} 方向尚未开放"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        distance = self._command_distance(command)
        minimum = float(capability["min_m"])
        maximum = float(capability["max_m"])
        if distance + 1e-9 < minimum:
            reason = f"{mode} 方向目标 {distance:.3f} m 小于已验证最小步长 {minimum:.3f} m"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        if distance > maximum + 1e-9:
            reason = f"{mode} 方向目标 {distance:.3f} m 超过当前上限 {maximum:.3f} m"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        blocked = self.grid.inflated_obstacles(self.robot_radius_m + 0.02) if blocked is None else blocked
        cells = self._motion_cells(command)
        swept_cells = self._swept_body_cells(cells)
        if any(not self.grid.in_bounds(*cell) for cell in swept_cells):
            reason = "地图边界导致无路"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        unknown = [cell for cell in swept_cells if self.grid.state(*cell) != self.grid.FREE]
        if unknown:
            reason = "动作扫过区域包含未知空间"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        if any(cell in blocked for cell in cells):
            reason = "动作扫过区域进入膨胀障碍边界"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        if len(self.latest_scan) < 12:
            reason = "最新完整雷达回波不足"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        if not self._command_has_clearance(command):
            reason = "最新雷达回波显示车体扫过距离不足"
            self._last_motion_rejection_reason = reason
            return VelocityCommand(), reason
        if not allow_backtracking and not self.forward_only:
            route_start = self.grid.world_to_cell(self.start_pose.x, self.start_pose.y)
            route_distances, _ = self.grid.reachable_tree(route_start, blocked)
            current = self.grid.world_to_cell(self.pose.x, self.pose.y)
            current_progress = route_distances.get(current)
            if current_progress is None or not self._is_forward_path(cells, route_distances, current_progress):
                reason = "动作不满足单向路线进度约束"
                self._last_motion_rejection_reason = reason
                return VelocityCommand(), reason
        self._last_motion_rejection_reason = ""
        return command, None

    def _set_motion_blocked(self, reason: str | None = None) -> VelocityCommand:
        self.state = "无可执行安全动作"
        self.detail = reason or self._last_motion_rejection_reason or "当前规划步长或路径不满足底盘能力与安全约束"
        return VelocityCommand()

    @classmethod
    def _is_diagonal_heading(cls, forward_mps: float, right_mps: float) -> bool:
        dominant = max(abs(forward_mps), abs(right_mps))
        minor = min(abs(forward_mps), abs(right_mps))
        return dominant > 1e-9 and minor / dominant >= cls.DIAGONAL_DIRECTION_RATIO

    def _diagonal_command_candidates(
        self,
        forward_hint: float,
        right_hint: float,
        max_distance_m: float | None = None,
    ) -> tuple[VelocityCommand, ...]:
        if abs(forward_hint) < 1e-9 and abs(right_hint) < 1e-9:
            return ()
        ranked: list[tuple[float, int, int]] = []
        for forward_sign in (-1, 1):
            for right_sign in (-1, 1):
                alignment = forward_sign * forward_hint + right_sign * right_hint
                if alignment > 0:
                    ranked.append((alignment, forward_sign, right_sign))
        ranked.sort(reverse=True)
        speed = self.DIAGONAL_MOTION_SPEED_MPS
        requested_distance = self.MAX_DIAGONAL_SEGMENT_M if max_distance_m is None else min(
            self.MAX_DIAGONAL_SEGMENT_M,
            max_distance_m,
        )
        component = speed / math.sqrt(2.0)
        commands = []
        for _, forward_sign, right_sign in ranked:
            mode = (
                "E" if forward_sign > 0 and right_sign > 0
                else "Q" if forward_sign > 0
                else "C" if right_sign > 0
                else "Z"
            )
            capability = self._capability(mode)
            if not capability["enabled"]:
                self._last_motion_rejection_reason = f"{mode} 方向尚未开放"
                continue
            distance = min(requested_distance, float(capability["max_m"]))
            minimum = float(capability["min_m"])
            if distance + 1e-9 < minimum:
                self._last_motion_rejection_reason = (
                    f"{mode} 方向目标 {distance:.3f} m 小于已验证最小步长 {minimum:.3f} m"
                )
                continue
            if distance <= 1e-9:
                self._last_motion_rejection_reason = f"{mode} 方向没有可执行的正距离"
                continue
            commands.append(VelocityCommand(
                component * forward_sign,
                component * right_sign,
                0.0,
                distance / speed,
            ))
        return tuple(commands)

    def _safe_diagonal_command(
        self,
        forward_hint: float,
        right_hint: float,
        *,
        blocked: set[tuple[int, int]] | None = None,
        max_distance_m: float | None = None,
    ) -> VelocityCommand | None:
        if len(self.latest_scan) < 12:
            self._last_motion_rejection_reason = "最新完整雷达回波不足"
            return None
        blocked = self.grid.inflated_obstacles(self.robot_radius_m + 0.02) if blocked is None else blocked
        for command in self._diagonal_command_candidates(forward_hint, right_hint, max_distance_m):
            guarded, _ = self._translation_guard(command, blocked)
            if not guarded.stopped:
                return guarded
        return None

    def _command_has_clearance(self, command: VelocityCommand) -> bool:
        if command.stopped:
            return True
        speed = math.hypot(command.forward_mps, command.right_mps)
        if speed < 1e-9:
            return True
        travel_angle = math.atan2(command.right_mps, command.forward_mps)
        travel_distance = speed * command.duration_s
        footprint_radius = self.robot_radius_m + 0.035
        stopping_buffer = (self.robot_radius_m + self.safety_clearance_m
                           + self.safety_stop_distance_m
                           + max(speed, self.safety_speed_upper_bound_mps or 0.0)
                           * self.safety_max_observation_age_s + 0.035)
        for point in self.latest_scan:
            if not point.has_echo(self.max_range_m):
                continue
            sensor_x, sensor_y = point.x, point.y
            body_x, body_y = self._sensor_to_body(sensor_x, sensor_y)
            along = body_y * math.cos(travel_angle) + body_x * math.sin(travel_angle)
            lateral = abs(body_x * math.cos(travel_angle) - body_y * math.sin(travel_angle))
            error = max(0.0, point.angle_error_rad or 0.0)
            margin = point.distance_m * math.sin(min(math.pi / 2, error)) + max(0.0, point.distance_error_m or 0.0)
            if self.distance_controlled_motion:
                # A distance-controlled MOVE is a finite swept disk, not a
                # velocity extrapolation plus a second full stopping envelope.
                measured_extra = (self.safety_stop_distance_m
                                  + (self.safety_speed_upper_bound_mps or 0) * self.safety_max_observation_age_s)
                closest = min(travel_distance + measured_extra, max(0.0, along))
                if (command.recovery_translation and travel_distance <= .05 + 1e-9
                        and along < 0 and math.hypot(along, lateral) > self.robot_radius_m + .005 + margin):
                    # Allow monotonically leaving only the comfort buffer;
                    # actual body clearance plus measurement error still holds.
                    continue
                if math.hypot(along - closest, lateral) < footprint_radius + margin:
                    return False
                continue
            if -margin <= along < travel_distance + stopping_buffer + margin and lateral < footprint_radius + margin:
                return False
        return True

    def request_obstacle_recovery(self, command: VelocityCommand, elapsed_s: float) -> None:
        """Replace the unexecuted part of the motion prior; require a new scan."""
        remaining = max(0.0, command.duration_s - max(0.0, elapsed_s))
        self.pose.x, self.pose.y = self.pose.local_to_world(
            -command.right_mps * remaining, -command.forward_mps * remaining)
        self._predicted_travel_m = max(0.0, self._predicted_travel_m - math.hypot(command.right_mps, command.forward_mps) * remaining)
        self._predicted_strafe_m = max(0.0, self._predicted_strafe_m - abs(command.right_mps) * remaining)
        self._predicted_motion_uncertainty_m += 0.03
        if self.trajectory:
            self.trajectory[-1] = (self.pose.x, self.pose.y)
        self.path_cells.clear()
        self.target_cell = None
        self.recovery_requested = True
        self.state = "避障重规划"
        self.detail = "已停止当前动作，等待新扫描后平移避让"

    def _recovery_command(self) -> VelocityCommand:
        if not self.recovery_requested:
            return VelocityCommand()
        echoes = [self._sensor_to_body(p.x, p.y) for p in self.latest_scan if p.has_echo(self.max_range_m)]
        if not echoes:
            return self._set_motion_blocked("等待新的障碍测距，暂不执行避让")
        obstacle_x, obstacle_y = min(echoes, key=lambda xy: math.hypot(*xy))
        initial = math.hypot(obstacle_x, obstacle_y)
        candidates = []
        directions = ((0, 1), (0, -1)) if self.forward_turn_only else (
            (1, 0), (0, 1), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        )
        for forward, right in directions:
            norm = math.hypot(forward, right)
            velocity = VelocityCommand(0.1 * forward / norm, 0.1 * right / norm, duration_s=1)
            capability = self._capability(self._translation_mode(velocity))
            distance = min(0.05, float(capability['max_m']))
            if not capability['enabled'] or distance < float(capability['min_m']):
                continue
            command = VelocityCommand(velocity.forward_mps, velocity.right_mps, duration_s=distance / 0.1,
                                      recovery_translation=True)
            guarded, _ = self._translation_guard(command, allow_backtracking=True)
            if guarded.stopped:
                continue
            dx, dy = command.right_mps * command.duration_s, command.forward_mps * command.duration_s
            end_clearance = min(math.hypot(x - dx, y - dy) for x, y in echoes)
            if end_clearance < initial - 0.005:
                continue
            # Prefer translation opposite the closest obstacle, not a forward
            # bias that can keep the robot alongside the same obstruction.
            away = -(dx * obstacle_x + dy * obstacle_y) / max(initial * distance, 1e-9)
            candidates.append((end_clearance - initial, away, command))
        if candidates:
            command = max(candidates, key=lambda item: (item[0], item[1]))[2]
            self.recovery_requested = False
            self.state = "平移避让"
            self.detail = "沿已扫描空闲区域短距离避让，随后重新规划"
            return command
        self.state = "等待避让空间"
        self.detail = "继续扫描，当前没有满足间距的平移动作"
        return VelocityCommand()

    def _collision_guard(self, command: VelocityCommand) -> VelocityCommand:
        if command.stopped:
            return command
        blocked = self.grid.inflated_obstacles(self.robot_radius_m + 0.02)
        guarded, reason = self._translation_guard(command, blocked)
        if guarded.stopped:
            self.state = "避障重规划"
            self.detail = reason or "车体扫过区域距离不足，停车更新障碍边界"
        return guarded

    def _grid_segment_is_clear(self, command: VelocityCommand, blocked: set[tuple[int, int]]) -> bool:
        if command.stopped:
            return True
        target = self.pose.local_to_world(
            command.right_mps * command.duration_s,
            command.forward_mps * command.duration_s,
        )
        cells = self.grid._line_cells(
            self.grid.world_to_cell(self.pose.x, self.pose.y),
            self.grid.world_to_cell(*target),
        )
        return all(self.grid.state(*cell) == self.grid.FREE and cell not in blocked for cell in cells)

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
            is_echo = true_distance < max_range_m - 1e-9
            if is_echo:
                distance = true_distance + self.distance_bias_m + self.random.gauss(0.0, point_noise_m)
                if self.random.random() < 0.012:
                    distance += self.random.uniform(-0.12, 0.12)
                distance = clamp(distance, 0.08, max_range_m)
            else:
                distance = max_range_m
            points.append(ScanPoint(angle, distance, 1.0, is_echo))
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

