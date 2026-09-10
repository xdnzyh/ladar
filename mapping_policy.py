from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from navigation_core import ScanPoint, VelocityCommand


DEFAULT_MAP_RESOLUTION_M = 0.02
MIN_WALL_POINTS = 6
MAX_WALL_RMS_M = 0.020
MIN_WALL_SPAN_M = 0.12
MAX_WALL_ANGLE_GAP_DEG = 22.0
MAX_WALL_NEIGHBOR_GAP_M = 0.16
MAX_WALL_POINT_GAP_M = 0.095
MIN_TURN_VECTOR_M = 0.035
WALL_CORNER_TURN_DEG = 35.0
MAX_COLLINEAR_MERGE_ANGLE_DEG = 10.0
MAX_COLLINEAR_MERGE_OFFSET_M = 0.035
MAX_COLLINEAR_MERGE_GAP_M = 0.20
MAX_CORNER_EXTENSION_M = 0.07
MAX_CORNER_ENDPOINT_GAP_M = 0.10


@dataclass(frozen=True)
class MappingPointSelection:
    points: tuple[ScanPoint, ...]
    raw_echoes: int
    supported_echoes: int
    noise_echoes: int
    fitted_segments: int = 0


@dataclass
class _EchoItem:
    point: ScanPoint
    angle: float
    x: float
    y: float
    weight: float


@dataclass
class _WallSegment:
    x: float
    y: float
    dx: float
    dy: float
    t_min: float
    t_max: float
    rms: float
    quality: float
    raw_count: int


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _point_quality(point: ScanPoint, resolution_m: float) -> float:
    try:
        return _clamp(float(point.evidence_weight(resolution_m)), 0.0, 1.0)
    except (TypeError, ValueError, OverflowError):
        return 0.0


def _fit_segment_unchecked(items: Sequence[_EchoItem]) -> _WallSegment | None:
    if len(items) < MIN_WALL_POINTS:
        return None
    total_weight = sum(max(item.weight, 1e-6) for item in items)
    mean_x = sum(item.x * max(item.weight, 1e-6) for item in items) / total_weight
    mean_y = sum(item.y * max(item.weight, 1e-6) for item in items) / total_weight
    sxx = syy = sxy = 0.0
    for item in items:
        weight = max(item.weight, 1e-6)
        x = item.x - mean_x
        y = item.y - mean_y
        sxx += weight * x * x
        syy += weight * y * y
        sxy += weight * x * y
    sxx /= total_weight
    syy /= total_weight
    sxy /= total_weight
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    dx, dy = math.cos(theta), math.sin(theta)
    nx, ny = -dy, dx
    projections = []
    error_sum = 0.0
    for item in items:
        px = item.x - mean_x
        py = item.y - mean_y
        t = px * dx + py * dy
        signed_error = px * nx + py * ny
        weight = max(item.weight, 1e-6)
        projections.append(t)
        error_sum += weight * signed_error * signed_error
    span = max(projections) - min(projections)
    if span < MIN_WALL_SPAN_M:
        return None
    projection_gaps = [
        right - left
        for left, right in zip(sorted(projections), sorted(projections)[1:])
    ]
    if projection_gaps and max(projection_gaps) > MAX_WALL_POINT_GAP_M:
        return None
    rms = math.sqrt(error_sum / total_weight)
    mean_quality = sum(item.weight for item in items) / len(items)
    line_quality = 1.0 / (1.0 + (rms / max(MAX_WALL_RMS_M, 1e-6)) ** 2)
    quality = _clamp(max(0.90, mean_quality * (0.70 + 0.30 * line_quality)), 0.0, 1.0)
    return _WallSegment(
        mean_x,
        mean_y,
        dx,
        dy,
        min(projections),
        max(projections),
        rms,
        quality,
        len(items),
    )


def _fit_segment(items: Sequence[_EchoItem]) -> _WallSegment | None:
    segment = _fit_segment_unchecked(items)
    if segment is None or segment.rms > MAX_WALL_RMS_M:
        return None
    return segment


def _angle_gap(left: float, right: float, *, wrap: bool = False) -> float:
    gap = right - left
    if wrap:
        gap += math.tau
    return gap


def _connected(left: _EchoItem, right: _EchoItem, *, wrap: bool = False) -> bool:
    return (
        _angle_gap(left.angle, right.angle, wrap=wrap) <= math.radians(MAX_WALL_ANGLE_GAP_DEG)
        and math.hypot(right.x - left.x, right.y - left.y) <= MAX_WALL_NEIGHBOR_GAP_M
    )


def _connected_runs(items: Sequence[_EchoItem]) -> list[list[_EchoItem]]:
    if not items:
        return []
    ordered = sorted(items, key=lambda item: item.angle)
    runs: list[list[_EchoItem]] = [[ordered[0]]]
    for item in ordered[1:]:
        if _connected(runs[-1][-1], item):
            runs[-1].append(item)
        else:
            runs.append([item])
    if len(runs) > 1 and _connected(runs[-1][-1], runs[0][0], wrap=True):
        runs[0] = runs[-1] + runs[0]
        runs.pop()
    return runs


def _line_angle(segment: _WallSegment) -> float:
    angle = math.atan2(segment.dy, segment.dx) % math.pi
    return angle


def _line_angle_difference(left: _WallSegment, right: _WallSegment) -> float:
    difference = abs(_line_angle(left) - _line_angle(right))
    return min(difference, math.pi - difference)


def _turn_angle(
    previous: _EchoItem,
    current: _EchoItem,
    following: _EchoItem,
) -> float:
    left_x = current.x - previous.x
    left_y = current.y - previous.y
    right_x = following.x - current.x
    right_y = following.y - current.y
    left_length = math.hypot(left_x, left_y)
    right_length = math.hypot(right_x, right_y)
    if left_length < MIN_TURN_VECTOR_M or right_length < MIN_TURN_VECTOR_M:
        return 0.0
    cosine = _clamp(
        (left_x * right_x + left_y * right_y) / (left_length * right_length),
        -1.0,
        1.0,
    )
    return math.acos(cosine)


def _smoothed_turn(run: Sequence[_EchoItem], index: int, *, closed: bool) -> float:
    count = len(run)
    if count < 3:
        return 0.0
    window = 2 if count >= 5 else 1
    if closed:
        previous = run[(index - window) % count]
        following = run[(index + window) % count]
    else:
        if index - window < 0 or index + window >= count:
            return 0.0
        previous = run[index - window]
        following = run[index + window]
    return _turn_angle(previous, run[index], following)


def _closed_run(run: Sequence[_EchoItem]) -> bool:
    return len(run) >= 2 and _connected(run[-1], run[0], wrap=True)


def _rotate_closed_run_to_corner(run: Sequence[_EchoItem]) -> list[_EchoItem]:
    if not _closed_run(run):
        return list(run)
    turns = [_smoothed_turn(run, index, closed=True) for index in range(len(run))]
    if not turns:
        return list(run)
    corner_index = max(range(len(turns)), key=turns.__getitem__)
    if turns[corner_index] < math.radians(WALL_CORNER_TURN_DEG):
        return list(run)
    start = (corner_index + 1) % len(run)
    return list(run[start:]) + list(run[:start])


def _split_run_at_corners(run: Sequence[_EchoItem]) -> list[list[_EchoItem]]:
    ordered = _rotate_closed_run_to_corner(run)
    if len(ordered) < MIN_WALL_POINTS:
        return []
    chunks: list[list[_EchoItem]] = []
    start = 0
    for index in range(1, len(ordered) - 1):
        left_count = index - start + 1
        right_count = len(ordered) - index
        if left_count < MIN_WALL_POINTS or right_count < MIN_WALL_POINTS:
            continue
        if _smoothed_turn(ordered, index, closed=False) >= math.radians(WALL_CORNER_TURN_DEG):
            chunks.append(list(ordered[start:index + 1]))
            start = index
    tail = list(ordered[start:])
    if len(tail) >= MIN_WALL_POINTS:
        chunks.append(tail)
    return chunks


def _point_to_segment_line_distance(point: _EchoItem, segment: _WallSegment) -> float:
    normal_x, normal_y = -segment.dy, segment.dx
    return abs((point.x - segment.x) * normal_x + (point.y - segment.y) * normal_y)


def _fit_chunk_recursive(run: Sequence[_EchoItem]) -> list[_WallSegment]:
    if len(run) < MIN_WALL_POINTS:
        return []
    segment = _fit_segment(run)
    if segment is not None:
        return [segment]
    if len(run) < MIN_WALL_POINTS * 2 - 1:
        return []
    loose = _fit_segment_unchecked(run)
    if loose is not None:
        split = max(
            range(MIN_WALL_POINTS - 1, len(run) - MIN_WALL_POINTS + 1),
            key=lambda index: _point_to_segment_line_distance(run[index], loose),
        )
    else:
        split = len(run) // 2
    return _fit_chunk_recursive(run[:split + 1]) + _fit_chunk_recursive(run[split:])


def _segment_endpoint_gap(left: _WallSegment, right: _WallSegment) -> float:
    return min(
        math.hypot(left_x - right_x, left_y - right_y)
        for left_x, left_y in (_endpoint(left, left.t_min), _endpoint(left, left.t_max))
        for right_x, right_y in (_endpoint(right, right.t_min), _endpoint(right, right.t_max))
    )


def _line_offset(left: _WallSegment, right: _WallSegment) -> float:
    normal_x, normal_y = -left.dy, left.dx
    return abs((right.x - left.x) * normal_x + (right.y - left.y) * normal_y)


def _can_merge_collinear(left: _WallSegment, right: _WallSegment) -> bool:
    return (
        _line_angle_difference(left, right) <= math.radians(MAX_COLLINEAR_MERGE_ANGLE_DEG)
        and _line_offset(left, right) <= MAX_COLLINEAR_MERGE_OFFSET_M
        and _segment_endpoint_gap(left, right) <= MAX_COLLINEAR_MERGE_GAP_M
    )


def _merge_collinear_adjacent(segments: Sequence[_WallSegment]) -> list[_WallSegment]:
    merged: list[_WallSegment] = []
    for segment in segments:
        if not merged or not _can_merge_collinear(merged[-1], segment):
            merged.append(segment)
            continue
        target = merged[-1]
        for x, y in (_endpoint(segment, segment.t_min), _endpoint(segment, segment.t_max)):
            t = (x - target.x) * target.dx + (y - target.y) * target.dy
            target.t_min = min(target.t_min, t)
            target.t_max = max(target.t_max, t)
        target.quality = max(target.quality, segment.quality)
        target.rms = max(target.rms, segment.rms)
        target.raw_count += segment.raw_count
    if len(merged) > 1 and _can_merge_collinear(merged[-1], merged[0]):
        first = merged[0]
        last = merged.pop()
        for x, y in (_endpoint(last, last.t_min), _endpoint(last, last.t_max)):
            t = (x - first.x) * first.dx + (y - first.y) * first.dy
            first.t_min = min(first.t_min, t)
            first.t_max = max(first.t_max, t)
        first.quality = max(first.quality, last.quality)
        first.rms = max(first.rms, last.rms)
        first.raw_count += last.raw_count
    return merged


def _fit_run(run: Sequence[_EchoItem]) -> list[_WallSegment]:
    segments: list[_WallSegment] = []
    for chunk in _split_run_at_corners(run):
        segments.extend(_fit_chunk_recursive(chunk))
    return _merge_collinear_adjacent(segments)


def _endpoint(segment: _WallSegment, t: float) -> tuple[float, float]:
    return segment.x + segment.dx * t, segment.y + segment.dy * t


def _nearest_endpoint(segment: _WallSegment, x: float, y: float) -> float:
    endpoints = (segment.t_min, segment.t_max)
    return min(endpoints, key=lambda t: math.hypot(_endpoint(segment, t)[0] - x, _endpoint(segment, t)[1] - y))


def _line_intersection(left: _WallSegment, right: _WallSegment) -> tuple[float, float, float, float] | None:
    cross = left.dx * right.dy - left.dy * right.dx
    if abs(cross) < math.sin(math.radians(12.0)):
        return None
    rx = right.x - left.x
    ry = right.y - left.y
    left_t = (rx * right.dy - ry * right.dx) / cross
    right_t = (rx * left.dy - ry * left.dx) / cross
    x, y = _endpoint(left, left_t)
    return left_t, right_t, x, y


def _close_small_corners(segments: list[_WallSegment]) -> None:
    if len(segments) < 2:
        return
    for index, left in enumerate(segments):
        right = segments[(index + 1) % len(segments)]
        intersection = _line_intersection(left, right)
        if intersection is None:
            continue
        left_t, right_t, x, y = intersection
        left_end = _nearest_endpoint(left, x, y)
        right_end = _nearest_endpoint(right, x, y)
        left_gap = abs(left_t - left_end)
        right_gap = abs(right_t - right_end)
        left_xy = _endpoint(left, left_end)
        right_xy = _endpoint(right, right_end)
        endpoint_gap = math.hypot(left_xy[0] - right_xy[0], left_xy[1] - right_xy[1])
        if (left_gap <= MAX_CORNER_EXTENSION_M and right_gap <= MAX_CORNER_EXTENSION_M
                and endpoint_gap <= MAX_CORNER_ENDPOINT_GAP_M):
            if left_end == left.t_min:
                left.t_min = left_t
            else:
                left.t_max = left_t
            if right_end == right.t_min:
                right.t_min = right_t
            else:
                right.t_max = right_t


def _segment_points(
    segments: Sequence[_WallSegment],
    min_range_m: float,
    max_range_m: float,
    resolution_m: float,
) -> list[ScanPoint]:
    points: list[ScanPoint] = []
    spacing = max(0.5 * resolution_m, 0.015)
    for segment in segments:
        t_min, t_max = sorted((segment.t_min, segment.t_max))
        length = t_max - t_min
        count = max(2, int(math.ceil(length / spacing)) + 1)
        for offset in range(count):
            fraction = offset / (count - 1) if count > 1 else 0.0
            t = t_min + length * fraction
            x, y = _endpoint(segment, t)
            distance = math.hypot(x, y)
            if not min_range_m <= distance <= max_range_m:
                continue
            points.append(ScanPoint(
                math.atan2(x, y) % math.tau,
                distance,
                segment.quality,
                True,
                source="thin_wall",
            ))
    return points


def _fit_thin_walls(
    echoes: Sequence[ScanPoint],
    min_range_m: float,
    max_range_m: float,
    resolution_m: float,
) -> tuple[list[ScanPoint], int, int]:
    items = [
        _EchoItem(
            point,
            point.angle_rad % math.tau,
            point.x,
            point.y,
            _point_quality(point, resolution_m),
        )
        for point in echoes
    ]
    segments: list[_WallSegment] = []
    for run in _connected_runs(items):
        segments.extend(_fit_run(run))
    _close_small_corners(segments)
    fitted = _segment_points(segments, min_range_m, max_range_m, resolution_m)
    supported = sum(segment.raw_count for segment in segments)
    return fitted, supported, len(segments)


def prepare_mapping_points(
    points: Sequence[ScanPoint],
    max_range_m: float,
    min_range_m: float,
    resolution_m: float = DEFAULT_MAP_RESOLUTION_M,
) -> MappingPointSelection:
    """Keep coherent wall evidence and turn it into zero-thickness walls.

    Raw range endpoints jitter by a few cells while the real wall is stable.
    The mapper therefore fits contiguous local wall segments, projects the
    evidence onto the fitted line, and resamples that line at map resolution.
    The result writes thin walls quickly without turning measurement wobble
    into thick occupied bands.
    """
    unique: dict[float, ScanPoint] = {}
    for point in points:
        if (not math.isfinite(point.distance_m) or not math.isfinite(point.angle_rad)
                or not math.isfinite(point.quality)
                or not min_range_m <= point.distance_m <= max_range_m):
            continue
        key = round(point.angle_rad % math.tau, 6)
        previous = unique.get(key)
        if previous is None or (point.quality, -point.distance_m) > (previous.quality, -previous.distance_m):
            unique[key] = point
    ordered = sorted(unique.values(), key=lambda point: point.angle_rad % math.tau)
    echoes = [point for point in ordered if point.has_echo(max_range_m)]
    fitted, supported_count, fitted_segments = _fit_thin_walls(
        echoes,
        min_range_m,
        max_range_m,
        max(float(resolution_m), 1e-6),
    )
    fallback = fitted
    if not fallback and len(echoes) >= 2:
        # Sparse data is still useful for preview/local mapping.  Keep it with
        # reduced confidence, but only when no line model was reliable enough.
        fallback = [
            ScanPoint(point.angle_rad, point.distance_m, min(point.quality, 0.45), True)
            for point in echoes
        ]
    selected = tuple(
        point
        for point in ordered
        if not point.has_echo(max_range_m)
    ) + tuple(fallback)
    raw_echoes = len(echoes)
    supported_echoes = min(raw_echoes, supported_count)
    noise_echoes = max(0, raw_echoes - supported_echoes)
    return MappingPointSelection(
        selected,
        raw_echoes=raw_echoes,
        supported_echoes=supported_echoes,
        noise_echoes=noise_echoes,
        fitted_segments=fitted_segments,
    )


def process_radar_debug_scan(navigator, selection: MappingPointSelection, min_range_m: float) -> VelocityCommand:
    """Build a fixed-pose occupancy map while automatic navigation is off."""
    navigator.latest_scan = list(selection.points)
    navigator.path_cells.clear()
    navigator.target_cell = None
    navigator.frontier_count = 0
    navigator.reachable_frontier_count = 0

    if selection.supported_echoes < 4:
        navigator.state = "仅雷达建图"
        navigator.detail = (
            f"直线结构不足：回波 {selection.raw_echoes} 个，"
            f"仅 {selection.supported_echoes} 个具有短直线支持；暂不写图"
        )
        return VelocityCommand()

    navigator.completed_scans += 1
    before = navigator.grid.update_count
    summary = navigator.grid.update_scan(
        navigator._sensor_pose(),
        selection.points,
        navigator.max_range_m,
        min_range_m=min_range_m,
        scan_confidence=1.0,
    )
    if navigator.grid.update_count > before:
        navigator.local_map_updates += 1
    navigator.state = "仅雷达建图"
    navigator.detail = (
        f"固定姿态薄墙建图：线段 {selection.fitted_segments} 条，"
        f"直线回波 {selection.supported_echoes}/{selection.raw_echoes}，"
        f"忽略孤立噪声 {selection.noise_echoes} 个"
    )
    if summary.out_of_bounds_points:
        navigator.detail += f"，另有 {summary.out_of_bounds_points} 个点超出地图"
    return VelocityCommand()
