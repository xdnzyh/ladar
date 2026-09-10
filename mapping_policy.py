from __future__ import annotations

from dataclasses import dataclass, replace
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
MAX_COLLINEAR_MERGE_GAP_M = 0.06


@dataclass(frozen=True)
class MappingPointSelection:
    points: tuple[ScanPoint, ...]
    raw_echoes: int
    supported_echoes: int
    noise_echoes: int
    fitted_segments: int = 0
    confirmed_scans: int = 1
    mapping_passes: int = 1
    unconfirmed_sectors: tuple[int, ...] = ()
    reset_required: bool = False
    rejection_reason: str = ""
    mapping_layers: tuple[tuple[ScanPoint, ...], ...] = ()


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
    items: list[_EchoItem]


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
    mean_quality = sum(_clamp(float(item.point.quality), 0.0, 1.0) for item in items) / len(items)
    line_quality = 1.0 / (1.0 + (rms / max(MAX_WALL_RMS_M, 1e-6)) ** 2)
    quality = _clamp(mean_quality * (0.70 + 0.30 * line_quality), 0.0, 1.0)
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
        list(items),
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
        target.items.extend(segment.items)
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
        first.items.extend(last.items)
    return merged


def _fit_run(run: Sequence[_EchoItem]) -> list[_WallSegment]:
    segments: list[_WallSegment] = []
    for chunk in _split_run_at_corners(run):
        segments.extend(_fit_chunk_recursive(chunk))
    return _merge_collinear_adjacent(segments)


def _endpoint(segment: _WallSegment, t: float) -> tuple[float, float]:
    return segment.x + segment.dx * t, segment.y + segment.dy * t


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
            source_item = min(
                segment.items,
                key=lambda item: (item.x - x) ** 2 + (item.y - y) ** 2,
            )
            source_point = source_item.point
            source = "thin_wall" if not source_point.source else f"thin_wall:{source_point.source}"
            points.append(ScanPoint(
                math.atan2(x, y) % math.tau,
                distance,
                min(segment.quality, source_point.quality),
                True,
                timestamp_s=source_point.timestamp_s,
                time_error_s=source_point.time_error_s,
                angle_error_rad=source_point.angle_error_rad,
                distance_error_m=source_point.distance_error_m,
                pixel=source_point.pixel,
                calibration_version=source_point.calibration_version,
                source=source,
                session=source_point.session,
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
    selected = tuple(fitted)
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


class TwoSweepWallEvidence:
    SECTOR_COUNT = 24

    def __init__(self, resolution_m: float = DEFAULT_MAP_RESOLUTION_M) -> None:
        self.resolution_m = max(float(resolution_m), 1e-6)
        self.reset()

    def reset(self) -> None:
        self._session: str | None = None
        self._sequence: int | None = None
        self._selection: MappingPointSelection | None = None
        self._has_confirmed_pair = False

    @classmethod
    def _unconfirmed_sectors(cls, points: Sequence[ScanPoint]) -> tuple[int, ...]:
        confirmed = {
            min(cls.SECTOR_COUNT - 1, int((point.angle_rad % math.tau) / math.tau * cls.SECTOR_COUNT))
            for point in points
        }
        return tuple(index for index in range(cls.SECTOR_COUNT) if index not in confirmed)

    def update(
        self,
        session: str,
        sequence: int,
        selection: MappingPointSelection,
    ) -> MappingPointSelection:
        session = str(session)
        sequence = int(sequence)
        continuous = (
            self._session == session
            and self._sequence is not None
            and sequence == self._sequence + 1
        )
        reset_required = self._sequence is not None and not continuous
        previous = self._selection if continuous else None
        had_confirmed_pair = self._has_confirmed_pair if continuous else False
        self._session = session
        self._sequence = sequence
        self._selection = selection

        if previous is None or not previous.points or not selection.points:
            self._has_confirmed_pair = False
            return MappingPointSelection(
                (),
                raw_echoes=selection.raw_echoes,
                supported_echoes=0,
                noise_echoes=selection.noise_echoes,
                fitted_segments=0,
                confirmed_scans=1,
                mapping_passes=0,
                unconfirmed_sectors=tuple(range(self.SECTOR_COUNT)),
                reset_required=reset_required,
                rejection_reason="等待第二个连续可信整圈",
            )

        match_distance = max(2.5 * self.resolution_m, 0.05)
        previous_points = tuple(previous.points)
        stable = []
        previous_layer = []
        for point in selection.points:
            nearest = min(
                previous_points,
                key=lambda candidate: (candidate.x - point.x) ** 2 + (candidate.y - point.y) ** 2,
            )
            if math.hypot(nearest.x - point.x, nearest.y - point.y) > match_distance:
                continue
            source = point.source or "thin_wall"
            if source.startswith("thin_wall"):
                source = "stable_wall" + source[len("thin_wall"):]
            stable_point = ScanPoint(
                point.angle_rad,
                point.distance_m,
                min(point.quality, nearest.quality),
                True,
                timestamp_s=point.timestamp_s,
                time_error_s=point.time_error_s,
                angle_error_rad=point.angle_error_rad,
                distance_error_m=point.distance_error_m,
                pixel=point.pixel,
                calibration_version=point.calibration_version,
                source=source,
                session=point.session,
            )
            previous_source = nearest.source or "thin_wall"
            if previous_source.startswith("thin_wall"):
                previous_source = "stable_wall" + previous_source[len("thin_wall"):]
            previous_layer.append(ScanPoint(
                stable_point.angle_rad,
                stable_point.distance_m,
                min(point.quality, nearest.quality),
                True,
                timestamp_s=nearest.timestamp_s,
                time_error_s=nearest.time_error_s,
                angle_error_rad=nearest.angle_error_rad,
                distance_error_m=nearest.distance_error_m,
                pixel=nearest.pixel,
                calibration_version=nearest.calibration_version,
                source=previous_source,
                session=nearest.session,
            ))
            stable.append(stable_point)

        ratio = len(stable) / max(1, len(selection.points))
        supported = min(
            selection.supported_echoes,
            previous.supported_echoes,
            int(round(selection.supported_echoes * ratio)),
        )
        self._has_confirmed_pair = bool(stable and supported >= 4)
        reason = "" if self._has_confirmed_pair else "相邻两圈墙段缺少重复支持"
        return MappingPointSelection(
            tuple(stable),
            raw_echoes=selection.raw_echoes,
            supported_echoes=supported,
            noise_echoes=max(selection.noise_echoes, selection.raw_echoes - supported),
            fitted_segments=selection.fitted_segments if stable else 0,
            confirmed_scans=2 if self._has_confirmed_pair else 1,
            mapping_passes=(1 if had_confirmed_pair else 2) if self._has_confirmed_pair else 0,
            unconfirmed_sectors=self._unconfirmed_sectors(stable),
            reset_required=reset_required,
            rejection_reason=reason,
            mapping_layers=(tuple(previous_layer), tuple(stable)) if self._has_confirmed_pair else (),
        )


def prepare_free_space_points(points, max_range_m, min_range_m, resolution_m):
    result = []
    for point in points:
        if (not math.isfinite(point.angle_rad) or not math.isfinite(point.distance_m)
                or not math.isfinite(point.quality) or point.quality < 0.25
                or not min_range_m <= point.distance_m <= max_range_m):
            continue
        distance = point.distance_m
        if point.has_echo(max_range_m):
            distance -= math.sqrt(2) * resolution_m
        if distance >= min_range_m:
            result.append(replace(point, distance_m=distance, is_echo=False, source="free_space"))
    return tuple(result)


def process_radar_debug_scan(navigator, selection: MappingPointSelection, min_range_m: float,
                             *, free_space_points: Sequence[ScanPoint] = ()) -> VelocityCommand:
    """Build a fixed-pose occupancy map while automatic navigation is off."""
    navigator.latest_scan = list(selection.points)
    navigator.path_cells.clear()
    navigator.target_cell = None
    navigator.frontier_count = 0
    navigator.reachable_frontier_count = 0

    if free_space_points:
        navigator.grid.update_scan(navigator._sensor_pose(), free_space_points,
                                   navigator.max_range_m, min_range_m=min_range_m, add_only=True)

    if selection.confirmed_scans < 2 or selection.supported_echoes < 4:
        navigator.state = "仅雷达建图"
        navigator.detail = selection.rejection_reason or (
            f"直线结构不足：回波 {selection.raw_echoes} 个，"
            f"仅 {selection.supported_echoes} 个具有连续两圈支持；暂不写图"
        )
        return VelocityCommand()

    navigator.completed_scans += selection.mapping_passes
    before = navigator.grid.update_count
    summary = None
    layers = selection.mapping_layers or (selection.points,)
    for layer in layers:
        summary = navigator.grid.update_scan(
            navigator._sensor_pose(),
            layer,
            navigator.max_range_m,
            min_range_m=min_range_m,
            scan_confidence=1.0,
            add_only=True,
        )
    if navigator.grid.update_count > before:
        navigator.local_map_updates += 1
    navigator.state = "仅雷达建图"
    navigator.detail = (
        f"固定姿态薄墙建图：线段 {selection.fitted_segments} 条，"
        f"两圈重复回波 {selection.supported_echoes}/{selection.raw_echoes}，"
        f"忽略孤立噪声 {selection.noise_echoes} 个，"
        f"未确认扇区 {len(selection.unconfirmed_sectors)}/{TwoSweepWallEvidence.SECTOR_COUNT}"
    )
    if summary is not None and summary.out_of_bounds_points:
        navigator.detail += f"，另有 {summary.out_of_bounds_points} 个点超出地图"
    return VelocityCommand()
