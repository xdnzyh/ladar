from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from navigation_core import ScanPoint, VelocityCommand
from scan_geometry import line_supported_indices


@dataclass(frozen=True)
class MappingPointSelection:
    points: tuple[ScanPoint, ...]
    raw_echoes: int
    supported_echoes: int
    noise_echoes: int


def prepare_mapping_points(
    points: Sequence[ScanPoint],
    max_range_m: float,
    min_range_m: float,
) -> MappingPointSelection:
    """Keep coherent short wall segments and discard isolated echo clutter.

    Non-echo/max-range rays are kept because they contribute free-space
    evidence.  Echo endpoints are accepted only when they have local straight
    line support.  This matches the intended small indoor scene, where walls
    appear as several short straight fragments and isolated dots are usually
    optical/measurement clutter.
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
    supported = line_supported_indices(echoes)
    supported_ids = {id(echoes[index]) for index in supported}
    selected = tuple(
        point
        for point in ordered
        if not point.has_echo(max_range_m) or id(point) in supported_ids
    )
    return MappingPointSelection(
        selected,
        raw_echoes=len(echoes),
        supported_echoes=len(supported),
        noise_echoes=max(0, len(echoes) - len(supported)),
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
        f"固定姿态建图：直线回波 {selection.supported_echoes}/{selection.raw_echoes}，"
        f"忽略孤立噪声 {selection.noise_echoes} 个"
    )
    if summary.out_of_bounds_points:
        navigator.detail += f"，另有 {summary.out_of_bounds_points} 个点超出地图"
    return VelocityCommand()
