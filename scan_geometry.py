from __future__ import annotations

import math
from typing import Sequence


def range_tolerance_scale(distance_m: float) -> float:
    """Bounded geometric noise allowance: unchanged through 0.5 m, up to 3x.

    Range scatter grows with distance. This scales spatial residuals only;
    it does not increase measurement evidence or relax clock/safety gates.
    """
    return 1.0 + min(2.0, max(0.0, 2.0 * (distance_m - 0.5)))


def _window_is_linear(items, max_rms_m: float, min_span_m: float) -> bool:
    count = len(items)
    if count < 2:
        return False
    # Smooth, consistently signed turning is curvature, not range scatter.
    # Keep this independent of the distance-dependent residual allowance.
    turns = []
    for left, middle, right in zip(items, items[1:], items[2:]):
        ax, ay = middle[2] - left[2], middle[3] - left[3]
        bx, by = right[2] - middle[2], right[3] - middle[3]
        turns.append(math.atan2(ax * by - ay * bx, ax * bx + ay * by))
    if (len(turns) >= 3 and abs(sum(turns)) > math.radians(20)
            and (all(turn > math.radians(2) for turn in turns)
                 or all(turn < -math.radians(2) for turn in turns))):
        return False
    mean_x = sum(item[2] for item in items) / count
    mean_y = sum(item[3] for item in items) / count
    sxx = syy = sxy = 0.0
    for _, _, x, y in items:
        dx = x - mean_x
        dy = y - mean_y
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    sxx /= count
    syy /= count
    sxy /= count
    trace = sxx + syy
    discriminant = math.sqrt(max(0.0, (sxx - syy) ** 2 + 4.0 * sxy * sxy))
    minor_variance = max(0.0, 0.5 * (trace - discriminant))
    rms = math.sqrt(minor_variance)
    span = 0.0
    for index, item in enumerate(items):
        for other in items[index + 1:]:
            span = max(span, math.hypot(item[2] - other[2], item[3] - other[3]))
    return rms <= max_rms_m and span >= min_span_m


def line_supported_indices(
    points: Sequence[object],
    *,
    min_window_points: int = 5,
    max_angle_gap_deg: float = 20.0,
    max_neighbor_gap_m: float = 0.18,
    max_rms_m: float = 0.012,
    min_span_m: float = 0.050,
) -> set[int]:
    """Return indices belonging to locally straight, contiguous scan runs.

    The filter is intentionally local.  It keeps short wall fragments while
    rejecting isolated echoes and small irregular clusters.  At least five
    consecutive samples and a 12 mm near-range RMS bound are enforced even
    if a legacy caller supplies looser values. Farther runs allow bounded
    range scatter while retaining angular continuity and minimum support.
    """
    min_window_points = max(5, int(min_window_points))
    max_rms_m = min(0.012, float(max_rms_m))
    max_angle_gap = math.radians(float(max_angle_gap_deg))
    if not all(math.isfinite(value) and value > 0 for value in (
            max_angle_gap, max_neighbor_gap_m, max_rms_m, min_span_m)):
        raise ValueError("line support thresholds must be positive finite values")

    items = []
    for index, point in enumerate(points):
        try:
            angle = float(getattr(point, "angle_rad")) % math.tau
            x = float(getattr(point, "x"))
            y = float(getattr(point, "y"))
        except (TypeError, ValueError, AttributeError):
            continue
        if all(math.isfinite(value) for value in (angle, x, y)):
            items.append((index, angle, x, y))
    if len(items) < min_window_points:
        return set()
    items.sort(key=lambda item: item[1])

    def connected(left, right, *, wrap=False):
        angle_gap = right[1] - left[1]
        if wrap:
            angle_gap += math.tau
        return (
            angle_gap <= max_angle_gap + 1e-12
            and math.hypot(right[2] - left[2], right[3] - left[3]) <= max_neighbor_gap_m * range_tolerance_scale(
                min(math.hypot(left[2], left[3]), math.hypot(right[2], right[3])))
        )

    runs = [[items[0]]]
    for item in items[1:]:
        if connected(runs[-1][-1], item):
            runs[-1].append(item)
        else:
            runs.append([item])
    if len(runs) > 1 and connected(runs[-1][-1], runs[0][0], wrap=True):
        runs[0] = runs[-1] + runs[0]
        runs.pop()

    supported: set[int] = set()
    for run in runs:
        if len(run) < min_window_points:
            continue
        for start in range(0, len(run) - min_window_points + 1):
            window = run[start:start + min_window_points]
            distance = min(math.hypot(item[2], item[3]) for item in window)
            if _window_is_linear(window, max_rms_m * range_tolerance_scale(distance), min_span_m):
                supported.update(item[0] for item in window)
    return supported
