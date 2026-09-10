from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from motion_safety import MotionSafetyGuard
from navigation_core import HiddenWorld, wrap_angle
from runtime_config import build_navigation_engine, configuration_fingerprint, resolve_runtime_config
from scan_acquisition import scan_points_from_polar
from virtual_hardware import HardwareSimulation, PRESETS


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def evaluate(
    profile: str,
    seed: int,
    seconds: float,
    map_path: Path | None = None,
    base_config: dict | None = None,
):
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("seconds 必须是正有限数")
    raw_config = dict(base_config or {})
    raw_config.update({"simulation_profile": profile, "simulation_seed": int(seed)})
    config = resolve_runtime_config(
        "simulation",
        "navigation",
        raw_config,
        prefer_mode_defaults=True,
    )
    world = HiddenWorld(seed=seed, map_path=map_path)
    if world.map_error:
        raise ValueError(world.map_error)
    simulator = HardwareSimulation(world, config)
    navigator = build_navigation_engine(config)
    navigator.set_auto(True)
    errors: list[float] = []
    yaw_errors: list[float] = []
    simulation_stage_ms: list[float] = []
    mapping_stage_ms: list[float] = []
    replans = 0
    previous_target = None
    reached = False
    local_scan_count = 0
    degenerate_matches = 0
    boundary_matches = 0
    input_points = 0
    accepted_points = 0
    motion_scan_skipped = 0
    safety_stops = 0
    safety = MotionSafetyGuard(config)

    while simulator.time < seconds - 1e-9:
        step = min(0.05, seconds - simulator.time)
        started = time.perf_counter()
        sweeps = simulator.advance(step)
        simulation_stage_ms.append((time.perf_counter() - started) * 1000)
        local_scan_count += len(simulator.last_local_results)
        safety_interrupted = False
        if safety.command is not None:
            for packet, estimate, observation_time in simulator.last_safety_observations:
                reason = safety.observe(packet, estimate, observation_time)
                if reason:
                    safety_stops += 1
                    simulator.stop()
                    safety.clear()
                    safety_interrupted = True
                    break
            if safety.command is not None:
                reason = safety.poll(simulator.time)
                if reason:
                    safety_stops += 1
                    simulator.stop()
                    safety.clear()
                    safety_interrupted = True
        if safety.command is not None and simulator.time >= simulator.resume_at:
            safety.clear()
        for _, polar, _ in sweeps:
            if safety_interrupted or simulator.time < simulator.resume_at:
                motion_scan_skipped += 1
                continue
            points = scan_points_from_polar(polar)
            input_points += len(points)
            started = time.perf_counter()
            command = navigator.process_scan(points)
            mapping_stage_ms.append((time.perf_counter() - started) * 1000)
            accepted_points += len(navigator.latest_scan)
            result = navigator.last_match_result
            if result is not None:
                degenerate_matches += int(result.degenerate)
                boundary_matches += int(result.touched_search_boundary)
            true_x, true_y = world.start_pose.world_to_local(world.pose.x, world.pose.y)
            errors.append((navigator.pose.x - true_x) ** 2 + (navigator.pose.y - true_y) ** 2)
            yaw_errors.append(wrap_angle(navigator.pose.yaw - (world.pose.yaw - world.start_pose.yaw)) ** 2)
            if navigator.target_cell != previous_target:
                replans += 1
                previous_target = navigator.target_cell
            reached = math.dist((world.pose.x, world.pose.y), world.finish) <= 0.20
            if reached:
                break
            if not command.stopped:
                safety.start(command, simulator.time)
                simulator.execute(command)
                navigator.predict_motion(command)
        if reached:
            break

    grid = navigator.grid
    compared = mismatched = 0
    for row in range(grid.height):
        for col in range(grid.width):
            state = grid.state(col, row)
            if state == grid.UNKNOWN:
                continue
            point = world.start_pose.local_to_world(*grid.cell_to_world(col, row))
            compared += 1
            mismatched += (state == grid.OCCUPIED) != world._occupied(*point)
    diagnostics = simulator.diagnostics()
    map_area = grid.width * grid.height * grid.resolution_m ** 2
    return {
        "schema_version": "1.0",
        "source_type": "synthetic_simulation",
        "config_fingerprint": configuration_fingerprint(config),
        "profile": profile,
        "seed": seed,
        "runtime": {
            "seconds": seconds,
            "map_width_cells": grid.width,
            "map_height_cells": grid.height,
            "map_resolution_m": grid.resolution_m,
            "min_range_m": config["simulation_min_range_m"],
            "max_range_m": config["simulation_max_range_m"],
        },
        "reached_goal": reached,
        "goal_error_m": math.dist((world.pose.x, world.pose.y), world.finish),
        "pose_rmse_m": math.sqrt(statistics.fmean(errors)) if errors else None,
        "yaw_rmse_deg": math.degrees(math.sqrt(statistics.fmean(yaw_errors))) if yaw_errors else None,
        "known_cell_map_error_rate": mismatched / compared if compared else None,
        "known_cells": compared,
        "known_area_m2": grid.known_area_m2(),
        "map_coverage": grid.known_area_m2() / map_area if map_area else None,
        "target_changes": replans,
        "mapping_attempts": navigator.mapping_attempts,
        "mapping_rejected_scans": navigator.rejected_scans,
        "mapping_rejection_rate": navigator.rejected_scans / max(1, navigator.mapping_attempts),
        "mapping_rejection_reasons": dict(navigator.rejection_reason_counts),
        "map_updates": grid.update_count,
        "input_points": input_points,
        "accepted_points": accepted_points,
        "point_rejection_rate": 1 - accepted_points / input_points if input_points else None,
        "local_scans": local_scan_count,
        "motion_scan_skipped": motion_scan_skipped,
        "safety_stops": safety_stops,
        "degenerate_matches": degenerate_matches,
        "boundary_matches": boundary_matches,
        "timing_ms": {
            "simulation_mean": statistics.fmean(simulation_stage_ms) if simulation_stage_ms else None,
            "simulation_p50": _percentile(simulation_stage_ms, 0.50),
            "simulation_p95": _percentile(simulation_stage_ms, 0.95),
            "mapping_mean": statistics.fmean(mapping_stage_ms) if mapping_stage_ms else None,
            "mapping_p50": _percentile(mapping_stage_ms, 0.50),
            "mapping_p95": _percentile(mapping_stage_ms, 0.95),
        },
        **diagnostics,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("navigation_config.json"))
    parser.add_argument("--seconds", type=float, default=120)
    parser.add_argument("--seeds", type=int, nargs="+", default=[20260907, 20260908, 20260909])
    parser.add_argument("--profiles", nargs="+", choices=tuple(PRESETS), default=list(PRESETS))
    parser.add_argument("--map", type=Path, default=Path(__file__).with_name("simulation_map.json"))
    parser.add_argument("--output", type=Path, default=Path("tmp/simulation_metrics.json"))
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds 必须是正有限数")
    try:
        base_config = json.loads(args.config.read_text(encoding="utf-8")) if args.config.exists() else {}
    except (OSError, json.JSONDecodeError) as exc:
        parser.error(f"无法读取配置：{exc}")
    if not isinstance(base_config, dict):
        parser.error("配置根节点必须是 JSON 对象")
    runs = []
    for profile in args.profiles:
        for seed in args.seeds:
            result = evaluate(profile, seed, args.seconds, args.map, base_config)
            runs.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
    summary = {}
    for profile in args.profiles:
        selected = [r for r in runs if r["profile"] == profile]
        if not selected:
            continue
        pose_values = [r["pose_rmse_m"] for r in selected if r["pose_rmse_m"] is not None]
        summary[profile] = {
            "runs": len(selected),
            "success_rate": sum(r["reached_goal"] for r in selected) / len(selected),
            "pose_rmse_m_mean": statistics.fmean(pose_values) if pose_values else None,
            "mapping_rejection_rate_mean": statistics.fmean(r["mapping_rejection_rate"] for r in selected),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps({
            "schema_version": "1.0",
            "source_type": "synthetic_simulation",
            "summary": summary,
            "runs": runs,
        }, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
