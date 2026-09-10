from __future__ import annotations

import argparse
import json
import math
import queue
from pathlib import Path
import statistics
import time

from motion_safety import MotionSafetyGuard
from mapping_runtime import MappingRuntime
from navigation_core import HiddenWorld, wrap_angle
from runtime_config import build_navigation_engine, configuration_fingerprint, resolve_runtime_config
from scan_acquisition import scan_points_from_polar
from virtual_hardware import HardwareSimulation, PRESETS
from simulation_chassis import SimulationChassis


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
    *,
    chassis_transactions: bool = False,
    drop_chassis_ack: bool = False,
    drop_chassis_result: bool = False,
    navigation_policy: str = "simulation",
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
    if navigation_policy not in {"simulation", "hardware"}:
        raise ValueError("导航策略必须为 simulation 或 hardware")
    if navigation_policy == "hardware":
        hardware_config = resolve_runtime_config("hardware", "navigation", raw_config,
                                                 validate_port_assignments=False)
        for key in ("map_width_cells", "map_height_cells", "map_resolution_m"):
            config[key] = hardware_config[key]
        config["simulation_min_range_m"] = hardware_config["min_range_m"]
        config["simulation_max_range_m"] = hardware_config["max_range_m"]
        config["simulation_estimated_sweeps"] = False
        config["simulation_unobserved_clear_range_m"] = hardware_config["hardware_unobserved_clear_range_m"]
        config["simulation_prefer_forward_exploration"] = hardware_config["hardware_prefer_forward_exploration"]
        engine_config = hardware_config
    else:
        engine_config = config
    world = HiddenWorld(seed=seed, map_path=map_path)
    if world.map_error:
        raise ValueError(world.map_error)
    simulator = HardwareSimulation(world, config)
    navigator = build_navigation_engine(engine_config)
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
    results = queue.Queue()
    runtime = MappingRuntime(navigator, min_range_m=config["simulation_min_range_m"], on_result=results.put)
    chassis = (SimulationChassis(simulator, config, runtime, safety,
                                 drop_ack=drop_chassis_ack, drop_result=drop_chassis_result)
               if chassis_transactions else None)
    parked = False
    fatal_safety_stop = False

    def interrupt_motion(reason):
        nonlocal safety_stops, fatal_safety_stop
        safety_stops += 1
        command = safety.command
        elapsed = simulator.time - safety.started_at
        simulator.stop()
        safety.clear()
        runtime.invalidate()
        if chassis is not None:
            chassis.stop(reason)
            fatal_safety_stop = True
        elif reason == "运动方向出现近距离障碍，紧急停车" and command is not None:
            with runtime.lock:
                navigator.request_obstacle_recovery(command, elapsed)
        else:
            fatal_safety_stop = True
            navigator.state = "安全停车"
            navigator.detail = reason

    try:
        while simulator.time < seconds - 1e-9:
            step = min(0.05, seconds - simulator.time)
            started = time.perf_counter()
            sweeps = simulator.advance(step)
            if chassis is not None:
                chassis.poll()
                if chassis.failure:
                    fatal_safety_stop = True
                    navigator.state = "安全停车"
                    navigator.detail = chassis.failure
            simulation_stage_ms.append((time.perf_counter() - started) * 1000)
            local_scan_count += len(simulator.last_local_results)
            safety_interrupted = False
            if safety.command is not None:
                for packet, estimate, observation_time in simulator.last_safety_observations:
                    reason = safety.observe(packet, estimate, observation_time)
                    if reason:
                        interrupt_motion(reason)
                        safety_interrupted = True
                        break
                if safety.command is not None:
                    reason = safety.poll(simulator.time)
                    if reason:
                        interrupt_motion(reason)
                        safety_interrupted = True
            if safety.command is not None and simulator.time >= simulator.resume_at:
                safety.clear()
            if fatal_safety_stop:
                break
            for sequence, polar, _ in sweeps:
                if (safety_interrupted or simulator.time < simulator.resume_at
                        or (chassis is not None and not chassis.can_map)):
                    motion_scan_skipped += 1
                    continue
                points = scan_points_from_polar(polar)
                input_points += len(points)
                started = time.perf_counter()
                point_times = [p.timestamp_s for p in points
                               if p.timestamp_s is not None and math.isfinite(p.timestamp_s)]
                request = runtime.submit("simulation", sequence, points, simulator.time,
                                         scan_start_s=min(point_times, default=simulator.time),
                                         scan_end_s=max(point_times, default=simulator.time))
                if request is None:
                    raise RuntimeError("评估建图队列拒收")
                while True:
                    mapped = results.get(timeout=30)
                    if mapped.request.mode != "control":
                        break
                if mapped.error is not None:
                    raise mapped.error
                command = mapped.command
                mapping_stage_ms.append((time.perf_counter() - started) * 1000)
                accepted_points += mapped.snapshot.accepted_input_points
                result = navigator.last_match_result
                if mapped.snapshot.scan_accepted and result is not None:
                    degenerate_matches += int(result.degenerate)
                    boundary_matches += int(result.touched_search_boundary)
                true_x, true_y = world.start_pose.world_to_local(world.pose.x, world.pose.y)
                if mapped.snapshot.scan_accepted:
                    errors.append((navigator.pose.x - true_x) ** 2 + (navigator.pose.y - true_y) ** 2)
                    yaw_errors.append(wrap_angle(navigator.pose.yaw - (world.pose.yaw - world.start_pose.yaw)) ** 2)
                if navigator.target_cell != previous_target:
                    replans += 1
                    previous_target = navigator.target_cell
                reached = reached or math.dist((world.pose.x, world.pose.y), world.finish) <= 0.20
                parked = navigator.state == "泊车完成"
                if parked:
                    break
                if chassis is not None and not chassis.accept_scan(mapped):
                    continue
                if not command.stopped:
                    if chassis is not None:
                        chassis.execute(command)
                    else:
                        safety.start(command, simulator.time)
                        runtime.invalidate()
                        simulator.execute(command)
                        runtime.predict_motion(command)
            if parked:
                break

    finally:
        runtime.stop()
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
        "schema_version": "2.0",
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
        "approached_goal": reached,
        "correct_parking": parked and math.dist((world.pose.x, world.pose.y), world.finish) <= 0.20,
        "wrong_parking": parked and math.dist((world.pose.x, world.pose.y), world.finish) > 0.20,
        "timed_out": not parked and not fatal_safety_stop,
        "safety_stopped": fatal_safety_stop,
        "final_state": navigator.state,
        "mapping_pipeline": "MappingRuntime",
        "navigation_policy": navigation_policy,
        "sweep_builder": "estimated_phase" if config["simulation_estimated_sweeps"] else "adjacent_zero",
        "chassis_model": "MOVE_RESULT" if chassis is not None else "command_prediction",
        "chassis_fault_injection": {"drop_ack": drop_chassis_ack, "drop_result": drop_chassis_result},
        **(chassis.diagnostics() if chassis is not None else {}),
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
    parser.add_argument("--chassis-transactions", action="store_true")
    parser.add_argument("--drop-chassis-ack", action="store_true")
    parser.add_argument("--drop-chassis-result", action="store_true")
    parser.add_argument("--navigation-policy", choices=("simulation", "hardware"), default="simulation")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds 必须是正有限数")
    if (args.drop_chassis_ack or args.drop_chassis_result) and not args.chassis_transactions:
        parser.error("底盘丢包注入需要 --chassis-transactions")
    try:
        base_config = json.loads(args.config.read_text(encoding="utf-8")) if args.config.exists() else {}
    except (OSError, json.JSONDecodeError) as exc:
        parser.error(f"无法读取配置：{exc}")
    if not isinstance(base_config, dict):
        parser.error("配置根节点必须是 JSON 对象")
    runs = []
    for profile in args.profiles:
        for seed in args.seeds:
            result = evaluate(profile, seed, args.seconds, args.map, base_config,
                              chassis_transactions=args.chassis_transactions,
                              drop_chassis_ack=args.drop_chassis_ack,
                              drop_chassis_result=args.drop_chassis_result,
                              navigation_policy=args.navigation_policy)
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
            "success_rate": sum(r["correct_parking"] for r in selected) / len(selected),
            "approach_rate": sum(r["approached_goal"] for r in selected) / len(selected),
            "wrong_parking_rate": sum(r["wrong_parking"] for r in selected) / len(selected),
            "timeout_rate": sum(r["timed_out"] for r in selected) / len(selected),
            "safety_stop_rate": sum(r["safety_stopped"] for r in selected) / len(selected),
            "pose_rmse_m_mean": statistics.fmean(pose_values) if pose_values else None,
            "mapping_rejection_rate_mean": statistics.fmean(r["mapping_rejection_rate"] for r in selected),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps({
            "schema_version": "2.0",
            "source_type": "synthetic_simulation",
            "summary": summary,
            "runs": runs,
        }, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
