from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from navigation_core import HiddenWorld, NavigationEngine, OccupancyGrid, ScanPoint, wrap_angle
from virtual_hardware import HardwareSimulation, PRESETS


def evaluate(profile: str, seed: int, seconds: float, map_path: Path | None = None):
    world = HiddenWorld(map_path=map_path)
    if world.map_error:
        raise ValueError(world.map_error)
    config = {"simulation_profile": profile, "simulation_seed": seed}
    simulator = HardwareSimulation(world, config)
    navigator = NavigationEngine(OccupancyGrid())
    navigator.set_auto(True)
    errors, yaw_errors = [], []
    replans = 0
    previous_target = None
    reached = False
    while simulator.time < seconds - 1e-9:
        sweeps = simulator.advance(min(0.05, seconds - simulator.time))
        for _, polar, _ in sweeps:
            command = navigator.process_scan([ScanPoint(p.angle_rad, p.distance_m) for p in polar])
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
    return {"profile": profile, "seed": seed, "reached_goal": reached,
            "goal_error_m": math.dist((world.pose.x, world.pose.y), world.finish),
            "pose_rmse_m": math.sqrt(sum(errors) / len(errors)) if errors else None,
            "yaw_rmse_deg": math.degrees(math.sqrt(sum(yaw_errors) / len(yaw_errors))) if yaw_errors else None,
            "known_cell_map_error_rate": mismatched / compared if compared else None,
            "known_cells": compared, "target_changes": replans,
            "mapping_rejected_scans": getattr(navigator, "rejected_scans", 0),
            "mapping_rejection_rate": getattr(navigator, "rejected_scans", 0) / max(1, getattr(navigator, "mapping_attempts", navigator.completed_scans)),
            "map_updates": navigator.grid.update_count,
            **simulator.diagnostics()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=120)
    parser.add_argument("--seeds", type=int, nargs="+", default=[20260907, 20260908, 20260909])
    parser.add_argument("--profiles", nargs="+", choices=tuple(PRESETS), default=list(PRESETS))
    parser.add_argument("--map", type=Path, default=Path(__file__).with_name("simulation_map.json"))
    parser.add_argument("--output", type=Path, default=Path("simulation_metrics.json"))
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds 必须是正有限数")
    runs = []
    for profile in args.profiles:
        for seed in args.seeds:
            result = evaluate(profile, seed, args.seconds, args.map)
            runs.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
    summary = {profile: {"runs": len(selected),
                        "success_rate": sum(r["reached_goal"] for r in selected) / len(selected)}
               for profile in args.profiles if (selected := [r for r in runs if r["profile"] == profile])}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"summary": summary, "runs": runs}, indent=2, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    main()
