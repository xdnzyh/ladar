from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from navigation_core import Pose2D, ScanPoint
from runtime_config import build_navigation_engine, configuration_fingerprint, resolve_runtime_config


def _load_fixture(path: Path) -> list[list[ScanPoint]]:
    scans = []
    for line_number, text in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not text.strip():
            continue
        item = json.loads(text)
        if item.get("record_type") == "header":
            continue
        if "points" not in item:
            raise ValueError(f"基准文件第 {line_number} 行缺少 points")
        points = []
        for raw in item["points"]:
            points.append(ScanPoint(
                float(raw["angle_rad"]),
                float(raw["distance_m"]),
                float(raw.get("quality", 1.0)),
                raw.get("is_echo", True),
                timestamp_s=raw.get("timestamp_s", item.get("timestamp_s")),
                angle_error_rad=raw.get("angle_error_rad"),
                distance_error_m=raw.get("distance_error_m"),
                source="benchmark",
                session=str(item.get("session", "benchmark")),
            ))
        scans.append(points)
    if not scans:
        raise ValueError("基准文件没有扫描")
    return scans


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def _measure(config: dict, scans: list[list[ScanPoint]], repeat: int, cold_cache: bool) -> dict:
    elapsed = []
    scan_count = 0
    for _ in range(repeat):
        navigator = build_navigation_engine(config)
        for _ in range(3):
            navigator.grid.update_scan(
                Pose2D(),
                scans[0],
                float(config["simulation_max_range_m"]),
                min_range_m=float(config["simulation_min_range_m"]),
            )
        for points in scans:
            if cold_cache:
                navigator.grid._occupied_cache = None
                navigator.grid._inflated_cache.clear()
                navigator.grid._field_cache.clear()
            started = time.perf_counter()
            navigator.matcher.match(
                navigator.grid,
                Pose2D(),
                points,
                window_scale=0.0,
                minimum_evidence=0.25,
            )
            elapsed.append((time.perf_counter() - started) * 1000)
            scan_count += 1
    return {
        "repeat": repeat,
        "scans": scan_count,
        "mean_ms_per_scan": statistics.fmean(elapsed),
        "p50_ms_per_scan": _percentile(elapsed, 0.50),
        "p95_ms_per_scan": _percentile(elapsed, 0.95),
        "total_ms": sum(elapsed),
        "match_runs": scan_count,
    }


def benchmark(fixture: Path, config_path: Path, repeat: int) -> dict:
    base = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(base, dict):
        raise ValueError("配置根节点必须是 JSON 对象")
    config = resolve_runtime_config("simulation", "navigation", base, prefer_mode_defaults=True)
    scans = _load_fixture(fixture)
    baseline = _measure(config, scans, repeat, cold_cache=True)
    optimized = _measure(config, scans, repeat, cold_cache=False)
    baseline_time = baseline["mean_ms_per_scan"]
    optimized_time = optimized["mean_ms_per_scan"]
    return {
        "schema_version": "1.0",
        "source_type": "synthetic_benchmark_fixture",
        "fixture": str(fixture),
        "config": str(config_path),
        "config_fingerprint": configuration_fingerprint(config),
        "b1": {
            "definition": "每次匹配前清空距离场与障碍缓存的 B1 参考路径",
            "measured": baseline,
        },
        "optimized": {
            "definition": "复用固定地图上的距离场与障碍缓存的当前路径",
            "measured": optimized,
        },
        "comparison": {
            "speedup": baseline_time / optimized_time if optimized_time > 0 else None,
            "baseline_mean_ms": baseline_time,
            "optimized_mean_ms": optimized_time,
            "evidence_scope": "固定合成回放；不是硬件吞吐承诺",
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", type=Path, default=Path("tests/fixtures/mapping_replay.jsonl"))
    parser.add_argument("--config", type=Path, default=Path("navigation_config.json"))
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--output", type=Path, default=Path("tmp/performance_metrics.json"))
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error("--repeat 必须是正整数")
    try:
        result = benchmark(args.fixture, args.config, args.repeat)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
