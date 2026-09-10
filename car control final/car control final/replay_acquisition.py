from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from measurement_protocol import parse_observation
from radar_core import CalibrationModel
from runtime_config import build_navigation_engine, configuration_fingerprint, resolve_runtime_config
from scan_acquisition import DistanceObservationReceiver, ReceivedObservation, scan_points_from_polar
from synchronized_acquisition import ClockEstimate


def _load_fixture(path: Path) -> tuple[dict, list[dict]]:
    header = {}
    records = []
    for line_number, text in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not text.strip():
            continue
        try:
            item = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError(f"回放文件第 {line_number} 行不是 JSON") from exc
        if not isinstance(item, dict):
            raise ValueError(f"回放文件第 {line_number} 行必须是对象")
        if not records and item.get("record_type") == "header":
            header = item
            continue
        if not {"source", "arrival_s", "line"}.issubset(item):
            raise ValueError(f"回放文件第 {line_number} 行缺少 source、arrival_s 或 line")
        records.append(item)
    if not records:
        raise ValueError("回放文件没有观测记录")
    return header, records


def _load_config(path: Path) -> dict:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取配置：{exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("配置根节点必须是 JSON 对象")
    config = resolve_runtime_config("hardware", "radar", data)
    calibration = CalibrationModel.from_dict(config.get("calibration"))
    if not calibration.ready:
        calibration_path = Path(str(config.get("calibration_file", "")))
        if not calibration_path.is_absolute():
            calibration_path = path.parent / calibration_path
        if calibration_path.exists():
            calibration = CalibrationModel.from_csv(calibration_path)
    if not calibration.ready:
        raise ValueError("离线回放需要有效的距离标定模型")
    config["calibration"] = calibration.to_dict()
    return config


def _clock_models(header: dict, config: dict) -> dict[str, dict[int, ClockEstimate]]:
    default = {
        source: {0: ClockEstimate(0.0, 0.0001, 0.0, float(config.get("clock_drift_bound_ppm", 500.0)), 0)}
        for source in ("measurement", "rotation")
    }
    for exchange in header.get("sync_exchanges", []):
        if not isinstance(exchange, dict):
            continue
        source = str(exchange.get("source", ""))
        if source not in default:
            continue
        try:
            estimate = ClockEstimate.exchange(
                float(exchange["t1_s"]),
                int(exchange["t2_us"]),
                int(exchange["t3_us"]),
                float(exchange["t4_s"]),
                float(exchange.get("drift_ppm", config.get("clock_drift_bound_ppm", 500.0))),
            )
            version = int(exchange.get("model_version", estimate.version))
            if version > 0:
                previous = default[source].get(version - 1, default[source][max(default[source])])
                try:
                    estimate = previous.updated(estimate)
                except ValueError:
                    estimate = ClockEstimate(
                        estimate.offset,
                        estimate.uncertainty,
                        estimate.observed_at,
                        estimate.drift_ppm,
                        version,
                    )
            default[source][version] = ClockEstimate(
                estimate.offset,
                estimate.uncertainty,
                estimate.observed_at,
                estimate.drift_ppm,
                version,
            )
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    return default


def replay(input_path: Path, config_path: Path) -> dict:
    header, records = _load_fixture(input_path)
    config = _load_config(config_path)
    session = str(header.get("session", "replay"))
    receiver = DistanceObservationReceiver(config)
    clocks = _clock_models(header, config)
    navigator = build_navigation_engine(config)
    formal_scans = []
    local_scans = []
    diagnostics = []
    parse_errors = []
    input_points = 0
    accepted_points = 0
    started = time.perf_counter()
    for item in records:
        source = str(item["source"])
        arrival = float(item["arrival_s"])
        line = str(item["line"])
        if source not in clocks or not math.isfinite(arrival):
            diagnostics.append(f"ignored_source:{source}")
            continue
        try:
            parsed = parse_observation(source, line, session, navigator_config_calibration(config), config)
        except (TypeError, ValueError, OverflowError) as exc:
            parse_errors.append({"line": line, "error": str(exc)})
            continue
        if parsed is None:
            diagnostics.append("unrecognized_line")
            continue
        model_version = item.get("clock_model_version")
        if model_version is None:
            clock = clocks[source][max(clocks[source])]
        else:
            clock = clocks[source].get(int(model_version), clocks[source][max(clocks[source])])
        receiver.feed(ReceivedObservation(parsed.normalize(clock, arrival), arrival))
        result = receiver.poll(arrival)
        formal_scans.extend(result.formal_scans)
        local_scans.extend(result.local_scans)
        diagnostics.extend(result.diagnostics)
    final_time = max(float(item["arrival_s"]) for item in records) + receiver.reorder_s + 1.0
    result = receiver.poll(final_time)
    formal_scans.extend(result.formal_scans)
    local_scans.extend(result.local_scans)
    diagnostics.extend(result.diagnostics)
    mapping_scans = 0
    for _, points, _ in local_scans:
        scan_points = scan_points_from_polar(points)
        input_points += len(scan_points)
        navigator.process_local_scan(scan_points, min_range_m=config["min_range_m"])
        accepted_points += len(navigator.latest_scan)
        mapping_scans += 1
    elapsed_ms = (time.perf_counter() - started) * 1000
    point_total = sum(len(points) for _, points, _ in formal_scans)
    echo_total = sum(sum(point.is_echo for point in points) for _, points, _ in formal_scans)
    return {
        "schema_version": "1.0",
        "source_type": "offline_replay",
        "input": str(input_path),
        "config": str(config_path),
        "config_fingerprint": configuration_fingerprint(config),
        "session": session,
        "fixture_header": header,
        "clock_models": {
            source: [vars(model) for _, model in sorted(models.items())]
            for source, models in clocks.items()
        },
        "records": len(records),
        "parse_errors": parse_errors,
        "formal_scans": len(formal_scans),
        "local_scans": len(local_scans),
        "mapping_scans": mapping_scans,
        "formal_points": point_total,
        "formal_echo_points": echo_total,
        "formal_echo_rate": echo_total / point_total if point_total else None,
        "mapped_points": input_points,
        "accepted_mapping_points": accepted_points,
        "mapping_point_rejection_rate": 1 - accepted_points / input_points if input_points else None,
        "map_updates": navigator.grid.update_count,
        "known_area_m2": navigator.grid.known_area_m2(),
        "accepted_counter": receiver.accepted,
        "warmup_counter": receiver.warmup,
        "discarded_counter": receiver.discarded,
        "late_packets": receiver.late,
        "duplicate_packets": receiver.duplicates,
        "pending_depth_peak": receiver.max_pending,
        "builder_rejected_points": receiver.builder.rejected_points,
        "diagnostics": sorted(set(diagnostics)),
        "elapsed_ms": elapsed_ms,
    }


def navigator_config_calibration(config: dict) -> CalibrationModel:
    return CalibrationModel.from_dict(config.get("calibration"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=Path("tests/fixtures/sync_replay.jsonl"))
    parser.add_argument("--config", type=Path, default=Path("navigation_config.json"))
    parser.add_argument("--output", type=Path, default=Path("tmp/replay_metrics.json"))
    args = parser.parse_args()
    try:
        result = replay(args.input, args.config)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
