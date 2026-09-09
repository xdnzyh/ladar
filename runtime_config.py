from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping


RUNTIME_DEFAULTS = {
    "synchronized_acquisition": True,
    "observation_reorder_s": 0.3,
    "hardware_settle_s": 0.2,
    "safety_clearance_m": 0.12,
    "safety_max_observation_age_s": 0.5,
    "safety_blind_timeout_s": 0.75,
    "safety_speed_upper_bound_mps": None,
    "safety_stop_distance_m": None,
    "safety_max_angle_error_deg": 15.0,
    "hardware_sample_rate_hz": 80.0,
    "exposure_index": 3,
    "actual_exposure_index": 3,
    "measurement_firmware_version": "CCD-CAL-1.3-SYNC",
    "rotation_firmware_version": "ROTATION_SYNC_MP_V2",
    "pixel_min": 0,
    "pixel_max": 1500,
    "calibration_model": "table",
    "calibration_file": "CCD_Distance_App_v1_2/calibration.csv",
    "max_timing_position_error_m": 0.04,
    "clock_drift_bound_ppm": 500.0,
    "irq_timestamp_uncertainty_ms": 2.0,
    "sync_interval_s": 0.5,
    "sync_max_age_s": 8.0,
    "keepalive_interval_s": 5.0,
    "period_tolerance": 0.05,
    "max_scan_gap_deg": 25.0,
    "scan_gap_factor": 2.5,
    "scan_gap_hard_limit_deg": 45.0,
    "measurement_port": "",
    "rotation_port": "",
    "chassis_port": "",
    "baudrate": 115200,
    "radar_baudrate": 115200,
    "chassis_baudrate": 9600,
    "measurement_mode": "fffe",
    "ccd_command": "@c0071#@",
    "sample_rate_hz": 80.0,
    "min_range_m": 0.10,
    "max_range_m": 0.50,
    "hardware_min_range_m": 0.10,
    "hardware_max_range_m": 0.50,
    "display_radius_m": 0.60,
    "angle_offset_deg": 0.0,
    "clockwise": True,
    "radar_period_s": 1.5,
    "fusion_delay_ms": 80,
    "map_resolution_m": 0.02,
    "map_width_cells": 100,
    "map_height_cells": 100,
    "radar_offset_x_m": 0.0,
    "radar_offset_y_m": 0.0,
    "radar_offset_yaw_deg": 0.0,
    "robot_radius_m": 0.15,
    "path_turn_penalty": 0.75,
    "chassis_command_template": "",
    "chassis_stop_command": "",
    "chassis_feedback_protocol": "",
    "chassis_protocol_version": "MECANUM UNIVERSAL V6.3 COMM",
    "chassis_ack_timeout_s": 2.0,
    "chassis_action_timeout_s": 12.0,
    "chassis_stop_timeout_s": 3.0,
    "chassis_stop_status_delay_s": 0.35,
    "chassis_max_line_bytes": 1024,
    "chassis_min_counts": 1,
    "chassis_max_counts": 2000,
    "chassis_max_translation_m": 0.20,
    "chassis_max_rotation_rad": 0.5,
    "chassis_speed_validated": False,
    "chassis_braking_validated": False,
    "chassis_calibration": {
        mode: {"status": "uncalibrated"}
        for mode in "WSADQEZCRF"
    },
    "wheel_output_scale": 1000,
    "wheel_signs": [1, 1, 1, 1],
    "simulation_sample_rate_hz": 80.0,
    "simulation_speed": 1.0,
    "simulation_profile": "NOMINAL",
    "simulation_seed": 20260907,
    "simulation_reorder_s": 0.3,
    "simulation_settle_s": 0.2,
    "simulation_map_file": "simulation_map.json",
    "simulation_min_range_m": 0.08,
    "simulation_max_range_m": 3.0,
    "mapping_min_evidence": 4.0,
    "mapping_resolution_weight": 1.0,
    "mapping_queue_size": 2,
    "mapping_poll_budget_ms": 4.0,
}


class RuntimeConfigError(ValueError):
    pass


def _finite(config: Mapping[str, object], key: str, *, positive: bool = False,
            nonnegative: bool = False) -> float:
    try:
        value = float(config[key])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise RuntimeConfigError(f"配置 {key} 必须是有限数值") from exc
    if not math.isfinite(value):
        raise RuntimeConfigError(f"配置 {key} 必须是有限数值")
    if positive and value <= 0:
        raise RuntimeConfigError(f"配置 {key} 必须大于 0")
    if nonnegative and value < 0:
        raise RuntimeConfigError(f"配置 {key} 不能为负数")
    return value


def _set_number(config: dict, key: str, *, positive: bool = False,
                nonnegative: bool = False) -> None:
    config[key] = _finite(config, key, positive=positive, nonnegative=nonnegative)


def resolve_runtime_config(
    source: str,
    view: str = "radar",
    overrides: Mapping[str, object] | None = None,
    *,
    prefer_mode_defaults: bool = False,
) -> dict:
    source = str(source).strip().lower()
    view = str(view).strip().lower()
    if source not in {"hardware", "simulation"}:
        raise RuntimeConfigError("运行来源必须是 hardware 或 simulation")
    if view not in {"radar", "navigation"}:
        raise RuntimeConfigError("运行视图必须是 radar 或 navigation")

    supplied = dict(overrides or {})
    config = deepcopy(RUNTIME_DEFAULTS)
    config.update(supplied)

    legacy_range = (config.get("min_range_m"), config.get("max_range_m"))
    if "hardware_min_range_m" not in supplied and "hardware_max_range_m" not in supplied:
        if legacy_range in {(0.08, 3.0), (0.08, 3)}:
            config["hardware_min_range_m"] = RUNTIME_DEFAULTS["hardware_min_range_m"]
            config["hardware_max_range_m"] = RUNTIME_DEFAULTS["hardware_max_range_m"]
        else:
            config["hardware_min_range_m"] = config.get("min_range_m", RUNTIME_DEFAULTS["min_range_m"])
            config["hardware_max_range_m"] = config.get("max_range_m", RUNTIME_DEFAULTS["max_range_m"])
    config["min_range_m"] = config["hardware_min_range_m"]
    config["max_range_m"] = config["hardware_max_range_m"]

    mode_defaults = {
        "hardware": (100, 100, 0.02),
        "simulation": (180, 280, 0.04),
    }
    mode_prefix = f"{source}_map_"
    map_keys = ("map_width_cells", "map_height_cells", "map_resolution_m")
    mode_map_keys = (mode_prefix + "width_cells", mode_prefix + "height_cells", mode_prefix + "resolution_m")
    supplied_map = tuple(supplied.get(key) for key in map_keys)
    legacy_hardware_map = (100, 100, 0.02)
    if any(key in supplied for key in mode_map_keys):
        config["map_width_cells"] = supplied.get(mode_prefix + "width_cells", mode_defaults[source][0])
        config["map_height_cells"] = supplied.get(mode_prefix + "height_cells", mode_defaults[source][1])
        config["map_resolution_m"] = supplied.get(mode_prefix + "resolution_m", mode_defaults[source][2])
    elif not any(key in supplied for key in map_keys) or (
        prefer_mode_defaults and source == "simulation" and supplied_map == legacy_hardware_map
    ):
        config["map_width_cells"] = mode_defaults[source][0]
        config["map_height_cells"] = mode_defaults[source][1]
        config["map_resolution_m"] = mode_defaults[source][2]

    for key in (
        "observation_reorder_s", "hardware_settle_s", "safety_clearance_m",
        "safety_max_observation_age_s", "safety_blind_timeout_s",
        "max_timing_position_error_m", "clock_drift_bound_ppm",
        "irq_timestamp_uncertainty_ms", "sync_interval_s", "sync_max_age_s",
        "keepalive_interval_s", "period_tolerance", "scan_gap_factor",
        "hardware_sample_rate_hz", "sample_rate_hz", "display_radius_m",
        "radar_period_s", "map_resolution_m", "robot_radius_m", "path_turn_penalty",
        "simulation_sample_rate_hz", "simulation_speed", "simulation_settle_s",
        "simulation_min_range_m", "simulation_max_range_m", "mapping_min_evidence",
        "mapping_resolution_weight", "mapping_poll_budget_ms", "max_scan_gap_deg",
        "scan_gap_factor", "scan_gap_hard_limit_deg", "safety_max_angle_error_deg",
        "fusion_delay_ms", "sync_interval_s", "sync_max_age_s", "keepalive_interval_s",
    ):
        _set_number(config, key, nonnegative=True)
    for key in ("min_range_m", "max_range_m", "hardware_min_range_m", "hardware_max_range_m"):
        _set_number(config, key, nonnegative=True)
    if config["min_range_m"] >= config["max_range_m"]:
        raise RuntimeConfigError("可信距离范围下限必须小于上限")
    if config["hardware_min_range_m"] >= config["hardware_max_range_m"]:
        raise RuntimeConfigError("硬件可信距离范围下限必须小于上限")
    if config["simulation_min_range_m"] >= config["simulation_max_range_m"]:
        raise RuntimeConfigError("仿真距离范围下限必须小于上限")

    for key in ("map_width_cells", "map_height_cells"):
        try:
            value = int(config[key])
        except (TypeError, ValueError, OverflowError) as exc:
            raise RuntimeConfigError(f"配置 {key} 必须是正整数") from exc
        if value <= 0 or value > 4000:
            raise RuntimeConfigError(f"配置 {key} 超出有效范围")
        config[key] = value
    for key in ("pixel_min", "pixel_max"):
        try:
            value = int(config[key])
        except (TypeError, ValueError, OverflowError) as exc:
            raise RuntimeConfigError(f"配置 {key} 必须是整数") from exc
        if not 0 <= value <= 65535:
            raise RuntimeConfigError(f"配置 {key} 超出有效范围")
        config[key] = value
    if config["pixel_min"] >= config["pixel_max"]:
        raise RuntimeConfigError("像素范围下限必须小于上限")

    try:
        legacy_baudrate = int(config.get("baudrate", RUNTIME_DEFAULTS["radar_baudrate"]))
        config["radar_baudrate"] = int(config.get("radar_baudrate", legacy_baudrate))
        config["chassis_baudrate"] = int(config.get("chassis_baudrate", RUNTIME_DEFAULTS["chassis_baudrate"]))
    except (TypeError, ValueError, OverflowError) as exc:
        raise RuntimeConfigError("串口波特率必须是正整数") from exc
    if config["radar_baudrate"] <= 0 or config["chassis_baudrate"] <= 0:
        raise RuntimeConfigError("串口波特率必须是正整数")
    config["baudrate"] = config["radar_baudrate"]
    config["exposure_index"] = int(config.get("exposure_index", 3))
    config["actual_exposure_index"] = int(config.get("actual_exposure_index", config["exposure_index"]))
    if not 0 <= config["exposure_index"] <= 13 or not 0 <= config["actual_exposure_index"] <= 13:
        raise RuntimeConfigError("曝光档位必须在 0 到 13 之间")
    config["clockwise"] = bool(config.get("clockwise", True))
    signs = config.get("wheel_signs")
    if not isinstance(signs, (list, tuple)) or len(signs) != 4:
        raise RuntimeConfigError("wheel_signs 必须包含四个元素")
    try:
        config["wheel_signs"] = [1 if int(value) >= 0 else -1 for value in signs]
    except (TypeError, ValueError, OverflowError) as exc:
        raise RuntimeConfigError("wheel_signs 必须是整数序列") from exc

    for key in ("angle_offset_deg", "radar_offset_x_m", "radar_offset_y_m", "radar_offset_yaw_deg"):
        _set_number(config, key)
    for key in (
        "chassis_ack_timeout_s", "chassis_action_timeout_s", "chassis_stop_timeout_s",
        "chassis_stop_status_delay_s", "chassis_max_translation_m",
        "chassis_max_rotation_rad",
    ):
        _set_number(config, key, nonnegative=True)
    for key in ("chassis_min_counts", "chassis_max_counts", "chassis_max_line_bytes"):
        try:
            value = int(config.get(key))
        except (TypeError, ValueError, OverflowError) as exc:
            raise RuntimeConfigError(f"配置 {key} 必须是正整数") from exc
        if value <= 0:
            raise RuntimeConfigError(f"配置 {key} 必须是正整数")
        config[key] = value
    if config["chassis_min_counts"] > config["chassis_max_counts"]:
        raise RuntimeConfigError("底盘 CNT 最小值不能大于最大值")
    config["chassis_speed_validated"] = bool(config.get("chassis_speed_validated", False))
    config["chassis_braking_validated"] = bool(config.get("chassis_braking_validated", False))
    calibration = config.get("chassis_calibration", {})
    if not isinstance(calibration, Mapping):
        raise RuntimeConfigError("chassis_calibration 必须是对象")
    normalized_calibration = {}
    for mode in "WSADQEZCRF":
        entry = calibration.get(mode, {"status": "uncalibrated"})
        if not isinstance(entry, Mapping):
            raise RuntimeConfigError(f"底盘 {mode} 标定项必须是对象")
        normalized = dict(entry)
        normalized["status"] = str(normalized.get("status", "uncalibrated")).lower()
        if normalized["status"] not in {"uncalibrated", "validated", "ready", "confirmed"}:
            raise RuntimeConfigError(f"底盘 {mode} 标定状态无效")
        for key in ("counts_per_m", "counts_per_rad", "counts_per_unit", "request_bias_counts", "min_counts", "max_counts", "uncertainty_m", "uncertainty_rad"):
            if key in normalized and normalized[key] is not None:
                try:
                    value = float(normalized[key])
                except (TypeError, ValueError, OverflowError) as exc:
                    raise RuntimeConfigError(f"底盘 {mode} 标定字段 {key} 必须是有限数值") from exc
                if not math.isfinite(value):
                    raise RuntimeConfigError(f"底盘 {mode} 标定字段 {key} 必须是有限数值")
                normalized[key] = value
        normalized_calibration[mode] = normalized
    config["chassis_calibration"] = normalized_calibration
    try:
        config["mapping_queue_size"] = int(config.get("mapping_queue_size", 2))
    except (TypeError, ValueError, OverflowError) as exc:
        raise RuntimeConfigError("mapping_queue_size 必须是正整数") from exc
    if config["mapping_queue_size"] < 1:
        raise RuntimeConfigError("mapping_queue_size 必须是正整数")
    config["measurement_mode"] = str(config.get("measurement_mode", "fffe"))
    if config["measurement_mode"] not in {"fffe", "raw2", "timestamped_ascii"}:
        raise RuntimeConfigError("measurement_mode 不受支持")
    config["runtime_source"] = source
    config["runtime_view"] = view
    config["auto_enabled"] = source == "simulation" and view == "navigation"
    config["config_fingerprint"] = configuration_fingerprint(config)
    return config


def build_navigation_engine(config: Mapping[str, object]):
    from navigation_core import NavigationEngine, OccupancyGrid

    resolved = resolve_runtime_config(
        str(config.get("runtime_source", "simulation")),
        str(config.get("runtime_view", "navigation")),
        config,
    )
    grid = OccupancyGrid(
        int(resolved["map_width_cells"]),
        int(resolved["map_height_cells"]),
        float(resolved["map_resolution_m"]),
    )
    return NavigationEngine(
        grid,
        max_range_m=float(resolved["max_range_m"] if resolved["runtime_source"] == "hardware" else resolved["simulation_max_range_m"]),
        robot_radius_m=float(resolved["robot_radius_m"]),
        sensor_offset_x_m=float(resolved["radar_offset_x_m"]),
        sensor_offset_y_m=float(resolved["radar_offset_y_m"]),
        sensor_offset_yaw_rad=math.radians(float(resolved["radar_offset_yaw_deg"])),
        min_range_m=float(resolved["min_range_m"] if resolved["runtime_source"] == "hardware" else resolved["simulation_min_range_m"]),
        path_turn_penalty=float(resolved["path_turn_penalty"]),
    )


def configuration_fingerprint(config: Mapping[str, object]) -> str:
    serializable = {}
    for key, value in sorted(config.items()):
        if key in {"calibration_error", "config_fingerprint"}:
            continue
        if isinstance(value, Path):
            value = str(value)
        try:
            json.dumps(value, sort_keys=True)
        except TypeError:
            value = repr(value)
        serializable[key] = value
    payload = json.dumps(serializable, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]
