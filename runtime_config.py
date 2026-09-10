from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping


CHASSIS_TRANSLATION_MODES = tuple("WSADQEZC")
CHASSIS_ROTATION_MODES = tuple("RF")
CHASSIS_COUNTS_PER_MM = {
    "W": 7.1775,
    "S": 7.1317,
    "A": 7.2372,
    "D": 7.3864,
    "Q": 10.1756,
    "E": 9.1232,
    "Z": 9.6476,
    "C": 10.1874,
}
CHASSIS_FIXED_COUNTS_PER_MM = {
    "W": 71775,
    "S": 71317,
    "A": 72372,
    "D": 73864,
    "Q": 101756,
    "E": 91232,
    "Z": 96476,
    "C": 101874,
}
CHASSIS_BRAKE_COUNT_RANGES = {
    "W": (700, 1400),
    "S": (700, 1400),
    "A": (700, 1400),
    "D": (700, 1400),
    "Q": (600, 1200),
    "E": (600, 1200),
    "Z": (600, 1200),
    "C": (600, 1200),
}
CHASSIS_CALIBRATION_SOURCE = "control/标定/20260909_现场长距离标定结果.md"


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
    "hardware_sample_rate_hz": 100.0,
    "exposure_index": 5,
    "actual_exposure_index": 5,
    "calibration_firmware_version": "CCD-PEAK-RAW-CAL-3.0",
    "measurement_firmware_version": "MEASUREMENT_SYNC_CAL_V3",
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
    "formal_scan_stable_periods": 1,
    "min_scan_points": 40,
    "period_tolerance": 0.05,
    "max_scan_gap_deg": 25.0,
    "scan_gap_factor": 2.5,
    "scan_gap_hard_limit_deg": 45.0,
    "display_preview_min_points": 40,
    "display_preview_min_ratio": 0.70,
    "measurement_port": "",
    "rotation_port": "",
    "chassis_port": "",
    "baudrate": 115200,
    "radar_baudrate": 115200,
    "chassis_baudrate": 9600,
    "measurement_mode": "fffe",
    "ccd_command": "@c0071#@",
    "sample_rate_hz": 100.0,
    "min_range_m": 0.15,
    "max_range_m": 1.00,
    "hardware_min_range_m": 0.15,
    "hardware_max_range_m": 1.00,
    "display_radius_m": 1.10,
    "angle_offset_deg": 0.0,
    "clockwise": False,
    "radar_period_s": 1.5,
    "fusion_delay_ms": 80,
    "map_resolution_m": 0.02,
    "map_width_cells": 120,
    "map_height_cells": 120,
    "radar_offset_x_m": 0.0,
    "radar_offset_y_m": 0.0,
    "radar_offset_yaw_deg": 0.0,
    "robot_radius_m": 0.15,
    "path_turn_penalty": 0.75,
    "chassis_protocol_version": "MECANUM UNIVERSAL V6.3 COMM",
    "chassis_capability_mode": "unknown",
    "chassis_firmware_confirmed": False,
    "chassis_result_recovery": False,
    "chassis_idle_preflight": False,
    "chassis_preferred_translation_unit": "MM",
    "chassis_config1_file": "",
    "chassis_tx_padding_spaces": 0,
    "chassis_raw_log_enabled": True,
    "chassis_raw_log_file": "logs/chassis_serial.jsonl",
    "chassis_rx_silence_before_ping_s": 0.5,
    "chassis_rx_silence_after_pong_s": 0.25,
    "chassis_silence_wait_timeout_s": 4.0,
    "chassis_ping_timeout_s": 2.0,
    "chassis_ack_timeout_s": 2.0,
    "chassis_action_timeout_s": 12.0,
    "chassis_total_timeout_s": 12.0,
    "chassis_stop_timeout_s": 3.0,
    "chassis_stop_status_delay_s": 0.35,
    "chassis_tx_max_command_bytes": 47,
    "chassis_max_line_bytes": 1024,
    "chassis_min_counts": 1,
    "chassis_max_counts": 2000,
    "chassis_max_translation_m": 0.20,
    "chassis_max_rotation_rad": 0.5,
    "chassis_speed_validated": False,
    "chassis_distance_control": False,
    "chassis_braking_validated": False,
    "chassis_translation_capabilities": {
        mode: {
            "fixed_counts_per_mm": CHASSIS_FIXED_COUNTS_PER_MM[mode],
            "counts_per_mm": CHASSIS_COUNTS_PER_MM[mode],
            "coefficient_status": "initial",
            "coefficient_source": CHASSIS_CALIBRATION_SOURCE,
            "brake_min_counts": CHASSIS_BRAKE_COUNT_RANGES[mode][0],
            "brake_max_counts": CHASSIS_BRAKE_COUNT_RANGES[mode][1],
            "motion_range_validated": False,
            "validated_min_mm": None,
            "validated_max_mm": None,
            "uncertainty_m": None,
            "enabled": True,
        }
        for mode in CHASSIS_TRANSLATION_MODES
    },
    "chassis_rotation_capabilities": {
        mode: {
            "unit": "CNT",
            "status": "unvalidated",
            "enabled": False,
            "counts_per_rad": None,
            "uncertainty_rad": None,
            "validated_min_counts": None,
            "validated_max_counts": None,
        }
        for mode in CHASSIS_ROTATION_MODES
    },
    "chassis_calibration": {
        **{
            mode: {
                "status": "initial",
                "counts_per_m": CHASSIS_COUNTS_PER_MM[mode] * 1000.0,
                "counts_per_unit": CHASSIS_COUNTS_PER_MM[mode] * 1000.0,
                "request_bias_counts": 0.0,
                "min_counts": CHASSIS_BRAKE_COUNT_RANGES[mode][0],
                "max_counts": CHASSIS_BRAKE_COUNT_RANGES[mode][1],
                "uncertainty_m": None,
            }
            for mode in CHASSIS_TRANSLATION_MODES
        },
        **{
            mode: {
                "status": "unvalidated",
                "counts_per_rad": None,
                "uncertainty_rad": None,
            }
            for mode in CHASSIS_ROTATION_MODES
        },
    },
    "wheel_output_scale": 1000,
    "wheel_signs": [1, 1, 1, 1],
    "simulation_sample_rate_hz": 100.0,
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


def _strict_bool(config: Mapping[str, object], key: str) -> bool:
    value = config.get(key)
    if type(value) is not bool:
        raise RuntimeConfigError(f"配置 {key} 必须是布尔值")
    return value


def _strict_int(value: object, label: str, *, minimum: int | None = None,
                maximum: int | None = None) -> int:
    if type(value) is not int:
        raise RuntimeConfigError(f"{label} 必须是整数")
    if minimum is not None and value < minimum:
        raise RuntimeConfigError(f"{label} 小于有效范围")
    if maximum is not None and value > maximum:
        raise RuntimeConfigError(f"{label} 超出有效范围")
    return value


def _strict_number(value: object, label: str, *, positive: bool = False,
                   nonnegative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeConfigError(f"{label} 必须是有限数值")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise RuntimeConfigError(f"{label} 必须是有限数值")
    if positive and parsed <= 0:
        raise RuntimeConfigError(f"{label} 必须大于 0")
    if nonnegative and parsed < 0:
        raise RuntimeConfigError(f"{label} 不能为负数")
    return parsed


def _require_exact_modes(value: object, label: str, modes: tuple[str, ...]) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise RuntimeConfigError(f"{label} 必须是对象")
    expected = set(modes)
    actual = set(value)
    if actual != expected:
        missing = "".join(mode for mode in modes if mode not in actual)
        extra = ",".join(sorted(str(mode) for mode in actual if mode not in expected))
        detail = []
        if missing:
            detail.append(f"缺少 {missing}")
        if extra:
            detail.append(f"包含未知方向 {extra}")
        raise RuntimeConfigError(f"{label} 方向无效：{'；'.join(detail)}")
    return value


def _normalize_translation_capabilities(value: object) -> dict[str, dict[str, object]]:
    table = _require_exact_modes(value, "chassis_translation_capabilities", CHASSIS_TRANSLATION_MODES)
    required = {
        "fixed_counts_per_mm", "counts_per_mm", "coefficient_status",
        "coefficient_source", "brake_min_counts", "brake_max_counts",
        "motion_range_validated", "validated_min_mm", "validated_max_mm",
        "uncertainty_m", "enabled",
    }
    normalized_table: dict[str, dict[str, object]] = {}
    for mode in CHASSIS_TRANSLATION_MODES:
        raw = table[mode]
        if not isinstance(raw, Mapping):
            raise RuntimeConfigError(f"底盘 {mode} 平移能力必须是对象")
        fields = set(raw)
        if fields != required:
            missing = ",".join(sorted(required - fields))
            extra = ",".join(sorted(str(key) for key in fields - required))
            detail = []
            if missing:
                detail.append(f"缺少字段 {missing}")
            if extra:
                detail.append(f"未知字段 {extra}")
            raise RuntimeConfigError(f"底盘 {mode} 平移能力无效：{'；'.join(detail)}")

        fixed = _strict_int(raw["fixed_counts_per_mm"], f"底盘 {mode} fixed_counts_per_mm",
                            minimum=1, maximum=10_000_000)
        coefficient = _strict_number(raw["counts_per_mm"], f"底盘 {mode} counts_per_mm", positive=True)
        if coefficient > 1000.0:
            raise RuntimeConfigError(f"底盘 {mode} counts_per_mm 超出有效范围")
        expected_fixed = int(math.floor(coefficient * 10000.0 + 0.5))
        if fixed != expected_fixed:
            raise RuntimeConfigError(f"底盘 {mode} 定点系数与 counts_per_mm 不一致")

        coefficient_status = raw["coefficient_status"]
        if not isinstance(coefficient_status, str) or coefficient_status not in {"initial", "validated"}:
            raise RuntimeConfigError(f"底盘 {mode} coefficient_status 无效")
        source = raw["coefficient_source"]
        if not isinstance(source, str) or not source.strip():
            raise RuntimeConfigError(f"底盘 {mode} coefficient_source 必须是非空文本")

        brake_min = _strict_int(raw["brake_min_counts"], f"底盘 {mode} brake_min_counts",
                                minimum=1, maximum=2_147_483_647)
        brake_max = _strict_int(raw["brake_max_counts"], f"底盘 {mode} brake_max_counts",
                                minimum=1, maximum=2_147_483_647)
        if brake_min > brake_max:
            raise RuntimeConfigError(f"底盘 {mode} 刹车计数下限不能大于上限")

        range_validated = raw["motion_range_validated"]
        enabled = raw["enabled"]
        if type(range_validated) is not bool:
            raise RuntimeConfigError(f"底盘 {mode} motion_range_validated 必须是布尔值")
        if type(enabled) is not bool:
            raise RuntimeConfigError(f"底盘 {mode} enabled 必须是布尔值")

        minimum_mm = raw["validated_min_mm"]
        maximum_mm = raw["validated_max_mm"]
        uncertainty = raw["uncertainty_m"]
        if range_validated:
            minimum_mm = _strict_number(minimum_mm, f"底盘 {mode} validated_min_mm", positive=True)
            maximum_mm = _strict_number(maximum_mm, f"底盘 {mode} validated_max_mm", positive=True)
            if minimum_mm > maximum_mm:
                raise RuntimeConfigError(f"底盘 {mode} 已验证距离下限不能大于上限")
            uncertainty = _strict_number(uncertainty, f"底盘 {mode} uncertainty_m", nonnegative=True)
        else:
            if minimum_mm is not None or maximum_mm is not None:
                raise RuntimeConfigError(f"底盘 {mode} 未验证动作范围不能填写距离边界")
            if uncertainty is not None:
                uncertainty = _strict_number(uncertainty, f"底盘 {mode} uncertainty_m", nonnegative=True)

        normalized_table[mode] = {
            "fixed_counts_per_mm": fixed,
            "counts_per_mm": coefficient,
            "coefficient_status": coefficient_status,
            "coefficient_source": source.strip(),
            "brake_min_counts": brake_min,
            "brake_max_counts": brake_max,
            "motion_range_validated": range_validated,
            "validated_min_mm": minimum_mm,
            "validated_max_mm": maximum_mm,
            "uncertainty_m": uncertainty,
            "enabled": enabled,
        }
    return normalized_table


def _normalize_rotation_capabilities(value: object) -> dict[str, dict[str, object]]:
    table = _require_exact_modes(value, "chassis_rotation_capabilities", CHASSIS_ROTATION_MODES)
    required = {
        "unit", "status", "enabled", "counts_per_rad", "uncertainty_rad",
        "validated_min_counts", "validated_max_counts",
    }
    normalized_table: dict[str, dict[str, object]] = {}
    for mode in CHASSIS_ROTATION_MODES:
        raw = table[mode]
        if not isinstance(raw, Mapping):
            raise RuntimeConfigError(f"底盘 {mode} 旋转能力必须是对象")
        fields = set(raw)
        if fields != required:
            missing = ",".join(sorted(required - fields))
            extra = ",".join(sorted(str(key) for key in fields - required))
            detail = []
            if missing:
                detail.append(f"缺少字段 {missing}")
            if extra:
                detail.append(f"未知字段 {extra}")
            raise RuntimeConfigError(f"底盘 {mode} 旋转能力无效：{'；'.join(detail)}")
        if not isinstance(raw["unit"], str) or raw["unit"] != "CNT":
            raise RuntimeConfigError(f"底盘 {mode} 旋转单位必须是 CNT")
        if not isinstance(raw["status"], str) or raw["status"] not in {"unvalidated", "validated"}:
            raise RuntimeConfigError(f"底盘 {mode} 旋转状态无效")
        if type(raw["enabled"]) is not bool:
            raise RuntimeConfigError(f"底盘 {mode} enabled 必须是布尔值")

        counts_per_rad = raw["counts_per_rad"]
        uncertainty = raw["uncertainty_rad"]
        minimum_counts = raw["validated_min_counts"]
        maximum_counts = raw["validated_max_counts"]
        if raw["status"] == "validated":
            counts_per_rad = _strict_number(counts_per_rad, f"底盘 {mode} counts_per_rad", positive=True)
            uncertainty = _strict_number(uncertainty, f"底盘 {mode} uncertainty_rad", nonnegative=True)
            minimum_counts = _strict_int(minimum_counts, f"底盘 {mode} validated_min_counts",
                                         minimum=1, maximum=2_147_483_647)
            maximum_counts = _strict_int(maximum_counts, f"底盘 {mode} validated_max_counts",
                                         minimum=1, maximum=2_147_483_647)
            if minimum_counts > maximum_counts:
                raise RuntimeConfigError(f"底盘 {mode} 已验证计数下限不能大于上限")
        else:
            if raw["enabled"]:
                raise RuntimeConfigError(f"底盘 {mode} 未验证时不能启用")
            if any(item is not None for item in (counts_per_rad, uncertainty, minimum_counts, maximum_counts)):
                raise RuntimeConfigError(f"底盘 {mode} 未验证时不能填写旋转换算或范围")

        normalized_table[mode] = {
            "unit": "CNT",
            "status": raw["status"],
            "enabled": raw["enabled"],
            "counts_per_rad": counts_per_rad,
            "uncertainty_rad": uncertainty,
            "validated_min_counts": minimum_counts,
            "validated_max_counts": maximum_counts,
        }
    return normalized_table


def _finite(config: Mapping[str, object], key: str, *, positive: bool = False,
            nonnegative: bool = False) -> float:
    if isinstance(config.get(key), bool):
        raise RuntimeConfigError(f"配置 {key} 必须是有限数值")
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
    validate_port_assignments: bool = True,
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
    profile_file = config.get("chassis_config1_file", "")
    if not isinstance(profile_file, str):
        raise RuntimeConfigError("chassis_config1_file 必须是项目内相对路径")
    config["chassis_tx_padding_spaces"] = _strict_int(
        config.get("chassis_tx_padding_spaces", 0), "发送前导空格", minimum=0, maximum=64
    )
    if profile_file:
        from chassis_config1 import load_profile, values_crc
        try:
            values, coefficients = load_profile(profile_file)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise RuntimeConfigError(f"底盘参数文件加载失败：{exc}") from exc
        config["chassis_config1_values"] = values
        config["chassis_config1_crc"] = values_crc(values)
        config["chassis_preferred_translation_unit"] = "CNT"
        table = deepcopy(config["chassis_translation_capabilities"])
        for index, mode in enumerate("WSADQEZC"):
            table[mode]["counts_per_mm"] = coefficients[mode]
            table[mode]["fixed_counts_per_mm"] = int(math.floor(coefficients[mode]*10000 + 0.5))
            table[mode]["coefficient_source"] = profile_file
            table[mode]["brake_min_counts"] = values[44 + 4*index]
            table[mode]["brake_max_counts"] = values[45 + 4*index]
        config["chassis_translation_capabilities"] = table
        timeout = max(float(config["chassis_total_timeout_s"]), 14.0, values[76]/1000 + 6.0)
        config["chassis_action_timeout_s"] = timeout
        config["chassis_total_timeout_s"] = timeout
    else:
        config.pop("chassis_config1_values", None)
        config.pop("chassis_config1_crc", None)

    for key in (
        "synchronized_acquisition", "clockwise", "chassis_firmware_confirmed",
        "chassis_speed_validated", "chassis_braking_validated", "chassis_raw_log_enabled",
        "chassis_result_recovery", "chassis_idle_preflight", "chassis_distance_control",
    ):
        config[key] = _strict_bool(config, key)

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
        "hardware": (120, 120, 0.02),
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
        "display_preview_min_ratio",
    ):
        _set_number(config, key, nonnegative=True)
    for key in ("min_range_m", "max_range_m", "hardware_min_range_m", "hardware_max_range_m"):
        _set_number(config, key, nonnegative=True)
    speed_bound = config.get("safety_speed_upper_bound_mps")
    if speed_bound is not None:
        config["safety_speed_upper_bound_mps"] = _strict_number(
            speed_bound, "底盘实际速度上界", positive=True,
        )
    stop_distance = config.get("safety_stop_distance_m")
    if stop_distance is not None:
        config["safety_stop_distance_m"] = _strict_number(
            stop_distance, "底盘完整停止距离", nonnegative=True,
        )
    if config["chassis_speed_validated"] and config["safety_speed_upper_bound_mps"] is None:
        raise RuntimeConfigError("速度上界标记为已测时必须填写 safety_speed_upper_bound_mps")
    if config["chassis_braking_validated"] and config["safety_stop_distance_m"] is None:
        raise RuntimeConfigError("停止距离标记为已测时必须填写 safety_stop_distance_m")
    if config["robot_radius_m"] <= 0:
        raise RuntimeConfigError("车体安全包络半径必须大于 0")
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

    for key in ("measurement_port", "rotation_port", "chassis_port"):
        value = config.get(key, "")
        if not isinstance(value, str):
            raise RuntimeConfigError(f"配置 {key} 必须是文本")
        config[key] = value.strip()
    selected_ports = [
        (key, config[key].casefold())
        for key in ("measurement_port", "rotation_port", "chassis_port")
        if config[key]
    ]
    if validate_port_assignments and len({value for _, value in selected_ports}) != len(selected_ports):
        raise RuntimeConfigError("测距、旋转和底盘串口不能重复选择")

    legacy_baudrate = config.get("baudrate", RUNTIME_DEFAULTS["radar_baudrate"])
    config["radar_baudrate"] = _strict_int(
        config.get("radar_baudrate", legacy_baudrate), "雷达串口波特率", minimum=1,
    )
    config["chassis_baudrate"] = _strict_int(
        config.get("chassis_baudrate", RUNTIME_DEFAULTS["chassis_baudrate"]),
        "底盘串口波特率", minimum=1,
    )
    config["baudrate"] = config["radar_baudrate"]
    config["exposure_index"] = _strict_int(
        config.get("exposure_index", 5), "曝光档位", minimum=0, maximum=13,
    )
    config["actual_exposure_index"] = _strict_int(
        config.get("actual_exposure_index", config["exposure_index"]),
        "实际曝光档位", minimum=0, maximum=13,
    )
    config["min_scan_points"] = _strict_int(
        config.get("min_scan_points", RUNTIME_DEFAULTS["min_scan_points"]),
        "完整扫描最少点数",
        minimum=3,
        maximum=1000,
    )
    config["display_preview_min_points"] = _strict_int(
        config.get("display_preview_min_points", RUNTIME_DEFAULTS["display_preview_min_points"]),
        "预览显示最少点数",
        minimum=1,
        maximum=1000,
    )
    signs = config.get("wheel_signs")
    if not isinstance(signs, (list, tuple)) or len(signs) != 4:
        raise RuntimeConfigError("wheel_signs 必须包含四个元素")
    try:
        config["wheel_signs"] = [1 if int(value) >= 0 else -1 for value in signs]
    except (TypeError, ValueError, OverflowError) as exc:
        raise RuntimeConfigError("wheel_signs 必须是整数序列") from exc

    for key in ("angle_offset_deg", "radar_offset_x_m", "radar_offset_y_m", "radar_offset_yaw_deg"):
        _set_number(config, key)
    if "chassis_action_timeout_s" in supplied and "chassis_total_timeout_s" not in supplied:
        config["chassis_total_timeout_s"] = config["chassis_action_timeout_s"]
    elif "chassis_total_timeout_s" in supplied and "chassis_action_timeout_s" not in supplied:
        config["chassis_action_timeout_s"] = config["chassis_total_timeout_s"]
    for key in (
        "chassis_rx_silence_before_ping_s", "chassis_rx_silence_after_pong_s",
        "chassis_silence_wait_timeout_s", "chassis_ping_timeout_s",
        "chassis_ack_timeout_s", "chassis_action_timeout_s", "chassis_total_timeout_s",
        "chassis_stop_timeout_s",
    ):
        _set_number(config, key, positive=True)
    for key in (
        "chassis_stop_status_delay_s", "chassis_max_translation_m",
        "chassis_max_rotation_rad",
    ):
        _set_number(config, key, nonnegative=True)
    if not math.isclose(
        config["chassis_action_timeout_s"], config["chassis_total_timeout_s"],
        rel_tol=0.0, abs_tol=1e-9,
    ):
        raise RuntimeConfigError("chassis_action_timeout_s 与 chassis_total_timeout_s 必须一致")
    if config["chassis_silence_wait_timeout_s"] < config["chassis_rx_silence_before_ping_s"]:
        raise RuntimeConfigError("底盘静默等待上限不能短于 PING 前静默时间")
    if config["chassis_total_timeout_s"] <= config["chassis_ack_timeout_s"]:
        raise RuntimeConfigError("底盘总时限必须大于 ACK 时限")

    for key in ("chassis_min_counts", "chassis_max_counts"):
        config[key] = _strict_int(
            config.get(key), f"配置 {key}", minimum=1, maximum=2_147_483_647,
        )
    config["chassis_tx_max_command_bytes"] = _strict_int(
        config.get("chassis_tx_max_command_bytes"),
        "底盘 TX 命令上限", minimum=1, maximum=47,
    )
    config["chassis_max_line_bytes"] = _strict_int(
        config.get("chassis_max_line_bytes"),
        "底盘 RX 报告上限", minimum=64, maximum=65536,
    )
    if config["chassis_min_counts"] > config["chassis_max_counts"]:
        raise RuntimeConfigError("底盘 CNT 最小值不能大于最大值")

    capability_mode = config.get("chassis_capability_mode")
    if not isinstance(capability_mode, str) or capability_mode not in {"unknown", "cnt_only", "mm_ping_v1"}:
        raise RuntimeConfigError("chassis_capability_mode 必须是 unknown、cnt_only 或 mm_ping_v1")
    preferred_unit = config.get("chassis_preferred_translation_unit")
    if not isinstance(preferred_unit, str) or preferred_unit not in {"MM", "CNT"}:
        raise RuntimeConfigError("chassis_preferred_translation_unit 必须是 MM 或 CNT")
    if capability_mode == "cnt_only" and preferred_unit != "CNT":
        raise RuntimeConfigError("cnt_only 底盘只能使用 CNT 平移请求")
    if config["chassis_firmware_confirmed"] and capability_mode == "unknown":
        raise RuntimeConfigError("底盘固件已确认时必须选择明确的能力模式")
    config["chassis_capability_mode"] = capability_mode
    config["chassis_preferred_translation_unit"] = preferred_unit

    raw_log_file = config.get("chassis_raw_log_file")
    if not isinstance(raw_log_file, str) or not raw_log_file.strip():
        raise RuntimeConfigError("chassis_raw_log_file 必须是非空相对路径")
    try:
        raw_log_path = Path(raw_log_file.strip())
    except (TypeError, ValueError) as exc:
        raise RuntimeConfigError("chassis_raw_log_file 必须是非空相对路径") from exc
    if (
        raw_log_path.is_absolute()
        or raw_log_path.drive
        or raw_log_path == Path(".")
        or ".." in raw_log_path.parts
    ):
        raise RuntimeConfigError("chassis_raw_log_file 必须位于项目目录内")
    config["chassis_raw_log_file"] = raw_log_path.as_posix()

    config["chassis_translation_capabilities"] = _normalize_translation_capabilities(
        config.get("chassis_translation_capabilities")
    )
    config["chassis_rotation_capabilities"] = _normalize_rotation_capabilities(
        config.get("chassis_rotation_capabilities")
    )
    for mode, entry in config["chassis_translation_capabilities"].items():
        if entry["motion_range_validated"]:
            maximum_m = float(entry["validated_max_mm"]) / 1000.0
            if maximum_m > config["chassis_max_translation_m"] + 1e-12:
                raise RuntimeConfigError(f"底盘 {mode} 已验证距离超过全局平移上限")

    calibration = config.get("chassis_calibration", {})
    if not isinstance(calibration, Mapping):
        raise RuntimeConfigError("chassis_calibration 必须是对象")
    extra_calibration_modes = set(calibration) - set(CHASSIS_TRANSLATION_MODES + CHASSIS_ROTATION_MODES)
    if extra_calibration_modes:
        raise RuntimeConfigError("chassis_calibration 包含未知方向")
    normalized_calibration = {}
    for mode in CHASSIS_TRANSLATION_MODES + CHASSIS_ROTATION_MODES:
        entry = calibration.get(mode, deepcopy(RUNTIME_DEFAULTS["chassis_calibration"][mode]))
        if not isinstance(entry, Mapping):
            raise RuntimeConfigError(f"底盘 {mode} 标定项必须是对象")
        normalized = dict(entry)
        status = normalized.get("status", "uncalibrated")
        if not isinstance(status, str) or status not in {"unvalidated", "uncalibrated", "initial", "validated", "ready", "confirmed"}:
            raise RuntimeConfigError(f"底盘 {mode} 标定状态无效")
        normalized["status"] = status
        for key in ("counts_per_m", "counts_per_rad", "counts_per_unit", "request_bias_counts", "min_counts", "max_counts", "uncertainty_m", "uncertainty_rad"):
            if key in normalized and normalized[key] is not None:
                normalized[key] = _strict_number(
                    normalized[key], f"底盘 {mode} 标定字段 {key}",
                    positive=key in {"counts_per_m", "counts_per_rad", "counts_per_unit", "min_counts", "max_counts"},
                    nonnegative=key in {"uncertainty_m", "uncertainty_rad"},
                )
        if (
            normalized.get("min_counts") is not None
            and normalized.get("max_counts") is not None
            and normalized["min_counts"] > normalized["max_counts"]
        ):
            raise RuntimeConfigError(f"底盘 {mode} 标定计数下限不能大于上限")
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
        # The engine has no serial transport; validate device assignments when
        # connecting, so its construction cannot prevent configuration editing.
        validate_port_assignments=False,
    )
    grid = OccupancyGrid(
        int(resolved["map_width_cells"]),
        int(resolved["map_height_cells"]),
        float(resolved["map_resolution_m"]),
    )
    if resolved["runtime_source"] == "hardware":
        raw_capabilities = resolved.get("chassis_translation_capabilities", {})
        global_maximum = min(
            NavigationEngine.MAX_MOTION_SEGMENT_M,
            float(resolved.get("chassis_max_translation_m", NavigationEngine.MAX_MOTION_SEGMENT_M)),
        )
        translation_capabilities = {
            mode: {
                "enabled": bool(
                    isinstance(raw_capabilities, Mapping)
                    and isinstance(raw_capabilities.get(mode), Mapping)
                    and raw_capabilities[mode].get("enabled") is True
                    and raw_capabilities[mode].get("motion_range_validated") is True
                ),
                "min_m": (
                    float(raw_capabilities[mode]["validated_min_mm"]) / 1000.0
                    if isinstance(raw_capabilities, Mapping)
                    and isinstance(raw_capabilities.get(mode), Mapping)
                    and raw_capabilities[mode].get("enabled") is True
                    and raw_capabilities[mode].get("motion_range_validated") is True
                    else 0.0
                ),
                "max_m": (
                    min(float(raw_capabilities[mode]["validated_max_mm"]) / 1000.0, global_maximum)
                    if isinstance(raw_capabilities, Mapping)
                    and isinstance(raw_capabilities.get(mode), Mapping)
                    and raw_capabilities[mode].get("enabled") is True
                    and raw_capabilities[mode].get("motion_range_validated") is True
                    else 0.0
                ),
            }
            for mode in NavigationEngine.TRANSLATION_MODES
        }
        if resolved["chassis_distance_control"]:
            for mode, capability in translation_capabilities.items():
                entry = raw_capabilities[mode]
                counts_per_m = float(entry["counts_per_mm"]) * 1000.0
                capability.update(
                    enabled=entry["enabled"],
                    min_m=float(resolved["chassis_min_counts"]) / counts_per_m,
                    max_m=min(global_maximum, float(resolved["chassis_max_counts"]) / counts_per_m),
                )
    else:
        translation_capabilities = {
            mode: {"enabled": True, "min_m": 0.0, "max_m": NavigationEngine.MAX_MOTION_SEGMENT_M}
            for mode in NavigationEngine.TRANSLATION_MODES
        }
    return NavigationEngine(
        grid,
        unobserved_clear_range_m=1.0 if resolved['runtime_source'] == 'hardware' else 0.0,
        max_range_m=float(resolved["max_range_m"] if resolved["runtime_source"] == "hardware" else resolved["simulation_max_range_m"]),
        robot_radius_m=float(resolved["robot_radius_m"]),
        sensor_offset_x_m=float(resolved["radar_offset_x_m"]),
        sensor_offset_y_m=float(resolved["radar_offset_y_m"]),
        sensor_offset_yaw_rad=math.radians(float(resolved["radar_offset_yaw_deg"])),
        min_range_m=float(resolved["min_range_m"] if resolved["runtime_source"] == "hardware" else resolved["simulation_min_range_m"]),
        path_turn_penalty=float(resolved["path_turn_penalty"]),
        translation_capabilities=translation_capabilities,
        safety_clearance_m=float(resolved['safety_clearance_m']),
        safety_stop_distance_m=float(resolved.get('safety_stop_distance_m') or 0.0),
        safety_max_observation_age_s=float(resolved['safety_max_observation_age_s']),
        safety_speed_upper_bound_mps=(resolved.get('safety_speed_upper_bound_mps')
                                      if resolved['runtime_source'] == 'hardware' else None),
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
