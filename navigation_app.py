from __future__ import annotations

import argparse
from collections import deque
from copy import deepcopy
from datetime import datetime
import heapq
import math
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from app_utils import capture_window, enable_windows_dpi_awareness, load_json_config, save_json
from data_fusion import DeviceClock, SequenceMonitor, parse_timestamped_distance, parse_trigger
from chassis_controller import (
    ChassisController,
    ChassisMotionAdapter,
    ChassisState,
    MotionConversionError,
)
from chassis_log import ChassisTrafficLogger
from motion_safety import MotionSafetyGuard
from navigation_core import (
    HiddenWorld,
    NavigationEngine,
    OccupancyGrid,
    ScanPoint,
    VelocityCommand,
)
from mapping_runtime import MappingResult, MappingRuntime
from radar_app import COLORS, RadarCanvas
from radar_core import CalibrationModel, CCDFrameParser, MotorLineParser, RotationTracker, SlidingRate
from runtime_config import (
    RUNTIME_DEFAULTS,
    RuntimeConfigError,
    build_navigation_engine,
    resolve_runtime_config,
)
from serial_backend import SerialEndpoint, list_serial_ports
from scan_acquisition import polar_to_scan_point
from synchronized_acquisition import SynchronizedAcquisition
from virtual_hardware import HardwareSimulation


enable_windows_dpi_awareness()


APP_DIR = Path(__file__).resolve().parent
RADAR_CONFIG_PATH = APP_DIR / "radar_config.json"
NAV_CONFIG_PATH = APP_DIR / "navigation_config.json"

DEFAULT_CONFIG = deepcopy(RUNTIME_DEFAULTS)


def preview_item_angle_distance(item) -> tuple[float, float]:
    angle = item.angle_rad if hasattr(item, "angle_rad") else item[1]
    distance = item.distance_m if hasattr(item, "distance_m") else item[2]
    return float(angle), float(distance)


def preview_is_stable_for_display(
    preview,
    latest_points,
    *,
    min_points: int = 40,
    min_ratio_to_latest: float = 0.70,
) -> bool:
    preview_count = len(preview)
    if preview_count <= 0:
        return False
    latest_count = len(latest_points)
    if latest_count <= 0:
        return preview_count >= min_points
    required = max(min_points, math.ceil(latest_count * min_ratio_to_latest))
    return preview_count >= required


def load_configuration(
    source: str = "hardware",
    view: str = "radar",
    overrides: dict | None = None,
) -> dict:
    config = dict(DEFAULT_CONFIG)
    radar_config = load_json_config(RADAR_CONFIG_PATH, {})
    for key in (
        "measurement_port",
        "baudrate",
        "ccd_command",
        "sample_rate_hz",
        "min_range_m",
        "max_range_m",
        "hardware_min_range_m",
        "hardware_max_range_m",
        "calibration_model",
        "calibration_file",
        "angle_offset_deg",
        "clockwise",
        "exposure_index",
    ):
        if key in radar_config:
            config[key] = radar_config[key]
    if "motor_port" in radar_config:
        config["rotation_port"] = radar_config["motor_port"]
    if "ccd_parser" in radar_config:
        config["measurement_mode"] = radar_config["ccd_parser"]
    radar_calibration = radar_config.get("calibration", {})
    config["calibration"] = radar_calibration

    nav_config = load_json_config(NAV_CONFIG_PATH, {})
    config.update(nav_config)
    # Rotation direction is a physical radar property.  Keep one source of
    # truth so the radar and navigation entry points cannot mirror the same
    # sweep by loading conflicting saved values.
    if "clockwise" in radar_config:
        config["clockwise"] = radar_config["clockwise"]
    if "rotation_port" not in nav_config and "motor_port" in radar_config:
        config["rotation_port"] = radar_config["motor_port"]
    if "measurement_mode" not in nav_config and "ccd_parser" in radar_config:
        config["measurement_mode"] = radar_config["ccd_parser"]
    legacy_calibration = CalibrationModel.from_dict(radar_calibration)
    config["calibration_error"] = ""
    if legacy_calibration.ready and "calibration_file" not in nav_config and "calibration_file" not in radar_config:
        config["calibration"] = legacy_calibration.to_dict()
        config["calibration_model"] = "inverse"
        if "ccd_parser" in radar_config:
            config["measurement_mode"] = radar_config["ccd_parser"]
    else:
        calibration_file = config.get("calibration_file", DEFAULT_CONFIG["calibration_file"])
        calibration_path = Path(str(calibration_file))
        if not calibration_path.is_absolute():
            calibration_path = APP_DIR / calibration_path
        try:
            table = CalibrationModel.from_csv(calibration_path)
        except (OSError, ValueError, TypeError) as exc:
            config["calibration"] = {}
            config["calibration_error"] = f"标定表加载失败：{calibration_path}（{exc}）"
        else:
            config["calibration"] = table.to_dict()
            config["calibration_model"] = "table"
            config["calibration_file"] = str(calibration_path.relative_to(APP_DIR)) if calibration_path.is_relative_to(APP_DIR) else str(calibration_path)
    config["actual_exposure_index"] = 5
    config["exposure_index"] = 5
    config["pixel_min"] = 0
    config["pixel_max"] = 1500
    legacy_range = (config.get("min_range_m"), config.get("max_range_m"))
    has_explicit_hardware_range = any(
        key in nav_config or key in radar_config
        for key in ("hardware_min_range_m", "hardware_max_range_m")
    )
    if not has_explicit_hardware_range:
        if legacy_range in {(0.08, 3.0), (0.08, 3)}:
            config["hardware_min_range_m"] = DEFAULT_CONFIG["hardware_min_range_m"]
            config["hardware_max_range_m"] = DEFAULT_CONFIG["hardware_max_range_m"]
        else:
            config["hardware_min_range_m"] = config.get("min_range_m", DEFAULT_CONFIG["min_range_m"])
            config["hardware_max_range_m"] = config.get("max_range_m", DEFAULT_CONFIG["max_range_m"])
    config["min_range_m"] = config.get("hardware_min_range_m", config.get("min_range_m", DEFAULT_CONFIG["min_range_m"]))
    config["max_range_m"] = config.get("hardware_max_range_m", config.get("max_range_m", DEFAULT_CONFIG["max_range_m"]))
    config["hardware_min_range_m"] = config["min_range_m"]
    config["hardware_max_range_m"] = config["max_range_m"]
    if overrides:
        config.update(overrides)
    return resolve_runtime_config(
        source,
        view,
        config,
        prefer_mode_defaults=str(source).lower() == "simulation",
        # Load saved selections so the UI can correct conflicts. connect() and
        # parameter application still validate ports before opening devices.
        validate_port_assignments=False,
    )


def save_configuration(config: dict) -> None:
    persisted = dict(config)
    for key in ("calibration", "calibration_error", "runtime_source", "runtime_view", "auto_enabled", "config_fingerprint", "chassis_config1_values", "chassis_config1_crc"):
        persisted.pop(key, None)
    save_json(NAV_CONFIG_PATH, persisted)


class MapCanvas(tk.Canvas):
    def __init__(self, master, **kwargs) -> None:
        super().__init__(master, background="#08111f", highlightthickness=0, **kwargs)
        self.grid: OccupancyGrid | None = None
        self.pose = None
        self.path: list[tuple[int, int]] = []
        self.target: tuple[int, int] | None = None
        self.enabled = False
        self._static_signature = None
        self._dynamic_signature = None
        self.bind("<Configure>", lambda _event: self.redraw())

    def update_scene(
        self,
        grid: OccupancyGrid,
        pose,
        path: list[tuple[int, int]],
        target: tuple[int, int] | None,
        enabled: bool,
    ) -> None:
        static_signature = (
            grid._revision if grid is not None else None,
            grid.width if grid is not None else None,
            grid.height if grid is not None else None,
            grid.resolution_m if grid is not None else None,
            enabled,
            self.winfo_width(),
            self.winfo_height(),
        )
        dynamic_signature = (
            round(pose.x, 3) if pose is not None else None,
            round(pose.y, 3) if pose is not None else None,
            round(pose.yaw, 3) if pose is not None else None,
            tuple(path),
            target,
        )
        self.grid = grid
        self.pose = pose
        self.path = list(path)
        self.target = target
        self.enabled = enabled
        if static_signature != self._static_signature:
            self._static_signature = static_signature
            self._redraw_static()
        if dynamic_signature != self._dynamic_signature:
            self._dynamic_signature = dynamic_signature
            self._redraw_dynamic()

    def redraw(self) -> None:
        self._static_signature = None
        self._dynamic_signature = None
        self._redraw_static()
        self._redraw_dynamic()

    def _redraw_static(self) -> None:
        self.delete("all")
        width = max(160, self.winfo_width())
        height = max(160, self.winfo_height())
        self.create_rectangle(0, 0, width, height, fill="#08111f", outline="")
        if not self.enabled or self.grid is None:
            self.create_text(
                width / 2,
                height / 2,
                text="仅雷达模式\n未建立环境地图",
                fill=COLORS["muted"],
                justify="center",
                font=("Microsoft YaHei UI", 12),
            )
            return

        grid = self.grid
        margin = 20
        cell_size = min((width - margin * 2) / grid.width, (height - margin * 2) / grid.height)
        left = (width - grid.width * cell_size) / 2
        top = (height - grid.height * cell_size) / 2
        colors = {grid.UNKNOWN: "#0c1929", grid.FREE: "#26394d", grid.OCCUPIED: "#dce9f4"}

        for row in range(grid.height):
            run_state = grid.state(0, row)
            run_start = 0
            for col in range(1, grid.width + 1):
                state = grid.state(col, row) if col < grid.width else None
                if state == run_state:
                    continue
                if run_state != grid.UNKNOWN:
                    self.create_rectangle(
                        left + run_start * cell_size,
                        top + row * cell_size,
                        left + col * cell_size + 0.5,
                        top + (row + 1) * cell_size + 0.5,
                        fill=colors[run_state],
                        outline="",
                    )
                run_start = col
                run_state = state

        self.create_text(16, 14, text="累计栅格地图", anchor="nw", fill=COLORS["muted"], font=("Microsoft YaHei UI", 10))

    def _redraw_dynamic(self) -> None:
        self.delete("dynamic")
        if not self.enabled or self.grid is None:
            return
        width = max(160, self.winfo_width())
        height = max(160, self.winfo_height())
        grid = self.grid
        margin = 20
        cell_size = min((width - margin * 2) / grid.width, (height - margin * 2) / grid.height)
        left = (width - grid.width * cell_size) / 2
        top = (height - grid.height * cell_size) / 2

        if self.path:
            coordinates = []
            for col, row in self.path:
                coordinates.extend((left + (col + 0.5) * cell_size, top + (row + 0.5) * cell_size))
            if len(coordinates) >= 4:
                self.create_line(*coordinates, fill=COLORS["yellow"], width=2, tags="dynamic")

        if self.target is not None:
            col, row = self.target
            x = left + (col + 0.5) * cell_size
            y = top + (row + 0.5) * cell_size
            self.create_oval(x - 5, y - 5, x + 5, y + 5, outline=COLORS["yellow"], width=2, tags="dynamic")

        if self.pose is not None:
            col, row = grid.world_to_cell(self.pose.x, self.pose.y)
            x = left + (col + 0.5) * cell_size
            y = top + (row + 0.5) * cell_size
            heading = self.pose.yaw
            tip = (x + 12 * math.sin(heading), y - 12 * math.cos(heading))
            side_a = (x + 7 * math.sin(heading + 2.45), y - 7 * math.cos(heading + 2.45))
            side_b = (x + 7 * math.sin(heading - 2.45), y - 7 * math.cos(heading - 2.45))
            self.create_polygon(*tip, *side_a, *side_b, fill=COLORS["green"], outline="#ffffff", tags="dynamic")


class SimulationSource:
    def __init__(self, event_queue: queue.Queue, config: dict) -> None:
        self.events = event_queue
        self.config = config
        self._world = self._create_world()
        self.hardware = HardwareSimulation(self._world, self.config)
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.lock = threading.Lock()
        self.scan_count = 0
        self.generation = 0

    def _create_world(self) -> HiddenWorld:
        configured = Path(str(self.config.get("simulation_map_file", "simulation_map.json")))
        map_path = configured if configured.is_absolute() else APP_DIR / configured
        return HiddenWorld(map_path=map_path)

    @property
    def running(self) -> bool:
        return bool(self.thread and self.thread.is_alive())

    def start(self) -> None:
        if self.running:
            return
        now = time.perf_counter()
        if self._world.map_error:
            self.events.put(("error", f"自定义地图无法读取，已使用内置地图：{self._world.map_error}", now))
        elif self._world.map_loaded and self._world.map_path is not None:
            self.events.put(("info", f"已加载模拟地图  {self._world.map_path.name}", now))
        else:
            self.events.put(("info", "未找到自定义地图，使用内置场景", now))
        self.generation += 1
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, name="navigation-simulator", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread and self.thread.is_alive() and self.thread is not threading.current_thread():
            self.thread.join(timeout=0.5)
        self.thread = None
        with self.lock:
            self.hardware.stop()

    def reset(self) -> None:
        with self.lock:
            self._world = self._create_world()
            self.hardware = HardwareSimulation(self._world, self.config)
            self.scan_count = 0

    def execute(self, command: VelocityCommand) -> None:
        with self.lock:
            self.hardware.execute(command)

    def _run(self) -> None:
        try:
            deadline = time.perf_counter()
            while not self.stop_event.is_set():
                started = time.perf_counter()
                with self.lock:
                    sweeps = self.hardware.advance(0.02)
                    safety_observations = list(self.hardware.last_safety_observations)
                    motion_completed = self.hardware.last_motion_completed
                    # Publish the whole step atomically with its simulation time.
                    # The UI must consume these observations before checking a
                    # timeout against that time, including the completion event.
                    for sequence, points, period in sweeps:
                        self.events.put(("simulation_sweep", (self.generation, sequence, points, period), started))
                    for sequence, points, period in self.hardware.last_local_results:
                        self.events.put(("simulation_local_sweep", (self.generation, sequence, points, period), started))
                    for packet, estimate, simulation_time in safety_observations:
                        self.events.put(("simulation_observation", (self.generation, packet, estimate, simulation_time), started))
                    if motion_completed:
                        self.events.put(("simulation_motion_stopped", self.generation, started))
                speed = max(0.1, float(self.config.get("simulation_speed", 1.0)))
                deadline += 0.02 / speed
                now = time.perf_counter()
                if now - deadline > 0.5:
                    deadline = now
                self.stop_event.wait(max(0.0, deadline - now))
        except Exception as exc:
            self.events.put(("error", f"模拟采集停止：{exc}", time.perf_counter()))


class NavigationApp:
    def __init__(self, root: tk.Tk, source: str, initial_view: str = "radar", simulation_speed: float | None = None, simulation_profile: str | None = None) -> None:
        self.root = root
        self.source = source
        self._closed = False
        overrides = {}
        if simulation_speed is not None:
            overrides["simulation_speed"] = simulation_speed
        if simulation_profile is not None:
            overrides["simulation_profile"] = simulation_profile
        self.config = load_configuration(source, initial_view, overrides)
        self.events: queue.Queue = queue.Queue()
        self.safety_events: queue.Queue = queue.Queue()
        self.mapping_tasks: queue.Queue = queue.Queue(maxsize=2)
        self.mapping_results: queue.Queue = queue.Queue()
        self.mapping_stop = threading.Event()
        self.mapping_generation = 0
        self.mapping_lock = threading.RLock()
        self.runtime_min_range_m = float(self.config.get(
            "simulation_min_range_m" if source == "simulation" else "min_range_m",
            0.08 if source == "simulation" else 0.15,
        ))
        self.runtime_max_range_m = float(self.config.get(
            "simulation_max_range_m" if source == "simulation" else "max_range_m",
            3.0 if source == "simulation" else 1.00,
        ))
        self.pending_events: list[tuple[float, int, str, object]] = []
        self.event_counter = 0
        self.sequence_monitor = SequenceMonitor()
        self.device_clocks = {name: DeviceClock() for name in ("measurement", "rotation")}
        self.motion_safety = MotionSafetyGuard(self.config)
        self.runtime_min_range_m = float(
            self.config.get("simulation_min_range_m" if source == "simulation" else "min_range_m")
        )
        self.runtime_max_range_m = float(
            self.config.get("simulation_max_range_m" if source == "simulation" else "max_range_m")
        )
        self.navigator = build_navigation_engine(self.config)
        self.grid = self.navigator.grid
        self.navigator.min_range_m = self.runtime_min_range_m
        self.mapping_snapshot = None
        self.mapping_runtime = MappingRuntime(
            self.navigator,
            min_range_m=self.runtime_min_range_m,
            queue_size=int(self.config.get("mapping_queue_size", 2)),
            on_result=self._mapping_runtime_result,
            lock=self.mapping_lock,
        )
        self.mapping_thread = self.mapping_runtime._thread
        self.view_mode = tk.StringVar(value="navigation" if initial_view == "navigation" else "radar")
        self.running = False
        self.connected = source == "simulation"
        self.latest_points: list[ScanPoint] = []
        self.latest_raw_points: list[ScanPoint] = []
        self.latest_distance: float | None = None
        self.latest_angle: float | None = None
        self.current_pixel: int | None = None
        self.latest_bias = 0.0
        self.latest_chassis_state: object = {}
        self.scan_rate = SlidingRate(window_s=8.0)
        self.measurement_rate = SlidingRate(window_s=5.0)
        self.status_history: deque[str] = deque(maxlen=80)
        self.last_request_time = 0.0
        self.accept_samples = True
        self.moving = False
        self.motion_generation = 0
        self.manual_motion = False
        self.disconnect_requested = False
        self._close_requested = False
        self._close_deadline = None
        self._resume_radar_on_settle = False
        self._skip_next_chassis_stop = False
        self.scan_collect_after = -math.inf
        self._post_motion_map_revision: int | None = None
        self._post_motion_scan_count: int | None = None
        self._action_commands: dict[int, VelocityCommand] = {}

        self.calibration = CalibrationModel.from_dict(self.config.get("calibration"))
        self.ccd_parser = CCDFrameParser(str(self.config.get("measurement_mode", "fffe")))
        self.measure_line_parser = MotorLineParser()
        self.rotation_parser = MotorLineParser()
        self.rotation = RotationTracker(
            angle_offset_deg=float(self.config.get("angle_offset_deg", 0.0)),
            clockwise=bool(self.config.get("clockwise", False)),
            initial_period_s=float(self.config.get("radar_period_s", 1.5)),
            keep_revolutions=1,
        )

        self.simulation = SimulationSource(self.events, self.config) if source == "simulation" else None
        self.measure_endpoint = SerialEndpoint("测距串口", self._measurement_data, self._serial_error)
        self.rotation_endpoint = SerialEndpoint("旋转串口", self._rotation_data, self._serial_error)
        self.chassis_endpoint = SerialEndpoint("底盘串口", self._chassis_data, self._serial_error)
        self.chassis_adapter = ChassisMotionAdapter(self.config)
        self.chassis_controller = ChassisController(
            self.chassis_endpoint,
            self._emit_chassis_event,
            self.config,
        )
        raw_log_path = Path(str(self.config.get("chassis_raw_log_file", "logs/chassis_serial.jsonl")))
        if not raw_log_path.is_absolute():
            raw_log_path = APP_DIR / raw_log_path
        self.chassis_traffic_logger = ChassisTrafficLogger(
            raw_log_path,
            enabled=bool(self.config.get("chassis_raw_log_enabled", True)),
        )
        self.sync = SynchronizedAcquisition(
            self.measure_endpoint, self.rotation_endpoint, self.calibration, self.config,
            self._receive_sync_event,
        )

        self._build_window()
        self._build_styles()
        self._build_ui()
        self._load_ui_values()
        self.refresh_ports()
        self._set_view(initial_view)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(15, self._poll)
        self.root.after(100, self._draw)
        if source == "simulation":
            self.root.after(250, self.start)

    def _build_window(self) -> None:
        title = "TriScan 模拟探索" if self.source == "simulation" else "TriScan 实物导航"
        self.root.title(title)
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        width = min(1380, max(1120, screen_width - 120))
        height = min(860, max(700, screen_height - 140))
        self.root.geometry(f"{width}x{height}+30+30")
        self.root.minsize(min(1080, width), min(680, height))
        self.root.configure(background=COLORS["bg"])

    def _build_styles(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("App.TFrame", background=COLORS["bg"])
        style.configure("Panel.TFrame", background=COLORS["panel"])
        style.configure("Alt.TFrame", background=COLORS["panel_alt"])
        style.configure("TLabel", background=COLORS["panel"], foreground=COLORS["text"], font=("Microsoft YaHei UI", 10))
        style.configure("Muted.TLabel", background=COLORS["panel"], foreground=COLORS["muted"], font=("Microsoft YaHei UI", 9))
        style.configure("Header.TLabel", background=COLORS["bg"], foreground=COLORS["text"], font=("Microsoft YaHei UI", 20, "bold"))
        style.configure("HeaderSub.TLabel", background=COLORS["bg"], foreground=COLORS["muted"], font=("Microsoft YaHei UI", 9))
        style.configure("Metric.TLabel", background=COLORS["panel"], foreground=COLORS["green"], font=("Segoe UI", 17, "bold"))
        style.configure("TButton", background=COLORS["panel_alt"], foreground=COLORS["text"], bordercolor=COLORS["border"], padding=(11, 7), font=("Microsoft YaHei UI", 9))
        style.map("TButton", background=[("active", "#1b3350"), ("pressed", "#0d1827")])
        style.configure("Primary.TButton", background=COLORS["green_dark"], foreground="#ffffff", bordercolor=COLORS["green"], padding=(14, 8), font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Danger.TButton", background="#612d39", foreground="#ffffff", bordercolor=COLORS["red"], padding=(14, 8), font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("TRadiobutton", background=COLORS["bg"], foreground=COLORS["muted"], font=("Microsoft YaHei UI", 10, "bold"), padding=(12, 5))
        style.map("TRadiobutton", foreground=[("selected", COLORS["green"])], background=[("active", COLORS["bg"])])
        style.configure("TEntry", fieldbackground="#0b1726", foreground=COLORS["text"], insertcolor=COLORS["text"], bordercolor=COLORS["border"], padding=5)
        style.configure("TCombobox", fieldbackground="#0b1726", background="#0b1726", foreground=COLORS["text"], arrowcolor=COLORS["muted"], bordercolor=COLORS["border"], padding=4)
        style.map("TCombobox", fieldbackground=[("readonly", "#0b1726")], foreground=[("readonly", COLORS["text"])])

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, style="App.TFrame", padding=18)
        outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer, style="App.TFrame")
        header.pack(fill="x", pady=(0, 14))
        ttk.Label(header, text="TriScan", style="Header.TLabel").pack(side="left")
        ttk.Label(header, text="自主建图与麦轮导航", style="HeaderSub.TLabel").pack(side="left", padx=(12, 0), pady=(9, 0))
        self.source_badge = tk.Label(
            header,
            text=(f"硬件仿真 · {self.config.get('simulation_profile', 'NOMINAL')}"
                  if self.source == "simulation"
                  else ("实物设备 · 仅雷达" if self.view_mode.get() == "radar" else "实物设备 · 自动导航")),
            bg="#193852" if self.source == "simulation" else COLORS["panel_alt"],
            fg=COLORS["cyan"] if self.source == "simulation" else COLORS["text"],
            padx=12,
            pady=6,
            font=("Microsoft YaHei UI", 9, "bold"),
        )
        self.source_badge.pack(side="right")
        ttk.Radiobutton(header, text="自动导航", variable=self.view_mode, value="navigation", command=lambda: self._set_view("navigation")).pack(side="right", padx=(4, 12))
        ttk.Radiobutton(header, text="仅雷达", variable=self.view_mode, value="radar", command=lambda: self._set_view("radar")).pack(side="right")

        self.content = ttk.Frame(outer, style="App.TFrame")
        self.content.pack(fill="both", expand=True)
        self.radar_panel = ttk.Frame(self.content, style="Panel.TFrame", padding=1)
        self.side_panel = ttk.Frame(self.content, style="Panel.TFrame", width=400)
        self.radar_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        self.side_panel.grid(row=0, column=1, sticky="nsew")
        self.content.rowconfigure(0, weight=1)
        self.content.columnconfigure(0, weight=1, minsize=680)
        self.content.columnconfigure(1, weight=0, minsize=400)

        toolbar = ttk.Frame(self.radar_panel, style="Panel.TFrame", padding=(14, 11))
        toolbar.pack(fill="x")
        self.connection_label = tk.Label(toolbar, text="●  未启动", bg=COLORS["panel"], fg=COLORS["muted"], font=("Microsoft YaHei UI", 10, "bold"))
        self.connection_label.pack(side="left")
        ttk.Button(toolbar, text="紧急停止", style="Danger.TButton", command=self.emergency_stop).pack(side="right")
        self.start_button = ttk.Button(toolbar, text="开始", style="Primary.TButton", command=self.toggle_start)
        self.start_button.pack(side="right", padx=(0, 8))
        if self.source == "simulation":
            ttk.Button(toolbar, text="重置场景", command=self.reset).pack(side="right", padx=(0, 8))
        else:
            self.connect_button = ttk.Button(toolbar, text="连接设备", command=self.toggle_connection)
            self.connect_button.pack(side="right", padx=(0, 8))

        self.radar_canvas = RadarCanvas(self.radar_panel)
        self.radar_canvas.pack(fill="both", expand=True)

        metrics = ttk.Frame(self.radar_panel, style="Panel.TFrame", padding=(14, 10))
        metrics.pack(fill="x")
        self.metric_vars = {key: tk.StringVar(value="—") for key in (
            "distance", "angle", "period", "frequency", "points", "scans", "local_maps", "drift"
        )}
        metric_defs = [
            ("最近距离", "distance", "m"),
            ("当前方位", "angle", "°"),
            ("旋转周期", "period", "s"),
            ("有效测距", "frequency", "Hz"),
            ("本圈回波", "points", "点"),
            ("完整扫描", "scans", "圈"),
            ("局部建图", "local_maps", "圈"),
            ("测距漂移", "drift", "mm"),
        ]
        for index, (label, key, unit) in enumerate(metric_defs):
            block = ttk.Frame(metrics, style="Panel.TFrame")
            block.grid(row=0, column=index, sticky="ew", padx=(0 if index == 0 else 14, 0))
            ttk.Label(block, text=label, style="Muted.TLabel").pack(anchor="w")
            row = ttk.Frame(block, style="Panel.TFrame")
            row.pack(anchor="w")
            ttk.Label(row, textvariable=self.metric_vars[key], style="Metric.TLabel").pack(side="left")
            ttk.Label(row, text=unit, style="Muted.TLabel").pack(side="left", padx=(4, 0), pady=(7, 0))
            metrics.columnconfigure(index, weight=1)

        self._build_side_panel()

    def _build_side_panel(self) -> None:
        map_header = ttk.Frame(self.side_panel, style="Panel.TFrame", padding=(14, 12))
        map_header.pack(fill="x")
        ttk.Label(map_header, text="已建图区域", font=("Microsoft YaHei UI", 12, "bold")).pack(side="left")
        self.map_percent_var = tk.StringVar(value="0.0%")
        ttk.Label(map_header, textvariable=self.map_percent_var, style="Muted.TLabel").pack(side="right")
        ttk.Button(map_header, text="清空局部地图", command=self.clear_local_map).pack(side="right", padx=(0, 10))
        self.map_canvas = MapCanvas(self.side_panel, height=210 if self.source == "hardware" else 420)
        self.map_canvas.pack(fill="both", expand=True, padx=1)

        status = ttk.Frame(self.side_panel, style="Panel.TFrame", padding=14)
        status.pack(fill="x")
        self.nav_state_var = tk.StringVar(value="仅雷达")
        self.nav_detail_var = tk.StringVar(value="自动导航未启用")
        ttk.Label(status, textvariable=self.nav_state_var, font=("Microsoft YaHei UI", 14, "bold"), foreground=COLORS["green"]).pack(anchor="w")
        ttk.Label(status, textvariable=self.nav_detail_var, style="Muted.TLabel", wraplength=390).pack(anchor="w", pady=(3, 10))
        self.measurement_detail_var = tk.StringVar(value="像素 —   距离 —")
        ttk.Label(status, textvariable=self.measurement_detail_var, style="Muted.TLabel").pack(anchor="w")
        self.pose_var = tk.StringVar(value="x 0.00 m   y 0.00 m   θ 0.0°")
        ttk.Label(status, textvariable=self.pose_var).pack(anchor="w")
        self.frontier_var = tk.StringVar(value="前沿 0   匹配 —")
        ttk.Label(status, textvariable=self.frontier_var, style="Muted.TLabel").pack(anchor="w", pady=(3, 0))

        if self.source == "simulation":
            self.sim_status_var = tk.StringVar()
            ttk.Label(status, textvariable=self.sim_status_var, style="Muted.TLabel", wraplength=390).pack(anchor="w", pady=(6, 0))
        if self.source == "hardware":
            self._build_hardware_controls(status)

    def _build_hardware_controls(self, parent) -> None:
        self.hardware_controls_expanded = False
        self.hardware_toggle_button = ttk.Button(
            parent,
            text="展开串口与指令",
            command=self._toggle_hardware_controls,
        )
        self.hardware_toggle_button.pack(fill="x", pady=(12, 0))
        self.hardware_controls_frame = ttk.Frame(parent, style="Panel.TFrame")
        controls = self.hardware_controls_frame
        ttk.Separator(controls).pack(fill="x", pady=(12, 10))
        ttk.Label(controls, text="设备连接", font=("Microsoft YaHei UI", 10, "bold")).pack(anchor="w", pady=(0, 7))
        self.measure_port_var = tk.StringVar()
        self.rotation_port_var = tk.StringVar()
        self.chassis_port_var = tk.StringVar()
        radar_baud = int(self.config.get("radar_baudrate", 115200))
        chassis_baud = int(self.config.get("chassis_baudrate", 9600))
        self.measure_combo = self._compact_combo(controls, f"测距 {radar_baud}", self.measure_port_var)
        self.rotation_combo = self._compact_combo(controls, f"旋转 {radar_baud}", self.rotation_port_var)
        self.chassis_combo = self._compact_combo(controls, f"底盘 {chassis_baud}", self.chassis_port_var)
        ttk.Button(controls, text="刷新串口", command=self.refresh_ports).pack(fill="x", pady=(7, 0))
        setup_row = ttk.Frame(controls, style="Panel.TFrame")
        setup_row.pack(fill="x", pady=(7, 0))
        ttk.Button(
            setup_row,
            text="通信检查（20次）",
            command=self._start_chassis_communication_check,
        ).pack(side="left", fill="x", expand=True)
        ttk.Button(
            setup_row,
            text="保存底盘现场参数",
            command=self._open_chassis_setup,
        ).pack(side="left", fill="x", expand=True, padx=(6, 0))
        ttk.Separator(controls).pack(fill="x", pady=(12, 10))
        ttk.Label(controls, text="底盘人工短动作", font=("Microsoft YaHei UI", 10, "bold")).pack(anchor="w", pady=(0, 7))
        manual_row = ttk.Frame(controls, style="Panel.TFrame")
        manual_row.pack(fill="x", pady=2)
        ttk.Label(manual_row, text="方向", style="Muted.TLabel", width=8).pack(side="left")
        self.manual_mode_var = tk.StringVar(value="W")
        self.manual_mode_combo = ttk.Combobox(
            manual_row,
            textvariable=self.manual_mode_var,
            values=tuple("WSADQEZCRF"),
            state="readonly",
            width=6,
        )
        self.manual_mode_combo.pack(side="left")
        self.manual_unit_var = tk.StringVar(value="MM")
        self.manual_unit_combo = ttk.Combobox(
            manual_row,
            textvariable=self.manual_unit_var,
            values=("MM", "CNT"),
            state="readonly",
            width=6,
        )
        self.manual_unit_combo.pack(side="left", padx=(8, 4))
        self.manual_mode_combo.bind(
            "<<ComboboxSelected>>",
            lambda _event: self.manual_unit_var.set("CNT")
            if self.manual_mode_var.get() in {"R", "F"} else None,
        )
        self.manual_value_var = tk.StringVar(value="100")
        ttk.Entry(manual_row, textvariable=self.manual_value_var, width=10).pack(side="left", fill="x", expand=True)
        self.manual_authorized_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            controls,
            text="现场单步授权",
            variable=self.manual_authorized_var,
        ).pack(anchor="w", pady=(6, 0))
        action_row = ttk.Frame(controls, style="Panel.TFrame")
        action_row.pack(fill="x", pady=(7, 0))
        ttk.Button(action_row, text="发送人工动作", command=self._send_manual_move).pack(side="left", fill="x", expand=True)
        ttk.Button(action_row, text="读取底盘状态", command=self._request_chassis_status).pack(side="left", fill="x", expand=True, padx=(6, 0))
        config_row = ttk.Frame(controls, style="Panel.TFrame")
        config_row.pack(fill="x", pady=(7, 0))
        ttk.Button(config_row, text="读取底盘参数", command=lambda: self._sync_chassis_parameters(False)).pack(side="left", fill="x", expand=True)
        ttk.Button(config_row, text="重载并同步参数", command=lambda: self._sync_chassis_parameters(True)).pack(side="left", fill="x", expand=True, padx=(6, 0))
        self.chassis_status_var = tk.StringVar(value="底盘未连接")
        ttk.Label(controls, textvariable=self.chassis_status_var, style="Muted.TLabel", wraplength=370).pack(anchor="w", pady=(7, 0))
        self.chassis_capability_var = tk.StringVar(value=self._chassis_capability_summary())
        ttk.Label(
            controls,
            textvariable=self.chassis_capability_var,
            style="Muted.TLabel",
            wraplength=370,
        ).pack(anchor="w", pady=(5, 0))

    def _toggle_hardware_controls(self) -> None:
        self._set_hardware_controls(not self.hardware_controls_expanded)

    def _set_hardware_controls(self, expanded: bool) -> None:
        if self.source != "hardware":
            return
        self.hardware_controls_expanded = bool(expanded)
        if expanded:
            self.hardware_controls_frame.pack(fill="x")
            self.hardware_toggle_button.configure(text="收起串口与指令")
        else:
            self.hardware_controls_frame.pack_forget()
            self.hardware_toggle_button.configure(text="展开串口与指令")

    def _sync_chassis_parameters(self, apply: bool) -> None:
        controller = self.chassis_controller
        if self.running or controller.in_flight or controller.communication_check is not None:
            self._log("请先停止导航并确认底盘空闲，再读取/同步参数")
            return
        try:
            config = resolve_runtime_config(self.source, self.view_mode.get(), self.config)
        except (RuntimeConfigError, ValueError, OSError) as exc:
            messagebox.showerror("底盘参数文件无效", str(exc))
            return
        if not config.get("chassis_config1_file"):
            self._log("请在 navigation_config.json 设置 chassis_config1_file")
            return
        self._activate_chassis_config(config, reset_mapping=False)
        if not controller.request_config_sync(apply=apply):
            self._log("参数已在电脑重载；请连接底盘并读取空闲状态，再同步")

    def _request_chassis_status(self) -> None:
        controller = getattr(self, "chassis_controller", None)
        if controller is None or not controller.request_status():
            self._log("当前底盘状态不允许读取")

    def _verify_chassis_on_idle(self, generation: int, state: str) -> None:
        controller = getattr(self, "chassis_controller", None)
        if (controller is None or generation != controller.connection_generation
                or not self.config.get("chassis_config1_file")):
            return
        if state == ChassisState.CONNECTED_WAITING:
            self._chassis_config_read_generation = None
        elif (state == ChassisState.IDLE
              and getattr(self, "_chassis_config_read_generation", None) != generation):
            if controller.config_verified or controller.config_exchange is not None:
                self._chassis_config_read_generation = generation
            elif controller.request_config_sync(apply=False):
                self._chassis_config_read_generation = generation

    def _start_chassis_communication_check(self) -> None:
        controller = getattr(self, "chassis_controller", None)
        if controller is None or not controller.request_communication_check(20):
            self._log("通信检查未启动：需要底盘空闲且已确认 PING 能力")
            return
        if hasattr(self, "chassis_status_var"):
            self.chassis_status_var.set("通信检查 0/20；只发送 PING，不发送 MOVE")
        self._log("已启动 20 次底盘 PING 通信检查")

    def _send_manual_move(self) -> None:
        controller = getattr(self, "chassis_controller", None)
        if controller is None:
            self._log("底盘控制器未初始化")
            return
        if not self.manual_authorized_var.get():
            messagebox.showwarning("需要现场授权", "确认车辆周围安全后，勾选“现场单步授权”。")
            return
        try:
            request = self.chassis_adapter.request_for_manual(
                self.manual_mode_var.get().strip(),
                self.manual_value_var.get().strip(),
                self.manual_unit_var.get().strip(),
            )
        except MotionConversionError as exc:
            messagebox.showwarning("人工动作不可发送", str(exc))
            return
        if self.moving or controller.in_flight:
            self._log("底盘仍在执行或等待停止，未发送新动作")
            return
        if not controller.request_move(
            request,
            source="manual",
            operator_authorized=True,
        ):
            self._log("人工底盘动作未发送")
            return
        action = controller.pending
        command = self._command_for_chassis_request(request)
        if action is not None:
            self._action_commands[action.action_id] = command
        self._begin_hardware_motion(command, manual=True)
        self.manual_authorized_var.set(False)
        self._log(
            f"已登记人工动作 {request.mode} {request.request_value} {request.unit}；等待静默与握手"
        )

    def _compact_combo(self, parent, label: str, variable: tk.StringVar):
        row = ttk.Frame(parent, style="Panel.TFrame")
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=label, style="Muted.TLabel", width=13).pack(side="left")
        combo = ttk.Combobox(row, textvariable=variable, state="readonly")
        combo.pack(side="left", fill="x", expand=True)
        return combo

    def _load_ui_values(self) -> None:
        if self.source == "hardware":
            self.measure_port_var.set(str(self.config.get("measurement_port", "")))
            self.rotation_port_var.set(str(self.config.get("rotation_port", "")))
            self.chassis_port_var.set(str(self.config.get("chassis_port", "")))
            preferred = str(self.config.get("chassis_preferred_translation_unit", "MM"))
            self.manual_unit_var.set(preferred if preferred in {"MM", "CNT"} else "MM")
            self.chassis_capability_var.set(self._chassis_capability_summary())

    def _chassis_capability_summary(self) -> str:
        mode = str(self.config.get("chassis_capability_mode", "unknown"))
        mode_label = {
            "unknown": "能力未确认",
            "cnt_only": "CNT 兼容模式",
            "mm_ping_v1": "MM/PING 模式",
        }.get(mode, mode)
        table = self.config.get("chassis_translation_capabilities", {})
        validated = []
        initial = []
        if isinstance(table, dict):
            for direction in "WSADQEZC":
                entry = table.get(direction, {})
                if not isinstance(entry, dict) or not entry.get("enabled", False):
                    continue
                if entry.get("coefficient_status") in {"initial", "validated"}:
                    initial.append(direction)
                if entry.get("motion_range_validated") is True:
                    validated.append(direction)
        version = "固件已确认" if self.config.get("chassis_firmware_confirmed", False) else "固件待确认"
        if self.config.get("chassis_distance_control", False):
            enabled = "".join(direction for direction in "WSADQEZC" if table.get(direction, {}).get("enabled")) or "无"
            return f"方向距离模式 · {version} · 自动方向 {enabled} · R/F 锁定"
        opened = "".join(validated) or "无"
        return f"{mode_label} · {version} · CNT/mm 初值 {len(initial)}/8 · 自动范围 {opened} · R/F 锁定"

    @staticmethod
    def _command_for_chassis_request(request) -> VelocityCommand:
        signs = {
            "W": (0.0, 1.0, 0.0), "S": (0.0, -1.0, 0.0),
            "A": (-1.0, 0.0, 0.0), "D": (1.0, 0.0, 0.0),
            "Q": (-1 / math.sqrt(2), 1 / math.sqrt(2), 0.0),
            "E": (1 / math.sqrt(2), 1 / math.sqrt(2), 0.0),
            "Z": (-1 / math.sqrt(2), -1 / math.sqrt(2), 0.0),
            "C": (1 / math.sqrt(2), -1 / math.sqrt(2), 0.0),
            "R": (0.0, 0.0, 1.0), "F": (0.0, 0.0, -1.0),
        }
        right, forward, yaw = signs[request.mode]
        if request.target_unit == "m":
            speed = 0.10
            return VelocityCommand(
                forward_mps=forward * speed,
                right_mps=right * speed,
                duration_s=max(0.001, request.target / speed),
            )
        angular_speed = 0.20
        return VelocityCommand(
            yaw_rps=yaw * angular_speed,
            duration_s=max(0.001, request.target / angular_speed),
        )

    def _open_chassis_setup(self) -> None:
        controller = getattr(self, "chassis_controller", None)
        if self.running or (controller is not None and (
                controller.in_flight or controller.communication_check is not None)):
            messagebox.showwarning("底盘忙", "请先停止导航并确认底盘空闲。")
            return
        window = tk.Toplevel(self.root)
        window.title("底盘现场参数")
        window.geometry("760x690")
        window.minsize(700, 600)
        window.transient(self.root)

        canvas = tk.Canvas(window, highlightthickness=0, background=COLORS["panel"])
        scrollbar = ttk.Scrollbar(window, orient="vertical", command=canvas.yview)
        body = ttk.Frame(canvas, style="Panel.TFrame", padding=16)
        body_id = canvas.create_window((0, 0), window=body, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        body.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(body_id, width=event.width))

        ttk.Label(body, text="固件与安全边界", font=("Microsoft YaHei UI", 12, "bold")).grid(
            row=0, column=0, columnspan=6, sticky="w", pady=(0, 10)
        )
        variables: dict[str, object] = {
            "capability": tk.StringVar(value=str(self.config.get("chassis_capability_mode", "unknown"))),
            "firmware": tk.BooleanVar(value=bool(self.config.get("chassis_firmware_confirmed", False))),
            "preferred": tk.StringVar(value=str(self.config.get("chassis_preferred_translation_unit", "MM"))),
            "speed_validated": tk.BooleanVar(value=bool(self.config.get("chassis_speed_validated", False))),
            "braking_validated": tk.BooleanVar(value=bool(self.config.get("chassis_braking_validated", False))),
        }
        optional_fields = {
            "speed": ("速度上界 (m/s)", self.config.get("safety_speed_upper_bound_mps")),
            "stop": ("完整停止距离 (m)", self.config.get("safety_stop_distance_m")),
            "radius": ("车体安全包络半径 (m)", self.config.get("robot_radius_m")),
            "offset_x": ("雷达向右偏置 (m)", self.config.get("radar_offset_x_m")),
            "offset_y": ("雷达向前偏置 (m)", self.config.get("radar_offset_y_m")),
            "offset_yaw": ("雷达零角偏置 (°)", self.config.get("radar_offset_yaw_deg")),
        }
        ttk.Label(body, text="能力模式").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Combobox(
            body,
            textvariable=variables["capability"],
            values=("unknown", "cnt_only", "mm_ping_v1"),
            state="readonly",
            width=16,
        ).grid(row=1, column=1, sticky="ew", padx=(8, 18), pady=3)
        ttk.Checkbutton(body, text="固件已确认", variable=variables["firmware"]).grid(
            row=1, column=2, sticky="w", pady=3
        )
        ttk.Label(body, text="平移单位").grid(row=1, column=3, sticky="e", pady=3)
        ttk.Combobox(
            body,
            textvariable=variables["preferred"],
            values=("MM", "CNT"),
            state="readonly",
            width=7,
        ).grid(row=1, column=4, sticky="w", padx=(8, 0), pady=3)
        for index, (key, (label, value)) in enumerate(optional_fields.items()):
            row = 2 + index // 2
            column = (index % 2) * 3
            ttk.Label(body, text=label).grid(row=row, column=column, sticky="w", pady=3)
            variable = tk.StringVar(value="" if value is None else f"{float(value):g}")
            variables[key] = variable
            ttk.Entry(body, textvariable=variable, width=15).grid(
                row=row, column=column + 1, sticky="ew", padx=(8, 18), pady=3
            )
        validation_row = 5
        ttk.Checkbutton(
            body,
            text="速度上界已测",
            variable=variables["speed_validated"],
        ).grid(row=validation_row, column=0, columnspan=2, sticky="w", pady=(6, 3))
        ttk.Checkbutton(
            body,
            text="完整停止距离已测",
            variable=variables["braking_validated"],
        ).grid(row=validation_row, column=2, columnspan=3, sticky="w", pady=(6, 3))

        ttk.Separator(body).grid(row=6, column=0, columnspan=6, sticky="ew", pady=12)
        ttk.Label(body, text="平移方向验证范围", font=("Microsoft YaHei UI", 12, "bold")).grid(
            row=7, column=0, columnspan=6, sticky="w", pady=(0, 8)
        )
        headers = ("方向", "启用", "范围已验证", "最小 mm", "最大 mm", "不确定度 m")
        for column, label in enumerate(headers):
            ttk.Label(body, text=label, style="Muted.TLabel").grid(
                row=8, column=column, sticky="w", padx=(0, 8), pady=3
            )
        direction_vars: dict[str, dict[str, object]] = {}
        table = self.config.get("chassis_translation_capabilities", {})
        for offset, direction in enumerate("WSADQEZC", start=9):
            entry = table.get(direction, {}) if isinstance(table, dict) else {}
            item = {
                "enabled": tk.BooleanVar(value=bool(entry.get("enabled", False))),
                "validated": tk.BooleanVar(value=bool(entry.get("motion_range_validated", False))),
                "minimum": tk.StringVar(value="" if entry.get("validated_min_mm") is None else f"{float(entry['validated_min_mm']):g}"),
                "maximum": tk.StringVar(value="" if entry.get("validated_max_mm") is None else f"{float(entry['validated_max_mm']):g}"),
                "uncertainty": tk.StringVar(value="" if entry.get("uncertainty_m") is None else f"{float(entry['uncertainty_m']):g}"),
            }
            direction_vars[direction] = item
            ttk.Label(body, text=direction).grid(row=offset, column=0, sticky="w", pady=2)
            ttk.Checkbutton(body, variable=item["enabled"]).grid(row=offset, column=1, sticky="w")
            ttk.Checkbutton(body, variable=item["validated"]).grid(row=offset, column=2, sticky="w")
            ttk.Entry(body, textvariable=item["minimum"], width=10).grid(row=offset, column=3, sticky="ew", padx=(0, 8))
            ttk.Entry(body, textvariable=item["maximum"], width=10).grid(row=offset, column=4, sticky="ew", padx=(0, 8))
            ttk.Entry(body, textvariable=item["uncertainty"], width=12).grid(row=offset, column=5, sticky="ew")

        ttk.Label(
            body,
            text="R/F 旋转保持锁定；只有独立旋转标定和验证范围完整后才能另行开放。",
            style="Muted.TLabel",
            wraplength=700,
        ).grid(row=17, column=0, columnspan=6, sticky="w", pady=(10, 8))
        buttons = ttk.Frame(body, style="Panel.TFrame")
        buttons.grid(row=18, column=0, columnspan=6, sticky="ew", pady=(8, 0))
        ttk.Button(
            buttons,
            text="保存底盘现场参数",
            style="Primary.TButton",
            command=lambda: self._save_chassis_setup(window, variables, direction_vars),
        ).pack(side="right")
        ttk.Button(buttons, text="取消", command=window.destroy).pack(side="right", padx=(0, 8))
        for column in (1, 3, 4, 5):
            body.columnconfigure(column, weight=1)

    @staticmethod
    def _optional_float(variable, label: str) -> float | None:
        text = variable.get().strip()
        if not text:
            return None
        try:
            value = float(text)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"{label} 必须是有限数值或留空") from exc
        if not math.isfinite(value):
            raise ValueError(f"{label} 必须是有限数值或留空")
        return value

    def _save_chassis_setup(self, window, variables, direction_vars) -> None:
        controller = getattr(self, "chassis_controller", None)
        if self.running or (controller is not None and (
                controller.in_flight or controller.communication_check is not None)):
            messagebox.showwarning("底盘忙", "请先停止导航并确认底盘空闲。", parent=window)
            return
        candidate = deepcopy(self.config)
        try:
            candidate["chassis_capability_mode"] = variables["capability"].get()
            candidate["chassis_firmware_confirmed"] = bool(variables["firmware"].get())
            candidate["chassis_preferred_translation_unit"] = variables["preferred"].get()
            candidate["chassis_speed_validated"] = bool(variables["speed_validated"].get())
            candidate["chassis_braking_validated"] = bool(variables["braking_validated"].get())
            candidate["safety_speed_upper_bound_mps"] = self._optional_float(variables["speed"], "速度上界")
            candidate["safety_stop_distance_m"] = self._optional_float(variables["stop"], "完整停止距离")
            candidate["robot_radius_m"] = self._optional_float(variables["radius"], "车体安全包络半径")
            candidate["radar_offset_x_m"] = self._optional_float(variables["offset_x"], "雷达向右偏置")
            candidate["radar_offset_y_m"] = self._optional_float(variables["offset_y"], "雷达向前偏置")
            candidate["radar_offset_yaw_deg"] = self._optional_float(variables["offset_yaw"], "雷达零角偏置")
            if candidate["robot_radius_m"] is None:
                raise ValueError("车体安全包络半径不能为空")
            for key in ("radar_offset_x_m", "radar_offset_y_m", "radar_offset_yaw_deg"):
                if candidate[key] is None:
                    raise ValueError("雷达安装偏置不能为空")
            capabilities = deepcopy(candidate.get("chassis_translation_capabilities", {}))
            for direction, item in direction_vars.items():
                entry = capabilities[direction]
                entry["enabled"] = bool(item["enabled"].get())
                validated_range = bool(item["validated"].get())
                entry["motion_range_validated"] = validated_range
                entry["validated_min_mm"] = (
                    self._optional_float(item["minimum"], f"{direction} 最小距离")
                    if validated_range else None
                )
                entry["validated_max_mm"] = (
                    self._optional_float(item["maximum"], f"{direction} 最大距离")
                    if validated_range else None
                )
                entry["uncertainty_m"] = self._optional_float(
                    item["uncertainty"], f"{direction} 不确定度"
                )
            candidate["chassis_translation_capabilities"] = capabilities
            candidate.update({
                "measurement_port": self.measure_port_var.get().strip(),
                "rotation_port": self.rotation_port_var.get().strip(),
                "chassis_port": self.chassis_port_var.get().strip(),
            })
            resolved = resolve_runtime_config(
                "hardware",
                self.view_mode.get(),
                candidate,
            )
        except (KeyError, RuntimeConfigError, ValueError, TypeError) as exc:
            messagebox.showerror("现场参数无效", str(exc), parent=window)
            return
        try:
            save_configuration(resolved)
        except OSError as exc:
            messagebox.showerror("保存失败", str(exc), parent=window)
            return
        self._activate_chassis_config(resolved)
        window.destroy()
        messagebox.showinfo(
            "现场参数已保存",
            "参数已应用。连接中的设备请断开并重新连接，再执行通信检查和自动导航预检。",
            parent=self.root,
        )

    def _activate_chassis_config(self, config: dict, *, reset_mapping: bool = True) -> None:
        self.config = config
        self.chassis_adapter.update_config(config)
        self.chassis_controller.update_config(config)
        self.motion_safety.config = config
        self.sync.config = config
        configured_navigator = build_navigation_engine(config)
        with self.mapping_lock:
            self.navigator.robot_radius_m = configured_navigator.robot_radius_m
            self.navigator.sensor_offset_x_m = configured_navigator.sensor_offset_x_m
            self.navigator.sensor_offset_y_m = configured_navigator.sensor_offset_y_m
            self.navigator.sensor_offset_yaw_rad = configured_navigator.sensor_offset_yaw_rad
            self.navigator.path_turn_penalty = configured_navigator.path_turn_penalty
            self.navigator.translation_capabilities = configured_navigator.translation_capabilities
            if reset_mapping:
                self.navigator.reset()
                self.mapping_snapshot = None
            self.grid = self.navigator.grid
        if reset_mapping:
            self.mapping_generation += 1
            self._clear_mapping_tasks()
        raw_log_path = Path(str(config.get("chassis_raw_log_file", "logs/chassis_serial.jsonl")))
        if not raw_log_path.is_absolute():
            raw_log_path = APP_DIR / raw_log_path
        self.chassis_traffic_logger = ChassisTrafficLogger(
            raw_log_path,
            enabled=bool(config.get("chassis_raw_log_enabled", True)),
        )
        self.manual_unit_var.set(str(config.get("chassis_preferred_translation_unit", "MM")))
        self.chassis_capability_var.set(self._chassis_capability_summary())

    def _set_view(self, view: str) -> None:
        target = "navigation" if view == "navigation" else "radar"
        if target == "navigation":
            if self.running and not self._navigation_preflight(show=True):
                target = "radar"
        elif self.running and (
            self.moving
            or bool(getattr(getattr(self, "chassis_controller", None), "in_flight", False))
        ):
            self._stop_motion_for_mode_switch()
        self.view_mode.set(target)
        self.navigator.set_auto(target == "navigation" and self.running)
        self.nav_state_var.set(self.navigator.state)
        self.nav_detail_var.set(self.navigator.detail)

    def _navigation_preflight(self, show: bool = False) -> bool:
        if self.source == "simulation":
            return True
        calibration_detail = str(self.config.get("calibration_error", "")) or "当前测距格式需要有效距离标定。"
        controller = getattr(self, "chassis_controller", None)
        chassis_open = bool(controller is not None and self.chassis_endpoint.is_open)
        chassis_confirmed = bool(controller is not None and controller.confirmed)
        if controller is not None:
            adapter_ready, adapter_reasons = self.chassis_adapter.readiness()
            adapter_detail = "；".join(adapter_reasons)
        else:
            adapter_ready, adapter_detail = False, "底盘控制器未初始化"
        config_reading = controller is not None and controller.config_exchange is not None
        checks = (
            (self.connected, "设备未连接", "请先连接测距、旋转串口。"),
            (bool(self.calibration.ready), "标定不可用", calibration_detail),
            (bool(self.config.get("synchronized_acquisition", True)), "采集模式不支持导航", "自动导航需要同步观测和实时安全检测。"),
            (chassis_open, "底盘未连接", "自动导航需要连接底盘串口。"),
            (chassis_confirmed, "底盘状态未确认", "请先读取底盘状态，并核对或保存固件能力。"),
            (not self.config.get("chassis_config1_file") or bool(getattr(controller, "config_verified", False)),
             "底盘参数核对中" if config_reading else "底盘参数未同步",
             "正在读取底盘参数，请等待核对完成后再开始。" if config_reading
             else "请停止导航，在展开串口与指令中点击重载并同步参数，等待不同项为 0。"),
            (adapter_ready, "底盘标定未完成", adapter_detail),
        )
        for ok, title, detail in checks:
            if ok:
                continue
            if show:
                if title in {"底盘未连接", "底盘状态未确认", "底盘标定未完成", "底盘参数未同步"}:
                    self._set_hardware_controls(True)
                messagebox.showwarning(title, detail)
            return False
        if not controller.allow_automatic():
            if show:
                messagebox.showwarning("底盘自动准入失败", "底盘当前状态不允许自动动作，请读取状态并重新预检。")
            return False
        return True

    def refresh_ports(self) -> None:
        if self.source != "hardware":
            return
        ports = list_serial_ports()
        for combo in (self.measure_combo, self.rotation_combo, self.chassis_combo):
            combo.configure(values=ports)

    def toggle_connection(self) -> None:
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        ports = [self.measure_port_var.get().strip(), self.rotation_port_var.get().strip(), self.chassis_port_var.get().strip()]
        radar_only = self.view_mode.get() == "radar"
        required = ports[:2] if radar_only else ports
        if not all(required):
            self._set_hardware_controls(True)
            messagebox.showwarning("串口未选择", "仅雷达需要测距和旋转串口；自动导航还需要底盘串口。")
            return
        selected = [port.casefold() for port in ports if port]
        if len(set(selected)) != len(selected):
            messagebox.showwarning("串口重复", "三个设备必须使用不同串口。")
            return
        radar_baudrate = int(self.config.get("radar_baudrate", self.config.get("baudrate", 115200)))
        chassis_baudrate = int(self.config.get("chassis_baudrate", 9600))
        opened = []
        chassis_generation = None
        try:
            self.measure_endpoint.open(ports[0], radar_baudrate)
            opened.append(self.measure_endpoint)
            self.rotation_endpoint.open(ports[1], radar_baudrate)
            opened.append(self.rotation_endpoint)
            if ports[2]:
                chassis_generation = self.chassis_controller.begin_connection()
                self.chassis_endpoint.open(
                    ports[2],
                    chassis_baudrate,
                    on_data=lambda data, stamp, generation=chassis_generation: self._chassis_data(data, stamp, generation),
                    on_error=lambda message, generation=chassis_generation: self._chassis_error(message, generation),
                )
                opened.append(self.chassis_endpoint)
                self.chassis_controller.mark_connection_open()
        except Exception as exc:
            for endpoint in opened:
                endpoint.close()
            if chassis_generation is not None:
                self.chassis_controller.disconnect()
            messagebox.showerror("连接失败", str(exc))
            return
        self.connected = True
        self.connection_label.configure(text="●  设备已连接", fg=COLORS["green"])
        self.connect_button.configure(text="断开设备")
        if ports[2]:
            self.chassis_status_var.set("底盘串口已打开，等待启动标识或状态回复")
            self.chassis_controller.request_status()
        self._log("测距、旋转和底盘通道已连接" if ports[2] else "测距和旋转通道已连接")

    def disconnect(self) -> None:
        controller = getattr(self, "chassis_controller", None)
        if controller is not None and self.chassis_endpoint.is_open:
            self.disconnect_requested = True
            self.stop()
            if not controller.in_flight:
                self._complete_disconnect()
            else:
                self._log("已请求底盘停止，保持连接直至收到停止确认")
            return
        self.stop()
        if controller is not None:
            controller.disconnect()
        self.measure_endpoint.close()
        self.rotation_endpoint.close()
        self.chassis_endpoint.close()
        self.connected = False
        self.disconnect_requested = False
        if hasattr(self, "chassis_status_var"):
            self.chassis_status_var.set("底盘未连接")
        self.connection_label.configure(text="●  未连接", fg=COLORS["muted"])
        self.connect_button.configure(text="连接设备")

    def toggle_start(self) -> None:
        if self.running:
            self.stop()
        else:
            self.start()

    def start(self) -> None:
        if self.source == "hardware":
            try:
                self._settle_duration()
            except (ValueError, TypeError) as exc:
                messagebox.showwarning("停车参数无效", str(exc))
                return
            if not self.connected:
                messagebox.showwarning("设备未连接", "请先连接对应串口。")
                return
            mode = str(self.config.get("measurement_mode", "fffe"))
            if (self.config.get("synchronized_acquisition", True) or mode != "timestamped_ascii") and not self.calibration.ready:
                messagebox.showwarning(
                    "标定不可用",
                    str(self.config.get("calibration_error", "")) or "当前测距格式需要有效距离标定。",
                )
                return
            if self.view_mode.get() == "navigation":
                if not self._navigation_preflight(show=True):
                    return
        self.mapping_generation += 1
        self._clear_mapping_tasks()
        if self.source == "hardware":
            self._start_hardware_scan()
        else:
            assert self.simulation is not None
            self.simulation.start()
            self.connection_label.configure(text="●  模拟运行", fg=COLORS["cyan"])
        self.running = True
        self.navigator.set_auto(self.view_mode.get() == "navigation")
        self.start_button.configure(text="暂停")
        self._log("开始自动导航" if self.view_mode.get() == "navigation" else "开始雷达扫描")

    def stop(self) -> None:
        self.motion_safety.clear()
        self.motion_generation += 1
        self.mapping_generation += 1
        self._clear_mapping_tasks()
        self.mapping_snapshot = None
        if self.simulation:
            self.simulation.stop()
        chassis_in_flight = False
        if self.source == "hardware":
            if getattr(self, "_skip_next_chassis_stop", False):
                self._skip_next_chassis_stop = False
            else:
                self._send_chassis_stop(wait=True)
            controller = getattr(self, "chassis_controller", None)
            chassis_in_flight = bool(controller is not None and controller.in_flight)
            if self.config.get("synchronized_acquisition", True):
                self.sync.stop()
            else:
                SynchronizedAcquisition._stop_endpoint(self.measure_endpoint, ["STOP", "LASER 0"])
                SynchronizedAcquisition._stop_endpoint(self.rotation_endpoint, ["OFF"])
        self.running = False
        self.accept_samples = False
        self.moving = chassis_in_flight
        self.manual_motion = False
        self.navigator.set_auto(False)
        if chassis_in_flight:
            self.navigator.state = "停止中"
            self.navigator.detail = "已发送停止字节，等待底盘报告或状态确认"
        self.start_button.configure(text="开始")
        self.connection_label.configure(
            text="●  停止中" if chassis_in_flight else ("●  已暂停" if self.connected else "●  未连接"),
            fg=COLORS["yellow"] if self.connected else COLORS["muted"],
        )

    def emergency_stop(self) -> None:
        self.stop()
        self.navigator.set_auto(False)
        self.view_mode.set("radar")
        self._log("紧急停止")

    def reset(self) -> None:
        controller = getattr(self, "chassis_controller", None)
        if self.source == "hardware" and controller is not None and controller.in_flight:
            controller.request_stop(reason="重置前停止")
            self._log("底盘未确认停止，暂不重置导航状态")
            return
        was_running = self.running
        self.motion_safety.clear()
        self.moving = False
        self.accept_samples = True
        self.mapping_generation += 1
        self._clear_mapping_tasks()
        if self.simulation:
            self.simulation.stop()
            self.simulation.reset()
        with self.mapping_lock:
            self.navigator.reset()
            self.mapping_snapshot = None
            self.grid = self.navigator.grid
        self.navigator.set_auto(self.view_mode.get() == "navigation")
        self.latest_points.clear()
        self.latest_raw_points.clear()
        self.current_pixel = None
        self.latest_bias = 0.0
        if was_running and self.simulation:
            self.simulation.start()
        self._log("模拟场景已重置")

    def clear_local_map(self) -> None:
        controller = getattr(self, "chassis_controller", None)
        if self.source == "hardware" and controller is not None and controller.in_flight:
            controller.request_stop(reason="清图前停止")
            self._log("底盘未确认停止，暂不清空地图")
            return
        self.mapping_generation += 1
        self._clear_mapping_tasks()
        with self.mapping_lock:
            self.navigator.reset()
            self.navigator.set_auto(self.view_mode.get() == "navigation" and self.running)
            self.mapping_snapshot = None
            self.grid = self.navigator.grid
        self._log("局部地图已清空")

    def _start_hardware_scan(self) -> None:
        self.scan_collect_after = time.perf_counter()
        self.latest_points.clear()
        self.latest_raw_points.clear()
        self.latest_distance = None
        self.latest_angle = None
        self.current_pixel = None
        if self.config.get("synchronized_acquisition", True):
            self.accept_samples = True
            self.rotation.reset()
            self.sync.start()
            self.connection_label.configure(text="●  校时中", fg=COLORS["yellow"])
            return
        self.accept_samples = True
        self.rotation.reset()
        self.ccd_parser.reset()
        self.measure_line_parser.reset()
        self.rotation_parser.reset()
        self.measure_endpoint.write_line("LASER 1")
        self.rotation_endpoint.write_line("RESETCNT")
        self.rotation_endpoint.write_line("ON")
        self.last_request_time = 0.0
        self.connection_label.configure(text="●  实物扫描", fg=COLORS["green"])

    def _measurement_data(self, data: bytes, host_time: float) -> None:
        if self.config.get("synchronized_acquisition", True):
            self.sync.feed("measurement", data, host_time)
            return
        if str(self.config.get("measurement_mode")) == "timestamped_ascii":
            for line in self.measure_line_parser.feed(data):
                parsed = parse_timestamped_distance(line)
                if parsed is None:
                    continue
                sequence, device_us, distance, pixel = parsed
                timestamp = self.device_clocks["measurement"].observe(device_us, host_time)
                self.events.put(("range", (distance, pixel or 0, sequence), timestamp))
        else:
            for pixel in self.ccd_parser.feed(data):
                distance = self.calibration.distance(pixel)
                if distance is not None:
                    self.events.put(("range", (distance, pixel, None), host_time))

    def _rotation_data(self, data: bytes, host_time: float) -> None:
        if self.config.get("synchronized_acquisition", True):
            self.sync.feed("rotation", data, host_time)
            return
        for line in self.rotation_parser.feed(data):
            parsed = parse_trigger(line)
            if parsed is not None:
                count, device_us = parsed
                timestamp = host_time if device_us is None else self.device_clocks["rotation"].observe(device_us, host_time)
                self.events.put(("trigger", count, timestamp))
            else:
                self.events.put(("rotation_status", line, host_time))

    def _emit_chassis_event(self, kind: str, value: object, timestamp: float) -> None:
        self.events.put((kind, value, timestamp))

    def _chassis_data(self, data: bytes, host_time: float, connection_generation: int | None = None) -> None:
        controller = getattr(self, "chassis_controller", None)
        if controller is None:
            return
        generation = controller.connection_generation if connection_generation is None else connection_generation
        controller.feed_data(data, host_time, generation)

    def _chassis_error(self, message: str, connection_generation: int) -> None:
        self.events.put(("chassis_transport_error", (connection_generation, message), time.perf_counter()))

    def _serial_error(self, message: str) -> None:
        self.events.put(("error", (self.sync.generation, message), time.perf_counter()))

    def _queue_event(self, kind: str, value: object, timestamp: float) -> None:
        self.event_counter += 1
        heapq.heappush(self.pending_events, (timestamp, self.event_counter, kind, value))

    def _receive_sync_event(self, kind, value, timestamp):
        if kind in {"sync_status", "sync_error", "sync_diagnostic"}:
            value = (self.sync.generation, value)
        target = self.safety_events if kind == "sync_observation" else self.events
        target.put((kind, value, timestamp))

    def _mapping_runtime_result(self, result: MappingResult) -> None:
        self.mapping_results.put(result)

    def _mapping_worker(self) -> None:
        while not self.mapping_stop.is_set():
            try:
                item = self.mapping_tasks.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                self.mapping_tasks.task_done()
                break
            generation, kind, sequence, points, timestamp = item
            try:
                with self.mapping_lock:
                    command = (self.navigator.process_local_scan(
                        points, min_range_m=self.runtime_min_range_m,
                    ) if kind == "local" else self.navigator.process_scan(points))
                self.mapping_results.put((generation, kind, sequence, command, timestamp, None))
            except Exception as exc:
                self.mapping_results.put((generation, kind, sequence, None, timestamp, exc))
            finally:
                self.mapping_tasks.task_done()

    def _clear_mapping_tasks(self) -> None:
        runtime = getattr(self, "mapping_runtime", None)
        if runtime is not None:
            runtime.invalidate(getattr(self, "mapping_generation", runtime.generation))
        while True:
            try:
                self.mapping_tasks.get_nowait()
            except queue.Empty:
                break
            else:
                self.mapping_tasks.task_done()
        while True:
            try:
                self.mapping_results.get_nowait()
            except queue.Empty:
                return

    def _submit_mapping(self, sequence: int, points: list[ScanPoint], timestamp: float) -> None:
        self._submit_mapping_task("navigation", sequence, points, timestamp)

    def _submit_local_mapping(self, sequence: int, points: list[ScanPoint], timestamp: float) -> None:
        self._submit_mapping_task("local", sequence, points, timestamp)

    def _submit_mapping_task(self, kind: str, sequence: int, points: list[ScanPoint], timestamp: float) -> None:
        runtime = getattr(self, "mapping_runtime", None)
        if runtime is not None:
            session = self.sync.session if hasattr(self, "sync") else ""
            point_times = [
                point.timestamp_s
                for point in points
                if point.timestamp_s is not None and math.isfinite(point.timestamp_s)
            ]
            scan_start = min(point_times, default=timestamp)
            scan_end = max(point_times, default=timestamp)
            runtime.submit(
                session,
                sequence,
                points,
                timestamp,
                mode="local" if kind == "local" else "navigation",
                scan_start_s=scan_start,
                scan_end_s=scan_end,
            )
            return
        item = (self.mapping_generation, kind, sequence, points, timestamp)
        try:
            self.mapping_tasks.put_nowait(item)
        except queue.Full:
            try:
                self.mapping_tasks.get_nowait()
                self.mapping_tasks.task_done()
            except queue.Empty:
                pass
            try:
                self.mapping_tasks.put_nowait(item)
            except queue.Full:
                pass

    def _handle_mapping_results(self) -> None:
        while True:
            try:
                result = self.mapping_results.get_nowait()
            except queue.Empty:
                return
            if isinstance(result, MappingResult):
                generation = result.request.generation
                kind = result.request.mode
                command = result.command
                error = result.error
                if generation != self.mapping_generation:
                    continue
                if error is not None:
                    self._runtime_fault(f"建图处理异常：{error}")
                    return
                if result.snapshot is not None:
                    self.mapping_snapshot = result.snapshot
                    self.grid = result.snapshot.grid
                    latest_scan = getattr(self.navigator, "latest_scan", None)
                    if isinstance(latest_scan, (list, tuple)):
                        self.latest_points = list(latest_scan)
                controller = getattr(self, "chassis_controller", None)
                if (
                    kind == "navigation"
                    and controller is not None
                    and controller.state == ChassisState.WAITING_SCAN
                ):
                    waiting_action = controller.pending
                    request = result.request
                    current_session = self.sync.session if hasattr(self, "sync") else request.session
                    after_boundary = bool(
                        request.session == current_session
                        and request.scan_start_s is not None
                        and request.scan_end_s is not None
                        and request.scan_start_s >= self.scan_collect_after
                        and request.scan_end_s >= self.scan_collect_after
                    )
                    map_advanced = bool(
                        result.snapshot is not None
                        and self._post_motion_map_revision is not None
                        and self._post_motion_scan_count is not None
                        and result.snapshot.map_version > self._post_motion_map_revision
                        and result.snapshot.completed_scans > self._post_motion_scan_count
                    )
                    if not (after_boundary and map_advanced and controller.mark_scan_ready()):
                        continue
                    self.moving = False
                    self.manual_motion = False
                    self.motion_safety.clear()
                    if waiting_action is not None:
                        self._action_commands.pop(waiting_action.action_id, None)
                    self._post_motion_map_revision = None
                    self._post_motion_scan_count = None
                if (kind == "navigation" and command is not None
                        and self.navigator.state == "泊车完成"):
                    self._finish_parking()
                    continue
                if (kind == "navigation" and self.view_mode.get() == "navigation"
                        and self.running and command is not None and not command.stopped):
                    self._execute_navigation_command(command)
                continue
            generation, kind, sequence, command, timestamp, error = result
            if generation != self.mapping_generation:
                continue
            if error is not None:
                self._runtime_fault(f"建图处理异常：{error}")
                return
            if (kind == "navigation" and command is not None
                    and self.navigator.state == "泊车完成"):
                self._finish_parking()
                continue
            if (kind == "navigation" and self.view_mode.get() == "navigation"
                    and self.running and command is not None and not command.stopped):
                self._execute_navigation_command(command)

    def _runtime_fault(self, message: str) -> None:
        self._log(message)
        self.stop()
        self.navigator.state = "控制故障"
        self.navigator.detail = message

    def _finish_parking(self) -> None:
        if not self.running:
            return
        self.motion_safety.clear()
        self.motion_generation += 1
        self.mapping_generation += 1
        self._clear_mapping_tasks()
        if self.simulation:
            self.simulation.stop()
        if self.source == "hardware":
            self._send_chassis_stop(wait=True)
            if self.config.get("synchronized_acquisition", True):
                self.sync.stop()
            else:
                SynchronizedAcquisition._stop_endpoint(self.measure_endpoint, ["STOP", "LASER 0"])
                SynchronizedAcquisition._stop_endpoint(self.rotation_endpoint, ["OFF"])
        self.running = False
        self.accept_samples = False
        controller = getattr(self, "chassis_controller", None)
        self.moving = bool(controller is not None and controller.in_flight)
        self.navigator.set_auto(False)
        self.navigator.state = "泊车完成"
        self.navigator.detail = "连续三次终点复测通过，雷达已停止"
        self.start_button.configure(text="开始")
        self.connection_label.configure(text="●  泊车完成", fg=COLORS["green"])
        self._log("泊车完成，已停止雷达")

    def _stop_motion_for_mode_switch(self) -> None:
        self.motion_generation += 1
        generation = self.motion_generation
        self.motion_safety.clear()
        self.accept_samples = False
        if self.source == "simulation":
            assert self.simulation is not None
            self.simulation.execute(VelocityCommand())
            self.moving = False
            self.accept_samples = True
            return
        controller = getattr(self, "chassis_controller", None)
        if controller is not None:
            self._resume_radar_on_settle = True
            controller.request_stop(reason="切换到仅雷达", now=time.perf_counter())
            self.moving = controller.in_flight
            self._stop_radar_only()
            return
        self.moving = False
        self._send_chassis_stop(wait=True)
        if self.config.get("synchronized_acquisition", True):
            self.sync.stop()
        else:
            SynchronizedAcquisition._stop_endpoint(self.measure_endpoint, ["STOP", "LASER 0"])
            SynchronizedAcquisition._stop_endpoint(self.rotation_endpoint, ["OFF"])
        self.scan_collect_after = time.perf_counter() + self._settle_duration()
        self.root.after(max(1, round(self._settle_duration() * 1000)),
                        lambda: self._resume_radar_after_mode_switch(generation))

    def _resume_radar_after_mode_switch(self, generation: int) -> None:
        if not self.running or generation != self.motion_generation or self.view_mode.get() != "radar":
            return
        self.accept_samples = True
        self._start_hardware_scan()

    def _check_motion_safety(self, now):
        while not self.safety_events.empty():
            _, (session, packet), _ = self.safety_events.get_nowait()
            if not self.running or not self.moving or session != self.sync.session:
                continue
            reason = self.motion_safety.observe(packet, self.sync.receiver.estimate_angle(packet), now)
            if reason:
                self._safety_stop(reason)
                return
        if self.running and self.moving:
            reason = self.motion_safety.poll(now)
            if reason:
                self._safety_stop(reason)

    def _safety_stop(self, reason):
        if (getattr(self, "source", None) == "simulation" and self.running
                and self.motion_safety.command is not None
                and reason == "运动方向出现近距离障碍，紧急停车"):
            command = self.motion_safety.command
            with self.simulation.lock:
                elapsed = self.simulation.hardware.time - self.motion_safety.started_at
                self.simulation.hardware.stop()
            self.motion_safety.clear()
            self.motion_generation += 1
            self.mapping_generation += 1
            with self.mapping_lock:
                self._clear_mapping_tasks()
                self.navigator.request_obstacle_recovery(command, elapsed)
            self.moving = False
            self.accept_samples = True
            self._log("近障碍保护：当前动作已停止，继续扫描并自动平移避让")
            return
        self._send_chassis_stop()
        self._skip_next_chassis_stop = True
        self.stop()
        self.navigator.state = "安全停车"
        self.navigator.detail = reason
        self._log(reason)

    def _poll(self) -> None:
        if self._closed:
            return
        now = time.perf_counter()
        try:
            self._handle_mapping_results()
            if self.source == "hardware" and hasattr(self, "chassis_controller"):
                self.chassis_controller.poll(now)
            if self.source == "hardware" and self.config.get("synchronized_acquisition", True):
                self.sync.poll(now)
                self._check_motion_safety(now)
            simulation_time = None
            if self.source == "simulation" and self.simulation is not None:
                # Capture a finite batch and its matching clock under the same
                # lock used by the producer. Rendering delays must not look like
                # missing sensor data; later steps wait until the next UI poll.
                with self.simulation.lock:
                    simulation_time = self.simulation.hardware.time
                    batch = [self.events.get_nowait() for _ in range(self.events.qsize())]
                for kind, value, timestamp in batch:
                    self._queue_event(str(kind), value, float(timestamp))
            else:
                for _ in range(64):
                    try:
                        kind, value, timestamp = self.events.get_nowait()
                        self._queue_event(str(kind), value, float(timestamp))
                    except queue.Empty:
                        break

            delay = (0.0 if self.source == "simulation" or self.config.get("synchronized_acquisition", True)
                      else float(self.config.get("fusion_delay_ms", 80)) / 1000.0)
            cutoff = time.perf_counter() if simulation_time is not None else now - delay
            while self.pending_events and self.pending_events[0][0] <= cutoff:
                timestamp, _, kind, value = heapq.heappop(self.pending_events)
                self._handle_event(kind, value, timestamp)
            if simulation_time is not None and self.running and self.moving:
                reason = self.motion_safety.poll(simulation_time)
                if reason:
                    self._safety_stop(reason)

            if (self.running and self.source == "hardware"
                    and not self.config.get("synchronized_acquisition", True)
                    and self.accept_samples and self.measure_endpoint.is_open):
                interval = 1.0 / max(1.0, float(self.config.get("sample_rate_hz", 100.0)))
                if now - self.last_request_time >= interval:
                    self.measure_endpoint.write_line(str(self.config.get("ccd_command", "@c0071#@")))
                    self.last_request_time = now
        except Exception as exc:
            self._runtime_fault(f"运行调度异常：{exc}")
        finally:
            if not self._closed:
                self.root.after(15, self._poll)

    def _handle_event(self, kind: str, value: object, timestamp: float) -> None:
        if kind in {"sync_status", "sync_error", "sync_diagnostic"}:
            if not (isinstance(value, tuple) and len(value) == 2 and isinstance(value[0], int)):
                return
            generation, value = value
            if generation != self.sync.generation:
                return
        if kind == "error" and isinstance(value, tuple) and len(value) == 2 and isinstance(value[0], int):
            generation, value = value
            if generation != self.sync.generation:
                return
        if kind == "sync_status":
            if not self.running:
                return
            self._log(str(value))
            starting = self.sync.state in {"starting_measurement", "starting_rotation"}
            label = "校时中" if self.sync.state == "syncing" else ("设备启动中" if starting else "同步采集中")
            if getattr(self.sync, "recovery_started_at", None) is not None:
                label = "超时恢复中（保持旋转）"
            self.connection_label.configure(text="●  " + label, fg=COLORS["cyan"])
            if str(value) != "完整扫描":
                self.navigator.state = label if starting or self.sync.state == "syncing" else "等待有效扫描"
                self.navigator.detail = str(value)
        elif kind == "sync_error":
            self.stop()
            self._log(str(value))
            self.connection_label.configure(text="●  同步采集异常", fg=COLORS["red"])
            self.navigator.state = "采集异常"
            self.navigator.detail = str(value)
        elif kind == "sync_diagnostic":
            self._log(str(value))
        elif kind == "sync_expired":
            if self.running and not self.moving:
                self._start_hardware_scan()
        elif kind == "sync_range":
            session, pixel, distance = value
            if self.running and session == self.sync.session:
                self.current_pixel = int(pixel)
                self.latest_distance = distance
                self.measurement_rate.add(timestamp)
        elif kind == "sync_period":
            session, period = value
            if self.running and session == self.sync.session:
                self.rotation.period_s = period
                self.rotation.period_history.append(period)
        elif kind == "sync_sweep":
            session, sequence, polar_points, period = value
            if (self.running and not self.moving and session == self.sync.session
                    and polar_points
                    and polar_points[0].timestamp >= self.scan_collect_after
                    and polar_points[-1].timestamp >= self.scan_collect_after):
                self.rotation.period_s = period
                self.rotation.period_history.append(period)
                self._handle_sweep(
                    sequence,
                    [polar_to_scan_point(p) for p in polar_points],
                    timestamp,
                )
        elif kind == "sync_local_sweep":
            session, sequence, polar_points, period = value
            if (self.running and self.view_mode.get() == "radar" and not self.moving
                    and session == self.sync.session and polar_points
                    and polar_points[0].timestamp >= self.scan_collect_after
                    and polar_points[-1].timestamp >= self.scan_collect_after):
                self.rotation.period_s = period
                self.rotation.period_history.append(period)
                points = [polar_to_scan_point(p) for p in polar_points]
                self._submit_local_mapping(sequence, points, timestamp)
        elif kind == "simulation_sweep":
            if self.running and not self.moving:
                generation, sequence, polar_points, period = value
                if self.simulation is None or generation != self.simulation.generation:
                    return
                self.rotation.period_s = period
                self.rotation.period_history.append(period)
                self._handle_sweep(sequence, [polar_to_scan_point(p) for p in polar_points], timestamp)
        elif kind == "simulation_local_sweep":
            if self.running and not self.moving and self.view_mode.get() == "radar":
                generation, sequence, polar_points, period = value
                if self.simulation is None or generation != self.simulation.generation or not polar_points:
                    return
                self.rotation.period_s = period
                self.rotation.period_history.append(period)
                self._submit_local_mapping(sequence, [polar_to_scan_point(p) for p in polar_points], timestamp)
        elif kind == "simulation_observation":
            if self.running and self.moving:
                generation, packet, estimate, simulation_time = value
                if self.simulation is None or generation != self.simulation.generation:
                    return
                reason = self.motion_safety.observe(packet, estimate, float(simulation_time))
                if reason:
                    self._safety_stop(reason)
        elif kind == "simulation_motion_stopped":
            if self.running and self.moving and self.simulation is not None and value == self.simulation.generation:
                self.motion_safety.clear()
                self.moving = False
                self.accept_samples = True
        elif kind == "range" and self.accept_samples:
            distance, pixel, sequence = value  # type: ignore[misc]
            if sequence is not None:
                missing = self.sequence_monitor.add("measurement", int(sequence))
                if missing:
                    self._log(f"测距数据缺失 {missing} 帧")
            self.latest_distance = float(distance)
            self.current_pixel = int(pixel)
            self.measurement_rate.add(timestamp)
            point = self.rotation.add_sample(float(distance), int(pixel), timestamp)
            if point is not None:
                self.latest_angle = math.degrees(point.angle_rad) % 360
        elif kind == "trigger" and self.accept_samples:
            completed = self.rotation.trigger(timestamp, value if isinstance(value, int) else None)
            if completed:
                points = [polar_to_scan_point(point) for point in completed]
                self._handle_sweep(self.rotation.trigger_count - 1, points, timestamp)
        elif kind == "chassis_frame":
            generation, frame = value
            controller = getattr(self, "chassis_controller", None)
            if controller is not None:
                controller.handle_frame(int(generation), frame, timestamp)
        elif kind == "chassis_ack":
            generation, action, ack = value
            controller = getattr(self, "chassis_controller", None)
            if controller is None or generation != controller.connection_generation:
                return
            self._log(f"底盘已接受 {ack.mode} {ack.request_value} {ack.unit}")
            self.navigator.state = "底盘执行中"
            self.navigator.detail = "已收到 ACK，等待完整 DONE 报告"
        elif kind == "chassis_ack_missing":
            generation, action = value
            controller = getattr(self, "chassis_controller", None)
            if controller is None or generation != controller.connection_generation:
                return
            self.navigator.state = "底盘执行中"
            self.navigator.detail = "ACK 缺失；MOVE 不重发，继续等待匹配 DONE"
            self._log(f"底盘 ACK 缺失：动作 {action.action_id}，继续等待 DONE")
        elif kind == "chassis_done":
            generation, action, report = value
            self._handle_chassis_done(int(generation), action, report, timestamp)
        elif kind == "chassis_stop_confirmed":
            generation, status, _action = value
            controller = getattr(self, "chassis_controller", None)
            if controller is None or generation != controller.connection_generation:
                return
            self.chassis_status_var.set("底盘返回 IDLE；仍需停稳观察，未视为外部测量证明") if hasattr(self, "chassis_status_var") else None
            self._log("底盘状态回复 IDLE")
            if self.disconnect_requested and not controller.in_flight:
                self._complete_disconnect()
        elif kind == "chassis_idle":
            generation, _status = value
            controller = getattr(self, "chassis_controller", None)
            if controller is not None and generation == controller.connection_generation:
                self._log("底盘返回 IDLE")
        elif kind == "chassis_capability":
            generation, mode, _markers = value
            controller = getattr(self, "chassis_controller", None)
            if controller is None or generation != controller.connection_generation:
                return
            if hasattr(self, "chassis_status_var"):
                self.chassis_status_var.set(f"已识别底盘能力：{mode}")
            self._log(f"已识别底盘能力：{mode}")
        elif kind == "chassis_ready":
            generation, action, detail = value
            controller = getattr(self, "chassis_controller", None)
            if controller is None or generation != controller.connection_generation:
                return
            if self.disconnect_requested and not controller.in_flight:
                self._complete_disconnect()
            if hasattr(self, "chassis_status_var"):
                self.chassis_status_var.set(str(detail))
        elif kind == "chassis_waiting_scan":
            self.navigator.state = "等待停稳后新扫描"
            self.navigator.detail = "底盘 DONE 已收到，等待停稳并重新获取完整相邻扫描"
        elif kind in {"chassis_fault", "chassis_timeout", "chassis_protocol_error", "chassis_unmatched_done"}:
            self._handle_chassis_fault(kind, value)
        elif kind == "chassis_rejected":
            self._log(f"底盘动作拒绝：{value[1] if isinstance(value, tuple) and len(value) > 1 else value}")
        elif kind == "chassis_move_sent":
            generation, action = value
            controller = getattr(self, "chassis_controller", None)
            if controller is None or generation != controller.connection_generation or controller.pending is not action:
                return
            command = self._action_commands.get(action.action_id)
            if command is None:
                controller.mark_execution_failed("缺少动作安全方向，已请求停止", timestamp)
                return
            self.motion_safety.start(command, timestamp)
            self.navigator.state = "底盘执行中"
            self.navigator.detail = "单次 MOVE 已开始写入，等待 ACK/DONE 并持续监视近障"
            self._log(
                f"MOVE 开始写入：{action.mode} {action.request_value} {action.unit}，动作 {action.action_id}"
            )
        elif kind == "chassis_move_written":
            generation, action = value
            controller = getattr(self, "chassis_controller", None)
            if controller is not None and generation == controller.connection_generation:
                self._log(f"MOVE 已完整写出：动作 {action.action_id}")
        elif kind in {"chassis_raw_rx", "chassis_raw_tx"}:
            generation, action_id, payload = value
            direction = "RX" if kind.endswith("rx") else "TX"
            try:
                self.chassis_traffic_logger.record(
                    direction,
                    payload,
                    timestamp,
                    connection_generation=int(generation),
                    action_id=action_id,
                )
            except OSError as exc:
                self._log(f"底盘原始日志写入失败：{exc}")
        elif kind == "chassis_tx_failed":
            generation, action_id, written, total, detail = value
            controller = getattr(self, "chassis_controller", None)
            if controller is not None and generation == controller.connection_generation:
                self._log(f"底盘写入失败：动作 {action_id}，{written}/{total} 字节，{detail}")
        elif kind == "chassis_communication_progress":
            detail = value.get("detail", "") if isinstance(value, dict) else ""
            if isinstance(value, dict) and hasattr(self, "chassis_status_var"):
                self.chassis_status_var.set(
                    f"通信检查 {value['completed']}/{value['requested']} · 首发 {value['first_successes']} · "
                    f"重试 {value['retry_successes']} · 失败 {value['failures']}"
                )
            if detail:
                self._log(str(detail))
        elif kind == "chassis_communication_result":
            generation, result = value
            controller = getattr(self, "chassis_controller", None)
            if controller is None or generation != controller.connection_generation:
                return
            first_rate = float(result.get("first_success_rate", 0.0)) * 100.0
            retry_rate = float(result.get("retry_recovery_rate", 0.0)) * 100.0
            median = result.get("latency_median_ms")
            latency = "—" if median is None else f"{float(median):.1f} ms"
            summary = (
                f"通信检查完成：首发 {first_rate:.1f}% · 重试恢复 {retry_rate:.1f}% · "
                f"失败 {result.get('failures', 0)} · 中位延迟 {latency} · 重启 {result.get('restarts', 0)}"
            )
            if hasattr(self, "chassis_status_var"):
                self.chassis_status_var.set(summary)
            self._log(summary)
        elif kind == "error":
            if self.source == "hardware" and self.running:
                self.stop()
            self._log(str(value))
        elif kind == "info":
            self._log(str(value))
        elif kind == "rotation_status":
            self._log(f"旋转  {value}")
        elif kind == "chassis_status":
            if isinstance(value, tuple) and len(value) == 2:
                self._log(f"底盘  {value[1]}")
                detail = str(value[1])
                if (hasattr(self, "chassis_status_var")
                        and detail.startswith(("底盘参数 CRC=", "正在读取/同步底盘参数", "CONFIG1 "))):
                    self.chassis_status_var.set(detail)
            else:
                self._log(f"底盘  {value}")
        elif kind == "chassis_state":
            if isinstance(value, tuple) and len(value) == 3:
                _generation, state, detail = value
                self._verify_chassis_on_idle(_generation, state)
                if hasattr(self, "chassis_status_var"):
                    self.chassis_status_var.set(str(detail))
                self._log(f"底盘状态：{state}，{detail}")
            elif isinstance(value, tuple) and len(value) == 2:
                sequence, values = value
                missing = self.sequence_monitor.add("chassis", int(sequence))
                if missing:
                    self._log(f"底盘状态缺失 {missing} 帧")
                self.latest_chassis_state = tuple(float(item) for item in values)
        elif kind == "chassis_transport_error":
            generation, message = value
            controller = getattr(self, "chassis_controller", None)
            if controller is not None:
                controller.handle_transport_error(int(generation), str(message), timestamp)

    def _handle_sweep(self, sequence: int, points: list[ScanPoint], timestamp: float) -> None:
        self.latest_raw_points = points
        if points:
            nearest = min(points, key=lambda point: point.distance_m)
            self.latest_distance = nearest.distance_m
            self.latest_angle = math.degrees(nearest.angle_rad) % 360
        self.scan_rate.add(timestamp)
        self._submit_mapping(sequence, points, timestamp)

    def _handle_chassis_done(self, generation: int, action, report, timestamp: float) -> None:
        controller = getattr(self, "chassis_controller", None)
        if controller is None or generation != controller.connection_generation:
            return
        if action is None or controller.pending is not action:
            return
        self.latest_chassis_state = {
            "mode": report.mode,
            "reason": report.reason,
            "request_value": report.request_value,
            "request_unit": report.unit,
            "target_counts": report.target_counts,
            "encoder_progress": report.enc,
            "brake_counts": report.brake,
            "dx_counts": report.dx,
            "dy_counts": report.dy,
            "dr_counts": report.dr,
            "ds_counts": report.ds,
            "wheel_counts": report.wheels,
        }
        target_text = "" if report.target_counts is None else f" TARGET_CNT={report.target_counts}"
        self._log(
            f"底盘 DONE {report.mode} {report.reason} REQ={report.request_value} "
            f"UNIT={report.unit}{target_text} ENC={report.enc:g}"
        )
        self.motion_safety.clear()
        self.accept_samples = False
        self.manual_motion = action.source == "manual"
        estimate = None
        if action.source == "auto":
            try:
                estimate = self.chassis_adapter.execution_from_report(report)
                with self.mapping_lock:
                    self.navigator.apply_execution_delta(
                        estimate.local_x_m,
                        estimate.local_y_m,
                        estimate.yaw_rad,
                        estimate.uncertainty_m,
                        estimate.uncertainty_rad,
                    )
            except (MotionConversionError, ValueError) as exc:
                self.navigator.state = "位置待重新确认"
                self.navigator.detail = str(exc)
                self._log(str(exc))
                self._finish_chassis_action_after_settle(action, generation, timestamp, resume_auto=False)
                return
        can_resume = bool(
            action.source == "auto"
            and report.reason == "TARGET"
            and not action.stop_requested
            and self.running
            and estimate is not None
            and estimate.trusted
        )
        if can_resume:
            snapshot = self.mapping_snapshot
            self._post_motion_map_revision = (
                snapshot.map_version if snapshot is not None else int(getattr(self.navigator.grid, "_revision", 0))
            )
            self._post_motion_scan_count = (
                snapshot.completed_scans if snapshot is not None else int(self.navigator.completed_scans)
            )
            self.navigator.state = "停稳等待新扫描"
            self.navigator.detail = "已应用一次编码器执行先验，等待停稳后的完整相邻扫描"
            self._finish_chassis_action_after_settle(action, generation, timestamp, resume_auto=True)
            return

        if getattr(self, "_resume_radar_on_settle", False):
            self.navigator.set_auto(False)
            self.navigator.state = "等待停稳"
            self.navigator.detail = "模式切换已停止底盘，等待停稳后恢复仅雷达扫描"
            self._stop_radar_only()
            self._finish_chassis_action_after_settle(action, generation, timestamp, resume_auto=False)
            return
        if report.reason == "TARGET" and action.source == "manual":
            detail = "人工动作已完成；固定姿态地图参考点已失效"
        elif action.stop_requested:
            detail = f"底盘已停止（{report.reason}），不恢复自动动作"
        else:
            detail = f"底盘异常结束（{report.reason}）；已应用报告编码器先验，禁止自动续航"
        self.navigator.state = "底盘动作结束"
        self.navigator.detail = detail
        self._stop_radar_only()
        self.running = False
        self._finish_chassis_action_after_settle(action, generation, timestamp, resume_auto=False)

    def _finish_chassis_action_after_settle(self, action, generation: int, timestamp: float, *, resume_auto: bool) -> None:
        self.scan_collect_after = max(time.perf_counter(), float(timestamp)) + self._settle_duration()
        if self.config.get("synchronized_acquisition", True) and hasattr(self, "sync"):
            self.sync.begin_after(self.scan_collect_after)
        delay = max(1, round(max(0.0, self.scan_collect_after - time.perf_counter()) * 1000))
        motion_generation = self.motion_generation
        callback = lambda: self._complete_chassis_settle(
            generation,
            action.action_id,
            motion_generation,
            resume_auto,
        )
        if hasattr(self, "root"):
            self.root.after(delay, callback)
        else:
            callback()

    def _complete_chassis_settle(
        self,
        connection_generation: int,
        action_id: int,
        motion_generation: int,
        resume_auto: bool,
    ) -> None:
        controller = getattr(self, "chassis_controller", None)
        if controller is None or connection_generation != controller.connection_generation:
            return
        action = controller.pending
        if action is None or action.action_id != action_id:
            return
        should_resume = bool(
            resume_auto
            and self.running
            and not action.stop_requested
            and self.motion_generation == motion_generation
        )
        if not controller.complete_settle(resume_auto=should_resume):
            return
        if should_resume:
            self._restart_hardware_scan(motion_generation)
            return
        self._action_commands.pop(action.action_id, None)
        self.motion_safety.clear()
        self.moving = False
        self.manual_motion = False
        if getattr(self, "_resume_radar_on_settle", False) and self.running:
            self._resume_radar_on_settle = False
            self._resume_radar_after_mode_switch(motion_generation)
            return
        self.accept_samples = False
        if self.disconnect_requested and not controller.in_flight:
            self._complete_disconnect()

    def _handle_chassis_fault(self, kind: str, value: object) -> None:
        if kind == "chassis_protocol_error":
            detail = value[2] if isinstance(value, tuple) and len(value) > 2 else value
            self._log(f"底盘协议行已丢弃：{detail}")
            return
        if kind == "chassis_unmatched_done":
            detail = "收到无法归属的底盘 DONE，自动动作保持锁定"
        elif kind == "chassis_timeout" and isinstance(value, tuple):
            detail = value[-1]
        elif isinstance(value, tuple) and len(value) >= 2:
            detail = value[1]
        else:
            detail = value
        self.navigator.state = "底盘故障"
        self.navigator.detail = str(detail)
        self._log(str(detail))
        self.running = False
        self.accept_samples = False
        self.motion_safety.clear()
        self._clear_mapping_tasks()
        controller = getattr(self, "chassis_controller", None)
        self.moving = bool(controller is not None and controller.in_flight)

    def _stop_radar_only(self) -> None:
        if self.source != "hardware":
            return
        if self.config.get("synchronized_acquisition", True):
            self.sync.stop()
        else:
            SynchronizedAcquisition._stop_endpoint(self.measure_endpoint, ["STOP", "LASER 0"])
            SynchronizedAcquisition._stop_endpoint(self.rotation_endpoint, ["OFF"])

    def _complete_disconnect(self) -> None:
        if not self.disconnect_requested:
            return
        controller = getattr(self, "chassis_controller", None)
        if controller is not None and controller.in_flight:
            return
        self.disconnect_requested = False
        if controller is not None:
            controller.disconnect()
        self.measure_endpoint.close()
        self.rotation_endpoint.close()
        self.chassis_endpoint.close()
        self.connected = False
        if hasattr(self, "chassis_status_var"):
            self.chassis_status_var.set("底盘未连接")
        self.connection_label.configure(text="●  未连接", fg=COLORS["muted"])
        self.connect_button.configure(text="连接设备")

    def _execute_navigation_command(self, command: VelocityCommand) -> None:
        if self.moving or bool(getattr(getattr(self, "chassis_controller", None), "in_flight", False)):
            return
        if self.source == "simulation":
            assert self.simulation is not None
            if command.stopped:
                return
            self.moving = True
            self.accept_samples = False
            self.motion_generation += 1
            self.motion_safety.start(command, self.simulation.hardware.time)
            self.simulation.execute(command)
            with self.mapping_lock:
                self.navigator.predict_motion(command)
            return

        controller = getattr(self, "chassis_controller", None)
        if controller is None:
            self.navigator.state = "底盘控制不可用"
            self.navigator.detail = "统一底盘控制器未初始化"
            return
        try:
            request = self.chassis_adapter.request_for_command(command, automatic=True)
        except MotionConversionError as exc:
            self.navigator.state = "底盘动作拒绝"
            self.navigator.detail = str(exc)
            self._log(str(exc))
            return
        if not controller.request_move(request, source="auto"):
            self._hardware_motion_failed("底盘动作未能登记")
            return
        action = controller.pending
        if action is not None:
            self._action_commands[action.action_id] = command
        self._begin_hardware_motion(command, manual=False)

    def _begin_hardware_motion(self, command: VelocityCommand | None, *, manual: bool) -> None:
        self.manual_motion = bool(manual)
        self.moving = True
        self.accept_samples = False
        self.motion_generation += 1
        self.mapping_generation = getattr(self, "mapping_generation", 0) + 1
        if hasattr(self, "_clear_mapping_tasks"):
            self._clear_mapping_tasks()
        if manual and hasattr(self, "navigator"):
            with self.mapping_lock:
                self.navigator.reset()
                self.grid = self.navigator.grid
        self.mapping_snapshot = None
        self.scan_collect_after = math.inf
        if self.config.get("synchronized_acquisition", True):
            receiver = getattr(self.sync, "receiver", None)
            if receiver is not None:
                self.sync.begin_after(math.inf)
            else:
                self.sync.stop()
        elif hasattr(self, "rotation_endpoint"):
            self.rotation_endpoint.write_line("OFF")
        if hasattr(self, "navigator"):
            self.navigator.state = "等待底盘握手"
            self.navigator.detail = "动作已登记，等待 RX 静默、PING/PONG 与单次 MOVE"

    def _hardware_motion_failed(self, detail: str) -> None:
        self.motion_safety.clear()
        self.accept_samples = False
        self.moving = False
        self.manual_motion = False
        self.navigator.state = "底盘动作未登记"
        self.navigator.detail = detail
        self._log(detail)

    def _settle_duration(self):
        duration = float(self.config.get("hardware_settle_s", 0.2))
        if not math.isfinite(duration) or duration < 0:
            raise ValueError("停车等待时间必须是非负有限数")
        return duration

    def _restart_hardware_scan(self, generation: int) -> None:
        if not self.running or generation != self.motion_generation:
            return
        self.accept_samples = True
        self.moving = False
        if self.config.get("synchronized_acquisition", True):
            if self.sync.state == "stopped":
                self._start_hardware_scan()
            return
        self.rotation.reset()
        self.rotation_endpoint.write_line("RESETCNT")
        self.rotation_endpoint.write_line("ON")

    def _send_chassis_stop(self, wait: bool = False) -> bool:
        if self.source != "hardware" or not self.chassis_endpoint.is_open:
            return False
        controller = getattr(self, "chassis_controller", None)
        if controller is None:
            return False
        sent = bool(controller.request_stop(reason="上位机停止", now=time.perf_counter()))
        if wait:
            flush = getattr(self.chassis_endpoint, "flush", None)
            if callable(flush):
                sent = bool(flush(0.5)) and sent
        return sent

    def _draw(self) -> None:
        if self._closed:
            return
        radar_points = [(point.x, point.y, point.quality) for point in self.latest_raw_points]
        displayed_radar_count = len(self.latest_raw_points)
        receiver = self.simulation.hardware.receiver if self.simulation is not None else self.sync.receiver
        if receiver is not None:
            preview = receiver.preview_points
            if preview:
                radar_points = []
                for item in preview:
                    angle, distance = preview_item_angle_distance(item)
                    radar_points.append((distance * math.sin(angle), distance * math.cos(angle), 1.0))
                displayed_radar_count = len(preview)
                nearest = min(
                    preview,
                    key=lambda point: preview_item_angle_distance(point)[1],
                )
                angle, distance = preview_item_angle_distance(nearest)
                self.latest_angle = math.degrees(angle) % 360
                self.latest_distance = distance
                self.current_pixel = nearest.pixel if hasattr(nearest, "pixel") else self.current_pixel
            elif self.running and not self.latest_raw_points:
                self.latest_angle = None
                self.latest_distance = None
                self.current_pixel = None
        if receiver is not None and self.running and not radar_points:
            waiting_text = receiver.builder.reason
        elif self.running and not radar_points:
            waiting_text = "等待转速估计" if self.simulation is not None else "等待零位"
        else:
            waiting_text = ""
        if self.running and radar_points and self.source == "hardware" and receiver is not None:
            if receiver.preview_points:
                phase = "实时预览（尚未闭合）" if receiver.builder.samples else "闭合圈原始回波"
            else:
                phase = "最近有效扫描"
            waiting_text = f"{phase} · {displayed_radar_count} 点"
        self.radar_canvas.update_scene(
            radar_points,
            float(self.config.get("simulation_max_range_m", 3.0)
                  if self.simulation is not None else self.config.get("display_radius_m", 1.10)),
            self.latest_angle or 0.0,
            waiting_text,
        )
        snapshot = self.mapping_snapshot
        map_grid = snapshot.grid if snapshot is not None else self.grid
        map_pose = snapshot.pose if snapshot is not None else self.navigator.pose
        map_path = list(snapshot.path) if snapshot is not None else self.navigator.path_cells
        map_target = snapshot.target if snapshot is not None else self.navigator.target_cell
        map_enabled = bool(
            map_grid is not None
            and (map_grid.update_count > 0 or self.navigator.local_map_updates > 0)
        )
        self.map_canvas.update_scene(
            map_grid,
            map_pose,
            map_path,
            map_target,
            map_enabled,
        )
        completed_scans = snapshot.completed_scans if snapshot is not None else self.navigator.completed_scans
        local_map_updates = snapshot.local_map_updates if snapshot is not None else self.navigator.local_map_updates
        nav_state = snapshot.state if snapshot is not None else self.navigator.state
        nav_detail = snapshot.detail if snapshot is not None else self.navigator.detail
        self.metric_vars["distance"].set("—" if self.latest_distance is None else f"{self.latest_distance:.2f}")
        self.metric_vars["angle"].set("—" if self.latest_angle is None else f"{self.latest_angle:.1f}")
        if self.rotation.period_history:
            period = self.rotation.period_s
        else:
            period = float(self.config.get("radar_period_s", 1.5))
        self.metric_vars["period"].set("—" if not self.rotation.period_history else f"{period:.2f}")
        frequency = self.measurement_rate.value()
        self.metric_vars["frequency"].set("—" if frequency <= 0 else f"{frequency:.1f}")
        self.metric_vars["points"].set(str(displayed_radar_count))
        self.metric_vars["scans"].set(str(completed_scans))
        self.metric_vars["local_maps"].set(str(local_map_updates))
        self.metric_vars["drift"].set("—")
        self.map_percent_var.set(f"{map_grid.known_area_m2():.1f} m²")
        self.measurement_detail_var.set(
            f"像素 {self.current_pixel if self.current_pixel is not None else '—'}   "
            f"距离 {self.latest_distance:.3f} m" if self.latest_distance is not None
            else f"像素 {self.current_pixel if self.current_pixel is not None else '—'}   距离 —"
        )
        self.nav_state_var.set(nav_state)
        self.nav_detail_var.set(nav_detail)
        pose = map_pose
        self.pose_var.set(f"x {pose.x:+.2f} m   y {pose.y:+.2f} m   θ {math.degrees(pose.yaw):+.1f}°")
        score = "—" if self.navigator.match_score == 0 else f"{self.navigator.match_score:.2f}"
        self.frontier_var.set(
            f"可达前沿 {self.navigator.reachable_frontier_count}/{self.navigator.frontier_count}   匹配 {score}"
        )
        if self.simulation is not None:
            receiver = self.simulation.hardware.receiver
            self.sim_status_var.set(f"有效 {receiver.accepted} 圈 · 丢弃 {receiver.discarded} 圈 · 迟到 {receiver.late} 包\n{receiver.builder.reason}")
        if not self._closed:
            self.root.after(50, self._draw)

    def _log(self, message: str) -> None:
        self.status_history.append(f"{datetime.now():%H:%M:%S}  {message}")

    def _collect_config(self) -> dict:
        result = dict(self.config)
        if self.source == "hardware":
            result.update(
                {
                    "measurement_port": self.measure_port_var.get().strip(),
                    "rotation_port": self.rotation_port_var.get().strip(),
                    "chassis_port": self.chassis_port_var.get().strip(),
                }
            )
        return result

    def on_close(self) -> None:
        if self._closed or self._close_requested:
            return
        controller = getattr(self, "chassis_controller", None)
        if controller is not None and self.chassis_endpoint.is_open:
            self._close_requested = True
            self._close_deadline = time.perf_counter() + max(
                0.5, float(self.config.get("chassis_stop_timeout_s", 3.0)) + 0.5
            )
            self.stop()
            self.connection_label.configure(text="●  关闭中，等待底盘停止", fg=COLORS["yellow"])
            self.root.after(50, self._poll_close)
            return
        self.stop()
        self._finalize_close()

    def _poll_close(self) -> None:
        if self._closed or not self._close_requested:
            return
        controller = getattr(self, "chassis_controller", None)
        deadline = self._close_deadline if self._close_deadline is not None else time.perf_counter()
        if controller is not None and controller.in_flight:
            if time.perf_counter() >= deadline:
                self.connection_label.configure(text="●  停止未确认，保持底盘连接", fg=COLORS["red"])
            self.root.after(250 if time.perf_counter() >= deadline else 50, self._poll_close)
            return
        self._finalize_close()

    def _finalize_close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._close_requested = False
        self._close_deadline = None
        controller = getattr(self, "chassis_controller", None)
        if controller is not None:
            controller.disconnect()
        self.measure_endpoint.close()
        self.rotation_endpoint.close()
        self.chassis_endpoint.close()
        if self.source == "hardware":
            try:
                save_configuration(self._collect_config())
            except (OSError, ValueError, tk.TclError):
                pass
        runtime = getattr(self, "mapping_runtime", None)
        if runtime is not None:
            runtime.stop()
        else:
            self.mapping_stop.set()
            self._clear_mapping_tasks()
            self.mapping_tasks.put(None)
        try:
            for callback_id in self.root.tk.call("after", "info"):
                self.root.after_cancel(callback_id)
        except tk.TclError:
            pass
        self.root.destroy()


def run_app(source: str, argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="TriScan 自主建图与导航")
    parser.add_argument("--view", choices=("radar", "navigation"), default="radar")
    parser.add_argument("--profile", choices=("IDEAL", "NOMINAL", "STRESS"), help="仿真误差档位")
    parser.add_argument("--speed", type=float, help="模拟速度倍率")
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--screenshot-delay", type=int, default=2600)
    arguments = parser.parse_args(argv)
    root = tk.Tk()
    app = NavigationApp(root, source=source, initial_view=arguments.view, simulation_speed=arguments.speed, simulation_profile=arguments.profile)
    if arguments.screenshot:
        capture_window(root, arguments.screenshot.resolve(), arguments.screenshot_delay, app.on_close)
    root.mainloop()

