from __future__ import annotations

import argparse
from collections import deque
import ctypes
from datetime import datetime
import heapq
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from data_fusion import DeviceClock, SequenceMonitor, parse_chassis_state, parse_timestamped_distance, parse_trigger
from navigation_core import (
    HiddenWorld,
    NavigationEngine,
    OccupancyGrid,
    ScanPoint,
    VelocityCommand,
    mecanum_mix,
)
from radar_app import COLORS, RadarCanvas
from radar_core import CalibrationModel, CCDFrameParser, MotorLineParser, RotationTracker, SlidingRate
from serial_backend import SerialEndpoint, list_serial_ports
from synchronized_acquisition import SynchronizedAcquisition
from virtual_hardware import HardwareSimulation


if sys.platform == "win32":
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except (AttributeError, OSError):
            pass


APP_DIR = Path(__file__).resolve().parent
RADAR_CONFIG_PATH = APP_DIR / "radar_config.json"
NAV_CONFIG_PATH = APP_DIR / "navigation_config.json"

DEFAULT_CONFIG = {
    "synchronized_acquisition": True,
    "arbitrary_phase_scans": True,
    "observation_reorder_s": 0.3,
    "hardware_settle_s": 0.2,
    "hardware_sample_rate_hz": 20.0,
    "exposure_index": 8,
    "max_timing_position_error_m": 0.04,
    "clock_drift_bound_ppm": 500,
    "irq_timestamp_uncertainty_ms": 2.0,
    "sync_scan_duration_s": 30,
    "period_tolerance": 0.05,
    "max_scan_gap_deg": 25,
    "measurement_port": "",
    "rotation_port": "",
    "chassis_port": "",
    "baudrate": 115200,
    "measurement_mode": "fffe",
    "ccd_command": "@c0071#@",
    "sample_rate_hz": 80.0,
    "min_range_m": 0.08,
    "max_range_m": 3.0,
    "angle_offset_deg": 0.0,
    "clockwise": True,
    "radar_period_s": 1.5,
    "fusion_delay_ms": 80,
    "map_resolution_m": 0.04,
    "map_width_cells": 180,
    "map_height_cells": 180,
    "robot_radius_m": 0.15,
    "chassis_command_template": "",
    "chassis_stop_command": "",
    "wheel_output_scale": 1000,
    "wheel_signs": [1, 1, 1, 1],
    "simulation_sample_rate_hz": 20.0,
    "simulation_speed": 1.0,
    "simulation_profile": "NOMINAL",
    "simulation_seed": 20260907,
    "simulation_reorder_s": 0.3,
    "simulation_settle_s": 0.2,
    "simulation_map_file": "simulation_map.json",
}


def load_configuration() -> dict:
    config = dict(DEFAULT_CONFIG)
    radar_config: dict = {}
    if RADAR_CONFIG_PATH.exists():
        try:
            loaded = json.loads(RADAR_CONFIG_PATH.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                radar_config = loaded
        except (OSError, ValueError):
            pass
    for key in (
        "measurement_port",
        "baudrate",
        "ccd_command",
        "sample_rate_hz",
        "min_range_m",
        "max_range_m",
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
    config["calibration"] = radar_config.get("calibration", {})

    if NAV_CONFIG_PATH.exists():
        try:
            loaded = json.loads(NAV_CONFIG_PATH.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                config.update(loaded)
                if CalibrationModel.from_dict(radar_config.get("calibration")).ready:
                    config["calibration"] = radar_config.get("calibration", {})
                    config["measurement_mode"] = radar_config.get("ccd_parser", config["measurement_mode"])
                    config["exposure_index"] = radar_config.get("exposure_index", config["exposure_index"])
        except (OSError, ValueError):
            pass
    return config


def save_configuration(config: dict) -> None:
    temporary = NAV_CONFIG_PATH.with_suffix(".tmp")
    temporary.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(NAV_CONFIG_PATH)


class MapCanvas(tk.Canvas):
    def __init__(self, master, **kwargs) -> None:
        super().__init__(master, background="#08111f", highlightthickness=0, **kwargs)
        self.grid: OccupancyGrid | None = None
        self.pose = None
        self.path: list[tuple[int, int]] = []
        self.target: tuple[int, int] | None = None
        self.enabled = False
        self._signature = None
        self.bind("<Configure>", lambda _event: self.redraw())

    def update_scene(
        self,
        grid: OccupancyGrid,
        pose,
        path: list[tuple[int, int]],
        target: tuple[int, int] | None,
        enabled: bool,
    ) -> None:
        signature = (
            grid.update_count,
            round(pose.x, 3) if pose is not None else None,
            round(pose.y, 3) if pose is not None else None,
            round(pose.yaw, 3) if pose is not None else None,
            tuple(path),
            target,
            enabled,
            self.winfo_width(),
            self.winfo_height(),
        )
        if signature == self._signature:
            return
        self._signature = signature
        self.grid = grid
        self.pose = pose
        self.path = path
        self.target = target
        self.enabled = enabled
        self.redraw()

    def redraw(self) -> None:
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

        if self.path:
            coordinates = []
            for col, row in self.path:
                coordinates.extend((left + (col + 0.5) * cell_size, top + (row + 0.5) * cell_size))
            if len(coordinates) >= 4:
                self.create_line(*coordinates, fill=COLORS["yellow"], width=2)

        if self.target is not None:
            col, row = self.target
            x = left + (col + 0.5) * cell_size
            y = top + (row + 0.5) * cell_size
            self.create_oval(x - 5, y - 5, x + 5, y + 5, outline=COLORS["yellow"], width=2)

        if self.pose is not None:
            col, row = grid.world_to_cell(self.pose.x, self.pose.y)
            x = left + (col + 0.5) * cell_size
            y = top + (row + 0.5) * cell_size
            heading = self.pose.yaw
            tip = (x + 12 * math.sin(heading), y - 12 * math.cos(heading))
            side_a = (x + 7 * math.sin(heading + 2.45), y - 7 * math.cos(heading + 2.45))
            side_b = (x + 7 * math.sin(heading - 2.45), y - 7 * math.cos(heading - 2.45))
            self.create_polygon(*tip, *side_a, *side_b, fill=COLORS["green"], outline="#ffffff")

        self.create_text(16, 14, text="累计栅格地图", anchor="nw", fill=COLORS["muted"], font=("Microsoft YaHei UI", 10))


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
                for sequence, points, period in sweeps:
                    self.events.put(("simulation_sweep", (self.generation, sequence, points, period), started))
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
        self.config = load_configuration()
        if simulation_speed is not None:
            self.config["simulation_speed"] = simulation_speed
        if simulation_profile is not None:
            self.config["simulation_profile"] = simulation_profile
        self.events: queue.Queue = queue.Queue()
        self.pending_events: list[tuple[float, int, str, object]] = []
        self.event_counter = 0
        self.sequence_monitor = SequenceMonitor()
        self.device_clocks = {name: DeviceClock() for name in ("measurement", "rotation", "chassis")}
        self.grid = OccupancyGrid(
            int(self.config.get("map_width_cells", 180)),
            int(self.config.get("map_height_cells", 180)),
            float(self.config.get("map_resolution_m", 0.04)),
        )
        self.navigator = NavigationEngine(
            self.grid,
            float(self.config.get("max_range_m", 3.0)),
            float(self.config.get("robot_radius_m", 0.16)),
        )
        self.view_mode = tk.StringVar(value="navigation" if initial_view == "navigation" else "radar")
        self.running = False
        self.connected = source == "simulation"
        self.latest_points: list[ScanPoint] = []
        self.latest_distance: float | None = None
        self.latest_angle: float | None = None
        self.latest_bias = 0.0
        self.latest_chassis_state: tuple[float, ...] = ()
        self.scan_rate = SlidingRate(window_s=8.0)
        self.status_history: deque[str] = deque(maxlen=80)
        self.last_request_time = 0.0
        self.accept_samples = True
        self.moving = False
        self.motion_generation = 0
        self.scan_collect_after = -math.inf

        self.calibration = CalibrationModel.from_dict(self.config.get("calibration"))
        self.ccd_parser = CCDFrameParser(str(self.config.get("measurement_mode", "fffe")))
        self.measure_line_parser = MotorLineParser()
        self.rotation_parser = MotorLineParser()
        self.chassis_parser = MotorLineParser()
        self.rotation = RotationTracker(
            angle_offset_deg=float(self.config.get("angle_offset_deg", 0.0)),
            clockwise=bool(self.config.get("clockwise", True)),
            initial_period_s=float(self.config.get("radar_period_s", 1.5)),
            keep_revolutions=1,
        )

        self.simulation = SimulationSource(self.events, self.config) if source == "simulation" else None
        self.measure_endpoint = SerialEndpoint("测距串口", self._measurement_data, self._serial_error)
        self.rotation_endpoint = SerialEndpoint("旋转串口", self._rotation_data, self._serial_error)
        self.chassis_endpoint = SerialEndpoint("底盘串口", self._chassis_data, self._serial_error)
        self.sync = SynchronizedAcquisition(
            self.measure_endpoint, self.rotation_endpoint, self.calibration, self.config,
            lambda kind, value, timestamp: self.events.put((kind, value, timestamp)),
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
            text=f"硬件仿真 · {self.config.get('simulation_profile', 'NOMINAL')}" if self.source == "simulation" else "三串口实物",
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
        self.metric_vars = {key: tk.StringVar(value="—") for key in ("distance", "angle", "period", "scans", "drift")}
        metric_defs = [
            ("最近距离", "distance", "m"),
            ("当前方位", "angle", "°"),
            ("旋转周期", "period", "s"),
            ("完整扫描", "scans", "圈"),
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
        self.map_canvas = MapCanvas(self.side_panel, height=210 if self.source == "hardware" else 420)
        self.map_canvas.pack(fill="both", expand=True, padx=1)

        status = ttk.Frame(self.side_panel, style="Panel.TFrame", padding=14)
        status.pack(fill="x")
        self.nav_state_var = tk.StringVar(value="仅雷达")
        self.nav_detail_var = tk.StringVar(value="自动导航未启用")
        ttk.Label(status, textvariable=self.nav_state_var, font=("Microsoft YaHei UI", 14, "bold"), foreground=COLORS["green"]).pack(anchor="w")
        ttk.Label(status, textvariable=self.nav_detail_var, style="Muted.TLabel", wraplength=390).pack(anchor="w", pady=(3, 10))
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
        self.measure_combo = self._compact_combo(controls, "测距", self.measure_port_var)
        self.rotation_combo = self._compact_combo(controls, "旋转", self.rotation_port_var)
        self.chassis_combo = self._compact_combo(controls, "底盘", self.chassis_port_var)
        ttk.Button(controls, text="刷新串口", command=self.refresh_ports).pack(fill="x", pady=(7, 0))
        self.protocol_var = tk.StringVar()
        self.stop_command_var = tk.StringVar()
        row = ttk.Frame(controls, style="Panel.TFrame")
        row.pack(fill="x", pady=(10, 0))
        ttk.Label(row, text="四轮指令", style="Muted.TLabel", width=8).pack(side="left")
        ttk.Entry(row, textvariable=self.protocol_var).pack(side="left", fill="x", expand=True)
        row = ttk.Frame(controls, style="Panel.TFrame")
        row.pack(fill="x", pady=(5, 0))
        ttk.Label(row, text="停止指令", style="Muted.TLabel", width=8).pack(side="left")
        ttk.Entry(row, textvariable=self.stop_command_var).pack(side="left", fill="x", expand=True)

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

    def _compact_combo(self, parent, label: str, variable: tk.StringVar):
        row = ttk.Frame(parent, style="Panel.TFrame")
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=label, style="Muted.TLabel", width=8).pack(side="left")
        combo = ttk.Combobox(row, textvariable=variable, state="readonly")
        combo.pack(side="left", fill="x", expand=True)
        return combo

    def _load_ui_values(self) -> None:
        if self.source == "hardware":
            self.measure_port_var.set(str(self.config.get("measurement_port", "")))
            self.rotation_port_var.set(str(self.config.get("rotation_port", "")))
            self.chassis_port_var.set(str(self.config.get("chassis_port", "")))
            self.protocol_var.set(str(self.config.get("chassis_command_template", "")))
            self.stop_command_var.set(str(self.config.get("chassis_stop_command", "")))

    def _set_view(self, view: str) -> None:
        self.view_mode.set("navigation" if view == "navigation" else "radar")
        enabled = self.view_mode.get() == "navigation"
        self.navigator.set_auto(enabled)
        self.nav_state_var.set(self.navigator.state)
        self.nav_detail_var.set(self.navigator.detail)

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
        required = ports[:2] if self.view_mode.get() == "radar" else ports
        if not all(required):
            self._set_hardware_controls(True)
            messagebox.showwarning("串口未选择", "仅雷达需要测距和旋转串口；自动导航还需要底盘串口。")
            return
        if len(set(port for port in required if port)) != len(required):
            messagebox.showwarning("串口重复", "三个设备必须使用不同串口。")
            return
        baudrate = int(self.config.get("baudrate", 115200))
        opened = []
        try:
            self.measure_endpoint.open(ports[0], baudrate)
            opened.append(self.measure_endpoint)
            self.rotation_endpoint.open(ports[1], baudrate)
            opened.append(self.rotation_endpoint)
            if ports[2]:
                self.chassis_endpoint.open(ports[2], baudrate)
                opened.append(self.chassis_endpoint)
        except Exception as exc:
            for endpoint in opened:
                endpoint.close()
            messagebox.showerror("连接失败", str(exc))
            return
        self.connected = True
        self.connection_label.configure(text="●  设备已连接", fg=COLORS["green"])
        self.connect_button.configure(text="断开设备")
        self._log("三个数据通道已连接" if ports[2] else "测距和旋转通道已连接")

    def disconnect(self) -> None:
        self.emergency_stop()
        self.measure_endpoint.close()
        self.rotation_endpoint.close()
        self.chassis_endpoint.close()
        self.connected = False
        self.connection_label.configure(text="●  未连接", fg=COLORS["muted"])
        self.connect_button.configure(text="连接设备")

    def toggle_start(self) -> None:
        if self.running:
            self.stop()
        else:
            self.start()

    def start(self) -> None:
        if self.source == "hardware":
            if not self.connected:
                messagebox.showwarning("设备未连接", "请先连接对应串口。")
                return
            mode = str(self.config.get("measurement_mode", "fffe"))
            if (self.config.get("synchronized_acquisition", True) or mode != "timestamped_ascii") and not self.calibration.ready:
                messagebox.showwarning("尚未标定", "当前测距格式需要先在原雷达程序中完成距离标定。")
                return
            if self.view_mode.get() == "navigation":
                if not self.chassis_endpoint.is_open:
                    messagebox.showwarning("底盘未连接", "自动导航需要连接底盘串口。")
                    return
                if not self.protocol_var.get().strip() or not self.stop_command_var.get().strip():
                    self._set_hardware_controls(True)
                    messagebox.showwarning("底盘协议未配置", "请填写四轮指令模板和停止指令。")
                    return
            self._start_hardware_scan()
        else:
            assert self.simulation is not None
            self.simulation.start()
            self.connection_label.configure(text="●  模拟运行", fg=COLORS["cyan"])
        self.running = True
        self.start_button.configure(text="暂停")
        self._log("开始自动导航" if self.view_mode.get() == "navigation" else "开始雷达扫描")

    def stop(self) -> None:
        self.motion_generation += 1
        if self.simulation:
            self.simulation.stop()
        if self.source == "hardware":
            if self.config.get("synchronized_acquisition", True):
                self.sync.stop()
            self.rotation_endpoint.write_line("OFF")
            self.measure_endpoint.write_line("LASER 0")
            self._send_chassis_stop()
        self.running = False
        self.accept_samples = False
        self.moving = False
        self.start_button.configure(text="开始")
        self.connection_label.configure(
            text="●  已暂停" if self.connected else "●  未连接",
            fg=COLORS["yellow"] if self.connected else COLORS["muted"],
        )

    def emergency_stop(self) -> None:
        self.stop()
        self.navigator.set_auto(False)
        self.view_mode.set("radar")
        self._log("紧急停止")

    def reset(self) -> None:
        was_running = self.running
        if self.simulation:
            self.simulation.stop()
            self.simulation.reset()
        self.navigator.reset()
        self.navigator.set_auto(self.view_mode.get() == "navigation")
        self.latest_points.clear()
        self.latest_bias = 0.0
        if was_running and self.simulation:
            self.simulation.start()
        self._log("模拟场景已重置")

    def _start_hardware_scan(self) -> None:
        self.scan_collect_after = time.perf_counter()
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

    def _chassis_data(self, data: bytes, host_time: float) -> None:
        for line in self.chassis_parser.feed(data):
            parsed = parse_chassis_state(line)
            if parsed is None:
                self.events.put(("chassis_status", line, host_time))
                continue
            sequence, device_us, values = parsed
            timestamp = self.device_clocks["chassis"].observe(device_us, host_time)
            self.events.put(("chassis_state", (sequence, values), timestamp))

    def _serial_error(self, message: str) -> None:
        self.events.put(("error", message, time.perf_counter()))

    def _queue_event(self, kind: str, value: object, timestamp: float) -> None:
        self.event_counter += 1
        heapq.heappush(self.pending_events, (timestamp, self.event_counter, kind, value))

    def _poll(self) -> None:
        now = time.perf_counter()
        if self.source == "hardware" and self.config.get("synchronized_acquisition", True):
            self.sync.poll(now)
        event_limit = 2 if self.source == "simulation" else 64
        for _ in range(event_limit):
            try:
                kind, value, timestamp = self.events.get_nowait()
                self._queue_event(str(kind), value, float(timestamp))
            except queue.Empty:
                break

        delay = (0.0 if self.source == "simulation" or self.config.get("synchronized_acquisition", True)
                 else float(self.config.get("fusion_delay_ms", 80)) / 1000.0)
        cutoff = now - delay
        while self.pending_events and self.pending_events[0][0] <= cutoff:
            timestamp, _, kind, value = heapq.heappop(self.pending_events)
            self._handle_event(kind, value, timestamp)

        if self.running and self.source == "hardware" and not self.config.get("synchronized_acquisition", True) and self.accept_samples and self.measure_endpoint.is_open:
            interval = 1.0 / max(1.0, float(self.config.get("sample_rate_hz", 80.0)))
            if now - self.last_request_time >= interval:
                self.measure_endpoint.write_line(str(self.config.get("ccd_command", "@c0071#@")))
                self.last_request_time = now
        self.root.after(15, self._poll)

    def _handle_event(self, kind: str, value: object, timestamp: float) -> None:
        if kind == "sync_status":
            if not self.running:
                return
            self._log(str(value))
            starting = self.sync.state in {"starting_measurement", "starting_rotation"}
            label = "校时中" if self.sync.state == "syncing" else ("设备启动中" if starting else "同步采集中")
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
        elif kind == "sync_expired":
            if self.running and not self.moving:
                self._start_hardware_scan()
        elif kind == "sync_range":
            session, distance = value
            if self.running and session == self.sync.session:
                self.latest_distance = distance
        elif kind == "sync_period":
            session, period = value
            if self.running and session == self.sync.session:
                self.rotation.period_s = period
                self.rotation.period_history.append(period)
        elif kind == "sync_sweep":
            session, sequence, polar_points, period = value
            if (self.running and not self.moving and session == self.sync.session
                    and polar_points and polar_points[0].timestamp >= self.scan_collect_after):
                self.rotation.period_s = period
                self.rotation.period_history.append(period)
                self._handle_sweep(sequence, [ScanPoint(p.angle_rad, p.distance_m) for p in polar_points], timestamp)
        elif kind == "simulation_sweep":
            if self.running:
                generation, sequence, polar_points, period = value
                if self.simulation is None or generation != self.simulation.generation:
                    return
                self.rotation.period_s = period
                self.rotation.period_history.append(period)
                self._handle_sweep(sequence, [ScanPoint(p.angle_rad, p.distance_m) for p in polar_points], timestamp)
        elif kind == "range" and self.accept_samples:
            distance, pixel, sequence = value  # type: ignore[misc]
            if sequence is not None:
                missing = self.sequence_monitor.add("measurement", int(sequence))
                if missing:
                    self._log(f"测距数据缺失 {missing} 帧")
            self.latest_distance = float(distance)
            point = self.rotation.add_sample(float(distance), int(pixel), timestamp)
            if point is not None:
                self.latest_angle = math.degrees(point.angle_rad) % 360
        elif kind == "trigger" and self.accept_samples:
            completed = self.rotation.trigger(timestamp, value if isinstance(value, int) else None)
            if completed:
                points = [ScanPoint(point.angle_rad, point.distance_m) for point in completed]
                self._handle_sweep(self.rotation.trigger_count - 1, points, timestamp)
        elif kind == "motion_sent":
            generation, command = value
            if self.running and self.moving and generation == self.motion_generation:
                self.navigator.predict_motion(command)
                remaining = timestamp + command.duration_s - time.perf_counter()
                self.root.after(max(1, round(remaining * 1000)),
                                lambda: self._finish_hardware_motion(generation))
        elif kind == "motion_stopped":
            generation = value
            if self.running and self.moving and generation == self.motion_generation:
                self.scan_collect_after = timestamp + float(self.config.get("hardware_settle_s", 0.2))
                self.sync.begin_after(self.scan_collect_after)
                remaining = self.scan_collect_after - time.perf_counter()
                self.root.after(max(1, round(remaining * 1000)),
                                lambda: self._restart_hardware_scan(generation))
        elif kind == "error":
            if self.source == "hardware" and self.running:
                self.stop()
            self._log(str(value))
        elif kind == "info":
            self._log(str(value))
        elif kind == "rotation_status":
            self._log(f"旋转  {value}")
        elif kind == "chassis_status":
            self._log(f"底盘  {value}")
        elif kind == "chassis_state":
            sequence, values = value  # type: ignore[misc]
            missing = self.sequence_monitor.add("chassis", int(sequence))
            if missing:
                self._log(f"底盘状态缺失 {missing} 帧")
            self.latest_chassis_state = tuple(float(item) for item in values)

    def _handle_sweep(self, sequence: int, points: list[ScanPoint], timestamp: float) -> None:
        self.latest_points = points
        if points:
            nearest = min(points, key=lambda point: point.distance_m)
            self.latest_distance = nearest.distance_m
            self.latest_angle = math.degrees(nearest.angle_rad) % 360
        self.scan_rate.add(timestamp)
        command = self.navigator.process_scan(points)
        if self.view_mode.get() == "navigation" and self.running and not command.stopped:
            self._execute_navigation_command(command)

    def _execute_navigation_command(self, command: VelocityCommand) -> None:
        if self.moving:
            return
        if self.source == "simulation":
            assert self.simulation is not None
            self.simulation.execute(command)
            self.navigator.predict_motion(command)
            return

        template = self.protocol_var.get().strip()
        if not template:
            self.navigator.state = "等待底盘协议"
            self.navigator.detail = "未发送运动指令"
            return
        wheel_values = mecanum_mix(command)
        signs = self.config.get("wheel_signs", [1, 1, 1, 1])
        if not isinstance(signs, list) or len(signs) != 4:
            signs = [1, 1, 1, 1]
        scale = int(self.config.get("wheel_output_scale", 1000))
        outputs = [round(value * scale * int(sign)) for value, sign in zip(wheel_values, signs)]
        try:
            line = template.format(
                fl=outputs[0],
                fr=outputs[1],
                rl=outputs[2],
                rr=outputs[3],
                duration_ms=round(command.duration_s * 1000),
            )
        except (KeyError, ValueError) as exc:
            self.navigator.state = "底盘协议错误"
            self.navigator.detail = str(exc)
            return
        self.moving = True
        self.accept_samples = False
        self.motion_generation += 1
        generation = self.motion_generation
        self.scan_collect_after = math.inf
        if self.config.get("synchronized_acquisition", True):
            if self.sync.receiver is not None:
                self.sync.begin_after(math.inf)
            else:
                self.sync.stop()
        else:
            self.rotation_endpoint.write_line("OFF")
        if not self.chassis_endpoint.write_line(
                line, lambda stamp: self.events.put(("motion_sent", (generation, command), stamp))):
            self.stop()
            self._log("底盘运动指令发送失败")

    def _finish_hardware_motion(self, generation: int) -> None:
        if not self.running or not self.moving or generation != self.motion_generation:
            return
        command = self.stop_command_var.get().strip()
        if not command or not self.chassis_endpoint.write_line(
                command, lambda stamp: self.events.put(("motion_stopped", generation, stamp))):
            self.stop()
            self._log("底盘停止指令发送失败")

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

    def _send_chassis_stop(self) -> None:
        if self.source != "hardware" or not self.chassis_endpoint.is_open:
            return
        command = self.stop_command_var.get().strip() if hasattr(self, "stop_command_var") else ""
        if command:
            self.chassis_endpoint.write_line(command)

    def _draw(self) -> None:
        radar_points = [(point.x, point.y, 1.0) for point in self.latest_points]
        receiver = self.simulation.hardware.receiver if self.simulation is not None else self.sync.receiver
        if receiver is not None:
            preview = receiver.preview_points
            radar_points = [(distance * math.sin(angle), distance * math.cos(angle), 1.0)
                            for _, angle, distance in preview]
            if preview:
                self.latest_angle = math.degrees(preview[-1][1]) % 360
                self.latest_distance = min(point[2] for point in preview)
        self.radar_canvas.update_scene(
            radar_points,
            float(self.config.get("max_range_m", 3.0)),
            self.latest_angle or 0.0,
            ("等待转速估计" if self.simulation is not None else "等待完整扫描") if self.running and not radar_points else "",
        )
        navigation_view = self.view_mode.get() == "navigation"
        self.map_canvas.update_scene(
            self.grid,
            self.navigator.pose,
            self.navigator.path_cells,
            self.navigator.target_cell,
            navigation_view,
        )
        self.metric_vars["distance"].set("—" if self.latest_distance is None else f"{self.latest_distance:.2f}")
        self.metric_vars["angle"].set("—" if self.latest_angle is None else f"{self.latest_angle:.1f}")
        if self.rotation.period_history:
            period = self.rotation.period_s
        else:
            period = float(self.config.get("radar_period_s", 1.5))
        self.metric_vars["period"].set("—" if not self.rotation.period_history else f"{period:.2f}")
        self.metric_vars["scans"].set(str(self.navigator.completed_scans))
        self.metric_vars["drift"].set("—")
        self.map_percent_var.set(f"{self.grid.known_area_m2():.1f} m²" if navigation_view else "—")
        self.nav_state_var.set(self.navigator.state)
        self.nav_detail_var.set(self.navigator.detail)
        pose = self.navigator.pose
        self.pose_var.set(f"x {pose.x:+.2f} m   y {pose.y:+.2f} m   θ {math.degrees(pose.yaw):+.1f}°")
        score = "—" if self.navigator.match_score == 0 else f"{self.navigator.match_score:.2f}"
        self.frontier_var.set(
            f"可达前沿 {self.navigator.reachable_frontier_count}/{self.navigator.frontier_count}   匹配 {score}"
        )
        if self.simulation is not None:
            receiver = self.simulation.hardware.receiver
            self.sim_status_var.set(f"有效 {receiver.accepted} 圈 · 丢弃 {receiver.discarded} 圈 · 迟到 {receiver.late} 包\n{receiver.builder.reason}")
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
                    "chassis_command_template": self.protocol_var.get().strip(),
                    "chassis_stop_command": self.stop_command_var.get().strip(),
                }
            )
        return result

    def on_close(self) -> None:
        self.stop()
        self.measure_endpoint.close()
        self.rotation_endpoint.close()
        self.chassis_endpoint.close()
        if self.source == "hardware":
            try:
                save_configuration(self._collect_config())
            except (OSError, ValueError, tk.TclError):
                pass
        self.root.destroy()


def capture_window(root: tk.Tk, path: Path, delay_ms: int) -> None:
    def grab() -> None:
        root.deiconify()
        root.lift()
        root.attributes("-topmost", True)
        root.update_idletasks()
        root.update()
        try:
            from PIL import ImageGrab

            x = root.winfo_rootx()
            y = root.winfo_rooty()
            image = ImageGrab.grab(
                bbox=(x, y, x + root.winfo_width(), y + root.winfo_height()),
                all_screens=True,
            )
            path.parent.mkdir(parents=True, exist_ok=True)
            image.save(path)
        finally:
            root.attributes("-topmost", False)
            root.after(80, root.destroy)

    root.after(delay_ms, grab)


def run_app(source: str, argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="TriScan 自主建图与导航")
    parser.add_argument("--view", choices=("radar", "navigation"), default="radar")
    parser.add_argument("--profile", choices=("IDEAL", "NOMINAL", "STRESS"), help="仿真误差档位")
    parser.add_argument("--speed", type=float, help="模拟速度倍率")
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--screenshot-delay", type=int, default=2600)
    arguments = parser.parse_args(argv)
    root = tk.Tk()
    NavigationApp(root, source=source, initial_view=arguments.view, simulation_speed=arguments.speed, simulation_profile=arguments.profile)
    if arguments.screenshot:
        capture_window(root, arguments.screenshot.resolve(), arguments.screenshot_delay)
    root.mainloop()

