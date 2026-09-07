from __future__ import annotations

import argparse
from collections import deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import random
import re
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

from radar_core import CalibrationModel, CCDFrameParser, MotorLineParser, RotationTracker, SlidingRate
from serial_backend import SerialEndpoint, list_serial_ports


APP_NAME = "TriScan 雷达控制台"
APP_DIR = Path(__file__).resolve().parent
CONFIG_PATH = APP_DIR / "radar_config.json"

COLORS = {
    "bg": "#08111f",
    "panel": "#101c2c",
    "panel_alt": "#132237",
    "border": "#233852",
    "text": "#e8f1fb",
    "muted": "#8396ad",
    "green": "#38e7a3",
    "green_dark": "#127a5a",
    "cyan": "#4cc9f0",
    "yellow": "#ffcc66",
    "red": "#ff6b6b",
    "grid": "#17324b",
    "grid_major": "#28506c",
}


DEFAULT_CONFIG = {
    "measurement_port": "",
    "motor_port": "",
    "baudrate": 115200,
    "ccd_command": "@c0071#@",
    "ccd_parser": "fffe",
    "exposure_index": 8,
    "sample_rate_hz": 20.0,
    "max_range_m": 3.0,
    "min_range_m": 0.08,
    "angle_offset_deg": 0.0,
    "clockwise": True,
    "keep_revolutions": 3,
    "initial_period_s": 2.5,
    "calibration": {"p0": None, "k": None, "rmse": None, "points": []},
}


def load_config() -> dict:
    config = dict(DEFAULT_CONFIG)
    if CONFIG_PATH.exists():
        try:
            stored = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            if isinstance(stored, dict):
                config.update(stored)
        except (OSError, ValueError):
            pass
    return config


def save_config(config: dict) -> None:
    temporary = CONFIG_PATH.with_suffix(".tmp")
    temporary.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(CONFIG_PATH)


class RadarCanvas(tk.Canvas):
    def __init__(self, master, **kwargs) -> None:
        super().__init__(master, background=COLORS["bg"], highlightthickness=0, **kwargs)
        self.points: list[tuple[float, float, float]] = []
        self.max_range_m = 3.0
        self.heading_deg = 0.0
        self.waiting_text = "等待光电零位"
        self.bind("<Configure>", lambda _: self.redraw())

    def update_scene(
        self,
        points: list[tuple[float, float, float]],
        max_range_m: float,
        heading_deg: float,
        waiting_text: str = "",
    ) -> None:
        self.points = points
        self.max_range_m = max(0.1, max_range_m)
        self.heading_deg = heading_deg
        self.waiting_text = waiting_text
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        width = max(100, self.winfo_width())
        height = max(100, self.winfo_height())
        cx = width / 2
        cy = height / 2
        radius = max(30, min(width, height) * 0.43)

        self.create_rectangle(0, 0, width, height, fill=COLORS["bg"], outline="")
        for index in range(1, 6):
            r = radius * index / 5
            self.create_oval(cx - r, cy - r, cx + r, cy + r, outline=COLORS["grid_major"] if index == 5 else COLORS["grid"], width=1)
            distance = self.max_range_m * index / 5
            self.create_text(cx + 6, cy - r + 3, text=f"{distance:.1f} m", anchor="nw", fill=COLORS["muted"], font=("Segoe UI", 9))

        for angle_deg in range(0, 360, 30):
            angle = math.radians(angle_deg)
            x = cx + radius * math.sin(angle)
            y = cy - radius * math.cos(angle)
            self.create_line(cx, cy, x, y, fill=COLORS["grid"], width=1)
            label_r = radius + 18
            lx = cx + label_r * math.sin(angle)
            ly = cy - label_r * math.cos(angle)
            self.create_text(lx, ly, text=f"{angle_deg}°", fill=COLORS["muted"], font=("Segoe UI", 9))

        heading = math.radians(self.heading_deg)
        hx = cx + radius * math.sin(heading)
        hy = cy - radius * math.cos(heading)
        self.create_line(cx, cy, hx, hy, fill=COLORS["green_dark"], width=2, dash=(7, 5))

        scale = radius / self.max_range_m
        for x_m, y_m, alpha in self.points:
            if math.hypot(x_m, y_m) > self.max_range_m:
                continue
            x = cx + x_m * scale
            y = cy - y_m * scale
            color = blend(COLORS["bg"], COLORS["green"], alpha)
            size = 2.0 + alpha * 1.8
            self.create_oval(x - size, y - size, x + size, y + size, fill=color, outline="")

        self.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill=COLORS["cyan"], outline=COLORS["text"], width=2)
        self.create_polygon(cx, cy - 18, cx - 6, cy - 6, cx + 6, cy - 6, fill=COLORS["cyan"], outline="")

        self.create_text(22, 20, text="局部坐标 / 车头朝上", anchor="nw", fill=COLORS["muted"], font=("Microsoft YaHei UI", 10))
        if self.waiting_text:
            self.create_text(cx, cy + radius * 0.62, text=self.waiting_text, fill=COLORS["yellow"], font=("Microsoft YaHei UI", 13, "bold"))


class ScrollableTab(ttk.Frame):
    def __init__(self, master) -> None:
        super().__init__(master, style="Panel.TFrame")
        self.canvas = tk.Canvas(self, background=COLORS["panel"], highlightthickness=0, borderwidth=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.body = ttk.Frame(self.canvas, style="Panel.TFrame", padding=14)
        self.window_id = self.canvas.create_window((0, 0), window=self.body, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        self.body.bind("<Configure>", self._update_scrollregion)
        self.canvas.bind("<Configure>", self._resize_body)
        self.canvas.bind("<Enter>", lambda _: self.canvas.bind_all("<MouseWheel>", self._mousewheel))
        self.canvas.bind("<Leave>", lambda _: self.canvas.unbind_all("<MouseWheel>"))

    def _update_scrollregion(self, _event=None) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _resize_body(self, event) -> None:
        self.canvas.itemconfigure(self.window_id, width=event.width)

    def _mousewheel(self, event) -> None:
        self.canvas.yview_scroll(int(-event.delta / 120), "units")


def blend(background: str, foreground: str, alpha: float) -> str:
    alpha = max(0.0, min(1.0, alpha))
    bg = tuple(int(background[i : i + 2], 16) for i in (1, 3, 5))
    fg = tuple(int(foreground[i : i + 2], 16) for i in (1, 3, 5))
    result = tuple(round(b + (f - b) * alpha) for b, f in zip(bg, fg))
    return "#" + "".join(f"{value:02x}" for value in result)


class Simulator:
    def __init__(self, event_queue: queue.Queue) -> None:
        self.event_queue = event_queue
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.period_s = 2.5
        self.rate_hz = 28.0
        self.p0 = 740.0
        self.k = 120.0

    def set_rate_hz(self, value: float) -> None:
        self.rate_hz = max(0.5, min(200.0, float(value)))

    def start(self) -> None:
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, name="radar-simulator", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None

    def _run(self) -> None:
        now = time.perf_counter()
        last_update = now
        next_sample = now
        phase = 0.0
        count = 0
        count += 1
        self.event_queue.put(("motor_line", f"TRIG {count}", now))
        while not self.stop_event.is_set():
            now = time.perf_counter()
            elapsed = now - last_update
            last_update = now
            phase += elapsed / max(self.period_s, 1e-3)
            while phase >= 1.0:
                phase -= 1.0
                count += 1
                self.event_queue.put(("motor_line", f"TRIG {count}", now))
            if now >= next_sample:
                angle = math.tau * phase
                distance = simulated_distance(angle) + random.gauss(0, 0.008)
                pixel = round(self.p0 + self.k / max(distance, 0.1) + random.gauss(0, 0.45))
                self.event_queue.put(("ccd_pixel", pixel, now))
                next_sample += 1.0 / self.rate_hz
            time.sleep(0.002)


def simulated_distance(angle: float) -> float:
    x_dir = math.sin(angle)
    y_dir = math.cos(angle)
    distances = [3.0]
    walls = [("y", 1.8), ("y", -1.25), ("x", 1.45), ("x", -1.05)]
    for axis, value in walls:
        direction = y_dir if axis == "y" else x_dir
        if abs(direction) > 1e-6 and value / direction > 0:
            distances.append(value / direction)
    for center_x, center_y, radius in ((0.45, 0.7, 0.18), (-0.55, 0.35, 0.14)):
        projection = center_x * x_dir + center_y * y_dir
        perpendicular_sq = center_x * center_x + center_y * center_y - projection * projection
        if projection > 0 and perpendicular_sq <= radius * radius:
            distances.append(projection - math.sqrt(radius * radius - perpendicular_sq))
    return max(0.12, min(distances))


class RadarApp:
    def __init__(self, root: tk.Tk, demo: bool = False) -> None:
        self.root = root
        self.demo = demo
        self.config = load_config()
        self.events: queue.Queue = queue.Queue()
        self.ccd_parser = CCDFrameParser(str(self.config.get("ccd_parser", "fffe")))
        self.motor_parser = MotorLineParser()
        self.calibration = CalibrationModel.from_dict(self.config.get("calibration"))
        if demo:
            self.calibration = CalibrationModel(p0=740.0, k=120.0, points=[(860, 1.0), (800, 2.0)], rmse=0.0)
        self.rotation = RotationTracker(
            angle_offset_deg=float(self.config.get("angle_offset_deg", 0.0)),
            clockwise=bool(self.config.get("clockwise", True)),
            initial_period_s=float(self.config.get("initial_period_s", 2.5)),
            keep_revolutions=int(self.config.get("keep_revolutions", 3)),
        )
        self.sample_rate = SlidingRate()
        self.latest_pixel: int | None = None
        self.latest_distance: float | None = None
        self.latest_angle_deg: float | None = None
        self.scanning = False
        self.connected = False
        self.laser_on = False
        self.last_request_time = 0.0
        self.last_motor_message = "—"
        self.log_file = None
        self.log_writer = None
        self.log_path: Path | None = None
        self.log_rows = 0
        self.simulator = Simulator(self.events)
        self.status_history: deque[str] = deque(maxlen=80)

        self.measure_endpoint = SerialEndpoint("测距串口", self._measurement_data, self._serial_error)
        self.motor_endpoint = SerialEndpoint("传动串口", self._motor_data, self._serial_error)

        self._build_window()
        self._build_styles()
        self._build_ui()
        self._load_variables()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(20, self._poll)
        self.root.after(100, self._draw)
        if demo:
            self.root.after(250, self.start_simulation)

    def _build_window(self) -> None:
        self.root.title(APP_NAME)
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        width = min(1380, max(1060, screen_width - 90))
        height = min(860, max(680, screen_height - 110))
        self.root.geometry(f"{width}x{height}")
        self.root.minsize(min(1060, width), min(680, height))
        self.root.configure(background=COLORS["bg"])

    def _build_styles(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("App.TFrame", background=COLORS["bg"])
        style.configure("Panel.TFrame", background=COLORS["panel"])
        style.configure("Alt.TFrame", background=COLORS["panel_alt"])
        style.configure("TLabel", background=COLORS["panel"], foreground=COLORS["text"], font=("Microsoft YaHei UI", 10))
        style.configure("Muted.TLabel", background=COLORS["panel"], foreground=COLORS["muted"], font=("Microsoft YaHei UI", 9))
        style.configure("Title.TLabel", background=COLORS["bg"], foreground=COLORS["text"], font=("Microsoft YaHei UI", 20, "bold"))
        style.configure("Subtitle.TLabel", background=COLORS["bg"], foreground=COLORS["muted"], font=("Microsoft YaHei UI", 9))
        style.configure("Metric.TLabel", background=COLORS["panel"], foreground=COLORS["green"], font=("Segoe UI", 18, "bold"))
        style.configure("MetricUnit.TLabel", background=COLORS["panel"], foreground=COLORS["muted"], font=("Microsoft YaHei UI", 9))
        style.configure("TButton", background=COLORS["panel_alt"], foreground=COLORS["text"], bordercolor=COLORS["border"], padding=(10, 7), font=("Microsoft YaHei UI", 9))
        style.map("TButton", background=[("active", "#1b3350"), ("pressed", "#0d1827")])
        style.configure("Primary.TButton", background=COLORS["green_dark"], foreground="#ffffff", bordercolor=COLORS["green"], padding=(14, 8), font=("Microsoft YaHei UI", 10, "bold"))
        style.map("Primary.TButton", background=[("active", "#15966d"), ("pressed", "#0b6248")])
        style.configure("Danger.TButton", background="#612d39", foreground="#ffffff", bordercolor=COLORS["red"], padding=(14, 8), font=("Microsoft YaHei UI", 10, "bold"))
        style.map("Danger.TButton", background=[("active", "#81394a")])
        style.configure("TEntry", fieldbackground="#0b1726", foreground=COLORS["text"], insertcolor=COLORS["text"], bordercolor=COLORS["border"], padding=5)
        style.configure("TCombobox", fieldbackground="#0b1726", background="#0b1726", foreground=COLORS["text"], arrowcolor=COLORS["muted"], bordercolor=COLORS["border"], padding=4)
        style.map("TCombobox", fieldbackground=[("readonly", "#0b1726")], foreground=[("readonly", COLORS["text"])])
        style.configure("TCheckbutton", background=COLORS["panel"], foreground=COLORS["text"], indicatorbackground="#0b1726", indicatorforeground=COLORS["green"], font=("Microsoft YaHei UI", 9))
        style.map("TCheckbutton", background=[("active", COLORS["panel"])])
        style.configure("Treeview", background="#0b1726", fieldbackground="#0b1726", foreground=COLORS["text"], bordercolor=COLORS["border"], rowheight=25, font=("Segoe UI", 9))
        style.configure("Treeview.Heading", background=COLORS["panel_alt"], foreground=COLORS["muted"], bordercolor=COLORS["border"], font=("Microsoft YaHei UI", 9, "bold"))
        style.map("Treeview", background=[("selected", COLORS["green_dark"])])
        style.configure("TNotebook", background=COLORS["panel"], borderwidth=0)
        style.configure("TNotebook.Tab", background=COLORS["panel_alt"], foreground=COLORS["muted"], padding=(12, 7), font=("Microsoft YaHei UI", 9))
        style.map("TNotebook.Tab", background=[("selected", COLORS["panel"])], foreground=[("selected", COLORS["text"])])

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, style="App.TFrame", padding=18)
        outer.pack(fill="both", expand=True)

        header = ttk.Frame(outer, style="App.TFrame")
        header.pack(fill="x", pady=(0, 14))
        ttk.Label(header, text="TriScan", style="Title.TLabel").pack(side="left")
        ttk.Label(header, text="旋转三角测距雷达", style="Subtitle.TLabel").pack(side="left", padx=(12, 0), pady=(9, 0))
        self.mode_badge = tk.Label(header, text="模拟模式" if self.demo else "设备模式", bg="#193852" if self.demo else COLORS["panel_alt"], fg=COLORS["cyan"] if self.demo else COLORS["muted"], padx=12, pady=6, font=("Microsoft YaHei UI", 9, "bold"))
        self.mode_badge.pack(side="right")

        content = ttk.Panedwindow(outer, orient="horizontal")
        content.pack(fill="both", expand=True)

        radar_panel = ttk.Frame(content, style="Panel.TFrame", padding=1)
        side_panel = ttk.Frame(content, style="Panel.TFrame", width=400)
        content.add(radar_panel, weight=4)
        content.add(side_panel, weight=2)

        toolbar = ttk.Frame(radar_panel, style="Panel.TFrame", padding=(14, 12))
        toolbar.pack(fill="x")
        self.connection_label = tk.Label(toolbar, text="●  未连接", bg=COLORS["panel"], fg=COLORS["muted"], font=("Microsoft YaHei UI", 10, "bold"))
        self.connection_label.pack(side="left")
        ttk.Button(toolbar, text="刷新串口", command=self.refresh_ports).pack(side="right", padx=(8, 0))
        self.connect_button = ttk.Button(toolbar, text="连接设备", command=self.toggle_connection)
        self.connect_button.pack(side="right")

        self.radar_canvas = RadarCanvas(radar_panel)
        self.radar_canvas.pack(fill="both", expand=True)

        metrics = ttk.Frame(radar_panel, style="Panel.TFrame", padding=(14, 10))
        metrics.pack(fill="x")
        self.metric_vars = {name: tk.StringVar(value="—") for name in ("distance", "angle", "rpm", "rate")}
        metric_defs = [("实时距离", "distance", "m"), ("方位角", "angle", "°"), ("转速", "rpm", "rpm"), ("采样率", "rate", "Hz")]
        for index, (label, key, unit) in enumerate(metric_defs):
            block = ttk.Frame(metrics, style="Panel.TFrame")
            block.grid(row=0, column=index, sticky="ew", padx=(0 if index == 0 else 18, 0))
            ttk.Label(block, text=label, style="Muted.TLabel").pack(anchor="w")
            value_row = ttk.Frame(block, style="Panel.TFrame")
            value_row.pack(anchor="w")
            ttk.Label(value_row, textvariable=self.metric_vars[key], style="Metric.TLabel").pack(side="left")
            ttk.Label(value_row, text=unit, style="MetricUnit.TLabel").pack(side="left", padx=(5, 0), pady=(8, 0))
            metrics.columnconfigure(index, weight=1)

        notebook = ttk.Notebook(side_panel)
        notebook.pack(fill="both", expand=True, padx=12, pady=12)
        control_scroll = ScrollableTab(notebook)
        control_tab = control_scroll.body
        calibration_tab = ttk.Frame(notebook, style="Panel.TFrame", padding=14)
        log_tab = ttk.Frame(notebook, style="Panel.TFrame", padding=14)
        notebook.add(control_scroll, text="控制")
        notebook.add(calibration_tab, text="标定")
        notebook.add(log_tab, text="记录")
        self._build_control_tab(control_tab)
        self._build_calibration_tab(calibration_tab)
        self._build_log_tab(log_tab)

    def _build_control_tab(self, parent) -> None:
        self.measure_port_var = tk.StringVar()
        self.motor_port_var = tk.StringVar()
        self.command_var = tk.StringVar()
        self.parser_var = tk.StringVar()
        self.exposure_var = tk.IntVar()
        self.sample_rate_var = tk.DoubleVar()
        self.min_range_var = tk.DoubleVar()
        self.max_range_var = tk.DoubleVar()
        self.angle_offset_var = tk.DoubleVar()
        self.clockwise_var = tk.BooleanVar()
        self.keep_revolutions_var = tk.IntVar()

        ttk.Label(parent, text="设备连接", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w", pady=(0, 10))
        grid = ttk.Frame(parent, style="Panel.TFrame")
        grid.pack(fill="x")
        self.measure_combo = self._labeled_combo(grid, 0, "测距串口", self.measure_port_var, [])
        self.motor_combo = self._labeled_combo(grid, 1, "传动串口", self.motor_port_var, [])

        ttk.Separator(parent).pack(fill="x", pady=16)
        ttk.Label(parent, text="采集协议", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w", pady=(0, 10))
        proto = ttk.Frame(parent, style="Panel.TFrame")
        proto.pack(fill="x")
        self._labeled_entry(proto, 0, "中心读取指令", self.command_var)
        parser_values = list(CCDFrameParser.MODES.values())
        parser_combo = self._labeled_combo(proto, 1, "返回帧格式", self.parser_var, parser_values)
        parser_combo.configure(state="readonly")
        self._labeled_combo(proto, 2, "曝光档位", self.exposure_var, [str(i) for i in range(14)]).configure(state="readonly")
        self._labeled_entry(proto, 3, "采样频率 / Hz", self.sample_rate_var)

        ttk.Separator(parent).pack(fill="x", pady=16)
        ttk.Label(parent, text="扫描参数", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w", pady=(0, 10))
        scan = ttk.Frame(parent, style="Panel.TFrame")
        scan.pack(fill="x")
        self._labeled_entry(scan, 0, "最近距离 / m", self.min_range_var)
        self._labeled_entry(scan, 1, "显示半径 / m", self.max_range_var)
        self._labeled_entry(scan, 2, "零位偏移 / °", self.angle_offset_var)
        self._labeled_combo(scan, 3, "保留圈数", self.keep_revolutions_var, ["1", "2", "3", "4", "5"]).configure(state="readonly")
        ttk.Checkbutton(scan, text="顺时针旋转", variable=self.clockwise_var, command=self._apply_live_settings).grid(row=4, column=0, columnspan=2, sticky="w", pady=(8, 0))

        actions = ttk.Frame(parent, style="Panel.TFrame")
        actions.pack(fill="x", pady=(18, 0))
        self.scan_button = ttk.Button(actions, text="开始扫描", style="Primary.TButton", command=self.toggle_scan)
        self.scan_button.pack(side="left", fill="x", expand=True)
        self.laser_button = ttk.Button(actions, text="打开激光", command=self.toggle_laser)
        self.laser_button.pack(side="left", padx=(8, 0))
        ttk.Button(parent, text="紧急停止", style="Danger.TButton", command=self.emergency_stop).pack(fill="x", pady=(10, 0))

        command_row = ttk.Frame(parent, style="Panel.TFrame")
        command_row.pack(fill="x", pady=(12, 0))
        ttk.Button(command_row, text="PING", command=lambda: self.send_motor("PING")).pack(side="left", expand=True, fill="x")
        ttk.Button(command_row, text="STATUS", command=lambda: self.send_motor("STATUS")).pack(side="left", expand=True, fill="x", padx=6)
        ttk.Button(command_row, text="清零计数", command=lambda: self.send_motor("RESETCNT")).pack(side="left", expand=True, fill="x")

    def _build_calibration_tab(self, parent) -> None:
        ttk.Label(parent, text="距离标定", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w")
        self.calibration_state_var = tk.StringVar(value="未标定")
        ttk.Label(parent, textvariable=self.calibration_state_var, style="Muted.TLabel", wraplength=330).pack(anchor="w", pady=(4, 12))

        self.cal_tree = ttk.Treeview(parent, columns=("pixel", "distance"), show="headings", height=11)
        self.cal_tree.heading("pixel", text="中心像素")
        self.cal_tree.heading("distance", text="实测距离 / m")
        self.cal_tree.column("pixel", width=110, anchor="center")
        self.cal_tree.column("distance", width=130, anchor="center")
        self.cal_tree.pack(fill="x")

        row = ttk.Frame(parent, style="Panel.TFrame")
        row.pack(fill="x", pady=10)
        ttk.Button(row, text="添加当前点", command=self.add_calibration_point).pack(side="left", expand=True, fill="x")
        ttk.Button(row, text="删除选中", command=self.delete_calibration_point).pack(side="left", expand=True, fill="x", padx=(8, 0))
        ttk.Button(parent, text="拟合并保存", style="Primary.TButton", command=self.fit_calibration).pack(fill="x")
        ttk.Button(parent, text="清空标定", command=self.clear_calibration).pack(fill="x", pady=(8, 0))

        note = (
            "在多个已知距离处保持雷达静止，每个位置等待读数稳定后添加当前点。"
            "建议覆盖实际量程并采集不少于 6 个距离。"
        )
        ttk.Label(parent, text=note, style="Muted.TLabel", wraplength=330, justify="left").pack(anchor="w", pady=(16, 0))
        self._refresh_calibration_view()

    def _build_log_tab(self, parent) -> None:
        ttk.Label(parent, text="运行记录", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w")
        self.log_state_var = tk.StringVar(value="未记录")
        ttk.Label(parent, textvariable=self.log_state_var, style="Muted.TLabel", wraplength=330).pack(anchor="w", pady=(4, 10))
        log_buttons = ttk.Frame(parent, style="Panel.TFrame")
        log_buttons.pack(fill="x", pady=(0, 12))
        self.log_button = ttk.Button(log_buttons, text="开始 CSV 记录", command=self.toggle_logging)
        self.log_button.pack(side="left", expand=True, fill="x")
        ttk.Button(log_buttons, text="打开记录目录", command=self.open_log_directory).pack(side="left", expand=True, fill="x", padx=(8, 0))

        self.console = tk.Text(
            parent,
            height=24,
            background="#07101c",
            foreground=COLORS["muted"],
            insertbackground=COLORS["text"],
            borderwidth=0,
            font=("Cascadia Mono", 9),
            padx=10,
            pady=10,
            state="disabled",
        )
        self.console.pack(fill="both", expand=True)
        ttk.Button(parent, text="清空消息", command=self.clear_console).pack(fill="x", pady=(10, 0))

    def _labeled_entry(self, parent, row: int, label: str, variable: tk.Variable):
        ttk.Label(parent, text=label, style="Muted.TLabel").grid(row=row, column=0, sticky="w", pady=4)
        entry = ttk.Entry(parent, textvariable=variable)
        entry.grid(row=row, column=1, sticky="ew", padx=(12, 0), pady=4)
        entry.bind("<FocusOut>", lambda _: self._apply_live_settings())
        parent.columnconfigure(1, weight=1)
        return entry

    def _labeled_combo(self, parent, row: int, label: str, variable: tk.Variable, values: list[str]):
        ttk.Label(parent, text=label, style="Muted.TLabel").grid(row=row, column=0, sticky="w", pady=4)
        combo = ttk.Combobox(parent, textvariable=variable, values=values)
        combo.grid(row=row, column=1, sticky="ew", padx=(12, 0), pady=4)
        combo.bind("<<ComboboxSelected>>", lambda _: self._apply_live_settings())
        parent.columnconfigure(1, weight=1)
        return combo

    def _load_variables(self) -> None:
        self.measure_port_var.set(str(self.config.get("measurement_port", "")))
        self.motor_port_var.set(str(self.config.get("motor_port", "")))
        self.command_var.set(str(self.config.get("ccd_command", "@c0071#@")))
        parser_mode = str(self.config.get("ccd_parser", "fffe"))
        self.parser_var.set(CCDFrameParser.MODES.get(parser_mode, CCDFrameParser.MODES["fffe"]))
        self.exposure_var.set(int(self.config.get("exposure_index", 8)))
        self.sample_rate_var.set(float(self.config.get("sample_rate_hz", 20.0)))
        self.min_range_var.set(float(self.config.get("min_range_m", 0.08)))
        self.max_range_var.set(float(self.config.get("max_range_m", 3.0)))
        self.angle_offset_var.set(float(self.config.get("angle_offset_deg", 0.0)))
        self.clockwise_var.set(bool(self.config.get("clockwise", True)))
        self.keep_revolutions_var.set(int(self.config.get("keep_revolutions", 3)))

    def _collect_config(self) -> dict:
        parser_label = self.parser_var.get()
        parser_mode = next((key for key, label in CCDFrameParser.MODES.items() if label == parser_label), "fffe")
        result = {
            "measurement_port": self.measure_port_var.get().strip(),
            "motor_port": self.motor_port_var.get().strip(),
            "baudrate": 115200,
            "ccd_command": self.command_var.get().strip() or "@c0071#@",
            "ccd_parser": parser_mode,
            "exposure_index": int(self.exposure_var.get()),
            "sample_rate_hz": float(self.sample_rate_var.get()),
            "min_range_m": float(self.min_range_var.get()),
            "max_range_m": float(self.max_range_var.get()),
            "angle_offset_deg": float(self.angle_offset_var.get()),
            "clockwise": bool(self.clockwise_var.get()),
            "keep_revolutions": int(self.keep_revolutions_var.get()),
            "initial_period_s": self.rotation.period_s,
            "calibration": self.calibration.to_dict(),
        }
        return result

    def _apply_live_settings(self) -> None:
        try:
            max_range = float(self.max_range_var.get())
            min_range = float(self.min_range_var.get())
            sample_rate = float(self.sample_rate_var.get())
            if not (0.2 <= max_range <= 100 and 0.0 <= min_range < max_range and 0.5 <= sample_rate <= 200):
                return
            self.rotation.configure(
                float(self.angle_offset_var.get()),
                bool(self.clockwise_var.get()),
                int(self.keep_revolutions_var.get()),
            )
            if self.demo:
                self.simulator.set_rate_hz(sample_rate)
            parser_label = self.parser_var.get()
            mode = next((key for key, label in CCDFrameParser.MODES.items() if label == parser_label), "fffe")
            if mode != self.ccd_parser.mode:
                self.ccd_parser.mode = mode
                self.ccd_parser.reset()
        except (ValueError, tk.TclError):
            return

    def refresh_ports(self) -> None:
        ports = list_serial_ports()
        self.measure_combo.configure(values=ports)
        self.motor_combo.configure(values=ports)
        if ports:
            if not self.measure_port_var.get():
                self.measure_port_var.set(ports[0])
            if not self.motor_port_var.get():
                self.motor_port_var.set(ports[1] if len(ports) > 1 else ports[0])
        self._log(f"发现串口：{', '.join(ports) if ports else '无'}")

    def toggle_connection(self) -> None:
        if self.connected:
            self.disconnect_devices()
        else:
            self.connect_devices()

    def connect_devices(self) -> None:
        measure_port = self.measure_port_var.get().strip()
        motor_port = self.motor_port_var.get().strip()
        if not measure_port or not motor_port:
            messagebox.showwarning("端口未选择", "请选择测距串口和传动串口。")
            return
        if measure_port == motor_port:
            messagebox.showwarning("端口冲突", "测距和传动必须使用两个不同串口。")
            return
        try:
            self.measure_endpoint.open(measure_port, 115200)
            self.motor_endpoint.open(motor_port, 115200)
        except Exception as exc:
            self.measure_endpoint.close()
            self.motor_endpoint.close()
            messagebox.showerror("连接失败", str(exc))
            self._log(f"连接失败：{exc}")
            return
        self.connected = True
        self.connection_label.configure(text="●  已连接", fg=COLORS["green"])
        self.connect_button.configure(text="断开设备")
        self._log(f"已连接测距 {measure_port} / 传动 {motor_port}")
        self.send_motor("PING")
        self.send_motor("STATUS")

    def disconnect_devices(self) -> None:
        self.stop_scan()
        self.measure_endpoint.close()
        self.motor_endpoint.close()
        self.connected = False
        self.connection_label.configure(text="●  未连接", fg=COLORS["muted"])
        self.connect_button.configure(text="连接设备")
        self._log("设备已断开")

    def toggle_scan(self) -> None:
        if self.scanning:
            self.stop_scan()
        else:
            self.start_scan()

    def start_scan(self) -> None:
        if self.demo:
            self.start_simulation()
            return
        if not self.connected:
            messagebox.showwarning("设备未连接", "请先连接测距串口和传动串口。")
            return
        if not self.calibration.ready:
            messagebox.showwarning("尚未标定", "请先在“标定”页完成距离标定。")
            return
        self._apply_live_settings()
        try:
            exposure = int(self.exposure_var.get())
            sample_rate = float(self.sample_rate_var.get())
            if not 0 <= exposure <= 13:
                raise ValueError("曝光档位应为 0～13")
            if not 0.5 <= sample_rate <= 200:
                raise ValueError("采样频率应为 0.5～200 Hz")
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        self.rotation.reset()
        self.ccd_parser.reset()
        self.motor_parser.reset()
        self.measure_endpoint.write_line(f"@c{exposure:04d}#@")
        self.measure_endpoint.write_line("LASER 1")
        self.laser_on = True
        self.laser_button.configure(text="关闭激光")
        self.send_motor("RESETCNT")
        self.send_motor("ON")
        self.scanning = True
        self.last_request_time = 0.0
        self.scan_button.configure(text="停止扫描")
        self._log(f"扫描启动：曝光 {exposure}，请求频率 {sample_rate:g} Hz")

    def stop_scan(self) -> None:
        if self.demo:
            self.simulator.stop()
        if self.motor_endpoint.is_open:
            self.send_motor("OFF")
        self.scanning = False
        self.scan_button.configure(text="开始扫描")
        self._log("扫描已停止")

    def start_simulation(self) -> None:
        if self.scanning:
            return
        self.rotation.reset()
        self.scanning = True
        self.connected = True
        self.connection_label.configure(text="●  模拟运行", fg=COLORS["cyan"])
        self.connect_button.configure(text="模拟模式", state="disabled")
        self.scan_button.configure(text="停止扫描")
        self.laser_on = True
        self.laser_button.configure(text="关闭激光")
        try:
            self.simulator.set_rate_hz(float(self.sample_rate_var.get()))
        except (ValueError, tk.TclError):
            self.simulator.set_rate_hz(20.0)
        self.simulator.start()
        self._log("模拟雷达已启动")

    def toggle_laser(self) -> None:
        self.laser_on = not self.laser_on
        if not self.demo:
            if not self.measure_endpoint.is_open:
                self.laser_on = False
                messagebox.showwarning("测距串口未连接", "请先连接设备。")
                return
            self.measure_endpoint.write_line("LASER 1" if self.laser_on else "LASER 0")
        self.laser_button.configure(text="关闭激光" if self.laser_on else "打开激光")
        self._log("激光已打开" if self.laser_on else "激光已关闭")

    def emergency_stop(self) -> None:
        self.simulator.stop()
        if self.motor_endpoint.is_open:
            self.motor_endpoint.write_line("OFF")
        if self.measure_endpoint.is_open:
            self.measure_endpoint.write_line("LASER 0")
        self.scanning = False
        self.laser_on = False
        self.scan_button.configure(text="开始扫描")
        self.laser_button.configure(text="打开激光")
        self._log("紧急停止：电机与激光关闭")

    def send_motor(self, command: str) -> None:
        if self.demo:
            self._log(f"TX MOTOR  {command}")
            return
        if self.motor_endpoint.write_line(command):
            self._log(f"TX MOTOR  {command}")

    def _measurement_data(self, data: bytes, timestamp: float) -> None:
        for pixel in self.ccd_parser.feed(data):
            self.events.put(("ccd_pixel", pixel, timestamp))

    def _motor_data(self, data: bytes, timestamp: float) -> None:
        for line in self.motor_parser.feed(data):
            self.events.put(("motor_line", line, timestamp))

    def _serial_error(self, message: str) -> None:
        self.events.put(("error", message, time.perf_counter()))

    def _poll(self) -> None:
        now = time.perf_counter()
        try:
            while True:
                kind, value, timestamp = self.events.get_nowait()
                if kind == "ccd_pixel":
                    self._handle_pixel(int(value), float(timestamp))
                elif kind == "motor_line":
                    self._handle_motor_line(str(value), float(timestamp))
                elif kind == "error":
                    self._log(str(value))
        except queue.Empty:
            pass

        if self.scanning and self.demo:
            try:
                self.simulator.set_rate_hz(float(self.sample_rate_var.get()))
            except (ValueError, tk.TclError):
                pass

        if self.scanning and not self.demo and self.measure_endpoint.is_open:
            try:
                interval = 1.0 / max(0.5, float(self.sample_rate_var.get()))
            except (ValueError, tk.TclError):
                interval = 0.05
            if now - self.last_request_time >= interval:
                self.measure_endpoint.write_line(self.command_var.get().strip() or "@c0071#@")
                self.last_request_time = now

        self.root.after(15, self._poll)

    def _handle_pixel(self, pixel: int, timestamp: float) -> None:
        self.latest_pixel = pixel
        distance = self.calibration.distance(pixel)
        if distance is None:
            return
        try:
            minimum = float(self.min_range_var.get())
            maximum = float(self.max_range_var.get())
        except (ValueError, tk.TclError):
            minimum, maximum = 0.08, 3.0
        if not minimum <= distance <= maximum * 1.5:
            return
        self.latest_distance = distance
        point = self.rotation.add_sample(distance, pixel, timestamp)
        if point is not None:
            self.latest_angle_deg = math.degrees(point.angle_rad) % 360
        self.sample_rate.add(timestamp)
        if self.log_writer:
            angle = self.latest_angle_deg if self.latest_angle_deg is not None else ""
            x = point.x if point else ""
            y = point.y if point else ""
            self.log_writer.writerow([datetime.now().isoformat(timespec="milliseconds"), f"{timestamp:.6f}", pixel, f"{distance:.6f}", angle, x, y, self.rotation.trigger_count])
            self.log_file.flush()
            self.log_rows += 1
            self.log_state_var.set(f"记录中 · {self.log_rows} 行\n{self.log_path}")

    def _handle_motor_line(self, line: str, timestamp: float) -> None:
        self.last_motor_message = line
        self._log(f"RX MOTOR  {line}")
        match = re.match(r"TRIG(?:\s+(\d+))?", line, re.I)
        if match:
            count = int(match.group(1)) if match.group(1) else None
            self.rotation.trigger(timestamp, count)

    def _draw(self) -> None:
        points = [(point.x, point.y, alpha) for point, alpha in self.rotation.all_points()]
        waiting = "" if self.rotation.last_trigger is not None else ("等待光电零位" if self.scanning else "")
        try:
            max_range = float(self.max_range_var.get())
        except (ValueError, tk.TclError):
            max_range = 3.0
        heading = self.latest_angle_deg or 0.0
        self.radar_canvas.update_scene(points, max_range, heading, waiting)
        self.metric_vars["distance"].set("—" if self.latest_distance is None else f"{self.latest_distance:.2f}")
        self.metric_vars["angle"].set("—" if self.latest_angle_deg is None else f"{self.latest_angle_deg:.1f}")
        self.metric_vars["rpm"].set(f"{self.rotation.rpm:.1f}" if self.rotation.period_history else "—")
        self.metric_vars["rate"].set(f"{self.sample_rate.value():.1f}")
        self.root.after(80, self._draw)

    def add_calibration_point(self) -> None:
        if self.latest_pixel is None:
            messagebox.showwarning("没有测量值", "请先连接测距模块并取得稳定的中心像素。")
            return
        value = simpledialog.askfloat("添加标定点", f"当前中心像素：{self.latest_pixel}\n请输入目标实际距离（m）：", minvalue=0.02, maxvalue=100, parent=self.root)
        if value is None:
            return
        try:
            self.calibration.add_point(self.latest_pixel, value)
        except ValueError as exc:
            messagebox.showerror("标定点无效", str(exc))
            return
        self._refresh_calibration_view()
        self._log(f"添加标定点：pixel={self.latest_pixel}, distance={value:.4f} m")

    def delete_calibration_point(self) -> None:
        selection = self.cal_tree.selection()
        if not selection:
            return
        indices = sorted((int(self.cal_tree.item(item, "tags")[0]) for item in selection), reverse=True)
        for index in indices:
            if 0 <= index < len(self.calibration.points):
                del self.calibration.points[index]
        self.calibration.p0 = None
        self.calibration.k = None
        self.calibration.rmse = None
        self._refresh_calibration_view()

    def fit_calibration(self) -> None:
        try:
            p0, k, rmse = self.calibration.fit()
        except ValueError as exc:
            messagebox.showerror("无法拟合", str(exc))
            return
        self._refresh_calibration_view()
        self.config = self._collect_config()
        save_config(self.config)
        self._log(f"标定完成：p0={p0:.4f}, k={k:.4f}, RMSE={rmse:.3f} px")
        messagebox.showinfo("标定完成", f"p₀ = {p0:.3f}\nk = {k:.3f}\n像素 RMSE = {rmse:.3f}")

    def clear_calibration(self) -> None:
        if not messagebox.askyesno("清空标定", "确定删除全部距离标定点和拟合参数吗？"):
            return
        self.calibration.clear()
        self._refresh_calibration_view()

    def _refresh_calibration_view(self) -> None:
        for item in self.cal_tree.get_children():
            self.cal_tree.delete(item)
        for index, (pixel, distance) in enumerate(self.calibration.points):
            self.cal_tree.insert("", "end", values=(f"{pixel:.1f}", f"{distance:.4f}"), tags=(str(index),))
        if self.calibration.ready:
            rmse = "—" if self.calibration.rmse is None else f"{self.calibration.rmse:.3f} px"
            self.calibration_state_var.set(f"已标定 · p₀={self.calibration.p0:.3f} · k={self.calibration.k:.3f} · RMSE={rmse}")
        else:
            self.calibration_state_var.set(f"未标定 · 已采集 {len(self.calibration.points)} 个点")

    def toggle_logging(self) -> None:
        if self.log_file:
            self.stop_logging()
        else:
            self.start_logging()

    def start_logging(self) -> None:
        logs_dir = APP_DIR / "logs"
        logs_dir.mkdir(exist_ok=True)
        default = logs_dir / f"radar_{datetime.now():%Y%m%d_%H%M%S}.csv"
        selected = filedialog.asksaveasfilename(
            title="保存雷达记录",
            initialdir=logs_dir,
            initialfile=default.name,
            defaultextension=".csv",
            filetypes=[("CSV 文件", "*.csv")],
        )
        if not selected:
            return
        try:
            self.log_path = Path(selected)
            self.log_file = self.log_path.open("w", newline="", encoding="utf-8-sig")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(["local_time", "monotonic_time", "pixel", "distance_m", "angle_deg", "x_m", "y_m", "trigger_count"])
            self.log_rows = 0
        except OSError as exc:
            self.log_file = None
            self.log_writer = None
            messagebox.showerror("无法创建记录", str(exc))
            return
        self.log_button.configure(text="停止记录")
        self.log_state_var.set(f"记录中 · 0 行\n{self.log_path}")
        self._log(f"开始记录：{self.log_path}")

    def stop_logging(self) -> None:
        if self.log_file:
            try:
                self.log_file.close()
            except OSError:
                pass
        path = self.log_path
        self.log_file = None
        self.log_writer = None
        self.log_button.configure(text="开始 CSV 记录")
        self.log_state_var.set(f"已保存 · {self.log_rows} 行\n{path}" if path else "未记录")
        self._log(f"记录已保存：{path}")

    def open_log_directory(self) -> None:
        logs_dir = APP_DIR / "logs"
        logs_dir.mkdir(exist_ok=True)
        try:
            import os

            os.startfile(logs_dir)
        except (AttributeError, OSError) as exc:
            messagebox.showerror("无法打开目录", str(exc))

    def _log(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        line = f"{stamp}  {message}"
        self.status_history.append(line)
        if hasattr(self, "console"):
            self.console.configure(state="normal")
            self.console.insert("end", line + "\n")
            self.console.see("end")
            self.console.configure(state="disabled")

    def clear_console(self) -> None:
        self.status_history.clear()
        self.console.configure(state="normal")
        self.console.delete("1.0", "end")
        self.console.configure(state="disabled")

    def on_close(self) -> None:
        self.emergency_stop()
        self.stop_logging()
        self.measure_endpoint.close()
        self.motor_endpoint.close()
        if not self.demo:
            try:
                self.config = self._collect_config()
                save_config(self.config)
            except (OSError, ValueError, tk.TclError):
                pass
        self.root.destroy()


def capture_window(root: tk.Tk, output_path: Path, delay_ms: int = 1800) -> None:
    def grab() -> None:
        root.deiconify()
        root.lift()
        root.attributes("-topmost", True)
        root.focus_force()
        root.update_idletasks()
        root.update()
        try:
            from PIL import ImageGrab

            x = root.winfo_rootx()
            y = root.winfo_rooty()
            width = root.winfo_width()
            height = root.winfo_height()
            image = ImageGrab.grab(bbox=(x, y, x + width, y + height), all_screens=True)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            image.save(output_path)
        finally:
            root.attributes("-topmost", False)
            root.after(100, root.destroy)

    root.after(delay_ms, grab)


def main() -> None:
    parser = argparse.ArgumentParser(description=APP_NAME)
    parser.add_argument("--demo", action="store_true", help="启动内置模拟雷达")
    parser.add_argument("--screenshot", type=Path, help="保存窗口截图后退出")
    parser.add_argument("--screenshot-delay", type=int, default=1800)
    args = parser.parse_args()

    root = tk.Tk()
    RadarApp(root, demo=args.demo)
    if args.screenshot:
        capture_window(root, args.screenshot.resolve(), args.screenshot_delay)
    root.mainloop()


if __name__ == "__main__":
    main()
