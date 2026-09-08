from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from app_utils import capture_window, enable_windows_dpi_awareness, save_json

enable_windows_dpi_awareness()


APP_DIR = Path(__file__).resolve().parent
DEFAULT_MAP_PATH = APP_DIR / "simulation_map.json"

COLORS = {
    "background": "#07111f",
    "panel": "#101f31",
    "panel_alt": "#0b1929",
    "border": "#29445f",
    "grid": "#193149",
    "axis": "#315777",
    "text": "#eef7ff",
    "muted": "#82a3c1",
    "accent": "#22e6b1",
    "accent_dark": "#127e65",
    "wall": "#d9e7f2",
    "start": "#23d3ff",
    "finish": "#ffc857",
    "danger": "#ff6b78",
}


class MapEditor:
    def __init__(self, root: tk.Tk, map_path: Path) -> None:
        self.root = root
        self.map_path = map_path
        self.root.title("TriScan 模拟地图编辑器")
        self.root.configure(bg=COLORS["background"])
        self.root.geometry("1220x820")
        self.root.minsize(980, 660)

        self.min_x = -3.0
        self.max_x = 3.0
        self.min_y = -3.0
        self.max_y = 3.0
        self.wall_thickness_m = 0.06
        self.start = (0.0, -2.5)
        self.start_yaw_deg = 0.0
        self.finish = (0.0, 2.5)
        self.obstacles: list[tuple[float, float, float, float]] = []
        self.history: list[tuple[str, object]] = []
        self.drag_start: tuple[float, float] | None = None
        self.drag_current: tuple[float, float] | None = None
        self.mode = tk.StringVar(value="wall")
        self.status = tk.StringVar(value="在画布上拖动鼠标绘制直线障碍")
        self.cursor_text = tk.StringVar(value="x +0.00 m   y +0.00 m")
        self.file_text = tk.StringVar(value=str(self.map_path))
        self.width_var = tk.StringVar(value="6.0")
        self.height_var = tk.StringVar(value="6.0")
        self.thickness_var = tk.StringVar(value="0.06")
        self.mode_buttons: dict[str, tk.Button] = {}
        self._build_ui()
        if self.map_path.exists():
            self.load_map(self.map_path, quiet=True)
        self.root.after_idle(self.redraw)

    def _build_ui(self) -> None:
        top = tk.Frame(self.root, bg=COLORS["background"], height=72)
        top.pack(fill="x", padx=18, pady=(12, 8))
        top.pack_propagate(False)
        tk.Label(
            top,
            text="TriScan",
            fg=COLORS["text"],
            bg=COLORS["background"],
            font=("Segoe UI", 23, "bold"),
        ).pack(side="left", padx=(2, 14))
        tk.Label(
            top,
            text="模拟地图编辑器",
            fg=COLORS["muted"],
            bg=COLORS["background"],
            font=("Microsoft YaHei UI", 11),
        ).pack(side="left", pady=(12, 0))
        tk.Button(
            top,
            text="保存地图",
            command=self.save_map,
            bg=COLORS["accent_dark"],
            fg="white",
            activebackground=COLORS["accent"],
            activeforeground=COLORS["background"],
            relief="flat",
            font=("Microsoft YaHei UI", 11, "bold"),
            padx=24,
            pady=9,
            cursor="hand2",
        ).pack(side="right", pady=8)
        tk.Button(
            top,
            text="另存为",
            command=self.save_as,
            bg=COLORS["panel"],
            fg=COLORS["text"],
            activebackground=COLORS["border"],
            activeforeground="white",
            relief="flat",
            font=("Microsoft YaHei UI", 10),
            padx=18,
            pady=9,
            cursor="hand2",
        ).pack(side="right", padx=10, pady=8)

        body = tk.Frame(self.root, bg=COLORS["background"])
        body.pack(fill="both", expand=True, padx=18, pady=(0, 14))

        sidebar = tk.Frame(body, bg=COLORS["panel"], width=245)
        sidebar.pack(side="left", fill="y", padx=(0, 10))
        sidebar.pack_propagate(False)
        self._section_label(sidebar, "绘制工具").pack(anchor="w", padx=16, pady=(14, 7))
        self._mode_button(sidebar, "wall", "直线障碍", "拖动画线").pack(fill="x", padx=14, pady=2)
        self._mode_button(sidebar, "start", "设置起点", "拖动设置车头方向").pack(fill="x", padx=14, pady=2)
        self._mode_button(sidebar, "finish", "设置终点", "单击放置").pack(fill="x", padx=14, pady=2)

        self._separator(sidebar).pack(fill="x", padx=14, pady=10)
        self._section_label(sidebar, "场地参数").pack(anchor="w", padx=16, pady=(0, 6))
        self._entry_row(sidebar, "宽度 / m", self.width_var)
        self._entry_row(sidebar, "高度 / m", self.height_var)
        self._entry_row(sidebar, "障碍线宽 / m", self.thickness_var)
        self._plain_button(sidebar, "应用参数", self.apply_dimensions).pack(fill="x", padx=14, pady=(5, 2))

        self._separator(sidebar).pack(fill="x", padx=14, pady=10)
        self._plain_button(sidebar, "撤销上一步", self.undo).pack(fill="x", padx=14, pady=2)
        self._plain_button(sidebar, "清空障碍", self.clear_obstacles).pack(fill="x", padx=14, pady=2)
        self._plain_button(sidebar, "打开地图", self.choose_map).pack(fill="x", padx=14, pady=2)
        self._plain_button(sidebar, "新建地图", self.new_map).pack(fill="x", padx=14, pady=2)

        canvas_panel = tk.Frame(body, bg=COLORS["panel_alt"], highlightbackground=COLORS["border"], highlightthickness=1)
        canvas_panel.pack(side="left", fill="both", expand=True)
        canvas_top = tk.Frame(canvas_panel, bg=COLORS["panel"])
        canvas_top.pack(fill="x")
        tk.Label(
            canvas_top,
            textvariable=self.status,
            fg=COLORS["accent"],
            bg=COLORS["panel"],
            font=("Microsoft YaHei UI", 10),
        ).pack(side="left", padx=14, pady=10)
        tk.Label(
            canvas_top,
            textvariable=self.cursor_text,
            fg=COLORS["muted"],
            bg=COLORS["panel"],
            font=("Consolas", 10),
        ).pack(side="right", padx=14)
        self.canvas = tk.Canvas(canvas_panel, bg=COLORS["background"], highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Motion>", self.on_motion)
        self.canvas.bind("<Button-3>", self.cancel_drag)
        self.root.bind("<Control-z>", lambda _event: self.undo())
        self.root.bind("<Control-s>", lambda _event: self.save_map())
        tk.Label(
            canvas_panel,
            textvariable=self.file_text,
            anchor="w",
            fg=COLORS["muted"],
            bg=COLORS["panel"],
            font=("Microsoft YaHei UI", 9),
            padx=14,
            pady=8,
        ).pack(fill="x")
        self._refresh_mode_buttons()

    def _section_label(self, parent: tk.Widget, text: str) -> tk.Label:
        return tk.Label(parent, text=text, fg=COLORS["text"], bg=COLORS["panel"], font=("Microsoft YaHei UI", 11, "bold"))

    def _separator(self, parent: tk.Widget) -> ttk.Separator:
        return ttk.Separator(parent, orient="horizontal")

    def _mode_button(self, parent: tk.Widget, mode: str, title: str, subtitle: str) -> tk.Button:
        button = tk.Button(
            parent,
            text=f"{title}   {subtitle}",
            command=lambda: self.set_mode(mode),
            anchor="w",
            relief="flat",
            font=("Microsoft YaHei UI", 9),
            padx=12,
            pady=6,
            cursor="hand2",
        )
        self.mode_buttons[mode] = button
        return button

    def _plain_button(self, parent: tk.Widget, text: str, command) -> tk.Button:
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg=COLORS["panel_alt"],
            fg=COLORS["text"],
            activebackground=COLORS["border"],
            activeforeground="white",
            relief="flat",
            font=("Microsoft YaHei UI", 9),
            pady=5,
            cursor="hand2",
        )

    def _entry_row(self, parent: tk.Widget, label: str, variable: tk.StringVar) -> None:
        row = tk.Frame(parent, bg=COLORS["panel"])
        row.pack(fill="x", padx=14, pady=3)
        tk.Label(row, text=label, width=12, anchor="w", fg=COLORS["muted"], bg=COLORS["panel"], font=("Microsoft YaHei UI", 9)).pack(side="left")
        tk.Entry(
            row,
            textvariable=variable,
            bg=COLORS["background"],
            fg=COLORS["text"],
            insertbackground=COLORS["text"],
            relief="flat",
            justify="right",
            font=("Consolas", 10),
        ).pack(side="right", fill="x", expand=True, ipady=3)

    def set_mode(self, mode: str) -> None:
        self.cancel_drag()
        self.mode.set(mode)
        messages = {
            "wall": "在画布上拖动鼠标绘制直线障碍",
            "start": "从起点位置向车头方向拖动",
            "finish": "在可通行区域单击放置终点",
        }
        self.status.set(messages[mode])
        self._refresh_mode_buttons()

    def _refresh_mode_buttons(self) -> None:
        active = self.mode.get()
        for mode, button in self.mode_buttons.items():
            if mode == active:
                button.configure(bg=COLORS["accent_dark"], fg="white", activebackground=COLORS["accent_dark"])
            else:
                button.configure(bg=COLORS["panel_alt"], fg=COLORS["text"], activebackground=COLORS["border"])

    def _scale(self) -> tuple[float, float, float]:
        canvas_width = max(100, self.canvas.winfo_width())
        canvas_height = max(100, self.canvas.winfo_height())
        margin = 48.0
        scale = min(
            (canvas_width - 2 * margin) / (self.max_x - self.min_x),
            (canvas_height - 2 * margin) / (self.max_y - self.min_y),
        )
        origin_x = (canvas_width - (self.max_x - self.min_x) * scale) * 0.5 - self.min_x * scale
        origin_y = (canvas_height - (self.max_y - self.min_y) * scale) * 0.5 + self.max_y * scale
        return scale, origin_x, origin_y

    def world_to_canvas(self, x: float, y: float) -> tuple[float, float]:
        scale, origin_x, origin_y = self._scale()
        return origin_x + x * scale, origin_y - y * scale

    def canvas_to_world(self, canvas_x: float, canvas_y: float, snap: bool = True) -> tuple[float, float]:
        scale, origin_x, origin_y = self._scale()
        x = (canvas_x - origin_x) / scale
        y = (origin_y - canvas_y) / scale
        x = min(self.max_x, max(self.min_x, x))
        y = min(self.max_y, max(self.min_y, y))
        if snap:
            snap_size = 0.05
            x = round(x / snap_size) * snap_size
            y = round(y / snap_size) * snap_size
        return x, y

    def redraw(self) -> None:
        if not hasattr(self, "canvas"):
            return
        self.canvas.delete("all")
        scale, _, _ = self._scale()
        grid_step = 0.5
        start_x = math.ceil(self.min_x / grid_step) * grid_step
        start_y = math.ceil(self.min_y / grid_step) * grid_step
        x = start_x
        while x <= self.max_x + 1e-9:
            x1, y1 = self.world_to_canvas(x, self.min_y)
            x2, y2 = self.world_to_canvas(x, self.max_y)
            color = COLORS["axis"] if abs(x) < 1e-9 else COLORS["grid"]
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=1)
            if abs(x) > 1e-9:
                self.canvas.create_text(x1 + 3, y1 + 17, text=f"{x:g}", fill=COLORS["muted"], anchor="nw", font=("Consolas", 8))
            x += grid_step
        y = start_y
        while y <= self.max_y + 1e-9:
            x1, y1 = self.world_to_canvas(self.min_x, y)
            x2, y2 = self.world_to_canvas(self.max_x, y)
            color = COLORS["axis"] if abs(y) < 1e-9 else COLORS["grid"]
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=1)
            if abs(y) > 1e-9:
                self.canvas.create_text(x1 - 8, y1, text=f"{y:g}", fill=COLORS["muted"], anchor="e", font=("Consolas", 8))
            y += grid_step

        left, bottom = self.world_to_canvas(self.min_x, self.min_y)
        right, top = self.world_to_canvas(self.max_x, self.max_y)
        self.canvas.create_rectangle(left, top, right, bottom, outline=COLORS["wall"], width=3)

        line_width = max(2, round(self.wall_thickness_m * scale))
        for x1, y1, x2, y2 in self.obstacles:
            c1 = self.world_to_canvas(x1, y1)
            c2 = self.world_to_canvas(x2, y2)
            self.canvas.create_line(*c1, *c2, fill=COLORS["wall"], width=line_width, capstyle="round")

        self._draw_finish()
        self._draw_robot()
        if self.drag_start is not None and self.drag_current is not None:
            c1 = self.world_to_canvas(*self.drag_start)
            c2 = self.world_to_canvas(*self.drag_current)
            color = COLORS["wall"] if self.mode.get() == "wall" else COLORS["start"]
            self.canvas.create_line(*c1, *c2, fill=color, width=max(2, line_width), dash=(7, 5), arrow="last" if self.mode.get() == "start" else "none")

        self.canvas.create_text(
            right,
            top - 12,
            text=f"{self.max_x - self.min_x:.1f} m × {self.max_y - self.min_y:.1f} m    障碍 {len(self.obstacles)} 条",
            fill=COLORS["muted"],
            anchor="se",
            font=("Microsoft YaHei UI", 9),
        )

    def _draw_robot(self) -> None:
        x, y = self.start
        yaw = math.radians(self.start_yaw_deg)
        corners = [(-0.10, -0.10), (0.10, -0.10), (0.10, 0.10), (-0.10, 0.10)]
        polygon: list[float] = []
        for local_x, local_y in corners:
            world_x = x + local_x * math.cos(yaw) + local_y * math.sin(yaw)
            world_y = y - local_x * math.sin(yaw) + local_y * math.cos(yaw)
            polygon.extend(self.world_to_canvas(world_x, world_y))
        self.canvas.create_polygon(*polygon, fill="#0d6f89", outline=COLORS["start"], width=2)
        tip = (x + math.sin(yaw) * 0.28, y + math.cos(yaw) * 0.28)
        self.canvas.create_line(*self.world_to_canvas(x, y), *self.world_to_canvas(*tip), fill=COLORS["start"], width=3, arrow="last")
        label = self.world_to_canvas(x, y)
        self.canvas.create_text(label[0] + 12, label[1] + 15, text=f"起点 {self.start_yaw_deg:.0f}°", fill=COLORS["start"], anchor="nw", font=("Microsoft YaHei UI", 9, "bold"))

    def _draw_finish(self) -> None:
        x, y = self.world_to_canvas(*self.finish)
        radius = 9
        self.canvas.create_oval(x - radius, y - radius, x + radius, y + radius, outline=COLORS["finish"], width=2)
        self.canvas.create_line(x - 5, y, x + 5, y, fill=COLORS["finish"], width=2)
        self.canvas.create_line(x, y - 5, x, y + 5, fill=COLORS["finish"], width=2)
        self.canvas.create_text(x + 13, y, text="终点", fill=COLORS["finish"], anchor="w", font=("Microsoft YaHei UI", 9, "bold"))

    def on_press(self, event: tk.Event) -> None:
        self.drag_start = self.canvas_to_world(event.x, event.y)
        self.drag_current = self.drag_start
        self.redraw()

    def on_drag(self, event: tk.Event) -> None:
        if self.drag_start is None:
            return
        self.drag_current = self.canvas_to_world(event.x, event.y)
        self._set_cursor(*self.drag_current)
        self.redraw()

    def on_release(self, event: tk.Event) -> None:
        if self.drag_start is None:
            return
        end = self.canvas_to_world(event.x, event.y)
        start = self.drag_start
        mode = self.mode.get()
        if mode == "wall":
            if math.hypot(end[0] - start[0], end[1] - start[1]) >= 0.02:
                self.obstacles.append((start[0], start[1], end[0], end[1]))
                self.history.append(("wall", None))
                self.status.set(f"已添加第 {len(self.obstacles)} 条障碍")
        elif mode == "start":
            previous = (self.start, self.start_yaw_deg)
            self.start = start
            delta_x = end[0] - start[0]
            delta_y = end[1] - start[1]
            if math.hypot(delta_x, delta_y) >= 0.05:
                self.start_yaw_deg = math.degrees(math.atan2(delta_x, delta_y)) % 360.0
            self.history.append(("start", previous))
            self.status.set(f"起点 ({start[0]:+.2f}, {start[1]:+.2f})  车头 {self.start_yaw_deg:.1f}°")
        elif mode == "finish":
            previous = self.finish
            self.finish = end
            self.history.append(("finish", previous))
            self.status.set(f"终点 ({end[0]:+.2f}, {end[1]:+.2f})")
        self.drag_start = None
        self.drag_current = None
        self.redraw()

    def on_motion(self, event: tk.Event) -> None:
        self._set_cursor(*self.canvas_to_world(event.x, event.y, snap=False))

    def _set_cursor(self, x: float, y: float) -> None:
        self.cursor_text.set(f"x {x:+.2f} m   y {y:+.2f} m")

    def cancel_drag(self, _event: tk.Event | None = None) -> None:
        self.drag_start = None
        self.drag_current = None
        if hasattr(self, "canvas"):
            self.redraw()

    def undo(self) -> None:
        self.cancel_drag()
        if not self.history:
            self.status.set("没有可撤销的操作")
            return
        kind, value = self.history.pop()
        if kind == "wall" and self.obstacles:
            self.obstacles.pop()
        elif kind == "start" and isinstance(value, tuple):
            self.start, self.start_yaw_deg = value
        elif kind == "finish" and isinstance(value, tuple):
            self.finish = value
        elif kind == "clear" and isinstance(value, list):
            self.obstacles = value
        self.status.set("已撤销上一步")
        self.redraw()

    def clear_obstacles(self) -> None:
        if not self.obstacles:
            return
        self.history.append(("clear", list(self.obstacles)))
        self.obstacles.clear()
        self.status.set("障碍已清空")
        self.redraw()

    def apply_dimensions(self) -> None:
        try:
            width = float(self.width_var.get())
            height = float(self.height_var.get())
            thickness = float(self.thickness_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "宽度、高度和障碍线宽必须是数字。", parent=self.root)
            return
        if not 0.5 <= width <= 20 or not 0.5 <= height <= 20:
            messagebox.showerror("参数错误", "场地宽度和高度必须在 0.5 m 到 20 m 之间。", parent=self.root)
            return
        if not 0.01 <= thickness <= 0.30:
            messagebox.showerror("参数错误", "障碍线宽必须在 0.01 m 到 0.30 m 之间。", parent=self.root)
            return
        self.min_x, self.max_x = -width * 0.5, width * 0.5
        self.min_y, self.max_y = -height * 0.5, height * 0.5
        self.wall_thickness_m = thickness
        self.status.set(f"场地尺寸 {width:.2f} m × {height:.2f} m")
        self.redraw()

    def _map_data(self) -> dict:
        return {
            "version": 1,
            "bounds": {
                "min_x": round(self.min_x, 4),
                "max_x": round(self.max_x, 4),
                "min_y": round(self.min_y, 4),
                "max_y": round(self.max_y, 4),
            },
            "wall_thickness_m": round(self.wall_thickness_m, 4),
            "start": {
                "x": round(self.start[0], 4),
                "y": round(self.start[1], 4),
                "yaw_deg": round(self.start_yaw_deg, 3),
            },
            "finish": {"x": round(self.finish[0], 4), "y": round(self.finish[1], 4)},
            "obstacles": [
                {"x1": round(x1, 4), "y1": round(y1, 4), "x2": round(x2, 4), "y2": round(y2, 4)}
                for x1, y1, x2, y2 in self.obstacles
            ],
        }

    @staticmethod
    def _point_segment_distance(point: tuple[float, float], segment: tuple[float, float, float, float]) -> float:
        x, y = point
        x1, y1, x2, y2 = segment
        delta_x, delta_y = x2 - x1, y2 - y1
        length_squared = delta_x * delta_x + delta_y * delta_y
        if length_squared <= 1e-12:
            return math.hypot(x - x1, y - y1)
        fraction = max(0.0, min(1.0, ((x - x1) * delta_x + (y - y1) * delta_y) / length_squared))
        return math.hypot(x - (x1 + fraction * delta_x), y - (y1 + fraction * delta_y))

    def _validate_positions(self) -> bool:
        clearance = 0.15 + self.wall_thickness_m * 0.5
        for name, point in (("起点", self.start), ("终点", self.finish)):
            x, y = point
            boundary_clearance = min(x - self.min_x, self.max_x - x, y - self.min_y, self.max_y - y)
            if boundary_clearance < clearance:
                messagebox.showerror("无法保存", f"{name}距离场地边界太近，请至少留出 {clearance:.2f} m。", parent=self.root)
                return False
            if any(self._point_segment_distance(point, segment) < clearance for segment in self.obstacles):
                messagebox.showerror("无法保存", f"{name}与障碍距离太近，请至少留出 {clearance:.2f} m。", parent=self.root)
                return False
        return True

    def save_map(self) -> None:
        if not self._validate_positions():
            return
        try:
            save_json(self.map_path, self._map_data(), trailing_newline=True)
        except OSError as exc:
            messagebox.showerror("保存失败", str(exc), parent=self.root)
            return
        self.file_text.set(str(self.map_path))
        self.status.set(f"地图已保存：{self.map_path.name}")

    def save_as(self) -> None:
        selected = filedialog.asksaveasfilename(
            parent=self.root,
            title="保存模拟地图",
            initialdir=str(self.map_path.parent),
            initialfile=self.map_path.name,
            defaultextension=".json",
            filetypes=[("JSON 地图", "*.json")],
        )
        if selected:
            self.map_path = Path(selected)
            self.save_map()

    def choose_map(self) -> None:
        selected = filedialog.askopenfilename(
            parent=self.root,
            title="打开模拟地图",
            initialdir=str(self.map_path.parent),
            filetypes=[("JSON 地图", "*.json"), ("所有文件", "*.*")],
        )
        if selected:
            self.load_map(Path(selected))

    def load_map(self, path: Path, quiet: bool = False) -> None:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            bounds = data["bounds"]
            start = data["start"]
            finish = data["finish"]
            obstacles = data["obstacles"]
            self.min_x = float(bounds["min_x"])
            self.max_x = float(bounds["max_x"])
            self.min_y = float(bounds["min_y"])
            self.max_y = float(bounds["max_y"])
            self.wall_thickness_m = float(data.get("wall_thickness_m", 0.06))
            self.start = (float(start["x"]), float(start["y"]))
            self.start_yaw_deg = float(start.get("yaw_deg", 0.0))
            self.finish = (float(finish["x"]), float(finish["y"]))
            self.obstacles = [
                (float(item["x1"]), float(item["y1"]), float(item["x2"]), float(item["y2"]))
                for item in obstacles
            ]
            if self.max_x <= self.min_x or self.max_y <= self.min_y:
                raise ValueError("场地边界无效")
        except (OSError, ValueError, TypeError, KeyError) as exc:
            if not quiet:
                messagebox.showerror("地图无法打开", str(exc), parent=self.root)
            return
        self.map_path = path
        self.width_var.set(f"{self.max_x - self.min_x:g}")
        self.height_var.set(f"{self.max_y - self.min_y:g}")
        self.thickness_var.set(f"{self.wall_thickness_m:g}")
        self.file_text.set(str(path))
        self.history.clear()
        self.status.set(f"已打开：{path.name}")
        self.redraw()

    def new_map(self) -> None:
        self.min_x, self.max_x = -3.0, 3.0
        self.min_y, self.max_y = -3.0, 3.0
        self.wall_thickness_m = 0.06
        self.start = (0.0, -2.5)
        self.start_yaw_deg = 0.0
        self.finish = (0.0, 2.5)
        self.obstacles.clear()
        self.history.clear()
        self.map_path = DEFAULT_MAP_PATH
        self.width_var.set("6.0")
        self.height_var.set("6.0")
        self.thickness_var.set("0.06")
        self.file_text.set(str(self.map_path))
        self.status.set("已新建空白地图")
        self.redraw()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="TriScan 模拟地图编辑器")
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP_PATH)
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--screenshot-delay", type=int, default=1200)
    arguments = parser.parse_args(argv)
    root = tk.Tk()
    MapEditor(root, arguments.map.resolve())
    if arguments.screenshot:
        capture_window(root, arguments.screenshot.resolve(), arguments.screenshot_delay)
    root.mainloop()


if __name__ == "__main__":
    main()
