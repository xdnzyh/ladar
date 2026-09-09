"""Windows desktop application; run on the PC, never upload to ESP32."""
from collections import deque
from datetime import datetime
from pathlib import Path
import queue
import statistics
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from core import Calibration, parse_reference, write_records
from transport import EXPOSURE_INDEX, SerialWorker, SyncWorker

BG = "#f1f5f9"
INK = "#172b4d"
BLUE = "#2563eb"
MUTED = "#64748b"


class DistanceApp:
    def __init__(self, root):
        self.root = root
        root.title("CCD 测距工作台 1.2 · 连接诊断")
        root.geometry("1160x850")
        root.minsize(960, 760)
        root.configure(bg=BG)
        root.option_add("*Font", ("Microsoft YaHei UI", 10))
        self.calibration = Calibration()
        self.records = []
        self.chart_start = 0
        self.window = deque(maxlen=20)
        self.worker = None
        self.events = queue.Queue()
        self.ready = False
        self.pending = False
        self.laser = False
        self.continuous = False
        self.sync_active = False
        self.next_due = 0.0
        self.failures = 0
        self.closing = False
        self.source = ""
        self.log_history = deque(maxlen=200)
        self.sync_angles = {}
        self.port_name = ""
        self.demo = tk.BooleanVar(value=False)
        self.sync_mode = tk.BooleanVar(value=False)
        self.port_text = tk.StringVar()
        self.reference = tk.StringVar()
        self.interval = tk.StringVar(value="500")
        self.sync_rate = tk.StringVar(value="10")
        self.chart_axis = tk.StringVar(value="距离 / cm")
        self.status = tk.StringVar(value="未连接 · 先关闭占用同一串口的 Thonny 或串口助手")
        self.distance_text = tk.StringVar(value="—")
        self.coordinate_text = tk.StringVar(value="—")
        self.error_text = tk.StringVar(value="—")
        self.range_text = tk.StringVar(value="尚无测量")
        self.stats_text = tk.StringVar(value="最近 20 次坐标：暂无数据")
        self.counter = tk.StringVar(value="0 条记录")
        self.cal_text = tk.StringVar()
        self._style()
        self._build()
        self._cal_label()
        self.refresh_ports()
        self._controls()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, self._tick)

    def _style(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG, foreground=INK)
        style.configure("TButton", padding=(12, 7))
        style.configure("Primary.TButton", background=BLUE, foreground="white")
        style.map("Primary.TButton", background=[("active", "#1d4ed8"), ("disabled", "#cbd5e1")])
        style.configure("TCheckbutton", background=BG)
        style.configure("Treeview", rowheight=27, font=("Microsoft YaHei UI", 9))
        style.configure("Treeview.Heading", font=("Microsoft YaHei UI", 9, "bold"))

    def _build(self):
        outer = ttk.Frame(self.root, padding=20)
        outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer)
        header.pack(fill="x")
        ttk.Label(header, text="CCD 测距工作台", font=("Microsoft YaHei UI", 23, "bold")).pack(side="left")
        ttk.Button(header, text="查看 / 复制通信日志", command=self.show_log).pack(side="right")
        ttk.Label(outer, text="原始坐标 → 标定换算 → 与尺量距离对照   /   曝光档位 5", foreground=MUTED).pack(anchor="w", pady=(4, 12))

        connection = ttk.Frame(outer)
        connection.pack(fill="x")
        ttk.Label(connection, text="串口").pack(side="left")
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_text, width=39, state="readonly")
        self.port_combo.pack(side="left", padx=8)
        self.refresh_button = ttk.Button(connection, text="刷新", command=self.refresh_ports)
        self.refresh_button.pack(side="left")
        self.connect_button = ttk.Button(connection, text="连接", command=self.connect, style="Primary.TButton")
        self.connect_button.pack(side="left", padx=8)
        self.disconnect_button = ttk.Button(connection, text="断开", command=self.disconnect)
        self.disconnect_button.pack(side="left")
        self.demo_check = ttk.Checkbutton(connection, text="演示数据（无硬件）", variable=self.demo)
        self.demo_check.pack(side="right")
        self.sync_check = ttk.Checkbutton(connection, text="同步协议", variable=self.sync_mode)
        self.banner = tk.Label(outer, textvariable=self.status, anchor="w", bg="#e2e8f0", fg=INK, padx=12, pady=9)
        self.banner.pack(fill="x", pady=12)
        self.banner.bind("<Configure>", lambda e: self.banner.configure(wraplength=max(300,e.width-24)))

        cards = ttk.Frame(outer)
        cards.pack(fill="x")
        for index, (title, variable, subtitle) in enumerate((
            ("换算距离 / cm", self.distance_text, "只在已标定范围内换算"),
            ("CCD 原始坐标", self.coordinate_text, "保留原始值，不自动删离群点"),
            ("相对尺量值的偏差 / cm", self.error_text, "显示值 − 尺量值；不是精度保证"),
        )):
            cards.columnconfigure(index, weight=1)
            card = tk.Frame(cards, bg="white", padx=16, pady=12)
            card.grid(row=0, column=index, sticky="nsew", padx=(0 if index == 0 else 8, 0))
            tk.Label(card, text=title, bg="white", fg=MUTED).pack(anchor="w")
            tk.Label(card, textvariable=variable, bg="white", fg=BLUE,
                     font=("Microsoft YaHei UI", 29, "bold")).pack(anchor="w", pady=4)
            tk.Label(card, text=subtitle, bg="white", fg=MUTED, font=("Microsoft YaHei UI", 9)).pack(anchor="w")

        controls = ttk.Frame(outer)
        controls.pack(fill="x", pady=(14, 5))
        self.laser_button = ttk.Button(controls, text="开启激光", command=self.toggle_laser)
        self.laser_button.pack(side="left")
        self.single_button = ttk.Button(controls, text="测一次", command=self.measure_once)
        self.single_button.pack(side="left", padx=8)
        self.run_button = ttk.Button(controls, text="连续测量", command=self.toggle_continuous, style="Primary.TButton")
        self.run_button.pack(side="left")
        self.sync_button = ttk.Button(controls, text="开始同步扫描", command=self.toggle_sync, style="Primary.TButton")
        ttk.Label(controls, text="  采样间隔 ms").pack(side="left")
        self.interval_entry = ttk.Spinbox(controls, from_=200, to=10000, increment=100,
                                          textvariable=self.interval, width=6)
        self.interval_entry.pack(side="left", padx=5)
        self.sync_rate_entry = ttk.Spinbox(controls, from_=1, to=50, increment=1,
                                            textvariable=self.sync_rate, width=5)
        ttk.Label(controls, text="尺量距离 cm（可留空）").pack(side="left", padx=(14, 5))
        self.reference_entry = ttk.Entry(controls, textvariable=self.reference, width=9)
        self.reference_entry.pack(side="left")
        ttk.Label(outer, textvariable=self.range_text, foreground=MUTED).pack(anchor="w", pady=3)

        graph_top = ttk.Frame(outer)
        graph_top.pack(fill="x", pady=(8, 4))
        ttk.Label(graph_top, text="最近 60 次测量", font=("Microsoft YaHei UI", 11, "bold")).pack(side="left")
        self.axis_combo = ttk.Combobox(graph_top, values=("距离 / cm", "原始坐标"),
                                      textvariable=self.chart_axis, width=12, state="readonly")
        self.axis_combo.pack(side="right")
        self.axis_combo.bind("<<ComboboxSelected>>", lambda _: self.draw_chart())
        self.canvas = tk.Canvas(outer, height=185, bg="#112139", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _: self.draw_chart())
        ttk.Label(outer, textvariable=self.stats_text, foreground=MUTED).pack(anchor="w", pady=5)

        bottom = ttk.Frame(outer)
        bottom.pack(fill="x", pady=(4, 7))
        self.import_button = ttk.Button(bottom, text="导入标定 CSV", command=self.import_calibration)
        self.import_button.pack(side="left")
        ttk.Button(bottom, text="查看标定表", command=self.show_calibration).pack(side="left", padx=6)
        ttk.Label(bottom, textvariable=self.cal_text, foreground=MUTED).pack(side="left", padx=8)
        self.clear_button = ttk.Button(bottom, text="清空记录", command=self.clear_records)
        self.clear_button.pack(side="right")
        ttk.Button(bottom, text="导出 CSV", command=self.export).pack(side="right", padx=8)

        tabs = ttk.Notebook(outer)
        tabs.pack(fill="both", expand=True)
        table_frame = ttk.Frame(tabs)
        tabs.add(table_frame, text="测量记录")
        columns = ("seq", "time", "x", "distance", "reference", "error", "state")
        self.table = ttk.Treeview(table_frame, columns=columns, show="headings", height=5)
        for key, title, width in zip(columns, ("序号", "电脑接收时间", "CCD 坐标", "距离 cm", "尺量 cm", "偏差 cm", "状态"),
                                     (50, 130, 90, 90, 90, 90, 260)):
            self.table.heading(key, text=title)
            self.table.column(key, width=width, anchor="center", stretch=key == "state")
        scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=self.table.yview)
        self.table.configure(yscrollcommand=scrollbar.set)
        self.table.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        log_frame = ttk.Frame(tabs)
        tabs.add(log_frame, text="通信日志（排查时查看）")
        self.log_widget = tk.Text(log_frame, height=5, font=("Consolas", 9), wrap="word", state="disabled")
        self.log_widget.pack(fill="both", expand=True)
        footer = ttk.Frame(outer)
        footer.pack(fill="x", pady=(6, 0))
        ttk.Label(footer, text="电脑时间仅用于记录请求与回复；距离只在当前标定范围内换算。", foreground=MUTED,
                  font=("Microsoft YaHei UI", 9)).pack(side="left")
        ttk.Label(footer, textvariable=self.counter, foreground=MUTED).pack(side="right")

    def _cal_label(self):
        points = self.calibration.points
        self.cal_text.set("{} 点 · {:g}–{:g} cm".format(len(points), points[0][0], points[-1][0]))

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [p.device + " | " + p.description for p in list_ports.comports()]
        except ImportError:
            ports = []
            self.status.set("缺少 pyserial；可使用打包版，或运行 pip install pyserial")
        self.port_combo["values"] = ports
        if self.port_text.get() not in ports:
            self.port_text.set(ports[0] if ports else "")

    def _controls(self):
        disconnected = self.worker is None
        idle = self.ready and not self.pending and not self.continuous
        static_mode = not self.sync_mode.get()
        for widget, enabled in (
            (self.connect_button, disconnected), (self.refresh_button, disconnected),
            (self.demo_check, disconnected), (self.sync_check, disconnected),
            (self.disconnect_button, not disconnected),
            (self.laser_button, idle and static_mode), (self.single_button, idle and self.laser and static_mode),
            (self.run_button, static_mode and (self.continuous or (idle and self.laser))),
            (self.sync_button, self.ready and self.sync_mode.get() and not self.pending),
            (self.import_button, disconnected), (self.clear_button, not self.pending and not self.continuous),
            (self.interval_entry, static_mode and not self.continuous),
            (self.sync_rate_entry, disconnected or (self.sync_mode.get() and not self.sync_active)),
        ):
            widget.configure(state="normal" if enabled else "disabled")
        self.port_combo.configure(state="readonly" if disconnected else "disabled")
        self.laser_button.configure(text="关闭激光" if self.laser else "开启激光")
        self.run_button.configure(text="停止连续测量" if self.continuous else "连续测量")
        self.sync_button.configure(text="停止同步扫描" if self.sync_active else "开始同步扫描")

    def connect(self):
        if self.worker:
            return
        if not self.demo.get() and not self.port_text.get():
            messagebox.showinfo("选择串口", "请接入设备并选择串口；无硬件时可以勾选演示数据。")
            return
        self.port_name = "DEMO" if self.demo.get() else self.port_text.get().split(" | ")[0]
        self.source = "DEMO" if self.demo.get() else "HARDWARE"
        self.events = queue.Queue()
        worker_type = SyncWorker if self.sync_mode.get() else SerialWorker
        if self.sync_mode.get():
            try:
                rate = float(self.sync_rate.get())
                if not 1 <= rate <= 50:
                    raise ValueError()
            except ValueError:
                messagebox.showerror("同步频率", "请输入 1～50 Hz 的同步频率。")
                return
            self.sync_rate.set("{:g}".format(rate))
        self.worker = worker_type(self.port_name, self.events, self.demo.get(),
                                  rate_hz=float(self.sync_rate.get())) if self.sync_mode.get() else \
                     worker_type(self.port_name, self.events, self.demo.get())
        self.ready = False
        self.pending = True
        self.laser = False
        self.sync_active = False
        self.sync_angles.clear()
        self.window.clear()
        self.chart_start = len(self.records)
        self.draw_chart()
        self._reset_cards("正在连接；等待设备启动并设置曝光 5…")
        self.status.set("正在连接 {} · 请稍候".format(self.port_name))
        self.banner.configure(bg="#fef3c7")
        self.worker.start()
        self._controls()

    def disconnect(self):
        self.continuous = False
        self.ready = False
        if self.worker:
            self.worker.stop()
            self.status.set("正在断开，并尝试关闭激光…")
        self._controls()

    def toggle_laser(self):
        if not self.ready or self.pending or self.sync_mode.get():
            return
        self.pending = True
        self.window.clear()
        self.worker.submit("laser", not self.laser)
        self._controls()

    def toggle_sync(self):
        if not self.ready or self.pending or not self.sync_mode.get() or not isinstance(self.worker, SyncWorker):
            return
        if self.sync_active:
            self.pending = True
            self.worker.submit("sync_stop")
            self.status.set("正在停止同步扫描…")
        else:
            self.pending = True
            session = "S{}".format(datetime.now().strftime("%m%d%H%M%S"))
            self.worker.submit("sync_start", {"session_id": session,
                                                "rate_hz": float(self.sync_rate.get()),
                                                "mode": "fffe"})
            self.status.set("正在启动同步扫描…")
        self._controls()

    def measure_once(self):
        if not self.ready or self.pending or not self.laser or self.sync_mode.get():
            return
        if len(self.records) >= 10000:
            self.continuous = False
            self.status.set("已达到 10000 条记录，请导出后清空，再继续测量")
            self._controls()
            return
        try:
            reference = parse_reference(self.reference.get())
        except ValueError:
            self.continuous = False
            messagebox.showerror("尺量距离", "请输入大于 0 的距离（厘米），或留空。")
            self._controls()
            return
        self.pending = True
        self.worker.submit("measure", {"reference": reference, "calibration": self.calibration})
        self._controls()

    def toggle_continuous(self):
        if self.sync_mode.get():
            return
        if self.continuous:
            self.continuous = False
            self.status.set("连续测量已停止；已发出的最后一次请求仍会记录")
        else:
            try:
                ms = int(self.interval.get())
                if not 200 <= ms <= 10000:
                    raise ValueError()
                parse_reference(self.reference.get())
            except ValueError:
                messagebox.showerror("参数", "采样间隔请输入 200～10000 毫秒的整数；尺量距离应为正数或留空。")
                return
            self.continuous = True
            self.failures = 0
            self.next_due = 0
            self.status.set("演示连续测量（模拟数据）" if self.source == "DEMO" else "连续测量中 · 每次等待回复后再发送下一条")
        self._controls()

    def _reset_cards(self, reason, clear_stats=True):
        self.distance_text.set("—")
        self.coordinate_text.set("—")
        self.error_text.set("—")
        self.range_text.set(reason)
        if clear_stats:
            self.stats_text.set("最近 20 次坐标：暂无数据")

    def _sample(self, data):
        calibration = data["calibration"]
        x = data["x"]
        reference = data["reference"]
        rx = data.get("rx") or datetime.now().astimezone().isoformat(timespec="milliseconds")
        tx = data.get("tx") or rx
        elapsed = data.get("elapsed")
        session_id = data.get("session_id", "")
        sample_id = data.get("sample_id", "")
        device_begin_us = data.get("device_begin_us", "")
        device_end_us = data.get("device_end_us", "")
        angle_deg = data.get("angle_deg", "")
        trigger_id = data.get("trigger_id", "")
        if session_id and sample_id != "":
            angle = self.sync_angles.pop((session_id, sample_id), None)
            if angle:
                angle_deg = angle.get("angle_deg", "")
                trigger_id = angle.get("trigger_id", "")
        if x is None:
            distance, state = None, "测量失败：" + data.get("failure", "无有效坐标")
            self.failures += 1
        else:
            distance, state = calibration.convert(x)
            self.failures = 0
            self.window.append(x)
        error = distance - reference if distance is not None and reference is not None else None
        record = {"sequence": len(self.records) + 1, "source": self.source, "port": self.port_name,
                  "pc_request_time": tx, "pc_receive_time": rx,
                  "round_trip_ms": elapsed, "session_id": session_id,
                  "sample_id": sample_id, "device_begin_us": device_begin_us,
                  "device_end_us": device_end_us, "angle_deg": angle_deg,
                  "trigger_id": trigger_id, "ccd_x": x, "distance_cm": distance,
                  "reference_cm": reference, "error_cm": error, "status": state,
                  "calibration_id": calibration.identifier, "calibration_points": repr(calibration.points),
                  "exposure_sent": EXPOSURE_INDEX}
        self.records.append(record)
        self.distance_text.set("—" if distance is None else "{:.2f}".format(distance))
        self.coordinate_text.set("—" if x is None else str(x))
        self.error_text.set("—" if error is None else "{:+.2f}".format(error))
        tag = "演示数据 · " if self.source == "DEMO" else ""
        sync_note = ""
        if session_id and sample_id != "":
            sync_note = " · 同步 {} #{} 设备 {}–{} μs".format(
                session_id, sample_id, device_begin_us, device_end_us)
        self.range_text.set(tag + state + " · 上次回复 " + rx[11:23] + sync_note + " · 保留两位小数不代表毫米级准确度")
        if self.window:
            values = list(self.window)
            std = statistics.stdev(values) if len(values) > 1 else 0.0
            self.stats_text.set("最近 {} 次有效坐标：中位数 {:g}   最小 {}   最大 {}   极差 {}   标准差 {:.2f}（重复性，不是准确度）".format(
                len(values), statistics.median(values), min(values), max(values), max(values)-min(values), std))
        show = lambda value: "—" if value is None else "{:.2f}".format(value)
        self.table.insert("", "end", values=(record["sequence"], rx[11:23],
            x if x is not None else "—", show(distance), show(reference), show(error), tag + state))
        children = self.table.get_children()
        if len(children) > 200:
            self.table.delete(children[0])
        self.table.yview_moveto(1)
        self.counter.set("{} 条记录 · 表格显示最近 200 条".format(len(self.records)))
        if self.failures >= 3:
            self.continuous = False
            self.status.set("连续 3 次测量失败，已停止；查看通信日志并检查 CCD")
        self.draw_chart()

    def _log(self, text):
        self.log_history.append(text)
        self.log_widget.configure(state="normal")
        self.log_widget.insert("end", text + "\n")
        count = int(self.log_widget.index("end-1c").split(".")[0])
        if count > 200:
            self.log_widget.delete("1.0", "{}.0".format(count-200))
        self.log_widget.see("end")
        self.log_widget.configure(state="disabled")

    def show_log(self):
        window = tk.Toplevel(self.root)
        window.title("连接诊断 · 最近通信日志")
        window.geometry("760x450")
        text = tk.Text(window, wrap="word", font=("Consolas", 10))
        text.pack(fill="both", expand=True, padx=12, pady=12)
        contents = "\n".join(self.log_history) or "暂无通信记录"
        text.insert("1.0", contents)
        text.see("end")
        text.configure(state="disabled")
        def copy():
            window.clipboard_clear()
            window.clipboard_append(contents)
        ttk.Button(window, text="复制全部日志", command=copy).pack(pady=(0,12))

    def _tick(self):
        for _ in range(200):
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                self._log(value)
            elif kind == "phase":
                self.status.set(value)
            elif kind == "ready":
                self.ready = True
                self.pending = False
                if self.source == "DEMO":
                    message = "演示模式 · 模拟数据，未连接硬件"
                elif self.sync_mode.get():
                    message = "已连接 {} · 同步协议就绪 · 点击「开始同步扫描」".format(self.port_name)
                else:
                    message = "已连接 {} · 曝光 5 已发送 · 点击「开启激光」".format(self.port_name)
                self.status.set(message)
                self.banner.configure(bg="#fef3c7" if self.source == "DEMO" else "#dcfce7")
            elif kind == "laser":
                self.laser = value
                if not value:
                    self.continuous = False
                    self._reset_cards("激光已关闭；历史数据可在下方查看")
            elif kind == "sample":
                self._sample(value)
            elif kind == "sync_started":
                self.pending = False
                self.sync_active = True
                self.status.set("同步扫描中 · 会话 {} · {:.1f} Hz".format(
                    value["session_id"], value["rate_hz"]))
            elif kind == "sync_stopped":
                self.pending = False
                self.sync_active = False
                self.status.set("同步扫描已停止；设备时间和序号已保留")
            elif kind == "sync_angle":
                self.sync_angles[(value["session_id"], value["sample_id"])] = value
            elif kind == "sync_sample":
                self.pending = False
                sample = dict(value)
                sample.update({"calibration": self.calibration,
                               "reference": None,
                               "x": value.get("ccd_x"),
                               "elapsed": (value["device_end_us"] - value["device_begin_us"]) / 1000.0})
                self._sample(sample)
            elif kind == "idle":
                self.pending = False
                # Delay after completion. No accumulation of outstanding MINs.
                self.next_due = time.monotonic() + int(self.interval.get()) / 1000 if self.continuous else 0
            elif kind == "device_error":
                self.continuous = False
                self.status.set("设备拒绝指令：" + value)
                self._log(value)
            elif kind == "fatal":
                self.continuous = False
                self.sync_active = False
                self.ready = False
                self.status.set("连接异常：" + value)
                self.banner.configure(bg="#fee2e2")
                self._reset_cards("没有新的有效测量；下方保留历史记录，请排查后重连", clear_stats=False)
                self._log(value)
                if not self.closing:
                    self.show_log()
            elif kind == "closed":
                self.worker = None
                self.pending = False
                self.ready = False
                self.laser = False
                self.continuous = False
                self.sync_active = False
                if not value:
                    self.status.set("连接已关闭，但未确认激光关闭；请检查设备，必要时手动断电")
                elif not self.status.get().startswith("连接异常"):
                    self.status.set("已断开 · 显示值为历史记录")
                if self.closing:
                    self.root.destroy()
                    return
            self._controls()
        if self.continuous and self.ready and not self.pending and time.monotonic() >= self.next_due:
            self.measure_once()
        self.root.after(50, self._tick)

    def draw_chart(self):
        canvas = self.canvas
        canvas.delete("all")
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 100 or height < 80:
            return
        rows = self.records[self.chart_start:][-60:]
        key = "distance_cm" if self.chart_axis.get() == "距离 / cm" else "ccd_x"
        values = [r[key] for r in rows if r[key] is not None]
        left, right, top, bottom = 62, width-22, 22, height-28
        if not values:
            canvas.create_text(width/2, height/2, text="连接设备并测量后，曲线将在这里显示", fill="#94a3b8",
                               font=("Microsoft YaHei UI", 12))
            return
        low, high = min(values), max(values)
        padding = max((high-low)*0.15, 0.5 if key == "distance_cm" else 2)
        low, high = low-padding, high+padding
        for i in range(5):
            y = top + (bottom-top)*i/4
            canvas.create_line(left, y, right, y, fill="#263954")
            canvas.create_text(left-9, y, text="{:.1f}".format(high-(high-low)*i/4), fill="#b5c6dd", anchor="e")
        previous = None
        for index, row in enumerate(rows):
            value = row[key]
            if value is None:
                previous = None
                continue
            point = (left+(right-left)*index/max(len(rows)-1, 1), bottom-(value-low)/(high-low)*(bottom-top))
            if previous:
                canvas.create_line(*previous, *point, fill="#60a5fa", width=2)
            canvas.create_oval(point[0]-2, point[1]-2, point[0]+2, point[1]+2, fill="#a5d8ff", outline="")
            previous = point
        canvas.create_text(left, height-12, text="序号 {}".format(rows[0]["sequence"]), fill="#94a3b8", anchor="w")
        canvas.create_text(right, height-12, text="{} · 自动纵轴；无效值断线".format(rows[-1]["sequence"]), fill="#94a3b8", anchor="e")

    def import_calibration(self):
        path = filedialog.askopenfilename(title="选择标定表", filetypes=[("CSV", "*.csv")])
        if not path:
            return
        try:
            calibration = Calibration.from_csv(path)
        except (OSError, ValueError, KeyError) as error:
            messagebox.showerror("标定表无效", str(error))
            return
        self.calibration = calibration
        self.window.clear()
        self._reset_cards("已更换标定表；历史记录保持原换算结果")
        self._cal_label()
        self.status.set("已载入标定表：" + Path(path).name)

    def show_calibration(self):
        window = tk.Toplevel(self.root)
        window.title("当前标定表 · 分段线性插值")
        window.geometry("440x470")
        ttk.Label(window, text="标定编号 " + self.calibration.identifier, padding=12).pack()
        table = ttk.Treeview(window, columns=("distance", "x"), show="headings", height=12)
        table.heading("distance", text="距离 / cm")
        table.heading("x", text="CCD 坐标")
        table.pack(fill="both", expand=True, padx=12)
        for d, x in self.calibration.points:
            table.insert("", "end", values=("{:g}".format(d), "{:g}".format(x)))
        def save():
            path = filedialog.asksaveasfilename(parent=window, defaultextension=".csv", initialfile="calibration.csv")
            if path:
                try:
                    self.calibration.save(path)
                except OSError as error:
                    messagebox.showerror("保存失败", str(error), parent=window)
        ttk.Button(window, text="导出此标定表", command=save).pack(pady=12)

    def export(self):
        if not self.records:
            messagebox.showinfo("导出", "还没有测量记录")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv",
            initialfile="测距记录_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv",
            filetypes=[("CSV", "*.csv")])
        if path:
            try:
                write_records(path, list(self.records))
                self.status.set("已导出 {} 条记录：{}".format(len(self.records), Path(path).name))
            except OSError as error:
                messagebox.showerror("导出失败", str(error))

    def clear_records(self):
        if self.records and not messagebox.askyesno("清空记录", "清空当前记录？尚未导出的数据将丢失。"):
            return
        self.records.clear()
        self.chart_start = 0
        self.window.clear()
        self.sync_angles.clear()
        for item in self.table.get_children():
            self.table.delete(item)
        self._reset_cards("记录已清空")
        self.counter.set("0 条记录")
        self.draw_chart()

    def close(self):
        if self.closing:
            return
        if self.records and not messagebox.askyesno("退出", "测量记录不会自动保存。确认已导出需要的数据并退出？"):
            return
        self.closing = True
        if self.worker:
            self.disconnect()
        else:
            self.root.destroy()


def main():
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass
    root = tk.Tk()
    DistanceApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
