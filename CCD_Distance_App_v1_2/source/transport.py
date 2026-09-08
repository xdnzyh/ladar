"""One worker owns the serial port. Requests never overlap."""
from collections import deque
from datetime import datetime
import math
import queue
import threading
import time

from core import LineBuffer, parse_coordinate_line, parse_pix_line, parse_angle_line


class LinkError(Exception):
    pass


class ReplyTimeout(LinkError):
    pass


class DeviceError(Exception):
    pass


class Cancelled(Exception):
    pass


class Protocol:
    def __init__(self, port, stop_event, log=lambda line: None):
        self.port = port
        self.stop_event = stop_event
        self.log = log
        self.buffer = LineBuffer()
        self.lines = deque()

    def transact(self, command, predicate, timeout=5.0, cancellable=True):
        if cancellable and self.stop_event.is_set():
            raise Cancelled()
        self.log("> " + command)
        payload = (command + "\r\n").encode("ascii")
        if self.port.write(payload) != len(payload):
            raise LinkError("串口发送不完整")
        deadline = time.monotonic() + timeout
        line_count = 0
        recent = deque(maxlen=3)
        while time.monotonic() < deadline:
            if cancellable and self.stop_event.is_set():
                raise Cancelled()
            if not self.lines:
                try:
                    self.lines.extend(self.buffer.feed(self.port.read(256)))
                except (UnicodeError, ValueError) as error:
                    raise LinkError("串口数据格式异常：" + str(error)) from error
            while self.lines:
                line = self.lines.popleft().strip()
                line_count += 1
                if line_count > 100:
                    raise LinkError("非预期消息过多，已停止接收；检查串口自动发送及固件")
                self.log("< " + line)
                recent.append(line)
                if line.startswith("ERR") or line.startswith("CCD INVALID:"):
                    raise DeviceError(line)
                result = predicate(line)
                if result is not None:
                    return result
        # No next measurement is issued after a timeout: a delayed old reply
        # has no request id in this legacy protocol and cannot be assigned safely.
        received = " | ".join(recent) if recent else "没有完整回复"
        if self.buffer.buffer:
            received += "；残片 HEX=" + bytes(self.buffer.buffer[:48]).hex(" ")
        raise ReplyTimeout("指令 [{}] 回复超时；最近收到：{}".format(command, received))

    def discard_before_retry(self):
        self.lines.clear()
        self.buffer = LineBuffer()
        self.port.reset_input_buffer()


def exact(expected):
    return lambda line: line if line == expected else None


class SerialWorker(threading.Thread):
    def __init__(self, port_name, events, demo=False):
        super().__init__(daemon=True)
        self.port_name = port_name
        self.events = events
        self.demo = demo
        self.jobs = queue.Queue()
        self.stop_event = threading.Event()
        self.laser_may_be_on = False
        self.demo_count = 0

    def emit(self, kind, value=None):
        self.events.put((kind, value))

    def submit(self, action, value=None):
        self.jobs.put((action, value))

    def stop(self):
        self.stop_event.set()

    def _connect(self):
        import serial
        port = serial.Serial(port=None, baudrate=115200, timeout=0.05,
                             write_timeout=0.5, rtscts=False, dsrdtr=False)
        port.dtr = False
        port.rts = False
        port.port = self.port_name
        try:
            port.open()
            # Some USB adapters reset the board on open despite the line settings.
            if self.stop_event.wait(3.0):
                raise Cancelled()
            port.reset_input_buffer()
            return port
        except BaseException:
            port.close()
            raise

    def _handshake(self, protocol):
        def step(command, predicate):
            # Only the idempotent setup commands below may be retried.
            # MIN and LASER 1 are not routed through this helper.
            for attempt in range(2):
                self.emit("phase", "连接中：{}（第 {} 次）".format(command, attempt+1))
                # Space setup requests without changing measurement timestamps.
                if self.stop_event.wait(.25):
                    raise Cancelled()
                try:
                    return protocol.transact(command, predicate)
                except ReplyTimeout as error:
                    self.emit("log", str(error))
                    if attempt:
                        raise
                    self.emit("log", "连接指令丢失回复；等待后重试一次：" + command)
                    if self.stop_event.wait(.5):
                        raise Cancelled()
                    protocol.discard_before_retry()
        step("STOP", exact("OK STOP"))
        step("DEBUG 0", exact("OK DEBUG 0"))
        step("LASER 0", exact("OK LASER 0"))
        status = step("STATUS", lambda s: s if s.startswith("STATUS CCD-CAL-") else None)
        if "BATCH=0" not in status:
            raise LinkError("设备仍在批量采样；请复位后重连")
        step("EXPOSURE 3", lambda s: s if s.startswith("EXPOSURE SENT: 3 ") and s.endswith("(NO READBACK VERIFICATION)") else None)
        return status

    def run(self):
        port = None
        protocol = None
        off_confirmed = True
        try:
            if not self.demo:
                port = self._connect()
                protocol = Protocol(port, self.stop_event, lambda s: self.emit("log", s))
                status = self._handshake(protocol)
            else:
                status = "演示模式：所有数值均为模拟数据，未连接硬件"
            self.emit("ready", status)
            while not self.stop_event.is_set():
                try:
                    action, context = self.jobs.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    if action == "laser":
                        # A missing ON acknowledgement does not prove the laser stayed off.
                        if context:
                            self.laser_may_be_on = True
                        if protocol:
                            protocol.transact("LASER " + str(int(context)), exact("OK LASER " + str(int(context))))
                        self.laser_may_be_on = bool(context)
                        self.emit("laser", bool(context))
                    elif action == "measure":
                        if not self.laser_may_be_on:
                            raise DeviceError("激光未开启")
                        tx = datetime.now().astimezone().isoformat(timespec="milliseconds")
                        start = time.monotonic()
                        try:
                            if protocol:
                                x = protocol.transact("MIN", parse_coordinate_line)
                            else:
                                if self.stop_event.wait(0.08):
                                    raise Cancelled()
                                self.demo_count += 1
                                x = 936 + round(10 * math.sin(self.demo_count * 0.35))
                            self.emit("sample", dict(context, x=x, tx=tx,
                                rx=datetime.now().astimezone().isoformat(timespec="milliseconds"),
                                elapsed=round((time.monotonic()-start)*1000, 1)))
                        except (DeviceError, LinkError) as error:
                            self.emit("sample", dict(context, x=None, tx=tx,
                                rx=datetime.now().astimezone().isoformat(timespec="milliseconds"),
                                elapsed=round((time.monotonic()-start)*1000, 1), failure=str(error)))
                            if isinstance(error, LinkError):
                                raise
                except DeviceError as error:
                    self.emit("device_error", str(error))
                finally:
                    self.emit("idle")
        except Cancelled:
            pass
        except Exception as error:
            self.emit("fatal", str(error))
        finally:
            if self.laser_may_be_on and protocol:
                try:
                    protocol.transact("LASER 0", exact("OK LASER 0"), timeout=1.5, cancellable=False)
                    self.laser_may_be_on = False
                except Exception:
                    off_confirmed = False
            if port:
                port.close()
            self.emit("closed", off_confirmed)


class SyncWorker(SerialWorker):
    """Worker for the synchronized PIX protocol used by main_sync_v3_fixed."""

    def __init__(self, port_name, events, demo=False, rate_hz=10.0):
        super().__init__(port_name, events, demo=demo)
        self.rate_hz = float(rate_hz)
        self.session = "S{}".format(datetime.now().strftime("%m%d%H%M%S"))
        self.scan_active = False

    def _poll_lines(self, protocol):
        if not protocol.lines:
            data = protocol.port.read(256)
            if data:
                protocol.lines.extend(protocol.buffer.feed(data))
        while protocol.lines:
            line = protocol.lines.popleft().strip()
            protocol.log("< " + line)
            yield line

    def _handle_line(self, line):
        if line.startswith("PIX "):
            try:
                packet = parse_pix_line(line)
            except ValueError as error:
                self.emit("log", "同步包错误：{}；原文={}".format(error, line))
                return
            if packet is not None:
                packet["rx"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
                packet["source"] = "SYNC"
                self.emit("sync_sample", packet)
                return
        if line.startswith("ANGLE ") or line.startswith("TRIG "):
            try:
                packet = parse_angle_line(line)
            except ValueError as error:
                self.emit("log", "角度包错误：{}；原文={}".format(error, line))
                return
            if packet is not None:
                self.emit("sync_angle", packet)
                return
        if line.startswith("ERROR") or line.startswith("CCD INVALID"):
            self.emit("device_error", line)

    def run(self):
        port = None
        protocol = None
        off_confirmed = True
        try:
            if self.demo:
                self.emit("ready", "演示模式：同步数据为模拟数据，未连接硬件")
                self.emit("sync_state", "ready")
                next_sample = 0.0
                while not self.stop_event.is_set():
                    try:
                        action, context = self.jobs.get(timeout=.02)
                    except queue.Empty:
                        action, context = None, None
                    if action == "sync_start":
                        self.scan_active = True
                        next_sample = 0.0
                        self.emit("sync_started", {"session_id": self.session,
                            "rate_hz": self.rate_hz})
                    elif action == "sync_stop":
                        self.scan_active = False
                        self.emit("sync_stopped")
                    if self.scan_active and time.monotonic() >= next_sample:
                        self.demo_count += 1
                        now = int(time.monotonic() * 1000000)
                        self.emit("sync_sample", {
                            "session_id": self.session, "sample_id": self.demo_count,
                            "device_begin_us": now, "device_end_us": now + 1000,
                            "ccd_x": 936 + round(10 * math.sin(self.demo_count * .35)),
                            "rx": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                            "source": "DEMO"})
                        next_sample = time.monotonic() + max(.01, 1.0/self.rate_hz)
                self.emit("closed", True)
                return

            port = self._connect()
            protocol = Protocol(port, self.stop_event, lambda s: self.emit("log", s))
            status = self._handshake(protocol)
            self.emit("ready", status)
            self.emit("sync_state", "ready")
            while not self.stop_event.is_set():
                for line in self._poll_lines(protocol):
                    self._handle_line(line)
                try:
                    action, context = self.jobs.get(timeout=.02)
                except queue.Empty:
                    continue
                if action == "sync_start":
                    session = context.get("session_id", self.session)
                    rate_hz = float(context.get("rate_hz", self.rate_hz))
                    mode = context.get("mode", "fffe")
                    command = "START {} {} 3 {}".format(session, rate_hz, mode)
                    protocol.transact(command, exact("OK START " + session))
                    self.session = session
                    self.rate_hz = rate_hz
                    self.scan_active = True
                    self.laser_may_be_on = True
                    self.emit("sync_started", {"session_id": session, "rate_hz": rate_hz})
                elif action == "sync_stop":
                    protocol.transact("STOP", exact("OK STOP"), cancellable=False)
                    self.scan_active = False
                    self.laser_may_be_on = False
                    self.emit("sync_stopped")
                elif action == "laser":
                    value = bool(context)
                    protocol.transact("LASER " + str(int(value)), exact("OK LASER " + str(int(value))))
                    self.laser_may_be_on = value
                    self.emit("laser", value)
        except Cancelled:
            pass
        except Exception as error:
            self.emit("fatal", str(error))
        finally:
            if protocol and (self.scan_active or self.laser_may_be_on):
                try:
                    protocol.transact("STOP", exact("OK STOP"), timeout=1.5, cancellable=False)
                    self.scan_active = False
                    self.laser_may_be_on = False
                except Exception:
                    off_confirmed = False
            if port:
                port.close()
            self.emit("closed", off_confirmed)
