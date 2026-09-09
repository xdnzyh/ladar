from __future__ import annotations

import ctypes
from ctypes import wintypes
import queue
import sys
import threading
import time
from typing import Callable


DataCallback = Callable[[bytes, float], None]
ErrorCallback = Callable[[str], None]


def list_serial_ports() -> list[str]:
    ports: set[str] = set()
    try:
        from serial.tools import list_ports

        ports.update(info.device for info in list_ports.comports())
    except ImportError:
        pass

    if sys.platform == "win32":
        try:
            import winreg

            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
            index = 0
            while True:
                try:
                    _, value, _ = winreg.EnumValue(key, index)
                except OSError:
                    break
                ports.add(str(value))
                index += 1
            winreg.CloseKey(key)
        except OSError:
            pass

    def sort_key(name: str) -> tuple[str, int]:
        prefix = "".join(c for c in name if not c.isdigit())
        digits = "".join(c for c in name if c.isdigit())
        return prefix, int(digits) if digits else 0

    return sorted(ports, key=sort_key)


class SerialEndpoint:
    def __init__(
        self,
        name: str,
        on_data: DataCallback,
        on_error: ErrorCallback,
    ) -> None:
        self.name = name
        self.on_data = on_data
        self.on_error = on_error
        self.port = ""
        self.baudrate = 115200
        self._transport = None
        self._reader_thread: threading.Thread | None = None
        self._writer_thread: threading.Thread | None = None
        self._faulted = threading.Event()
        self._transport_lock = threading.Lock()
        self._write_queue: queue.Queue = queue.Queue()
        self._session_stop: threading.Event | None = None
        self._session_id = 0
        self._active_on_data = on_data
        self._active_on_error = on_error

    @property
    def is_open(self) -> bool:
        with self._transport_lock:
            transport = self._transport
        return not self._faulted.is_set() and transport is not None and transport.is_open

    def open(
        self,
        port: str,
        baudrate: int = 115200,
        *,
        on_data: DataCallback | None = None,
        on_error: ErrorCallback | None = None,
    ) -> None:
        if self.is_open:
            return
        self.port = port
        self.baudrate = baudrate
        self._faulted.clear()
        transport = _open_transport(port, baudrate)
        stop = threading.Event()
        write_queue: queue.Queue = queue.Queue()
        with self._transport_lock:
            self._session_id += 1
            session_id = self._session_id
            self._session_stop = stop
            self._write_queue = write_queue
            self._active_on_data = on_data or self.on_data
            self._active_on_error = on_error or self.on_error
        self._write_queue = write_queue
        with self._transport_lock:
            self._transport = transport
        self._reader_thread = threading.Thread(
            target=self._read_loop,
            args=(session_id, stop, transport, self._active_on_data, self._active_on_error),
            name=f"{self.name}-reader",
            daemon=True,
        )
        self._writer_thread = threading.Thread(
            target=self._write_loop,
            args=(session_id, stop, transport, write_queue, self._active_on_error),
            name=f"{self.name}-writer",
            daemon=True,
        )
        self._reader_thread.start()
        self._writer_thread.start()

    def close(self) -> None:
        with self._transport_lock:
            transport, self._transport = self._transport, None
            stop, self._session_stop = self._session_stop, None
            write_queue = self._write_queue
            self._session_id += 1
        if stop is not None:
            stop.set()
        self._cancel_queue(write_queue)
        write_queue.put(None)
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass
        for thread in (self._reader_thread, self._writer_thread):
            if thread and thread.is_alive() and thread is not threading.current_thread():
                thread.join(timeout=0.4)
        self._reader_thread = None
        self._writer_thread = None
        self._faulted.clear()

    def write(
        self,
        data: bytes | str,
        on_sent: Callable[[float], None] | None = None,
        on_written: Callable[[float], None] | None = None,
        priority: bool = False,
    ) -> bool:
        if isinstance(data, str):
            data = data.encode("ascii", "ignore")
        item = (bytes(data), on_sent, on_written)
        with self._transport_lock:
            transport = self._transport
            write_queue = self._write_queue
            session_stop = self._session_stop
        if transport is None or session_stop is None or session_stop.is_set() or not transport.is_open:
            return False
        if priority:
            pending = []
            while True:
                try:
                    pending.append(write_queue.get_nowait())
                except queue.Empty:
                    break
            for old_item in pending:
                write_queue.task_done()
            write_queue.put(item)
            for old_item in pending:
                if old_item is not None:
                    write_queue.put(old_item)
        else:
            write_queue.put(item)
        return True

    def write_line(
        self,
        text: str,
        on_sent: Callable[[float], None] | None = None,
        on_written: Callable[[float], None] | None = None,
        priority: bool = False,
    ) -> bool:
        return self.write(text.rstrip("\r\n") + "\r\n", on_sent, on_written, priority)

    def cancel_pending(self) -> int:
        with self._transport_lock:
            write_queue = self._write_queue
        return self._cancel_queue(write_queue)

    @staticmethod
    def _cancel_queue(write_queue: queue.Queue) -> int:
        cancelled = 0
        while True:
            try:
                item = write_queue.get_nowait()
            except queue.Empty:
                break
            write_queue.task_done()
            if item is not None:
                cancelled += 1
        return cancelled

    def flush(self, timeout: float = 0.5) -> bool:
        if timeout < 0:
            raise ValueError("发送等待时间不能为负数")
        with self._transport_lock:
            write_queue = self._write_queue
        deadline = time.perf_counter() + timeout
        while write_queue.unfinished_tasks:
            if time.perf_counter() >= deadline:
                return False
            time.sleep(min(0.01, max(0.0, deadline - time.perf_counter())))
        return self.is_open

    def stop_and_flush(self, lines: list[str], timeout: float = 0.5) -> bool:
        self.cancel_pending()
        for line in lines:
            if not self.write_line(line, priority=True):
                return False
        return self.flush(timeout)

    def _mark_faulted(self) -> None:
        self._faulted.set()
        with self._transport_lock:
            transport, self._transport = self._transport, None
            stop = self._session_stop
            self._session_stop = None
        if stop is not None:
            stop.set()
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass

    def _read_loop(self, session_id, stop, transport, on_data, on_error) -> None:
        try:
            while not stop.is_set():
                if transport is None or not transport.is_open:
                    break
                data = transport.read(4096)
                if data:
                    on_data(data, time.perf_counter())
        except Exception as exc:
            if not stop.is_set():
                self._mark_faulted_for_session(session_id, transport, stop)
                on_error(f"{self.name}读取失败：{exc}")

    def _write_loop(self, session_id, stop, transport, write_queue, on_error) -> None:
        try:
            while not stop.is_set():
                try:
                    data = write_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                if data is None:
                    write_queue.task_done()
                    break
                if transport is None or not transport.is_open:
                    write_queue.task_done()
                    break
                payload, on_sent, on_written = data
                if on_sent is not None:
                    on_sent(time.perf_counter())
                transport.write(payload)
                if on_written is not None:
                    on_written(time.perf_counter())
                write_queue.task_done()
        except Exception as exc:
            if not stop.is_set():
                self._mark_faulted_for_session(session_id, transport, stop)
                on_error(f"{self.name}发送失败：{exc}")

    def _mark_faulted_for_session(self, session_id, transport, stop) -> None:
        with self._transport_lock:
            current = self._session_id == session_id and self._transport is transport
            if current:
                self._transport = None
                self._session_stop = None
        stop.set()
        if current:
            self._faulted.set()
            try:
                transport.close()
            except Exception:
                pass


def _open_transport(port: str, baudrate: int):
    try:
        import serial

        return _PySerialTransport(serial.Serial(port, baudrate, timeout=0.05, write_timeout=0.5))
    except ImportError:
        if sys.platform != "win32":
            raise RuntimeError("请安装 pyserial：python -m pip install pyserial")
        return _Win32SerialTransport(port, baudrate)


class _PySerialTransport:
    def __init__(self, serial_port) -> None:
        self.serial_port = serial_port

    @property
    def is_open(self) -> bool:
        return bool(self.serial_port and self.serial_port.is_open)

    def read(self, size: int) -> bytes:
        return self.serial_port.read(max(1, min(size, self.serial_port.in_waiting)))

    def write(self, data: bytes) -> None:
        count = self.serial_port.write(data)
        if count is not None and count != len(data):
            raise OSError(f"串口只写入 {count}/{len(data)} 字节")

    def close(self) -> None:
        if self.serial_port:
            self.serial_port.close()


if sys.platform == "win32":
    class COMSTAT(ctypes.Structure):
        _fields_ = [
            ("flags", wintypes.DWORD),
            ("cbInQue", wintypes.DWORD),
            ("cbOutQue", wintypes.DWORD),
        ]


    class DCB(ctypes.Structure):
        _fields_ = [
            ("DCBlength", wintypes.DWORD),
            ("BaudRate", wintypes.DWORD),
            ("flags", wintypes.DWORD),
            ("wReserved", wintypes.WORD),
            ("XonLim", wintypes.WORD),
            ("XoffLim", wintypes.WORD),
            ("ByteSize", wintypes.BYTE),
            ("Parity", wintypes.BYTE),
            ("StopBits", wintypes.BYTE),
            ("XonChar", ctypes.c_char),
            ("XoffChar", ctypes.c_char),
            ("ErrorChar", ctypes.c_char),
            ("EofChar", ctypes.c_char),
            ("EvtChar", ctypes.c_char),
            ("wReserved1", wintypes.WORD),
        ]


    class COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout", wintypes.DWORD),
            ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
            ("ReadTotalTimeoutConstant", wintypes.DWORD),
            ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
            ("WriteTotalTimeoutConstant", wintypes.DWORD),
        ]


class _Win32SerialTransport:
    GENERIC_READ = 0x80000000
    GENERIC_WRITE = 0x40000000
    OPEN_EXISTING = 3
    PURGE_TXABORT = 0x0001
    PURGE_RXABORT = 0x0002
    PURGE_TXCLEAR = 0x0004
    PURGE_RXCLEAR = 0x0008

    def __init__(self, port: str, baudrate: int) -> None:
        if sys.platform != "win32":
            raise RuntimeError("Win32 串口后端只能在 Windows 使用")
        self.kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self.kernel32.CreateFileW.restype = wintypes.HANDLE
        path = port if port.startswith("\\\\.\\") else "\\\\.\\" + port
        self.handle = self.kernel32.CreateFileW(
            path,
            self.GENERIC_READ | self.GENERIC_WRITE,
            0,
            None,
            self.OPEN_EXISTING,
            0,
            None,
        )
        invalid_handle = wintypes.HANDLE(-1).value
        if self.handle == invalid_handle:
            raise ctypes.WinError(ctypes.get_last_error())

        try:
            self.kernel32.SetupComm(self.handle, 65536, 65536)
            dcb = DCB()
            dcb.DCBlength = ctypes.sizeof(DCB)
            if not self.kernel32.GetCommState(self.handle, ctypes.byref(dcb)):
                raise ctypes.WinError(ctypes.get_last_error())
            settings = f"baud={baudrate} parity=N data=8 stop=1"
            if not self.kernel32.BuildCommDCBW(settings, ctypes.byref(dcb)):
                raise ctypes.WinError(ctypes.get_last_error())
            dcb.flags |= 0x00000001
            if not self.kernel32.SetCommState(self.handle, ctypes.byref(dcb)):
                raise ctypes.WinError(ctypes.get_last_error())
            timeouts = COMMTIMEOUTS(30, 0, 35, 0, 500)
            if not self.kernel32.SetCommTimeouts(self.handle, ctypes.byref(timeouts)):
                raise ctypes.WinError(ctypes.get_last_error())
            self.kernel32.PurgeComm(
                self.handle,
                self.PURGE_TXABORT | self.PURGE_RXABORT | self.PURGE_TXCLEAR | self.PURGE_RXCLEAR,
            )
        except Exception:
            self.close()
            raise

    @property
    def is_open(self) -> bool:
        return bool(self.handle)

    def read(self, size: int) -> bytes:
        if not self.handle:
            return b""
        errors = wintypes.DWORD()
        state = COMSTAT()
        if not self.kernel32.ClearCommError(self.handle, ctypes.byref(errors), ctypes.byref(state)):
            raise ctypes.WinError(ctypes.get_last_error())
        if errors.value:
            raise OSError(f"串口通信错误：{errors.value}")
        size = max(1, min(size, state.cbInQue))
        buffer = ctypes.create_string_buffer(size)
        count = wintypes.DWORD()
        ok = self.kernel32.ReadFile(self.handle, buffer, size, ctypes.byref(count), None)
        if not ok:
            error = ctypes.get_last_error()
            if error in (995, 6):
                return b""
            raise ctypes.WinError(error)
        return buffer.raw[: count.value]

    def write(self, data: bytes) -> None:
        if not self.handle:
            raise RuntimeError("串口已关闭")
        count = wintypes.DWORD()
        buffer = ctypes.create_string_buffer(data)
        ok = self.kernel32.WriteFile(self.handle, buffer, len(data), ctypes.byref(count), None)
        if not ok:
            raise ctypes.WinError(ctypes.get_last_error())
        if count.value != len(data):
            raise OSError(f"串口只写入 {count.value}/{len(data)} 字节")

    def close(self) -> None:
        handle, self.handle = getattr(self, "handle", None), None
        if handle:
            self.kernel32.CancelIo(handle)
            self.kernel32.CloseHandle(handle)
