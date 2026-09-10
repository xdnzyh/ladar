from __future__ import annotations

import ctypes
from collections import deque
from ctypes import wintypes
from dataclasses import dataclass, field
import sys
import threading
import time
from typing import Callable


DataCallback = Callable[[bytes, float], None]
ErrorCallback = Callable[[str], None]


class PartialWriteError(OSError):
    def __init__(self, written: int, total: int) -> None:
        super().__init__(f"串口只写入 {written}/{total} 字节")
        self.written = int(written)
        self.total = int(total)


@dataclass
class SerialWriteTicket:
    write_id: int
    session_id: int
    payload: bytes
    on_sent: Callable[[float], None] | None = None
    on_written: Callable[[float], None] | None = None
    on_failed: Callable[[float, int, int, str], None] | None = None
    on_cancelled: Callable[[float], None] | None = None
    state: str = "queued"
    bytes_written: int = 0


@dataclass
class _WriteSession:
    session_id: int
    stop: threading.Event
    transport: object
    condition: threading.Condition = field(default_factory=threading.Condition)
    pending: deque[SerialWriteTicket] = field(default_factory=deque)
    active: SerialWriteTicket | None = None
    closing: bool = False


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
        self._session_stop: threading.Event | None = None
        self._write_session: _WriteSession | None = None
        self._session_id = 0
        self._write_id = 0
        self._active_on_data = on_data
        self._active_on_error = on_error

    @property
    def is_open(self) -> bool:
        with self._transport_lock:
            transport = self._transport
            session = self._write_session
        return (
            not self._faulted.is_set()
            and transport is not None
            and session is not None
            and not session.stop.is_set()
            and bool(transport.is_open)
        )

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
        with self._transport_lock:
            self._session_id += 1
            session_id = self._session_id
            session = _WriteSession(session_id, stop, transport)
            self._session_stop = stop
            self._write_session = session
            self._active_on_data = on_data or self.on_data
            self._active_on_error = on_error or self.on_error
            self._transport = transport
        self._reader_thread = threading.Thread(
            target=self._read_loop,
            args=(session_id, stop, transport, self._active_on_data, self._active_on_error),
            name=f"{self.name}-reader",
            daemon=True,
        )
        self._writer_thread = threading.Thread(
            target=self._write_loop,
            args=(session, self._active_on_error),
            name=f"{self.name}-writer",
            daemon=True,
        )
        self._reader_thread.start()
        self._writer_thread.start()

    def close(self) -> None:
        with self._transport_lock:
            transport, self._transport = self._transport, None
            stop, self._session_stop = self._session_stop, None
            session, self._write_session = self._write_session, None
            self._session_id += 1
        cancelled: list[SerialWriteTicket] = []
        if stop is not None:
            stop.set()
        if session is not None:
            with session.condition:
                session.closing = True
                while session.pending:
                    ticket = session.pending.popleft()
                    if ticket.state == "queued":
                        ticket.state = "cancelled"
                        cancelled.append(ticket)
                session.condition.notify_all()
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass
        stamp = time.perf_counter()
        for ticket in cancelled:
            self._invoke(ticket.on_cancelled, stamp)
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
        *,
        on_failed: Callable[[float, int, int, str], None] | None = None,
        on_cancelled: Callable[[float], None] | None = None,
        discard_pending: bool = False,
    ) -> bool:
        return self.write_ticket(
            data,
            on_sent=on_sent,
            on_written=on_written,
            priority=priority,
            on_failed=on_failed,
            on_cancelled=on_cancelled,
            discard_pending=discard_pending,
        ) is not None

    def write_ticket(
        self,
        data: bytes | str,
        on_sent: Callable[[float], None] | None = None,
        on_written: Callable[[float], None] | None = None,
        priority: bool = False,
        *,
        on_failed: Callable[[float, int, int, str], None] | None = None,
        on_cancelled: Callable[[float], None] | None = None,
        discard_pending: bool = False,
    ) -> SerialWriteTicket | None:
        if isinstance(data, str):
            data = data.encode("ascii")
        payload = bytes(data)
        cancelled: list[SerialWriteTicket] = []
        with self._transport_lock:
            transport = self._transport
            session = self._write_session
            if (
                transport is None
                or session is None
                or session.stop.is_set()
                or not transport.is_open
            ):
                return None
            self._write_id += 1
            ticket = SerialWriteTicket(
                self._write_id,
                session.session_id,
                payload,
                on_sent,
                on_written,
                on_failed,
                on_cancelled,
            )
            with session.condition:
                if session.closing or session.stop.is_set():
                    return None
                if discard_pending:
                    while session.pending:
                        old = session.pending.popleft()
                        if old.state == "queued":
                            old.state = "cancelled"
                            cancelled.append(old)
                if priority:
                    session.pending.appendleft(ticket)
                else:
                    session.pending.append(ticket)
                session.condition.notify()
        stamp = time.perf_counter()
        for old in cancelled:
            self._invoke(old.on_cancelled, stamp)
        return ticket

    def write_line(
        self,
        text: str,
        on_sent: Callable[[float], None] | None = None,
        on_written: Callable[[float], None] | None = None,
        priority: bool = False,
        **kwargs,
    ) -> bool:
        return self.write(
            text.rstrip("\r\n") + "\r\n",
            on_sent,
            on_written,
            priority,
            **kwargs,
        )

    def cancel_pending(self) -> int:
        with self._transport_lock:
            session = self._write_session
        if session is None:
            return 0
        cancelled: list[SerialWriteTicket] = []
        with session.condition:
            while session.pending:
                ticket = session.pending.popleft()
                if ticket.state == "queued":
                    ticket.state = "cancelled"
                    cancelled.append(ticket)
            session.condition.notify_all()
        stamp = time.perf_counter()
        for ticket in cancelled:
            self._invoke(ticket.on_cancelled, stamp)
        return len(cancelled)

    def cancel_write(self, ticket: SerialWriteTicket | int) -> str:
        write_id = ticket.write_id if isinstance(ticket, SerialWriteTicket) else int(ticket)
        with self._transport_lock:
            session = self._write_session
        if session is None:
            return "not_found"
        cancelled: SerialWriteTicket | None = None
        with session.condition:
            if session.active is not None and session.active.write_id == write_id:
                return "started"
            for item in tuple(session.pending):
                if item.write_id == write_id and item.state == "queued":
                    session.pending.remove(item)
                    item.state = "cancelled"
                    cancelled = item
                    break
        if cancelled is not None:
            self._invoke(cancelled.on_cancelled, time.perf_counter())
            return "cancelled"
        return "not_found"

    def flush(self, timeout: float = 0.5) -> bool:
        if timeout < 0:
            raise ValueError("发送等待时间不能为负数")
        with self._transport_lock:
            session = self._write_session
        if session is None:
            return False
        deadline = time.perf_counter() + timeout
        with session.condition:
            while session.pending or session.active is not None:
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    return False
                session.condition.wait(min(0.05, remaining))
        return self.is_open

    def stop_and_flush(self, lines: list[str], timeout: float = 0.5) -> bool:
        payload = "".join(line.rstrip("\r\n") + "\r\n" for line in lines)
        if not self.write(payload, priority=True, discard_pending=True):
            return False
        return self.flush(timeout)

    def _mark_faulted(self) -> None:
        with self._transport_lock:
            session = self._write_session
        if session is not None:
            self._mark_faulted_for_session(session)

    def _read_loop(self, session_id, stop, transport, on_data, on_error) -> None:
        try:
            while not stop.is_set():
                if transport is None or not transport.is_open:
                    break
                data = transport.read(4096)
                if data and not stop.is_set() and self._session_is_current(session_id, transport):
                    on_data(data, time.perf_counter())
        except Exception as exc:
            if not stop.is_set():
                current = self._mark_faulted_for_session(session_id, transport, stop)
                if current:
                    self._invoke(on_error, f"{self.name}读取失败：{exc}")

    def _write_loop(self, session: _WriteSession, on_error) -> None:
        while True:
            with session.condition:
                while not session.pending and not session.stop.is_set():
                    session.condition.wait(0.1)
                if session.stop.is_set():
                    break
                ticket = session.pending.popleft()
                if ticket.state != "queued":
                    continue
                ticket.state = "started"
                session.active = ticket
            self._invoke(ticket.on_sent, time.perf_counter())
            failure: Exception | None = None
            try:
                count = session.transport.write(ticket.payload)
                written = len(ticket.payload) if count is None else int(count)
                ticket.bytes_written = max(0, written)
                if written != len(ticket.payload):
                    raise PartialWriteError(written, len(ticket.payload))
            except Exception as exc:
                failure = exc
                if isinstance(exc, PartialWriteError):
                    ticket.bytes_written = exc.written
            current = self._session_is_current(session.session_id, session.transport)
            if failure is None:
                ticket.state = "completed"
                if current and not session.stop.is_set():
                    self._invoke(ticket.on_written, time.perf_counter())
            else:
                ticket.state = "failed"
                stamp = time.perf_counter()
                if current and not session.stop.is_set():
                    self._invoke(
                        ticket.on_failed,
                        stamp,
                        ticket.bytes_written,
                        len(ticket.payload),
                        str(failure),
                    )
            with session.condition:
                if session.active is ticket:
                    session.active = None
                session.condition.notify_all()
            if failure is not None and current:
                current = self._mark_faulted_for_session(session)
                if current:
                    self._invoke(on_error, f"{self.name}发送失败：{failure}")
                break

    def _session_is_current(self, session_id: int, transport: object) -> bool:
        with self._transport_lock:
            return bool(
                self._session_id == session_id
                and self._transport is transport
                and self._write_session is not None
                and self._write_session.session_id == session_id
            )

    def _mark_faulted_for_session(self, session_or_id, transport=None, stop=None) -> bool:
        if isinstance(session_or_id, _WriteSession):
            session = session_or_id
        else:
            with self._transport_lock:
                session = self._write_session
            if session is None or session.session_id != session_or_id or session.transport is not transport:
                if stop is not None:
                    stop.set()
                return False
        cancelled: list[SerialWriteTicket] = []
        with self._transport_lock:
            current = self._session_id == session.session_id and self._transport is session.transport
            if current:
                self._transport = None
                self._session_stop = None
                self._write_session = None
                self._faulted.set()
        session.stop.set()
        with session.condition:
            session.closing = True
            while session.pending:
                ticket = session.pending.popleft()
                if ticket.state == "queued":
                    ticket.state = "cancelled"
                    cancelled.append(ticket)
            session.condition.notify_all()
        stamp = time.perf_counter()
        for ticket in cancelled:
            self._invoke(ticket.on_cancelled, stamp)
        if current:
            try:
                session.transport.close()
            except Exception:
                pass
        return current

    @staticmethod
    def _invoke(callback, *args) -> None:
        if callback is None:
            return
        try:
            callback(*args)
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

    def write(self, data: bytes) -> int:
        count = self.serial_port.write(data)
        return len(data) if count is None else int(count)

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

    def write(self, data: bytes) -> int:
        if not self.handle:
            raise RuntimeError("串口已关闭")
        count = wintypes.DWORD()
        buffer = ctypes.create_string_buffer(data)
        ok = self.kernel32.WriteFile(self.handle, buffer, len(data), ctypes.byref(count), None)
        if not ok:
            raise ctypes.WinError(ctypes.get_last_error())
        return int(count.value)

    def close(self) -> None:
        handle, self.handle = getattr(self, "handle", None), None
        if handle:
            self.kernel32.CancelIo(handle)
            self.kernel32.CloseHandle(handle)
