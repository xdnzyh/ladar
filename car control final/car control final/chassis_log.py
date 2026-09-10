from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import threading


class ChassisTrafficLogger:
    def __init__(self, path: str | Path, *, enabled: bool = True) -> None:
        self.path = Path(path)
        self.enabled = bool(enabled)
        self._lock = threading.Lock()

    def record(
        self,
        direction: str,
        payload: bytes | bytearray | memoryview,
        host_time_s: float,
        *,
        connection_generation: int,
        action_id: int | None = None,
    ) -> None:
        if not self.enabled:
            return
        if direction not in {"RX", "TX"}:
            raise ValueError("串口日志方向必须是 RX 或 TX")
        raw = bytes(payload)
        item = {
            "wall_time": datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds"),
            "host_monotonic_s": round(float(host_time_s), 9),
            "direction": direction,
            "connection_generation": int(connection_generation),
            "action_id": None if action_id is None else int(action_id),
            "length": len(raw),
            "hex": raw.hex(" ").upper(),
            "ascii": raw.decode("ascii", "backslashreplace"),
        }
        encoded = json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n"
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8", newline="") as stream:
                stream.write(encoded)

