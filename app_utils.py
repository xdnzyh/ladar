from __future__ import annotations

from copy import deepcopy
import ctypes
import json
from pathlib import Path
import sys


def enable_windows_dpi_awareness() -> None:
    if sys.platform != "win32":
        return
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except (AttributeError, OSError):
            pass


def load_json_config(path: Path, defaults: dict) -> dict:
    config = deepcopy(defaults)
    if not path.exists():
        return config
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return config
    if not isinstance(loaded, dict):
        return config
    config.update(loaded)
    return config


def save_json(path: Path, data: dict, *, trailing_newline: bool = False) -> None:
    text = json.dumps(data, ensure_ascii=False, indent=2)
    if trailing_newline:
        text += "\n"
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def capture_window(root, path: Path, delay_ms: int, cleanup=None) -> None:
    def capture() -> None:
        root.deiconify()
        root.lift()
        root.attributes("-topmost", True)
        root.update_idletasks()
        root.update()
        try:
            from PIL import ImageGrab

            x, y = root.winfo_rootx(), root.winfo_rooty()
            width, height = root.winfo_width(), root.winfo_height()
            path.parent.mkdir(parents=True, exist_ok=True)
            ImageGrab.grab(
                bbox=(x, y, x + width, y + height),
                all_screens=True,
            ).save(path)
        finally:
            root.attributes("-topmost", False)
            if cleanup is not None:
                try:
                    cleanup()
                except Exception:
                    pass
            else:
                root.after(80, root.destroy)

    root.after(max(200, delay_ms), capture)
