"""Explicitly invoked, bounded real-device navigation capture using the normal UI."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import argparse
from dataclasses import asdict, is_dataclass
import json
import threading
import time
import tkinter as tk
from navigation_app import NavigationApp, messagebox
from runtime_diagnostics import RuntimeDiagnostics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=300)
    args = parser.parse_args()
    if not 0 < args.seconds <= 300:
        parser.error('Duration must be between 0 and 300 seconds')
    out = Path('logs') / ('hardware_capture_' + time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True)
    stream = (out / 'events.jsonl').open('w', encoding='utf-8')
    lock = threading.Lock()
    def encode(value):
        if is_dataclass(value):
            return asdict(value)
        return str(value)
    def record(kind, **data):
        with lock:
            stream.write(json.dumps(dict(at=time.time(), kind=kind, **data), default=encode,
                                    ensure_ascii=False) + '\n')
            stream.flush()
    class CaptureApp(NavigationApp):
        def _receive_sync_event(self, kind, value, timestamp):
            if kind in ('sync_sweep', 'sync_local_sweep', 'sync_error', 'sync_diagnostic'):
                record(kind, value=value, device_time=timestamp)
            super()._receive_sync_event(kind, value, timestamp)
        def _mapping_runtime_result(self, result):
            snap = result.snapshot
            record('mapping', sequence=result.request.scan_sequence, points=result.request.points,
                   command=result.command, error=result.error,
                   state=snap.state if snap else None, detail=snap.detail if snap else None,
                   pose=snap.pose if snap else None, match=snap.match_score if snap else None,
                   occupied=len(snap.grid.occupied_cells()) if snap else None,
                   map_version=snap.map_version if snap else None)
            super()._mapping_runtime_result(result)
        def _execute_navigation_command(self, command):
            record('planned_action', command=command)
            super()._execute_navigation_command(command)
        def _safety_stop(self, reason):
            record('safety_stop', reason=reason)
            super()._safety_stop(reason)
        def _log(self, message):
            record('app_log', message=message)
            super()._log(message)
    # Report errors without a modal dialog blocking the bounded stop timer.
    for name in ('showwarning', 'showerror'):
        setattr(messagebox, name, lambda title, message, **kw: record('dialog', title=title, message=message))
    root = tk.Tk()
    diagnostics = RuntimeDiagnostics(root, out)
    app = CaptureApp(root, source='hardware', initial_view='navigation')
    root.title('TriScan 实车 5 分钟记录 · 可随时紧急停止')
    record('config', config=app.config)
    started = None
    connected_at = time.monotonic()
    ending = False
    def finish(reason):
        nonlocal ending
        if ending:
            return
        ending = True
        record('test_end', reason=reason, elapsed=None if started is None else time.monotonic()-started)
        app.stop()
        app.on_close()
    def tick():
        nonlocal started
        if app._closed or ending:
            return
        now = time.monotonic()
        if started is None:
            if app.connected and app._navigation_preflight(show=False):
                app.start()
                if app.running:
                    started = now
                    record('test_start', duration=args.seconds)
            elif now-connected_at > 30:
                finish('connection_or_preflight_timeout')
                return
        else:
            record('status', elapsed=now-started, running=app.running, moving=app.moving,
                   state=app.navigator.state, detail=app.navigator.detail,
                   chassis=str(app.chassis_controller.state))
            if now-started >= args.seconds:
                finish('duration_complete')
                return
            if not app.running:
                finish('navigation_stopped')
                return
        root.after(1000, tick)
    print(str(out.resolve()), flush=True)
    root.after(200, app.connect)
    root.after(1200, tick)
    try:
        root.mainloop()
    finally:
        diagnostics.close()
        record('capture_closed')
        stream.close()


if __name__ == '__main__':
    main()
