"""Bounded real navigation trial via application API, without UI automation."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import json
import logging
import time
import tkinter as tk
import hardware_app
from navigation_app import NavigationApp

out = Path('tmp') / ('navigation_trial_' + time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
logging.basicConfig(filename=out / 'runtime.log', level=logging.INFO,
                    format='%(asctime)s %(message)s', encoding='utf-8')
root = tk.Tk()
root.withdraw()
app = NavigationApp(root, source='hardware', initial_view='navigation')
app.measure_port_var.set('COM8')
app.rotation_port_var.set('COM9')
app.chassis_port_var.set('COM11')
events = (out / 'states.jsonl').open('w', encoding='utf-8')
started = None
deadline = time.perf_counter() + 15
closing = False

def finish():
    global closing
    if not closing:
        closing = True
        print('STOPPING', flush=True)
        app.on_close()

def tick():
    global started
    try:
        now = time.perf_counter()
        nav = app.navigator
        row = dict(time=now, state=nav.state, detail=nav.detail,
                   pose=vars(nav.pose), running=app.running, moving=app.moving,
                   chassis=app.chassis_controller.state,
                   scans=getattr(nav, 'completed_scans', None))
        events.write(json.dumps(row, ensure_ascii=False, default=str)+'\n')
        events.flush()
        print(json.dumps(row, ensure_ascii=True, default=str), flush=True)
        if started is None:
            if app.chassis_controller.state == 'connected_waiting':
                app.chassis_controller.request_status()
            if app._navigation_preflight(show=False):
                app.start()
                started = now
            elif now >= deadline:
                finish()
        elif now-started >= 45 or not app.running:
            finish()
        if not closing:
            root.after(2000, tick)
    except Exception:
        logging.exception('trial failed')
        finish()

def callback_error(*args):
    logging.error('callback failed', exc_info=args)
    finish()

root.report_callback_exception = callback_error
print('OUTPUT', out, flush=True)
try:
    app.connect()
    root.after(1000, tick)
    root.mainloop()
finally:
    if not app._closed:
        app.stop()
    events.close()
print('CLOSED', flush=True)
