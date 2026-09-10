"""Bounded on-device diagnostic; always stops laser and rotation on exit."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import json, time, threading, collections, argparse
from serial_backend import SerialEndpoint
from synchronized_acquisition import SynchronizedAcquisition
from sync_resilience import ResilientSynchronizedAcquisition
from radar_core import CalibrationModel
from runtime_config import RUNTIME_DEFAULTS

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=40)
p.add_argument('--rate', type=float, default=100)
p.add_argument('--status-only', action='store_true')
p.add_argument('--link-test', action='store_true')
a = p.parse_args()
out = Path('tmp') / ('radar_diagnostic_' + time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
log = (out / 'raw.jsonl').open('w', encoding='utf-8')
lock = threading.Lock()
counts = collections.Counter()
sweeps = []
acq = None
def record(**item):
    with lock:
        log.write(json.dumps(dict(host=time.perf_counter(), **item), ensure_ascii=False, default=str)+'\n')
        log.flush()
def rx(source, data, at):
    record(kind='rx', source=source, hex=data.hex(), text=data.decode('ascii', 'backslashreplace'))
    if acq is not None:
        acq.feed(source, data, at)
class LoggedEndpoint(SerialEndpoint):
    def write_line(self, line, *args, **kwargs):
        record(kind='tx', source=self.name, text=line)
        return super().write_line(line, *args, **kwargs)
    def stop_and_flush(self, lines, timeout=.5):
        record(kind='stop_tx', source=self.name, lines=lines)
        return super().stop_and_flush(lines, timeout)
def event(kind, value, at):
    counts[kind] += 1
    if kind in ('sync_sweep', 'sync_local_sweep'):
        session, seq, points, period = value
        item = dict(kind=kind, sequence=seq, count=len(points), period=period, points=[vars(x) for x in points])
        sweeps.append(item)
        record(**item)
        print(kind, seq, len(points), period, flush=True)
    elif kind not in ('sync_range', 'sync_observation', 'sync_period'):
        record(kind=kind, value=value)
        print(kind, value, flush=True)
ends = {}
try:
    for source, port in [('measurement','COM6'), ('rotation','COM7')]:
        ep = LoggedEndpoint(source, lambda data, at, s=source: rx(s, data, at), lambda err, s=source: record(kind='serial_error', source=s, error=err))
        ep.open(port, 115200)
        ends[source] = ep
    for source, ep in ends.items():
        ep.write_line('STATUS')
        record(kind='tx', source=source, text='STATUS')
    time.sleep(2)
    if a.link_test:
        for source, ep in ends.items():
            ep.stop_and_flush(['STOP', 'LASER 0'] if source == 'measurement' else ['OFF'], .5)
        time.sleep(1)
        for source in ['measurement', 'rotation']:
            for i in range(10):
                command = 'SYNC diag' + str(i)
                record(kind='tx', source=source, text=command)
                ends[source].write_line(command)
                time.sleep(.7)
            record(kind='tx', source=source, text='STATUS')
            ends[source].write_line('STATUS')
            time.sleep(2)
    elif not a.status_only:
        config = {**RUNTIME_DEFAULTS, **json.loads(Path('navigation_config.json').read_text(encoding='utf-8')), 'hardware_sample_rate_hz': a.rate, 'sample_rate_hz': a.rate}
        config.update(runtime_source='hardware', runtime_view='radar')
        (out / 'config.json').write_text(json.dumps(config, indent=2), encoding='utf-8')
        cal = CalibrationModel.from_csv(config['calibration_file'])
        acq = ResilientSynchronizedAcquisition(ends['measurement'], ends['rotation'], cal, config, event)
        acq.start()
        until = time.perf_counter() + a.seconds
        while time.perf_counter() < until:
            acq.poll()
            if acq.builder.last_quality_counts and getattr(acq, '_diag_anchor', None) != acq.builder.anchor:
                acq._diag_anchor = acq.builder.anchor
                record(kind='closed_quality', anchor=acq.builder.anchor, counts=acq.builder.last_quality_counts,
                       display=[vars(x) for x in acq.builder.last_display_points])
            if acq.state == 'stopped':
                break
            time.sleep(.005)
        record(kind='summary', state=acq.state, counts=dict(counts), ignored=acq.ignored_counts, sync_stats=acq.sync_stats)
finally:
    if acq is not None:
        acq.stop(force=True)
    else:
        for source, ep in ends.items():
            ep.stop_and_flush(['STOP','LASER 0'] if source == 'measurement' else ['OFF'], .5)
    for ep in ends.values():
        ep.write_line('STATUS')
    time.sleep(1)
    for ep in ends.values():
        ep.close()
    log.close()
print('ARTIFACT', out, flush=True)
if not a.status_only and not a.link_test:
    good = [s for s in sweeps if s['kind'] == 'sync_sweep']
    failures = []
    if not good: failures.append('no formal complete scans')
    if any(s['count'] < 50 for s in good): failures.append('scan below 50 points')
    if any(s['count'] < 50 for s in sweeps if s['kind']=='sync_local_sweep'):
        failures.append('local scan below 50 points')
    if acq and any(d.get('跨端口数据',0) or d.get('外来协议消息',0) for d in acq.ignored_counts.values()):
        failures.append('foreign or cross-port traffic observed')
    raw = (out/'raw.jsonl').read_text(encoding='utf-8')
    if 'ERROR' in raw: failures.append('device ERROR observed (inspect raw context)')
    print('VERDICT', failures or 'PASS', dict(counts), flush=True)
    sys.exit(bool(failures))
