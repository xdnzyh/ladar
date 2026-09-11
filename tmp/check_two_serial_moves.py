"""Two small protected moves; no retries of MOVE, always STOP on exit."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import json
import time
import serial
from chassis_protocol import encode_ping, encode_move, encode_result_query, ChassisStreamParser, STOP_SEQUENCE

out = Path('tmp') / ('two_moves_' + time.strftime('%Y%m%d_%H%M%S') + '.jsonl')
log = out.open('w', encoding='utf-8')
parser = ChassisStreamParser()
s = serial.Serial(port=None, baudrate=9600, timeout=.05, write_timeout=1)
s.dtr = False
s.rts = False
s.port = 'COM11'
s.open()

def record(direction, data):
    log.write(json.dumps(dict(time=time.time(), direction=direction, text=data.decode('ascii', 'backslashreplace'))) + '\n')
    log.flush()

def send(data):
    record('TX', data)
    s.write(data)

def wait_for(predicate, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        data = s.read(2048)
        if data:
            record('RX', data)
            for frame in parser.feed(data):
                if predicate(frame):
                    return frame
    return None

print('LOG', out, flush=True)
try:
    send(b'P\r\n')
    if not wait_for(lambda f: f.kind == 'idle', 3):
        raise RuntimeError('No IDLE')
    for index in range(2):
        nonce = f'{int(time.time()*1000) & 0xffffffff:08X}'
        send(b' '*32 + encode_ping(nonce))
        if not wait_for(lambda f: f.kind == 'pong' and f.value.nonce == nonce, 3):
            raise RuntimeError('No matching PONG')
        time.sleep(.35)
        send(b' '*32 + encode_move('W', 100, 'CNT', nonce=nonce))
        match = lambda f: f.kind == 'result' and f.value.nonce == nonce and f.value.report is not None
        response = wait_for(match, 3)
        if response is None:
            parser.finalize()
            send(b' '*32 + encode_result_query(nonce))
            response = wait_for(match, 3)
        if response is None:
            raise RuntimeError('No complete result; stop without another move')
        report = response.value.report
        print('MOVE', index+1, report.reason, 'ENC', report.enc, 'WHEELS', report.wheels, flush=True)
        if report.reason != 'TARGET':
            raise RuntimeError('Move did not complete normally')
        time.sleep(.5)
        send(b'P\r\n')
        if not wait_for(lambda f: f.kind == 'idle', 3):
            raise RuntimeError('No IDLE after move')
finally:
    send(STOP_SEQUENCE)
    time.sleep(.5)
    send(b'P\r\n')
    print('FINAL_IDLE', bool(wait_for(lambda f: f.kind == 'idle', 3)), flush=True)
    s.close()
    log.close()
