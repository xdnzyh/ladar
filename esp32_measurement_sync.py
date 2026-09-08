# 历史兼容脚本：当前设备基准见 firmware_baseline/measurement_main_MEASUREMENT_SYNC_CAL_V3.py。
# 本文件仅保留供旧实验回放使用，不得作为当前设备的默认烧录对象。

import utime
from machine import Pin, UART


class ExtendedClock:
    def __init__(self):
        self.raw = utime.ticks_us()
        self.total = 0

    def now(self):
        raw = utime.ticks_us()
        self.total += utime.ticks_diff(raw, self.raw)
        self.raw = raw
        return self.total


clock = ExtendedClock()
laser = Pin(25, Pin.OUT, value=0)
aux = Pin(18, Pin.IN, Pin.PULL_UP)
ccd = UART(2, baudrate=256000, tx=17, rx=16, timeout=0, txbuf=128, rxbuf=512)
lora = UART(1, baudrate=115200, tx=4, rx=5, timeout=0, txbuf=256, rxbuf=512)
rx_buf = bytearray()
ccd_buf = bytearray()
tx_queue = []
session = ''
single_capture = False
legacy_passthrough = False
sequence = 0
period_us = 50000
next_sample = 0
capture_begin = None
center_mode = 'fffe'
last_command = 0


def stop():
    global session, capture_begin, single_capture, legacy_passthrough
    session = ''
    single_capture = False
    legacy_passthrough = False
    capture_begin = None
    ccd_buf.clear()
    laser.value(0)


def enqueue(message):
    if len(tx_queue) >= 32:
        stop()
        tx_queue.clear()
        tx_queue.append('ERROR TX_OVERFLOW')
    else:
        tx_queue.append(message)


def handle(line, received_at):
    global session, sequence, period_us, next_sample, center_mode, last_command
    global single_capture, legacy_passthrough
    text = line.decode('ascii', 'ignore').strip()
    parts = text.split()
    if not parts:
        return
    command = parts[0].upper()
    last_command = received_at
    if command == 'STOP':
        stop()
        tx_queue.clear()
        enqueue('OK STOP')
    elif command == 'LASER' and len(parts) == 2:
        laser.value(1 if parts[1] == '1' else 0)
        enqueue('OK LASER ' + parts[1])
    elif command == 'PING':
        enqueue('PONG')
    elif command == 'SYNC' and len(parts) == 2:
        enqueue(('SYNC', parts[1], received_at))
    elif command == 'CAL' and len(parts) in (3, 4):
        token = parts[1] if len(parts) == 4 else 'CAL'
        handle(('START {} 1 {} {}'.format(token, parts[-2], parts[-1])).encode(), received_at)
        single_capture = bool(session)
    elif command == 'START' and len(parts) == 5:
        stop()
        try:
            rate, exposure = float(parts[2]), int(parts[3])
            if not 1 <= rate <= 100 or not 0 <= exposure <= 13 or parts[4] not in ('fffe', 'raw2'):
                raise ValueError
        except ValueError:
            enqueue('ERROR START_ARGUMENTS')
            return
        if ccd.any():
            ccd.read(ccd.any())
        tx_queue.clear()
        session, center_mode = parts[1], parts[4]
        sequence = 0
        period_us = int(1000000 / rate)
        ccd.write(('@c{:04d}#@'.format(exposure)).encode())
        next_sample = clock.now() + 200000
        laser.value(1)
        enqueue('OK START ' + session)
    elif text.startswith('@c') and not session:
        legacy_passthrough = True
        ccd.write(text.encode())
    else:
        enqueue('ERROR COMMAND')


def poll():
    global capture_begin, next_sample, sequence, session
    now = clock.now()
    data = lora.read(lora.any()) if lora.any() else None
    if data:
        rx_buf.extend(data)
        if len(rx_buf) > 512:
            rx_buf.clear()
            stop()
            enqueue('ERROR RX_OVERFLOW')
        while b'\n' in rx_buf:
            pos = rx_buf.find(b'\n')
            line = bytes(rx_buf[:pos])
            del rx_buf[:pos + 1]
            handle(line, clock.now())
    data = ccd.read(ccd.any()) if ccd.any() else None
    end = clock.now()
    # Check the interval before accepting a complete frame; otherwise a late
    # reply clears capture_begin and bypasses the timeout below.
    if capture_begin is not None and end - capture_begin > 250000:
        stop()
        enqueue('ERROR CCD_TIMEOUT')
        data = None
    if data:
        if not session:
            if legacy_passthrough:
                enqueue(data)
        elif capture_begin is not None:
            ccd_buf.extend(data)
            if center_mode == 'fffe':
                index = ccd_buf.find(b'\xff\xfe')
                if index >= 0:
                    del ccd_buf[:index]
                elif len(ccd_buf) > 1:
                    del ccd_buf[:-1]
            size = 4 if center_mode == 'fffe' else 2
            if len(ccd_buf) >= size:
                pixel = (ccd_buf[size - 2] << 8) | ccd_buf[size - 1]
                sequence += 1
                enqueue('PIX {} {} {} {} {}'.format(session, sequence, capture_begin, end, pixel if pixel <= 1499 else -1))
                ccd_buf.clear()
                capture_begin = None
                if single_capture:
                    stop()
    now = clock.now()
    if session and now - last_command > 40000000:
        stop()
        enqueue('ERROR WATCHDOG')
    if session and capture_begin is None and now >= next_sample and not tx_queue and aux.value():
        command = b'@c0071#@' if center_mode == 'fffe' else b'@c0081#@'
        capture_begin = clock.now()
        ccd.write(command)
        next_sample = capture_begin + period_us
    if tx_queue and aux.value():
        message = tx_queue.pop(0)
        if isinstance(message, tuple):
            message = 'SYNC {} {} {}'.format(message[1], message[2], clock.now())
        packet = message if isinstance(message, bytes) else (message + '\r\n').encode()
        if lora.write(packet) != len(packet):
            stop()
            tx_queue.clear()
            enqueue('ERROR UART_WRITE')


def main():
    enqueue('READY MEASUREMENT_SYNC_V2')
    try:
        while True:
            poll()
            utime.sleep_ms(1)
    finally:
        stop()


if __name__ == '__main__':
    main()
