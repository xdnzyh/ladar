import micropython
import utime
from machine import Pin, UART

micropython.alloc_emergency_exception_buf(200)

relay = Pin(13, Pin.OUT, value=0)
sensor = Pin(27, Pin.IN, Pin.PULL_UP)
aux = Pin(26, Pin.IN, Pin.PULL_UP)
md0 = Pin(33, Pin.OUT, value=0)
md1 = Pin(32, Pin.OUT, value=0)
lora = UART(2, baudrate=115200, bits=8, parity=None, stop=1,
            tx=Pin(17), rx=Pin(16), timeout=10)

count_total = 0
last_irq_us = 0
motor_start_us = 0
pending_count = 0
pending_tick_us = 0
DEBOUNCE_US = 8000
MOTOR_ON_IGNORE_US = 300000


def motor_on():
    global motor_start_us
    relay.value(1)
    motor_start_us = utime.ticks_us()


def motor_off():
    relay.value(0)


def send(text):
    t0 = utime.ticks_ms()
    while aux.value() == 0 and utime.ticks_diff(utime.ticks_ms(), t0) < 200:
        utime.sleep_ms(2)
    try:
        lora.write((str(text).rstrip("\r\n") + "\r\n").encode())
    except Exception as exc:
        print("LoRa TX error:", exc)


def send_trigger(_):
    global pending_count, pending_tick_us
    send("TRIG {} TICK_US={}".format(pending_count, pending_tick_us))


def sensor_irq(_):
    global count_total, last_irq_us, pending_count, pending_tick_us
    now = utime.ticks_us()
    if utime.ticks_diff(now, motor_start_us) < MOTOR_ON_IGNORE_US:
        return
    if utime.ticks_diff(now, last_irq_us) < DEBOUNCE_US:
        return
    last_irq_us = now
    count_total += 1
    pending_count = count_total
    pending_tick_us = now
    micropython.schedule(send_trigger, 0)


sensor.irq(trigger=Pin.IRQ_FALLING, handler=sensor_irq)
rx_buf = bytearray()


def handle_cmd(raw):
    global count_total, rx_buf
    cmd = raw.decode("utf-8", "ignore").strip().upper()
    if cmd == "PING":
        send("PONG")
    elif cmd == "ON":
        motor_on(); send("OK ON")
    elif cmd == "OFF":
        motor_off(); send("OK OFF")
    elif cmd == "RESETCNT":
        count_total = 0; send("OK RESETCNT")
    elif cmd == "STATUS":
        send("STATUS MOTOR={} TOTAL={}".format(relay.value(), count_total))
    elif cmd == "ROT 1":
        count_total = 0; motor_on(); send("OK ROT_PRECISE SET=1")
    else:
        send("ERR UNKNOWN: {}".format(cmd))


def poll():
    global rx_buf
    n = lora.any()
    if n:
        rx_buf.extend(lora.read(n) or b"")
    while b"\n" in rx_buf or b"\r" in rx_buf:
        positions = [p for p in (rx_buf.find(b"\r"), rx_buf.find(b"\n")) if p >= 0]
        pos = min(positions)
        line = bytes(rx_buf[:pos]); del rx_buf[:pos + 1]
        while rx_buf and rx_buf[0] in (10, 13):
            del rx_buf[0]
        if line.strip():
            handle_cmd(line)


motor_off()
utime.sleep_ms(300)
send("READY")
while True:
    poll()
    utime.sleep_ms(2)
