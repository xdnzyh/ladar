# rotation_main_v3.py
# ESP32 / MicroPython
# TriScan rotation node firmware
#
# Save THIS FILE to the rotation ESP32 root filesystem as:
#     main.py
#
# Hardware:
#   Relay:       GPIO13
#   Optical gate GPIO27 (falling edge, one trigger per revolution)
#   LoRa UART2:  RX=GPIO16, TX=GPIO17, 115200
#   LoRa AUX:    GPIO26
#   LoRa M0:     GPIO33 -> LOW
#   LoRa M1:     GPIO32 -> LOW
#
# PC protocol:
#   PC  -> SYNC <token>
#   ESP -> SYNC <token> <receive_us> <send_us>
#   PC  -> ROT <session>
#   ESP -> OK ROT <session>
#   ESP -> TRIG <session> <count> <zero_time_us>
#   PC  -> PING
#   ESP -> PONG
#   PC  -> OFF
#   ESP -> OK OFF
#
# USB/Thonny:
#   This firmware prints compact debug lines to USB REPL.
#   Ctrl+C can still interrupt the program and return to >>>.

import machine
import micropython
import utime
from machine import Pin, UART


VERSION = "ROTATION_SYNC_MP_V3"

# -------------------- pins --------------------

RELAY_PIN = 13
SENSOR_PIN = 27

LORA_UART_ID = 2
LORA_RX_PIN = 16
LORA_TX_PIN = 17
AUX_PIN = 26
M0_PIN = 33
M1_PIN = 32

LORA_BAUDRATE = 115200

# -------------------- timing / safety --------------------

DEBOUNCE_US = 8000              # reject repeated edges within 8 ms
BOOT_IGNORE_US = 800000         # ignore optical input for 0.8 s after boot
MOTOR_IGNORE_US = 300000        # ignore optical input for 0.3 s after motor starts
WATCHDOG_US = 40000000          # 40 s command watchdog

RX_MAX_LENGTH = 512
TX_QUEUE_MAX = 32
TRIGGER_CAPACITY = 32
LOOP_SLEEP_MS = 1

# AUX is useful but must not be able to permanently block all communication.
# If AUX stays LOW for more than this interval, UART TX is attempted anyway.
AUX_FORCE_TX_AFTER_US = 250000

USB_DEBUG = True

micropython.alloc_emergency_exception_buf(256)


def debug(*items):
    if not USB_DEBUG:
        return
    try:
        print(*items)
    except Exception:
        pass


class ExtendedClock:
    """Extend ticks_us() into a monotonic microsecond counter."""
    def __init__(self):
        self.raw = utime.ticks_us()
        self.total = 0

    def now(self):
        raw = utime.ticks_us()
        self.total += utime.ticks_diff(raw, self.raw)
        self.raw = raw
        return self.total

    def from_raw(self, raw):
        # raw must be a recently captured ticks_us() value.
        self.now()
        age = utime.ticks_diff(self.raw, raw)
        return self.total - age


clock = ExtendedClock()

relay = Pin(RELAY_PIN, Pin.OUT, value=0)
sensor = Pin(SENSOR_PIN, Pin.IN, Pin.PULL_UP)

aux = Pin(AUX_PIN, Pin.IN, Pin.PULL_UP)
m0 = Pin(M0_PIN, Pin.OUT, value=0)
m1 = Pin(M1_PIN, Pin.OUT, value=0)

lora = UART(
    LORA_UART_ID,
    baudrate=LORA_BAUDRATE,
    bits=8,
    parity=None,
    stop=1,
    tx=Pin(LORA_TX_PIN),
    rx=Pin(LORA_RX_PIN),
    timeout=0,
    txbuf=256,
    rxbuf=512,
)

# -------------------- IRQ-owned trigger ring --------------------

trigger_time_raw = [0] * TRIGGER_CAPACITY
trigger_counts = [0] * TRIGGER_CAPACITY
trigger_head = 0
trigger_tail = 0
trigger_overflow = False

count_total = 0
last_irq_raw = 0

motor_enabled = False
motor_start_raw = 0

boot_ignore_active = True
motor_ignore_active = False
boot_start_raw = utime.ticks_us()


def _reset_trigger_ring(reset_count=False):
    global trigger_head, trigger_tail, trigger_overflow
    global count_total, last_irq_raw

    state = machine.disable_irq()
    trigger_head = 0
    trigger_tail = 0
    trigger_overflow = False
    last_irq_raw = 0
    if reset_count:
        count_total = 0
    machine.enable_irq(state)


def motor_state():
    return 1 if motor_enabled else 0


def motor_off():
    global motor_enabled, motor_ignore_active

    state = machine.disable_irq()
    motor_enabled = False
    motor_ignore_active = False
    machine.enable_irq(state)

    relay.value(0)
    debug("[MOTOR] OFF")


def motor_on(reset_count=True):
    global motor_enabled, motor_start_raw, motor_ignore_active
    global trigger_head, trigger_tail, trigger_overflow
    global count_total, last_irq_raw

    # Keep logical motor state disabled while the relay edge occurs.
    state = machine.disable_irq()
    motor_enabled = False
    machine.enable_irq(state)

    relay.value(1)
    start_raw = utime.ticks_us()

    state = machine.disable_irq()
    motor_start_raw = start_raw
    motor_ignore_active = True
    last_irq_raw = 0
    trigger_head = 0
    trigger_tail = 0
    trigger_overflow = False
    if reset_count:
        count_total = 0
    motor_enabled = True
    machine.enable_irq(state)

    debug("[MOTOR] ON")


def sensor_irq(_pin):
    """IRQ: record only raw event time + count. No strings, UART or print."""
    global count_total, last_irq_raw
    global trigger_head, trigger_overflow

    now = utime.ticks_us()

    if not motor_enabled:
        return

    if boot_ignore_active:
        return

    if motor_ignore_active:
        return

    if last_irq_raw and utime.ticks_diff(now, last_irq_raw) < DEBOUNCE_US:
        return

    last_irq_raw = now
    count_total += 1

    next_head = trigger_head + 1
    if next_head >= TRIGGER_CAPACITY:
        next_head = 0

    if next_head == trigger_tail:
        trigger_overflow = True
        return

    trigger_time_raw[trigger_head] = now
    trigger_counts[trigger_head] = count_total
    trigger_head = next_head


sensor.irq(trigger=Pin.IRQ_FALLING, handler=sensor_irq)

# -------------------- application state --------------------

session = ""
last_command_us = clock.now()

rx_buf = bytearray()
tx_queue = []

# Track how long AUX has stayed low while we have something to transmit.
aux_low_since_us = None


def enqueue_text(text):
    if len(tx_queue) >= TX_QUEUE_MAX:
        fail_safe("TX_OVERFLOW")
        return False
    tx_queue.append(("TEXT", str(text)))
    return True


def enqueue_sync(token, received_us):
    if len(tx_queue) >= TX_QUEUE_MAX:
        fail_safe("TX_OVERFLOW")
        return False
    tx_queue.append(("SYNC", str(token), int(received_us)))
    return True


def fail_safe(reason):
    global session

    motor_off()
    session = ""
    _reset_trigger_ring(reset_count=False)

    tx_queue[:] = []
    tx_queue.append(("TEXT", "ERROR " + str(reason)))
    debug("[FAILSAFE]", reason)


def flush_tx():
    """Flush at most one queued LoRa frame per loop."""
    global aux_low_since_us

    if not tx_queue:
        aux_low_since_us = None
        return

    now_us = clock.now()
    aux_ready = (aux.value() == 1)

    if aux_ready:
        aux_low_since_us = None
    else:
        if aux_low_since_us is None:
            aux_low_since_us = now_us
            return
        if now_us - aux_low_since_us < AUX_FORCE_TX_AFTER_US:
            return
        # AUX has been low too long. Do not let it deadlock all PC communication.
        debug("[WARN] AUX LOW >250ms, force UART TX")
        aux_low_since_us = now_us

    item = tx_queue[0]
    kind = item[0]

    if kind == "SYNC":
        _, token, received_us = item
        send_us = clock.now()
        line = "SYNC {} {} {}".format(token, received_us, send_us)
    else:
        line = item[1].rstrip("\r\n")

    payload = (line + "\r\n").encode()

    try:
        written = lora.write(payload)
    except Exception as exc:
        debug("[UART TX EXC]", repr(exc))
        written = None

    if written != len(payload):
        fail_safe("UART_WRITE")
        return

    del tx_queue[0]
    debug("[TX]", line)


def _pop_trigger():
    global trigger_tail

    state = machine.disable_irq()

    if trigger_tail == trigger_head:
        machine.enable_irq(state)
        return None

    raw = trigger_time_raw[trigger_tail]
    count = trigger_counts[trigger_tail]

    trigger_tail += 1
    if trigger_tail >= TRIGGER_CAPACITY:
        trigger_tail = 0

    machine.enable_irq(state)
    return raw, count


def _take_trigger_overflow():
    global trigger_overflow

    state = machine.disable_irq()
    value = trigger_overflow
    trigger_overflow = False
    machine.enable_irq(state)
    return value


def process_trigger_events():
    if _take_trigger_overflow():
        fail_safe("IRQ_OVERFLOW")
        return

    for _ in range(TRIGGER_CAPACITY):
        event = _pop_trigger()
        if event is None:
            break

        raw, count = event
        event_us = clock.from_raw(raw)

        if session:
            line = "TRIG {} {} {}".format(session, count, event_us)
            enqueue_text(line)
            debug("[ZERO]", "count=", count, "time_us=", event_us)
        else:
            debug("[ZERO manual]", "count=", count, "time_us=", event_us)


def send_status():
    enqueue_text(
        "STATUS {} MOTOR={} COUNT={} SESSION={} AUX={} TXQ={}".format(
            VERSION,
            motor_state(),
            count_total,
            session if session else "-",
            aux.value(),
            len(tx_queue),
        )
    )


def send_help():
    for line in (
        VERSION,
        "SYNC <token>",
        "ROT <session>",
        "OFF",
        "PING",
        "STATUS",
        "ON",
        "RESETCNT",
        "HELP",
    ):
        enqueue_text(line)


def stop_synchronized():
    global session

    motor_off()
    session = ""
    _reset_trigger_ring(reset_count=False)

    # Remove stale TRIG/SYNC from the old session before acknowledging stop.
    tx_queue[:] = []
    enqueue_text("OK OFF")


def start_synchronized(new_session):
    global session

    motor_off()
    session = ""
    tx_queue[:] = []
    _reset_trigger_ring(reset_count=True)

    session = str(new_session)
    motor_on(reset_count=True)
    enqueue_text("OK ROT " + session)

    debug("[SESSION] START", session)


def start_manual():
    global session

    motor_off()
    session = ""
    tx_queue[:] = []
    _reset_trigger_ring(reset_count=True)

    motor_on(reset_count=True)
    enqueue_text("OK ON")


def handle_cmd(raw_cmd):
    global last_command_us

    try:
        text = raw_cmd.decode("utf-8", "ignore").strip()
    except Exception:
        return

    if not text:
        return

    parts = text.split()
    cmd = parts[0].upper()
    received_us = clock.now()

    last_command_us = received_us
    debug("[RX]", text)

    if cmd == "PING" and len(parts) == 1:
        enqueue_text("PONG")
        return

    if cmd == "SYNC" and len(parts) == 2:
        enqueue_sync(parts[1], received_us)
        return

    if cmd == "ROT" and len(parts) == 2:
        start_synchronized(parts[1])
        return

    if cmd == "OFF" and len(parts) == 1:
        stop_synchronized()
        return

    if cmd == "STATUS" and len(parts) == 1:
        send_status()
        return

    if cmd == "HELP" and len(parts) == 1:
        send_help()
        return

    if cmd == "ON" and len(parts) == 1:
        start_manual()
        return

    if cmd == "RESETCNT" and len(parts) == 1:
        if session:
            enqueue_text("ERR RESETCNT NOT ALLOWED DURING SYNC")
            return
        _reset_trigger_ring(reset_count=True)
        enqueue_text("OK RESETCNT")
        return

    enqueue_text("ERROR COMMAND")


def process_complete_lines():
    while True:
        pos_r = rx_buf.find(b"\r")
        pos_n = rx_buf.find(b"\n")

        if pos_r < 0 and pos_n < 0:
            return

        if pos_r < 0:
            pos = pos_n
        elif pos_n < 0:
            pos = pos_r
        else:
            pos = min(pos_r, pos_n)

        one_cmd = bytes(rx_buf[:pos])
        del rx_buf[:pos + 1]

        while rx_buf and rx_buf[0] in (10, 13):
            del rx_buf[0]

        if one_cmd.strip():
            handle_cmd(one_cmd)


def poll_lora_rx():
    try:
        n = lora.any()
    except Exception as exc:
        debug("[UART RX EXC]", repr(exc))
        return

    if not n:
        return

    try:
        data = lora.read(min(n, 256))
    except Exception as exc:
        debug("[UART RX EXC]", repr(exc))
        return

    if not data:
        return

    rx_buf.extend(data)

    if len(rx_buf) > RX_MAX_LENGTH:
        rx_buf[:] = b""
        fail_safe("RX_OVERFLOW")
        return

    process_complete_lines()


def poll_watchdog(now_us):
    if motor_enabled and now_us - last_command_us > WATCHDOG_US:
        fail_safe("WATCHDOG")


def poll_startup_guards():
    global boot_ignore_active, motor_ignore_active

    now = utime.ticks_us()

    if boot_ignore_active:
        if utime.ticks_diff(now, boot_start_raw) >= BOOT_IGNORE_US:
            boot_ignore_active = False
            debug("[GUARD] boot ignore released")

    if motor_ignore_active:
        if utime.ticks_diff(now, motor_start_raw) >= MOTOR_IGNORE_US:
            motor_ignore_active = False
            debug("[GUARD] motor ignore released")


def main():
    # Transparent LoRa mode.
    m0.value(0)
    m1.value(0)

    motor_off()
    _reset_trigger_ring(reset_count=True)

    utime.sleep_ms(300)

    debug("")
    debug("========================================")
    debug("TriScan rotation firmware")
    debug("VERSION:", VERSION)
    debug("LoRa UART2 RX=GPIO16 TX=GPIO17 @115200")
    debug("AUX GPIO26 =", aux.value())
    debug("Waiting for LoRa commands...")
    debug("========================================")

    enqueue_text("READY " + VERSION)

    try:
        while True:
            # Receive PC commands first.
            poll_lora_rx()

            # Convert IRQ events to LoRa TRIG packets.
            process_trigger_events()

            poll_startup_guards()
            poll_watchdog(clock.now())

            # Flush one queued packet.
            flush_tx()

            utime.sleep_ms(LOOP_SLEEP_MS)

    finally:
        motor_off()


if __name__ == "__main__":
    main()
