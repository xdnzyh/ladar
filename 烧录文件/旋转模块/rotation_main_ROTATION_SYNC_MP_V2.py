# rotation_main.py
# ESP32 / MicroPython
# TriScan rotation node: motor relay + one optical gate + LoRa synchronization
#
# Save this file to the ROTATION ESP32 as main.py.
#
# Hardware pins retained from the user's existing program:
#   Relay:       GPIO13
#   Optical gate GPIO27, falling-edge interrupt
#   LoRa UART2:  RX=GPIO16, TX=GPIO17, 115200
#   LoRa AUX:    GPIO26
#   LoRa M0:     GPIO33 -> LOW
#   LoRa M1:     GPIO32 -> LOW
#
# Protocol expected by the current PC synchronized acquisition:
#   PC  -> SYNC <token>
#   ESP -> SYNC <token> <receive_us> <send_us>
#
#   PC  -> ROT <session>
#   ESP -> OK ROT <session>
#   ESP -> TRIG <session> <count> <zero_time_us>
#
#   PC  -> PING
#   ESP -> PONG
#
#   PC  -> OFF
#   ESP -> OK OFF
#
# Manual/debug commands kept:
#   ON
#   OFF
#   STATUS
#   RESETCNT            (manual mode only)
#   WINDOW <n>          (manual sliding-window RPS report, n >= 2)
#   WINDOW 0            (disable manual RPS report)
#   PING
#   HELP
#
# Important:
# - Only one optical gate is assumed: one TRIG per revolution.
# - The IRQ records the event time immediately; LoRa delay does not change it.
# - Formal navigation does NOT use SLIDE_REPORT. It uses TRIG timestamps.
# - Relay startup glitches are ignored for 300 ms after motor start.
# - Optical gate edges within 8 ms are treated as bounce/noise.
# - A 40 s command watchdog switches the motor off if the PC disappears.

import utime
import micropython
import machine
from machine import Pin, UART


VERSION = "ROTATION_SYNC_MP_V2"

# -------------------- hardware --------------------

RELAY_PIN = 13
SENSOR_PIN = 27

LORA_UART_ID = 2
LORA_RX_PIN = 16
LORA_TX_PIN = 17
AUX_PIN = 26
MD0_PIN = 33
MD1_PIN = 32

LORA_BAUDRATE = 115200

# -------------------- timing / safety --------------------

DEBOUNCE_US = 8000
BOOT_IGNORE_US = 800000
MOTOR_ON_IGNORE_US = 300000

WATCHDOG_US = 40000000       # 40 s
RX_MAX_LENGTH = 256
TX_QUEUE_MAX = 32
TRIGGER_CAPACITY = 32
LOOP_SLEEP_MS = 1

micropython.alloc_emergency_exception_buf(200)


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
        """Map a recently captured raw ticks_us value into the extended clock."""
        self.now()
        age = utime.ticks_diff(self.raw, raw)
        return self.total - age


clock = ExtendedClock()

relay = Pin(RELAY_PIN, Pin.OUT, value=0)
sensor = Pin(SENSOR_PIN, Pin.IN, Pin.PULL_UP)

aux = Pin(AUX_PIN, Pin.IN, Pin.PULL_UP)
md0 = Pin(MD0_PIN, Pin.OUT, value=0)
md1 = Pin(MD1_PIN, Pin.OUT, value=0)

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

# -------------------- IRQ-owned state --------------------
#
# No list append, string creation, LoRa I/O, or scheduled Python callbacks are
# performed in the interrupt. The IRQ only writes into this fixed ring buffer.

trigger_time_raw = [0] * TRIGGER_CAPACITY
trigger_counts = [0] * TRIGGER_CAPACITY

trigger_head = 0
trigger_tail = 0
trigger_overflow = False

count_total = 0
last_irq_raw = 0
motor_start_raw = 0
boot_start_raw = utime.ticks_us()
motor_enabled = False


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


def motor_off():
    global motor_enabled
    state = machine.disable_irq()
    motor_enabled = False
    machine.enable_irq(state)
    relay.value(0)


def motor_on(reset_count=True):
    """Start relay safely; startup optical edges are ignored for 300 ms."""
    global motor_enabled, motor_start_raw, last_irq_raw, count_total
    global trigger_head, trigger_tail, trigger_overflow

    # Keep IRQ disabled logically while the relay transitions. Any relay glitch
    # that occurs here is ignored because motor_enabled is False.
    state = machine.disable_irq()
    motor_enabled = False
    machine.enable_irq(state)

    relay.value(1)
    start = utime.ticks_us()

    state = machine.disable_irq()
    motor_start_raw = start
    last_irq_raw = 0
    trigger_head = 0
    trigger_tail = 0
    trigger_overflow = False
    if reset_count:
        count_total = 0
    motor_enabled = True
    machine.enable_irq(state)


def motor_state():
    return 1 if motor_enabled else 0


def sensor_irq(_pin):
    global count_total, last_irq_raw
    global trigger_head, trigger_overflow

    now = utime.ticks_us()

    if not motor_enabled:
        return

    # 1. Ignore early boot transients.
    if utime.ticks_diff(now, boot_start_raw) < BOOT_IGNORE_US:
        return

    # 2. Ignore relay/motor startup transient.
    if utime.ticks_diff(now, motor_start_raw) < MOTOR_ON_IGNORE_US:
        return

    # 3. Debounce / reject implausibly close repeated edges.
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

# Manual sliding-window diagnostics.
# These are updated in the main loop, not in the interrupt.
manual_window_size = 0
manual_ticks = []


# -------------------- LoRa TX --------------------

def _fail_safe_motor(reason):
    """Stop motor/session and replace queued traffic with one ERROR line."""
    global session, manual_ticks

    motor_off()
    session = ""
    _reset_trigger_ring(reset_count=False)
    manual_ticks = []

    tx_queue[:] = []
    tx_queue.append(("TEXT", "ERROR " + reason))


def enqueue_text(text):
    if len(tx_queue) >= TX_QUEUE_MAX:
        _fail_safe_motor("TX_OVERFLOW")
        return False
    tx_queue.append(("TEXT", str(text)))
    return True


def enqueue_sync(token, received_us):
    if len(tx_queue) >= TX_QUEUE_MAX:
        _fail_safe_motor("TX_OVERFLOW")
        return False
    # The send timestamp is generated in flush_tx(), immediately before write.
    tx_queue.append(("SYNC", str(token), int(received_us)))
    return True


def flush_tx():
    if not tx_queue:
        return

    # E22/E32-style AUX: HIGH means the radio is ready for more UART data.
    if aux.value() != 1:
        return

    item = tx_queue[0]
    kind = item[0]

    if kind == "SYNC":
        _, token, received_us = item
        send_us = clock.now()
        payload = "SYNC {} {} {}\r\n".format(
            token, received_us, send_us
        ).encode()
    else:
        payload = (item[1].rstrip("\r\n") + "\r\n").encode()

    try:
        written = lora.write(payload)
    except Exception:
        written = None

    if written != len(payload):
        _fail_safe_motor("UART_WRITE")
        return

    del tx_queue[0]


# -------------------- trigger event processing --------------------

def _pop_trigger():
    global trigger_tail

    state = machine.disable_irq()
    if trigger_tail == trigger_head:
        machine.enable_irq(state)
        return None

    raw = trigger_time_raw[trigger_tail]
    count = trigger_counts[trigger_tail]

    next_tail = trigger_tail + 1
    if next_tail >= TRIGGER_CAPACITY:
        next_tail = 0
    trigger_tail = next_tail

    machine.enable_irq(state)
    return raw, count


def _take_overflow_flag():
    global trigger_overflow

    state = machine.disable_irq()
    value = trigger_overflow
    trigger_overflow = False
    machine.enable_irq(state)
    return value


def _manual_window_observe(count, event_us):
    if manual_window_size < 2:
        return

    manual_ticks.append(event_us)
    if len(manual_ticks) > manual_window_size:
        manual_ticks.pop(0)

    if len(manual_ticks) != manual_window_size:
        return

    delta_us = manual_ticks[-1] - manual_ticks[0]
    intervals = manual_window_size - 1

    if delta_us <= 0:
        return

    delta_s = delta_us / 1000000.0

    # N trigger timestamps span N-1 complete revolution intervals.
    avg_rps = intervals / delta_s

    start_cycle = count - intervals
    end_cycle = count

    enqueue_text(
        "SLIDE_REPORT WINDOW={} CYCLE[{}-{}] TIME_S={:.3f} AVG_RPS={:.4f}".format(
            manual_window_size,
            start_cycle,
            end_cycle,
            delta_s,
            avg_rps,
        )
    )


def process_trigger_events():
    global session

    if _take_overflow_flag():
        _fail_safe_motor("IRQ_OVERFLOW")
        return

    # Drain the small ring quickly. Normally only one event exists because the
    # optical gate triggers once per revolution.
    for _ in range(TRIGGER_CAPACITY):
        event = _pop_trigger()
        if event is None:
            break

        raw, count = event
        event_us = clock.from_raw(raw)

        if session:
            # Formal synchronized navigation path.
            enqueue_text(
                "TRIG {} {} {}".format(
                    session,
                    count,
                    event_us,
                )
            )
        else:
            # Manual mode only.
            _manual_window_observe(count, event_us)


# -------------------- commands --------------------

def send_status():
    enqueue_text(
        "STATUS {} MOTOR={} COUNT={} SESSION={} WINDOW={} TXQ={}".format(
            VERSION,
            motor_state(),
            count_total,
            session if session else "-",
            manual_window_size,
            len(tx_queue),
        )
    )


def send_help():
    for line in (
        VERSION,
        "SYNC <token>                 clock synchronization",
        "ROT <session>                start synchronized rotation",
        "OFF                         stop motor and synchronized session",
        "PING                         reply PONG",
        "STATUS                       motor/count/session status",
        "ON                           manual motor start",
        "RESETCNT                     manual mode count reset",
        "WINDOW <n>                   manual RPS window, n>=2",
        "WINDOW 0                     disable manual RPS report",
    ):
        enqueue_text(line)


def stop_synchronized():
    global session, manual_ticks

    motor_off()
    session = ""
    _reset_trigger_ring(reset_count=False)
    manual_ticks = []

    # Drop old queued TRIG/SYNC traffic before acknowledging OFF.
    tx_queue[:] = []
    enqueue_text("OK OFF")


def start_synchronized(new_session):
    global session, manual_window_size, manual_ticks

    motor_off()
    session = ""

    # A new session must never inherit queued TRIG from the previous one.
    tx_queue[:] = []

    manual_window_size = 0
    manual_ticks = []

    _reset_trigger_ring(reset_count=True)

    session = new_session
    motor_on(reset_count=True)

    enqueue_text("OK ROT " + session)


def start_manual():
    global session, manual_ticks

    motor_off()
    session = ""
    tx_queue[:] = []
    manual_ticks = []
    _reset_trigger_ring(reset_count=True)
    motor_on(reset_count=True)
    enqueue_text("OK ON")


def handle_cmd(raw_cmd):
    global session, last_command_us
    global manual_window_size, manual_ticks

    try:
        text = raw_cmd.decode("utf-8", "ignore").strip()
    except Exception:
        return

    if not text:
        return

    parts = text.split()
    command = parts[0].upper()
    received_us = clock.now()

    # Every valid command refreshes the watchdog.
    last_command_us = received_us

    if command == "PING" and len(parts) == 1:
        enqueue_text("PONG")
        return

    if command == "SYNC" and len(parts) == 2:
        enqueue_sync(parts[1], received_us)
        return

    if command == "ROT" and len(parts) == 2:
        # IMPORTANT: the argument is a session token, NOT a window length.
        # This matches SynchronizedAcquisition on the PC.
        start_synchronized(parts[1])
        return

    if command == "OFF" and len(parts) == 1:
        stop_synchronized()
        return

    if command == "STATUS" and len(parts) == 1:
        send_status()
        return

    if command == "HELP" and len(parts) == 1:
        send_help()
        return

    # ---------- manual/debug compatibility ----------

    if command == "ON" and len(parts) == 1:
        start_manual()
        return

    if command == "RESETCNT" and len(parts) == 1:
        if session:
            enqueue_text("ERR RESETCNT NOT ALLOWED DURING SYNC")
            return
        _reset_trigger_ring(reset_count=True)
        manual_ticks = []
        enqueue_text("OK RESETCNT")
        return

    if command == "WINDOW" and len(parts) == 2:
        if session:
            enqueue_text("ERR WINDOW NOT ALLOWED DURING SYNC")
            return
        try:
            value = int(parts[1])
        except ValueError:
            enqueue_text("ERR WINDOW: need integer")
            return

        if value == 0:
            manual_window_size = 0
            manual_ticks = []
            enqueue_text("OK WINDOW OFF")
            return

        if not 2 <= value <= 32:
            enqueue_text("ERR WINDOW: use 0 or 2..32")
            return

        manual_window_size = value
        manual_ticks = []
        enqueue_text("OK WINDOW {}".format(value))
        return

    enqueue_text("ERROR COMMAND")


# -------------------- UART RX --------------------

def process_complete_lines():
    global rx_buf

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

        # MicroPython's bytearray does not reliably support ``del buf[...]``.
        # Skip the line ending (and any following CR/LF bytes), then rebuild
        # the small RX buffer from the unconsumed tail. RX_MAX_LENGTH is only
        # 256 bytes, so this bounded copy is safe and simple.
        next_pos = pos + 1
        buf_len = len(rx_buf)
        while next_pos < buf_len and rx_buf[next_pos] in (10, 13):
            next_pos += 1
        rx_buf = bytearray(rx_buf[next_pos:])

        if one_cmd.strip():
            handle_cmd(one_cmd)


def poll_lora_rx():
    global rx_buf

    n = lora.any()
    if not n:
        return

    data = lora.read(min(n, 256))
    if not data:
        return

    rx_buf.extend(data)

    if len(rx_buf) > RX_MAX_LENGTH:
        rx_buf = bytearray()
        _fail_safe_motor("RX_OVERFLOW")
        return

    process_complete_lines()


# -------------------- watchdog --------------------

def poll_watchdog(now_us):
    if motor_enabled and now_us - last_command_us > WATCHDOG_US:
        _fail_safe_motor("WATCHDOG")


# -------------------- main --------------------

def main():
    # Radio transparent mode.
    md0.value(0)
    md1.value(0)

    motor_off()
    _reset_trigger_ring(reset_count=True)

    # Let power/relay/radio rails settle.
    utime.sleep_ms(300)

    enqueue_text("READY " + VERSION)

    try:
        while True:
            # Commands first: OFF/PING/SYNC should be handled promptly.
            poll_lora_rx()

            process_trigger_events()

            now_us = clock.now()
            poll_watchdog(now_us)

            flush_tx()
            utime.sleep_ms(LOOP_SLEEP_MS)

    finally:
        motor_off()


if __name__ == "__main__":
    main()
