"""ESP32 / MicroPython CCD short-coordinate calibration firmware.

This version is for distance calibration only:
- CCD request is the short coordinate command: @c0071#@
- Output is raw CCD minimum coordinate X only, with sample statistics.
- No distance conversion table is used here.
"""

from machine import UART, Pin
import time
import sys
import select
import math


VERSION = "CCD-PEAK-RAW-CAL-3.0"

DEFAULT_INPUT_MODE = "USB"  # USB for Thonny calibration. Use INPUT BOTH for LoRa too.
DEFAULT_EXPOSURE = 5

CCD_MIN_COMMAND = b"@c0071#@"
CCD_BAUD = 256000
CCD_TIMEOUT_MS = 300
CCD_DRAIN_TIMEOUT_MS = 80
CCD_MAX_RESPONSE = 64
COORDINATE_MAX = 1500

SAMPLE_DEFAULT_COUNT = 10
SAMPLE_MAX_COUNT = 500
SAMPLE_INTERVAL_MS = 20

COMMAND_MAX_LENGTH = 160
LORA_BAUD = 115200


def ticks_ms():
    return time.ticks_ms()


def ticks_diff(newer, older):
    return time.ticks_diff(newer, older)


def ticks_add(value, delta):
    return time.ticks_add(value, delta)


def hex_text(data):
    return " ".join("{:02X}".format(b) for b in data)


def parse_coordinate(data):
    """Parse CCD short-coordinate replies.

    The working short reply is usually:
        FF FE HH LL
    where HH LL is a big-endian 16-bit coordinate.
    """
    if len(data) >= 4:
        for i in range(len(data) - 3):
            if data[i] == 0xFF and data[i + 1] == 0xFE:
                value = (data[i + 2] << 8) | data[i + 3]
                if 0 <= value <= COORDINATE_MAX:
                    return value, "HEADER_BE16"
                return None, "COORDINATE_OUT_OF_RANGE"

    stripped = data.strip()
    if stripped and all(48 <= b <= 57 for b in stripped):
        value = int(stripped)
        if 0 <= value <= COORDINATE_MAX:
            return value, "ASCII"
        return None, "COORDINATE_OUT_OF_RANGE"

    if len(data) == 2:
        value = (data[0] << 8) | data[1]
        if 0 <= value <= COORDINATE_MAX:
            return value, "BARE_BE16"

    return None, "UNKNOWN_FRAME"


def summarize(values):
    if not values:
        return None

    ordered = sorted(values)
    count = len(ordered)
    middle = count // 2

    if count & 1:
        median = ordered[middle]
    else:
        median = (ordered[middle - 1] + ordered[middle]) / 2.0

    mean = sum(ordered) / count
    if count > 1:
        variance = sum((x - mean) ** 2 for x in ordered) / (count - 1)
        std = math.sqrt(variance)
    else:
        std = None

    return median, mean, ordered[0], ordered[-1], std


def parse_sample_count(cmd):
    if cmd == "SAMPLE":
        return SAMPLE_DEFAULT_COUNT

    if cmd.startswith("SAMPLE "):
        text = cmd[7:].strip()
    elif cmd.startswith("SAMPLE"):
        text = cmd[6:].strip()
    else:
        return None

    if text.isdigit():
        return int(text)
    return 0


class PeakRawCalibrationNode:
    def __init__(self):
        self.laser = Pin(25, Pin.OUT, value=0)
        self.lora = UART(
            1,
            baudrate=LORA_BAUD,
            tx=4,
            rx=5,
            bits=8,
            parity=None,
            stop=1,
            rxbuf=1024,
        )
        self.ccd = UART(
            2,
            baudrate=CCD_BAUD,
            tx=17,
            rx=16,
            bits=8,
            parity=None,
            stop=1,
            rxbuf=256,
        )

        self.exposure_sent = None
        self.input_mode = DEFAULT_INPUT_MODE
        self.debug = False
        self.batch = None

        self.buffers = {"USB": bytearray(), "LORA": bytearray()}
        self.discarding = {"USB": False, "LORA": False}
        self.last_lora_ms = ticks_ms()
        self.usb_poll = None

    def reply(self, message):
        print(message)
        if self.input_mode in ("LORA", "BOTH"):
            self.lora.write((message + "\r\n").encode())
        time.sleep_ms(2)

    def command_error(self, message, source, raw):
        print("{} SOURCE={} BYTES={} RAW_HEX={}{}".format(
            message,
            source,
            len(raw),
            hex_text(raw[:48]),
            " ..." if len(raw) > 48 else "",
        ))

    def drain_ccd(self):
        start = ticks_ms()
        while self.ccd.any():
            self.ccd.read(min(128, self.ccd.any()))
            if ticks_diff(ticks_ms(), start) >= CCD_DRAIN_TIMEOUT_MS:
                return False
            time.sleep_ms(1)
        return True

    def set_exposure(self, value):
        if not 0 <= value <= 13:
            self.reply("ERR EXPOSURE USE 0..13")
            return

        self.drain_ccd()
        command = "@c{:04d}#@".format(value).encode()
        written = self.ccd.write(command)
        if written != len(command):
            self.reply("ERR EXPOSURE CCD_WRITE_FAILED")
            return

        time.sleep_ms(80)
        self.drain_ccd()
        self.exposure_sent = value
        self.reply("EXPOSURE SENT: {}".format(value))

    def read_short_coordinate(self):
        if not self.drain_ccd():
            return None, b"", "CCD_BUSY"

        written = self.ccd.write(CCD_MIN_COMMAND)
        if written != len(CCD_MIN_COMMAND):
            return None, b"", "CCD_WRITE_FAILED"

        start = ticks_ms()
        response = bytearray()

        while ticks_diff(ticks_ms(), start) < CCD_TIMEOUT_MS:
            count = self.ccd.any()
            if count:
                chunk = self.ccd.read(min(count, CCD_MAX_RESPONSE - len(response)))
                if chunk:
                    response.extend(chunk)
                    value, fmt = parse_coordinate(response)
                    if value is not None:
                        return value, bytes(response), fmt
                    if len(response) >= CCD_MAX_RESPONSE:
                        return None, bytes(response), "RESPONSE_TOO_LONG"
            time.sleep_ms(1)

        if response:
            value, fmt = parse_coordinate(response)
            if value is not None:
                return value, bytes(response), fmt
            return None, bytes(response), "RESPONSE_TIMEOUT_" + fmt

        return None, b"", "CCD_TIMEOUT"

    def measure(self):
        value, raw, status = self.read_short_coordinate()

        if value is None:
            self.reply("CCD INVALID: {}".format(status))
            if raw:
                self.reply("CCD RAW HEX: {}".format(hex_text(raw)))
            return None

        if self.debug:
            self.reply("CCD RAW HEX: {}".format(hex_text(raw)))
            self.reply("CCD FORMAT: {}".format(status))

        self.reply("CCD MIN X: {}".format(value))
        return value

    def start_batch(self, count):
        self.batch = {
            "count": count,
            "attempted": 0,
            "values": [],
            "next_ms": ticks_ms(),
        }
        self.reply("SAMPLE BEGIN: N={} EXPOSURE_FIXED={}".format(
            count,
            self.exposure_sent,
        ))

    def finish_batch(self, stopped=False):
        batch = self.batch
        if batch is None:
            return

        self.batch = None
        values = batch["values"]
        invalid = batch["attempted"] - len(values)

        self.reply("SAMPLE {}: REQUESTED={} ATTEMPTED={} PARSED={} INVALID={}".format(
            "STOPPED" if stopped else "END",
            batch["count"],
            batch["attempted"],
            len(values),
            invalid,
        ))

        if not values:
            self.reply("SUMMARY: NO PARSED COORDINATES")
            return

        self.reply("SUMMARY VALUES_X={}".format(
            ",".join(str(value) for value in values)
        ))

        median, mean, low, high, std = summarize(values)
        self.reply("SUMMARY MEDIAN_X={:.2f} MEAN_X={:.2f}".format(median, mean))
        self.reply("SUMMARY MIN_X={} MAX_X={} SPAN_X={} STD_X={}".format(
            low,
            high,
            high - low,
            "NA" if std is None else "{:.2f}".format(std),
        ))
        self.reply("NOTE: RAW CCD MINIMUM COORDINATE ONLY; NO DISTANCE CONVERSION")

    def tick_batch(self):
        batch = self.batch
        if batch is None:
            return

        if ticks_diff(ticks_ms(), batch["next_ms"]) < 0:
            return

        value = self.measure()
        batch["attempted"] += 1
        if value is not None:
            batch["values"].append(value)

        if batch["attempted"] >= batch["count"]:
            self.finish_batch()
        else:
            batch["next_ms"] = ticks_add(ticks_ms(), SAMPLE_INTERVAL_MS)

    def help(self):
        for line in (
            VERSION + " SHORT COORDINATE ONLY",
            "CCD CMD: @c0071#@",
            "CMDS: LASER 1, LASER 0, MIN",
            "CMDS: SAMPLE or SAMPLE 10 or SAMPLE10",
            "CMDS: EXPOSURE n, STATUS, DEBUG 0/1, STOP, PING, HELP",
            "CMDS: INPUT USB / INPUT LORA / INPUT BOTH",
        ):
            self.reply(line)

    def normalize_command(self, raw, source):
        try:
            cmd = raw.decode().strip().upper()
        except UnicodeError:
            self.command_error("ERR COMMAND ENCODING", source, raw)
            return None

        if not cmd:
            return ""

        if cmd.startswith("CCD "):
            cmd = cmd[4:].strip()

        if cmd in ("@C0071#@", "@C0071", "@0071#@", "@0071", "0071"):
            return "MIN"

        if len(cmd) == 8 and cmd.startswith("@C") and cmd.endswith("#@"):
            digits = cmd[2:6]
            if digits.isdigit():
                code = int(digits)
                if code == 71:
                    return "MIN"
                return "EXPOSURE " + str(code)

        return cmd

    def command(self, raw, source):
        cmd = self.normalize_command(raw, source)
        if cmd is None or not cmd:
            return

        if self.debug:
            print("COMMAND SOURCE={} TEXT={}".format(source, cmd))

        if cmd == "PING":
            self.reply("PONG")
        elif cmd in ("HELP", "?"):
            self.help()
        elif cmd == "STATUS":
            self.reply("STATUS {} LASER={} EXPOSURE_SENT={} BATCH={} DEBUG={} INPUT={} CMD={}".format(
                VERSION,
                self.laser.value(),
                self.exposure_sent,
                int(self.batch is not None),
                int(self.debug),
                self.input_mode,
                CCD_MIN_COMMAND.decode(),
            ))
        elif cmd == "STOP":
            self.finish_batch(stopped=True)
            self.reply("OK STOP")
        elif cmd == "LASER 0":
            self.finish_batch(stopped=True)
            self.laser.value(0)
            self.reply("OK LASER 0")
        elif cmd == "LASER 1":
            self.laser.value(1)
            time.sleep_ms(120)
            self.reply("OK LASER 1")
        elif cmd in ("DEBUG 0", "DEBUG 1"):
            self.debug = cmd.endswith("1")
            self.reply("OK " + cmd)
        elif cmd in ("INPUT USB", "INPUT LORA", "INPUT BOTH"):
            self.input_mode = cmd.split()[1]
            for channel in self.buffers:
                self.buffers[channel] = bytearray()
                self.discarding[channel] = False
            self.reply("OK INPUT " + self.input_mode)
        elif cmd.startswith("EXPOSURE "):
            number = cmd[9:].strip()
            if number.isdigit():
                self.set_exposure(int(number))
            else:
                self.reply("ERR EXPOSURE USE 0..13")
        elif self.batch is not None:
            self.reply("ERR SAMPLE RUNNING; SEND STOP FIRST")
        elif cmd in ("MIN", "X", "MEASURE"):
            if not self.laser.value():
                self.reply("ERR LASER OFF; SEND LASER 1 FIRST")
            else:
                self.measure()
        elif cmd == "SAMPLE" or cmd.startswith("SAMPLE"):
            if not self.laser.value():
                self.reply("ERR LASER OFF; SEND LASER 1 FIRST")
                return

            count = parse_sample_count(cmd)
            if count is None:
                self.command_error("ERR UNKNOWN COMMAND; SEND HELP", source, raw)
                return
            if not 1 <= count <= SAMPLE_MAX_COUNT:
                self.reply("ERR SAMPLE USE 1..{}".format(SAMPLE_MAX_COUNT))
                return

            self.start_batch(count)
        else:
            self.command_error("ERR UNKNOWN COMMAND; SEND HELP", source, raw)

    def feed(self, source, data):
        for byte in data:
            if self.input_mode not in (source, "BOTH"):
                self.buffers[source] = bytearray()
                self.discarding[source] = False
                return

            if byte in (10, 13):
                if not self.discarding[source] and self.buffers[source]:
                    raw = bytes(self.buffers[source])
                    self.buffers[source] = bytearray()
                    self.command(raw, source)
                else:
                    self.buffers[source] = bytearray()
                self.discarding[source] = False
            elif byte == 0 or self.discarding[source]:
                continue
            elif byte in (8, 127):
                if self.buffers[source]:
                    del self.buffers[source][-1]
            elif len(self.buffers[source]) >= COMMAND_MAX_LENGTH:
                self.command_error("ERR COMMAND TOO LONG", source, bytes(self.buffers[source]))
                self.buffers[source] = bytearray()
                self.discarding[source] = True
            else:
                self.buffers[source].append(byte)

    def poll_usb(self):
        if self.usb_poll is None:
            return

        for _ in range(64):
            events = self.usb_poll.poll(0)
            if not events:
                return

            flags = events[0][1]
            if flags & (select.POLLERR | select.POLLHUP):
                try:
                    self.usb_poll.unregister(sys.stdin)
                except Exception:
                    pass
                self.usb_poll = None
                return

            if not flags & select.POLLIN:
                return

            char = sys.stdin.read(1)
            if not char:
                return
            self.feed("USB", char.encode())

    def poll_lora(self):
        if self.lora.any():
            data = self.lora.read(min(256, self.lora.any()))
            if data:
                self.last_lora_ms = ticks_ms()
                self.feed("LORA", data)

        if ticks_diff(ticks_ms(), self.last_lora_ms) > 200:
            if self.buffers["LORA"]:
                self.feed("LORA", b"\n")

    def run(self):
        time.sleep_ms(600)
        self.reply(VERSION + " STARTING; LASER OFF")
        self.set_exposure(DEFAULT_EXPOSURE)

        try:
            self.usb_poll = select.poll()
            self.usb_poll.register(sys.stdin, select.POLLIN)
        except (OSError, TypeError, ValueError):
            self.usb_poll = None
            self.reply("USB INPUT UNAVAILABLE")

        self.help()

        while True:
            self.poll_lora()
            self.poll_usb()
            self.tick_batch()
            time.sleep_ms(1)


node = PeakRawCalibrationNode()
try:
    node.run()
finally:
    node.laser.value(0)

