# main.py
# ESP32 / MicroPython
# TriScan measurement node: synchronized scanning + calibration + manual debug
#
# Fixed CCD exposure: 3
#
# LoRa protocol used by the current PC application:
#   SYNC <token>
#   START <session> <rate_hz> <requested_exposure> <fffe|raw2>
#   CAL <token> <requested_exposure> <fffe|raw2>
#   PING
#   STOP
#   LASER 0/1
#
# Responses:
#   READY MEASUREMENT_SYNC_CAL_V3
#   SYNC <token> <receive_us> <send_us>
#   OK START <session>
#   PIX <session> <sequence> <begin_us> <end_us> <pixel|-1>
#   PONG
#   OK STOP
#
# Manual/debug commands are also kept:
#   MIN / X / MEASURE
#   SAMPLE [n]
#   STATUS
#   DEBUG 0/1
#   INPUT USB/LORA/BOTH
#   EXPOSURE 3
#   HELP
#
# Legacy raw CCD commands are supported while no synchronized scan is running:
#   @c0071#@ -> FF FE + BE16 coordinate is returned raw
#   @c0081#@ -> BE16 coordinate is returned raw
#
# Wiring retained from the existing program:
#   Laser: GPIO25
#   LoRa UART1: TX=GPIO4 RX=GPIO5, 115200
#   LoRa AUX: GPIO18
#   CCD UART2: TX=GPIO17 RX=GPIO16, 256000

from machine import UART, Pin
import time
import sys
import select
import math


# Keep the CCD-CAL prefix and BATCH=... field in STATUS for compatibility
# with the existing static-verification desktop app. The synchronized
# scanning commands from the classmate's protocol remain available.
VERSION = "CCD-CAL-1.3-SYNC"

# -------------------- fixed hardware settings --------------------
EXPOSURE_INDEX = 3
DEFAULT_INPUT_MODE = "BOTH"        # USB / LORA / BOTH
DEFAULT_CENTER_MODE = "fffe"       # fffe / raw2
LASER_PIN = 25                      # laser module IN/TTL input
LASER_ON_LEVEL = 1                  # confirmed by the hardware test
LASER_OFF_LEVEL = 0

# Must remain below the PC-side 250 ms acceptance limit.
CCD_TIMEOUT_US = 240000
CCD_MAX_RESPONSE = 128
COORDINATE_MAX = 1500

# Manual SAMPLE interval only; synchronized scan uses START rate_hz.
MANUAL_SAMPLE_INTERVAL_US = 250000

COMMAND_MAX_LENGTH = 160
TX_QUEUE_MAX = 32
WATCHDOG_US = 40000000             # 40 s
START_SETTLE_US = 200000            # laser/exposure settling before first scan sample
CAL_SETTLE_US = 150000
LOOP_SLEEP_MS = 1


class ExtendedClock:
    """Monotonic microsecond clock extended across ticks_us() wraparound."""
    def __init__(self):
        self.raw = time.ticks_us()
        self.total = 0

    def now(self):
        raw = time.ticks_us()
        self.total += time.ticks_diff(raw, self.raw)
        self.raw = raw
        return self.total


def hex_text(data):
    return " ".join("{:02X}".format(b) for b in data)


def summarize(values):
    if not values:
        return None
    ordered = sorted(values)
    count = len(ordered)
    middle = count // 2
    median = (ordered[middle] if count % 2 else
              (ordered[middle - 1] + ordered[middle]) / 2.0)
    mean = sum(ordered) / count
    std = (math.sqrt(sum((x - mean) ** 2 for x in ordered) / (count - 1))
           if count > 1 else None)
    return median, mean, ordered[0], ordered[-1], std


class MeasurementNode:
    def __init__(self):
        self.clock = ExtendedClock()

        # The laser module is controlled only through its IN/TTL pin. Its
        # power must come from the rated supply and share GND with this ESP32.
        self.laser = Pin(LASER_PIN, Pin.OUT, value=LASER_OFF_LEVEL)
        self.aux = Pin(18, Pin.IN, Pin.PULL_UP)

        self.lora = UART(
            1,
            baudrate=115200,
            tx=4,
            rx=5,
            bits=8,
            parity=None,
            stop=1,
            rxbuf=512,
        )
        self.ccd = UART(
            2,
            baudrate=256000,
            tx=17,
            rx=16,
            bits=8,
            parity=None,
            stop=1,
            rxbuf=512,
        )

        self.input_mode = DEFAULT_INPUT_MODE
        self.debug = False

        self.buffers = {"USB": bytearray(), "LORA": bytearray()}
        self.discarding = {"USB": False, "LORA": False}
        self.last_lora_ms = time.ticks_ms()
        self.usb_poll = None

        self.tx_queue = []

        # Synchronized scanning state.
        self.session = ""
        self.scan_active = False
        self.center_mode = DEFAULT_CENTER_MODE
        self.scan_period_us = 20000       # 50 Hz until START changes it
        self.next_sample_us = 0
        self.sequence = 0
        self.last_command_us = self.clock.now()

        # One CCD request may be in flight at a time.
        # capture keys:
        #   owner: SCAN/CAL/MANUAL/BATCH/LEGACY
        #   mode: fffe/raw2
        #   begin: device timestamp
        #   seq/source/token as needed
        self.capture = None
        self.ccd_buf = bytearray()

        # Delayed single-shot CAL after exposure/laser settling.
        self.cal_pending = None
        self.cal_restore_laser = 0

        # Manual SAMPLE state.
        self.batch = None

        self.total_scan_attempts = 0
        self.total_scan_packets = 0
        self.total_ccd_timeouts = 0
        self.total_invalid_frames = 0

    def set_laser(self, enabled):
        """Set the laser input explicitly; high means ON for this module."""
        self.laser.value(LASER_ON_LEVEL if enabled else LASER_OFF_LEVEL)

    def laser_is_on(self):
        return self.laser.value() == LASER_ON_LEVEL

    # -------------------- output --------------------

    def usb_print(self, message):
        try:
            print(message)
        except Exception:
            pass

    def _queue_lora(self, item):
        if len(self.tx_queue) >= TX_QUEUE_MAX:
            # Fail safe: never let an overflowing radio queue leave the laser
            # and automatic scan running indefinitely.
            self._stop_all(laser_off=True)
            self.tx_queue[:] = []
            self.tx_queue.append(("TEXT", "ERROR TX_OVERFLOW"))
            return False
        self.tx_queue.append(item)
        return True

    def respond(self, source, message):
        """Return a textual reply to the command origin and mirror to USB."""
        self.usb_print(message)
        if source == "LORA":
            self._queue_lora(("TEXT", message))

    def protocol_text(self, message):
        """Send an autonomous protocol message to LoRa and mirror to USB."""
        self.usb_print(message)
        self._queue_lora(("TEXT", message))

    def protocol_bytes(self, payload):
        """Raw legacy bytes: LoRa only, plus optional USB hex trace."""
        if self.debug:
            self.usb_print("LEGACY TX HEX: " + hex_text(payload))
        self._queue_lora(("BYTES", payload))

    def queue_sync_reply(self, source, token, received_us):
        # For USB debugging we can reply immediately with the current time.
        if source == "USB":
            sent_us = self.clock.now()
            self.usb_print("SYNC {} {} {}".format(token, received_us, sent_us))
        else:
            # t3 must be taken as close as practical to the actual LoRa write.
            self._queue_lora(("SYNC", token, received_us))

    def flush_tx(self):
        if not self.tx_queue:
            return
        if not self.aux.value():
            return

        item = self.tx_queue[0]
        kind = item[0]

        if kind == "SYNC":
            _, token, received_us = item
            sent_us = self.clock.now()
            payload = ("SYNC {} {} {}\r\n".format(
                token, received_us, sent_us)).encode()
        elif kind == "TEXT":
            payload = (item[1] + "\r\n").encode()
        else:
            payload = item[1]

        written = self.lora.write(payload)
        if written != len(payload):
            self._stop_all(laser_off=True)
            self.tx_queue[:] = []
            self.tx_queue.append(("TEXT", "ERROR UART_WRITE"))
            return

        del self.tx_queue[0]

    # -------------------- CCD helpers --------------------

    def drain_ccd(self):
        self.ccd_buf[:] = b""
        # Bounded by the currently available UART bytes only; never wait here.
        while self.ccd.any():
            self.ccd.read(min(256, self.ccd.any()))

    def apply_fixed_exposure(self):
        """Always program exposure index 3. No readback exists in the guide."""
        request = ("@c{:04d}#@".format(EXPOSURE_INDEX)).encode()
        written = self.ccd.write(request)
        return written == len(request)

    def _cancel_capture(self):
        self.capture = None
        self.ccd_buf[:] = b""

    def _start_capture(self, owner, mode, **extra):
        if self.capture is not None:
            return False
        if mode not in ("fffe", "raw2"):
            return False

        self.drain_ccd()
        command = b"@c0071#@" if mode == "fffe" else b"@c0081#@"
        begin = self.clock.now()
        written = self.ccd.write(command)
        if written != len(command):
            return False

        capture = {
            "owner": owner,
            "mode": mode,
            "begin": begin,
        }
        capture.update(extra)
        self.capture = capture
        return True

    def _extract_complete_frame(self):
        """Return (pixel, raw_frame) once one strict frame is complete."""
        if self.capture is None:
            return None

        mode = self.capture["mode"]

        if mode == "fffe":
            # Strict legacy frame: FF FE high low.
            index = self.ccd_buf.find(b"\xff\xfe")
            if index < 0:
                # Preserve a trailing FF because it may be the first header byte
                # split across UART reads.
                if len(self.ccd_buf) > 1:
                    if self.ccd_buf[-1] == 0xFF:
                        last = self.ccd_buf[-1]
                        self.ccd_buf[:] = bytes((last,))
                    else:
                        self.ccd_buf[:] = b""
                return None
            if index > 0:
                if self.debug:
                    self.usb_print("CCD DROPPED PREFIX HEX: " +
                                   hex_text(self.ccd_buf[:index]))
                del self.ccd_buf[:index]
            if len(self.ccd_buf) < 4:
                return None

            frame = bytes(self.ccd_buf[:4])
            value = (frame[2] << 8) | frame[3]
            return value, frame

        # raw2: exactly the first big-endian 16-bit coordinate of this reply.
        if len(self.ccd_buf) < 2:
            return None
        frame = bytes(self.ccd_buf[:2])
        value = (frame[0] << 8) | frame[1]
        return value, frame

    def poll_ccd(self, now):
        if self.ccd.any():
            data = self.ccd.read(min(256, self.ccd.any()))
            if data:
                if self.capture is None:
                    # Exposure replies or stale bytes are not measurements.
                    if self.debug:
                        self.usb_print("CCD UNSOLICITED HEX: " + hex_text(data))
                else:
                    self.ccd_buf.extend(data)
                    if len(self.ccd_buf) > CCD_MAX_RESPONSE:
                        self._capture_failed("RESPONSE_TOO_LONG")
                        return

        if self.capture is None:
            return

        complete = self._extract_complete_frame()
        if complete is not None:
            value, raw_frame = complete
            end = self.clock.now()
            capture = self.capture
            self.capture = None
            self.ccd_buf[:] = b""
            self._capture_complete(capture, value, raw_frame, end)
            return

        if now - self.capture["begin"] > CCD_TIMEOUT_US:
            self._capture_failed("CCD_TIMEOUT")

    def _capture_failed(self, reason):
        capture = self.capture
        self.capture = None
        self.ccd_buf[:] = b""
        if capture is None:
            return

        owner = capture["owner"]
        if reason == "CCD_TIMEOUT":
            self.total_ccd_timeouts += 1
        else:
            self.total_invalid_frames += 1

        # A single failed range sample must not invalidate the whole sweep.
        # For SCAN we emit no PIX packet; the skipped sequence becomes a normal
        # missing range sample which the PC-side receiver already tolerates.
        if owner == "SCAN":
            if self.debug:
                self.usb_print("SCAN SAMPLE {} DROPPED: {}".format(
                    capture["seq"], reason))
            return

        if owner == "CAL":
            token = capture["token"]
            source = capture["source"]
            self.respond(source, "ERROR CAL_" + reason)
            self._finish_calibration()
            return

        if owner in ("MANUAL", "BATCH"):
            source = capture["source"]
            self.respond(source, "CCD INVALID: " + reason)
            if owner == "BATCH":
                self._batch_attempt_finished(None)
            return

        if owner == "LEGACY":
            if self.debug:
                self.usb_print("LEGACY CCD INVALID: " + reason)

    def _capture_complete(self, capture, value, raw_frame, end):
        pixel = value if 0 <= value <= COORDINATE_MAX else -1
        owner = capture["owner"]
        begin = capture["begin"]

        if owner == "SCAN":
            seq = capture["seq"]
            self.total_scan_packets += 1
            self.protocol_text("PIX {} {} {} {} {}".format(
                self.session, seq, begin, end, pixel))
            return

        if owner == "CAL":
            self.respond(
                capture["source"],
                "PIX {} 1 {} {} {}".format(
                    capture["token"], begin, end, pixel
                ),
            )
            self._finish_calibration()
            return

        if owner == "LEGACY":
            if pixel < 0:
                # Preserve frame shape but signal an invalid coordinate.
                if capture["mode"] == "fffe":
                    payload = b"\xff\xfe\xff\xff"
                else:
                    payload = b"\xff\xff"
            else:
                # Return the original strict frame, matching the legacy parser.
                payload = raw_frame
            self.protocol_bytes(payload)
            return

        if pixel < 0:
            self.respond(capture["source"], "CCD INVALID: COORDINATE_OUT_OF_RANGE")
            if owner == "BATCH":
                self._batch_attempt_finished(None)
            return

        if self.debug:
            self.respond(capture["source"], "CCD RAW HEX: " + hex_text(raw_frame))
            self.respond(capture["source"], "CCD FORMAT: " +
                         ("HEADER_BE16" if capture["mode"] == "fffe"
                          else "BARE_BE16"))

        self.respond(capture["source"], "CCD MIN X: {}".format(pixel))

        if owner == "BATCH":
            self._batch_attempt_finished(pixel)

    # -------------------- scan / calibration / manual state --------------------

    def _stop_all(self, laser_off=True):
        self.scan_active = False
        self.session = ""
        self.sequence = 0
        self.cal_pending = None
        self.batch = None
        self._cancel_capture()
        self.drain_ccd()
        if laser_off:
            self.set_laser(False)

    def stop(self, source):
        # If STOP is used to end a manual SAMPLE batch, keep the old useful
        # summary behavior. For normal synchronized scanning there is no batch.
        if self.batch is not None:
            self.finish_batch(stopped=True)
        self._stop_all(laser_off=True)
        # Drop queued PIX from the old session before acknowledging STOP.
        self.tx_queue[:] = []
        self.last_command_us = self.clock.now()
        self.respond(source, "OK STOP")

    def start_scan(self, source, session, rate_hz, requested_exposure, mode):
        if not session or mode not in ("fffe", "raw2"):
            self.respond(source, "ERROR START_ARGUMENTS")
            return
        if not math.isfinite(rate_hz) or not 1.0 <= rate_hz <= 100.0:
            self.respond(source, "ERROR START_ARGUMENTS")
            return

        self._stop_all(laser_off=True)
        # A new session must not inherit queued PIX from an older session.
        self.tx_queue[:] = []

        # The protocol keeps the exposure field for compatibility, but hardware
        # exposure is deliberately fixed at 3 as requested.
        if requested_exposure != EXPOSURE_INDEX:
            self.usb_print(
                "START requested exposure {}; hardware forced to {}".format(
                    requested_exposure, EXPOSURE_INDEX
                )
            )

        if not self.apply_fixed_exposure():
            self.respond(source, "ERROR EXPOSURE_WRITE")
            return

        now = self.clock.now()
        self.session = session
        self.center_mode = mode
        self.scan_period_us = max(10000, int(1000000.0 / rate_hz))
        self.next_sample_us = now + START_SETTLE_US
        self.sequence = 0
        self.scan_active = True
        self.set_laser(True)
        self.last_command_us = now

        self.respond(source, "OK START " + session)

    def start_calibration(self, source, token, requested_exposure, mode):
        if self.scan_active or self.capture is not None or self.batch is not None:
            self.respond(source, "ERROR BUSY")
            return
        if not token or mode not in ("fffe", "raw2"):
            self.respond(source, "ERROR CAL_ARGUMENTS")
            return

        if requested_exposure != EXPOSURE_INDEX:
            self.usb_print(
                "CAL requested exposure {}; hardware forced to {}".format(
                    requested_exposure, EXPOSURE_INDEX
                )
            )

        self.cal_restore_laser = int(self.laser_is_on())
        self.set_laser(True)

        if not self.apply_fixed_exposure():
            if not self.cal_restore_laser:
                self.set_laser(False)
            self.respond(source, "ERROR CAL_EXPOSURE_WRITE")
            return

        self.cal_pending = {
            "source": source,
            "token": token,
            "mode": mode,
            "ready_us": self.clock.now() + CAL_SETTLE_US,
        }
        self.last_command_us = self.clock.now()

    def _finish_calibration(self):
        self.cal_pending = None
        if not self.cal_restore_laser:
            self.set_laser(False)

    def poll_calibration(self, now):
        if self.cal_pending is None or self.capture is not None:
            return
        if now < self.cal_pending["ready_us"]:
            return

        item = self.cal_pending
        self.cal_pending = None
        if not self._start_capture(
            "CAL",
            item["mode"],
            source=item["source"],
            token=item["token"],
        ):
            self.respond(item["source"], "ERROR CAL_CCD_WRITE")
            self._finish_calibration()

    def poll_scan(self, now):
        if not self.scan_active or self.capture is not None:
            return
        if now < self.next_sample_us:
            return

        # If a previous capture/queue delay made us late, do not burst several
        # samples back-to-back. Skip missed schedule slots and continue.
        if now - self.next_sample_us >= self.scan_period_us:
            skipped = (now - self.next_sample_us) // self.scan_period_us
            self.sequence += int(skipped)
            self.next_sample_us += int(skipped) * self.scan_period_us

        self.sequence += 1
        seq = self.sequence
        self.total_scan_attempts += 1

        if not self._start_capture(
            "SCAN",
            self.center_mode,
            seq=seq,
        ):
            # This is a local write failure, not a malformed range packet.
            # Let the PC's "no measurement data for 2 s" fail-safe stop the run.
            if self.debug:
                self.usb_print("SCAN CCD WRITE FAILED")
        self.next_sample_us += self.scan_period_us

    def start_manual_measure(self, source, mode="fffe"):
        if self.scan_active or self.cal_pending is not None or self.batch is not None:
            self.respond(source, "ERR BUSY")
            return
        if self.capture is not None:
            self.respond(source, "ERR CCD BUSY")
            return
        if not self.laser_is_on():
            self.respond(source, "ERR LASER OFF; SEND LASER 1 FIRST")
            return
        if not self._start_capture("MANUAL", mode, source=source):
            self.respond(source, "CCD INVALID: CCD_WRITE_FAILED")

    def start_batch(self, source, count):
        if self.scan_active or self.cal_pending is not None or self.capture is not None:
            self.respond(source, "ERR BUSY")
            return
        if not self.laser_is_on():
            self.respond(source, "ERR LASER OFF; SEND LASER 1 FIRST")
            return

        self.batch = {
            "source": source,
            "count": count,
            "attempted": 0,
            "values": [],
            "next_us": self.clock.now(),
        }
        self.respond(
            source,
            "SAMPLE BEGIN: N={} EXPOSURE_FIXED={}".format(
                count, EXPOSURE_INDEX
            ),
        )

    def _batch_attempt_finished(self, value):
        if self.batch is None:
            return
        self.batch["attempted"] += 1
        if value is not None:
            self.batch["values"].append(value)

        if self.batch["attempted"] >= self.batch["count"]:
            self.finish_batch(False)
        else:
            self.batch["next_us"] = self.clock.now() + MANUAL_SAMPLE_INTERVAL_US

    def poll_batch(self, now):
        if self.batch is None or self.capture is not None:
            return
        if now < self.batch["next_us"]:
            return
        if not self._start_capture(
            "BATCH",
            "fffe",
            source=self.batch["source"],
        ):
            self.respond(self.batch["source"], "CCD INVALID: CCD_WRITE_FAILED")
            self._batch_attempt_finished(None)

    def finish_batch(self, stopped=False):
        batch = self.batch
        if batch is None:
            return

        # If a batch capture is currently in flight, cancel it.
        if self.capture is not None and self.capture["owner"] == "BATCH":
            self._cancel_capture()

        self.batch = None
        values = batch["values"]
        source = batch["source"]

        self.respond(
            source,
            "SAMPLE {}: REQUESTED={} ATTEMPTED={} PARSED={} INVALID={}".format(
                "STOPPED" if stopped else "END",
                batch["count"],
                batch["attempted"],
                len(values),
                batch["attempted"] - len(values),
            ),
        )

        stats = summarize(values)
        if stats is None:
            self.respond(source, "SUMMARY: NO PARSED COORDINATES")
            return

        median, mean, low, high, std = stats
        self.respond(
            source,
            "SUMMARY MEDIAN_X={:.2f} MEAN_X={:.2f}".format(median, mean),
        )
        self.respond(
            source,
            "SUMMARY MIN_X={} MAX_X={} SPAN_X={} STD_X={}".format(
                low,
                high,
                high - low,
                "NA" if std is None else "{:.2f}".format(std),
            ),
        )
        self.respond(
            source,
            "NOTE: PARSED DOES NOT VERIFY OPTICAL SIGNAL QUALITY",
        )

    # -------------------- command handling --------------------

    def status(self, source):
        self.respond(
            source,
            (
                "STATUS {} LASER={} EXPOSURE_FIXED={} SCAN={} BATCH={} SESSION={} "
                "MODE={} RATE_HZ={:.3f} SEQ={} TXQ={} DEBUG={} INPUT={} "
                "SCAN_ATTEMPTS={} PIX_SENT={} CCD_TIMEOUTS={} INVALID_FRAMES={}"
            ).format(
                VERSION,
                int(self.laser_is_on()),
                EXPOSURE_INDEX,
                int(self.scan_active),
                int(self.batch is not None),
                self.session if self.session else "-",
                self.center_mode,
                1000000.0 / self.scan_period_us,
                self.sequence,
                len(self.tx_queue),
                int(self.debug),
                self.input_mode,
                self.total_scan_attempts,
                self.total_scan_packets,
                self.total_ccd_timeouts,
                self.total_invalid_frames,
            ),
        )

    def help(self, source="USB"):
        lines = (
            VERSION + " EXPOSURE FIXED AT 3",
            "SYNC: SYNC <token>",
            "SCAN: START <session> <rate_hz> <exposure_field> <fffe|raw2>",
            "CAL: CAL <token> <exposure_field> <fffe|raw2>",
            "CMDS: PING, STOP, LASER 1, LASER 0, STATUS",
            "CMDS: MIN (or X/MEASURE), SAMPLE [n] (1..100)",
            "CMDS: DEBUG 1, DEBUG 0, INPUT USB/LORA/BOTH, HELP",
            "EXPOSURE: fixed at 3; EXPOSURE 3 may be sent again",
            "LEGACY: @c0071#@ or @c0081#@ while synchronized scan is stopped",
        )
        for line in lines:
            self.respond(source, line)

    def handle_command(self, raw, source):
        try:
            text = raw.decode().strip()
        except UnicodeError:
            self.respond(source, "ERROR COMMAND_ENCODING")
            return

        if not text:
            return

        upper = text.upper()
        parts = text.split()
        command = parts[0].upper()

        if self.debug:
            self.usb_print("COMMAND SOURCE={} TEXT={}".format(source, text))

        # --- protocol commands expected by the PC ---

        if command == "SYNC" and len(parts) == 2:
            received_us = self.clock.now()
            self.last_command_us = received_us
            self.queue_sync_reply(source, parts[1], received_us)
            return

        if command == "PING" and len(parts) == 1:
            self.last_command_us = self.clock.now()
            self.respond(source, "PONG")
            return

        if command == "START" and len(parts) == 5:
            try:
                rate_hz = float(parts[2])
                requested_exposure = int(parts[3])
            except (ValueError, OverflowError):
                self.respond(source, "ERROR START_ARGUMENTS")
                return
            self.start_scan(
                source,
                parts[1],
                rate_hz,
                requested_exposure,
                parts[4].lower(),
            )
            return

        if command == "CAL" and len(parts) in (3, 4):
            # Current PC sends CAL <token> <exposure> <mode>.
            # A 3-field form is also accepted as CAL <token> <mode>.
            try:
                if len(parts) == 4:
                    requested_exposure = int(parts[2])
                    mode = parts[3].lower()
                else:
                    requested_exposure = EXPOSURE_INDEX
                    mode = parts[2].lower()
            except (ValueError, OverflowError):
                self.respond(source, "ERROR CAL_ARGUMENTS")
                return
            self.start_calibration(
                source,
                parts[1],
                requested_exposure,
                mode,
            )
            return

        if command == "STOP" and len(parts) == 1:
            self.stop(source)
            return

        if command == "LASER" and len(parts) == 2 and parts[1] in ("0", "1"):
            self.last_command_us = self.clock.now()
            if parts[1] == "0":
                # Explicit laser-off is fail-safe: cancel manual/CAL capture.
                if self.scan_active:
                    self._stop_all(laser_off=True)
                else:
                    self.cal_pending = None
                    self.batch = None
                    self._cancel_capture()
                    self.set_laser(False)
            else:
                self.set_laser(True)
            self.respond(source, "OK LASER " + parts[1])
            return

        # --- manual/debug commands ---

        if command in ("HELP", "?"):
            self.last_command_us = self.clock.now()
            self.help(source)
            return

        if command == "STATUS":
            self.last_command_us = self.clock.now()
            self.status(source)
            return

        if upper in ("DEBUG 0", "DEBUG 1"):
            self.debug = upper.endswith("1")
            self.last_command_us = self.clock.now()
            self.respond(source, "OK " + upper)
            return

        if upper in ("INPUT USB", "INPUT LORA", "INPUT BOTH"):
            mode = upper.split()[1]
            self.input_mode = mode
            for channel in self.buffers:
                self.buffers[channel][:] = b""
                self.discarding[channel] = False
            self.last_command_us = self.clock.now()
            self.respond(source, "OK INPUT " + mode)
            return

        if command == "EXPOSURE":
            if len(parts) == 2 and parts[1] == str(EXPOSURE_INDEX):
                if self.scan_active or self.capture is not None:
                    self.respond(source, "ERR BUSY")
                    return
                if self.apply_fixed_exposure():
                    self.respond(source, "EXPOSURE SENT: 3 (NO READBACK VERIFICATION)")
                else:
                    self.respond(source, "ERR EXPOSURE CCD_WRITE_FAILED")
            else:
                self.respond(source, "ERR EXPOSURE FIXED AT 3")
            self.last_command_us = self.clock.now()
            return

        if command in ("MIN", "X", "MEASURE"):
            self.last_command_us = self.clock.now()
            self.start_manual_measure(source, "fffe")
            return

        if command == "SAMPLE":
            if self.batch is not None:
                self.respond(source, "ERR SAMPLE RUNNING; SEND STOP FIRST")
                return
            if len(parts) == 1:
                count = 20
            elif len(parts) == 2 and parts[1].isdigit():
                count = int(parts[1])
            else:
                count = 0
            if not 1 <= count <= 100:
                self.respond(source, "ERR SAMPLE USE 1..100")
                return
            self.last_command_us = self.clock.now()
            self.start_batch(source, count)
            return

        # --- legacy raw CCD commands, for the old radar viewer ---

        if upper == "@C0071#@":
            if self.scan_active or self.capture is not None:
                self.respond(source, "ERR BUSY")
                return
            if not self.laser_is_on():
                # The old PC normally sends LASER 1 before requesting pixels.
                self.respond(source, "ERR LASER OFF")
                return
            self.last_command_us = self.clock.now()
            if not self._start_capture("LEGACY", "fffe", source=source):
                self.respond(source, "ERR CCD_WRITE_FAILED")
            return

        if upper == "@C0081#@":
            if self.scan_active or self.capture is not None:
                self.respond(source, "ERR BUSY")
                return
            if not self.laser_is_on():
                self.respond(source, "ERR LASER OFF")
                return
            self.last_command_us = self.clock.now()
            if not self._start_capture("LEGACY", "raw2", source=source):
                self.respond(source, "ERR CCD_WRITE_FAILED")
            return

        # The legacy PC sends an exposure command like @c0008#@. We accept the
        # shape but intentionally program exposure 3 regardless.
        if (len(upper) == 8 and upper.startswith("@C") and
                upper.endswith("#@") and upper[2:6].isdigit()):
            value = int(upper[2:6])
            if 0 <= value <= 13:
                self.last_command_us = self.clock.now()
                if value != EXPOSURE_INDEX:
                    self.usb_print(
                        "LEGACY requested exposure {}; forced to 3".format(value)
                    )
                if self.apply_fixed_exposure():
                    # Raw exposure commands historically expect no radio text.
                    if source == "USB":
                        self.usb_print(
                            "EXPOSURE SENT: 3 (NO READBACK VERIFICATION)"
                        )
                else:
                    self.respond(source, "ERR EXPOSURE CCD_WRITE_FAILED")
                return

        # Unknown LoRa data is diagnosed on USB only, avoiding an accidental
        # wireless echo/noise loop.
        self.usb_print(
            "ERR UNKNOWN COMMAND SOURCE={} RAW_HEX={}".format(
                source, hex_text(raw[:48])
            )
        )

    # -------------------- input parsing --------------------

    def feed(self, source, data):
        for byte in data:
            # Recheck on every byte because an INPUT command may change the
            # accepted source while more bytes are still buffered in the same
            # UART chunk.
            if self.input_mode not in (source, "BOTH"):
                self.buffers[source][:] = b""
                self.discarding[source] = False
                return

            if byte in (10, 13):
                if not self.discarding[source] and self.buffers[source]:
                    raw = bytes(self.buffers[source])
                    self.buffers[source][:] = b""
                    self.handle_command(raw, source)
                else:
                    self.buffers[source][:] = b""
                self.discarding[source] = False
            elif byte == 0 or self.discarding[source]:
                continue
            elif byte in (8, 127):
                if self.buffers[source]:
                    del self.buffers[source][-1]
            elif len(self.buffers[source]) >= COMMAND_MAX_LENGTH:
                self.usb_print(
                    "ERR COMMAND TOO LONG SOURCE={} RAW_HEX={}".format(
                        source, hex_text(self.buffers[source][:48])
                    )
                )
                self.buffers[source][:] = b""
                self.discarding[source] = True
            else:
                self.buffers[source].append(byte)

    def poll_inputs(self):
        if self.lora.any():
            data = self.lora.read(min(256, self.lora.any()))
            if data:
                self.last_lora_ms = time.ticks_ms()
                self.feed("LORA", data)

        # If a radio sender omitted the final newline, close the line after
        # 200 ms of silence, matching the previous debug firmware behavior.
        if time.ticks_diff(time.ticks_ms(), self.last_lora_ms) > 200:
            if self.buffers["LORA"] or self.discarding["LORA"]:
                self.feed("LORA", b"\n")

        if self.usb_poll is not None:
            for _ in range(64):
                events = self.usb_poll.poll(0)
                if not events:
                    break
                flags = events[0][1]
                if flags & (select.POLLERR | select.POLLHUP):
                    try:
                        self.usb_poll.unregister(sys.stdin)
                    except Exception:
                        pass
                    self.usb_poll = None
                    break
                if not flags & select.POLLIN:
                    break
                char = sys.stdin.read(1)
                if not char:
                    break
                self.feed("USB", char.encode())

    # -------------------- watchdog / main loop --------------------

    def poll_watchdog(self, now):
        if self.scan_active and now - self.last_command_us > WATCHDOG_US:
            self._stop_all(laser_off=True)
            self.protocol_text("ERROR WATCHDOG")

    def boot(self):
        # Allow the CCD module to power up before the first exposure command.
        time.sleep_ms(600)
        self.set_laser(False)
        self.drain_ccd()

        if not self.apply_fixed_exposure():
            self.usb_print("BOOT WARNING: EXPOSURE WRITE FAILED")
        time.sleep_ms(150)
        self.drain_ccd()

        try:
            self.usb_poll = select.poll()
            self.usb_poll.register(sys.stdin, select.POLLIN)
        except (OSError, TypeError, ValueError):
            self.usb_poll = None
            self.usb_print("USB INPUT UNAVAILABLE")

        ready = "READY " + VERSION
        self.usb_print(ready)
        self._queue_lora(("TEXT", ready))

        # Keep full help on USB. LoRa receives only READY at boot.
        self.help("USB")

    def run(self):
        self.boot()
        try:
            while True:
                now = self.clock.now()

                # Commands first, so STOP/SYNC/PING are serviced promptly even
                # while a CCD request is outstanding.
                self.poll_inputs()

                now = self.clock.now()
                self.poll_ccd(now)

                now = self.clock.now()
                self.poll_calibration(now)
                self.poll_batch(now)
                self.poll_scan(now)
                self.poll_watchdog(now)

                self.flush_tx()
                time.sleep_ms(LOOP_SLEEP_MS)
        finally:
            self._stop_all(laser_off=True)


if __name__ == "__main__":
    MeasurementNode().run()
