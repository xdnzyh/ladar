from __future__ import annotations

import binascii
from dataclasses import replace
import heapq
import itertools
import math
from types import SimpleNamespace

from chassis_controller import ChassisController, ChassisMotionAdapter, ChassisState
from navigation_core import VelocityCommand


class SimulationChassis:
    WHEELS = {
        "W": (1, 1, 1, 1), "S": (-1, -1, -1, -1),
        "A": (1, -1, 1, -1), "D": (-1, 1, -1, 1),
        "Q": (1, 0, 1, 0), "E": (0, 1, 0, 1),
        "Z": (0, -1, 0, -1), "C": (-1, 0, -1, 0),
    }
    is_open = True

    def __init__(self, simulator, config, runtime, safety, *, drop_ack=False, drop_result=False):
        self.simulator, self.runtime, self.safety = simulator, runtime, safety
        self.config = dict(config, chassis_distance_control=True,
                           chassis_firmware_confirmed=True, chassis_capability_mode="mm_ping_v1",
                           chassis_idle_preflight=True, chassis_result_recovery=True,
                           chassis_config1_file="")
        self.adapter = ChassisMotionAdapter(self.config)
        self.drop_ack, self.drop_result = drop_ack, drop_result
        self.writes = []
        self.events = []
        self._order = itertools.count()
        self._nonces = itertools.count(1)
        self.armed = None
        self.cached_result = None
        self.command = None
        self.request = None
        self.done_at = math.inf
        self.scan_after = -math.inf
        self.settle_at = math.inf
        self.failure = None
        self.priors_applied = 0
        self.controller = ChassisController(
            self, self._emit, self.config, clock=lambda: simulator.time,
            nonce_factory=lambda: f"{next(self._nonces):08X}")
        self.generation = self.controller.begin_connection()
        self.controller.feed_data(b"IDLE X=0 Y=0 R=0 S=0\r\n", simulator.time, self.generation)
        if not self.controller.allow_automatic():
            raise RuntimeError("模拟底盘初始化失败")

    @staticmethod
    def checked(body):
        crc = binascii.crc_hqx(body.encode("ascii"), 0xFFFF)
        return f"{body},{crc:04X}\r\n".encode("ascii")

    def _schedule(self, data, at):
        heapq.heappush(self.events, (at, next(self._order), data))

    def write_ticket(self, data, **callbacks):
        now = self.simulator.time
        self.writes.append((now, bytes(data)))
        for name in ("on_sent", "on_written"):
            if callbacks.get(name):
                callbacks[name](now)
        wire = data.strip().decode("ascii")
        parts = wire.split(",")
        if parts[0] == "@PING":
            self.armed = parts[1]
            self._schedule(f"@PONG,{self.armed}\r\n".encode(), now + 0.01)
        elif parts[0] in {"@MOVE", "@RESULT"}:
            if self.checked(wire.rsplit(",", 1)[0]).strip() != data.strip():
                raise ValueError("模拟底盘命令 CRC 错误")
            if parts[0] == "@MOVE":
                if len(parts) != 6 or parts[4] != self.armed or self.command is None:
                    raise ValueError("模拟底盘 MOVE 未匹配当前握手")
                mode, value, unit, nonce = parts[1:5]
                if (mode, int(value), unit) != self.request.request_identity:
                    raise ValueError("模拟底盘 MOVE 与登记动作不一致")
                self.armed = None
                self.simulator.execute(self.command)
                self.safety.start(self.command, now)
                self.done_at = now + self.command.duration_s
                enc = self.request.target_counts
                wheels = ",".join(str(sign * enc) for sign in self.WHEELS[mode])
                self.cached_result = self.checked(
                    f"@RESULT,{nonce},{mode},0,{value},{unit},{enc:.2f},{enc:.2f},{wheels}")
                if not self.drop_ack:
                    self._schedule(f"@ACK,{mode},{value},{unit}\r\n".encode(), now + 0.01)
                if not self.drop_result:
                    self._schedule(self.cached_result, self.done_at + 0.01)
            else:
                response = (self.checked(f"@RESULT,{parts[1]},B") if now < self.done_at
                            else self.cached_result or self.checked(f"@RESULT,{parts[1]},N"))
                self._schedule(response, now + 0.01)
        elif "!" in wire:
            self.simulator.stop()
            self.failure = self.failure or "底盘事务已停止"
            self.events.clear()
        return SimpleNamespace(state="written")

    def cancel_write(self, ticket):
        return "started"

    def _emit(self, kind, value, stamp):
        if kind == "chassis_frame":
            self.controller.handle_frame(*value, stamp)
        elif kind == "chassis_done":
            report = value[2]
            estimate = self.adapter.execution_from_report(report)
            self.runtime.apply_execution_delta(
                estimate.local_x_m, estimate.local_y_m, estimate.yaw_rad,
                estimate.uncertainty_m, estimate.uncertainty_rad)
            self.priors_applied += 1
            self.safety.clear()
            self.settle_at = stamp + float(self.config.get("simulation_settle_s", 0.2))
            self.simulator.receiver.reset(self.settle_at)
        elif kind == "chassis_fault":
            self.failure = str(value)

    def execute(self, command):
        self.request = self.adapter.request_for_command(command, automatic=True)
        speed = math.hypot(command.forward_mps, command.right_mps)
        distance = self.request.target_counts / self.config["chassis_translation_capabilities"][self.request.mode]["counts_per_mm"] / 1000
        self.command = replace(command, duration_s=distance / speed)
        self.runtime.invalidate()
        if not self.controller.request_move(self.request, source="auto"):
            raise RuntimeError("模拟底盘拒绝登记动作")

    def poll(self):
        now = self.simulator.time
        self.controller.poll(now)
        while self.events and self.events[0][0] <= now:
            _, _, data = heapq.heappop(self.events)
            self.controller.feed_data(data, now, self.generation)
        if self.controller.state == ChassisState.SETTLING and now >= self.settle_at:
            self.scan_after = self.settle_at
            self.controller.complete_settle(resume_auto=True, now=now)

    @property
    def can_map(self):
        return self.controller.state in {ChassisState.IDLE, ChassisState.WAITING_SCAN}

    def accept_scan(self, mapped):
        if self.controller.state != ChassisState.WAITING_SCAN:
            return True
        request, snapshot = mapped.request, mapped.snapshot
        if (snapshot is None or not snapshot.scan_accepted
                or request.generation != self.runtime.generation
                or request.session != "simulation"
                or request.scan_start_s is None or request.scan_end_s is None
                or request.scan_start_s < self.scan_after or request.scan_end_s < self.scan_after):
            return False
        return self.controller.mark_scan_ready()

    def stop(self, reason):
        self.failure = reason
        self.controller.request_stop(reason=reason)

    def diagnostics(self):
        return {"chassis_transaction_state": self.controller.state,
                "chassis_moves": sum(data.lstrip().startswith(b"@MOVE,") for _, data in self.writes),
                "chassis_result_queries": sum(data.lstrip().startswith(b"@RESULT,") for _, data in self.writes),
                "chassis_priors_applied": self.priors_applied,
                "chassis_failure": self.failure}
