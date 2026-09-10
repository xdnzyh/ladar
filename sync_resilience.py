from __future__ import annotations

import math

import synchronized_acquisition as base


class ResilientSynchronizedAcquisition(base.SynchronizedAcquisition):
    """Hardware runtime policy for continuous clock maintenance.

    Initial 8-exchange synchronization remains strict.  Once a scan session is
    running, missing maintenance SYNC replies no longer stop a healthy ranging
    and TRIG stream merely because the last clock refresh is older than eight
    seconds.  The existing ClockEstimate.map() already expands timestamp
    uncertainty with clock age, so stale timing automatically loses mapping
    weight while probes continue in the background.

    If a later maintenance exchange is incompatible with the old drift-bounded
    interval, the clock model is explicitly rebased and the one in-progress
    revolution is discarded.  The session itself stays alive.
    """

    def _running_sync_poll(self, now):
        if now - self.last_keepalive >= float(self.config.get("keepalive_interval_s", 5)):
            for source, endpoint in self.endpoints.items():
                if not endpoint.write_line("PING"):
                    self._fail(f"采集失败：{self._endpoint_label(source)}保活指令发送失败")
                    return
            self.last_keepalive = now

        stale_limit = float(self.config.get("sync_max_age_s", 8))
        for source, endpoint in self.endpoints.items():
            clock_age = now - self.clocks[source].observed_at
            if clock_age > stale_limit:
                # Do not convert a maintenance-sync hiccup into an emergency
                # stop while real measurement/TRIG data is still flowing.  The
                # normal data watchdogs in poll() remain authoritative.
                self.next_probe[source] = min(self.next_probe[source], now)
                self._diagnostic(
                    f"{self._endpoint_label(source)}持续校时暂未刷新 {clock_age:.1f}s；"
                    "继续采集并按时钟漂移上界扩大时间不确定度",
                    timestamp=now,
                )

            if source in self.outstanding:
                token, queued_at = self.outstanding[source]
                if now - queued_at < base.SYNC_RESPONSE_TIMEOUT_S:
                    continue
                self.sent_times.pop((source, token), None)
                del self.outstanding[source]
                self._stat(source, "timeout")

            if now < self.next_probe[source]:
                continue
            self.attempts[source] += 1
            token = f"{self.session}-{self.attempts[source]}"
            self.outstanding[source] = token, now
            self.next_probe[source] = now + float(self.config.get("sync_interval_s", 0.5))
            generation = self.generation
            if not endpoint.write_line(
                    f"SYNC {token}",
                    lambda t, g=generation, s=source, k=token:
                    self.incoming.put(("sent", (g, s, k), t))):
                self._fail(f"持续校时失败：{self._endpoint_label(source)}串口发送失败")
                return
            self._stat(source, "sent")

    def _line(self, source, line, arrival):
        parts = line.split()
        if (self.state == "running" and len(parts) == 4
                and parts[0].upper() == "SYNC"):
            token = parts[1]
            if self.outstanding.get(source, (None,))[0] != token:
                self._record_ignored(source, line, "校时 token 不匹配", arrival, token_mismatch=True)
                return
            t1 = self.sent_times.pop((source, token), None)
            if t1 is None:
                self._stat(source, "missing_send_time")
                self._record_ignored(source, line, "缺失发送时间", arrival)
                return
            try:
                estimate = base.ClockEstimate.exchange(
                    t1,
                    int(parts[2]),
                    int(parts[3]),
                    arrival,
                    float(self.config.get("clock_drift_bound_ppm", 500)),
                )
            except (ValueError, OverflowError):
                self._stat(source, "invalid_timestamp")
                self._record_ignored(source, line, "时间戳无效", arrival)
                return

            self.last_communication[source] = arrival
            previous = self.clocks[source]
            try:
                updated = previous.updated(estimate)
            except ValueError:
                # Rebase explicitly instead of widening an incompatible
                # interval.  Discard only the mixed-clock revolution.
                updated = base.ClockEstimate(
                    estimate.offset,
                    max(estimate.uncertainty, 1e-6),
                    estimate.observed_at,
                    max(previous.drift_ppm, estimate.drift_ppm),
                    previous.version + 1,
                )
                self.receiver.invalidate("持续校时已重建时钟基准，当前圈丢弃，等待下一真实零位")
                self._diagnostic(
                    f"{self._endpoint_label(source)}持续校时区间不重叠；"
                    "已重建时钟基准并仅丢弃当前圈，采集继续",
                    force=True,
                    timestamp=arrival,
                )
            self.clocks[source] = updated
            self.next_probe[source] = arrival + float(self.config.get("sync_interval_s", 0.5))
            self._stat(source, "success")
            del self.outstanding[source]
            return
        super()._line(source, line, arrival)


def install_for_hardware_runtime() -> None:
    """Install the resilient class before navigation_app imports the symbol."""
    base.SynchronizedAcquisition = ResilientSynchronizedAcquisition
