from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, replace
import math
import threading
import time
from typing import Callable, Sequence

from mapping_policy import (TwoSweepWallEvidence, prepare_mapping_points,
                            complete_open_scan, prepare_free_space_points, process_radar_debug_scan)
from navigation_core import NavigationEngine, Pose2D, ScanPoint, VelocityCommand


@dataclass(frozen=True)
class MappingRequest:
    generation: int
    session: str
    scan_sequence: int
    scan_start_s: float | None
    scan_end_s: float | None
    mode: str
    base_state_version: int
    points: tuple[ScanPoint, ...]


@dataclass(frozen=True)
class MappingSnapshot:
    generation: int
    state_version: int
    map_version: int
    session: str
    scan_sequence: int | None
    mode: str
    grid: object
    pose: Pose2D
    path: tuple[tuple[int, int], ...]
    target: tuple[int, int] | None
    command: VelocityCommand
    state: str
    detail: str
    completed_scans: int
    local_map_updates: int
    rejected_scans: int
    dropped_requests: int
    queue_depth: int
    scan_accepted: bool = False
    accepted_input_points: int = 0
    match_score: float = 0.0
    frontier_count: int = 0
    reachable_frontier_count: int = 0


@dataclass(frozen=True)
class MappingResult:
    request: MappingRequest
    snapshot: MappingSnapshot | None
    command: VelocityCommand | None
    error: BaseException | None = None


class MappingRuntime:
    def __init__(
        self,
        navigator: NavigationEngine,
        *,
        min_range_m: float = 0.08,
        queue_size: int = 2,
        on_result: Callable[[MappingResult], None] | None = None,
        lock: threading.RLock | None = None,
    ) -> None:
        if queue_size < 1:
            raise ValueError("建图队列长度必须为正数")
        self.navigator = navigator
        self.min_range_m = float(min_range_m)
        self.queue_size = int(queue_size)
        self.on_result = on_result
        self.lock = lock or threading.RLock()
        self._condition = threading.Condition(self.lock)
        self._pending: deque[MappingRequest] = deque()
        self._stop = False
        self._generation = 0
        self._state_version = 0
        self._dropped_requests = 0
        self._wall_evidence = TwoSweepWallEvidence(navigator.grid.resolution_m)
        self._thread = threading.Thread(target=self._worker, name="navigation-mapper", daemon=True)
        self._thread.start()

    @property
    def generation(self) -> int:
        with self.lock:
            return self._generation

    @property
    def state_version(self) -> int:
        with self.lock:
            return self._state_version

    @property
    def dropped_requests(self) -> int:
        with self.lock:
            return self._dropped_requests

    @property
    def queue_depth(self) -> int:
        with self.lock:
            return len(self._pending)

    def submit(
        self,
        session: str,
        scan_sequence: int,
        points: Sequence[ScanPoint],
        timestamp_s: float | None = None,
        *,
        mode: str = "navigation",
        scan_start_s: float | None = None,
        scan_end_s: float | None = None,
    ) -> MappingRequest | None:
        if mode not in {"navigation", "local"}:
            raise ValueError("建图模式必须是 navigation 或 local")
        with self.lock:
            request = MappingRequest(
                self._generation,
                str(session),
                int(scan_sequence),
                scan_start_s if scan_start_s is not None else timestamp_s,
                scan_end_s if scan_end_s is not None else timestamp_s,
                mode,
                self._state_version,
                tuple(points),
            )
            if len(self._pending) >= self.queue_size:
                if mode == "local":
                    index = next((i for i, item in enumerate(self._pending) if item.mode == "local"), None)
                    if index is not None:
                        del self._pending[index]
                        self._dropped_requests += 1
                    else:
                        self._dropped_requests += 1
                        return None
                else:
                    index = next((i for i, item in enumerate(self._pending) if item.mode == "local"), None)
                    if index is not None:
                        del self._pending[index]
                        self._dropped_requests += 1
                    else:
                        self._dropped_requests += 1
                        return None
            self._pending.append(request)
            self._condition.notify()
            return request

    def invalidate(self, generation: int | None = None, *, reset: bool = False, reason: str = "") -> MappingSnapshot:
        with self.lock:
            self._generation = self._generation + 1 if generation is None else int(generation)
            self._state_version += 1
            self._pending.clear()
            self._wall_evidence.reset()
            if reset:
                self.navigator.reset()
            if reason:
                self.navigator.detail = reason
            snapshot = self._snapshot_locked("", None, "control", VelocityCommand())
            self._condition.notify_all()
        if self.on_result is not None:
            self.on_result(MappingResult(
                MappingRequest(snapshot.generation, "", -1, None, None, "control",
                                snapshot.state_version, ()),
                snapshot,
                VelocityCommand(),
            ))
        return snapshot

    def set_mode(self, mode: str) -> MappingSnapshot:
        return self.invalidate(reason="模式已切换")

    def predict_motion(self, command: VelocityCommand) -> None:
        with self.lock:
            self._state_version += 1
            self._pending.clear()
            self._wall_evidence.reset()
            self.navigator.predict_motion(command)

    def apply_execution_delta(self, *args: float) -> None:
        with self.lock:
            self._state_version += 1
            self._pending.clear()
            self._wall_evidence.reset()
            self.navigator.apply_execution_delta(*args)

    def stop(self, timeout: float = 1.0) -> None:
        with self.lock:
            self._stop = True
            self._pending.clear()
            self._generation += 1
            self._condition.notify_all()
        if self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=max(0.0, timeout))

    def _worker(self) -> None:
        while True:
            with self.lock:
                while not self._pending and not self._stop:
                    self._condition.wait(0.1)
                if self._stop:
                    return
                request = self._pending.popleft()
                if request.generation != self._generation:
                    continue
                # Queued scans contain sensor observations, not a computation
                # against the map at submit time. Start each one from the latest
                # committed map so the preceding scan cannot invalidate it.
                # Keep the version check at commit to reject work invalidated
                # by a stop, reset, or pose change while processing.
                request = replace(request, base_state_version=self._state_version)
                working = deepcopy(self.navigator)
                wall_evidence = deepcopy(self._wall_evidence)
            try:
                working._two_scan_action_attempted = False
                working._stationary_scan_attempts += 1
                scan_points = complete_open_scan(
                    request.points, working.unobserved_clear_range_m, working.max_range_m,
                    self.min_range_m, working.grid.resolution_m,
                    front_angle_rad=-working.sensor_offset_yaw_rad)
                free_rays = prepare_free_space_points(scan_points, working.max_range_m,
                                                      self.min_range_m, working.grid.resolution_m)
                selection = prepare_mapping_points(
                    request.points,
                    working.max_range_m,
                    self.min_range_m,
                    working.grid.resolution_m,
                )
                current_selection = selection
                fast_navigation = (working.immediate_navigation and working._map_initialized
                                   and request.mode == 'navigation' and working.auto_enabled)
                selection = wall_evidence.update(
                    request.session,
                    request.scan_sequence,
                    selection,
                )
                if fast_navigation and selection.confirmed_scans < 2:
                    selection = current_selection
                if request.mode == "local" or not working.auto_enabled:
                    command = process_radar_debug_scan(
                        working,
                        selection,
                        self.min_range_m,
                        free_space_points=free_rays,
                    )
                elif (selection.confirmed_scans < 2 and not fast_navigation) or selection.supported_echoes < 4:
                    if not working._motion_since_last_scan:
                        working.grid.update_scan(working._sensor_pose(), free_rays,
                                                 working.max_range_m, min_range_m=self.min_range_m,
                                                 add_only=True)
                    working.latest_scan = list(selection.points)
                    working.state = "两圈墙面确认"
                    working.detail = selection.rejection_reason or "等待连续两圈墙面证据"
                    command = VelocityCommand()
                    if fast_navigation or (working.local_probe_after_two_scans and working._stationary_scan_attempts >= 2):
                        working.latest_scan = list(scan_points)
                        command = working._reject_scan(0, "两圈墙面仍未确认")
                else:
                    # Wall fitting supplies obstacle evidence, but cannot describe
                    # open directions. Keep explicit, valid max-range observations.
                    clear_rays = tuple(
                        point for point in scan_points
                        if point.is_echo is False
                    )
                    command = working.process_scan(selection.points + clear_rays,
                                                   free_space_points=free_rays,
                                                   previous_wall_layer=(selection.mapping_layers[0]
                                                       if not fast_navigation and selection.mapping_passes == 2 and selection.mapping_layers else ()),
                                                   obstacle_points=request.points, add_only=True)
                if (request.mode == 'navigation' and working.auto_enabled
                        and working._two_scan_due()
                        and not working._two_scan_action_attempted):
                    command = working._two_scan_action()
                if request.mode == 'navigation' and working._two_scan_due():
                    if not command.stopped:
                        # This admits a checked action, not a successful global
                        # localization; retain the actual match score and map.
                        working.completed_scans = max(working.completed_scans,
                                                      self.navigator.completed_scans + 1)
            except BaseException as exc:
                result = MappingResult(request, None, None, exc)
            else:
                with self.lock:
                    if (request.generation != self._generation
                            or request.base_state_version != self._state_version
                            or self._stop):
                        continue
                    scan_accepted = (request.mode == "navigation" and working.auto_enabled
                                     and working.completed_scans > self.navigator.completed_scans)
                    self.navigator.__dict__ = deepcopy(working.__dict__)
                    self._wall_evidence = wall_evidence
                    self._state_version += 1
                    snapshot = self._snapshot_locked(
                        request.session, request.scan_sequence, request.mode, command, scan_accepted,
                    )
                    if scan_accepted:
                        clear_count = sum(not point.has_echo(working.max_range_m) for point in request.points
                                          if math.isfinite(point.distance_m) and math.isfinite(point.angle_rad)
                                          and point.quality >= working.grid.MIN_QUALITY
                                          and self.min_range_m <= point.distance_m <= working.max_range_m)
                        snapshot = replace(snapshot, accepted_input_points=min(
                            len(request.points), selection.supported_echoes + clear_count))
                result = MappingResult(request, snapshot, command)
            if self.on_result is not None:
                self.on_result(result)

    def _snapshot_locked(self, session: str, sequence: int | None, mode: str,
                         command: VelocityCommand, scan_accepted: bool = False) -> MappingSnapshot:
        return MappingSnapshot(
            self._generation,
            self._state_version,
            int(getattr(self.navigator.grid, "_revision", 0)),
            session,
            sequence,
            mode,
            deepcopy(self.navigator.grid),
            deepcopy(self.navigator.pose),
            tuple(self.navigator.path_cells),
            self.navigator.target_cell,
            command,
            self.navigator.state,
            self.navigator.detail,
            self.navigator.completed_scans,
            self.navigator.local_map_updates,
            self.navigator.rejected_scans,
            self._dropped_requests,
            len(self._pending),
            scan_accepted,
            match_score=self.navigator.match_score,
            frontier_count=self.navigator.frontier_count,
            reachable_frontier_count=self.navigator.reachable_frontier_count,
        )
