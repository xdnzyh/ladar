import math
import json
import queue
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from mapping_runtime import MappingRuntime
from navigation_app import NavigationApp
from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint
from radar_core import PolarPoint


class MappingQueueContinuityTests(unittest.TestCase):
    def test_two_consecutive_scans_in_one_batch_build_a_map(self):
        engine = NavigationEngine(OccupancyGrid(120, 120, 0.02), max_range_m=1)
        output = queue.Queue()
        runtime = MappingRuntime(engine, on_result=output.put)
        points = []
        for index in range(21):
            x, y = -0.25 + index * 0.025, 0.45
            points.append(ScanPoint(math.atan2(x, y), math.hypot(x, y), 1, True))
        try:
            # Serial data can release several closed revolutions in one UI poll.
            # Hold the shared lock so the worker sees the same deterministic batch.
            with runtime.lock:
                runtime.submit('session', 1, points, mode='local')
                runtime.submit('session', 2, points, mode='local')
            first = output.get(timeout=2)
            self.assertIsNone(first.error)
            try:
                second = output.get(timeout=2)
            except queue.Empty:
                self.fail('Second consecutive scan was lost; mapping stays at zero')
            self.assertIsNone(second.error)
            self.assertGreater(second.snapshot.completed_scans, 0)
            self.assertGreater(len(second.snapshot.grid.occupied_cells()), 0)
        finally:
            runtime.stop()

    def test_captured_wall_scans_reach_app_map_in_radar_mode(self):
        fixture = json.loads((Path(__file__).parent / 'fixtures' /
                              'radar_wall_scan_20260910.json').read_text())
        points = [PolarPoint(**point) for point in fixture['points']]
        app = object.__new__(NavigationApp)
        app.running, app.moving = True, False
        app.view_mode = Mock(get=lambda: 'radar')
        app.sync = SimpleNamespace(session='session')
        app.scan_collect_after = -math.inf
        app.scan_rate = Mock()
        app.rotation = SimpleNamespace(period_s=None, period_history=[])
        app.mapping_generation = 0
        app.mapping_results = queue.Queue()
        app.mapping_snapshot = None
        app.navigator = NavigationEngine(OccupancyGrid(120, 120, 0.02), max_range_m=1)
        app.chassis_controller = None
        app.mapping_runtime = MappingRuntime(app.navigator, min_range_m=0.15,
                                             on_result=app.mapping_results.put)
        try:
            with app.mapping_runtime.lock:
                for sequence in (1, 2):
                    shifted = [replace(point, timestamp=point.timestamp + sequence * 1.5)
                               for point in points]
                    app._handle_event('sync_sweep', ('session', sequence, shifted, 1.5),
                                      max(point.timestamp for point in shifted))
            results = [app.mapping_results.get(timeout=2) for _ in range(2)]
            for result in results:
                self.assertIsNone(result.error)
                app.mapping_results.put(result)
            app._handle_mapping_results()
            self.assertGreater(app.mapping_snapshot.completed_scans, 0)
            self.assertGreater(app.grid.known_area_m2(), 0)
            self.assertTrue(app.grid.occupied_cells())
            self.assertTrue(all(result.command.stopped for result in results))
        finally:
            app.mapping_runtime.stop()
