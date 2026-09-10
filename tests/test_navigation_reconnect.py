import queue
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from navigation_app import NavigationApp
from navigation_core import NavigationEngine, OccupancyGrid
from radar_core import PolarPoint


class NavigationReconnectTests(unittest.TestCase):
    def make_app(self):
        app = object.__new__(NavigationApp)
        app.source = "hardware"
        app.config = {"synchronized_acquisition": True}
        app.connected = True
        app.running = True
        app.accept_samples = True
        app.moving = False
        app.manual_motion = False
        app.motion_generation = 1
        app.mapping_generation = 1
        app.mapping_lock = threading.RLock()
        app.mapping_tasks = queue.Queue()
        app.mapping_results = queue.Queue()
        app.mapping_snapshot = None
        app.navigator = NavigationEngine(OccupancyGrid(40, 40, 0.05))
        app.navigator.set_auto(True)
        app.chassis_controller = SimpleNamespace(connection_generation=3, in_flight=False)
        app.motion_safety = Mock()
        app._log = Mock()
        app.sync = Mock()
        app.start_button = Mock()
        app.connection_label = Mock()
        app.view_mode = Mock()
        app.view_mode.get.return_value = "navigation"
        return app

    def test_previous_connection_fault_cannot_stop_new_scan(self):
        for kind, value in (
            ("chassis_fault", (1, "旧连接故障")),
            ("chassis_timeout", (1, None, "旧连接超时")),
            ("chassis_unmatched_done", (1, None)),
        ):
            with self.subTest(kind=kind):
                app = self.make_app()
                app._handle_event(kind, value, 1.0)
                self.assertTrue(app.running, "旧连接故障关闭了重连后的扫描")
                self.assertTrue(app.accept_samples)

    def test_current_fault_invalidates_mapping_and_stops_acquisition(self):
        app = self.make_app()
        app._handle_event("chassis_fault", (3, "当前故障"), 2.0)
        self.assertFalse(app.running)
        self.assertFalse(app.navigator.auto_enabled)
        self.assertGreater(app.mapping_generation, 1)
        app.sync.stop.assert_called_once()
        app.start_button.configure.assert_called_with(text="开始")

    def test_restart_after_reconnect_clears_old_motion_gate(self):
        app = self.make_app()
        app.running = False
        app.moving = True  # A fault left the old STOP transaction in flight.
        app.calibration = SimpleNamespace(ready=True)
        app._settle_duration = Mock(return_value=0.1)
        app._navigation_preflight = Mock(return_value=True)
        app._start_hardware_scan = Mock()
        app.start()
        self.assertTrue(app.running)
        self.assertFalse(app.moving, "重连后残留 moving 会丢弃全部 sync_sweep")
        app.sync.session = "reconnected"
        app.scan_collect_after = 2.0
        app.rotation = SimpleNamespace(period_history=[])
        app.scan_rate = Mock()
        app._submit_mapping = Mock()
        app._handle_event("sync_sweep", (
            "reconnected", 1, [PolarPoint(0.5, 0.0, 3.0, 800)], 1.0,
        ), 3.0)
        app._submit_mapping.assert_called_once()

    def test_start_does_not_clear_an_actual_in_flight_motion(self):
        app = self.make_app()
        app.running = False
        app.moving = True
        app.chassis_controller.in_flight = True
        app.calibration = SimpleNamespace(ready=True)
        app._settle_duration = Mock(return_value=0.1)
        app._navigation_preflight = Mock(return_value=True)
        app._start_hardware_scan = Mock()
        with patch("navigation_app.messagebox.showwarning") as warning:
            app.start()
        self.assertFalse(app.running)
        self.assertTrue(app.moving)
        app._start_hardware_scan.assert_not_called()
        warning.assert_called_once()


if __name__ == "__main__":
    unittest.main()
