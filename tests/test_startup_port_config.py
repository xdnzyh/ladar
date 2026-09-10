import unittest
from unittest.mock import Mock, patch

from navigation_app import NavigationApp, load_configuration
from runtime_config import RuntimeConfigError, build_navigation_engine, resolve_runtime_config


class StartupPortConfigTests(unittest.TestCase):
    def test_saved_duplicate_ports_can_be_loaded_for_ui_correction(self):
        for view in ('radar', 'navigation'):
            for ports in (
                {'measurement_port': 'COM3', 'rotation_port': 'com3', 'chassis_port': 'COM5'},
                {'measurement_port': 'COM3', 'rotation_port': 'COM4', 'chassis_port': 'COM3'},
            ):
                with self.subTest(view=view, ports=ports):
                    with patch('navigation_app.load_json_config', side_effect=[{}, ports]):
                        config = load_configuration('hardware', view)
                    for key, value in ports.items():
                        self.assertEqual(config[key], value)
                    self.assertIsNotNone(build_navigation_engine(config))

    def test_runtime_configuration_still_rejects_duplicate_ports(self):
        with self.assertRaises(RuntimeConfigError):
            resolve_runtime_config('hardware', 'navigation', {
                'measurement_port': 'COM3', 'rotation_port': 'com3',
            })

    def test_connect_rejects_duplicates_before_opening_any_endpoint(self):
        app = object.__new__(NavigationApp)
        app.measure_port_var = Mock(get=lambda: 'COM3')
        app.rotation_port_var = Mock(get=lambda: 'com3')
        app.chassis_port_var = Mock(get=lambda: 'COM5')
        app.view_mode = Mock(get=lambda: 'radar')
        app.measure_endpoint = Mock()
        app.rotation_endpoint = Mock()
        app.chassis_endpoint = Mock()
        with patch('navigation_app.messagebox.showwarning') as warning:
            app.connect()
        warning.assert_called_once()
        for endpoint in (app.measure_endpoint, app.rotation_endpoint, app.chassis_endpoint):
            endpoint.open.assert_not_called()
