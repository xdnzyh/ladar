import queue
import types
import unittest
from unittest.mock import Mock, patch

from radar_app import RadarApp
from radar_core import CCDFrameParser, MotorLineParser


class CalibrationSessionTests(unittest.TestCase):
    def setUp(self):
        self.app = object.__new__(RadarApp)
        self.app.calibration_pending = False
        self.app.calibration_session = ''
        self.app.calibration_parser = MotorLineParser()
        self.app.ccd_parser = CCDFrameParser('raw2')
        self.app.measure_endpoint = types.SimpleNamespace(is_open=True, write_line=Mock(return_value=True))
        self.app.scanning = False
        self.app.demo = False
        self.app.events = queue.Queue()
        self.app.root = Mock()
        self.app.current_pixel_var = Mock()
        self.app._log = Mock()
        self.app._collect_config = lambda: {'ccd_parser': 'raw2', 'exposure_index': 8}

    def request(self, now):
        with patch('radar_app.time.perf_counter', return_value=now):
            self.app.read_calibration_pixel()
        command = self.app.measure_endpoint.write_line.call_args.args[0].split()
        return command[1] if len(command) == 4 else 'CAL'

    def test_old_reply_does_not_satisfy_new_calibration_request(self):
        old = self.request(0)
        with patch('radar_app.time.perf_counter', return_value=2.1):
            self.app._calibration_timeout()
        current = self.request(3)
        self.app._measurement_data(f'PIX {old} 1 200000 204000 800\n'.encode(), 3.1)
        self.assertTrue(self.app.calibration_pending)
        self.assertTrue(self.app.events.empty())
        self.app._measurement_data(f'PIX {current} 1 3200000 3204000 900\n'.encode(), 3.2)
        self.assertFalse(self.app.calibration_pending)
        self.assertEqual(self.app.events.get_nowait()[:2], ('calibration_pixel', (current, 900)))

    def test_reply_after_deadline_is_rejected_before_ui_timeout_callback(self):
        session = self.request(0)
        self.app._measurement_data(f'PIX {session} 1 200000 204000 800\n'.encode(), 2.1)
        self.assertFalse(any(event[0] in ('ccd_pixel', 'calibration_pixel') for event in self.app.events.queue))

    def test_late_ascii_reply_is_not_parsed_as_raw_pixels(self):
        session = self.request(0)
        with patch('radar_app.time.perf_counter', return_value=2.1):
            self.app._calibration_timeout()
        packet = f'PIX {session} 1 200000 204000 800\r\n'.encode()
        self.app._measurement_data(packet, 2.2)
        self.app._measurement_data(packet, 2.3)
        self.assertFalse(any(event[0] in ('ccd_pixel', 'calibration_pixel') for event in self.app.events.queue))

    def test_failed_request_does_not_leave_calibration_pending(self):
        self.app.measure_endpoint.write_line.return_value = False
        self.request(0)
        self.assertFalse(self.app.calibration_pending)
        self.assertIsNone(self.app.latest_pixel)

    def test_duplicate_valid_reply_produces_only_one_pixel(self):
        session = self.request(0)
        packet = f'PIX {session} 1 200000 204000 800\n'.encode()
        self.app._measurement_data(packet + packet, 0.5)
        self.assertEqual(self.app.events.get_nowait()[:2], ('calibration_pixel', (session, 800)))
        self.assertTrue(self.app.events.empty())

    def test_queued_previous_reading_cannot_overwrite_new_request(self):
        old = self.request(0)
        self.app._measurement_data(f'PIX {old} 1 200000 204000 800\n'.encode(), 0.5)
        self.request(1)
        self.app._handle_pixel = Mock()
        self.app._poll()
        self.app._handle_pixel.assert_not_called()
        self.assertIsNone(self.app.latest_pixel)
        current = self.app.calibration_session
        self.app._measurement_data(f'PIX {current} 1 1200000 1204000 900\n'.encode(), 1.5)
        self.app._poll()
        self.app._handle_pixel.assert_called_once_with(900, 1.5)


if __name__ == '__main__':
    unittest.main()
