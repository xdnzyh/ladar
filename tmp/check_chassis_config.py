import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import sys
import time
from unittest.mock import patch

import serial

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from chassis_controller import ChassisController, ChassisState
from chassis_log import ChassisTrafficLogger
from navigation_app import NavigationApp, load_configuration
from serial_backend import SerialEndpoint, _PySerialTransport


class StationaryEndpoint(SerialEndpoint):
    def write_ticket(self, data, **kwargs):
        line = data.lstrip(b" ")
        if not (line in (b"P\r\n", b"!\r\nX\r\n") or re.fullmatch(rb"@PING,[0-9A-F]{8}\r\n", line)
                or re.fullmatch(rb"@CFG,[IGBSCA],[0-9A-F]{8}(?:,[0-9]+)*,[0-9A-F]{4}\r\n", line)):
            raise ValueError(f"Nonstationary diagnostic command rejected: {line!r}")
        return super().write_ticket(data, **kwargs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--auto-read', action='store_true')
    parser.add_argument('--check-stop', action='store_true')
    args = parser.parse_args()
    config = load_configuration('hardware', 'navigation')
    log_path = ROOT / 'logs' / ('chassis_config_check_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.jsonl')
    logger = ChassisTrafficLogger(log_path)
    result = {}
    errors = []

    def emit(kind, value, stamp):
        if kind in ('chassis_raw_rx', 'chassis_raw_tx'):
            generation, action_id, data = value
            logger.record('RX' if kind.endswith('rx') else 'TX', data, stamp,
                          connection_generation=generation, action_id=action_id)
        elif kind == 'chassis_frame':
            controller.handle_frame(*value, stamp)
        elif kind == 'chassis_communication_result':
            result.update(value[1])
            print(kind, value[1], flush=True)
        elif kind in ('chassis_status', 'chassis_state', 'chassis_fault'):
            print(kind, value, flush=True)
            if kind == 'chassis_state' and args.auto_read:
                app._verify_chassis_on_idle(value[0], value[1])

    endpoint = StationaryEndpoint('chassis-check',
        lambda data, stamp: controller.feed_data(data, stamp, generation),
        lambda error: errors.append(error))
    controller = ChassisController(endpoint, emit, config)
    app = NavigationApp.__new__(NavigationApp)
    app.config = config
    app.chassis_controller = controller
    generation = controller.begin_connection()

    def open_transport(port, baudrate):
        port_object = serial.Serial(port=None, baudrate=baudrate, timeout=0.05, write_timeout=0.5)
        port_object.dtr = False
        port_object.rts = False
        port_object.port = port
        port_object.open()
        return _PySerialTransport(port_object)

    def wait_for(predicate, seconds):
        deadline = time.perf_counter() + seconds
        while time.perf_counter() < deadline:
            controller.poll()
            if errors:
                raise RuntimeError(errors)
            if predicate():
                return
            time.sleep(0.02)
        raise TimeoutError(f'controller={controller.state}, verified={controller.config_verified}')

    try:
        with patch('serial_backend._open_transport', open_transport):
            endpoint.open(config['chassis_port'], config['chassis_baudrate'])
        controller.mark_connection_open()
        time.sleep(0.6)
        if not controller.request_status():
            raise RuntimeError('Status query was rejected')
        wait_for(lambda: controller.confirmed and controller.state == ChassisState.IDLE, 8)
        if not args.auto_read and not controller.request_config_sync(apply=args.apply):
            raise RuntimeError('Config sync was rejected')
        wait_for(lambda: controller.config_exchange is None, 90)
        if not controller.config_verified:
            raise RuntimeError('Complete parameter readback did not match the profile')
        if not controller.request_communication_check(20):
            raise RuntimeError('Communication check was rejected')
        wait_for(lambda: bool(result), 90)
        wait_for(lambda: controller.idle_preflight_ready, 8)
        if args.check_stop:
            if not controller.request_stop() or not controller.config_verified:
                raise RuntimeError('Idle stop lost verified parameters')
            wait_for(lambda: controller.confirmed and controller.state == ChassisState.IDLE, 8)
            if not controller.config_verified:
                raise RuntimeError('Verified parameters lost after stop acknowledgement')
            wait_for(lambda: controller.idle_preflight_ready, 8)
        print(json.dumps({'port': config['chassis_port'], 'crc': config['chassis_config1_crc'],
            'config_verified': controller.config_verified, 'state': controller.state,
            'automatic_ready': controller.allow_automatic(), 'move_commands': 0,
            'communication': result, 'log': str(log_path)}, ensure_ascii=True), flush=True)
    finally:
        controller.disconnect()
        endpoint.close()


if __name__ == '__main__':
    main()
