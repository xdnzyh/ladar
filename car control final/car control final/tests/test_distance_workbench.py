import importlib.util
import queue
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "CCD_Distance_App_v1_2" / "source"


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


workbench_core = load_module("_workbench_core", SOURCE / "core.py")
with patch.dict(sys.modules, {"core": workbench_core}):
    workbench_transport = load_module("_workbench_transport", SOURCE / "transport.py")


class DistanceWorkbenchTests(unittest.TestCase):
    def test_embedded_defaults_match_current_csv(self):
        embedded = workbench_core.Calibration()
        from_csv = workbench_core.Calibration.from_csv(
            ROOT / "CCD_Distance_App_v1_2" / "calibration.csv"
        )
        self.assertEqual(embedded.points, from_csv.points)
        self.assertEqual(embedded.distance_range_cm, (15.0, 100.0))
        self.assertEqual(embedded.coordinate_range, (814.0, 1203.0))
        distance, status = embedded.convert(910)
        self.assertAlmostEqual(distance, 42.857142857)
        self.assertEqual(status, "范围内")

    def test_handshake_matches_current_burned_firmware(self):
        status = (
            "STATUS CCD-PEAK-RAW-CAL-3.0 LASER=0 EXPOSURE_SENT=5 "
            "BATCH=0 DEBUG=0 INPUT=USB CMD=@c0071#@"
        )

        class Protocol:
            def __init__(self):
                self.commands = []

            def transact(self, command, predicate):
                self.commands.append(command)
                responses = {
                    "STOP": "OK STOP",
                    "DEBUG 0": "OK DEBUG 0",
                    "LASER 0": "OK LASER 0",
                    "STATUS": status,
                    "EXPOSURE 5": "EXPOSURE SENT: 5",
                }
                result = predicate(responses[command])
                if result is None:
                    raise AssertionError("握手谓词未接受当前固件回复")
                return result

            def discard_before_retry(self):
                raise AssertionError("当前固件回复不应触发重试")

        worker = workbench_transport.SerialWorker("COM1", queue.Queue())
        worker.stop_event = Mock()
        worker.stop_event.wait.return_value = False
        protocol = Protocol()
        self.assertEqual(worker._handshake(protocol), status)
        self.assertEqual(
            protocol.commands,
            ["STOP", "DEBUG 0", "LASER 0", "STATUS", "EXPOSURE 5", "STATUS"],
        )


if __name__ == "__main__":
    unittest.main()
