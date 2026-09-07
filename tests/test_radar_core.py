import math
import unittest

from radar_core import CalibrationModel, CCDFrameParser, MotorLineParser, RotationTracker


class CalibrationTests(unittest.TestCase):
    def test_fit_and_distance(self):
        model = CalibrationModel()
        p0 = 742.0
        k = 118.0
        for distance in (0.4, 0.6, 0.9, 1.3, 2.0):
            model.add_point(p0 + k / distance, distance)
        fitted_p0, fitted_k, rmse = model.fit()
        self.assertAlmostEqual(fitted_p0, p0, places=8)
        self.assertAlmostEqual(fitted_k, k, places=8)
        self.assertAlmostEqual(rmse, 0.0, places=8)
        self.assertAlmostEqual(model.distance(p0 + k / 1.1), 1.1, places=8)

    def test_reject_same_distance(self):
        model = CalibrationModel(points=[(700, 1.0), (800, 1.0)])
        with self.assertRaises(ValueError):
            model.fit()


class ParserTests(unittest.TestCase):
    def test_fffe_split_frames_and_noise(self):
        parser = CCDFrameParser("fffe")
        self.assertEqual(parser.feed(b"noise\xff"), [])
        self.assertEqual(parser.feed(b"\xfe\x03"), [])
        self.assertEqual(parser.feed(b"\x10\xff\xfe\x02\xf0"), [784, 752])

    def test_raw_two_byte(self):
        parser = CCDFrameParser("raw2")
        self.assertEqual(parser.feed(b"\x02"), [])
        self.assertEqual(parser.feed(b"\xf0\x03\x10"), [752, 784])

    def test_ascii_center(self):
        parser = CCDFrameParser("ascii")
        data = "宽度:459 左:514 右:973 中心:743 阈值:1433\r\n".encode("utf-8")
        self.assertEqual(parser.feed(data), [743])

    def test_motor_lines(self):
        parser = MotorLineParser()
        self.assertEqual(parser.feed(b"READY\r\nTR"), ["READY"])
        self.assertEqual(parser.feed(b"IG 2\nSTATUS MOTOR=1 COUNT=2\r"), ["TRIG 2", "STATUS MOTOR=1 COUNT=2"])


class RotationTests(unittest.TestCase):
    def test_completed_revolution_uses_actual_period(self):
        tracker = RotationTracker(angle_offset_deg=10, clockwise=True, initial_period_s=3.0)
        tracker.trigger(100.0, 1)
        point = tracker.add_sample(1.0, 800, 101.0)
        self.assertIsNotNone(point)
        completed = tracker.trigger(104.0, 2)
        self.assertEqual(len(completed), 1)
        self.assertAlmostEqual(math.degrees(completed[0].angle_rad), 100.0, places=7)
        self.assertAlmostEqual(tracker.period_s, 4.0, places=7)
        self.assertAlmostEqual(tracker.rpm, 15.0, places=7)

    def test_counterclockwise(self):
        tracker = RotationTracker(angle_offset_deg=0, clockwise=False, initial_period_s=2.0)
        tracker.trigger(10.0)
        point = tracker.add_sample(1.0, 800, 10.5)
        self.assertAlmostEqual(math.degrees(point.angle_rad), -90.0, places=7)


if __name__ == "__main__":
    unittest.main()
