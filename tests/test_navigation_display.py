import math
import unittest

from navigation_app import preview_is_stable_for_display
from navigation_core import ScanPoint
from scan_acquisition import PreviewObservation


def preview_points(count):
    return [
        PreviewObservation(
            float(index),
            math.tau * index / count,
            0.5,
            index,
            1.0,
            None,
            None,
        )
        for index in range(count)
    ]


class NavigationDisplayTests(unittest.TestCase):
    def test_sparse_preview_does_not_replace_last_stable_sweep(self):
        latest = [
            ScanPoint(math.tau * index / 50, 0.5, 1.0, True)
            for index in range(50)
        ]

        self.assertFalse(preview_is_stable_for_display(preview_points(20), latest))

    def test_complete_preview_can_replace_last_stable_sweep(self):
        latest = [
            ScanPoint(math.tau * index / 50, 0.5, 1.0, True)
            for index in range(50)
        ]

        self.assertTrue(preview_is_stable_for_display(preview_points(40), latest))

    def test_startup_preview_waits_for_minimum_points(self):
        self.assertFalse(preview_is_stable_for_display(preview_points(20), []))
        self.assertTrue(preview_is_stable_for_display(preview_points(40), []))


if __name__ == "__main__":
    unittest.main()
