#!/usr/bin/env python3

from io import BytesIO
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fixedwing_mini_dataset import (  # noqa: E402
    CaptureState,
    camera_frame_quality,
    motion_stats,
    quaternion_roll_deg,
    timestamp_stats,
)


class FixedWingMiniDatasetTest(unittest.TestCase):
    def test_timestamp_stats_detects_duplicates_and_regressions(self):
        stats = timestamp_stats([1_000_000, 2_000_000, 2_000_000, 1_500_000])
        self.assertEqual(stats["duplicates"], 1)
        self.assertEqual(stats["regressions"], 1)
        self.assertEqual(stats["period_ms"]["mean"], 1.0)

    def test_motion_stats_uses_ned_altitude_sign(self):
        state = CaptureState()
        state.gt_positions = [(0.0, 0.0, 0.0), (3.0, 4.0, -2.0)]
        state.gt_horizontal_speed = [0.0, 12.0]
        state.gt_roll_deg = [-10.0, 20.0]
        stats = motion_stats(state)
        self.assertAlmostEqual(stats["max_displacement_m"], 29.0**0.5)
        self.assertEqual(stats["max_relative_altitude_m"], 2.0)
        self.assertEqual(stats["roll_span_deg"], 30.0)

    def test_identity_quaternion_has_zero_roll(self):
        orientation = SimpleNamespace(
            x_val=0.0, y_val=0.0, z_val=0.0, w_val=1.0
        )
        self.assertEqual(quaternion_roll_deg(orientation), 0.0)

    def test_camera_frame_quality_detects_white_frame(self):
        buffer = BytesIO()
        Image.new("RGB", (16, 12), (255, 255, 255)).save(buffer, format="PNG")
        quality = camera_frame_quality(buffer.getvalue())
        self.assertEqual(quality["unique_colors"], 1)
        self.assertEqual(quality["white_ratio"], 1.0)
        self.assertEqual(quality["channel_stddev"], 0.0)


if __name__ == "__main__":
    unittest.main()
