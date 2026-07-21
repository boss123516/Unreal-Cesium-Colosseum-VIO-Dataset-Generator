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
    cross_sensor_mapping_stats,
    local_altitude_stats,
    mapping_error_stats,
    motion_stats,
    quaternion_roll_deg,
    timestamp_stats,
    turn_straight_stats,
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

    def test_mapping_error_stats_detects_capture_timing_shift(self):
        stats = mapping_error_stats([-20.0, 10.0, 80.0])
        self.assertEqual(stats["count"], 3)
        self.assertEqual(stats["min"], -20.0)
        self.assertEqual(stats["max"], 80.0)
        self.assertEqual(stats["span"], 100.0)

    def test_cross_sensor_mapping_stats_removes_shared_clock_drift(self):
        stats = cross_sensor_mapping_stats(
            {100: -800.0, 200: -900.0},
            {100: -820.0, 200: -930.0},
        )
        self.assertEqual(stats["count"], 2)
        self.assertEqual(stats["min"], 20.0)
        self.assertEqual(stats["max"], 30.0)
        self.assertEqual(stats["max_abs"], 30.0)

    def test_identity_quaternion_has_zero_roll(self):
        orientation = SimpleNamespace(
            x_val=0.0, y_val=0.0, z_val=0.0, w_val=1.0
        )
        self.assertEqual(quaternion_roll_deg(orientation), 0.0)

    def test_local_altitude_contract_uses_air_sim_ned_delta(self):
        stats = local_altitude_stats(
            [(0.0, 0.0, 2.0), (1.0, 0.0, -18.0)],
            reference_ned_z_m=2.0,
            nominal_altitude_m=500.0,
            tolerance_m=50.0,
        )
        self.assertTrue(stats["all_within_bounds"])
        self.assertEqual(stats["min_m"], 500.0)
        self.assertEqual(stats["max_m"], 520.0)

    def test_turn_straight_contract_requires_both_bank_signs(self):
        stats = turn_straight_stats(
            [-8.0, -2.0, 0.0, 2.0, 9.0],
            required_turn_bank_deg=5.0,
            max_abs_roll_deg=35.0,
            straight_bank_deg=3.0,
            minimum_straight_fraction=0.5,
        )
        self.assertTrue(stats["all_pass"])
        self.assertTrue(stats["left_turn_present"])
        self.assertTrue(stats["right_turn_present"])
        self.assertEqual(stats["straight_fraction"], 0.6)

    def test_camera_frame_quality_detects_white_frame(self):
        buffer = BytesIO()
        Image.new("RGB", (16, 12), (255, 255, 255)).save(buffer, format="PNG")
        quality = camera_frame_quality(buffer.getvalue())
        self.assertEqual(quality["unique_colors"], 1)
        self.assertEqual(quality["white_ratio"], 1.0)
        self.assertEqual(quality["channel_stddev"], 0.0)


if __name__ == "__main__":
    unittest.main()
