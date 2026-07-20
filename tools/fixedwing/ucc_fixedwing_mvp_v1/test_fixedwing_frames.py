#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fixedwing_frames import (  # noqa: E402
    FixedWingState,
    enu_world_to_ned,
    flu_body_to_frd,
    gazebo_state_to_airsim,
    orientation_enu_flu_to_ned_frd,
    quaternion_xyzw_to_matrix,
)


class FixedWingFramesTest(unittest.TestCase):
    def assertTupleAlmostEqual(self, actual, expected, places=9):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=places)

    def test_enu_to_ned_axes(self):
        self.assertEqual(enu_world_to_ned((1, 0, 0)), (0.0, 1.0, 0.0))
        self.assertEqual(enu_world_to_ned((0, 1, 0)), (1.0, 0.0, 0.0))
        self.assertEqual(enu_world_to_ned((0, 0, 1)), (0.0, 0.0, -1.0))

    def test_flu_to_frd_axes(self):
        self.assertEqual(flu_body_to_frd((1, 2, 3)), (1.0, -2.0, -3.0))

    def test_gazebo_identity_faces_east_in_airsim(self):
        converted = orientation_enu_flu_to_ned_frd((0, 0, 0, 1))
        rotation = quaternion_xyzw_to_matrix(converted)
        expected = (
            (0.0, -1.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        )
        for actual_row, expected_row in zip(rotation, expected):
            self.assertTupleAlmostEqual(actual_row, expected_row)

    def test_gazebo_north_heading_becomes_airsim_identity(self):
        half_angle = math.pi / 4.0
        gazebo_yaw_north = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
        converted = orientation_enu_flu_to_ned_frd(gazebo_yaw_north)
        self.assertTupleAlmostEqual(converted, (0.0, 0.0, 0.0, 1.0))

    def test_full_state_conversion(self):
        state = FixedWingState(
            source_time_ns=123,
            position_world_m=(10, 20, 30),
            orientation_world_body_xyzw=(0, 0, 0, 1),
            linear_velocity_world_mps=(1, 2, 3),
            angular_velocity_body_radps=(4, 5, 6),
            linear_acceleration_world_mps2=(7, 8, 9),
            angular_acceleration_body_radps2=(10, 11, 12),
        )
        converted = gazebo_state_to_airsim(state)
        self.assertEqual(converted.source_time_ns, 123)
        self.assertEqual(converted.position_world_m, (20.0, 10.0, -30.0))
        self.assertEqual(converted.linear_velocity_world_mps, (2.0, 1.0, -3.0))
        self.assertEqual(converted.angular_velocity_body_radps, (4.0, -5.0, -6.0))
        self.assertEqual(converted.linear_acceleration_world_mps2, (8.0, 7.0, -9.0))
        self.assertEqual(converted.angular_acceleration_body_radps2, (10.0, -11.0, -12.0))

    def test_invalid_quaternion_is_rejected(self):
        with self.assertRaises(ValueError):
            orientation_enu_flu_to_ned_frd((0, 0, 0, 0))

        with self.assertRaises(ValueError):
            orientation_enu_flu_to_ned_frd((math.nan, 0, 0, 1))


if __name__ == "__main__":
    unittest.main()
