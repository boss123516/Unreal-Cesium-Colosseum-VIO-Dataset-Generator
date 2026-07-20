#!/usr/bin/env python3

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fixedwing_frames import gazebo_state_to_airsim  # noqa: E402
from gazebo_airsim_bridge import (  # noqa: E402
    CONTRACT_LENGTH,
    decode_kinematics_payload,
    reanchor_state,
)


def payload() -> list[float]:
    return [
        1.0,
        1_000_000_000.0,
        10.0,
        20.0,
        30.0,
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
        7.0,
        8.0,
        9.0,
        10.0,
        11.0,
        12.0,
    ]


class GazeboAirSimBridgeTest(unittest.TestCase):
    def test_contract_length_is_stable(self):
        self.assertEqual(len(payload()), CONTRACT_LENGTH)

    def test_decode_preserves_framed_fields(self):
        state = decode_kinematics_payload(payload())
        self.assertEqual(state.source_time_ns, 1_000_000_000)
        self.assertEqual(state.position_world_m, (10.0, 20.0, 30.0))
        self.assertEqual(state.linear_velocity_world_mps, (1.0, 2.0, 3.0))
        self.assertEqual(state.angular_velocity_body_radps, (4.0, 5.0, 6.0))
        self.assertEqual(state.linear_acceleration_world_mps2, (7.0, 8.0, 9.0))
        self.assertEqual(state.angular_acceleration_body_radps2, (10.0, 11.0, 12.0))

    def test_frame_conversion_and_reanchoring(self):
        converted = gazebo_state_to_airsim(decode_kinematics_payload(payload()))
        anchored = reanchor_state(
            converted,
            source_origin_ned=(20.0, 10.0, -30.0),
            target_origin_ned=(100.0, 200.0, -300.0),
        )
        self.assertEqual(anchored.position_world_m, (100.0, 200.0, -300.0))
        self.assertEqual(anchored.linear_velocity_world_mps, (2.0, 1.0, -3.0))
        self.assertEqual(anchored.angular_velocity_body_radps, (4.0, -5.0, -6.0))
        self.assertEqual(anchored.linear_acceleration_world_mps2, (8.0, 7.0, -9.0))
        self.assertEqual(
            anchored.angular_acceleration_body_radps2, (10.0, -11.0, -12.0)
        )

    def test_invalid_length_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "must contain 21"):
            decode_kinematics_payload(payload()[:-1])

    def test_invalid_contract_is_rejected(self):
        values = payload()
        values[0] = 2.0
        with self.assertRaisesRegex(ValueError, "unsupported"):
            decode_kinematics_payload(values)

    def test_fractional_timestamp_is_rejected(self):
        values = payload()
        values[1] = 10.5
        with self.assertRaisesRegex(ValueError, "timestamp"):
            decode_kinematics_payload(values)


if __name__ == "__main__":
    unittest.main()
