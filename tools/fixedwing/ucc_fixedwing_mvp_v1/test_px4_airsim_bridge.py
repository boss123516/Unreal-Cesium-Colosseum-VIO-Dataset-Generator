#!/usr/bin/env python3

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from px4_airsim_bridge import MavlinkStateAssembler  # noqa: E402


class Message(SimpleNamespace):
    def get_type(self):
        return self.message_type


class MavlinkStateAssemblerTest(unittest.TestCase):
    def attitude(self, time_ms=1000, quaternion=(1.0, 0.0, 0.0, 0.0), rates=(0, 0, 0)):
        return Message(
            message_type="ATTITUDE_QUATERNION",
            time_boot_ms=time_ms,
            q1=quaternion[0],
            q2=quaternion[1],
            q3=quaternion[2],
            q4=quaternion[3],
            rollspeed=rates[0],
            pitchspeed=rates[1],
            yawspeed=rates[2],
        )

    def position(self, time_ms, position, velocity):
        return Message(
            message_type="LOCAL_POSITION_NED",
            time_boot_ms=time_ms,
            x=position[0],
            y=position[1],
            z=position[2],
            vx=velocity[0],
            vy=velocity[1],
            vz=velocity[2],
        )

    def test_origin_and_velocity_acceleration(self):
        assembler = MavlinkStateAssembler((0.0, 0.0, -0.25), acceleration_alpha=1.0)
        assembler.consume(self.attitude(), 1)
        first = assembler.consume(self.position(1000, (10, 20, 30), (1, 2, 3)), 2)
        self.assertEqual(first.position_ned_m, (0.0, 0.0, -0.25))
        self.assertEqual(first.orientation_ned_frd_xyzw, (0.0, 0.0, 0.0, 1.0))

        second = assembler.consume(self.position(1100, (11, 22, 27), (2, 4, 6)), 3)
        self.assertEqual(second.position_ned_m, (1.0, 2.0, -3.25))
        for actual, expected in zip(second.linear_acceleration_ned_mps2, (10, 20, 30)):
            self.assertAlmostEqual(actual, expected)

    def test_body_angular_acceleration(self):
        assembler = MavlinkStateAssembler((0, 0, 0), acceleration_alpha=1.0)
        assembler.consume(self.attitude(1000, rates=(0, 0, 0)), 1)
        assembler.consume(self.attitude(1100, rates=(0.1, -0.2, 0.3)), 2)
        state = assembler.consume(self.position(1100, (0, 0, 0), (0, 0, 0)), 3)
        for actual, expected in zip(state.angular_acceleration_frd_radps2, (1, -2, 3)):
            self.assertAlmostEqual(actual, expected)

    def test_invalid_quaternion_does_not_emit_state(self):
        assembler = MavlinkStateAssembler((0, 0, 0))
        assembler.consume(self.attitude(quaternion=(0, 0, 0, 0)), 1)
        state = assembler.consume(self.position(1000, (0, 0, 0), (0, 0, 0)), 2)
        self.assertIsNone(state)
        self.assertEqual(assembler.invalid_quaternions, 1)

    def test_timestamp_regression_is_checked_on_position_stream(self):
        assembler = MavlinkStateAssembler((0, 0, 0))
        assembler.consume(self.attitude(1200), 1)
        assembler.consume(self.position(1000, (0, 0, 0), (0, 0, 0)), 2)
        assembler.consume(self.position(900, (0, 0, 0), (0, 0, 0)), 3)
        self.assertEqual(assembler.timestamp_regressions, 1)

    def test_duplicate_position_timestamp_is_not_emitted_twice(self):
        assembler = MavlinkStateAssembler((0, 0, 0))
        assembler.consume(self.attitude(1000), 1)
        first = assembler.consume(self.position(1000, (0, 0, 0), (1, 0, 0)), 2)
        duplicate = assembler.consume(
            self.position(1000, (0, 0, 0), (1, 0, 0)), 3
        )
        self.assertIsNotNone(first)
        self.assertIsNone(duplicate)
        self.assertEqual(assembler.duplicate_source_states, 1)


if __name__ == "__main__":
    unittest.main()
