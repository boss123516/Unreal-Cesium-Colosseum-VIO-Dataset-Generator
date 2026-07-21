#!/usr/bin/env python3

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parent))

from px4_fixedwing_mission import (  # noqa: E402
    MAV_CMD_DO_CHANGE_SPEED,
    MAV_CMD_NAV_LOITER_UNLIM,
    VehicleSnapshot,
    build_dynamic_mission,
    build_prepare_mission,
    track_offset_to_global,
    wait_px4_heartbeat,
)


class Px4FixedWingMissionTest(unittest.TestCase):
    def setUp(self):
        self.snapshot = VehicleSnapshot(
            latitude_deg=47.397742,
            longitude_deg=8.545594,
            relative_altitude_m=100.0,
            heading_deg=90.0,
            ground_speed_mps=19.0,
        )

    def test_track_offset_respects_east_heading(self):
        latitude, longitude = track_offset_to_global(
            self.snapshot.latitude_deg,
            self.snapshot.longitude_deg,
            90.0,
            100.0,
            0.0,
        )
        self.assertAlmostEqual(latitude, self.snapshot.latitude_deg, places=7)
        self.assertGreater(longitude, self.snapshot.longitude_deg)

    def test_dynamic_profile_has_moderate_bilateral_turns_and_altitude(self):
        mission = build_dynamic_mission(self.snapshot, 180.0, 19.0)
        navigation = [item for item in mission if item.forward_m is not None]
        lateral = [item.right_m for item in navigation if item.right_m is not None]
        altitude = [
            item.altitude_offset_m
            for item in navigation
            if item.altitude_offset_m is not None
        ]
        self.assertLess(min(lateral), 0.0)
        self.assertGreater(max(lateral), 0.0)
        self.assertGreaterEqual(min(altitude), -15.0)
        self.assertLessEqual(max(altitude), 15.0)
        self.assertEqual(mission[-1].command, MAV_CMD_NAV_LOITER_UNLIM)
        self.assertEqual(
            sum(item.command == MAV_CMD_DO_CHANGE_SPEED for item in mission), 1
        )
        self.assertEqual([item.seq for item in mission], list(range(len(mission))))

    def test_route_is_sized_for_three_minutes_at_nineteen_mps(self):
        mission = build_dynamic_mission(self.snapshot, 180.0, 19.0)
        points = [
            item
            for item in mission
            if item.command
            not in (MAV_CMD_DO_CHANGE_SPEED, MAV_CMD_NAV_LOITER_UNLIM)
        ]
        length = 0.0
        previous_forward = 0.0
        previous_right = 0.0
        for point in points:
            length += math.hypot(
                float(point.forward_m) - previous_forward,
                float(point.right_m) - previous_right,
            )
            previous_forward = float(point.forward_m)
            previous_right = float(point.right_m)
        self.assertGreater(length, 19.0 * 160.0)
        self.assertLess(length, 19.0 * 195.0)

    def test_prepare_profile_ends_in_anchor_loiter(self):
        mission = build_prepare_mission(self.snapshot, 100.0)
        self.assertEqual(mission[-1].command, MAV_CMD_NAV_LOITER_UNLIM)
        self.assertTrue(
            all(item.relative_altitude_m == 100.0 for item in mission)
        )

    def test_heartbeat_filter_ignores_forwarded_gcs(self):
        class Message:
            def __init__(self, autopilot, system, component):
                self.autopilot = autopilot
                self._system = system
                self._component = component

            def get_srcSystem(self):
                return self._system

            def get_srcComponent(self):
                return self._component

        class Connection:
            def __init__(self):
                self.messages = iter(
                    [
                        Message(8, 0, 1),
                        Message(12, 1, 1),
                    ]
                )
                self.target_system = 0
                self.target_component = 0

            def recv_match(self, **_kwargs):
                return next(self.messages, None)

        connection = Connection()
        mavutil = SimpleNamespace(
            mavlink=SimpleNamespace(MAV_AUTOPILOT_PX4=12)
        )
        selected = wait_px4_heartbeat(connection, mavutil, 0.1)
        self.assertEqual(selected.get_srcSystem(), 1)
        self.assertEqual(connection.target_system, 1)
        self.assertEqual(connection.target_component, 1)


if __name__ == "__main__":
    unittest.main()
