#!/usr/bin/env python3
"""PX4 mission profiles for the 500 m fixed-wing dataset.

The prepare phase takes the Gazebo aircraft to a stable relative altitude before
the Gazebo-to-AirSim bridge is started.  The dynamic phase then builds a broad
S-turn mission from the live position and heading.  AirSim is reanchored at the
start of the bridge, so the Gazebo relative altitude becomes a delta around the
500 m Unreal spawn instead of an absolute Cesium altitude.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Any


EARTH_RADIUS_M = 6_378_137.0
MAV_FRAME_MISSION = 2
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_UNLIM = 17
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_MISSION_TYPE_MISSION = 0


@dataclass(frozen=True)
class MissionItem:
    seq: int
    frame: int
    command: int
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = math.nan
    latitude_deg: float = 0.0
    longitude_deg: float = 0.0
    relative_altitude_m: float = 0.0
    label: str = ""
    forward_m: float | None = None
    right_m: float | None = None
    altitude_offset_m: float | None = None


@dataclass(frozen=True)
class VehicleSnapshot:
    latitude_deg: float
    longitude_deg: float
    relative_altitude_m: float
    heading_deg: float
    ground_speed_mps: float


def offset_global(
    latitude_deg: float,
    longitude_deg: float,
    north_m: float,
    east_m: float,
) -> tuple[float, float]:
    latitude_rad = math.radians(latitude_deg)
    if abs(math.cos(latitude_rad)) < 1.0e-6:
        raise ValueError("latitude is too close to a pole for local conversion")
    return (
        latitude_deg + math.degrees(north_m / EARTH_RADIUS_M),
        longitude_deg
        + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(latitude_rad))),
    )


def track_offset_to_global(
    latitude_deg: float,
    longitude_deg: float,
    heading_deg: float,
    forward_m: float,
    right_m: float,
) -> tuple[float, float]:
    heading_rad = math.radians(heading_deg)
    north_m = forward_m * math.cos(heading_rad) - right_m * math.sin(heading_rad)
    east_m = forward_m * math.sin(heading_rad) + right_m * math.cos(heading_rad)
    return offset_global(latitude_deg, longitude_deg, north_m, east_m)


def navigation_item(
    seq: int,
    command: int,
    latitude_deg: float,
    longitude_deg: float,
    relative_altitude_m: float,
    label: str,
    *,
    acceptance_radius_m: float = 35.0,
    loiter_radius_m: float = 180.0,
    forward_m: float | None = None,
    right_m: float | None = None,
    altitude_offset_m: float | None = None,
) -> MissionItem:
    param2 = acceptance_radius_m if command == MAV_CMD_NAV_WAYPOINT else 0.0
    param3 = loiter_radius_m if command == MAV_CMD_NAV_LOITER_UNLIM else 0.0
    return MissionItem(
        seq=seq,
        frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=command,
        param1=15.0 if command == MAV_CMD_NAV_TAKEOFF else 0.0,
        param2=param2,
        param3=param3,
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        relative_altitude_m=relative_altitude_m,
        label=label,
        forward_m=forward_m,
        right_m=right_m,
        altitude_offset_m=altitude_offset_m,
    )


def build_prepare_mission(
    snapshot: VehicleSnapshot,
    target_relative_altitude_m: float,
) -> list[MissionItem]:
    takeoff_lat, takeoff_lon = track_offset_to_global(
        snapshot.latitude_deg,
        snapshot.longitude_deg,
        snapshot.heading_deg,
        450.0,
        0.0,
    )
    hold_lat, hold_lon = track_offset_to_global(
        snapshot.latitude_deg,
        snapshot.longitude_deg,
        snapshot.heading_deg,
        700.0,
        0.0,
    )
    return [
        navigation_item(
            0,
            MAV_CMD_NAV_TAKEOFF,
            takeoff_lat,
            takeoff_lon,
            target_relative_altitude_m,
            "takeoff",
            forward_m=450.0,
            right_m=0.0,
            altitude_offset_m=0.0,
        ),
        navigation_item(
            1,
            MAV_CMD_NAV_WAYPOINT,
            hold_lat,
            hold_lon,
            target_relative_altitude_m,
            "stabilize_straight",
            forward_m=700.0,
            right_m=0.0,
            altitude_offset_m=0.0,
        ),
        navigation_item(
            2,
            MAV_CMD_NAV_LOITER_UNLIM,
            hold_lat,
            hold_lon,
            target_relative_altitude_m,
            "bridge_anchor_loiter",
            loiter_radius_m=180.0,
            forward_m=700.0,
            right_m=0.0,
            altitude_offset_m=0.0,
        ),
    ]


def build_dynamic_mission(
    snapshot: VehicleSnapshot,
    duration_sec: float,
    speed_mps: float,
) -> list[MissionItem]:
    if duration_sec <= 0.0 or speed_mps <= 0.0:
        raise ValueError("duration and speed must be positive")

    # Longitudinal length is slightly shorter than speed * duration because
    # alternating lateral offsets add distance.  The last item is a loiter, so
    # a small timing mismatch remains safe after recording ends.
    route_length_m = speed_mps * duration_sec * 0.92
    lateral_amplitude_m = min(140.0, max(90.0, route_length_m * 0.035))
    shape = [
        (0.05, 0.0, 0.0, "entry_straight"),
        (0.18, +1.0, +8.0, "right_turn_entry"),
        (0.31, +1.0, +15.0, "right_offset_straight"),
        (0.44, -1.0, +5.0, "left_crossover"),
        (0.57, -1.0, -10.0, "left_offset_straight"),
        (0.70, +1.0, -15.0, "right_crossover"),
        (0.83, +1.0, -5.0, "right_offset_straight_2"),
        (0.95, 0.0, 0.0, "exit_straight"),
    ]

    mission: list[MissionItem] = []
    for fraction, lateral_sign, altitude_offset_m, label in shape:
        forward_m = route_length_m * fraction
        right_m = lateral_amplitude_m * lateral_sign
        latitude_deg, longitude_deg = track_offset_to_global(
            snapshot.latitude_deg,
            snapshot.longitude_deg,
            snapshot.heading_deg,
            forward_m,
            right_m,
        )
        mission.append(
            navigation_item(
                len(mission),
                MAV_CMD_NAV_WAYPOINT,
                latitude_deg,
                longitude_deg,
                snapshot.relative_altitude_m + altitude_offset_m,
                label,
                acceptance_radius_m=35.0,
                forward_m=forward_m,
                right_m=right_m,
                altitude_offset_m=altitude_offset_m,
            )
        )

        if len(mission) == 1:
            mission.append(
                MissionItem(
                    seq=1,
                    frame=MAV_FRAME_MISSION,
                    command=MAV_CMD_DO_CHANGE_SPEED,
                    param1=0.0,
                    param2=speed_mps,
                    param3=-1.0,
                    param4=0.0,
                    label="set_airspeed",
                )
            )

    last = mission[-1]
    mission.append(
        navigation_item(
            len(mission),
            MAV_CMD_NAV_LOITER_UNLIM,
            last.latitude_deg,
            last.longitude_deg,
            snapshot.relative_altitude_m,
            "final_loiter",
            loiter_radius_m=200.0,
            forward_m=last.forward_m,
            right_m=last.right_m,
            altitude_offset_m=0.0,
        )
    )
    return [
        MissionItem(**{**asdict(item), "seq": index})
        for index, item in enumerate(mission)
    ]


def mission_metadata(
    phase: str,
    snapshot: VehicleSnapshot,
    mission: list[MissionItem],
    **extra: Any,
) -> dict[str, Any]:
    return {
        "phase": phase,
        "generated_at_unix_sec": time.time(),
        "origin": asdict(snapshot),
        "items": [asdict(item) for item in mission],
        **extra,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("phase", choices=("prepare", "dynamic"))
    parser.add_argument("--endpoint", default="udpin:0.0.0.0:14550")
    parser.add_argument("--connect-timeout-sec", type=float, default=10.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--latitude-deg", type=float, default=47.397742)
    parser.add_argument("--longitude-deg", type=float, default=8.545594)
    parser.add_argument("--heading-deg", type=float, default=0.0)
    parser.add_argument("--relative-altitude-m", type=float, default=100.0)
    parser.add_argument("--target-relative-altitude-m", type=float, default=100.0)
    parser.add_argument("--prepare-timeout-sec", type=float, default=180.0)
    parser.add_argument("--duration-sec", type=float, default=180.0)
    parser.add_argument("--speed-mps", type=float, default=19.0)
    parser.add_argument("--bank-limit-deg", type=float, default=28.0)
    return parser.parse_args()


def write_json(path: Path | None, payload: dict[str, Any]) -> None:
    if path is None:
        return
    output = path.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"[MISSION_METADATA] {output}")


def wait_px4_heartbeat(connection: Any, mavutil: Any, timeout_sec: float) -> Any:
    """Ignore forwarded GCS heartbeats and select the PX4 autopilot endpoint."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=min(0.5, max(0.0, deadline - time.monotonic())),
        )
        if message is None:
            continue
        if (
            int(message.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_PX4
            and int(message.get_srcSystem()) > 0
        ):
            connection.target_system = int(message.get_srcSystem())
            connection.target_component = int(message.get_srcComponent()) or 1
            return message
    raise RuntimeError("timed out waiting for a PX4 autopilot heartbeat")


def wait_snapshot(connection: Any, timeout_sec: float) -> VehicleSnapshot:
    deadline = time.monotonic() + timeout_sec
    global_position = None
    heading_deg = None
    ground_speed_mps = 0.0
    while time.monotonic() < deadline:
        message = connection.recv_match(blocking=True, timeout=0.5)
        if message is None:
            continue
        if int(message.get_srcSystem()) not in (0, connection.target_system):
            continue
        message_type = message.get_type()
        if message_type == "GLOBAL_POSITION_INT":
            global_position = message
            if int(message.hdg) != 65535:
                heading_deg = float(message.hdg) * 0.01
            ground_speed_mps = math.hypot(float(message.vx), float(message.vy)) * 0.01
        elif message_type == "ATTITUDE" and heading_deg is None:
            heading_deg = math.degrees(float(message.yaw)) % 360.0
        elif message_type == "VFR_HUD":
            ground_speed_mps = float(message.groundspeed)

        if global_position is not None and heading_deg is not None:
            return VehicleSnapshot(
                latitude_deg=float(global_position.lat) * 1.0e-7,
                longitude_deg=float(global_position.lon) * 1.0e-7,
                relative_altitude_m=float(global_position.relative_alt) * 1.0e-3,
                heading_deg=heading_deg,
                ground_speed_mps=ground_speed_mps,
            )
    raise RuntimeError("timed out waiting for PX4 global position and heading")


def drain(connection: Any) -> None:
    while connection.recv_match(blocking=False) is not None:
        pass


def send_gcs_heartbeat(connection: Any, mavutil: Any) -> None:
    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def maintain_gcs_link(connection: Any, mavutil: Any, duration_sec: float) -> None:
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        send_gcs_heartbeat(connection, mavutil)
        interval_end = min(deadline, time.monotonic() + 0.5)
        while time.monotonic() < interval_end:
            connection.recv_match(blocking=True, timeout=0.1)


def send_item(connection: Any, item: MissionItem, use_int: bool) -> None:
    target_system = connection.target_system
    target_component = connection.target_component
    if use_int:
        connection.mav.mission_item_int_send(
            target_system,
            target_component,
            item.seq,
            item.frame,
            item.command,
            1 if item.seq == 0 else 0,
            1,
            item.param1,
            item.param2,
            item.param3,
            item.param4,
            round(item.latitude_deg * 1.0e7),
            round(item.longitude_deg * 1.0e7),
            item.relative_altitude_m,
            MAV_MISSION_TYPE_MISSION,
        )
    else:
        connection.mav.mission_item_send(
            target_system,
            target_component,
            item.seq,
            item.frame,
            item.command,
            1 if item.seq == 0 else 0,
            1,
            item.param1,
            item.param2,
            item.param3,
            item.param4,
            item.latitude_deg,
            item.longitude_deg,
            item.relative_altitude_m,
            MAV_MISSION_TYPE_MISSION,
        )


def upload_mission(connection: Any, mavutil: Any, mission: list[MissionItem]) -> None:
    drain(connection)
    connection.mav.mission_clear_all_send(
        connection.target_system,
        connection.target_component,
        MAV_MISSION_TYPE_MISSION,
    )
    clear_deadline = time.monotonic() + 2.0
    while time.monotonic() < clear_deadline:
        message = connection.recv_match(type="MISSION_ACK", blocking=True, timeout=0.2)
        if message is not None:
            break

    drain(connection)
    connection.mav.mission_count_send(
        connection.target_system,
        connection.target_component,
        len(mission),
        MAV_MISSION_TYPE_MISSION,
    )
    deadline = time.monotonic() + 20.0
    next_heartbeat = 0.0
    sent_sequences: set[int] = set()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_heartbeat:
            send_gcs_heartbeat(connection, mavutil)
            next_heartbeat = now + 1.0
        message = connection.recv_match(
            type=[
                "MISSION_REQUEST_INT",
                "MISSION_REQUEST",
                "MISSION_ACK",
                "STATUSTEXT",
            ],
            blocking=True,
            timeout=0.5,
        )
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            print(f"[PX4] {message.text}")
            continue
        if message.get_type() in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            sequence = int(message.seq)
            if not 0 <= sequence < len(mission):
                raise RuntimeError(f"PX4 requested invalid mission sequence {sequence}")
            send_item(
                connection,
                mission[sequence],
                message.get_type() == "MISSION_REQUEST_INT",
            )
            sent_sequences.add(sequence)
            continue

        if int(message.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise RuntimeError(f"PX4 rejected mission with ACK type {message.type}")
        if len(sent_sequences) != len(mission):
            raise RuntimeError(
                f"PX4 acknowledged incomplete upload: {len(sent_sequences)}/{len(mission)}"
            )
        print(f"[MISSION] uploaded {len(mission)} items")
        return
    raise RuntimeError("timed out uploading mission to PX4")


def set_parameter(
    connection: Any,
    mavutil: Any,
    name: str,
    value: float,
    timeout_sec: float = 3.0,
    parameter_type: int | None = None,
) -> float:
    connection.mav.param_set_send(
        connection.target_system,
        connection.target_component,
        name.encode("ascii"),
        float(value),
        parameter_type or mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if message is None:
            continue
        param_id = message.param_id
        if isinstance(param_id, bytes):
            param_id = param_id.decode("ascii", errors="ignore")
        if str(param_id).rstrip("\x00") == name:
            actual = float(message.param_value)
            print(f"[PARAM] {name}={actual}")
            return actual
    raise RuntimeError(f"timed out setting PX4 parameter {name}")


def wait_flight_mode(connection: Any, mavutil: Any, mode: str, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    next_command = 0.0
    next_heartbeat = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_heartbeat:
            send_gcs_heartbeat(connection, mavutil)
            next_heartbeat = now + 0.5
        if now >= next_command:
            connection.set_mode(mode)
            next_command = now + 2.0
        message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            message is not None
            and int(message.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_PX4
            and int(message.get_srcSystem()) == connection.target_system
            and mavutil.mode_string_v10(message) == mode
        ):
            print(f"[MODE] {mode}")
            return
    raise RuntimeError(f"PX4 did not enter {mode} mode")


def arm(connection: Any, mavutil: Any, timeout_sec: float = 10.0) -> None:
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    deadline = time.monotonic() + timeout_sec
    next_heartbeat = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_heartbeat:
            send_gcs_heartbeat(connection, mavutil)
            next_heartbeat = now + 0.5
        message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            message is not None
            and int(message.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_PX4
            and int(message.get_srcSystem()) == connection.target_system
            and int(message.base_mode)
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ):
            print("[ARM] armed")
            return
    raise RuntimeError("PX4 did not arm")


def wait_prepare_stable(
    connection: Any,
    mavutil: Any,
    target_relative_altitude_m: float,
    timeout_sec: float,
) -> VehicleSnapshot:
    deadline = time.monotonic() + timeout_sec
    next_heartbeat = 0.0
    stable_since = None
    latest = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_heartbeat:
            send_gcs_heartbeat(connection, mavutil)
            next_heartbeat = now + 1.0
        try:
            latest = wait_snapshot(connection, 1.0)
        except RuntimeError:
            continue
        stable = (
            abs(latest.relative_altitude_m - target_relative_altitude_m) <= 5.0
            and latest.ground_speed_mps >= 10.0
        )
        stable_since = now if stable and stable_since is None else stable_since
        if not stable:
            stable_since = None
        if stable_since is not None and now - stable_since >= 5.0:
            print(
                "[PREPARE_READY] "
                f"relative_altitude={latest.relative_altitude_m:.1f}m "
                f"ground_speed={latest.ground_speed_mps:.1f}m/s"
            )
            return latest
    raise RuntimeError("fixed-wing takeoff did not stabilize before timeout")


def main() -> int:
    args = parse_args()
    if args.speed_mps <= 0.0 or args.duration_sec <= 0.0:
        raise SystemExit("[ERROR] duration and speed must be positive")
    if not 0.0 < args.bank_limit_deg <= 50.0:
        raise SystemExit("[ERROR] bank limit must be in (0, 50] degrees")

    if args.dry_run:
        snapshot = VehicleSnapshot(
            args.latitude_deg,
            args.longitude_deg,
            args.relative_altitude_m,
            args.heading_deg % 360.0,
            args.speed_mps,
        )
        mission = (
            build_prepare_mission(snapshot, args.target_relative_altitude_m)
            if args.phase == "prepare"
            else build_dynamic_mission(snapshot, args.duration_sec, args.speed_mps)
        )
        metadata = mission_metadata(
            args.phase,
            snapshot,
            mission,
            dry_run=True,
            duration_sec=args.duration_sec,
            target_speed_mps=args.speed_mps,
            bank_limit_deg=args.bank_limit_deg,
        )
        write_json(args.output, metadata)
        print(json.dumps(metadata, indent=2))
        return 0

    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise SystemExit(f"[ERROR] pymavlink is not installed: {exc}") from exc

    connection = mavutil.mavlink_connection(args.endpoint)
    try:
        heartbeat = wait_px4_heartbeat(
            connection,
            mavutil,
            args.connect_timeout_sec,
        )
    except RuntimeError as exc:
        raise SystemExit(f"[ERROR] {exc} on {args.endpoint}") from exc
    print(
        f"[CONNECT] PX4 system={connection.target_system} "
        f"component={connection.target_component}"
    )
    send_gcs_heartbeat(connection, mavutil)
    initial_mode = mavutil.mode_string_v10(heartbeat)
    if initial_mode == "RTL":
        # PX4 defaults COM_DL_LOSS_T to 10 s.  If a previous GCS/bridge died,
        # keep a new heartbeat alive long enough to clear the latched data-link
        # loss before requesting MISSION again.
        print("[GCS] recovering a previously lost data link (11 s)")
        maintain_gcs_link(connection, mavutil, 11.0)
    snapshot = wait_snapshot(connection, args.connect_timeout_sec)

    if args.phase == "prepare":
        set_parameter(
            connection,
            mavutil,
            "MIS_TKO_LAND_REQ",
            0.0,
            parameter_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        mission = build_prepare_mission(snapshot, args.target_relative_altitude_m)
        upload_mission(connection, mavutil, mission)
        wait_flight_mode(connection, mavutil, "MISSION", 10.0)
        arm(connection, mavutil)
        ready_snapshot = wait_prepare_stable(
            connection,
            mavutil,
            args.target_relative_altitude_m,
            args.prepare_timeout_sec,
        )
        metadata = mission_metadata(
            "prepare",
            snapshot,
            mission,
            ready_state=asdict(ready_snapshot),
            target_relative_altitude_m=args.target_relative_altitude_m,
        )
    else:
        if snapshot.relative_altitude_m < 30.0 or snapshot.ground_speed_mps < 8.0:
            raise SystemExit(
                "[ERROR] aircraft is not ready for the dynamic mission: "
                f"relative_altitude={snapshot.relative_altitude_m:.1f}m, "
                f"ground_speed={snapshot.ground_speed_mps:.1f}m/s"
            )
        set_parameter(
            connection,
            mavutil,
            "MIS_TKO_LAND_REQ",
            0.0,
            parameter_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        actual_bank_limit = set_parameter(
            connection,
            mavutil,
            "FW_R_LIM",
            args.bank_limit_deg,
        )
        mission = build_dynamic_mission(snapshot, args.duration_sec, args.speed_mps)
        upload_mission(connection, mavutil, mission)
        connection.mav.mission_set_current_send(
            connection.target_system,
            connection.target_component,
            0,
        )
        wait_flight_mode(connection, mavutil, "MISSION", 10.0)
        metadata = mission_metadata(
            "dynamic",
            snapshot,
            mission,
            duration_sec=args.duration_sec,
            target_speed_mps=args.speed_mps,
            requested_bank_limit_deg=args.bank_limit_deg,
            actual_bank_limit_deg=actual_bank_limit,
            mission_takeoff_landing_requirement=0,
            commanded_altitude_offset_range_m=[-15.0, 15.0],
        )

    write_json(args.output, metadata)
    print("[MISSION_STARTED]" if args.phase == "dynamic" else "[PREPARE_COMPLETE]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
