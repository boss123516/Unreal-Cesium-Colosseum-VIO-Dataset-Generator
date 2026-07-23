#!/usr/bin/env python3
"""Fly a slow, inward-looking orbit around the KAU origin at local 30 m."""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from typing import Any

try:
    import airsim
except ImportError as exc:
    raise SystemExit(
        "[ERROR] Could not import airsim. Activate ~/vio_sim_ws/airsim_pyenv "
        "and add Colosseum/PythonClient to PYTHONPATH."
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Orbit KAU buildings at a 30 m local spawn altitude."
    )
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--spawn-altitude-m", type=float, default=30.0)
    parser.add_argument("--radius-m", type=float, default=90.0)
    parser.add_argument("--speed-mps", type=float, default=5.0)
    parser.add_argument("--loops", type=float, default=1.0)
    parser.add_argument("--control-hz", type=float, default=5.0)
    parser.add_argument(
        "--clockwise",
        action="store_true",
        help="Fly clockwise instead of the default counter-clockwise orbit.",
    )
    return parser.parse_args()


def make_client() -> Any:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    return client


def runtime_contract(client: Any, vehicle_name: str) -> tuple[float, str, float]:
    try:
        settings = json.loads(client.getSettingsString())
        return (
            float(settings["ClockSpeed"]),
            str(settings["PhysicsEngineName"]),
            float(settings["Vehicles"][vehicle_name]["Z"]),
        )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"AirSim runtime settings could not be parsed: {exc}") from exc


def yaw_toward_center_degrees(x: float, y: float) -> float:
    # AirSim yaw: 0 deg points +X and +90 deg points +Y.
    return math.degrees(math.atan2(-y, -x))


def force_relative_spawn(client: Any, vehicle_name: str) -> None:
    kinematics = client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)
    kinematics.position = airsim.Vector3r(0.0, 0.0, 0.0)
    kinematics.linear_velocity = airsim.Vector3r(0.0, 0.0, 0.0)
    kinematics.angular_velocity = airsim.Vector3r(0.0, 0.0, 0.0)
    kinematics.linear_acceleration = airsim.Vector3r(0.0, 0.0, 0.0)
    kinematics.angular_acceleration = airsim.Vector3r(0.0, 0.0, 0.0)
    client.simSetKinematics(
        kinematics,
        ignore_collision=True,
        vehicle_name=vehicle_name,
    )


def main() -> int:
    args = parse_args()
    if args.spawn_altitude_m <= 0.0:
        raise SystemExit("[ERROR] --spawn-altitude-m must be positive")
    if args.radius_m < 30.0:
        raise SystemExit("[ERROR] --radius-m must be at least 30 m")
    if not 0.5 <= args.speed_mps <= 10.0:
        raise SystemExit("[ERROR] --speed-mps must be within 0.5~10 m/s")
    if args.loops <= 0.0:
        raise SystemExit("[ERROR] --loops must be positive")
    if not 2.0 <= args.control_hz <= 20.0:
        raise SystemExit("[ERROR] --control-hz must be within 2~20 Hz")

    client = make_client()
    clock_speed, physics_engine, spawn_z = runtime_contract(client, args.vehicle)
    expected_z = -abs(args.spawn_altitude_m)
    errors: list[str] = []
    if not math.isclose(clock_speed, 1.0, abs_tol=1e-9):
        errors.append(f"ClockSpeed={clock_speed} (required 1.0)")
    if physics_engine != "FastPhysicsEngine":
        errors.append(
            f"PhysicsEngineName={physics_engine!r} (required 'FastPhysicsEngine')"
        )
    if not math.isclose(spawn_z, expected_z, abs_tol=1e-6):
        errors.append(f"{args.vehicle}.Z={spawn_z} (required {expected_z})")
    if errors:
        raise SystemExit(
            "[ERROR] Runtime profile mismatch: "
            + "; ".join(errors)
            + ". Run 00_apply_30m_profile.sh, restart Unreal Play/PIE, then retry."
        )

    stop_requested = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print("[PREFLIGHT] Resetting Drone1 to the configured 30 m spawn.")
    client.reset()
    client.enableApiControl(True, vehicle_name=args.vehicle)
    client.armDisarm(True, vehicle_name=args.vehicle)
    force_relative_spawn(client, args.vehicle)
    # An airborne settings spawn still starts with SimpleFlight's landed flag.
    # takeoffAsync clears that flag; the orbit then returns to relative Z=0.
    client.takeoffAsync(timeout_sec=15.0, vehicle_name=args.vehicle).join()
    client.moveToZAsync(
        0.0,
        2.0,
        timeout_sec=15.0,
        vehicle_name=args.vehicle,
    ).join()
    client.hoverAsync(vehicle_name=args.vehicle).join()

    state = client.getMultirotorState(vehicle_name=args.vehicle)
    initial = state.kinematics_estimated.position
    initial_offset = math.sqrt(
        float(initial.x_val) ** 2
        + float(initial.y_val) ** 2
        + float(initial.z_val) ** 2
    )
    if initial_offset > 0.5:
        raise SystemExit(
            f"[ERROR] Spawn lock failed: relative offset={initial_offset:.3f} m"
        )

    orbit_z = 0.0
    approach_speed = min(4.0, args.speed_mps)
    direction = -1.0 if args.clockwise else 1.0
    orbit_seconds = args.loops * 2.0 * math.pi * args.radius_m / args.speed_mps
    command_period = 1.0 / args.control_hz
    radial_gain = 0.8
    max_command_speed = args.speed_mps * 1.35

    print()
    print("=== KAU 30 m building inspection orbit ===")
    print(f"Altitude : {args.spawn_altitude_m:.1f} m local")
    print(f"Radius   : {args.radius_m:.1f} m")
    print(f"Speed    : {args.speed_mps:.1f} m/s")
    print(f"Loops    : {args.loops:.2f}")
    print(f"Orbit ETA: {orbit_seconds:.1f} s, plus approach")
    print("Camera   : vehicle nose points toward the campus center")
    print("End      : hover; no automatic landing")
    print()

    collision_abort = False
    try:
        print("[FLIGHT] Approaching the east side of the orbit.")
        client.moveToPositionAsync(
            args.radius_m,
            0.0,
            orbit_z,
            approach_speed,
            timeout_sec=max(30.0, args.radius_m / approach_speed + 20.0),
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=180.0),
            lookahead=-1.0,
            adaptive_lookahead=0,
            vehicle_name=args.vehicle,
        ).join()

        print("[FLIGHT] Orbit started. Press Ctrl+C to stop and hover.")
        start = time.monotonic()
        next_status = start
        while not stop_requested:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= orbit_seconds:
                break

            theta = direction * args.speed_mps * elapsed / args.radius_m
            target_x = args.radius_m * math.cos(theta)
            target_y = args.radius_m * math.sin(theta)

            state = client.getMultirotorState(vehicle_name=args.vehicle)
            position = state.kinematics_estimated.position
            current_x = float(position.x_val)
            current_y = float(position.y_val)

            tangent_x = -direction * args.speed_mps * math.sin(theta)
            tangent_y = direction * args.speed_mps * math.cos(theta)
            velocity_x = tangent_x + radial_gain * (target_x - current_x)
            velocity_y = tangent_y + radial_gain * (target_y - current_y)
            command_speed = math.hypot(velocity_x, velocity_y)
            if command_speed > max_command_speed:
                scale = max_command_speed / command_speed
                velocity_x *= scale
                velocity_y *= scale

            yaw_degrees = yaw_toward_center_degrees(current_x, current_y)
            client.moveByVelocityZAsync(
                velocity_x,
                velocity_y,
                orbit_z,
                command_period,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(
                    is_rate=False,
                    yaw_or_rate=yaw_degrees,
                ),
                vehicle_name=args.vehicle,
            ).join()

            collision = client.simGetCollisionInfo(vehicle_name=args.vehicle)
            if collision.has_collided:
                print(
                    "[SAFETY] Collision reported; orbit aborted "
                    f"(object={collision.object_name!r})."
                )
                collision_abort = True
                break

            if now >= next_status:
                radius_now = math.hypot(current_x, current_y)
                print(
                    f"[FLIGHT] {elapsed:6.1f}/{orbit_seconds:.1f} s "
                    f"position=({current_x:7.1f},{current_y:7.1f}) m "
                    f"radius={radius_now:5.1f} m"
                )
                next_status = now + 10.0

    finally:
        try:
            client.cancelLastTask(vehicle_name=args.vehicle)
        except Exception:
            pass
        print("[FLIGHT] Orbit ended. Drone1 is entering hover.")
        client.hoverAsync(vehicle_name=args.vehicle).join()

    if collision_abort:
        return 2
    print("[OK] Building inspection orbit complete; Drone1 remains hovering.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
