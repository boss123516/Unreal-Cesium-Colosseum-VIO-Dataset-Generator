#!/usr/bin/env python3
"""Deliberately hit one known KAU LoD1 wall and record the collision."""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    import airsim
except ImportError as exc:
    raise SystemExit(
        "[ERROR] Could not import airsim. Activate ~/vio_sim_ws/airsim_pyenv "
        "and add Colosseum/PythonClient to PYTHONPATH."
    ) from exc


DEFAULT_TARGET_X_M = 37.103313
DEFAULT_TARGET_Y_M = 52.579754
DEFAULT_BUILDING_UFID = "2004188062744552817400000000"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Hit a known KAU building wall at low speed."
    )
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--spawn-altitude-m", type=float, default=30.0)
    parser.add_argument("--impact-altitude-m", type=float, default=8.0)
    parser.add_argument("--impact-speed-mps", type=float, default=2.0)
    parser.add_argument("--warmup-sec", type=float, default=12.0)
    parser.add_argument("--target-x-m", type=float, default=DEFAULT_TARGET_X_M)
    parser.add_argument("--target-y-m", type=float, default=DEFAULT_TARGET_Y_M)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path.home() / "vio_sim_ws" / "datasets",
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


def vector_dict(vector: Any) -> dict[str, float]:
    return {
        "x": float(vector.x_val),
        "y": float(vector.y_val),
        "z": float(vector.z_val),
    }


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


def write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    if args.spawn_altitude_m <= 0.0:
        raise SystemExit("[ERROR] --spawn-altitude-m must be positive")
    if not 2.0 <= args.impact_altitude_m < args.spawn_altitude_m:
        raise SystemExit(
            "[ERROR] --impact-altitude-m must be >=2 m and below spawn altitude"
        )
    if not 0.5 <= args.impact_speed_mps <= 4.0:
        raise SystemExit("[ERROR] --impact-speed-mps must be within 0.5~4 m/s")
    if args.warmup_sec < 0.0:
        raise SystemExit("[ERROR] --warmup-sec must be non-negative")

    client = make_client()
    clock_speed, physics_engine, spawn_z = runtime_contract(
        client,
        args.vehicle,
    )
    expected_spawn_z = -abs(args.spawn_altitude_m)
    errors: list[str] = []
    if not math.isclose(clock_speed, 1.0, abs_tol=1e-9):
        errors.append(f"ClockSpeed={clock_speed} (required 1.0)")
    if physics_engine != "FastPhysicsEngine":
        errors.append(
            f"PhysicsEngineName={physics_engine!r} (required 'FastPhysicsEngine')"
        )
    if not math.isclose(spawn_z, expected_spawn_z, abs_tol=1e-6):
        errors.append(f"{args.vehicle}.Z={spawn_z} (required {expected_spawn_z})")
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

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = args.output_root / f"kau_collision_demo_{timestamp}.json"
    target_z = args.spawn_altitude_m - args.impact_altitude_m
    target_yaw = math.degrees(math.atan2(args.target_y_m, args.target_x_m))
    straight_distance = math.hypot(args.target_x_m, args.target_y_m)

    print("[PREFLIGHT] Resetting Drone1 to the configured 30 m spawn.")
    client.reset()
    client.enableApiControl(True, vehicle_name=args.vehicle)
    client.armDisarm(True, vehicle_name=args.vehicle)
    force_relative_spawn(client, args.vehicle)
    # SimpleFlight marks an airborne settings spawn as landed until a takeoff
    # command is issued. Clear that state before horizontal motion.
    client.takeoffAsync(timeout_sec=15.0, vehicle_name=args.vehicle).join()
    client.hoverAsync(vehicle_name=args.vehicle).join()

    print()
    print("=== KAU deliberate building-collision demo ===")
    print(f"Known building UFID : {DEFAULT_BUILDING_UFID}")
    print(f"Building centroid   : ({args.target_x_m:.3f}, {args.target_y_m:.3f}) m")
    print("Expected first wall : approximately (26.967, 38.215) m")
    print(f"Impact altitude     : {args.impact_altitude_m:.1f} m local")
    print(f"Impact speed        : {args.impact_speed_mps:.1f} m/s")
    print(f"Warmup              : {args.warmup_sec:.1f} s")
    print()

    if args.warmup_sec:
        print("[WAIT] Allowing nearby Cesium physics meshes to finish loading.")
        deadline = time.monotonic() + args.warmup_sec
        while not stop_requested and time.monotonic() < deadline:
            time.sleep(0.1)

    collision_detected = False
    collision: Any | None = None
    final_position: Any | None = None
    started_at = datetime.now().isoformat()
    start_monotonic = time.monotonic()

    try:
        if stop_requested:
            raise KeyboardInterrupt

        print(
            f"[FLIGHT] Descending from {args.spawn_altitude_m:.1f} m "
            f"to {args.impact_altitude_m:.1f} m at the clear origin."
        )
        client.moveToZAsync(
            target_z,
            2.0,
            timeout_sec=30.0,
            vehicle_name=args.vehicle,
        ).join()
        client.hoverAsync(vehicle_name=args.vehicle).join()

        baseline = client.simGetCollisionInfo(vehicle_name=args.vehicle)
        if baseline.has_collided:
            raise RuntimeError(
                "Collision was already set before the wall approach; "
                "restart Play/PIE and retry."
            )

        print("[FLIGHT] Moving straight toward the measured 17 m-high building.")
        client.moveToPositionAsync(
            args.target_x_m,
            args.target_y_m,
            target_z,
            args.impact_speed_mps,
            timeout_sec=straight_distance / args.impact_speed_mps + 20.0,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(
                is_rate=False,
                yaw_or_rate=target_yaw,
            ),
            lookahead=-1.0,
            adaptive_lookahead=0,
            vehicle_name=args.vehicle,
        )

        timeout = time.monotonic() + straight_distance / args.impact_speed_mps + 20.0
        next_status = time.monotonic()
        while not stop_requested and time.monotonic() < timeout:
            collision = client.simGetCollisionInfo(vehicle_name=args.vehicle)
            state = client.getMultirotorState(vehicle_name=args.vehicle)
            final_position = state.kinematics_estimated.position
            if collision.has_collided:
                collision_detected = True
                print(
                    "[COLLISION_CONFIRMED] "
                    f"object={collision.object_name!r} "
                    f"position=({final_position.x_val:.3f},"
                    f"{final_position.y_val:.3f},{final_position.z_val:.3f})"
                )
                break

            now = time.monotonic()
            if now >= next_status:
                traveled = math.hypot(
                    float(final_position.x_val),
                    float(final_position.y_val),
                )
                print(
                    f"[FLIGHT] horizontal progress "
                    f"{traveled:.1f}/{straight_distance:.1f} m"
                )
                next_status = now + 2.0
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("[STOP] Collision demo interrupted by the user.")
    finally:
        try:
            client.cancelLastTask(vehicle_name=args.vehicle)
        except Exception:
            pass
        try:
            client.hoverAsync(vehicle_name=args.vehicle).join()
        except Exception as exc:
            print(f"[WARN] hover failed: {exc}")

    if final_position is None:
        state = client.getMultirotorState(vehicle_name=args.vehicle)
        final_position = state.kinematics_estimated.position

    report: dict[str, Any] = {
        "demo": "KAU deliberate LoD1 building collision",
        "started_local": started_at,
        "finished_local": datetime.now().isoformat(),
        "elapsed_seconds": round(time.monotonic() - start_monotonic, 3),
        "building": {
            "ufid": DEFAULT_BUILDING_UFID,
            "height_m": 17.0,
            "base_ellipsoid_height_m": 37.631,
            "roof_ellipsoid_height_m": 54.631,
            "target_centroid_airsim_local_m": {
                "x": args.target_x_m,
                "y": args.target_y_m,
            },
            "expected_first_wall_airsim_local_m": {
                "x": 26.966942,
                "y": 38.215324,
            },
        },
        "flight": {
            "spawn_altitude_m": args.spawn_altitude_m,
            "impact_altitude_m": args.impact_altitude_m,
            "impact_speed_mps": args.impact_speed_mps,
            "warmup_sec": args.warmup_sec,
            "final_position_ned_m": vector_dict(final_position),
        },
        "collision_detected": collision_detected,
        "collision": None,
    }
    if collision is not None:
        report["collision"] = {
            "has_collided": bool(collision.has_collided),
            "object_name": str(collision.object_name),
            "object_id": int(collision.object_id),
            "penetration_depth": float(collision.penetration_depth),
            "time_stamp": int(collision.time_stamp),
            "normal": vector_dict(collision.normal),
            "impact_point": vector_dict(collision.impact_point),
            "position": vector_dict(collision.position),
        }
    write_report(report_path, report)

    print(f"[REPORT] {report_path}")
    if collision_detected:
        print("[PASS] The quad produced a real physics collision with the 3D tile.")
        print("[STATE] Drone1 remains hovering; run 02_safe_recover.py to land.")
        return 0

    print("[FAIL] No building collision was detected before timeout.")
    print("[HINT] Confirm EnableCollision=True and wait for the tile to load.")
    return 2


if __name__ == "__main__":
    sys.exit(main())
