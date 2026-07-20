#!/usr/bin/env python3
"""Apply the minimum fixed-wing External Physics settings to an AirSim profile."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--settings",
        type=Path,
        default=Path.home() / "Documents/AirSim/settings.json",
        help="existing settings.json used as the source profile",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="output path; defaults to updating --settings in place",
    )
    parser.add_argument(
        "--profile",
        choices=("validation", "runtime"),
        default="validation",
        help="validation disables IMU noise; runtime preserves existing noise values",
    )
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--camera", default="cam0")
    parser.add_argument("--imu", default="Imu")
    parser.add_argument("--camera-forward-m", type=float, default=1.0)
    parser.add_argument("--camera-down-m", type=float, default=0.0)
    parser.add_argument("--camera-pitch-deg", type=float, default=-30.0)
    parser.add_argument("--observer-follow-distance-m", type=int, default=-2)
    parser.add_argument("--no-backup", action="store_true")
    return parser.parse_args()


def load_settings(path: Path) -> dict:
    if not path.is_file():
        raise SystemExit(f"[ERROR] source settings file not found: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise SystemExit(f"[ERROR] settings JSON parse failed: {exc}") from exc
    if not isinstance(data, dict):
        raise SystemExit("[ERROR] settings JSON root must be an object")
    return data


def require_mapping(parent: dict, key: str, context: str) -> dict:
    value = parent.get(key)
    if not isinstance(value, dict):
        raise SystemExit(f"[ERROR] required object missing: {context}.{key}")
    return value


def main() -> int:
    args = parse_args()
    source = args.settings.expanduser().resolve()
    output = (args.output or source).expanduser().resolve()
    data = load_settings(source)

    vehicles = require_mapping(data, "Vehicles", "settings")
    drone = require_mapping(vehicles, args.vehicle, "settings.Vehicles")
    cameras = require_mapping(drone, "Cameras", f"Vehicles.{args.vehicle}")
    camera = require_mapping(cameras, args.camera, f"Vehicles.{args.vehicle}.Cameras")
    sensors = require_mapping(drone, "Sensors", f"Vehicles.{args.vehicle}")
    imu = require_mapping(sensors, args.imu, f"Vehicles.{args.vehicle}.Sensors")

    data["SettingsVersion"] = 1.2
    data["SimMode"] = "Multirotor"
    data["ClockSpeed"] = 1.0
    data["PhysicsEngineName"] = "ExternalPhysicsEngine"
    data["RpcEnabled"] = True
    data.setdefault("ApiServerPort", 41451)
    # The fixed-wing runtime patch turns SpringArmChase into a horizon-stable,
    # heading-following observer without changing the body-mounted cam0.
    data["ViewMode"] = "SpringArmChase"
    camera_director = data.setdefault("CameraDirector", {})
    if not isinstance(camera_director, dict):
        raise SystemExit("[ERROR] settings.CameraDirector must be an object")
    camera_director["FollowDistance"] = args.observer_follow_distance_m

    drone["VehicleType"] = "SimpleFlight"
    drone["AutoCreate"] = True
    camera.update(
        {
            "X": args.camera_forward_m,
            "Y": 0.0,
            "Z": args.camera_down_m,
            "Pitch": args.camera_pitch_deg,
            "Roll": 0.0,
            "Yaw": 0.0,
        }
    )
    capture_settings = camera.setdefault("CaptureSettings", [])
    if not isinstance(capture_settings, list):
        raise SystemExit(
            f"[ERROR] Vehicles.{args.vehicle}.Cameras.{args.camera}.CaptureSettings "
            "must be an array"
        )
    scene_capture = next(
        (
            item
            for item in capture_settings
            if isinstance(item, dict) and item.get("ImageType", 0) == 0
        ),
        None,
    )
    if scene_capture is None:
        scene_capture = {"ImageType": 0}
        capture_settings.append(scene_capture)
    scene_capture.update(
        {
            "Width": 640,
            "Height": 480,
            "FOV_Degrees": 90.0,
            "MotionBlurAmount": 0,
        }
    )
    imu["SensorType"] = 2
    imu["Enabled"] = True

    if args.profile == "validation":
        imu.update(
            {
                "AngularRandomWalk": 0.0,
                "GyroBiasStability": 0.0,
                "VelocityRandomWalk": 0.0,
                "AccelBiasStability": 0.0,
            }
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    if output == source and not args.no_backup:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = source.with_name(f"{source.name}.backup.fixedwing_{stamp}")
        shutil.copy2(source, backup)
        print(f"[BACKUP] {backup}")

    output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    print(f"[OK] fixed-wing {args.profile} profile written: {output}")
    print("[OK] PhysicsEngineName=ExternalPhysicsEngine, ClockSpeed=1.0")
    print("[OK] ViewMode=SpringArmChase (fixed-wing observer patch enabled)")
    print(
        f"[OK] fixed-wing observer FollowDistance="
        f"{args.observer_follow_distance_m} m"
    )
    print(
        f"[OK] {args.camera} mount X={args.camera_forward_m:.3f} m, "
        f"Z={args.camera_down_m:.3f} m, Pitch={args.camera_pitch_deg:.1f} deg"
    )
    print(f"[OK] {args.camera} Scene capture=640x480, FOV=90 deg, motion blur=0")
    if args.profile == "validation":
        print("[OK] AirSim IMU random walk and bias stability disabled for axis tests")
    print("[IMPORTANT] Stop and restart Unreal Play/PIE before running the probe.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
