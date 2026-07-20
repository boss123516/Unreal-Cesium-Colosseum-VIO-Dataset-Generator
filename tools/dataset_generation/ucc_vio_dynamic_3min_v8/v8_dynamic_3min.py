#!/usr/bin/env python3
"""
30 m/s, 180-second v8 AirSim flight and synchronized dataset recorder.

Behavior:
- Recording starts immediately and lasts exactly `duration_sec` in wall time.
- Flight starts directly from the configured airborne spawn; no takeoff climb.
- Camera: exact 10 Hz target grid, compressed Scene PNG.
- IMU + GT: exact 100 Hz target grid.
- Runtime contract: ClockSpeed 1.0 and Drone1 spawn at local NED Z=-400 m.
- Flight: continuous 3D slalom / broad turns inside the 300-500 m band.
- At timeout: cancel motion and hover.
- No landing, no disarm, no API-control release.
- CSV/JSON files are finalized before the process exits.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import statistics
import sys
import threading
import time
import traceback
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

try:
    import airsim
except ImportError as exc:
    raise SystemExit(
        "[ERROR] Could not import airsim. Activate ~/vio_sim_ws/airsim_pyenv "
        "and add Colosseum/PythonClient to PYTHONPATH."
    ) from exc


NANOSECONDS = 1_000_000_000


@dataclass
class SharedState:
    stop_event: threading.Event = field(default_factory=threading.Event)
    errors: list[str] = field(default_factory=list)
    camera_source_timestamps: list[int] = field(default_factory=list)
    camera_mapping_errors: list[int] = field(default_factory=list)
    imu_source_timestamps: list[int] = field(default_factory=list)
    imu_mapping_errors: list[int] = field(default_factory=list)
    gt_source_timestamps: list[int] = field(default_factory=list)
    gt_mapping_errors: list[int] = field(default_factory=list)
    gt_ned_z_values: list[float] = field(default_factory=list)
    camera_written: int = 0
    imu_written: int = 0
    gt_written: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def add_error(self, message: str) -> None:
        with self.lock:
            self.errors.append(message)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=180.0)
    parser.add_argument("--speed-mps", type=float, default=30.0)
    parser.add_argument("--camera-hz", type=float, default=10.0)
    parser.add_argument("--imu-hz", type=float, default=100.0)
    parser.add_argument("--spawn-altitude-m", type=float, default=400.0)
    parser.add_argument("--min-altitude-m", type=float, default=300.0)
    parser.add_argument("--max-altitude-m", type=float, default=500.0)
    parser.add_argument("--altitude-command-margin-m", type=float, default=15.0)
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--camera", default="cam0")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path.home() / "vio_sim_ws" / "datasets",
    )
    parser.add_argument(
        "--start-delay-sec",
        type=float,
        default=1.0,
        help="Shared delay before the first target timestamp.",
    )
    return parser.parse_args()


def sleep_until(deadline: float, stop_event: threading.Event) -> bool:
    while not stop_event.is_set():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return True
        stop_event.wait(min(remaining, 0.02))
    return False


def make_client() -> Any:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    return client


def safe_timestamp(value: Any) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def get_runtime_contract(client: Any, vehicle_name: str) -> dict[str, float]:
    try:
        settings = json.loads(client.getSettingsString())
    except Exception as exc:
        raise RuntimeError(f"AirSim runtime settings could not be parsed: {exc}") from exc

    try:
        vehicle = settings["Vehicles"][vehicle_name]
        return {
            "clock_speed": float(settings["ClockSpeed"]),
            "spawn_x_ned_m": float(vehicle["X"]),
            "spawn_y_ned_m": float(vehicle["Y"]),
            "spawn_z_ned_m": float(vehicle["Z"]),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(
            f"AirSim runtime settings are missing the {vehicle_name} spawn contract: {exc}"
        ) from exc


def local_altitude_stats(
    ned_z_values: list[float],
    altitude_reference_ned_z: float,
    spawn_altitude_m: float,
) -> dict[str, Any]:
    if not ned_z_values:
        return {
            "count": 0,
            "first_m": None,
            "last_m": None,
            "min_m": None,
            "max_m": None,
        }

    altitudes = [
        spawn_altitude_m - (z - altitude_reference_ned_z)
        for z in ned_z_values
    ]
    return {
        "count": len(altitudes),
        "first_m": altitudes[0],
        "last_m": altitudes[-1],
        "min_m": min(altitudes),
        "max_m": max(altitudes),
    }


def mapping_stats(errors_ns: list[int]) -> dict[str, Any]:
    if not errors_ns:
        return {"count": 0, "max_abs_ms": None, "mean_abs_ms": None, "p95_abs_ms": None}
    values_ms = sorted(abs(v) / 1e6 for v in errors_ns)
    p95_index = min(len(values_ms) - 1, max(0, math.ceil(len(values_ms) * 0.95) - 1))
    return {
        "count": len(values_ms),
        "max_abs_ms": max(values_ms),
        "mean_abs_ms": statistics.fmean(values_ms),
        "p95_abs_ms": values_ms[p95_index],
    }


def camera_worker(
    shared: SharedState,
    dataset_root: Path,
    start_monotonic: float,
    epoch_ns: int,
    duration_sec: float,
    hz: float,
    vehicle_name: str,
    camera_name: str,
) -> None:
    client = make_client()
    period_sec = 1.0 / hz
    period_ns = round(NANOSECONDS / hz)
    expected = int(round(duration_sec * hz))

    image_dir = dataset_root / "mav0" / "cam0" / "data"
    csv_path = dataset_root / "mav0" / "cam0" / "data.csv"
    mapping_path = dataset_root / "mav0" / "cam0" / "mapping.csv"

    with csv_path.open("w", newline="", encoding="utf-8") as data_file, \
         mapping_path.open("w", newline="", encoding="utf-8") as mapping_file:
        data_writer = csv.writer(data_file)
        mapping_writer = csv.writer(mapping_file)
        data_writer.writerow(["#timestamp [ns]", "filename"])
        mapping_writer.writerow(
            ["target_timestamp_ns", "source_timestamp_ns", "timestamp_error_ns"]
        )

        for index in range(expected):
            if not sleep_until(start_monotonic + index * period_sec, shared.stop_event):
                break

            target_ns = epoch_ns + index * period_ns
            try:
                responses = client.simGetImages(
                    [
                        airsim.ImageRequest(
                            camera_name,
                            airsim.ImageType.Scene,
                            pixels_as_float=False,
                            compress=True,
                        )
                    ],
                    vehicle_name=vehicle_name,
                )
                if not responses:
                    raise RuntimeError("simGetImages returned no responses")

                response = responses[0]
                source_ns = safe_timestamp(response.time_stamp)
                image_bytes = bytes(response.image_data_uint8)

                if response.width <= 0 or response.height <= 0 or not image_bytes:
                    raise RuntimeError(
                        f"invalid frame width={response.width}, "
                        f"height={response.height}, bytes={len(image_bytes)}"
                    )

                filename = f"{target_ns}.png"
                (image_dir / filename).write_bytes(image_bytes)
                data_writer.writerow([target_ns, filename])
                mapping_writer.writerow([target_ns, source_ns, source_ns - target_ns])
                data_file.flush()
                mapping_file.flush()

                with shared.lock:
                    shared.camera_written += 1
                    shared.camera_source_timestamps.append(source_ns)
                    shared.camera_mapping_errors.append(source_ns - target_ns)

            except Exception as exc:
                shared.add_error(f"camera[{index}]: {type(exc).__name__}: {exc}")


def inertial_gt_worker(
    shared: SharedState,
    dataset_root: Path,
    start_monotonic: float,
    epoch_ns: int,
    duration_sec: float,
    hz: float,
    vehicle_name: str,
) -> None:
    client = make_client()
    period_sec = 1.0 / hz
    period_ns = round(NANOSECONDS / hz)
    expected = int(round(duration_sec * hz))

    imu_csv = dataset_root / "mav0" / "imu0" / "data.csv"
    imu_mapping_csv = dataset_root / "mav0" / "imu0" / "mapping.csv"
    gt_csv = dataset_root / "mav0" / "state_groundtruth_estimate0" / "data.csv"
    gt_mapping_csv = dataset_root / "mav0" / "state_groundtruth_estimate0" / "mapping.csv"

    with imu_csv.open("w", newline="", encoding="utf-8") as imu_file, \
         imu_mapping_csv.open("w", newline="", encoding="utf-8") as imu_map_file, \
         gt_csv.open("w", newline="", encoding="utf-8") as gt_file, \
         gt_mapping_csv.open("w", newline="", encoding="utf-8") as gt_map_file:

        imu_writer = csv.writer(imu_file)
        imu_map_writer = csv.writer(imu_map_file)
        gt_writer = csv.writer(gt_file)
        gt_map_writer = csv.writer(gt_map_file)

        imu_writer.writerow(
            [
                "#timestamp [ns]",
                "w_RS_S_x [rad s^-1]",
                "w_RS_S_y [rad s^-1]",
                "w_RS_S_z [rad s^-1]",
                "a_RS_S_x [m s^-2]",
                "a_RS_S_y [m s^-2]",
                "a_RS_S_z [m s^-2]",
            ]
        )
        imu_map_writer.writerow(
            ["target_timestamp_ns", "source_timestamp_ns", "timestamp_error_ns"]
        )
        gt_writer.writerow(
            [
                "#timestamp [ns]",
                "p_RS_R_x [m]",
                "p_RS_R_y [m]",
                "p_RS_R_z [m]",
                "q_RS_w []",
                "q_RS_x []",
                "q_RS_y []",
                "q_RS_z []",
                "v_RS_R_x [m s^-1]",
                "v_RS_R_y [m s^-1]",
                "v_RS_R_z [m s^-1]",
                "w_RS_S_x [rad s^-1]",
                "w_RS_S_y [rad s^-1]",
                "w_RS_S_z [rad s^-1]",
                "a_RS_S_x [m s^-2]",
                "a_RS_S_y [m s^-2]",
                "a_RS_S_z [m s^-2]",
                "latitude [deg]",
                "longitude [deg]",
                "altitude [m]",
            ]
        )
        gt_map_writer.writerow(
            ["target_timestamp_ns", "source_timestamp_ns", "timestamp_error_ns"]
        )

        for index in range(expected):
            if not sleep_until(start_monotonic + index * period_sec, shared.stop_event):
                break

            target_ns = epoch_ns + index * period_ns

            try:
                imu = client.getImuData(imu_name="Imu", vehicle_name=vehicle_name)
                imu_source_ns = safe_timestamp(imu.time_stamp)

                av = imu.angular_velocity
                la = imu.linear_acceleration
                imu_writer.writerow(
                    [
                        target_ns,
                        av.x_val,
                        av.y_val,
                        av.z_val,
                        la.x_val,
                        la.y_val,
                        la.z_val,
                    ]
                )
                imu_map_writer.writerow(
                    [target_ns, imu_source_ns, imu_source_ns - target_ns]
                )

                with shared.lock:
                    shared.imu_written += 1
                    shared.imu_source_timestamps.append(imu_source_ns)
                    shared.imu_mapping_errors.append(imu_source_ns - target_ns)

            except Exception as exc:
                shared.add_error(f"imu[{index}]: {type(exc).__name__}: {exc}")

            try:
                state = client.getMultirotorState(vehicle_name=vehicle_name)
                gt_source_ns = safe_timestamp(state.timestamp)
                kin = state.kinematics_estimated
                pos = kin.position
                ori = kin.orientation
                lv = kin.linear_velocity
                av = kin.angular_velocity
                la = kin.linear_acceleration
                gps = state.gps_location

                gt_writer.writerow(
                    [
                        target_ns,
                        pos.x_val,
                        pos.y_val,
                        pos.z_val,
                        ori.w_val,
                        ori.x_val,
                        ori.y_val,
                        ori.z_val,
                        lv.x_val,
                        lv.y_val,
                        lv.z_val,
                        av.x_val,
                        av.y_val,
                        av.z_val,
                        la.x_val,
                        la.y_val,
                        la.z_val,
                        gps.latitude,
                        gps.longitude,
                        gps.altitude,
                    ]
                )
                gt_map_writer.writerow([target_ns, gt_source_ns, gt_source_ns - target_ns])

                with shared.lock:
                    shared.gt_written += 1
                    shared.gt_source_timestamps.append(gt_source_ns)
                    shared.gt_mapping_errors.append(gt_source_ns - target_ns)
                    shared.gt_ned_z_values.append(float(pos.z_val))

            except Exception as exc:
                shared.add_error(f"gt[{index}]: {type(exc).__name__}: {exc}")

            if index % max(1, int(hz)) == 0:
                imu_file.flush()
                imu_map_file.flush()
                gt_file.flush()
                gt_map_file.flush()


def smoothstep(value: float) -> float:
    x = min(1.0, max(0.0, value))
    return x * x * (3.0 - 2.0 * x)


def smoothstep_rate(value: float, duration_sec: float) -> float:
    if value <= 0.0 or value >= 1.0:
        return 0.0
    return 6.0 * value * (1.0 - value) / duration_sec


def generate_dynamic_path(
    origin: Any,
    duration_sec: float,
    speed_mps: float,
    spawn_altitude_m: float,
    min_altitude_m: float,
    max_altitude_m: float,
    altitude_command_margin_m: float,
    dt: float = 0.5,
) -> list[Any]:
    """
    Creates a continuous 3D path inside a local-altitude command band.

    NED convention:
    - +x north/forward local
    - +y east/right local
    - +z down
    """
    if not min_altitude_m < spawn_altitude_m < max_altitude_m:
        raise ValueError("spawn altitude must be strictly inside the altitude bounds")
    if altitude_command_margin_m < 0:
        raise ValueError("altitude command margin must be non-negative")

    climb_room = max_altitude_m - spawn_altitude_m
    descent_room = spawn_altitude_m - min_altitude_m
    amplitude_m = min(climb_room, descent_room) - altitude_command_margin_m
    if amplitude_m <= 0:
        raise ValueError("altitude command margin leaves no usable vertical range")

    point_count = max(2, int(math.ceil(duration_sec / dt)) + 1)
    x = float(origin.x_val)
    y = float(origin.y_val)
    altitude_reference_ned_z = float(origin.z_val)
    path: list[Any] = []

    for index in range(point_count):
        t = index * dt

        heading = (
            0.82 * math.sin(2.0 * math.pi * t / 27.0)
            + 0.30 * math.sin(2.0 * math.pi * t / 8.5)
        )

        if 42.0 <= t <= 76.0:
            heading += 2.0 * math.pi * smoothstep((t - 42.0) / 34.0)
        elif t > 76.0:
            heading += 2.0 * math.pi

        if 104.0 <= t <= 140.0:
            heading -= 2.0 * math.pi * smoothstep((t - 104.0) / 36.0)
        elif t > 140.0:
            heading -= 2.0 * math.pi

        raw_altitude_wave = (
            0.75 * math.sin(2.0 * math.pi * t / 60.0)
            + 0.25 * math.sin(2.0 * math.pi * t / 23.0)
        )
        raw_altitude_wave_rate = (
            0.75 * (2.0 * math.pi / 60.0) * math.cos(2.0 * math.pi * t / 60.0)
            + 0.25 * (2.0 * math.pi / 23.0) * math.cos(2.0 * math.pi * t / 23.0)
        )

        # Start horizontally at exactly the configured spawn altitude. Hold it
        # briefly, then blend in the vertical profile instead of starting with
        # an immediate climb.
        vertical_hold_sec = 5.0
        vertical_blend_duration_sec = 10.0
        vertical_blend_value = (
            t - vertical_hold_sec
        ) / vertical_blend_duration_sec
        vertical_blend = smoothstep(vertical_blend_value)
        vertical_blend_rate = smoothstep_rate(
            vertical_blend_value, vertical_blend_duration_sec
        )
        altitude_wave = vertical_blend * raw_altitude_wave
        altitude_wave_rate = (
            vertical_blend_rate * raw_altitude_wave
            + vertical_blend * raw_altitude_wave_rate
        )
        altitude_m = spawn_altitude_m + amplitude_m * altitude_wave
        vertical_speed = -amplitude_m * altitude_wave_rate
        horizontal_speed = math.sqrt(
            max(0.0, speed_mps * speed_mps - vertical_speed * vertical_speed)
        )

        x += horizontal_speed * math.cos(heading) * dt
        y += horizontal_speed * math.sin(heading) * dt
        z = altitude_reference_ned_z - (altitude_m - spawn_altitude_m)
        path.append(airsim.Vector3r(x, y, z))

    return path


def main() -> int:
    args = parse_args()
    if args.duration_sec <= 0:
        raise SystemExit("[ERROR] --duration-sec must be positive")
    if args.speed_mps <= 0:
        raise SystemExit("[ERROR] --speed-mps must be positive")
    if args.camera_hz <= 0 or args.imu_hz <= 0:
        raise SystemExit("[ERROR] sample rates must be positive")
    if not args.min_altitude_m < args.spawn_altitude_m < args.max_altitude_m:
        raise SystemExit("[ERROR] spawn altitude must be inside the altitude bounds")
    if args.altitude_command_margin_m < 0:
        raise SystemExit("[ERROR] altitude command margin must be non-negative")
    available_vertical_room = min(
        args.max_altitude_m - args.spawn_altitude_m,
        args.spawn_altitude_m - args.min_altitude_m,
    )
    if args.altitude_command_margin_m >= available_vertical_room:
        raise SystemExit("[ERROR] altitude command margin leaves no usable range")

    control = make_client()
    runtime_contract = get_runtime_contract(control, args.vehicle)
    expected_spawn_z = -abs(args.spawn_altitude_m)
    contract_errors: list[str] = []
    if not math.isclose(runtime_contract["clock_speed"], 1.0, abs_tol=1e-9):
        contract_errors.append(
            f"ClockSpeed={runtime_contract['clock_speed']} (required: 1.0)"
        )
    if not math.isclose(
        runtime_contract["spawn_z_ned_m"], expected_spawn_z, abs_tol=1e-6
    ):
        contract_errors.append(
            f"{args.vehicle}.Z={runtime_contract['spawn_z_ned_m']} "
            f"(required: {expected_spawn_z})"
        )
    if contract_errors:
        raise SystemExit(
            "[ERROR] AirSim runtime profile mismatch: "
            + "; ".join(contract_errors)
            + ". Run 00_apply_400m_profile.sh, fully stop Unreal Play/PIE, "
            "restart Play/PIE, then retry."
        )
    print("[PREFLIGHT] Teleporting Drone1 to the configured 400 m spawn.")
    control.reset()
    control.enableApiControl(True, vehicle_name=args.vehicle)
    control.armDisarm(True, vehicle_name=args.vehicle)

    # Vehicle API coordinates are relative to the configured starting point.
    # Force the exact spawn pose and clear the velocity that gravity can build
    # between reset() and the first flight command.
    spawn_kinematics = control.simGetGroundTruthKinematics(vehicle_name=args.vehicle)
    spawn_kinematics.position = airsim.Vector3r(0.0, 0.0, 0.0)
    spawn_kinematics.linear_velocity = airsim.Vector3r(0.0, 0.0, 0.0)
    spawn_kinematics.angular_velocity = airsim.Vector3r(0.0, 0.0, 0.0)
    spawn_kinematics.linear_acceleration = airsim.Vector3r(0.0, 0.0, 0.0)
    spawn_kinematics.angular_acceleration = airsim.Vector3r(0.0, 0.0, 0.0)
    control.simSetKinematics(
        spawn_kinematics, ignore_collision=True, vehicle_name=args.vehicle
    )
    control.hoverAsync(vehicle_name=args.vehicle).join()

    initial_state = control.getMultirotorState(vehicle_name=args.vehicle)
    initial_position = initial_state.kinematics_estimated.position
    initial_offset_m = math.sqrt(
        float(initial_position.x_val) ** 2
        + float(initial_position.y_val) ** 2
        + float(initial_position.z_val) ** 2
    )
    if initial_offset_m > 0.5:
        raise SystemExit(
            f"[ERROR] Failed to hold the configured spawn: relative offset "
            f"is {initial_offset_m:.3f} m (required <= 0.5 m)."
        )
    print(
        f"[PREFLIGHT] Spawn locked at relative NED "
        f"({initial_position.x_val:.3f}, {initial_position.y_val:.3f}, "
        f"{initial_position.z_val:.3f}) m."
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    dataset_root = args.output_root / f"ucc_dynamic_3min_{timestamp}"
    (dataset_root / "mav0" / "cam0" / "data").mkdir(parents=True, exist_ok=True)
    (dataset_root / "mav0" / "imu0").mkdir(parents=True, exist_ok=True)
    (dataset_root / "mav0" / "state_groundtruth_estimate0").mkdir(
        parents=True, exist_ok=True
    )

    incomplete_marker = dataset_root / ".recording_incomplete"
    incomplete_marker.write_text("recording in progress\n", encoding="utf-8")

    shared = SharedState()
    altitude_reference_ned_z = float(initial_position.z_val)
    epoch_ns = safe_timestamp(initial_state.timestamp)
    if epoch_ns <= 0:
        epoch_ns = time.time_ns()

    start_monotonic = time.monotonic() + args.start_delay_sec
    end_monotonic = start_monotonic + args.duration_sec

    config = {
        "vehicle": args.vehicle,
        "camera": args.camera,
        "duration_sec": args.duration_sec,
        "speed_mps": args.speed_mps,
        "camera_hz": args.camera_hz,
        "imu_hz": args.imu_hz,
        "gt_hz": args.imu_hz,
        "clock_speed_required": 1.0,
        "generator_version": "v8",
        "reset_to_configured_spawn_before_recording": True,
        "spawn_pose_forced_and_velocity_cleared": True,
        "runtime_settings": runtime_contract,
        "spawn_altitude_m": args.spawn_altitude_m,
        "spawn_z_ned_required_m": expected_spawn_z,
        "min_altitude_m": args.min_altitude_m,
        "max_altitude_m": args.max_altitude_m,
        "altitude_command_margin_m": args.altitude_command_margin_m,
        "altitude_frame": "local height relative to the configured 400 m spawn",
        "initial_position_ned_m": {
            "x": float(initial_position.x_val),
            "y": float(initial_position.y_val),
            "z": float(initial_position.z_val),
        },
        "landing_at_end": False,
        "end_behavior": "hover; armed; API control retained",
        "epoch_timestamp_ns": epoch_ns,
        "created_local": datetime.now().isoformat(),
    }
    write_json(dataset_root / "run_config.json", config)

    def request_stop(signum: int, _frame: Any) -> None:
        shared.add_error(f"signal received: {signum}")
        shared.stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    camera_thread = threading.Thread(
        target=camera_worker,
        name="camera-10hz",
        kwargs={
            "shared": shared,
            "dataset_root": dataset_root,
            "start_monotonic": start_monotonic,
            "epoch_ns": epoch_ns,
            "duration_sec": args.duration_sec,
            "hz": args.camera_hz,
            "vehicle_name": args.vehicle,
            "camera_name": args.camera,
        },
        daemon=False,
    )
    inertial_thread = threading.Thread(
        target=inertial_gt_worker,
        name="imu-gt-100hz",
        kwargs={
            "shared": shared,
            "dataset_root": dataset_root,
            "start_monotonic": start_monotonic,
            "epoch_ns": epoch_ns,
            "duration_sec": args.duration_sec,
            "hz": args.imu_hz,
            "vehicle_name": args.vehicle,
        },
        daemon=False,
    )

    camera_thread.start()
    inertial_thread.start()

    print()
    print("=== UCC VIO dynamic 3-minute run ===")
    print(f"Dataset root : {dataset_root}")
    print(f"Vehicle      : {args.vehicle}")
    print(f"Speed limit  : {args.speed_mps:.1f} m/s")
    print(f"Duration     : {args.duration_sec:.1f} s")
    print(f"Camera       : {args.camera_hz:.1f} Hz")
    print(f"IMU / GT     : {args.imu_hz:.1f} Hz")
    print(f"Runtime clock: {runtime_contract['clock_speed']:.1f}")
    print(f"Spawn        : {args.spawn_altitude_m:.1f} m (NED Z={expected_spawn_z:.1f} m)")
    print(f"Altitude     : {args.min_altitude_m:.1f}~{args.max_altitude_m:.1f} m")
    print(f"Command band : {args.min_altitude_m + args.altitude_command_margin_m:.1f}~{args.max_altitude_m - args.altitude_command_margin_m:.1f} m")
    print("End behavior : hover only; no landing")
    print()

    sleep_until(start_monotonic, shared.stop_event)

    try:
        # reset() has already placed the vehicle at the configured airborne spawn.
        # Calling takeoffAsync() here would add an unnecessary climb and delay.
        print(
            f"[FLIGHT] Starting directly from the configured "
            f"{args.spawn_altitude_m:.1f} m spawn; takeoff skipped."
        )
        current_state = control.getMultirotorState(vehicle_name=args.vehicle)
        origin = current_state.kinematics_estimated.position
        altitude_reference_ned_z = float(origin.z_val)
        remaining_sec = max(5.0, end_monotonic - time.monotonic())
        path = generate_dynamic_path(
            origin,
            remaining_sec + 15.0,
            args.speed_mps,
            args.spawn_altitude_m,
            args.min_altitude_m,
            args.max_altitude_m,
            args.altitude_command_margin_m,
        )

        print(
            f"[FLIGHT] Starting continuous 3D path with {len(path)} points "
            f"at {args.speed_mps:.1f} m/s."
        )

        control.moveOnPathAsync(
            path,
            velocity=args.speed_mps,
            timeout_sec=args.duration_sec + 60.0,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0.0),
            lookahead=20.0,
            adaptive_lookahead=1,
            vehicle_name=args.vehicle,
        )

        while not shared.stop_event.is_set():
            remaining = end_monotonic - time.monotonic()
            if remaining <= 0:
                break
            elapsed = args.duration_sec - remaining
            if int(elapsed) % 10 == 0:
                # Avoid repeated prints within the same second.
                time.sleep(min(1.0, remaining))
            else:
                time.sleep(min(0.2, remaining))

    except Exception as exc:
        shared.add_error(f"flight: {type(exc).__name__}: {exc}")
        traceback.print_exc()

    finally:
        shared.stop_event.set()
        try:
            control.cancelLastTask(vehicle_name=args.vehicle)
        except Exception as exc:
            shared.add_error(f"cancelLastTask: {type(exc).__name__}: {exc}")
        try:
            print(f"[FLIGHT] {args.duration_sec:.1f} seconds complete. Entering hover; no landing.")
            control.hoverAsync(vehicle_name=args.vehicle).join()
        except Exception as exc:
            shared.add_error(f"hover: {type(exc).__name__}: {exc}")

    camera_thread.join(timeout=30.0)
    inertial_thread.join(timeout=30.0)

    expected_camera = int(round(args.duration_sec * args.camera_hz))
    expected_imu = int(round(args.duration_sec * args.imu_hz))

    with shared.lock:
        altitude_summary = local_altitude_stats(
            shared.gt_ned_z_values,
            altitude_reference_ned_z,
            args.spawn_altitude_m,
        )
        summary = {
            **config,
            "dataset_root": str(dataset_root),
            "camera_expected": expected_camera,
            "camera_written": shared.camera_written,
            "imu_expected": expected_imu,
            "imu_written": shared.imu_written,
            "gt_expected": expected_imu,
            "gt_written": shared.gt_written,
            "camera_mapping": mapping_stats(shared.camera_mapping_errors),
            "imu_mapping": mapping_stats(shared.imu_mapping_errors),
            "gt_mapping": mapping_stats(shared.gt_mapping_errors),
            "altitude_reference_ned_z_m": altitude_reference_ned_z,
            "local_altitude_m": altitude_summary,
            "errors": list(shared.errors),
            "camera_thread_alive_after_join": camera_thread.is_alive(),
            "inertial_thread_alive_after_join": inertial_thread.is_alive(),
            "finished_local": datetime.now().isoformat(),
        }

    validation = {
        "camera_count_pass": summary["camera_written"] == expected_camera,
        "imu_count_pass": summary["imu_written"] == expected_imu,
        "gt_count_pass": summary["gt_written"] == expected_imu,
        "runtime_clock_speed_pass": math.isclose(
            runtime_contract["clock_speed"], 1.0, abs_tol=1e-9
        ),
        "runtime_spawn_z_pass": math.isclose(
            runtime_contract["spawn_z_ned_m"], expected_spawn_z, abs_tol=1e-6
        ),
        "altitude_samples_pass": altitude_summary["count"] == summary["gt_written"],
        "altitude_range_pass": (
            altitude_summary["min_m"] is not None
            and altitude_summary["max_m"] is not None
            and altitude_summary["min_m"] >= args.min_altitude_m
            and altitude_summary["max_m"] <= args.max_altitude_m
        ),
        "no_errors_pass": not summary["errors"],
        "camera_files_match_csv": len(
            list((dataset_root / "mav0" / "cam0" / "data").glob("*.png"))
        ) == summary["camera_written"],
        "no_worker_left_running": not (
            summary["camera_thread_alive_after_join"]
            or summary["inertial_thread_alive_after_join"]
        ),
        "recording_completed": True,
        "drone_landed": False,
        "drone_hovering_requested": True,
    }
    validation["all_pass"] = all(
        value for key, value in validation.items() if key.endswith("_pass")
    ) and validation["camera_files_match_csv"] and validation["no_worker_left_running"]

    write_json(dataset_root / "run_summary.json", summary)
    write_json(dataset_root / "validation_report.json", validation)
    (dataset_root / "DRONE_LEFT_HOVERING.txt").write_text(
        "The 180-second run completed without landing.\n"
        "Drone1 was commanded to hover and remains armed under API control.\n"
        "Run 02_safe_recover.py when landing is desired.\n",
        encoding="utf-8",
    )

    if not camera_thread.is_alive() and not inertial_thread.is_alive():
        incomplete_marker.unlink(missing_ok=True)

    print()
    print("=== Dataset finalized ===")
    print(f"Root   : {dataset_root}")
    print(f"Camera : {summary['camera_written']} / {expected_camera}")
    print(f"IMU    : {summary['imu_written']} / {expected_imu}")
    print(f"GT     : {summary['gt_written']} / {expected_imu}")
    print(f"Valid  : {validation['all_pass']}")
    print("[STATE] Drone was not landed. Hover command was issued.")
    if altitude_summary["min_m"] is not None:
        print(
            f"Altitude: {altitude_summary['min_m']:.3f}~"
            f"{altitude_summary['max_m']:.3f} m"
        )
    else:
        print("Altitude: unavailable")
    print()
    return 0 if validation["all_pass"] else 2


if __name__ == "__main__":
    sys.exit(main())
