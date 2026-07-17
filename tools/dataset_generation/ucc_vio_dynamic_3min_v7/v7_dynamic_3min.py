#!/usr/bin/env python3
"""
30 m/s, 180-second dynamic 3D AirSim flight and synchronized dataset recorder.

Behavior:
- Recording starts immediately and lasts exactly `duration_sec` in wall time.
- Camera: exact 10 Hz target grid, compressed Scene PNG.
- IMU + GT: exact 100 Hz target grid.
- Flight: takeoff, then continuous 3D slalom / broad turns / climb-descent.
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


def generate_dynamic_path(
    origin: Any,
    duration_sec: float,
    speed_mps: float,
    dt: float = 0.5,
) -> list[Any]:
    """
    Integrates a 30 m/s velocity field into a continuous 3D path.

    NED convention:
    - +x north/forward local
    - +y east/right local
    - +z down
    """
    point_count = max(2, int(math.ceil(duration_sec / dt)) + 1)
    x = float(origin.x_val)
    y = float(origin.y_val)
    z = float(origin.z_val)
    path: list[Any] = []

    for index in range(point_count):
        t = index * dt

        # Baseline fast slalom.
        heading = (
            0.82 * math.sin(2.0 * math.pi * t / 27.0)
            + 0.30 * math.sin(2.0 * math.pi * t / 8.5)
        )

        # One broad left 360-degree turn.
        if 42.0 <= t <= 76.0:
            heading += 2.0 * math.pi * smoothstep((t - 42.0) / 34.0)
        elif t > 76.0:
            heading += 2.0 * math.pi

        # One broad right 360-degree turn.
        if 104.0 <= t <= 140.0:
            heading -= 2.0 * math.pi * smoothstep((t - 104.0) / 36.0)
        elif t > 140.0:
            heading -= 2.0 * math.pi

        # First 30 seconds: gain about 120 m while still moving fast.
        if t < 30.0:
            vertical_speed = -4.0
        else:
            vertical_speed = (
                4.8 * math.sin(2.0 * math.pi * (t - 30.0) / 22.0)
                + 1.5 * math.sin(2.0 * math.pi * (t - 30.0) / 7.5)
            )
            vertical_speed = max(-6.0, min(6.0, vertical_speed))

        horizontal_speed = math.sqrt(
            max(0.0, speed_mps * speed_mps - vertical_speed * vertical_speed)
        )

        x += horizontal_speed * math.cos(heading) * dt
        y += horizontal_speed * math.sin(heading) * dt
        z += vertical_speed * dt
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
    control = make_client()
    control.enableApiControl(True, vehicle_name=args.vehicle)
    control.armDisarm(True, vehicle_name=args.vehicle)

    initial_state = control.getMultirotorState(vehicle_name=args.vehicle)
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
    print("End behavior : hover only; no landing")
    print()

    sleep_until(start_monotonic, shared.stop_event)

    try:
        # Takeoff is part of the 180-second recording window.
        try:
            print("[FLIGHT] Taking off.")
            control.takeoffAsync(timeout_sec=8, vehicle_name=args.vehicle).join()
        except Exception as exc:
            shared.add_error(f"takeoff: {type(exc).__name__}: {exc}")
            print(f"[WARN] takeoff returned an error; continuing: {exc}")

        current_state = control.getMultirotorState(vehicle_name=args.vehicle)
        origin = current_state.kinematics_estimated.position
        remaining_sec = max(5.0, end_monotonic - time.monotonic())
        path = generate_dynamic_path(origin, remaining_sec + 15.0, args.speed_mps)

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
            print("[FLIGHT] 180 seconds complete. Entering hover; no landing.")
            control.hoverAsync(vehicle_name=args.vehicle).join()
        except Exception as exc:
            shared.add_error(f"hover: {type(exc).__name__}: {exc}")

    camera_thread.join(timeout=30.0)
    inertial_thread.join(timeout=30.0)

    expected_camera = int(round(args.duration_sec * args.camera_hz))
    expected_imu = int(round(args.duration_sec * args.imu_hz))

    with shared.lock:
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
            "errors": list(shared.errors),
            "camera_thread_alive_after_join": camera_thread.is_alive(),
            "inertial_thread_alive_after_join": inertial_thread.is_alive(),
            "finished_local": datetime.now().isoformat(),
        }

    validation = {
        "camera_count_pass": summary["camera_written"] == expected_camera,
        "imu_count_pass": summary["imu_written"] == expected_imu,
        "gt_count_pass": summary["gt_written"] == expected_imu,
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
    print()
    return 0 if validation["all_pass"] else 2


if __name__ == "__main__":
    sys.exit(main())
