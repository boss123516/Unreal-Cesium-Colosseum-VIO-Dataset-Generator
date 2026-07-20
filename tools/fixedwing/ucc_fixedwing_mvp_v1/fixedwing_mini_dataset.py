#!/usr/bin/env python3
"""Record a synchronized camera / IMU / ground-truth fixed-wing mini dataset."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import csv
from io import BytesIO
import json
import math
from pathlib import Path
import signal
import statistics
import threading
import time
from typing import Any

import airsim
import numpy as np
from PIL import Image, ImageStat


NS_PER_SEC = 1_000_000_000


@dataclass
class CaptureState:
    errors: list[str] = field(default_factory=list)
    camera_timestamps: list[int] = field(default_factory=list)
    imu_timestamps: list[int] = field(default_factory=list)
    gt_timestamps: list[int] = field(default_factory=list)
    camera_jitter_ms: list[float] = field(default_factory=list)
    inertial_jitter_ms: list[float] = field(default_factory=list)
    camera_stddev: list[float] = field(default_factory=list)
    camera_white_ratio: list[float] = field(default_factory=list)
    camera_dimensions: set[tuple[int, int]] = field(default_factory=set)
    imu_accel_norm: list[float] = field(default_factory=list)
    gt_positions: list[tuple[float, float, float]] = field(default_factory=list)
    gt_horizontal_speed: list[float] = field(default_factory=list)
    gt_roll_deg: list[float] = field(default_factory=list)
    quaternion_norm_error: list[float] = field(default_factory=list)
    camera_written: int = 0
    imu_written: int = 0
    gt_written: int = 0
    blank_frames: int = 0
    stop_event: threading.Event = field(default_factory=threading.Event)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def add_error(self, message: str) -> None:
        with self.lock:
            self.errors.append(message)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--camera-hz", type=float, default=10.0)
    parser.add_argument("--imu-hz", type=float, default=100.0)
    parser.add_argument("--start-delay-sec", type=float, default=1.0)
    parser.add_argument("--camera-warmup-timeout-sec", type=float, default=30.0)
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--camera", default="cam0")
    parser.add_argument("--imu", default="Imu")
    return parser.parse_args()


def make_client() -> Any:
    client = airsim.MultirotorClient()
    client.confirmConnection()
    return client


def sleep_until(deadline: float, stop_event: threading.Event) -> bool:
    while not stop_event.is_set():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return True
        stop_event.wait(min(remaining, 0.005))
    return False


def percentile(values: list[float], percentage: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(
        len(ordered) - 1,
        max(0, math.ceil(len(ordered) * percentage / 100.0) - 1),
    )
    return ordered[index]


def numeric_stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"mean": None, "p95": None, "max": None}
    return {
        "mean": statistics.fmean(values),
        "p95": percentile(values, 95),
        "max": max(values),
    }


def timestamp_stats(values: list[int]) -> dict[str, Any]:
    duplicates = 0
    regressions = 0
    periods_ms: list[float] = []
    for previous, current in zip(values, values[1:]):
        if current == previous:
            duplicates += 1
        elif current < previous:
            regressions += 1
        else:
            periods_ms.append((current - previous) * 1.0e-6)
    return {
        "count": len(values),
        "duplicates": duplicates,
        "regressions": regressions,
        "period_ms": numeric_stats(periods_ms),
    }


def quaternion_roll_deg(orientation: Any) -> float:
    x = float(orientation.x_val)
    y = float(orientation.y_val)
    z = float(orientation.z_val)
    w = float(orientation.w_val)
    return math.degrees(
        math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    )


def camera_frame_quality(payload: bytes) -> dict[str, float | int]:
    image = Image.open(BytesIO(payload)).convert("RGB")
    image.load()
    pixels = np.asarray(image)
    return {
        "channel_stddev": statistics.fmean(ImageStat.Stat(image).stddev),
        "white_ratio": float(np.mean(np.all(pixels >= 250, axis=2))),
        "unique_colors": int(np.unique(pixels.reshape(-1, 3), axis=0).shape[0]),
    }


def warm_up_camera(
    client: Any,
    vehicle: str,
    camera: str,
    timeout_sec: float,
) -> dict[str, float | int]:
    """Wait for three consecutive rendered Cesium frames before recording."""
    deadline = time.monotonic() + timeout_sec
    consecutive_ready = 0
    attempts = 0
    latest: dict[str, float | int] = {}
    while time.monotonic() < deadline:
        responses = client.simGetImages(
            [airsim.ImageRequest(camera, airsim.ImageType.Scene, False, True)],
            vehicle_name=vehicle,
        )
        attempts += 1
        if responses and responses[0].image_data_uint8:
            response = responses[0]
            payload = bytes(response.image_data_uint8)
            latest = camera_frame_quality(payload)
            latest.update(
                {
                    "width": int(response.width),
                    "height": int(response.height),
                    "attempts": attempts,
                }
            )
            ready = (
                response.width == 640
                and response.height == 480
                and latest["channel_stddev"] >= 5.0
                and latest["white_ratio"] < 0.95
                and latest["unique_colors"] >= 10_000
            )
            consecutive_ready = consecutive_ready + 1 if ready else 0
            if consecutive_ready >= 3:
                latest["consecutive_ready_frames"] = consecutive_ready
                return latest
        time.sleep(0.25)

    raise RuntimeError(
        f"camera warm-up timed out after {timeout_sec:.1f}s: {latest}"
    )


def camera_worker(
    state: CaptureState,
    output: Path,
    start_monotonic: float,
    epoch_ns: int,
    duration_sec: float,
    hz: float,
    vehicle: str,
    camera: str,
) -> None:
    client = make_client()
    expected = int(round(duration_sec * hz))
    period_sec = 1.0 / hz
    period_ns = round(NS_PER_SEC / hz)
    image_dir = output / "mav0" / "cam0" / "data"

    with (output / "mav0" / "cam0" / "data.csv").open(
        "w", newline="", encoding="utf-8"
    ) as data_file, (output / "mav0" / "cam0" / "mapping.csv").open(
        "w", newline="", encoding="utf-8"
    ) as mapping_file:
        data_writer = csv.writer(data_file)
        mapping_writer = csv.writer(mapping_file)
        data_writer.writerow(["#timestamp [ns]", "filename"])
        mapping_writer.writerow(
            ["target_timestamp_ns", "source_timestamp_ns", "timestamp_error_ns"]
        )

        for index in range(expected):
            deadline = start_monotonic + index * period_sec
            if not sleep_until(deadline, state.stop_event):
                break
            state.camera_jitter_ms.append((time.monotonic() - deadline) * 1000.0)
            target_ns = epoch_ns + index * period_ns
            try:
                responses = client.simGetImages(
                    [airsim.ImageRequest(camera, airsim.ImageType.Scene, False, True)],
                    vehicle_name=vehicle,
                )
                if not responses:
                    raise RuntimeError("simGetImages returned no response")
                response = responses[0]
                payload = bytes(response.image_data_uint8)
                if response.width <= 0 or response.height <= 0 or not payload:
                    raise RuntimeError(
                        f"invalid image {response.width}x{response.height}, bytes={len(payload)}"
                    )

                image = Image.open(BytesIO(payload)).convert("RGB")
                image.load()
                channel_stddev = statistics.fmean(ImageStat.Stat(image).stddev)
                pixels = np.asarray(image)
                white_ratio = float(np.mean(np.all(pixels >= 250, axis=2)))
                is_blank = channel_stddev < 1.0 or white_ratio >= 0.99

                source_ns = int(response.time_stamp)
                filename = f"{target_ns}.png"
                (image_dir / filename).write_bytes(payload)
                data_writer.writerow([target_ns, filename])
                mapping_writer.writerow([target_ns, source_ns, source_ns - target_ns])

                state.camera_timestamps.append(source_ns)
                state.camera_stddev.append(channel_stddev)
                state.camera_white_ratio.append(white_ratio)
                state.camera_dimensions.add((response.width, response.height))
                state.blank_frames += int(is_blank)
                state.camera_written += 1
            except Exception as exc:
                state.add_error(f"camera[{index}]: {type(exc).__name__}: {exc}")

            if index % max(1, int(hz)) == 0:
                data_file.flush()
                mapping_file.flush()


def inertial_worker(
    state: CaptureState,
    output: Path,
    start_monotonic: float,
    epoch_ns: int,
    duration_sec: float,
    hz: float,
    vehicle: str,
    imu_name: str,
) -> None:
    client = make_client()
    expected = int(round(duration_sec * hz))
    period_sec = 1.0 / hz
    period_ns = round(NS_PER_SEC / hz)

    imu_path = output / "mav0" / "imu0" / "data.csv"
    imu_map_path = output / "mav0" / "imu0" / "mapping.csv"
    gt_path = output / "mav0" / "state_groundtruth_estimate0" / "data.csv"
    gt_map_path = output / "mav0" / "state_groundtruth_estimate0" / "mapping.csv"

    with imu_path.open("w", newline="", encoding="utf-8") as imu_file, \
        imu_map_path.open("w", newline="", encoding="utf-8") as imu_map_file, \
        gt_path.open("w", newline="", encoding="utf-8") as gt_file, \
        gt_map_path.open("w", newline="", encoding="utf-8") as gt_map_file:
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
                "a_RS_R_x [m s^-2]",
                "a_RS_R_y [m s^-2]",
                "a_RS_R_z [m s^-2]",
            ]
        )
        gt_map_writer.writerow(
            ["target_timestamp_ns", "source_timestamp_ns", "timestamp_error_ns"]
        )

        for index in range(expected):
            deadline = start_monotonic + index * period_sec
            if not sleep_until(deadline, state.stop_event):
                break
            state.inertial_jitter_ms.append((time.monotonic() - deadline) * 1000.0)
            target_ns = epoch_ns + index * period_ns

            try:
                imu = client.getImuData(imu_name=imu_name, vehicle_name=vehicle)
                source_ns = int(imu.time_stamp)
                angular = imu.angular_velocity
                acceleration = imu.linear_acceleration
                imu_writer.writerow(
                    [
                        target_ns,
                        angular.x_val,
                        angular.y_val,
                        angular.z_val,
                        acceleration.x_val,
                        acceleration.y_val,
                        acceleration.z_val,
                    ]
                )
                imu_map_writer.writerow([target_ns, source_ns, source_ns - target_ns])
                state.imu_timestamps.append(source_ns)
                state.imu_accel_norm.append(
                    math.sqrt(
                        acceleration.x_val**2
                        + acceleration.y_val**2
                        + acceleration.z_val**2
                    )
                )
                state.imu_written += 1
            except Exception as exc:
                state.add_error(f"imu[{index}]: {type(exc).__name__}: {exc}")

            try:
                multirotor_state = client.getMultirotorState(vehicle_name=vehicle)
                source_ns = int(multirotor_state.timestamp)
                kinematics = multirotor_state.kinematics_estimated
                position = kinematics.position
                orientation = kinematics.orientation
                velocity = kinematics.linear_velocity
                angular = kinematics.angular_velocity
                acceleration = kinematics.linear_acceleration
                gt_writer.writerow(
                    [
                        target_ns,
                        position.x_val,
                        position.y_val,
                        position.z_val,
                        orientation.w_val,
                        orientation.x_val,
                        orientation.y_val,
                        orientation.z_val,
                        velocity.x_val,
                        velocity.y_val,
                        velocity.z_val,
                        angular.x_val,
                        angular.y_val,
                        angular.z_val,
                        acceleration.x_val,
                        acceleration.y_val,
                        acceleration.z_val,
                    ]
                )
                gt_map_writer.writerow([target_ns, source_ns, source_ns - target_ns])
                quaternion_norm = math.sqrt(
                    orientation.w_val**2
                    + orientation.x_val**2
                    + orientation.y_val**2
                    + orientation.z_val**2
                )
                state.gt_timestamps.append(source_ns)
                state.gt_positions.append(
                    (position.x_val, position.y_val, position.z_val)
                )
                state.gt_horizontal_speed.append(
                    math.hypot(velocity.x_val, velocity.y_val)
                )
                state.gt_roll_deg.append(quaternion_roll_deg(orientation))
                state.quaternion_norm_error.append(abs(quaternion_norm - 1.0))
                state.gt_written += 1
            except Exception as exc:
                state.add_error(f"gt[{index}]: {type(exc).__name__}: {exc}")

            if index % max(1, int(hz)) == 0:
                imu_file.flush()
                imu_map_file.flush()
                gt_file.flush()
                gt_map_file.flush()


def motion_stats(state: CaptureState) -> dict[str, float | None]:
    if not state.gt_positions:
        return {
            "max_displacement_m": None,
            "max_horizontal_speed_mps": None,
            "max_relative_altitude_m": None,
            "roll_span_deg": None,
            "max_abs_roll_deg": None,
        }
    x0, y0, z0 = state.gt_positions[0]
    displacements = [
        math.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
        for x, y, z in state.gt_positions
    ]
    relative_altitudes = [z0 - z for _, _, z in state.gt_positions]
    return {
        "max_displacement_m": max(displacements),
        "max_horizontal_speed_mps": max(state.gt_horizontal_speed),
        "max_relative_altitude_m": max(relative_altitudes),
        "roll_span_deg": max(state.gt_roll_deg) - min(state.gt_roll_deg),
        "max_abs_roll_deg": max(abs(value) for value in state.gt_roll_deg),
    }


def main() -> int:
    args = parse_args()
    if min(args.duration_sec, args.camera_hz, args.imu_hz) <= 0.0:
        raise SystemExit("[ERROR] duration and rates must be positive")
    output = args.output.expanduser().resolve()
    if output.exists():
        raise SystemExit(f"[ERROR] output already exists: {output}")

    control = make_client()
    settings = json.loads(control.getSettingsString())
    if settings.get("PhysicsEngineName") != "ExternalPhysicsEngine":
        raise SystemExit("[ERROR] UCC is not using ExternalPhysicsEngine")
    if args.vehicle not in control.listVehicles():
        raise SystemExit(f"[ERROR] vehicle not found: {args.vehicle}")

    camera_warmup = warm_up_camera(
        control,
        args.vehicle,
        args.camera,
        args.camera_warmup_timeout_sec,
    )
    print(f"[CAMERA_READY] {json.dumps(camera_warmup, sort_keys=True)}")

    for relative in (
        "mav0/cam0/data",
        "mav0/imu0",
        "mav0/state_groundtruth_estimate0",
    ):
        (output / relative).mkdir(parents=True, exist_ok=False)
    incomplete = output / ".recording_incomplete"
    incomplete.write_text("recording in progress\n", encoding="utf-8")

    epoch_ns = time.time_ns() + round(args.start_delay_sec * NS_PER_SEC)
    start_monotonic = time.monotonic() + args.start_delay_sec
    run_config = {
        "vehicle": args.vehicle,
        "camera": args.camera,
        "imu": args.imu,
        "duration_sec": args.duration_sec,
        "camera_hz": args.camera_hz,
        "imu_hz": args.imu_hz,
        "gt_hz": args.imu_hz,
        "epoch_timestamp_ns": epoch_ns,
        "physics_engine": settings.get("PhysicsEngineName"),
        "motion_source": "PX4 gz_rc_cessna via Gazebo direct link kinematics",
        "acceleration_source": "Gazebo WorldLinearAcceleration component",
        "frame_contract": "Gazebo ENU/FLU to AirSim NED/FRD",
        "camera_warmup": camera_warmup,
    }
    (output / "run_config.json").write_text(
        json.dumps(run_config, indent=2) + "\n", encoding="utf-8"
    )

    state = CaptureState()

    def stop_handler(_signum, _frame) -> None:
        state.stop_event.set()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    camera_thread = threading.Thread(
        target=camera_worker,
        name="fixedwing-camera-10hz",
        kwargs={
            "state": state,
            "output": output,
            "start_monotonic": start_monotonic,
            "epoch_ns": epoch_ns,
            "duration_sec": args.duration_sec,
            "hz": args.camera_hz,
            "vehicle": args.vehicle,
            "camera": args.camera,
        },
    )
    inertial_thread = threading.Thread(
        target=inertial_worker,
        name="fixedwing-imu-gt-100hz",
        kwargs={
            "state": state,
            "output": output,
            "start_monotonic": start_monotonic,
            "epoch_ns": epoch_ns,
            "duration_sec": args.duration_sec,
            "hz": args.imu_hz,
            "vehicle": args.vehicle,
            "imu_name": args.imu,
        },
    )
    wall_start = time.monotonic()
    camera_thread.start()
    inertial_thread.start()
    camera_thread.join()
    inertial_thread.join()
    elapsed = time.monotonic() - wall_start

    expected_camera = int(round(args.duration_sec * args.camera_hz))
    expected_inertial = int(round(args.duration_sec * args.imu_hz))
    camera_time = timestamp_stats(state.camera_timestamps)
    imu_time = timestamp_stats(state.imu_timestamps)
    gt_time = timestamp_stats(state.gt_timestamps)
    dimensions = sorted([list(value) for value in state.camera_dimensions])
    accepted_dimensions = dimensions == [[640, 480]]
    all_pass = (
        not state.errors
        and state.camera_written == expected_camera
        and state.imu_written == expected_inertial
        and state.gt_written == expected_inertial
        and state.blank_frames == 0
        and accepted_dimensions
        and camera_time["duplicates"] == 0
        and camera_time["regressions"] == 0
        and imu_time["regressions"] == 0
        and gt_time["regressions"] == 0
        and max(state.quaternion_norm_error, default=math.inf) < 1.0e-4
    )
    report = {
        "all_pass": all_pass,
        "elapsed_wall_sec": elapsed,
        "expected": {
            "camera": expected_camera,
            "imu": expected_inertial,
            "gt": expected_inertial,
        },
        "written": {
            "camera": state.camera_written,
            "imu": state.imu_written,
            "gt": state.gt_written,
        },
        "effective_rate_hz": {
            "camera": state.camera_written / args.duration_sec,
            "imu": state.imu_written / args.duration_sec,
            "gt": state.gt_written / args.duration_sec,
        },
        "source_timestamps": {
            "camera": camera_time,
            "imu": imu_time,
            "gt": gt_time,
        },
        "schedule_jitter_ms": {
            "camera": numeric_stats(state.camera_jitter_ms),
            "imu_gt": numeric_stats(state.inertial_jitter_ms),
        },
        "camera_quality": {
            "dimensions": dimensions,
            "blank_frames": state.blank_frames,
            "mean_channel_stddev": statistics.fmean(state.camera_stddev)
            if state.camera_stddev
            else None,
            "max_white_ratio": max(state.camera_white_ratio, default=None),
        },
        "imu_acceleration_norm_mps2": numeric_stats(state.imu_accel_norm),
        "ground_truth": {
            **motion_stats(state),
            "max_quaternion_norm_error": max(
                state.quaternion_norm_error, default=None
            ),
        },
        "errors": state.errors,
    }
    (output / "timing_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    if all_pass:
        incomplete.unlink()
    print(json.dumps(report, indent=2))
    print(f"[DATASET] {output}")
    return 0 if all_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
