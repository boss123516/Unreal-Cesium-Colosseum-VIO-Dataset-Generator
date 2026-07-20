#!/usr/bin/env python3

import argparse
import bisect
import math
import statistics
import threading
import time
from dataclasses import dataclass, field
from typing import List

import airsim


@dataclass
class SensorSamples:
    timestamps_ns: List[int] = field(default_factory=list)
    gyro_x: List[float] = field(default_factory=list)
    gyro_y: List[float] = field(default_factory=list)
    gyro_z: List[float] = field(default_factory=list)
    accel_x: List[float] = field(default_factory=list)
    accel_y: List[float] = field(default_factory=list)
    accel_z: List[float] = field(default_factory=list)


def percentile(values, percent):
    if not values:
        return float("nan")

    ordered = sorted(values)
    index = min(
        len(ordered) - 1,
        max(0, math.ceil(percent / 100.0 * len(ordered)) - 1),
    )
    return ordered[index]


def summarize_timestamps(name, timestamps_ns, expected_hz):
    timestamps_ns = [int(v) for v in timestamps_ns if int(v) > 0]

    if len(timestamps_ns) < 2:
        print(f"[ERROR] {name}: timestamp sample 부족")
        return

    duplicate_count = sum(
        current == previous
        for previous, current in zip(timestamps_ns, timestamps_ns[1:])
    )
    backward_count = sum(
        current < previous
        for previous, current in zip(timestamps_ns, timestamps_ns[1:])
    )

    positive_dt = [
        (current - previous) * 1e-9
        for previous, current in zip(timestamps_ns, timestamps_ns[1:])
        if current > previous
    ]

    if not positive_dt:
        print(f"[ERROR] {name}: positive timestamp interval 없음")
        return

    expected_dt = 1.0 / expected_hz
    mean_dt = statistics.mean(positive_dt)
    mean_hz = 1.0 / mean_dt
    period_errors_ms = [
        abs(dt - expected_dt) * 1000.0
        for dt in positive_dt
    ]

    print()
    print(f"=== {name} timing ===")
    print(f"samples                    : {len(timestamps_ns)}")
    print(f"expected rate              : {expected_hz:.6f} Hz")
    print(f"measured rate              : {mean_hz:.6f} Hz")
    print(f"mean period                : {mean_dt * 1000.0:.6f} ms")
    print(
        "period std                : "
        f"{statistics.pstdev(positive_dt) * 1000.0:.6f} ms"
    )
    print(
        "absolute period error p95 : "
        f"{percentile(period_errors_ms, 95):.6f} ms"
    )
    print(
        "absolute period error max : "
        f"{max(period_errors_ms):.6f} ms"
    )
    print(f"duplicate timestamps       : {duplicate_count}")
    print(f"backward timestamps        : {backward_count}")


def nearest_timestamp_offsets_ms(camera_timestamps, imu_timestamps):
    imu_sorted = sorted(int(v) for v in imu_timestamps if int(v) > 0)
    offsets = []

    if not imu_sorted:
        return offsets

    for camera_timestamp in camera_timestamps:
        camera_timestamp = int(camera_timestamp)
        if camera_timestamp <= 0:
            continue

        index = bisect.bisect_left(imu_sorted, camera_timestamp)
        candidates = []

        if index < len(imu_sorted):
            candidates.append(imu_sorted[index])
        if index > 0:
            candidates.append(imu_sorted[index - 1])

        if candidates:
            nearest = min(
                candidates,
                key=lambda value: abs(value - camera_timestamp),
            )
            offsets.append(abs(nearest - camera_timestamp) * 1e-6)

    return offsets


def vector_stats(name, x_values, y_values, z_values):
    if not x_values:
        return

    norms = [
        math.sqrt(x * x + y * y + z * z)
        for x, y, z in zip(x_values, y_values, z_values)
    ]

    print()
    print(f"=== Static {name} sanity check ===")
    print(
        f"mean xyz : "
        f"[{statistics.mean(x_values):.9f}, "
        f"{statistics.mean(y_values):.9f}, "
        f"{statistics.mean(z_values):.9f}]"
    )
    print(
        f"std xyz  : "
        f"[{statistics.pstdev(x_values):.9f}, "
        f"{statistics.pstdev(y_values):.9f}, "
        f"{statistics.pstdev(z_values):.9f}]"
    )
    print(f"mean norm: {statistics.mean(norms):.9f}")
    print(f"std norm : {statistics.pstdev(norms):.9f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--camera", default="cam0")
    parser.add_argument("--imu", default="Imu")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-hz", type=float, default=10.0)
    parser.add_argument("--imu-hz", type=float, default=100.0)
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()

    client = airsim.MultirotorClient()
    client.confirmConnection()

    camera_info = client.simGetCameraInfo(
        args.camera,
        vehicle_name=args.vehicle,
    )

    response = client.simGetImages(
        [
            airsim.ImageRequest(
                args.camera,
                airsim.ImageType.Scene,
                False,
                False,
            )
        ],
        vehicle_name=args.vehicle,
    )[0]

    runtime_width = int(response.width)
    runtime_height = int(response.height)
    fov_x = float(camera_info.fov)

    fx = runtime_width / (
        2.0 * math.tan(math.radians(fov_x) / 2.0)
    )
    fy = fx
    cx = runtime_width / 2.0
    cy = runtime_height / 2.0
    fov_y = math.degrees(
        2.0 * math.atan(runtime_height / (2.0 * fy))
    )

    print()
    print("=== Camera model ===")
    print(f"runtime resolution : {runtime_width} x {runtime_height}")
    print(f"expected resolution: {args.width} x {args.height}")
    print(f"horizontal FOV     : {fov_x:.9f} deg")
    print(f"vertical FOV       : {fov_y:.9f} deg")
    print(f"fx                 : {fx:.9f}")
    print(f"fy                 : {fy:.9f}")
    print(f"cx                 : {cx:.9f}")
    print(f"cy                 : {cy:.9f}")
    print("K:")
    print(f"[{fx:.9f}, 0.000000000, {cx:.9f}]")
    print(f"[0.000000000, {fy:.9f}, {cy:.9f}]")
    print("[0.000000000, 0.000000000, 1.000000000]")
    print("Camera world pose:")
    print(camera_info.pose)

    try:
        distortion = client.simGetDistortionParams(
            args.camera,
            vehicle_name=args.vehicle,
        )
        print()
        print("=== Distortion parameters ===")
        print(distortion)
    except Exception as exc:
        print()
        print(
            "[WARN] simGetDistortionParams 호출 실패: "
            f"{type(exc).__name__}: {exc}"
        )

    print()
    print("=== Nadir-camera ground coverage assumption ===")
    print("Assumption: flat ground, camera optical axis points downward.")
    for altitude_m in (300.0, 400.0, 500.0):
        footprint_width = (
            2.0
            * altitude_m
            * math.tan(math.radians(fov_x) / 2.0)
        )
        footprint_height = (
            2.0
            * altitude_m
            * math.tan(math.radians(fov_y) / 2.0)
        )
        gsd_x = footprint_width / runtime_width
        gsd_y = footprint_height / runtime_height

        print(
            f"{altitude_m:6.1f} m: "
            f"footprint={footprint_width:.3f} x "
            f"{footprint_height:.3f} m, "
            f"GSD={gsd_x:.6f} x {gsd_y:.6f} m/px"
        )

    samples = SensorSamples()
    camera_timestamps = []
    errors = []
    stop_time = time.monotonic() + args.duration

    def collect_imu():
        try:
            imu_client = airsim.MultirotorClient()
            imu_client.confirmConnection()

            period = 1.0 / args.imu_hz
            next_deadline = time.monotonic()

            while time.monotonic() < stop_time:
                data = imu_client.getImuData(
                    imu_name=args.imu,
                    vehicle_name=args.vehicle,
                )

                samples.timestamps_ns.append(int(data.time_stamp))
                samples.gyro_x.append(float(data.angular_velocity.x_val))
                samples.gyro_y.append(float(data.angular_velocity.y_val))
                samples.gyro_z.append(float(data.angular_velocity.z_val))
                samples.accel_x.append(float(data.linear_acceleration.x_val))
                samples.accel_y.append(float(data.linear_acceleration.y_val))
                samples.accel_z.append(float(data.linear_acceleration.z_val))

                next_deadline += period
                sleep_seconds = next_deadline - time.monotonic()
                if sleep_seconds > 0:
                    time.sleep(sleep_seconds)
                else:
                    next_deadline = time.monotonic()

        except Exception as exc:
            errors.append(f"IMU collector: {type(exc).__name__}: {exc}")

    def collect_camera():
        try:
            camera_client = airsim.MultirotorClient()
            camera_client.confirmConnection()

            period = 1.0 / args.camera_hz
            next_deadline = time.monotonic()

            while time.monotonic() < stop_time:
                image = camera_client.simGetImages(
                    [
                        airsim.ImageRequest(
                            args.camera,
                            airsim.ImageType.Scene,
                            False,
                            False,
                        )
                    ],
                    vehicle_name=args.vehicle,
                )[0]

                camera_timestamps.append(int(image.time_stamp))

                next_deadline += period
                sleep_seconds = next_deadline - time.monotonic()
                if sleep_seconds > 0:
                    time.sleep(sleep_seconds)
                else:
                    next_deadline = time.monotonic()

        except Exception as exc:
            errors.append(
                f"Camera collector: {type(exc).__name__}: {exc}"
            )

    print()
    print(
        f"[INFO] {args.duration:.1f}초 동안 "
        f"Camera {args.camera_hz:.1f} Hz / "
        f"IMU {args.imu_hz:.1f} Hz 검사"
    )
    print("[INFO] IMU noise 검사는 기체가 정지한 상태에서 해석해야 합니다.")

    imu_thread = threading.Thread(target=collect_imu, daemon=True)
    camera_thread = threading.Thread(target=collect_camera, daemon=True)

    imu_thread.start()
    camera_thread.start()

    imu_thread.join()
    camera_thread.join()

    if errors:
        print()
        for error in errors:
            print(f"[ERROR] {error}")

    summarize_timestamps(
        "Camera",
        camera_timestamps,
        args.camera_hz,
    )
    summarize_timestamps(
        "IMU",
        samples.timestamps_ns,
        args.imu_hz,
    )

    offsets_ms = nearest_timestamp_offsets_ms(
        camera_timestamps,
        samples.timestamps_ns,
    )

    if offsets_ms:
        print()
        print("=== Camera–IMU timestamp association ===")
        print(f"mean nearest offset: {statistics.mean(offsets_ms):.6f} ms")
        print(f"p95 nearest offset : {percentile(offsets_ms, 95):.6f} ms")
        print(f"max nearest offset : {max(offsets_ms):.6f} ms")

    vector_stats(
        "gyroscope [rad/s]",
        samples.gyro_x,
        samples.gyro_y,
        samples.gyro_z,
    )
    vector_stats(
        "accelerometer [m/s^2]",
        samples.accel_x,
        samples.accel_y,
        samples.accel_z,
    )


if __name__ == "__main__":
    main()
