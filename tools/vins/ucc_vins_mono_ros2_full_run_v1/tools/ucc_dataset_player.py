#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import heapq
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, Imu


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    wx: float
    wy: float
    wz: float
    ax: float
    ay: float
    az: float


@dataclass(frozen=True)
class CameraSample:
    timestamp_ns: int
    image_path: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--rate-scale", type=float, default=1.0)
    parser.add_argument("--startup-wait-sec", type=float, default=3.0)
    parser.add_argument("--subscriber-timeout-sec", type=float, default=20.0)
    return parser.parse_args()


def normalize_header(value: str) -> str:
    return "".join(character for character in value.lower() if character.isalnum())


def resolve_dataset_file(root: Path, relative: str) -> Path:
    direct = root / relative
    if direct.exists():
        return direct

    target_name = Path(relative).name
    for match in root.rglob(target_name):
        if str(match).replace("\\", "/").endswith(relative):
            return match

    raise FileNotFoundError(f"required dataset path not found: {relative}")


def read_camera_samples(dataset_root: Path) -> list[CameraSample]:
    csv_path = resolve_dataset_file(dataset_root, "mav0/cam0/data.csv")
    image_dir = csv_path.parent / "data"

    samples: list[CameraSample] = []

    with csv_path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.reader(file)

        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue

            if len(row) < 2:
                raise ValueError(f"invalid camera CSV row: {row}")

            timestamp_ns = int(row[0].strip())
            filename = row[1].strip()
            image_path = image_dir / filename

            if not image_path.is_file():
                raise FileNotFoundError(f"camera image missing: {image_path}")

            samples.append(CameraSample(timestamp_ns, image_path))

    if not samples:
        raise ValueError(f"no camera samples in {csv_path}")

    validate_monotonic(
        [sample.timestamp_ns for sample in samples],
        "camera",
    )
    return samples


def find_column(
    normalized: dict[str, str],
    candidates: tuple[str, ...],
) -> str:
    for candidate in candidates:
        key = normalize_header(candidate)
        if key in normalized:
            return normalized[key]

    raise KeyError(
        "missing CSV column; expected one of: "
        + ", ".join(candidates)
    )


def read_imu_samples(dataset_root: Path) -> list[ImuSample]:
    csv_path = resolve_dataset_file(dataset_root, "mav0/imu0/data.csv")

    with csv_path.open("r", encoding="utf-8-sig", newline="") as file:
        first_line = file.readline()
        if not first_line:
            raise ValueError(f"empty IMU CSV: {csv_path}")

        headers = next(csv.reader([first_line]))
        normalized = {
            normalize_header(header.lstrip("#").strip()): header
            for header in headers
        }

        timestamp_column = find_column(
            normalized,
            ("timestamp [ns]", "timestamp_ns", "timestamp"),
        )
        wx_column = find_column(
            normalized,
            ("w_RS_S_x [rad s^-1]", "angular_velocity_x", "gyro_x", "wx"),
        )
        wy_column = find_column(
            normalized,
            ("w_RS_S_y [rad s^-1]", "angular_velocity_y", "gyro_y", "wy"),
        )
        wz_column = find_column(
            normalized,
            ("w_RS_S_z [rad s^-1]", "angular_velocity_z", "gyro_z", "wz"),
        )
        ax_column = find_column(
            normalized,
            ("a_RS_S_x [m s^-2]", "linear_acceleration_x", "accel_x", "ax"),
        )
        ay_column = find_column(
            normalized,
            ("a_RS_S_y [m s^-2]", "linear_acceleration_y", "accel_y", "ay"),
        )
        az_column = find_column(
            normalized,
            ("a_RS_S_z [m s^-2]", "linear_acceleration_z", "accel_z", "az"),
        )

        file.seek(0)
        reader = csv.DictReader(file)
        samples: list[ImuSample] = []

        for row in reader:
            if not row:
                continue

            sample = ImuSample(
                timestamp_ns=int(row[timestamp_column].strip()),
                wx=float(row[wx_column]),
                wy=float(row[wy_column]),
                wz=float(row[wz_column]),
                ax=float(row[ax_column]),
                ay=float(row[ay_column]),
                az=float(row[az_column]),
            )

            values = (
                sample.wx,
                sample.wy,
                sample.wz,
                sample.ax,
                sample.ay,
                sample.az,
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"non-finite IMU sample: {sample}")

            samples.append(sample)

    if not samples:
        raise ValueError(f"no IMU samples in {csv_path}")

    validate_monotonic(
        [sample.timestamp_ns for sample in samples],
        "IMU",
    )
    return samples


def validate_monotonic(timestamps: list[int], name: str) -> None:
    for previous, current in zip(timestamps, timestamps[1:]):
        if current <= previous:
            raise ValueError(
                f"{name} timestamps are not strictly increasing: "
                f"{previous} -> {current}"
            )


class UccDatasetPlayer(Node):
    def __init__(self) -> None:
        super().__init__("ucc_dataset_player")

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2000,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.image_publisher = self.create_publisher(
            Image,
            "/cam0/image_raw",
            qos,
        )
        self.imu_publisher = self.create_publisher(
            Imu,
            "/imu0",
            qos,
        )

    def wait_for_subscribers(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            image_count = self.image_publisher.get_subscription_count()
            imu_count = self.imu_publisher.get_subscription_count()

            if image_count >= 1 and imu_count >= 1:
                self.get_logger().info(
                    f"Subscribers ready: image={image_count}, imu={imu_count}"
                )
                return

        raise TimeoutError(
            "VINS subscribers were not detected. "
            "Check that ucc_vins.launch.py is running."
        )

    def publish_imu(self, sample: ImuSample) -> None:
        message = Imu()
        message.header.stamp.sec = sample.timestamp_ns // 1_000_000_000
        message.header.stamp.nanosec = sample.timestamp_ns % 1_000_000_000
        message.header.frame_id = "imu0"

        message.orientation_covariance[0] = -1.0

        message.angular_velocity.x = sample.wx
        message.angular_velocity.y = sample.wy
        message.angular_velocity.z = sample.wz

        message.linear_acceleration.x = sample.ax
        message.linear_acceleration.y = sample.ay
        message.linear_acceleration.z = sample.az

        self.imu_publisher.publish(message)

    def publish_camera(self, sample: CameraSample) -> None:
        image = cv2.imread(str(sample.image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"failed to read image: {sample.image_path}")

        height, width = image.shape

        message = Image()
        message.header.stamp.sec = sample.timestamp_ns // 1_000_000_000
        message.header.stamp.nanosec = sample.timestamp_ns % 1_000_000_000
        message.header.frame_id = "cam0"

        message.height = height
        message.width = width
        message.encoding = "mono8"
        message.is_bigendian = False
        message.step = width
        message.data = image.tobytes()

        self.image_publisher.publish(message)


def merged_events(
    imu_samples: list[ImuSample],
    camera_samples: list[CameraSample],
):
    # IMU is ordered before camera when timestamps are identical.
    events = [
        (sample.timestamp_ns, 0, sample)
        for sample in imu_samples
    ]
    events.extend(
        (sample.timestamp_ns, 1, sample)
        for sample in camera_samples
    )
    events.sort(key=lambda event: (event[0], event[1]))
    return events


def main() -> int:
    args = parse_args()

    if args.rate_scale <= 0:
        raise SystemExit("[ERROR] --rate-scale must be positive")

    dataset_root = args.dataset_root.expanduser().resolve()
    camera_samples = read_camera_samples(dataset_root)
    imu_samples = read_imu_samples(dataset_root)
    events = merged_events(imu_samples, camera_samples)

    first_timestamp_ns = events[0][0]
    last_timestamp_ns = events[-1][0]
    duration_sec = (last_timestamp_ns - first_timestamp_ns) / 1e9

    camera_periods = [
        (current.timestamp_ns - previous.timestamp_ns) / 1e9
        for previous, current in zip(camera_samples, camera_samples[1:])
    ]
    imu_periods = [
        (current.timestamp_ns - previous.timestamp_ns) / 1e9
        for previous, current in zip(imu_samples, imu_samples[1:])
    ]

    print("=== UCC dataset player ===")
    print(f"Dataset       : {dataset_root}")
    print(f"Camera count  : {len(camera_samples)}")
    print(f"IMU count     : {len(imu_samples)}")
    print(f"Dataset time  : {duration_sec:.3f} s")
    if camera_periods:
        print(f"Camera mean Hz: {1.0 / (sum(camera_periods) / len(camera_periods)):.3f}")
    if imu_periods:
        print(f"IMU mean Hz   : {1.0 / (sum(imu_periods) / len(imu_periods)):.3f}")
    print(f"Playback scale: {args.rate_scale:.3f}x")

    rclpy.init()
    node = UccDatasetPlayer()

    try:
        node.wait_for_subscribers(args.subscriber_timeout_sec)
        time.sleep(max(0.0, args.startup_wait_sec))

        wall_start = time.monotonic()
        camera_published = 0
        imu_published = 0
        last_log_second = -1

        for timestamp_ns, event_type, sample in events:
            if not rclpy.ok():
                break

            target_elapsed = (
                (timestamp_ns - first_timestamp_ns)
                / 1_000_000_000
                / args.rate_scale
            )
            deadline = wall_start + target_elapsed

            while rclpy.ok():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                rclpy.spin_once(node, timeout_sec=min(0.01, remaining))

            if event_type == 0:
                node.publish_imu(sample)
                imu_published += 1
            else:
                node.publish_camera(sample)
                camera_published += 1

            rclpy.spin_once(node, timeout_sec=0.0)

            elapsed_second = int(target_elapsed)
            if elapsed_second != last_log_second and elapsed_second % 5 == 0:
                last_log_second = elapsed_second
                node.get_logger().info(
                    f"t={target_elapsed:.1f}s "
                    f"camera={camera_published}/{len(camera_samples)} "
                    f"imu={imu_published}/{len(imu_samples)}"
                )

        node.get_logger().info(
            f"Playback complete: camera={camera_published}, imu={imu_published}"
        )

        final_deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < final_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().warning("Playback interrupted")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
