#!/usr/bin/env python3
"""Publish AirSim Drone1/cam0 to ROS 2 for live rqt_image_view display."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any

import airsim
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--camera", default="cam0")
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--fov-deg", type=float, default=90.0)
    parser.add_argument("--image-topic", default="/ucc/cam0/image_raw")
    parser.add_argument("--camera-info-topic", default="/ucc/cam0/camera_info")
    parser.add_argument("--frame-id", default="cam0_optical")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Exit after N frames; zero runs until interrupted.",
    )
    return parser.parse_args()


def camera_matrix(width: int, height: int, fov_deg: float) -> list[float]:
    if width <= 0 or height <= 0:
        raise ValueError("camera dimensions must be positive")
    if not 1.0 <= fov_deg < 179.0:
        raise ValueError("FOV must be within [1, 179) degrees")
    focal = width / (2.0 * math.tan(math.radians(fov_deg) * 0.5))
    cx = (width - 1.0) * 0.5
    cy = (height - 1.0) * 0.5
    return [
        focal,
        0.0,
        cx,
        0.0,
        focal,
        cy,
        0.0,
        0.0,
        1.0,
    ]


class AirSimCam0Publisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ucc_airsim_cam0_publisher")
        self.args = args
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_publisher = self.create_publisher(
            Image,
            args.image_topic,
            qos,
        )
        self.info_publisher = self.create_publisher(
            CameraInfo,
            args.camera_info_topic,
            qos,
        )
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        if args.vehicle not in self.client.listVehicles():
            raise RuntimeError(f"AirSim vehicle not found: {args.vehicle}")

        self.request = airsim.ImageRequest(
            args.camera,
            airsim.ImageType.Scene,
            pixels_as_float=False,
            compress=True,
        )
        self.frame_count = 0
        self.error_count = 0
        self.started = time.monotonic()
        self.last_report = self.started
        self.timer = self.create_timer(1.0 / args.hz, self.publish_frame)
        self.get_logger().info(
            f"publishing {args.vehicle}/{args.camera} at {args.hz:.1f} Hz "
            f"to {args.image_topic}"
        )

    def publish_frame(self) -> None:
        try:
            responses = self.client.simGetImages(
                [self.request],
                vehicle_name=self.args.vehicle,
            )
            if not responses:
                raise RuntimeError("simGetImages returned no responses")
            response = responses[0]
            payload = np.frombuffer(
                bytes(response.image_data_uint8),
                dtype=np.uint8,
            )
            frame = cv2.imdecode(payload, cv2.IMREAD_COLOR)
            if frame is None or frame.size == 0:
                raise RuntimeError("cam0 PNG decode failed")

            height, width = frame.shape[:2]
            stamp = self.get_clock().now().to_msg()
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = self.args.frame_id
            image.height = height
            image.width = width
            image.encoding = "bgr8"
            image.is_bigendian = False
            image.step = width * 3
            image.data = frame.tobytes()

            info = CameraInfo()
            info.header = image.header
            info.height = height
            info.width = width
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            info.k = camera_matrix(width, height, self.args.fov_deg)
            info.r = [
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ]
            info.p = [
                info.k[0],
                0.0,
                info.k[2],
                0.0,
                0.0,
                info.k[4],
                info.k[5],
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]
            self.image_publisher.publish(image)
            self.info_publisher.publish(info)
            self.frame_count += 1

            now = time.monotonic()
            if now - self.last_report >= 5.0:
                elapsed = max(now - self.started, 1.0e-9)
                self.get_logger().info(
                    f"frames={self.frame_count} "
                    f"average_rate={self.frame_count / elapsed:.2f} Hz "
                    f"errors={self.error_count}"
                )
                self.last_report = now

            if (
                self.args.max_frames > 0
                and self.frame_count >= self.args.max_frames
            ):
                rclpy.shutdown()
        except Exception as exc:
            self.error_count += 1
            if self.error_count <= 3 or self.error_count % 25 == 0:
                self.get_logger().error(
                    f"cam0 acquisition failed: {type(exc).__name__}: {exc}"
                )


def main() -> int:
    args = parse_args()
    if not 0.5 <= args.hz <= 30.0:
        raise SystemExit("[ERROR] --hz must be within 0.5~30")
    if args.max_frames < 0:
        raise SystemExit("[ERROR] --max-frames must be non-negative")

    rclpy.init()
    node: AirSimCam0Publisher | None = None
    try:
        node = AirSimCam0Publisher(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        print(f"[ERROR] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.get_logger().info(
                f"stopped after publishing {node.frame_count} frames "
                f"with {node.error_count} acquisition errors"
            )
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
