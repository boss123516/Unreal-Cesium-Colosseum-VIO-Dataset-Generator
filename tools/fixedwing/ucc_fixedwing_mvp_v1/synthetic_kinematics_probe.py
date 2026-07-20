#!/usr/bin/env python3
"""Runtime gate for simSetKinematics -> AirSim IMU propagation."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Callable

import airsim


@dataclass
class ImuSample:
    timestamp_ns: int
    gyro: tuple[float, float, float]
    accel: tuple[float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--imu", default="Imu")
    parser.add_argument("--duration-sec", type=float, default=0.8)
    parser.add_argument("--injection-hz", type=float, default=100.0)
    parser.add_argument("--gyro-test-radps", type=float, default=0.1)
    parser.add_argument("--accel-test-mps2", type=float, default=1.0)
    parser.add_argument("--gyro-tolerance-radps", type=float, default=0.02)
    parser.add_argument("--accel-tolerance-mps2", type=float, default=0.15)
    parser.add_argument("--gravity-tolerance-mps2", type=float, default=0.35)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def vector_tuple(vector) -> tuple[float, float, float]:
    return (float(vector.x_val), float(vector.y_val), float(vector.z_val))


def make_state(
    position=(0.0, 0.0, 0.0),
    orientation=(0.0, 0.0, 0.0, 1.0),
    linear_velocity=(0.0, 0.0, 0.0),
    angular_velocity=(0.0, 0.0, 0.0),
    linear_acceleration=(0.0, 0.0, 0.0),
    angular_acceleration=(0.0, 0.0, 0.0),
):
    state = airsim.KinematicsState()
    state.position = airsim.Vector3r(*position)
    state.orientation = airsim.Quaternionr(*orientation)
    state.linear_velocity = airsim.Vector3r(*linear_velocity)
    state.angular_velocity = airsim.Vector3r(*angular_velocity)
    state.linear_acceleration = airsim.Vector3r(*linear_acceleration)
    state.angular_acceleration = airsim.Vector3r(*angular_acceleration)
    return state


def copy_state(state):
    return make_state(
        vector_tuple(state.position),
        (
            float(state.orientation.x_val),
            float(state.orientation.y_val),
            float(state.orientation.z_val),
            float(state.orientation.w_val),
        ),
        vector_tuple(state.linear_velocity),
        vector_tuple(state.angular_velocity),
        vector_tuple(state.linear_acceleration),
        vector_tuple(state.angular_acceleration),
    )


def hold_and_sample(client, state, args) -> list[ImuSample]:
    period = 1.0 / args.injection_hz
    deadline = time.monotonic() + args.duration_sec
    next_tick = time.monotonic()
    samples: list[ImuSample] = []
    last_timestamp = -1

    while time.monotonic() < deadline:
        client.simSetKinematics(state, True, args.vehicle)
        imu = client.getImuData(args.imu, args.vehicle)
        timestamp = int(imu.time_stamp)
        if timestamp > 0 and timestamp != last_timestamp:
            samples.append(
                ImuSample(
                    timestamp_ns=timestamp,
                    gyro=vector_tuple(imu.angular_velocity),
                    accel=vector_tuple(imu.linear_acceleration),
                )
            )
            last_timestamp = timestamp
        next_tick += period
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    if len(samples) < 5:
        raise RuntimeError(f"only {len(samples)} unique IMU timestamps collected")
    return samples


def vector_mean(samples: list[ImuSample], selector: Callable[[ImuSample], tuple]) -> tuple:
    vectors = [selector(sample) for sample in samples]
    return tuple(statistics.mean(vector[index] for vector in vectors) for index in range(3))


def norm(vector) -> float:
    return math.sqrt(sum(value * value for value in vector))


def state_to_dict(state) -> dict:
    return {
        "position": vector_tuple(state.position),
        "orientation_xyzw": (
            float(state.orientation.x_val),
            float(state.orientation.y_val),
            float(state.orientation.z_val),
            float(state.orientation.w_val),
        ),
        "linear_velocity": vector_tuple(state.linear_velocity),
        "angular_velocity": vector_tuple(state.angular_velocity),
        "linear_acceleration": vector_tuple(state.linear_acceleration),
        "angular_acceleration": vector_tuple(state.angular_acceleration),
    }


def main() -> int:
    args = parse_args()
    client = airsim.MultirotorClient()
    client.confirmConnection()

    settings = json.loads(client.getSettingsString())
    if settings.get("PhysicsEngineName") != "ExternalPhysicsEngine":
        raise SystemExit(
            "[ERROR] runtime PhysicsEngineName is not ExternalPhysicsEngine; "
            "apply the validation profile and restart Play/PIE"
        )
    vehicles = client.listVehicles()
    if args.vehicle not in vehicles:
        raise SystemExit(f"[ERROR] vehicle {args.vehicle!r} not found; runtime vehicles={vehicles}")

    original = copy_state(client.simGetGroundTruthKinematics(args.vehicle))
    base = make_state(position=vector_tuple(original.position))
    report: dict = {
        "vehicle": args.vehicle,
        "imu": args.imu,
        "runtime_physics_engine": settings.get("PhysicsEngineName"),
        "tests": {},
    }

    try:
        static_samples = hold_and_sample(client, base, args)
        static_gyro = vector_mean(static_samples, lambda sample: sample.gyro)
        static_accel = vector_mean(static_samples, lambda sample: sample.accel)

        gyro_state = copy_state(base)
        gyro_state.angular_velocity = airsim.Vector3r(args.gyro_test_radps, 0.0, 0.0)
        gyro_samples = hold_and_sample(client, gyro_state, args)
        gyro_mean = vector_mean(gyro_samples, lambda sample: sample.gyro)

        accel_state = copy_state(base)
        accel_state.linear_acceleration = airsim.Vector3r(args.accel_test_mps2, 0.0, 0.0)
        accel_samples = hold_and_sample(client, accel_state, args)
        accel_mean = vector_mean(accel_samples, lambda sample: sample.accel)

        returned = client.simGetGroundTruthKinematics(args.vehicle)

        static_pass = (
            norm(static_gyro) <= args.gyro_tolerance_radps
            and abs(norm(static_accel) - 9.81) <= args.gravity_tolerance_mps2
        )
        gyro_pass = (
            abs(gyro_mean[0] - args.gyro_test_radps) <= args.gyro_tolerance_radps
            and abs(gyro_mean[1]) <= args.gyro_tolerance_radps
            and abs(gyro_mean[2]) <= args.gyro_tolerance_radps
        )
        accel_delta = tuple(accel_mean[i] - static_accel[i] for i in range(3))
        accel_pass = (
            abs(accel_delta[0] - args.accel_test_mps2) <= args.accel_tolerance_mps2
            and abs(accel_delta[1]) <= args.accel_tolerance_mps2
            and abs(accel_delta[2]) <= args.accel_tolerance_mps2
        )
        returned_accel = vector_tuple(returned.linear_acceleration)
        ground_truth_pass = abs(returned_accel[0] - args.accel_test_mps2) <= 1e-4

        report["tests"] = {
            "static": {
                "pass": static_pass,
                "unique_samples": len(static_samples),
                "gyro_mean_radps": static_gyro,
                "accel_mean_mps2": static_accel,
                "accel_norm_mps2": norm(static_accel),
            },
            "gyro_x": {
                "pass": gyro_pass,
                "injected_radps": args.gyro_test_radps,
                "measured_mean_radps": gyro_mean,
                "unique_samples": len(gyro_samples),
            },
            "accel_x": {
                "pass": accel_pass,
                "injected_mps2": args.accel_test_mps2,
                "measured_mean_mps2": accel_mean,
                "measured_delta_from_static_mps2": accel_delta,
                "unique_samples": len(accel_samples),
            },
            "ground_truth_roundtrip": {
                "pass": ground_truth_pass,
                "returned": state_to_dict(returned),
            },
        }
    finally:
        client.simSetKinematics(original, True, args.vehicle)

    report["all_pass"] = all(item["pass"] for item in report["tests"].values())
    print(json.dumps(report, indent=2))

    if args.output:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"[REPORT] {output}")

    return 0 if report["all_pass"] else 2


if __name__ == "__main__":
    sys.exit(main())
