#!/usr/bin/env python3
"""PX4 MAVLink telemetry to AirSim full-kinematics MVP bridge.

PX4 already publishes local position, velocity and attitude in NED/FRD. This
MVP therefore does not apply the Gazebo ENU/FLU conversion. Translational and
angular accelerations are finite differences for the integration gate only;
the research bridge must replace them with Gazebo ground-truth acceleration.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import signal
import socket
import statistics
import sys
import time
from typing import Any, Sequence


Vector3 = tuple[float, float, float]
QuaternionXyzw = tuple[float, float, float, float]


@dataclass(frozen=True)
class BridgeState:
    source_time_ns: int
    received_monotonic_ns: int
    position_ned_m: Vector3
    orientation_ned_frd_xyzw: QuaternionXyzw
    linear_velocity_ned_mps: Vector3
    angular_velocity_frd_radps: Vector3
    linear_acceleration_ned_mps2: Vector3
    angular_acceleration_frd_radps2: Vector3


@dataclass
class BridgeCounters:
    source_messages: int = 0
    source_states: int = 0
    duplicate_source_states: int = 0
    gcs_heartbeats_sent: int = 0
    injections: int = 0
    dropped_states: int = 0
    source_timestamp_regressions: int = 0
    invalid_quaternions: int = 0
    invalid_numeric_states: int = 0
    rpc_failures: int = 0
    max_consecutive_rpc_failures: int = 0


def finite_vector(values: Sequence[float], name: str) -> Vector3:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"invalid {name}: {result}")
    return result  # type: ignore[return-value]


def normalize_quaternion_xyzw(values: Sequence[float]) -> QuaternionXyzw:
    result = tuple(float(value) for value in values)
    if len(result) != 4 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"invalid quaternion: {result}")
    norm = math.sqrt(sum(value * value for value in result))
    if norm < 1e-9 or abs(norm - 1.0) > 0.1:
        raise ValueError(f"quaternion norm out of range: {norm}")
    normalized = tuple(value / norm for value in result)
    if normalized[3] < 0:
        normalized = tuple(-value for value in normalized)
    return normalized  # type: ignore[return-value]


def vector_subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] - right[index] for index in range(3))  # type: ignore[return-value]


def vector_add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] + right[index] for index in range(3))  # type: ignore[return-value]


def vector_scale(vector: Vector3, scale: float) -> Vector3:
    return tuple(value * scale for value in vector)  # type: ignore[return-value]


def low_pass(previous: Vector3, current: Vector3, alpha: float) -> Vector3:
    return tuple(
        alpha * current[index] + (1.0 - alpha) * previous[index]
        for index in range(3)
    )  # type: ignore[return-value]


class MavlinkStateAssembler:
    """Fuse PX4 attitude and local-position messages into framed bridge states."""

    def __init__(self, target_origin_ned: Vector3, acceleration_alpha: float = 0.2):
        if not 0.0 < acceleration_alpha <= 1.0:
            raise ValueError("acceleration_alpha must be in (0, 1]")
        self.target_origin_ned = finite_vector(target_origin_ned, "target origin")
        self.acceleration_alpha = acceleration_alpha
        self.source_origin_ned: Vector3 | None = None
        self.orientation: QuaternionXyzw | None = None
        self.angular_velocity: Vector3 = (0.0, 0.0, 0.0)
        self.angular_acceleration: Vector3 = (0.0, 0.0, 0.0)
        self.previous_angular_velocity: Vector3 | None = None
        self.previous_attitude_time_ns: int | None = None
        self.previous_velocity: Vector3 | None = None
        self.previous_position_time_ns: int | None = None
        self.linear_acceleration: Vector3 = (0.0, 0.0, 0.0)
        self.last_source_time_ns: int | None = None
        self.duplicate_source_states = 0
        self.timestamp_regressions = 0
        self.invalid_quaternions = 0
        self.invalid_numeric_states = 0

    @staticmethod
    def _message_time_ns(message: Any) -> int:
        time_boot_ms = int(getattr(message, "time_boot_ms"))
        if time_boot_ms < 0:
            raise ValueError("negative PX4 boot timestamp")
        return time_boot_ms * 1_000_000

    def _track_timestamp(self, timestamp_ns: int) -> None:
        if self.last_source_time_ns is not None and timestamp_ns < self.last_source_time_ns:
            self.timestamp_regressions += 1
        self.last_source_time_ns = max(timestamp_ns, self.last_source_time_ns or 0)

    def consume(self, message: Any, received_monotonic_ns: int) -> BridgeState | None:
        message_type = message.get_type()
        try:
            if message_type == "ATTITUDE_QUATERNION":
                timestamp_ns = self._message_time_ns(message)
                self.orientation = normalize_quaternion_xyzw(
                    (message.q2, message.q3, message.q4, message.q1)
                )
                current_rate = finite_vector(
                    (message.rollspeed, message.pitchspeed, message.yawspeed),
                    "body angular velocity",
                )
                if (
                    self.previous_angular_velocity is not None
                    and self.previous_attitude_time_ns is not None
                    and timestamp_ns > self.previous_attitude_time_ns
                ):
                    dt = (timestamp_ns - self.previous_attitude_time_ns) * 1e-9
                    raw = vector_scale(
                        vector_subtract(current_rate, self.previous_angular_velocity),
                        1.0 / dt,
                    )
                    self.angular_acceleration = low_pass(
                        self.angular_acceleration, raw, self.acceleration_alpha
                    )
                self.angular_velocity = current_rate
                self.previous_angular_velocity = current_rate
                self.previous_attitude_time_ns = timestamp_ns
                return None

            if message_type != "LOCAL_POSITION_NED":
                return None

            timestamp_ns = self._message_time_ns(message)
            self._track_timestamp(timestamp_ns)
            if (
                self.previous_position_time_ns is not None
                and timestamp_ns == self.previous_position_time_ns
            ):
                self.duplicate_source_states += 1
                return None
            source_position = finite_vector((message.x, message.y, message.z), "position")
            velocity = finite_vector((message.vx, message.vy, message.vz), "velocity")
            if self.orientation is None:
                return None
            if self.source_origin_ned is None:
                self.source_origin_ned = source_position

            if (
                self.previous_velocity is not None
                and self.previous_position_time_ns is not None
                and timestamp_ns > self.previous_position_time_ns
            ):
                dt = (timestamp_ns - self.previous_position_time_ns) * 1e-9
                raw = vector_scale(
                    vector_subtract(velocity, self.previous_velocity), 1.0 / dt
                )
                self.linear_acceleration = low_pass(
                    self.linear_acceleration, raw, self.acceleration_alpha
                )

            self.previous_velocity = velocity
            self.previous_position_time_ns = timestamp_ns
            target_position = vector_add(
                self.target_origin_ned,
                vector_subtract(source_position, self.source_origin_ned),
            )
            return BridgeState(
                source_time_ns=timestamp_ns,
                received_monotonic_ns=int(received_monotonic_ns),
                position_ned_m=target_position,
                orientation_ned_frd_xyzw=self.orientation,
                linear_velocity_ned_mps=velocity,
                angular_velocity_frd_radps=self.angular_velocity,
                linear_acceleration_ned_mps2=self.linear_acceleration,
                angular_acceleration_frd_radps2=self.angular_acceleration,
            )
        except ValueError:
            if message_type == "ATTITUDE_QUATERNION":
                self.invalid_quaternions += 1
            else:
                self.invalid_numeric_states += 1
            return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mavlink", default="udpin:0.0.0.0:14540")
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--source-timeout-sec", type=float, default=0.5)
    parser.add_argument("--connect-timeout-sec", type=float, default=15.0)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--acceleration-alpha", type=float, default=0.2)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--state-log", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def percentile(values: list[float], percentage: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(len(ordered) * percentage / 100) - 1))
    return ordered[index]


def airsim_state(airsim, state: BridgeState):
    kinematics = airsim.KinematicsState()
    kinematics.position = airsim.Vector3r(*state.position_ned_m)
    kinematics.orientation = airsim.Quaternionr(*state.orientation_ned_frd_xyzw)
    kinematics.linear_velocity = airsim.Vector3r(*state.linear_velocity_ned_mps)
    kinematics.angular_velocity = airsim.Vector3r(*state.angular_velocity_frd_radps)
    kinematics.linear_acceleration = airsim.Vector3r(*state.linear_acceleration_ned_mps2)
    kinematics.angular_acceleration = airsim.Vector3r(*state.angular_acceleration_frd_radps2)
    return kinematics


def request_message_interval(connection, mavutil, message_id: int, rate_hz: float) -> None:
    interval_us = int(round(1_000_000.0 / rate_hz))
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def open_state_log(path: Path | None):
    if path is None:
        return None, None
    output = path.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    file_handle = output.open("w", newline="", encoding="utf-8")
    writer = csv.writer(file_handle)
    writer.writerow(
        [
            "source_time_ns", "receive_monotonic_ns", "inject_monotonic_ns",
            "px", "py", "pz", "qx", "qy", "qz", "qw",
            "vx", "vy", "vz", "wx", "wy", "wz",
            "ax", "ay", "az", "alphax", "alphay", "alphaz", "latency_ms",
        ]
    )
    return file_handle, writer


def write_state_row(writer, state: BridgeState, inject_ns: int, latency_ms: float) -> None:
    if writer is None:
        return
    writer.writerow(
        [
            state.source_time_ns,
            state.received_monotonic_ns,
            inject_ns,
            *state.position_ned_m,
            *state.orientation_ned_frd_xyzw,
            *state.linear_velocity_ned_mps,
            *state.angular_velocity_frd_radps,
            *state.linear_acceleration_ned_mps2,
            *state.angular_acceleration_frd_radps2,
            latency_ms,
        ]
    )


def main() -> int:
    args = parse_args()
    if args.rate_hz <= 0 or args.source_timeout_sec <= 0:
        raise SystemExit("[ERROR] rates and timeouts must be positive")

    try:
        import airsim
        from pymavlink import mavutil
    except ImportError as exc:
        raise SystemExit(f"[ERROR] missing bridge dependency: {exc}") from exc

    client = None
    target_origin = (0.0, 0.0, 0.0)
    if not args.dry_run:
        try:
            with socket.create_connection(("127.0.0.1", 41451), timeout=1.0):
                pass
        except OSError as exc:
            raise SystemExit(f"[ERROR] AirSim RPC 127.0.0.1:41451 unavailable: {exc}")
        client = airsim.MultirotorClient()
        client.confirmConnection()
        if args.vehicle not in client.listVehicles():
            raise SystemExit(f"[ERROR] AirSim vehicle not found: {args.vehicle}")
        runtime_settings = json.loads(client.getSettingsString())
        if runtime_settings.get("PhysicsEngineName") != "ExternalPhysicsEngine":
            raise SystemExit("[ERROR] AirSim is not running ExternalPhysicsEngine")
        initial = client.simGetGroundTruthKinematics(args.vehicle)
        target_origin = (
            float(initial.position.x_val),
            float(initial.position.y_val),
            float(initial.position.z_val),
        )

    print(f"[CONNECT] waiting for PX4 heartbeat on {args.mavlink}")
    connection = mavutil.mavlink_connection(args.mavlink)
    heartbeat = connection.wait_heartbeat(timeout=args.connect_timeout_sec)
    if heartbeat is None:
        raise SystemExit(f"[ERROR] no PX4 heartbeat within {args.connect_timeout_sec}s")
    print(
        f"[CONNECTED] system={connection.target_system} "
        f"component={connection.target_component}"
    )

    request_message_interval(connection, mavutil, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, args.rate_hz)
    request_message_interval(connection, mavutil, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, args.rate_hz)

    assembler = MavlinkStateAssembler(target_origin, args.acceleration_alpha)
    counters = BridgeCounters()
    latencies_ms: list[float] = []
    start = time.monotonic()
    next_injection = start
    next_gcs_heartbeat = start
    next_report = start + 1.0
    first_state_deadline = start + args.source_timeout_sec
    latest_state: BridgeState | None = None
    latest_generation = 0
    injected_generation = 0
    consecutive_rpc_failures = 0
    stopping = False
    fatal_error: str | None = None

    def stop_handler(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    state_file, state_writer = open_state_log(args.state_log)

    try:
        try:
            while not stopping:
                now = time.monotonic()
                if args.duration_sec > 0 and now - start >= args.duration_sec:
                    break

                if now >= next_gcs_heartbeat:
                    connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        mavutil.mavlink.MAV_STATE_ACTIVE,
                    )
                    counters.gcs_heartbeats_sent += 1
                    next_gcs_heartbeat += 1.0

                while True:
                    message = connection.recv_match(blocking=False)
                    if message is None:
                        break
                    counters.source_messages += 1
                    state = assembler.consume(message, time.monotonic_ns())
                    if state is not None:
                        latest_state = state
                        latest_generation += 1
                        counters.source_states += 1

                if latest_state is None and now > first_state_deadline:
                    raise RuntimeError(
                        f"no complete PX4 state within {args.source_timeout_sec:.3f}s"
                    )
                if latest_state is not None:
                    age_sec = (
                        time.monotonic_ns() - latest_state.received_monotonic_ns
                    ) * 1e-9
                    if age_sec > args.source_timeout_sec:
                        raise RuntimeError(f"PX4 state timeout: {age_sec:.3f}s")

                if now >= next_injection and latest_state is not None:
                    inject_ns = time.monotonic_ns()
                    latency_ms = (
                        inject_ns - latest_state.received_monotonic_ns
                    ) * 1e-6
                    latencies_ms.append(latency_ms)
                    if latest_generation > injected_generation + 1:
                        counters.dropped_states += (
                            latest_generation - injected_generation - 1
                        )
                    injected_generation = latest_generation
                    if client is not None:
                        try:
                            client.simSetKinematics(
                                airsim_state(airsim, latest_state), True, args.vehicle
                            )
                            consecutive_rpc_failures = 0
                        except Exception as exc:
                            counters.rpc_failures += 1
                            consecutive_rpc_failures += 1
                            counters.max_consecutive_rpc_failures = max(
                                counters.max_consecutive_rpc_failures,
                                consecutive_rpc_failures,
                            )
                            if consecutive_rpc_failures >= 5:
                                raise RuntimeError(
                                    "five consecutive AirSim RPC failures"
                                ) from exc
                    counters.injections += 1
                    write_state_row(state_writer, latest_state, inject_ns, latency_ms)
                    next_injection += 1.0 / args.rate_hz
                    if now - next_injection > 0.25:
                        next_injection = now

                if now >= next_report:
                    elapsed = now - start
                    p95 = percentile(latencies_ms, 95)
                    print(
                        f"[RATE] source={counters.source_states / elapsed:.1f}Hz "
                        f"inject={counters.injections / elapsed:.1f}Hz "
                        f"latency_p95={p95:.2f}ms drops={counters.dropped_states}"
                    )
                    next_report += 1.0

                delay = min(max(0.0, next_injection - time.monotonic()), 0.002)
                if delay > 0:
                    time.sleep(delay)
        except RuntimeError as exc:
            fatal_error = str(exc)
            print(f"[ERROR] {fatal_error}", file=sys.stderr)
    finally:
        if state_file is not None:
            state_file.close()

    elapsed = max(time.monotonic() - start, 1e-9)
    counters.source_timestamp_regressions = assembler.timestamp_regressions
    counters.duplicate_source_states = assembler.duplicate_source_states
    counters.invalid_quaternions = assembler.invalid_quaternions
    counters.invalid_numeric_states = assembler.invalid_numeric_states
    summary = {
        "all_pass": (
            fatal_error is None
            and counters.injections > 0
            and counters.source_timestamp_regressions == 0
            and counters.invalid_quaternions == 0
            and counters.invalid_numeric_states == 0
            and counters.max_consecutive_rpc_failures < 5
        ),
        "dry_run": args.dry_run,
        "mavlink_endpoint": args.mavlink,
        "vehicle": args.vehicle,
        "fatal_error": fatal_error,
        "elapsed_sec": elapsed,
        "source_state_rate_hz": counters.source_states / elapsed,
        "injection_rate_hz": counters.injections / elapsed,
        "latency_ms": {
            "mean": statistics.mean(latencies_ms) if latencies_ms else math.nan,
            "p95": percentile(latencies_ms, 95),
            "max": max(latencies_ms) if latencies_ms else math.nan,
        },
        "counters": asdict(counters),
        "acceleration_source": "finite_difference_mvp_only",
    }
    print(json.dumps(summary, indent=2))

    if args.summary:
        output = args.summary.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"[SUMMARY] {output}")
    return 0 if summary["all_pass"] else 2


if __name__ == "__main__":
    sys.exit(main())
