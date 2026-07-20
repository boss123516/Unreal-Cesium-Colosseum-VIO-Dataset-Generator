#!/usr/bin/env python3
"""Direct Gazebo link-state to AirSim External Physics bridge.

The matching Gazebo system plugin publishes the following ``gz.msgs.Double_V``
contract at the physics rate:

0 contract version; 1 simulation time ns; 2:5 position world ENU;
5:9 body-FLU-to-world-ENU quaternion xyzw; 9:12 linear velocity world ENU;
12:15 angular velocity body FLU; 15:18 linear acceleration world ENU;
18:21 angular acceleration body FLU.

No acceleration field in this bridge is calculated by finite difference.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass, replace
import json
import math
import os
from pathlib import Path
import signal
import socket
import statistics
import sys
import threading
import time
from typing import Any, Sequence

from fixedwing_frames import FixedWingState, gazebo_state_to_airsim


Vector3 = tuple[float, float, float]
CONTRACT_VERSION = 1.0
CONTRACT_LENGTH = 21


@dataclass(frozen=True)
class ReceivedState:
    state: FixedWingState
    received_monotonic_ns: int


@dataclass
class BridgeCounters:
    source_messages: int = 0
    source_states: int = 0
    duplicate_source_states: int = 0
    superseded_source_states: int = 0
    injections: int = 0
    missed_injection_deadlines: int = 0
    source_timestamp_regressions: int = 0
    invalid_contract_messages: int = 0
    invalid_numeric_states: int = 0
    gcs_heartbeats_sent: int = 0
    rpc_failures: int = 0
    max_consecutive_rpc_failures: int = 0


def finite_tuple(values: Sequence[float], length: int, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != length:
        raise ValueError(f"{name} must contain {length} values, got {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} contains NaN or infinity: {result}")
    return result


def decode_kinematics_payload(values: Sequence[float]) -> FixedWingState:
    data = finite_tuple(values, CONTRACT_LENGTH, "Gazebo kinematics payload")
    if data[0] != CONTRACT_VERSION:
        raise ValueError(f"unsupported Gazebo kinematics contract: {data[0]}")
    source_time_float = data[1]
    source_time_ns = int(round(source_time_float))
    if source_time_ns < 0 or abs(source_time_float - source_time_ns) > 0.25:
        raise ValueError(f"invalid Gazebo simulation timestamp: {source_time_float}")
    return FixedWingState(
        source_time_ns=source_time_ns,
        position_world_m=data[2:5],  # type: ignore[arg-type]
        orientation_world_body_xyzw=data[5:9],  # type: ignore[arg-type]
        linear_velocity_world_mps=data[9:12],  # type: ignore[arg-type]
        angular_velocity_body_radps=data[12:15],  # type: ignore[arg-type]
        linear_acceleration_world_mps2=data[15:18],  # type: ignore[arg-type]
        angular_acceleration_body_radps2=data[18:21],  # type: ignore[arg-type]
    )


def reanchor_state(
    state_ned_frd: FixedWingState,
    source_origin_ned: Vector3,
    target_origin_ned: Vector3,
) -> FixedWingState:
    source = finite_tuple(source_origin_ned, 3, "source origin")
    target = finite_tuple(target_origin_ned, 3, "target origin")
    position = tuple(
        target[index]
        + state_ned_frd.position_world_m[index]
        - source[index]
        for index in range(3)
    )
    return replace(state_ned_frd, position_world_m=position)  # type: ignore[arg-type]


def percentile(values: list[float], percentage: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = min(
        len(ordered) - 1,
        max(0, math.ceil(len(ordered) * percentage / 100.0) - 1),
    )
    return ordered[index]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/ucc/fixed_wing/kinematics")
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--source-timeout-sec", type=float, default=0.5)
    parser.add_argument("--connect-timeout-sec", type=float, default=15.0)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument(
        "--gcs-heartbeat-endpoint", default="udpout:127.0.0.1:14580"
    )
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--state-log", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def airsim_state(airsim: Any, state: FixedWingState) -> Any:
    kinematics = airsim.KinematicsState()
    kinematics.position = airsim.Vector3r(*state.position_world_m)
    kinematics.orientation = airsim.Quaternionr(
        *state.orientation_world_body_xyzw
    )
    kinematics.linear_velocity = airsim.Vector3r(*state.linear_velocity_world_mps)
    kinematics.angular_velocity = airsim.Vector3r(
        *state.angular_velocity_body_radps
    )
    kinematics.linear_acceleration = airsim.Vector3r(
        *state.linear_acceleration_world_mps2
    )
    kinematics.angular_acceleration = airsim.Vector3r(
        *state.angular_acceleration_body_radps2
    )
    return kinematics


def open_state_log(path: Path | None):
    if path is None:
        return None, None
    output = path.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    file_handle = output.open("w", newline="", encoding="utf-8")
    writer = csv.writer(file_handle)
    writer.writerow(
        [
            "source_time_ns",
            "receive_monotonic_ns",
            "inject_monotonic_ns",
            "px",
            "py",
            "pz",
            "qx",
            "qy",
            "qz",
            "qw",
            "vx",
            "vy",
            "vz",
            "wx",
            "wy",
            "wz",
            "ax",
            "ay",
            "az",
            "alphax",
            "alphay",
            "alphaz",
            "receive_to_inject_ms",
        ]
    )
    return file_handle, writer


def write_state_row(
    writer: Any,
    received: ReceivedState,
    inject_ns: int,
    latency_ms: float,
) -> None:
    if writer is None:
        return
    state = received.state
    writer.writerow(
        [
            state.source_time_ns,
            received.received_monotonic_ns,
            inject_ns,
            *state.position_world_m,
            *state.orientation_world_body_xyzw,
            *state.linear_velocity_world_mps,
            *state.angular_velocity_body_radps,
            *state.linear_acceleration_world_mps2,
            *state.angular_acceleration_body_radps2,
            latency_ms,
        ]
    )


def main() -> int:
    args = parse_args()
    if args.rate_hz <= 0.0 or args.source_timeout_sec <= 0.0:
        raise SystemExit("[ERROR] rates and timeouts must be positive")

    try:
        import airsim
        from gz.msgs10.double_v_pb2 import Double_V
        from gz.transport13 import Node
        from pymavlink import mavutil
    except ImportError as exc:
        raise SystemExit(f"[ERROR] missing native bridge dependency: {exc}") from exc

    client = None
    target_origin: Vector3 = (0.0, 0.0, 0.0)
    if not args.dry_run:
        try:
            with socket.create_connection(("127.0.0.1", 41451), timeout=1.0):
                pass
        except OSError as exc:
            raise SystemExit(
                f"[ERROR] AirSim RPC 127.0.0.1:41451 unavailable: {exc}"
            ) from exc
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

    counters = BridgeCounters()
    state_lock = threading.Lock()
    latest_state: ReceivedState | None = None
    source_origin: Vector3 | None = None
    latest_generation = 0
    source_periods_ms: list[float] = []
    first_source_time_ns: int | None = None
    last_source_time_ns: int | None = None

    def source_callback(message: Any) -> None:
        nonlocal latest_state, source_origin, latest_generation
        nonlocal first_source_time_ns, last_source_time_ns
        received_ns = time.monotonic_ns()
        with state_lock:
            counters.source_messages += 1
        try:
            raw = decode_kinematics_payload(message.data)
            converted = gazebo_state_to_airsim(raw)
        except ValueError as exc:
            with state_lock:
                if "contract" in str(exc) or "payload" in str(exc):
                    counters.invalid_contract_messages += 1
                else:
                    counters.invalid_numeric_states += 1
            return

        with state_lock:
            if last_source_time_ns is not None:
                if converted.source_time_ns < last_source_time_ns:
                    counters.source_timestamp_regressions += 1
                    return
                if converted.source_time_ns == last_source_time_ns:
                    counters.duplicate_source_states += 1
                    return
                source_periods_ms.append(
                    (converted.source_time_ns - last_source_time_ns) * 1.0e-6
                )
            if source_origin is None:
                source_origin = converted.position_world_m
            anchored = reanchor_state(converted, source_origin, target_origin)
            latest_state = ReceivedState(anchored, received_ns)
            latest_generation += 1
            counters.source_states += 1
            if first_source_time_ns is None:
                first_source_time_ns = converted.source_time_ns
            last_source_time_ns = converted.source_time_ns

    node = Node()
    node.subscribe(Double_V, args.topic, source_callback)
    print(f"[CONNECT] waiting for Gazebo state on {args.topic}")

    gcs_connection = None
    if args.gcs_heartbeat_endpoint.lower() != "none":
        gcs_connection = mavutil.mavlink_connection(args.gcs_heartbeat_endpoint)

    start = time.monotonic()
    first_state_deadline = start + args.connect_timeout_sec
    next_injection = start
    next_gcs_heartbeat = start
    next_report = start + 1.0
    injected_generation = 0
    consecutive_rpc_failures = 0
    latencies_ms: list[float] = []
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
                if args.duration_sec > 0.0 and now - start >= args.duration_sec:
                    break

                with state_lock:
                    received = latest_state
                    generation = latest_generation

                if received is None:
                    if now > first_state_deadline:
                        raise RuntimeError(
                            f"no Gazebo state within {args.connect_timeout_sec:.3f}s"
                        )
                else:
                    age_sec = (
                        time.monotonic_ns() - received.received_monotonic_ns
                    ) * 1.0e-9
                    if age_sec > args.source_timeout_sec:
                        raise RuntimeError(f"Gazebo state timeout: {age_sec:.3f}s")

                if gcs_connection is not None and now >= next_gcs_heartbeat:
                    gcs_connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        mavutil.mavlink.MAV_STATE_ACTIVE,
                    )
                    counters.gcs_heartbeats_sent += 1
                    next_gcs_heartbeat += 1.0

                if now >= next_injection and received is not None:
                    period = 1.0 / args.rate_hz
                    if now - next_injection >= period:
                        missed = int((now - next_injection) / period)
                        counters.missed_injection_deadlines += missed
                        next_injection += missed * period

                    inject_ns = time.monotonic_ns()
                    latency_ms = (
                        inject_ns - received.received_monotonic_ns
                    ) * 1.0e-6
                    latencies_ms.append(latency_ms)
                    if generation > injected_generation + 1:
                        counters.superseded_source_states += (
                            generation - injected_generation - 1
                        )
                    injected_generation = generation

                    if client is not None:
                        try:
                            client.simSetKinematics(
                                airsim_state(airsim, received.state),
                                True,
                                args.vehicle,
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
                    write_state_row(state_writer, received, inject_ns, latency_ms)
                    next_injection += period

                if now >= next_report:
                    elapsed = now - start
                    print(
                        f"[RATE] source={counters.source_states / elapsed:.1f}Hz "
                        f"inject={counters.injections / elapsed:.1f}Hz "
                        f"latency_p95={percentile(latencies_ms, 95):.2f}ms "
                        f"missed={counters.missed_injection_deadlines}"
                    )
                    next_report += 1.0

                delay = min(max(0.0, next_injection - time.monotonic()), 0.002)
                if delay > 0.0:
                    time.sleep(delay)
        except RuntimeError as exc:
            fatal_error = str(exc)
            print(f"[ERROR] {fatal_error}", file=sys.stderr)
    finally:
        if state_file is not None:
            state_file.close()
        if gcs_connection is not None:
            gcs_connection.close()

    elapsed = max(time.monotonic() - start, 1.0e-9)
    sim_elapsed_sec = (
        (last_source_time_ns - first_source_time_ns) * 1.0e-9
        if first_source_time_ns is not None
        and last_source_time_ns is not None
        and last_source_time_ns >= first_source_time_ns
        else math.nan
    )
    injection_rate = counters.injections / elapsed
    source_rate = counters.source_states / elapsed
    summary = {
        "all_pass": (
            fatal_error is None
            and counters.injections > 0
            and injection_rate >= args.rate_hz * 0.95
            and counters.source_timestamp_regressions == 0
            and counters.invalid_contract_messages == 0
            and counters.invalid_numeric_states == 0
            and counters.max_consecutive_rpc_failures < 5
        ),
        "dry_run": args.dry_run,
        "gazebo_topic": args.topic,
        "vehicle": args.vehicle,
        "fatal_error": fatal_error,
        "elapsed_sec": elapsed,
        "gazebo_sim_elapsed_sec": sim_elapsed_sec,
        "sim_to_wall_time_ratio": sim_elapsed_sec / elapsed,
        "source_state_rate_hz": source_rate,
        "injection_rate_hz": injection_rate,
        "source_period_ms": {
            "mean": statistics.mean(source_periods_ms)
            if source_periods_ms
            else math.nan,
            "p95": percentile(source_periods_ms, 95),
            "max": max(source_periods_ms) if source_periods_ms else math.nan,
        },
        "receive_to_inject_latency_ms": {
            "mean": statistics.mean(latencies_ms) if latencies_ms else math.nan,
            "p95": percentile(latencies_ms, 95),
            "max": max(latencies_ms) if latencies_ms else math.nan,
        },
        "counters": asdict(counters),
        "acceleration_source": "gazebo_world_link_component",
        "angular_acceleration_source": "gazebo_world_link_component_rotated_to_body",
    }
    print(json.dumps(summary, indent=2))

    if args.summary:
        output = args.summary.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"[SUMMARY] {output}")

    # Gazebo Transport 13's Python subscriber teardown can deadlock or call
    # std::terminate while its callback worker is active. All owned files and
    # sockets are closed above, so bypass pybind interpreter finalization after
    # flushing the completed result.
    exit_code = 0 if summary["all_pass"] else 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)


if __name__ == "__main__":
    sys.exit(main())
