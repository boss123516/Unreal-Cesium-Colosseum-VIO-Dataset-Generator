#!/usr/bin/env python3
"""Explicit fixed-wing frame conversions used by the MVP bridge.

Conventions:

* Gazebo world: ENU (east, north, up)
* Gazebo body: FLU (forward, left, up)
* AirSim world: NED (north, east, down)
* AirSim body: FRD (forward, right, down)
* Quaternions: active body-to-world rotations in ``(x, y, z, w)`` order

Do not replace these functions with Euler-angle conversions. The matrices make
the world and body basis changes independently testable.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple

Vector3 = Tuple[float, float, float]
QuaternionXyzw = Tuple[float, float, float, float]
Matrix3 = Tuple[Vector3, Vector3, Vector3]

ENU_TO_NED: Matrix3 = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)

FLU_TO_FRD: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, -1.0),
)


@dataclass(frozen=True)
class FixedWingState:
    """A fully framed state at the Gazebo-to-UCC bridge boundary."""

    source_time_ns: int
    position_world_m: Vector3
    orientation_world_body_xyzw: QuaternionXyzw
    linear_velocity_world_mps: Vector3
    angular_velocity_body_radps: Vector3
    linear_acceleration_world_mps2: Vector3
    angular_acceleration_body_radps2: Vector3


def _as_finite_tuple(values: Iterable[float], length: int, name: str) -> tuple:
    result = tuple(float(value) for value in values)
    if len(result) != length:
        raise ValueError(f"{name} must contain {length} values, got {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} contains NaN or infinity: {result}")
    return result


def transform_vector(matrix: Matrix3, vector: Sequence[float]) -> Vector3:
    x, y, z = _as_finite_tuple(vector, 3, "vector")
    return tuple(
        row[0] * x + row[1] * y + row[2] * z
        for row in matrix
    )  # type: ignore[return-value]


def multiply_matrices(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def transpose(matrix: Matrix3) -> Matrix3:
    return tuple(
        tuple(matrix[column][row] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def normalize_quaternion_xyzw(quaternion: Sequence[float]) -> QuaternionXyzw:
    x, y, z, w = _as_finite_tuple(quaternion, 4, "quaternion")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    normalized = (x / norm, y / norm, z / norm, w / norm)
    if normalized[3] < 0.0:
        normalized = tuple(-value for value in normalized)
    return normalized  # type: ignore[return-value]


def quaternion_xyzw_to_matrix(quaternion: Sequence[float]) -> Matrix3:
    x, y, z, w = normalize_quaternion_xyzw(quaternion)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def matrix_to_quaternion_xyzw(matrix: Matrix3) -> QuaternionXyzw:
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m[2][1] - m[1][2]) / scale
        y = (m[0][2] - m[2][0]) / scale
        z = (m[1][0] - m[0][1]) / scale
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / scale
        x = 0.25 * scale
        y = (m[0][1] + m[1][0]) / scale
        z = (m[0][2] + m[2][0]) / scale
    elif m[1][1] > m[2][2]:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / scale
        x = (m[0][1] + m[1][0]) / scale
        y = 0.25 * scale
        z = (m[1][2] + m[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / scale
        x = (m[0][2] + m[2][0]) / scale
        y = (m[1][2] + m[2][1]) / scale
        z = 0.25 * scale

    return normalize_quaternion_xyzw((x, y, z, w))


def enu_world_to_ned(vector_enu: Sequence[float]) -> Vector3:
    """Convert an ENU world vector to NED: ``[n, e, d] = [y, x, -z]``."""

    return transform_vector(ENU_TO_NED, vector_enu)


def flu_body_to_frd(vector_flu: Sequence[float]) -> Vector3:
    """Convert a body vector from FLU to FRD: ``[f, r, d] = [x, -y, -z]``."""

    return transform_vector(FLU_TO_FRD, vector_flu)


def orientation_enu_flu_to_ned_frd(
    quaternion_enu_flu_xyzw: Sequence[float],
) -> QuaternionXyzw:
    """Convert a Gazebo FLU-to-ENU orientation to AirSim FRD-to-NED."""

    rotation_enu_flu = quaternion_xyzw_to_matrix(quaternion_enu_flu_xyzw)
    rotation_ned_frd = multiply_matrices(
        multiply_matrices(ENU_TO_NED, rotation_enu_flu),
        FLU_TO_FRD,
    )
    return matrix_to_quaternion_xyzw(rotation_ned_frd)


def gazebo_state_to_airsim(state: FixedWingState) -> FixedWingState:
    """Convert every framed vector in a Gazebo state to the AirSim convention."""

    if int(state.source_time_ns) < 0:
        raise ValueError("source_time_ns must be non-negative")

    return FixedWingState(
        source_time_ns=int(state.source_time_ns),
        position_world_m=enu_world_to_ned(state.position_world_m),
        orientation_world_body_xyzw=orientation_enu_flu_to_ned_frd(
            state.orientation_world_body_xyzw
        ),
        linear_velocity_world_mps=enu_world_to_ned(
            state.linear_velocity_world_mps
        ),
        angular_velocity_body_radps=flu_body_to_frd(
            state.angular_velocity_body_radps
        ),
        linear_acceleration_world_mps2=enu_world_to_ned(
            state.linear_acceleration_world_mps2
        ),
        angular_acceleration_body_radps2=flu_body_to_frd(
            state.angular_acceleration_body_radps2
        ),
    )
