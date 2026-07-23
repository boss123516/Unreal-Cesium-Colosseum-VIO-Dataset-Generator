#!/usr/bin/env python3
"""Validate KAU LoD1 3D Tiles, GLB payloads, and elevation metadata."""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path
from typing import Any

import numpy as np
from pyproj import Transformer


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_TILESET = SCRIPT_DIR / "output" / "tiles3d" / "tileset.json"
DEFAULT_BUILDINGS = (
    SCRIPT_DIR / "output" / "buildings_with_elevation.geojson"
)


class ValidationError(RuntimeError):
    pass


def read_glb(path: Path) -> tuple[dict[str, Any], bytes]:
    data = path.read_bytes()
    if len(data) < 28:
        raise ValidationError(f"{path.name}: GLB가 너무 작습니다.")
    magic, version, total_length = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF" or version != 2 or total_length != len(data):
        raise ValidationError(f"{path.name}: GLB 헤더가 잘못되었습니다.")
    json_length, json_type = struct.unpack_from("<I4s", data, 12)
    if json_type != b"JSON":
        raise ValidationError(f"{path.name}: JSON chunk가 없습니다.")
    json_start = 20
    document = json.loads(
        data[json_start : json_start + json_length].decode("utf-8").rstrip(" ")
    )
    binary_header = json_start + json_length
    binary_length, binary_type = struct.unpack_from(
        "<I4s", data, binary_header
    )
    if binary_type != b"BIN\x00":
        raise ValidationError(f"{path.name}: BIN chunk가 없습니다.")
    binary_start = binary_header + 8
    binary = data[binary_start : binary_start + binary_length]
    if len(binary) != binary_length:
        raise ValidationError(f"{path.name}: BIN chunk 길이가 잘못되었습니다.")
    return document, binary


def accessor_array(
    document: dict[str, Any],
    binary: bytes,
    accessor_index: int,
) -> np.ndarray:
    accessor = document["accessors"][accessor_index]
    view = document["bufferViews"][accessor["bufferView"]]
    component_types = {
        5125: ("<u4", 1),
        5126: ("<f4", 1),
    }
    type_widths = {"SCALAR": 1, "VEC3": 3}
    try:
        dtype, _ = component_types[accessor["componentType"]]
        width = type_widths[accessor["type"]]
    except KeyError as error:
        raise ValidationError(f"지원하지 않는 accessor: {accessor}") from error
    offset = int(view.get("byteOffset", 0)) + int(
        accessor.get("byteOffset", 0)
    )
    count = int(accessor["count"])
    values = np.frombuffer(
        binary,
        dtype=dtype,
        count=count * width,
        offset=offset,
    )
    return values.reshape(count, width)


def validate(
    tileset_path: Path,
    buildings_path: Path,
    output_report: Path,
) -> dict[str, Any]:
    tileset = json.loads(tileset_path.read_text(encoding="utf-8"))
    children = tileset["root"].get("children", [])
    if not children:
        raise ValidationError("tileset root에 child tile이 없습니다.")
    inverse_ecef = Transformer.from_crs(
        "EPSG:4978", "EPSG:4979", always_xy=True
    )
    seen_uris: set[str] = set()
    total_vertices = 0
    total_triangles = 0
    coordinate_failures = 0
    checked_samples = 0

    for child in children:
        uri = child["content"]["uri"]
        if uri in seen_uris:
            raise ValidationError(f"중복 content URI: {uri}")
        seen_uris.add(uri)
        glb_path = tileset_path.parent / uri
        if not glb_path.is_file():
            raise ValidationError(f"GLB 누락: {glb_path}")
        document, binary = read_glb(glb_path)
        primitive = document["meshes"][0]["primitives"][0]
        positions = accessor_array(
            document, binary, primitive["attributes"]["POSITION"]
        )
        normals = accessor_array(
            document, binary, primitive["attributes"]["NORMAL"]
        )
        indices = accessor_array(
            document, binary, primitive["indices"]
        ).reshape(-1)
        if len(normals) != len(positions):
            raise ValidationError(f"{uri}: normal 개수가 맞지 않습니다.")
        if not (
            np.all(np.isfinite(positions)) and np.all(np.isfinite(normals))
        ):
            raise ValidationError(f"{uri}: NaN/Inf 정점이 있습니다.")
        if len(indices) % 3 or int(indices.max()) >= len(positions):
            raise ValidationError(f"{uri}: triangle index가 잘못되었습니다.")
        total_vertices += len(positions)
        total_triangles += len(indices) // 3

        transform = np.asarray(child["transform"], dtype=np.float64).reshape(
            4, 4, order="F"
        )
        region = child["boundingVolume"]["region"]
        west, south, east, north = map(math.degrees, region[:4])
        min_height, max_height = region[4:6]
        sample_indices = sorted(
            {0, len(positions) // 2, len(positions) - 1}
        )
        for index in sample_indices:
            gltf = positions[index].astype(np.float64)
            enu_homogeneous = np.asarray(
                [gltf[0], -gltf[2], gltf[1], 1.0]
            )
            ecef = transform @ enu_homogeneous
            longitude, latitude, height = inverse_ecef.transform(*ecef[:3])
            checked_samples += 1
            if not (
                west - 1e-5 <= longitude <= east + 1e-5
                and south - 1e-5 <= latitude <= north + 1e-5
                and min_height - 0.2 <= height <= max_height + 0.2
            ):
                coordinate_failures += 1

    referenced_files = {
        (tileset_path.parent / uri).resolve() for uri in seen_uris
    }
    actual_files = {
        path.resolve()
        for path in (tileset_path.parent / "tiles").glob("*.glb")
    }
    if referenced_files != actual_files:
        raise ValidationError(
            f"참조/실제 GLB 불일치: referenced={len(referenced_files)}, "
            f"actual={len(actual_files)}"
        )
    if coordinate_failures:
        raise ValidationError(
            f"타일 bounding region 밖 샘플: {coordinate_failures}"
        )

    buildings = json.loads(buildings_path.read_text(encoding="utf-8"))
    features = buildings.get("features", [])
    missing_elevation = 0
    roof_equation_failures = 0
    base_values: list[float] = []
    roof_values: list[float] = []
    for feature in features:
        derived = (feature.get("properties") or {}).get("_kau5km") or {}
        required = (
            "height_final_m",
            "terrain_elevation_orthometric_m",
            "geoid_separation_m",
            "base_elevation_m",
            "roof_elevation_m",
        )
        if any(name not in derived for name in required):
            missing_elevation += 1
            continue
        base = float(derived["base_elevation_m"])
        roof = float(derived["roof_elevation_m"])
        height = float(derived["height_final_m"])
        if abs((base + height) - roof) > 0.002:
            roof_equation_failures += 1
        base_values.append(base)
        roof_values.append(roof)
    if missing_elevation or roof_equation_failures:
        raise ValidationError(
            f"고도 메타데이터 오류: missing={missing_elevation}, "
            f"equation={roof_equation_failures}"
        )

    report = {
        "status": "PASS",
        "tile_count": len(children),
        "glb_count": len(actual_files),
        "feature_count": len(features),
        "total_vertices": total_vertices,
        "total_triangles": total_triangles,
        "sampled_ecef_positions": checked_samples,
        "coordinate_failures": coordinate_failures,
        "missing_elevation": missing_elevation,
        "roof_equation_failures": roof_equation_failures,
        "base_elevation_range_m": [min(base_values), max(base_values)],
        "roof_elevation_range_m": [min(roof_values), max(roof_values)],
    }
    output_report.parent.mkdir(parents=True, exist_ok=True)
    output_report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="KAU LoD1 3D Tiles 검증")
    parser.add_argument("--tileset", type=Path, default=DEFAULT_TILESET)
    parser.add_argument("--buildings", type=Path, default=DEFAULT_BUILDINGS)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_TILESET.parent / "validation_report.json",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        report = validate(
            args.tileset.resolve(),
            args.buildings.resolve(),
            args.output.resolve(),
        )
    except ValidationError as error:
        print(f"[검증 실패] {error}")
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
