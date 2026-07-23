#!/usr/bin/env python3
"""Build terrain-grounded LoD1 building GLBs and a Cesium 3D Tiles tileset.

Input building geometry is WGS84 GeoJSON. Ground elevations are sampled from a
1 arc-second HGT DEM in EGM96 orthometric metres and transformed to WGS84
ellipsoid heights with the system EGM96 grid. Each building is extruded by its
existing `_kau5km.height_final_m` value.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import os
import struct
import sys
import time
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Iterator, Sequence

import mapbox_earcut
import numpy as np
import pyproj
from pyproj import Transformer
from shapely import make_valid
from shapely.geometry import MultiPolygon, Polygon, shape
from shapely.geometry.polygon import orient


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_BUILDINGS = SCRIPT_DIR / "output" / "buildings.geojson"
DEFAULT_PROFILE = SCRIPT_DIR / "profile.json"
DEFAULT_DEM = SCRIPT_DIR / "dem" / "N37E126.hgt.gz"
DEFAULT_OUTPUT = SCRIPT_DIR / "output" / "tiles3d"
DEFAULT_ENRICHED = SCRIPT_DIR / "output" / "buildings_with_elevation.geojson"
SYSTEM_PROJ_DATA = Path("/usr/share/proj")
WGS84_SEMI_MAJOR_M = 6_378_137.0


class BuildError(RuntimeError):
    """Raised when the 3D result cannot be built without silent data loss."""


@dataclass(frozen=True)
class TileFrame:
    longitude: float
    latitude: float
    origin_ecef: np.ndarray
    ecef_to_enu: np.ndarray
    enu_to_ecef_transform: list[float]


class HgtDem:
    """Bilinear and neighborhood sampling for one 1° HGT tile."""

    def __init__(self, path: Path) -> None:
        self.path = path
        stem = path.name
        if stem.endswith(".gz"):
            stem = stem[:-3]
        if stem.endswith(".hgt"):
            stem = stem[:-4]
        if len(stem) != 7 or stem[0] not in "NS" or stem[3] not in "EW":
            raise BuildError(f"HGT 파일명에서 좌표를 읽을 수 없습니다: {path.name}")
        latitude = int(stem[1:3]) * (1 if stem[0] == "N" else -1)
        longitude = int(stem[4:7]) * (1 if stem[3] == "E" else -1)
        self.south = float(latitude)
        self.west = float(longitude)
        opener = gzip.open if path.suffix == ".gz" else open
        with opener(path, "rb") as stream:
            raw = stream.read()
        sample_count = len(raw) // 2
        side = math.isqrt(sample_count)
        if side * side != sample_count or side not in (1201, 3601):
            raise BuildError(
                f"지원하지 않는 HGT 크기: {len(raw)} bytes ({sample_count} samples)"
            )
        self.side = side
        self.intervals = side - 1
        self.data = np.frombuffer(raw, dtype=">i2").reshape(side, side)

    def _fractional_index(
        self, longitude: float, latitude: float
    ) -> tuple[float, float]:
        if not (
            self.west <= longitude <= self.west + 1.0
            and self.south <= latitude <= self.south + 1.0
        ):
            raise BuildError(
                f"DEM 범위 밖 좌표: lon={longitude}, lat={latitude}"
            )
        row = (self.south + 1.0 - latitude) * self.intervals
        column = (longitude - self.west) * self.intervals
        return row, column

    def sample_ground(
        self,
        longitude: float,
        latitude: float,
        neighborhood: int = 2,
        percentile: float = 25.0,
    ) -> float:
        """Estimate bare ground from a local low percentile around the footprint."""

        row, column = self._fractional_index(longitude, latitude)
        center_row = int(round(row))
        center_column = int(round(column))
        row0 = max(0, center_row - neighborhood)
        row1 = min(self.side, center_row + neighborhood + 1)
        column0 = max(0, center_column - neighborhood)
        column1 = min(self.side, center_column + neighborhood + 1)
        values = self.data[row0:row1, column0:column1].astype(np.float64)
        valid = values[values > -32_000]
        if not valid.size:
            raise BuildError(
                f"DEM void 주변 좌표: lon={longitude}, lat={latitude}"
            )
        return float(np.percentile(valid, percentile))


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path: Path, document: Any, compact: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".part")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(
            document,
            stream,
            ensure_ascii=False,
            indent=None if compact else 2,
            separators=(",", ":") if compact else None,
        )
        stream.write("\n")
    temporary.replace(path)


def write_gzip(source: Path, destination: Path) -> None:
    temporary = destination.with_suffix(destination.suffix + ".part")
    with source.open("rb") as input_stream, temporary.open("wb") as raw_output:
        with gzip.GzipFile(
            filename="",
            mode="wb",
            compresslevel=9,
            fileobj=raw_output,
            mtime=0,
        ) as output_stream:
            while chunk := input_stream.read(1024 * 1024):
                output_stream.write(chunk)
    temporary.replace(destination)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def configure_proj_data() -> None:
    if SYSTEM_PROJ_DATA.is_dir():
        pyproj.datadir.append_data_dir(str(SYSTEM_PROJ_DATA))


def make_vertical_transformer() -> Transformer:
    configure_proj_data()
    transformer = Transformer.from_crs(
        "EPSG:9707",
        "EPSG:4979",
        always_xy=True,
        only_best=True,
    )
    test_lon, test_lat, test_orthometric = 126.86519, 37.60025, 0.0
    _, _, test_ellipsoid = transformer.transform(
        test_lon, test_lat, test_orthometric
    )
    if abs(test_ellipsoid - test_orthometric) < 1.0:
        raise BuildError(
            "EGM96 지오이드 변환이 적용되지 않았습니다. "
            "/usr/share/proj/egm96_15.gtx를 확인하세요."
        )
    return transformer


def polygons_from_geometry(geometry: Any) -> list[Polygon]:
    candidate = shape(geometry)
    if candidate.is_empty:
        return []
    if not candidate.is_valid:
        candidate = make_valid(candidate)
    if isinstance(candidate, Polygon):
        return [orient(candidate, sign=1.0)]
    if isinstance(candidate, MultiPolygon):
        return [
            orient(polygon, sign=1.0)
            for polygon in candidate.geoms
            if not polygon.is_empty
        ]
    polygons: list[Polygon] = []
    if hasattr(candidate, "geoms"):
        for child in candidate.geoms:
            if isinstance(child, Polygon) and not child.is_empty:
                polygons.append(orient(child, sign=1.0))
            elif isinstance(child, MultiPolygon):
                polygons.extend(
                    orient(polygon, sign=1.0)
                    for polygon in child.geoms
                    if not polygon.is_empty
                )
    return polygons


def local_east_north(
    longitude: float,
    latitude: float,
    center_lon: float,
    center_lat: float,
) -> tuple[float, float]:
    east = (
        math.radians(longitude - center_lon)
        * WGS84_SEMI_MAJOR_M
        * math.cos(math.radians(center_lat))
    )
    north = math.radians(latitude - center_lat) * WGS84_SEMI_MAJOR_M
    return east, north


def tile_index(
    longitude: float,
    latitude: float,
    center_lon: float,
    center_lat: float,
    radius_m: float,
    tile_size_m: float,
) -> tuple[int, int]:
    east, north = local_east_north(
        longitude, latitude, center_lon, center_lat
    )
    column = math.floor((east + radius_m) / tile_size_m)
    row = math.floor((north + radius_m) / tile_size_m)
    return int(column), int(row)


def build_tile_frame(
    longitude: float,
    latitude: float,
    ecef_transformer: Transformer,
) -> TileFrame:
    lon_rad = math.radians(longitude)
    lat_rad = math.radians(latitude)
    sin_lon, cos_lon = math.sin(lon_rad), math.cos(lon_rad)
    sin_lat, cos_lat = math.sin(lat_rad), math.cos(lat_rad)
    origin = np.asarray(
        ecef_transformer.transform(longitude, latitude, 0.0),
        dtype=np.float64,
    )
    east = np.asarray([-sin_lon, cos_lon, 0.0], dtype=np.float64)
    north = np.asarray(
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        dtype=np.float64,
    )
    up = np.asarray(
        [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        dtype=np.float64,
    )
    ecef_to_enu = np.vstack((east, north, up))
    transform = [
        float(east[0]),
        float(east[1]),
        float(east[2]),
        0.0,
        float(north[0]),
        float(north[1]),
        float(north[2]),
        0.0,
        float(up[0]),
        float(up[1]),
        float(up[2]),
        0.0,
        float(origin[0]),
        float(origin[1]),
        float(origin[2]),
        1.0,
    ]
    return TileFrame(
        longitude=longitude,
        latitude=latitude,
        origin_ecef=origin,
        ecef_to_enu=ecef_to_enu,
        enu_to_ecef_transform=transform,
    )


def coordinates_without_closure(
    coordinates: Iterable[Sequence[float]],
) -> np.ndarray:
    array = np.asarray(
        [[float(point[0]), float(point[1])] for point in coordinates],
        dtype=np.float64,
    )
    if len(array) >= 2 and np.allclose(array[0], array[-1]):
        array = array[:-1]
    return array


def polygon_rings(polygon: Polygon) -> list[np.ndarray]:
    rings = [coordinates_without_closure(polygon.exterior.coords)]
    rings.extend(
        coordinates_without_closure(interior.coords)
        for interior in polygon.interiors
    )
    return [ring for ring in rings if len(ring) >= 3]


def geodetic_to_gltf(
    lon_lat: np.ndarray,
    height_m: float,
    frame: TileFrame,
    ecef_transformer: Transformer,
) -> tuple[np.ndarray, np.ndarray]:
    heights = np.full(len(lon_lat), height_m, dtype=np.float64)
    x, y, z = ecef_transformer.transform(
        lon_lat[:, 0],
        lon_lat[:, 1],
        heights,
    )
    ecef = np.column_stack((x, y, z))
    enu = (frame.ecef_to_enu @ (ecef - frame.origin_ecef).T).T
    # 3D Tiles converts glTF Y-up to tile Z-up as (x, y, z)->(x,-z,y).
    # Encoding (east, up, -north) therefore produces tile-local ENU.
    gltf = np.column_stack((enu[:, 0], enu[:, 2], -enu[:, 1]))
    return gltf.astype(np.float32), enu


def add_polygon_mesh(
    polygon: Polygon,
    base_height_m: float,
    roof_height_m: float,
    frame: TileFrame,
    ecef_transformer: Transformer,
    position_parts: list[np.ndarray],
    face_parts: list[np.ndarray],
    vertex_offset: int,
) -> int:
    rings = polygon_rings(polygon)
    if not rings:
        return vertex_offset
    ring_lengths = np.asarray([len(ring) for ring in rings], dtype=np.uint32)
    ring_ends = np.cumsum(ring_lengths, dtype=np.uint32)
    lon_lat = np.vstack(rings)
    roof_gltf, roof_enu = geodetic_to_gltf(
        lon_lat, roof_height_m, frame, ecef_transformer
    )
    horizontal = np.ascontiguousarray(roof_enu[:, :2], dtype=np.float64)
    try:
        roof_indices = mapbox_earcut.triangulate_float64(
            horizontal,
            ring_ends,
        ).reshape(-1, 3)
    except Exception as error:
        raise BuildError(f"지붕 삼각분할 실패: {error}") from error
    if len(roof_indices):
        a = horizontal[roof_indices[:, 0]]
        b = horizontal[roof_indices[:, 1]]
        c = horizontal[roof_indices[:, 2]]
        signed_area = (
            (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1])
            - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
        )
        flip = signed_area < 0.0
        roof_indices[flip, 1], roof_indices[flip, 2] = (
            roof_indices[flip, 2].copy(),
            roof_indices[flip, 1].copy(),
        )
        position_parts.append(roof_gltf)
        face_parts.append(roof_indices.astype(np.uint32) + vertex_offset)
        vertex_offset += len(roof_gltf)

    start = 0
    for ring, ring_length in zip(rings, ring_lengths, strict=True):
        count = int(ring_length)
        base_gltf, _ = geodetic_to_gltf(
            ring, base_height_m, frame, ecef_transformer
        )
        top_gltf, _ = geodetic_to_gltf(
            ring, roof_height_m, frame, ecef_transformer
        )
        wall_positions = np.empty((count * 4, 3), dtype=np.float32)
        wall_faces = np.empty((count * 2, 3), dtype=np.uint32)
        for index in range(count):
            next_index = (index + 1) % count
            base = index * 4
            wall_positions[base : base + 4] = (
                base_gltf[index],
                base_gltf[next_index],
                top_gltf[next_index],
                top_gltf[index],
            )
            wall_faces[index * 2] = (
                vertex_offset + base,
                vertex_offset + base + 1,
                vertex_offset + base + 2,
            )
            wall_faces[index * 2 + 1] = (
                vertex_offset + base,
                vertex_offset + base + 2,
                vertex_offset + base + 3,
            )
        position_parts.append(wall_positions)
        face_parts.append(wall_faces)
        vertex_offset += len(wall_positions)
        start += count
    return vertex_offset


def compute_vertex_normals(
    positions: np.ndarray, faces: np.ndarray
) -> np.ndarray:
    triangles = positions[faces]
    face_normals = np.cross(
        triangles[:, 1] - triangles[:, 0],
        triangles[:, 2] - triangles[:, 0],
    )
    lengths = np.linalg.norm(face_normals, axis=1)
    valid = lengths > 1e-12
    face_normals[valid] /= lengths[valid, None]
    normals = np.zeros_like(positions, dtype=np.float64)
    for corner in range(3):
        np.add.at(normals, faces[:, corner], face_normals)
    normal_lengths = np.linalg.norm(normals, axis=1)
    valid = normal_lengths > 1e-12
    normals[valid] /= normal_lengths[valid, None]
    return normals.astype(np.float32)


def padded_bytes(data: bytes, padding: bytes) -> bytes:
    remainder = len(data) % 4
    if remainder:
        data += padding * (4 - remainder)
    return data


def write_glb(
    path: Path,
    positions: np.ndarray,
    normals: np.ndarray,
    faces: np.ndarray,
) -> None:
    positions = np.ascontiguousarray(positions, dtype="<f4")
    normals = np.ascontiguousarray(normals, dtype="<f4")
    indices = np.ascontiguousarray(faces.reshape(-1), dtype="<u4")
    chunks: list[bytes] = []
    buffer_views: list[dict[str, Any]] = []

    def add_buffer_view(data: bytes, target: int) -> int:
        offset = sum(len(chunk) for chunk in chunks)
        padded = padded_bytes(data, b"\x00")
        chunks.append(padded)
        index = len(buffer_views)
        buffer_views.append(
            {
                "buffer": 0,
                "byteOffset": offset,
                "byteLength": len(data),
                "target": target,
            }
        )
        return index

    position_view = add_buffer_view(positions.tobytes(), 34962)
    normal_view = add_buffer_view(normals.tobytes(), 34962)
    index_view = add_buffer_view(indices.tobytes(), 34963)
    binary = b"".join(chunks)
    document = {
        "asset": {
            "version": "2.0",
            "generator": "KAU LoD1 building builder",
        },
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "meshes": [
            {
                "primitives": [
                    {
                        "attributes": {"POSITION": 0, "NORMAL": 1},
                        "indices": 2,
                        "material": 0,
                        "mode": 4,
                    }
                ]
            }
        ],
        "materials": [
            {
                "name": "KAU_LoD1_neutral",
                "pbrMetallicRoughness": {
                    "baseColorFactor": [0.58, 0.61, 0.65, 1.0],
                    "metallicFactor": 0.0,
                    "roughnessFactor": 0.86,
                },
                "doubleSided": False,
            }
        ],
        "buffers": [{"byteLength": len(binary)}],
        "bufferViews": buffer_views,
        "accessors": [
            {
                "bufferView": position_view,
                "byteOffset": 0,
                "componentType": 5126,
                "count": len(positions),
                "type": "VEC3",
                "min": [float(value) for value in positions.min(axis=0)],
                "max": [float(value) for value in positions.max(axis=0)],
            },
            {
                "bufferView": normal_view,
                "byteOffset": 0,
                "componentType": 5126,
                "count": len(normals),
                "type": "VEC3",
            },
            {
                "bufferView": index_view,
                "byteOffset": 0,
                "componentType": 5125,
                "count": len(indices),
                "type": "SCALAR",
                "min": [int(indices.min())],
                "max": [int(indices.max())],
            },
        ],
    }
    json_chunk = padded_bytes(
        json.dumps(document, separators=(",", ":")).encode("utf-8"),
        b" ",
    )
    total_length = 12 + 8 + len(json_chunk) + 8 + len(binary)
    glb = (
        struct.pack("<4sII", b"glTF", 2, total_length)
        + struct.pack("<I4s", len(json_chunk), b"JSON")
        + json_chunk
        + struct.pack("<I4s", len(binary), b"BIN\x00")
        + binary
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".glb.part")
    temporary.write_bytes(glb)
    temporary.replace(path)


def radians_region(
    west: float,
    south: float,
    east: float,
    north: float,
    minimum_height: float,
    maximum_height: float,
) -> list[float]:
    return [
        math.radians(west),
        math.radians(south),
        math.radians(east),
        math.radians(north),
        float(minimum_height),
        float(maximum_height),
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="항공대 반경 5 km 건물의 지면고도 결합 및 LoD1 3D Tiles 생성"
    )
    parser.add_argument("--buildings", type=Path, default=DEFAULT_BUILDINGS)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--dem", type=Path, default=DEFAULT_DEM)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--enriched-geojson", type=Path, default=DEFAULT_ENRICHED)
    parser.add_argument("--tile-size-m", type=float, default=500.0)
    parser.add_argument(
        "--max-features",
        type=int,
        default=0,
        help="개발 검사용 처리 상한, 0이면 전량",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    started_at = datetime.now(timezone.utc)
    started_clock = time.monotonic()
    buildings_path = args.buildings.resolve()
    profile = load_json(args.profile.resolve())
    source = load_json(buildings_path)
    features = source.get("features")
    if not isinstance(features, list):
        raise BuildError("입력 GeoJSON에 features 배열이 없습니다.")
    if args.max_features > 0:
        features = features[: args.max_features]
        source["features"] = features

    center_lon = float(profile["center"]["longitude"])
    center_lat = float(profile["center"]["latitude"])
    radius_m = float(profile["radius_m"])
    tile_size_m = float(args.tile_size_m)
    dem = HgtDem(args.dem.resolve())
    vertical_transformer = make_vertical_transformer()
    ecef_transformer = Transformer.from_crs(
        "EPSG:4979", "EPSG:4978", always_xy=True
    )

    representative_longitudes: list[float] = []
    representative_latitudes: list[float] = []
    orthometric_heights: list[float] = []
    valid_features: list[dict[str, Any]] = []
    repaired_count = 0
    skipped_geometry_count = 0

    for index, feature in enumerate(features, start=1):
        geometry = feature.get("geometry") if isinstance(feature, dict) else None
        if not isinstance(geometry, dict):
            skipped_geometry_count += 1
            continue
        original_geometry = shape(geometry)
        if not original_geometry.is_valid:
            repaired_count += 1
        polygons = polygons_from_geometry(geometry)
        if not polygons:
            skipped_geometry_count += 1
            continue
        largest = max(polygons, key=lambda polygon: polygon.area)
        point = largest.representative_point()
        longitude, latitude = float(point.x), float(point.y)
        ground = dem.sample_ground(longitude, latitude)
        representative_longitudes.append(longitude)
        representative_latitudes.append(latitude)
        orthometric_heights.append(ground)
        valid_features.append(feature)
        if index % 10_000 == 0:
            print(f"[고도] {index:,}/{len(features):,} 건물 DEM 샘플링", flush=True)

    lon_array = np.asarray(representative_longitudes, dtype=np.float64)
    lat_array = np.asarray(representative_latitudes, dtype=np.float64)
    orthometric_array = np.asarray(orthometric_heights, dtype=np.float64)
    _, _, ellipsoid_array = vertical_transformer.transform(
        lon_array,
        lat_array,
        orthometric_array,
    )
    ellipsoid_array = np.asarray(ellipsoid_array, dtype=np.float64)
    geoid_array = ellipsoid_array - orthometric_array
    if not np.all(np.isfinite(ellipsoid_array)):
        raise BuildError("비정상 타원체고도가 생성되었습니다.")

    grouped: dict[tuple[int, int], list[int]] = defaultdict(list)
    height_sources: dict[str, int] = defaultdict(int)
    roof_heights: list[float] = []
    for feature_index, feature in enumerate(valid_features):
        properties = feature.setdefault("properties", {})
        derived = properties.setdefault("_kau5km", {})
        building_height = float(derived["height_final_m"])
        base_height = float(ellipsoid_array[feature_index])
        roof_height = base_height + building_height
        derived.update(
            {
                "terrain_elevation_orthometric_m": round(
                    float(orthometric_array[feature_index]), 3
                ),
                "geoid_separation_m": round(float(geoid_array[feature_index]), 3),
                "base_elevation_m": round(base_height, 3),
                "roof_elevation_m": round(roof_height, 3),
                "base_vertical_datum": "WGS84 ellipsoid",
                "terrain_source": "Mapzen/AWS Terrain Tiles SRTM 1 arc-second",
                "terrain_sample_method": "5x5 neighborhood p25 at footprint representative point",
            }
        )
        key = tile_index(
            representative_longitudes[feature_index],
            representative_latitudes[feature_index],
            center_lon,
            center_lat,
            radius_m,
            tile_size_m,
        )
        grouped[key].append(feature_index)
        height_sources[str(derived.get("height_source", "unknown"))] += 1
        roof_heights.append(roof_height)

    output_dir = args.output_dir.resolve()
    tiles_dir = output_dir / "tiles"
    tiles_dir.mkdir(parents=True, exist_ok=True)
    tile_entries: list[dict[str, Any]] = []
    tile_reports: list[dict[str, Any]] = []
    total_vertices = 0
    total_triangles = 0
    all_west = float("inf")
    all_south = float("inf")
    all_east = float("-inf")
    all_north = float("-inf")
    all_min_height = float("inf")
    all_max_height = float("-inf")

    sorted_groups = sorted(grouped.items())
    for tile_number, ((column, row), feature_indices) in enumerate(
        sorted_groups, start=1
    ):
        tile_features = [valid_features[index] for index in feature_indices]
        bounds = [
            shape(feature["geometry"]).bounds
            for feature in tile_features
        ]
        west = min(bound[0] for bound in bounds)
        south = min(bound[1] for bound in bounds)
        east = max(bound[2] for bound in bounds)
        north = max(bound[3] for bound in bounds)
        tile_lon = (west + east) / 2.0
        tile_lat = (south + north) / 2.0
        frame = build_tile_frame(tile_lon, tile_lat, ecef_transformer)
        position_parts: list[np.ndarray] = []
        face_parts: list[np.ndarray] = []
        vertex_offset = 0
        minimum_height = float("inf")
        maximum_height = float("-inf")
        polygon_count = 0

        for feature_index in feature_indices:
            feature = valid_features[feature_index]
            derived = feature["properties"]["_kau5km"]
            base_height = float(derived["base_elevation_m"])
            roof_height = float(derived["roof_elevation_m"])
            for polygon in polygons_from_geometry(feature["geometry"]):
                vertex_offset = add_polygon_mesh(
                    polygon,
                    base_height,
                    roof_height,
                    frame,
                    ecef_transformer,
                    position_parts,
                    face_parts,
                    vertex_offset,
                )
                polygon_count += 1
            minimum_height = min(minimum_height, base_height)
            maximum_height = max(maximum_height, roof_height)

        if not position_parts or not face_parts:
            raise BuildError(f"타일 {column},{row}에서 메시가 생성되지 않았습니다.")
        positions = np.vstack(position_parts)
        faces = np.vstack(face_parts)
        normals = compute_vertex_normals(positions, faces)
        filename = f"tile_{column:+03d}_{row:+03d}.glb".replace("+", "p").replace(
            "-", "m"
        )
        relative_uri = f"tiles/{filename}"
        write_glb(tiles_dir / filename, positions, normals, faces)
        region = radians_region(
            west,
            south,
            east,
            north,
            minimum_height,
            maximum_height,
        )
        tile_entries.append(
            {
                "boundingVolume": {"region": region},
                "geometricError": 0.0,
                "transform": frame.enu_to_ecef_transform,
                "content": {"uri": relative_uri},
            }
        )
        tile_reports.append(
            {
                "tile": [column, row],
                "uri": relative_uri,
                "buildings": len(feature_indices),
                "polygons": polygon_count,
                "vertices": int(len(positions)),
                "triangles": int(len(faces)),
                "bytes": (tiles_dir / filename).stat().st_size,
                "region_degrees": [
                    west,
                    south,
                    east,
                    north,
                    minimum_height,
                    maximum_height,
                ],
            }
        )
        total_vertices += len(positions)
        total_triangles += len(faces)
        all_west = min(all_west, west)
        all_south = min(all_south, south)
        all_east = max(all_east, east)
        all_north = max(all_north, north)
        all_min_height = min(all_min_height, minimum_height)
        all_max_height = max(all_max_height, maximum_height)
        if tile_number % 25 == 0 or tile_number == len(sorted_groups):
            print(
                f"[메시] {tile_number:,}/{len(sorted_groups):,} 타일, "
                f"정점 {total_vertices:,}, 삼각형 {total_triangles:,}",
                flush=True,
            )

    root_region = radians_region(
        all_west,
        all_south,
        all_east,
        all_north,
        all_min_height,
        all_max_height,
    )
    tileset = {
        "asset": {
            "version": "1.1",
            "tilesetVersion": "kau-5km-lod1-20260723",
            "gltfUpAxis": "Y",
            "extras": {
                "buildingSource": "MOLIT/VWorld GIS Building Integrated Information",
                "terrainSource": "Mapzen/AWS Terrain Tiles SRTM 1 arc-second",
                "groundVerticalDatum": "WGS84 ellipsoid after EGM96 conversion",
            },
        },
        "geometricError": 5_000.0,
        "root": {
            "boundingVolume": {"region": root_region},
            "geometricError": 5_000.0,
            "refine": "ADD",
            "children": tile_entries,
        },
    }
    write_json(output_dir / "tileset.json", tileset)
    source["features"] = valid_features
    enriched_path = args.enriched_geojson.resolve()
    write_json(enriched_path, source, compact=True)
    write_gzip(enriched_path, enriched_path.with_suffix(".geojson.gz"))

    finished_at = datetime.now(timezone.utc)
    glb_bytes = sum(report["bytes"] for report in tile_reports)
    report = {
        "started_at_utc": started_at.isoformat(),
        "finished_at_utc": finished_at.isoformat(),
        "elapsed_seconds": round(time.monotonic() - started_clock, 3),
        "input_buildings": len(features),
        "output_buildings": len(valid_features),
        "skipped_geometry_count": skipped_geometry_count,
        "repaired_geometry_count": repaired_count,
        "tile_count": len(tile_entries),
        "tile_size_m": tile_size_m,
        "total_vertices": int(total_vertices),
        "total_triangles": int(total_triangles),
        "total_glb_bytes": int(glb_bytes),
        "height_source_counts": dict(sorted(height_sources.items())),
        "terrain": {
            "file": str(args.dem.resolve()),
            "sha256": sha256_file(args.dem.resolve()),
            "resolution_samples": dem.side,
            "orthometric_min_m": round(float(orthometric_array.min()), 3),
            "orthometric_max_m": round(float(orthometric_array.max()), 3),
            "ellipsoid_min_m": round(float(ellipsoid_array.min()), 3),
            "ellipsoid_max_m": round(float(ellipsoid_array.max()), 3),
            "geoid_separation_min_m": round(float(geoid_array.min()), 3),
            "geoid_separation_max_m": round(float(geoid_array.max()), 3),
        },
        "roof_ellipsoid_height_m": {
            "minimum": round(float(min(roof_heights)), 3),
            "maximum": round(float(max(roof_heights)), 3),
        },
        "root_region_degrees": [
            all_west,
            all_south,
            all_east,
            all_north,
            all_min_height,
            all_max_height,
        ],
        "outputs": {
            "tileset": str(output_dir / "tileset.json"),
            "tiles": str(tiles_dir),
            "enriched_geojson": str(enriched_path),
            "enriched_geojson_gzip": str(
                enriched_path.with_suffix(".geojson.gz")
            ),
        },
        "tiles": tile_reports,
    }
    write_json(output_dir / "build_report.json", report)
    checksum_files = [
        output_dir / "tileset.json",
        output_dir / "build_report.json",
        enriched_path,
        enriched_path.with_suffix(".geojson.gz"),
    ]
    checksums = output_dir / "checksums.sha256"
    checksum_text = "".join(
        f"{sha256_file(path)}  {os.path.relpath(path, output_dir)}\n"
        for path in checksum_files
    )
    temporary_checksums = checksums.with_suffix(".sha256.part")
    temporary_checksums.write_text(checksum_text, encoding="ascii")
    temporary_checksums.replace(checksums)

    print(
        f"[완료] {len(valid_features):,}개 건물, {len(tile_entries):,}개 GLB 타일, "
        f"{total_triangles:,}개 삼각형",
        flush=True,
    )
    print(f"[완료] tileset: {output_dir / 'tileset.json'}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BuildError as error:
        print(f"[오류] {error}", file=sys.stderr)
        raise SystemExit(2)
