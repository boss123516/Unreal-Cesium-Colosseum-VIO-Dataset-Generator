#!/usr/bin/env python3
"""Download and validate VWorld buildings intersecting a 5 km circle around KAU.

Only the Python standard library is required. The VWorld API key is read from
the environment and is never written to a request manifest or cache filename.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import os
import re
import statistics
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import deque
from concurrent.futures import FIRST_COMPLETED, Future, ThreadPoolExecutor, wait
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Iterator, Sequence


EARTH_RADIUS_M = 6_378_137.0
DEFAULT_PROFILE_PATH = Path(__file__).with_name("profile.json")
DEFAULT_OUTPUT_DIR = Path(__file__).with_name("output")
DEFAULT_API_KEY_FILE = Path(__file__).with_name(".secret") / "vworld_api_key"
USER_AGENT = "KAU-VIO-research-building-fetcher/1.0"


class CollectionError(RuntimeError):
    """Raised when the collector cannot guarantee a complete result."""


@dataclass(frozen=True)
class LocalTile:
    """Axis-aligned tile in local east/north metres relative to the profile center."""

    west_m: float
    south_m: float
    east_m: float
    north_m: float
    depth: int = 0

    @property
    def width_m(self) -> float:
        return self.east_m - self.west_m

    @property
    def height_m(self) -> float:
        return self.north_m - self.south_m

    def intersects_disk(self, radius_m: float) -> bool:
        nearest_x = min(max(0.0, self.west_m), self.east_m)
        nearest_y = min(max(0.0, self.south_m), self.north_m)
        return nearest_x * nearest_x + nearest_y * nearest_y <= radius_m * radius_m

    def subdivide(self) -> tuple["LocalTile", "LocalTile", "LocalTile", "LocalTile"]:
        mid_x = (self.west_m + self.east_m) / 2.0
        mid_y = (self.south_m + self.north_m) / 2.0
        next_depth = self.depth + 1
        return (
            LocalTile(self.west_m, self.south_m, mid_x, mid_y, next_depth),
            LocalTile(mid_x, self.south_m, self.east_m, mid_y, next_depth),
            LocalTile(self.west_m, mid_y, mid_x, self.north_m, next_depth),
            LocalTile(mid_x, mid_y, self.east_m, self.north_m, next_depth),
        )

    def cache_id(self) -> str:
        coordinates = ",".join(
            f"{value:.3f}"
            for value in (self.west_m, self.south_m, self.east_m, self.north_m)
        )
        digest = hashlib.sha1(coordinates.encode("ascii")).hexdigest()[:12]
        return f"d{self.depth}_{digest}"


def load_profile(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        profile = json.load(stream)
    required = ("center", "radius_m", "source", "collection", "height_policy")
    missing = [name for name in required if name not in profile]
    if missing:
        raise CollectionError(f"프로필 필수 항목 누락: {', '.join(missing)}")
    return profile


def local_to_lon_lat(
    east_m: float, north_m: float, center_lon: float, center_lat: float
) -> tuple[float, float]:
    latitude = center_lat + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = center_lon + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(center_lat)))
    )
    return longitude, latitude


def lon_lat_to_local(
    longitude: float, latitude: float, center_lon: float, center_lat: float
) -> tuple[float, float]:
    east_m = (
        math.radians(longitude - center_lon)
        * EARTH_RADIUS_M
        * math.cos(math.radians(center_lat))
    )
    north_m = math.radians(latitude - center_lat) * EARTH_RADIUS_M
    return east_m, north_m


def tile_bbox_wfs(
    tile: LocalTile, center_lon: float, center_lat: float
) -> tuple[float, float, float, float]:
    """Return the EPSG:4326 WFS 1.1 bbox axis order: ymin,xmin,ymax,xmax."""

    west, south = local_to_lon_lat(
        tile.west_m, tile.south_m, center_lon, center_lat
    )
    east, north = local_to_lon_lat(
        tile.east_m, tile.north_m, center_lon, center_lat
    )
    return south, west, north, east


def make_initial_tiles(radius_m: float, tile_size_m: float) -> list[LocalTile]:
    if tile_size_m <= 0.0:
        raise CollectionError("tile_size_m은 0보다 커야 합니다.")
    side_count = math.ceil((2.0 * radius_m) / tile_size_m)
    tiles: list[LocalTile] = []
    for row in range(side_count):
        south = -radius_m + row * tile_size_m
        north = min(south + tile_size_m, radius_m)
        for column in range(side_count):
            west = -radius_m + column * tile_size_m
            east = min(west + tile_size_m, radius_m)
            tile = LocalTile(west, south, east, north)
            if tile.intersects_disk(radius_m):
                tiles.append(tile)
    return tiles


def build_request_url(
    endpoint: str,
    api_key: str,
    typename: str,
    bbox: tuple[float, float, float, float],
    max_features: int,
    domain: str | None,
) -> str:
    params = {
        "service": "WFS",
        "version": "1.1.0",
        "request": "GetFeature",
        "key": api_key,
        "typename": typename,
        "bbox": ",".join(f"{coordinate:.10f}" for coordinate in bbox),
        "srsname": "EPSG:4326",
        "output": "application/json",
        "maxfeatures": str(max_features),
    }
    if domain:
        params["domain"] = domain
    return f"{endpoint}?{urllib.parse.urlencode(params)}"


def service_exception_message(body: bytes) -> str | None:
    text = body.decode("utf-8", errors="replace")
    if "ServiceException" not in text and "<ows:Exception" not in text:
        return None
    match = re.search(
        r"<(?:\\w+:)?(?:ServiceException|ExceptionText)[^>]*>(.*?)</",
        text,
        flags=re.DOTALL,
    )
    if match:
        return re.sub(r"\\s+", " ", match.group(1)).strip()
    return "브이월드가 서비스 예외 응답을 반환했습니다."


def request_json(url: str, timeout_s: float, retries: int) -> dict[str, Any]:
    request = urllib.request.Request(
        url,
        headers={
            "Accept": "application/json",
            "User-Agent": USER_AGENT,
        },
    )
    last_error: Exception | None = None
    for attempt in range(retries + 1):
        try:
            with urllib.request.urlopen(request, timeout=timeout_s) as response:
                body = response.read()
            exception = service_exception_message(body)
            if exception:
                raise CollectionError(exception)
            document = json.loads(body.decode("utf-8"))
            if not isinstance(document, dict):
                raise CollectionError("WFS 응답 최상위 값이 JSON 객체가 아닙니다.")
            return document
        except CollectionError:
            raise
        except urllib.error.HTTPError as error:
            last_error = CollectionError(f"WFS HTTP 오류: {error.code}")
            retryable = error.code == 429 or 500 <= error.code < 600
            if not retryable:
                break
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as error:
            last_error = error
        if attempt < retries:
            time.sleep(min(2**attempt, 8))
    raise CollectionError(
        f"WFS 요청이 {retries + 1}회 모두 실패했습니다: "
        f"{type(last_error).__name__ if last_error else 'unknown error'}"
    )


class VWorldClient:
    def __init__(
        self,
        endpoint: str,
        api_key: str,
        typename: str,
        center_lon: float,
        center_lat: float,
        max_features: int,
        cache_dir: Path,
        timeout_s: float,
        retries: int,
        domain: str | None,
    ) -> None:
        self.endpoint = endpoint
        self.api_key = api_key
        self.typename = typename
        self.center_lon = center_lon
        self.center_lat = center_lat
        self.max_features = max_features
        self.cache_dir = cache_dir
        self.timeout_s = timeout_s
        self.retries = retries
        self.domain = domain

    def fetch_tile(self, tile: LocalTile) -> tuple[LocalTile, dict[str, Any], bool]:
        cache_path = self.cache_dir / f"{tile.cache_id()}.json"
        if cache_path.exists():
            try:
                with cache_path.open("r", encoding="utf-8") as stream:
                    cached = json.load(stream)
                if isinstance(cached, dict) and isinstance(
                    cached.get("features"), list
                ):
                    return tile, cached, True
            except (OSError, json.JSONDecodeError):
                pass

        bbox = tile_bbox_wfs(tile, self.center_lon, self.center_lat)
        url = build_request_url(
            self.endpoint,
            self.api_key,
            self.typename,
            bbox,
            self.max_features,
            self.domain,
        )
        document = request_json(url, self.timeout_s, self.retries)
        if not isinstance(document.get("features"), list):
            raise CollectionError(
                f"{tile.cache_id()}: 응답에 GeoJSON features 배열이 없습니다."
            )
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        temporary_path = cache_path.with_suffix(".json.part")
        with temporary_path.open("w", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, separators=(",", ":"))
        temporary_path.replace(cache_path)
        return tile, document, False


def response_is_truncated(document: dict[str, Any], limit: int) -> bool:
    features = document.get("features", [])
    returned = len(features) if isinstance(features, list) else 0
    for field in ("numberMatched", "totalFeatures"):
        value = document.get(field)
        try:
            if value not in (None, "unknown") and int(value) > returned:
                return True
        except (TypeError, ValueError):
            continue
    return returned >= limit


def iter_coordinate_pairs(value: Any) -> Iterator[Sequence[float]]:
    if (
        isinstance(value, (list, tuple))
        and len(value) >= 2
        and isinstance(value[0], (int, float))
        and isinstance(value[1], (int, float))
    ):
        yield value
        return
    if isinstance(value, (list, tuple)):
        for child in value:
            yield from iter_coordinate_pairs(child)


def swap_coordinate_axes(value: Any) -> Any:
    if (
        isinstance(value, list)
        and len(value) >= 2
        and isinstance(value[0], (int, float))
        and isinstance(value[1], (int, float))
    ):
        return [value[1], value[0], *value[2:]]
    if isinstance(value, list):
        return [swap_coordinate_axes(child) for child in value]
    return value


def normalize_geometry_axis(
    geometry: dict[str, Any], center_lon: float, center_lat: float
) -> tuple[dict[str, Any], bool]:
    if geometry.get("type") == "GeometryCollection":
        geometries = geometry.get("geometries", [])
        normalized = []
        swapped_any = False
        for child in geometries:
            if isinstance(child, dict):
                fixed, swapped = normalize_geometry_axis(
                    child, center_lon, center_lat
                )
                normalized.append(fixed)
                swapped_any = swapped_any or swapped
        copy = dict(geometry)
        copy["geometries"] = normalized
        return copy, swapped_any

    coordinates = geometry.get("coordinates")
    first = next(iter_coordinate_pairs(coordinates), None)
    if first is None:
        return geometry, False
    x, y = float(first[0]), float(first[1])
    normal_error = abs(x - center_lon) + abs(y - center_lat)
    swapped_error = abs(y - center_lon) + abs(x - center_lat)
    if swapped_error + 1e-9 < normal_error:
        copy = dict(geometry)
        copy["coordinates"] = swap_coordinate_axes(coordinates)
        return copy, True
    return geometry, False


def point_in_ring(point: tuple[float, float], ring: Sequence[tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    if len(ring) < 3:
        return False
    previous_x, previous_y = ring[-1]
    for current_x, current_y in ring:
        if (current_y > y) != (previous_y > y):
            intersection_x = (
                (previous_x - current_x)
                * (y - current_y)
                / (previous_y - current_y)
                + current_x
            )
            if x < intersection_x:
                inside = not inside
        previous_x, previous_y = current_x, current_y
    return inside


def segment_distance_squared_to_origin(
    start: tuple[float, float], end: tuple[float, float]
) -> float:
    start_x, start_y = start
    delta_x = end[0] - start_x
    delta_y = end[1] - start_y
    length_squared = delta_x * delta_x + delta_y * delta_y
    if length_squared == 0.0:
        return start_x * start_x + start_y * start_y
    fraction = -(start_x * delta_x + start_y * delta_y) / length_squared
    fraction = min(1.0, max(0.0, fraction))
    nearest_x = start_x + fraction * delta_x
    nearest_y = start_y + fraction * delta_y
    return nearest_x * nearest_x + nearest_y * nearest_y


def local_ring(
    coordinates: Sequence[Sequence[float]],
    center_lon: float,
    center_lat: float,
) -> list[tuple[float, float]]:
    return [
        lon_lat_to_local(float(point[0]), float(point[1]), center_lon, center_lat)
        for point in coordinates
        if len(point) >= 2
    ]


def polygon_intersects_circle(
    polygon: Sequence[Sequence[Sequence[float]]],
    center_lon: float,
    center_lat: float,
    radius_m: float,
) -> bool:
    if not polygon:
        return False
    radius_squared = radius_m * radius_m
    rings = [local_ring(ring, center_lon, center_lat) for ring in polygon]
    for ring in rings:
        if not ring:
            continue
        if any(x * x + y * y <= radius_squared for x, y in ring):
            return True
        previous = ring[-1]
        for current in ring:
            if segment_distance_squared_to_origin(previous, current) <= radius_squared:
                return True
            previous = current

    exterior_contains_center = point_in_ring((0.0, 0.0), rings[0])
    center_in_hole = any(point_in_ring((0.0, 0.0), ring) for ring in rings[1:])
    return exterior_contains_center and not center_in_hole


def geometry_intersects_circle(
    geometry: dict[str, Any],
    center_lon: float,
    center_lat: float,
    radius_m: float,
) -> bool:
    geometry_type = geometry.get("type")
    coordinates = geometry.get("coordinates")
    if geometry_type == "Polygon" and isinstance(coordinates, list):
        return polygon_intersects_circle(
            coordinates, center_lon, center_lat, radius_m
        )
    if geometry_type == "MultiPolygon" and isinstance(coordinates, list):
        return any(
            polygon_intersects_circle(polygon, center_lon, center_lat, radius_m)
            for polygon in coordinates
        )
    if geometry_type == "GeometryCollection":
        return any(
            geometry_intersects_circle(child, center_lon, center_lat, radius_m)
            for child in geometry.get("geometries", [])
            if isinstance(child, dict)
        )
    return False


def properties_casefold(properties: dict[str, Any]) -> dict[str, Any]:
    return {str(key).casefold(): value for key, value in properties.items()}


def feature_key(feature: dict[str, Any]) -> str:
    properties = feature.get("properties")
    folded = properties_casefold(properties) if isinstance(properties, dict) else {}
    for field in ("ufid", "bldrgst_pk", "bd_mgt_sn", "geoidn"):
        value = folded.get(field)
        if value not in (None, ""):
            return f"{field}:{value}"
    feature_id = feature.get("id")
    if feature_id not in (None, ""):
        return f"id:{feature_id}"
    geometry = json.dumps(
        feature.get("geometry"), ensure_ascii=False, sort_keys=True, separators=(",", ":")
    )
    return f"geometry:{hashlib.sha256(geometry.encode('utf-8')).hexdigest()}"


def parse_number(value: Any) -> float | None:
    if value in (None, "") or isinstance(value, bool):
        return None
    try:
        number = float(str(value).replace(",", "").strip())
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def geometry_center_distance_m(
    geometry: dict[str, Any], center_lon: float, center_lat: float
) -> float | None:
    coordinates = list(iter_coordinate_pairs(geometry.get("coordinates")))
    if not coordinates:
        return None
    eastings: list[float] = []
    northings: list[float] = []
    for coordinate in coordinates:
        east, north = lon_lat_to_local(
            float(coordinate[0]), float(coordinate[1]), center_lon, center_lat
        )
        eastings.append(east)
        northings.append(north)
    center_east = (min(eastings) + max(eastings)) / 2.0
    center_north = (min(northings) + max(northings)) / 2.0
    return math.hypot(center_east, center_north)


def annotate_height(
    feature: dict[str, Any],
    policy: dict[str, Any],
    center_lon: float,
    center_lat: float,
) -> tuple[str, float]:
    properties = feature.setdefault("properties", {})
    if not isinstance(properties, dict):
        properties = {}
        feature["properties"] = properties
    folded = properties_casefold(properties)

    measured_field = str(policy["measured_field"]).casefold()
    floor_field = str(policy["floor_field"]).casefold()
    measured = parse_number(folded.get(measured_field))
    floors = parse_number(folded.get(floor_field))
    minimum, maximum = map(float, policy["valid_measured_height_range_m"])

    if measured is not None and minimum <= measured <= maximum:
        source = "measured"
        final_height = measured
    elif floors is not None and 0.0 < floors <= 200.0:
        source = "floors"
        final_height = floors * float(policy["average_floor_height_m"])
    else:
        source = "default"
        final_height = float(policy["default_height_m"])

    geometry = feature.get("geometry")
    distance = (
        geometry_center_distance_m(geometry, center_lon, center_lat)
        if isinstance(geometry, dict)
        else None
    )
    properties["_kau5km"] = {
        "height_source": source,
        "height_raw": measured,
        "floor_count_raw": floors,
        "height_final_m": round(final_height, 3),
        "footprint_center_distance_m": (
            round(distance, 3) if distance is not None else None
        ),
        "source_feature_id": feature_key(feature),
    }
    return source, final_height


def percentile(sorted_values: Sequence[float], percent: float) -> float | None:
    if not sorted_values:
        return None
    index = (len(sorted_values) - 1) * percent
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return sorted_values[lower]
    fraction = index - lower
    return sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction


def safe_round(value: float | None) -> float | None:
    return round(value, 3) if value is not None else None


def collect_features(
    client: VWorldClient,
    initial_tiles: Iterable[LocalTile],
    radius_m: float,
    min_tile_size_m: float,
    workers: int,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    queue: deque[LocalTile] = deque(initial_tiles)
    pending: dict[Future[tuple[LocalTile, dict[str, Any], bool]], LocalTile] = {}
    accepted_features: list[dict[str, Any]] = []
    tile_records: list[dict[str, Any]] = []
    completed_count = 0

    with ThreadPoolExecutor(max_workers=workers) as executor:
        while queue or pending:
            while queue and len(pending) < workers * 2:
                tile = queue.popleft()
                pending[executor.submit(client.fetch_tile, tile)] = tile
            done, _ = wait(pending, return_when=FIRST_COMPLETED)
            for future in done:
                pending.pop(future)
                tile, document, from_cache = future.result()
                features = document["features"]
                truncated = response_is_truncated(document, client.max_features)
                record = {
                    "tile_id": tile.cache_id(),
                    "depth": tile.depth,
                    "local_bbox_m": [
                        tile.west_m,
                        tile.south_m,
                        tile.east_m,
                        tile.north_m,
                    ],
                    "returned_features": len(features),
                    "cache_hit": from_cache,
                    "subdivided": truncated,
                }
                tile_records.append(record)
                if truncated:
                    if min(tile.width_m, tile.height_m) / 2.0 < min_tile_size_m:
                        raise CollectionError(
                            f"{tile.cache_id()}가 최소 타일 크기에서도 "
                            f"{client.max_features}개 제한에 도달했습니다. "
                            "결과 누락 가능성이 있어 출력을 중단합니다."
                        )
                    queue.extend(
                        child
                        for child in tile.subdivide()
                        if child.intersects_disk(radius_m)
                    )
                else:
                    accepted_features.extend(features)
                completed_count += 1
                if completed_count % 20 == 0 or not (queue or pending):
                    print(
                        f"[진행] 요청 타일 {completed_count}개 완료, "
                        f"대기 {len(queue) + len(pending)}개, "
                        f"수집 피처 {len(accepted_features):,}개",
                        flush=True,
                    )
    return accepted_features, tile_records


def process_features(
    raw_features: Iterable[dict[str, Any]],
    profile: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    center_lat = float(profile["center"]["latitude"])
    center_lon = float(profile["center"]["longitude"])
    radius_m = float(profile["radius_m"])
    policy = profile["height_policy"]

    deduplicated: dict[str, dict[str, Any]] = {}
    raw_count = 0
    outside_count = 0
    missing_geometry_count = 0
    axis_swap_count = 0
    duplicate_count = 0

    for feature in raw_features:
        raw_count += 1
        if not isinstance(feature, dict):
            missing_geometry_count += 1
            continue
        geometry = feature.get("geometry")
        if not isinstance(geometry, dict):
            missing_geometry_count += 1
            continue
        normalized_geometry, swapped = normalize_geometry_axis(
            geometry, center_lon, center_lat
        )
        if swapped:
            feature = dict(feature)
            feature["geometry"] = normalized_geometry
            geometry = normalized_geometry
            axis_swap_count += 1
        if not geometry_intersects_circle(
            geometry, center_lon, center_lat, radius_m
        ):
            outside_count += 1
            continue
        key = feature_key(feature)
        if key in deduplicated:
            duplicate_count += 1
            continue
        deduplicated[key] = feature

    source_counts = {"measured": 0, "floors": 0, "default": 0}
    heights: list[float] = []
    ordered: list[dict[str, Any]] = []
    for key in sorted(deduplicated):
        feature = deduplicated[key]
        source, height = annotate_height(
            feature, policy, center_lon, center_lat
        )
        source_counts[source] += 1
        heights.append(height)
        ordered.append(feature)

    heights.sort()
    building_count = len(ordered)
    quality = {
        "raw_features_from_accepted_tiles": raw_count,
        "buildings_intersecting_circle": building_count,
        "duplicates_removed": duplicate_count,
        "features_outside_circle_removed": outside_count,
        "features_without_supported_geometry_removed": missing_geometry_count,
        "coordinate_axis_swaps": axis_swap_count,
        "height_source_counts": source_counts,
        "height_source_percent": {
            source: round(count * 100.0 / building_count, 3)
            if building_count
            else 0.0
            for source, count in source_counts.items()
        },
        "height_final_m": {
            "minimum": safe_round(min(heights) if heights else None),
            "p50": safe_round(percentile(heights, 0.50)),
            "p95": safe_round(percentile(heights, 0.95)),
            "maximum": safe_round(max(heights) if heights else None),
            "mean": safe_round(statistics.fmean(heights) if heights else None),
        },
    }
    return ordered, quality


def write_json(path: Path, document: Any, pretty: bool = True) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(path.suffix + ".part")
    with temporary_path.open("w", encoding="utf-8") as stream:
        json.dump(
            document,
            stream,
            ensure_ascii=False,
            indent=2 if pretty else None,
            separators=None if pretty else (",", ":"),
        )
        stream.write("\n")
    temporary_path.replace(path)


def write_gzip(source_path: Path, destination_path: Path) -> None:
    temporary_path = destination_path.with_suffix(destination_path.suffix + ".part")
    with source_path.open("rb") as source, temporary_path.open("wb") as raw_output:
        with gzip.GzipFile(
            filename="",
            mode="wb",
            compresslevel=9,
            fileobj=raw_output,
            mtime=0,
        ) as compressed:
            while chunk := source.read(1024 * 1024):
                compressed.write(chunk)
    temporary_path.replace(destination_path)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def write_checksums(output_dir: Path, filenames: Sequence[str]) -> None:
    path = output_dir / "checksums.sha256"
    temporary_path = path.with_suffix(".sha256.part")
    with temporary_path.open("w", encoding="ascii") as stream:
        for filename in filenames:
            stream.write(f"{sha256_file(output_dir / filename)}  {filename}\n")
    temporary_path.replace(path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="한국항공대학교 반경 5 km 브이월드 건물 전량 수집"
    )
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE_PATH)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--api-key-env",
        default="VWORLD_API_KEY",
        help="API 키를 읽을 환경변수 이름(기본: VWORLD_API_KEY)",
    )
    parser.add_argument(
        "--api-key-file",
        type=Path,
        default=DEFAULT_API_KEY_FILE,
        help="환경변수가 없을 때 API 키를 읽을 로컬 파일",
    )
    parser.add_argument(
        "--domain-env",
        default="VWORLD_API_DOMAIN",
        help="선택적 등록 도메인을 읽을 환경변수 이름",
    )
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="API 요청 없이 중심점, 경계, 예상 초기 타일만 검사",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    profile = load_profile(args.profile.resolve())
    output_dir = args.output_dir.resolve()
    center_lat = float(profile["center"]["latitude"])
    center_lon = float(profile["center"]["longitude"])
    radius_m = float(profile["radius_m"])
    source = profile["source"]
    collection = profile["collection"]
    initial_tile_size_m = float(collection["initial_tile_size_m"])
    min_tile_size_m = float(collection["minimum_tile_size_m"])
    max_features = int(collection["max_features_per_request"])
    initial_tiles = make_initial_tiles(radius_m, initial_tile_size_m)

    bbox = [
        center_lon
        - math.degrees(
            radius_m
            / (EARTH_RADIUS_M * math.cos(math.radians(center_lat)))
        ),
        center_lat - math.degrees(radius_m / EARTH_RADIUS_M),
        center_lon
        + math.degrees(
            radius_m
            / (EARTH_RADIUS_M * math.cos(math.radians(center_lat)))
        ),
        center_lat + math.degrees(radius_m / EARTH_RADIUS_M),
    ]
    if args.dry_run:
        print(
            json.dumps(
                {
                    "profile": profile["name"],
                    "center_lon_lat": [center_lon, center_lat],
                    "radius_m": radius_m,
                    "bbox_wgs84_lon_lat": bbox,
                    "initial_tile_count": len(initial_tiles),
                    "initial_tile_size_m": initial_tile_size_m,
                    "typename": source["typename"],
                    "output_dir": str(output_dir),
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0

    api_key = os.environ.get(args.api_key_env, "").strip()
    if not api_key and args.api_key_file.is_file():
        api_key = args.api_key_file.read_text(encoding="utf-8").strip()
    if not api_key:
        raise CollectionError(
            f"환경변수 {args.api_key_env}에 브이월드 API 키가 없습니다. "
            f"또한 로컬 키 파일 {args.api_key_file}도 없습니다. "
            "키를 채팅에 기록하지 말고 둘 중 하나로 안전하게 설정하세요."
        )
    domain = os.environ.get(args.domain_env, "").strip() or None
    workers = max(1, min(int(args.workers), 8))
    cache_dir = output_dir / "raw_tiles"
    client = VWorldClient(
        endpoint=str(source["endpoint"]),
        api_key=api_key,
        typename=str(source["typename"]),
        center_lon=center_lon,
        center_lat=center_lat,
        max_features=max_features,
        cache_dir=cache_dir,
        timeout_s=float(args.timeout_s),
        retries=max(0, int(args.retries)),
        domain=domain,
    )

    started_at = datetime.now(timezone.utc)
    raw_features, tile_records = collect_features(
        client,
        initial_tiles,
        radius_m,
        min_tile_size_m,
        workers,
    )
    buildings, quality = process_features(raw_features, profile)
    finished_at = datetime.now(timezone.utc)

    feature_collection = {
        "type": "FeatureCollection",
        "name": profile["name"],
        "features": buildings,
    }
    manifest = {
        "profile": profile,
        "started_at_utc": started_at.isoformat(),
        "finished_at_utc": finished_at.isoformat(),
        "elapsed_seconds": round((finished_at - started_at).total_seconds(), 3),
        "api_key_stored": False,
        "domain_stored": False,
        "request_contract": {
            "service": "WFS",
            "version": "1.1.0",
            "typename": source["typename"],
            "srsname": "EPSG:4326",
            "bbox_axis_order": "ymin,xmin,ymax,xmax",
            "maxfeatures": max_features,
        },
        "tile_summary": {
            "initial_tile_count": len(initial_tiles),
            "request_tile_count": len(tile_records),
            "subdivided_tile_count": sum(
                1 for record in tile_records if record["subdivided"]
            ),
            "cache_hit_count": sum(
                1 for record in tile_records if record["cache_hit"]
            ),
        },
        "tiles": sorted(tile_records, key=lambda item: item["tile_id"]),
        "outputs": {
            "buildings_geojson": "buildings.geojson",
            "buildings_geojson_gzip": "buildings.geojson.gz",
            "quality_report": "quality_report.json",
            "checksums": "checksums.sha256",
            "raw_tile_cache": "raw_tiles/",
        },
    }
    quality_document = {
        "profile_name": profile["name"],
        "center_lon_lat": [center_lon, center_lat],
        "radius_m": radius_m,
        "selection_rule": collection["include_rule"],
        **quality,
    }
    write_json(output_dir / "buildings.geojson", feature_collection, pretty=False)
    write_json(output_dir / "quality_report.json", quality_document)
    write_json(output_dir / "fetch_manifest.json", manifest)
    write_gzip(
        output_dir / "buildings.geojson",
        output_dir / "buildings.geojson.gz",
    )
    write_checksums(
        output_dir,
        (
            "buildings.geojson",
            "buildings.geojson.gz",
            "quality_report.json",
            "fetch_manifest.json",
        ),
    )
    print(
        f"[완료] {len(buildings):,}개 건물을 "
        f"{output_dir / 'buildings.geojson'}에 저장했습니다."
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except CollectionError as error:
        print(f"[오류] {error}", file=sys.stderr)
        raise SystemExit(2)
