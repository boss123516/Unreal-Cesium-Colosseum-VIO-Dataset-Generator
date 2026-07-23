#!/usr/bin/env python3
"""Render a compact 2D verification image of the terrain-grounded buildings."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT = (
    SCRIPT_DIR / "output" / "buildings_with_elevation.geojson"
)
DEFAULT_OUTPUT = (
    SCRIPT_DIR / "output" / "tiles3d" / "kau_5km_height_preview.png"
)
EARTH_RADIUS_M = 6_378_137.0
CENTER_LON = 126.86519
CENTER_LAT = 37.60025


def local_km(coordinates: list[list[float]]) -> np.ndarray:
    array = np.asarray(coordinates, dtype=np.float64)
    east = (
        np.radians(array[:, 0] - CENTER_LON)
        * EARTH_RADIUS_M
        * math.cos(math.radians(CENTER_LAT))
        / 1000.0
    )
    north = (
        np.radians(array[:, 1] - CENTER_LAT)
        * EARTH_RADIUS_M
        / 1000.0
    )
    return np.column_stack((east, north))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="KAU 5 km 건물 높이 미리보기")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    document = json.loads(args.input.resolve().read_text(encoding="utf-8"))
    polygons: list[np.ndarray] = []
    heights: list[float] = []
    source_counts = {"measured": 0, "floors": 0, "default": 0}
    for feature in document["features"]:
        geometry = feature["geometry"]
        properties = feature["properties"]["_kau5km"]
        height = float(properties["height_final_m"])
        source = str(properties["height_source"])
        source_counts[source] = source_counts.get(source, 0) + 1
        coordinate_sets = (
            geometry["coordinates"]
            if geometry["type"] == "MultiPolygon"
            else [geometry["coordinates"]]
        )
        for polygon in coordinate_sets:
            if polygon and len(polygon[0]) >= 4:
                polygons.append(local_km(polygon[0]))
                heights.append(height)

    height_array = np.asarray(heights, dtype=np.float64)
    color_max = max(20.0, float(np.percentile(height_array, 99.0)))
    figure, axis = plt.subplots(figsize=(12, 12), dpi=200)
    collection = PolyCollection(
        polygons,
        array=np.clip(height_array, 0.0, color_max),
        cmap="viridis",
        edgecolors="none",
        linewidths=0.0,
    )
    collection.set_clim(0.0, color_max)
    axis.add_collection(collection)
    circle = plt.Circle(
        (0.0, 0.0),
        5.0,
        fill=False,
        color="#d62728",
        linewidth=1.0,
        alpha=0.9,
    )
    axis.add_patch(circle)
    axis.scatter(
        [0.0],
        [0.0],
        marker="*",
        s=70,
        color="#d62728",
        edgecolor="white",
        linewidth=0.5,
        zorder=5,
        label="Korea Aerospace University",
    )
    axis.set_xlim(-5.15, 5.15)
    axis.set_ylim(-5.15, 5.15)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("East from campus center (km)")
    axis.set_ylabel("North from campus center (km)")
    axis.set_title(
        "KAU 5 km LoD1 buildings — height_final_m\n"
        f"54,007 buildings | measured {source_counts['measured']:,} | "
        f"floors {source_counts['floors']:,} | default {source_counts['default']:,}"
    )
    axis.grid(color="#cccccc", linewidth=0.25, alpha=0.5)
    axis.legend(loc="lower left")
    colorbar = figure.colorbar(collection, ax=axis, shrink=0.75, pad=0.02)
    colorbar.set_label(
        f"Building height (m, colors capped at p99={color_max:.1f} m)"
    )
    figure.tight_layout()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, bbox_inches="tight")
    plt.close(figure)
    print(f"[완료] {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
