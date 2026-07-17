#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import re
import struct
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--output-config", type=Path, required=True)
    parser.add_argument("--result-dir", type=Path, required=True)
    parser.add_argument("--support-path", type=Path, required=True)
    return parser.parse_args()


def find_dataset_file(root: Path, relative: str) -> Path:
    direct = root / relative
    if direct.exists():
        return direct

    matches = list(root.rglob(Path(relative).name))
    for match in matches:
        normalized = str(match).replace("\\", "/")
        if normalized.endswith(relative):
            return match

    raise FileNotFoundError(f"required dataset file not found: {relative}")


def first_png_size(image_dir: Path) -> tuple[int, int]:
    images = sorted(image_dir.glob("*.png"))
    if not images:
        raise FileNotFoundError(f"no PNG images found: {image_dir}")

    with images[0].open("rb") as file:
        signature = file.read(24)

    if len(signature) < 24 or signature[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"not a valid PNG: {images[0]}")

    width, height = struct.unpack(">II", signature[16:24])
    return int(width), int(height)


def recursive_dicts(value: Any):
    if isinstance(value, dict):
        yield value
        for child in value.values():
            yield from recursive_dicts(child)
    elif isinstance(value, list):
        for child in value:
            yield from recursive_dicts(child)


def normalized_keys(mapping: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in mapping.items():
        normalized = re.sub(r"[^a-z0-9]", "", str(key).lower())
        result[normalized] = value
    return result


def to_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def find_intrinsics_from_json(
    calibration: dict[str, Any],
    width: int,
    height: int,
) -> tuple[float, float, float, float, str] | None:
    for mapping in recursive_dicts(calibration):
        keys = normalized_keys(mapping)
        if all(name in keys for name in ("fx", "fy", "cx", "cy")):
            values = tuple(to_float(keys[name]) for name in ("fx", "fy", "cx", "cy"))
            if all(value is not None and value > 0 for value in values[:2]):
                fx, fy, cx, cy = values
                return float(fx), float(fy), float(cx), float(cy), "direct fx/fy/cx/cy"

    for mapping in recursive_dicts(calibration):
        keys = normalized_keys(mapping)

        for key in ("cameramatrix", "intrinsicmatrix", "k"):
            value = keys.get(key)
            if value is None:
                continue

            flat: list[float] = []

            def flatten(item: Any) -> None:
                if isinstance(item, list):
                    for child in item:
                        flatten(child)
                else:
                    number = to_float(item)
                    if number is not None:
                        flat.append(number)

            flatten(value)
            if len(flat) >= 9 and flat[0] > 0 and flat[4] > 0:
                return flat[0], flat[4], flat[2], flat[5], f"matrix field {key}"

        for key in ("intrinsics", "projectionparameters"):
            value = keys.get(key)
            if isinstance(value, list) and len(value) >= 4:
                parsed = [to_float(item) for item in value[:4]]
                if all(item is not None for item in parsed):
                    fx, fy, cx, cy = parsed
                    if fx > 0 and fy > 0:
                        return fx, fy, cx, cy, f"list field {key}"

    fov_candidates = []
    for mapping in recursive_dicts(calibration):
        keys = normalized_keys(mapping)
        for key in (
            "horizontalfovdegrees",
            "horizontalfov",
            "hfovdegrees",
            "hfov",
            "fovdegrees",
            "fov",
        ):
            if key in keys:
                value = to_float(keys[key])
                if value is not None and 10.0 <= value <= 170.0:
                    fov_candidates.append((value, key))

    if fov_candidates:
        hfov_deg, field = fov_candidates[0]
        fx = width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
        fy = fx
        return fx, fy, width / 2.0, height / 2.0, f"horizontal FOV field {field}"

    return None


def load_intrinsics(
    dataset_root: Path,
    width: int,
    height: int,
) -> tuple[float, float, float, float, str]:
    env_names = ("UCC_FX", "UCC_FY", "UCC_CX", "UCC_CY")
    if all(os.environ.get(name) for name in env_names):
        fx, fy, cx, cy = [float(os.environ[name]) for name in env_names]
        return fx, fy, cx, cy, "environment overrides"

    calibration_files = [
        dataset_root / "camera_calibration.json",
        *dataset_root.glob("**/camera_calibration.json"),
    ]

    seen: set[Path] = set()
    for path in calibration_files:
        path = path.resolve()
        if path in seen or not path.is_file():
            continue
        seen.add(path)

        try:
            calibration = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            print(f"[WARN] failed to parse {path}: {exc}")
            continue

        result = find_intrinsics_from_json(calibration, width, height)
        if result is not None:
            fx, fy, cx, cy, source = result
            return fx, fy, cx, cy, f"{path}: {source}"

    hfov_deg = float(os.environ.get("UCC_FALLBACK_HFOV_DEG", "90.0"))
    fx = width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
    fy = fx

    return (
        fx,
        fy,
        width / 2.0,
        height / 2.0,
        f"fallback horizontal FOV={hfov_deg} deg",
    )


def main() -> int:
    args = parse_args()

    dataset_root = args.dataset_root.expanduser().resolve()
    output_config = args.output_config.expanduser().resolve()
    result_dir = args.result_dir.expanduser().resolve()
    support_path = args.support_path.expanduser().resolve()

    camera_csv = find_dataset_file(dataset_root, "mav0/cam0/data.csv")
    imu_csv = find_dataset_file(dataset_root, "mav0/imu0/data.csv")
    image_dir = camera_csv.parent / "data"

    width, height = first_png_size(image_dir)
    fx, fy, cx, cy, intrinsic_source = load_intrinsics(
        dataset_root,
        width,
        height,
    )

    output_config.parent.mkdir(parents=True, exist_ok=True)
    result_dir.mkdir(parents=True, exist_ok=True)

    yaml = f"""%YAML:1.0

# UCC synthetic dataset configuration
imu_topic: "/imu0"
image_topic: "/cam0/image_raw"
output_path: "{result_dir}"

model_type: PINHOLE
camera_name: ucc_cam0
image_width: {width}
image_height: {height}

# AirSim Scene image is modeled as an ideal pinhole camera.
distortion_parameters:
   k1: 0.0
   k2: 0.0
   p1: 0.0
   p2: 0.0

projection_parameters:
   fx: {fx:.12f}
   fy: {fy:.12f}
   cx: {cx:.12f}
   cy: {cy:.12f}

# First smoke test: estimate camera-IMU extrinsic online.
# After confirming the exact AirSim optical/body frame transform, change to 0.
estimate_extrinsic: 2
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0]
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.0, 0.0, 0.0]

max_cnt: 200
min_dist: 20
freq: 10
F_threshold: 1.0
show_track: 1
equalize: 1
fisheye: 0

max_solver_time: 0.04
max_num_iterations: 8
keyframe_parallax: 8.0

# Initial values based on the repository's EuRoC configuration.
acc_n: 0.08
gyr_n: 0.004
acc_w: 0.00004
gyr_w: 2.0e-6
g_norm: 9.80665

# Loop closure is intentionally disabled for the first VIO test.
loop_closure: 0
load_previous_pose_graph: 0
fast_relocalization: 0
pose_graph_save_path: "{result_dir / 'pose_graph'}"
support_path: "{support_path}"

estimate_td: 0
td: 0.0

rolling_shutter: 0
rolling_shutter_tr: 0.0

save_image: 0
visualize_imu_forward: 0
visualize_camera_size: 0.4
"""

    output_config.write_text(yaml, encoding="utf-8")

    metadata = {
        "dataset_root": str(dataset_root),
        "camera_csv": str(camera_csv),
        "imu_csv": str(imu_csv),
        "image_dir": str(image_dir),
        "width": width,
        "height": height,
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "intrinsic_source": intrinsic_source,
        "estimate_extrinsic": 2,
        "loop_closure": 0,
        "config_path": str(output_config),
        "result_dir": str(result_dir),
    }

    metadata_path = output_config.with_suffix(".metadata.json")
    metadata_path.write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )

    print("[OK] UCC VINS configuration generated")
    print(f"  config     : {output_config}")
    print(f"  metadata   : {metadata_path}")
    print(f"  dataset    : {dataset_root}")
    print(f"  resolution : {width}x{height}")
    print(f"  intrinsics : fx={fx:.6f}, fy={fy:.6f}, cx={cx:.6f}, cy={cy:.6f}")
    print(f"  source     : {intrinsic_source}")
    print(f"  result dir : {result_dir}")

    if intrinsic_source.startswith("fallback"):
        print()
        print("[WARN] Calibration JSON did not expose fx/fy/cx/cy.")
        print("       This run uses a 90-degree horizontal-FOV fallback.")
        print("       It is valid only as a smoke test, not a final evaluation.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
