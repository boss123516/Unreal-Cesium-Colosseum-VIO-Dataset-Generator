#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import airsim
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--camera", default="cam0")
    parser.add_argument("--duration-sec", type=float, default=15.0)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path.home() / "vio_sim_ws" / "validation_runs",
    )
    return parser.parse_args()


def frame_metrics(response: object) -> dict[str, float]:
    width = int(response.width)
    height = int(response.height)
    raw = np.frombuffer(bytes(response.image_data_uint8), dtype=np.uint8)

    if width <= 0 or height <= 0:
        raise RuntimeError(f"invalid frame size {width}x{height}")

    pixel_count = width * height
    if raw.size % pixel_count != 0:
        raise RuntimeError(
            f"unexpected image bytes: {raw.size} for {width}x{height}"
        )

    channels = raw.size // pixel_count
    if channels not in (3, 4):
        raise RuntimeError(f"unsupported channel count: {channels}")

    image = raw.reshape(height, width, channels)[..., :3].astype(np.int16)

    # AirSim raw Scene images are commonly BGRA/BGR. Treat channel 0 as blue.
    blue = image[..., 0]
    green = image[..., 1]
    red = image[..., 2]

    lower = slice(int(height * 0.30), height)
    suspicious_blue = (
        (blue[lower] > 105)
        & (blue[lower] > red[lower] * 1.25 + 15)
        & (blue[lower] > green[lower] * 1.05 + 5)
        & ((blue[lower] - red[lower]) > 35)
    )

    quantized = (image[lower] // 16).astype(np.uint8)
    packed = (
        quantized[..., 0].astype(np.uint32)
        | (quantized[..., 1].astype(np.uint32) << 4)
        | (quantized[..., 2].astype(np.uint32) << 8)
    )
    _, counts = np.unique(packed, return_counts=True)

    return {
        "blue_ratio_lower70": float(suspicious_blue.mean()),
        "dominant_quantized_color_ratio_lower70": float(counts.max() / counts.sum()),
        "mean_blue": float(blue.mean()),
        "mean_green": float(green.mean()),
        "mean_red": float(red.mean()),
    }


def save_ppm(path: Path, response: object) -> None:
    width = int(response.width)
    height = int(response.height)
    raw = np.frombuffer(bytes(response.image_data_uint8), dtype=np.uint8)
    channels = raw.size // (width * height)
    bgr = raw.reshape(height, width, channels)[..., :3]
    rgb = bgr[..., ::-1]
    with path.open("wb") as file:
        file.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        file.write(rgb.tobytes())


def main() -> int:
    args = parse_args()
    if args.duration_sec <= 0 or args.hz <= 0:
        raise SystemExit("[ERROR] duration and hz must be positive")

    output = (
        args.output_root
        / f"cam0_cesium_probe_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    output.mkdir(parents=True, exist_ok=True)

    client = airsim.MultirotorClient()
    client.confirmConnection()

    count = max(1, int(round(args.duration_sec * args.hz)))
    period = 1.0 / args.hz
    samples: list[dict[str, float]] = []

    request = airsim.ImageRequest(
        args.camera,
        airsim.ImageType.Scene,
        pixels_as_float=False,
        compress=False,
    )

    print(f"[PROBE] {count} frames, {args.duration_sec:.1f}s, {args.hz:.1f}Hz")
    print(f"[PROBE] output: {output}")

    start = time.monotonic()
    for index in range(count):
        deadline = start + index * period
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)

        responses = client.simGetImages([request], vehicle_name=args.vehicle)
        if not responses:
            raise RuntimeError("simGetImages returned no response")

        response = responses[0]
        metrics = frame_metrics(response)
        metrics["index"] = index
        samples.append(metrics)

        if index in {0, count // 2, count - 1}:
            save_ppm(output / f"frame_{index:04d}.ppm", response)

    blue_ratios = np.array(
        [sample["blue_ratio_lower70"] for sample in samples],
        dtype=float,
    )
    dominant_ratios = np.array(
        [sample["dominant_quantized_color_ratio_lower70"] for sample in samples],
        dtype=float,
    )

    summary = {
        "vehicle": args.vehicle,
        "camera": args.camera,
        "duration_sec": args.duration_sec,
        "hz": args.hz,
        "frames": len(samples),
        "blue_ratio_lower70_mean": float(blue_ratios.mean()),
        "blue_ratio_lower70_max": float(blue_ratios.max()),
        "dominant_color_ratio_lower70_mean": float(dominant_ratios.mean()),
        "dominant_color_ratio_lower70_max": float(dominant_ratios.max()),
        # Conservative automated gate; visual inspection of 3 saved PPMs remains required.
        "automatic_pass": bool(
            blue_ratios.max() < 0.20
            and dominant_ratios.max() < 0.55
        ),
        "interpretation": (
            "PASS means no large blue/dominant blank region was detected in the "
            "lower 70% of the static cam0 frames. Inspect the saved PPM images too."
        ),
    }

    (output / "samples.json").write_text(
        json.dumps(samples, indent=2) + "\n",
        encoding="utf-8",
    )
    (output / "probe_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n",
        encoding="utf-8",
    )

    print(json.dumps(summary, indent=2))
    print(f"[RESULT] {'PASS' if summary['automatic_pass'] else 'FAIL'}")
    return 0 if summary["automatic_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
