#!/usr/bin/env python3

import argparse
import math
import statistics
import time

import airsim


def percentile(values, percent):
    if not values:
        return float("nan")

    ordered = sorted(values)
    index = math.ceil(len(ordered) * percent / 100.0) - 1
    index = max(0, min(index, len(ordered) - 1))
    return ordered[index]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", default="Drone1")
    parser.add_argument("--camera", default="cam0")
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--raw", action="store_true")
    args = parser.parse_args()

    if args.hz <= 0:
        raise ValueError("--hz must be greater than zero")

    if args.duration <= 0:
        raise ValueError("--duration must be greater than zero")

    client = airsim.MultirotorClient()
    client.confirmConnection()

    request = [
        airsim.ImageRequest(
            args.camera,
            airsim.ImageType.Scene,
            False,
            not args.raw,
        )
    ]

    for _ in range(5):
        response = client.simGetImages(
            request,
            vehicle_name=args.vehicle,
        )[0]

        if response.width <= 0 or response.height <= 0:
            raise RuntimeError("Camera warm-up failed")

    period_s = 1.0 / args.hz
    start_time = time.monotonic()
    stop_time = start_time + args.duration
    next_deadline = start_time

    timestamps_ns = []
    completion_times = []
    latencies_ms = []
    scheduling_lateness_ms = []

    while next_deadline < stop_time:
        sleep_seconds = next_deadline - time.monotonic()

        if sleep_seconds > 0:
            time.sleep(sleep_seconds)

        call_start = time.monotonic()

        scheduling_lateness_ms.append(
            max(0.0, call_start - next_deadline) * 1000.0
        )

        response = client.simGetImages(
            request,
            vehicle_name=args.vehicle,
        )[0]

        call_end = time.monotonic()

        if response.width <= 0 or response.height <= 0:
            raise RuntimeError(
                f"Invalid camera response at frame "
                f"{len(timestamps_ns)}"
            )

        timestamps_ns.append(int(response.time_stamp))
        completion_times.append(call_end)
        latencies_ms.append(
            (call_end - call_start) * 1000.0
        )

        # 다음 목표 시각은 고정된 10 Hz timeline을 유지합니다.
        next_deadline += period_s

        # 여러 frame 이상 늦었다면 밀린 요청을 연속 호출하지 않고
        # 다음 미래 deadline으로 건너뜁니다.
        now = time.monotonic()

        if next_deadline < now:
            skipped_periods = math.floor(
                (now - next_deadline) / period_s
            ) + 1
            next_deadline += skipped_periods * period_s

    actual_elapsed_s = time.monotonic() - start_time
    expected_frames = round(args.duration * args.hz)
    received_frames = len(timestamps_ns)
    frame_drop_count = max(0, expected_frames - received_frames)
    frame_drop_ratio = (
        frame_drop_count / expected_frames
        if expected_frames
        else 0.0
    )

    sensor_intervals_ms = [
        (current - previous) * 1e-6
        for previous, current in zip(
            timestamps_ns,
            timestamps_ns[1:],
        )
        if current > previous
    ]

    completion_intervals_ms = [
        (current - previous) * 1000.0
        for previous, current in zip(
            completion_times,
            completion_times[1:],
        )
    ]

    duplicate_count = sum(
        current == previous
        for previous, current in zip(
            timestamps_ns,
            timestamps_ns[1:],
        )
    )

    backward_count = sum(
        current < previous
        for previous, current in zip(
            timestamps_ns,
            timestamps_ns[1:],
        )
    )

    measured_rate = (
        received_frames / actual_elapsed_s
        if actual_elapsed_s > 0
        else float("nan")
    )

    mode = "Raw uncompressed" if args.raw else "PNG compressed"

    print()
    print(f"=== Exact-duration camera test: {mode} ===")
    print(f"requested rate          : {args.hz:.6f} Hz")
    print(f"requested duration      : {args.duration:.6f} s")
    print(f"actual duration         : {actual_elapsed_s:.6f} s")
    print(f"expected frames         : {expected_frames}")
    print(f"received frames         : {received_frames}")
    print(f"estimated dropped frames: {frame_drop_count}")
    print(f"estimated drop ratio    : {frame_drop_ratio * 100:.3f} %")
    print(f"measured rate           : {measured_rate:.6f} Hz")
    print(f"duplicate timestamps    : {duplicate_count}")
    print(f"backward timestamps     : {backward_count}")

    print()
    print("=== RPC latency ===")
    print(f"mean: {statistics.mean(latencies_ms):.6f} ms")
    print(f"p95 : {percentile(latencies_ms, 95):.6f} ms")
    print(f"p99 : {percentile(latencies_ms, 99):.6f} ms")
    print(f"max : {max(latencies_ms):.6f} ms")

    print()
    print("=== Scheduling lateness ===")
    print(
        f"mean: "
        f"{statistics.mean(scheduling_lateness_ms):.6f} ms"
    )
    print(
        f"p95 : "
        f"{percentile(scheduling_lateness_ms, 95):.6f} ms"
    )
    print(f"max : {max(scheduling_lateness_ms):.6f} ms")

    if completion_intervals_ms:
        print()
        print("=== Wall-clock completion interval ===")
        print(
            f"mean: "
            f"{statistics.mean(completion_intervals_ms):.6f} ms"
        )
        print(
            f"p95 : "
            f"{percentile(completion_intervals_ms, 95):.6f} ms"
        )
        print(
            f"max : "
            f"{max(completion_intervals_ms):.6f} ms"
        )

    if sensor_intervals_ms:
        print()
        print("=== Sensor timestamp interval ===")
        print(
            f"mean: "
            f"{statistics.mean(sensor_intervals_ms):.6f} ms"
        )
        print(
            f"p95 : "
            f"{percentile(sensor_intervals_ms, 95):.6f} ms"
        )
        print(f"max : {max(sensor_intervals_ms):.6f} ms")


if __name__ == "__main__":
    main()
