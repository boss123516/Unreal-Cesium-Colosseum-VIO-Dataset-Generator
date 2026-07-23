#!/usr/bin/env python3
from __future__ import annotations

import sys

try:
    import airsim
except ImportError as exc:
    raise SystemExit(f"[ERROR] AirSim Python package unavailable: {exc}") from exc


vehicle_name = sys.argv[1] if len(sys.argv) > 1 else "Drone1"
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, vehicle_name=vehicle_name)

try:
    client.cancelLastTask(vehicle_name=vehicle_name)
except Exception:
    pass

try:
    client.hoverAsync(vehicle_name=vehicle_name).join()
except Exception as exc:
    print(f"[WARN] hover failed: {exc}")

try:
    client.landAsync(timeout_sec=30, vehicle_name=vehicle_name).join()
except Exception as exc:
    print(f"[WARN] land failed: {exc}")

try:
    client.armDisarm(False, vehicle_name=vehicle_name)
except Exception as exc:
    print(f"[WARN] disarm failed: {exc}")

client.enableApiControl(False, vehicle_name=vehicle_name)
print("[OK] Drone landed, disarmed, and API control released.")
