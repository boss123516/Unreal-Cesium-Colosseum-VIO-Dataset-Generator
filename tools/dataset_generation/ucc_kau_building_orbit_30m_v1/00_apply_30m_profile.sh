#!/usr/bin/env bash
set -euo pipefail

SETTINGS_PATH="${AIRSIM_SETTINGS:-$HOME/Documents/AirSim/settings.json}"
mkdir -p "$(dirname "$SETTINGS_PATH")"

if [[ -f "$SETTINGS_PATH" ]]; then
  BACKUP="${SETTINGS_PATH}.backup.$(date +%Y%m%d_%H%M%S)"
  cp -a "$SETTINGS_PATH" "$BACKUP"
  echo "[BACKUP] $BACKUP"
fi

python3 - "$SETTINGS_PATH" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
if path.exists():
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise SystemExit(f"[ERROR] settings.json parse failed: {exc}")
else:
    data = {"SettingsVersion": 1.2, "SimMode": "Multirotor"}

data["SettingsVersion"] = 1.2
data["SimMode"] = "Multirotor"
data["ClockSpeed"] = 1.0
data["PhysicsEngineName"] = "FastPhysicsEngine"
data["ViewMode"] = "SpringArmChase"

camera_director = data.setdefault("CameraDirector", {})
camera_director["FollowDistance"] = -8.0

vehicles = data.setdefault("Vehicles", {})
drone = vehicles.setdefault("Drone1", {})
drone["VehicleType"] = "SimpleFlight"
drone["AutoCreate"] = True
drone["X"] = 0.0
drone["Y"] = 0.0
# AirSim positions are local NED metres relative to Unreal PlayerStart.
# Negative Z is up, so -30 m places the vehicle 30 m above the KAU origin.
drone["Z"] = -30.0
drone["Pitch"] = 0.0
drone["Roll"] = 0.0
drone["Yaw"] = 0.0

camera = drone.setdefault("Cameras", {}).setdefault("cam0", {})
camera["X"] = 0.20
camera["Y"] = 0.0
camera["Z"] = 0.0
camera["Pitch"] = -15.0
camera["Roll"] = 0.0
camera["Yaw"] = 0.0
capture_settings = camera.setdefault("CaptureSettings", [{}])
if not capture_settings:
    capture_settings.append({})
scene = capture_settings[0]
scene["ImageType"] = 0
scene.setdefault("Width", 640)
scene.setdefault("Height", 480)
scene.setdefault("FOV_Degrees", 90.0)
scene["MotionBlurAmount"] = 0

path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
print(
    "[OK] Applied KAU close-inspection profile: "
    "FastPhysicsEngine, ClockSpeed=1.0, Drone1.Z=-30 m, "
    "cam0 pitch=-15 deg."
)
print(f"[OK] Settings: {path}")
print("[IMPORTANT] Fully stop and restart Unreal Play/PIE before flying.")
PY
