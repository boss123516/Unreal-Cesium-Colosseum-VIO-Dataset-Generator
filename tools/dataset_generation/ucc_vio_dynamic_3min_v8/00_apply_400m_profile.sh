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

data["ClockSpeed"] = 1.0
data["ViewMode"] = "SpringArmChase"
camera_director = data.setdefault("CameraDirector", {})
camera_director["FollowDistance"] = -6.0
data.setdefault("SettingsVersion", 1.2)
data.setdefault("SimMode", "Multirotor")

vehicles = data.setdefault("Vehicles", {})
drone = vehicles.setdefault("Drone1", {})
drone.setdefault("VehicleType", "SimpleFlight")
drone["AutoCreate"] = True
drone["X"] = 0.0
drone["Y"] = 0.0
# AirSim vehicle positions use NED metres relative to Unreal PlayerStart.
# Negative Z is up, so -400 m means a 400 m local spawn altitude.
drone["Z"] = -400.0
drone.setdefault("Pitch", 0.0)
drone.setdefault("Roll", 0.0)
drone.setdefault("Yaw", 0.0)

path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
print(f"[OK] ClockSpeed=1.0, SpringArmChase third-person view, and Drone1 NED Z=-400.0 m written: {path}")
print("[IMPORTANT] Unreal Play/PIE must be fully stopped and restarted for this profile to apply.")
PY
