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
data.setdefault("SettingsVersion", 1.2)
data.setdefault("SimMode", "Multirotor")

path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
print(f"[OK] ClockSpeed=1.0 written: {path}")
print("[IMPORTANT] Unreal Play/PIE must be stopped and started again for ClockSpeed to apply.")
PY
