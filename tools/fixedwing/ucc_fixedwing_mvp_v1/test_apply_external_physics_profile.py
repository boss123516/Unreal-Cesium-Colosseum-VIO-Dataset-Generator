#!/usr/bin/env python3

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT_DIR = Path(__file__).resolve().parent
PROFILE_SCRIPT = SCRIPT_DIR / "apply_external_physics_profile.py"


class ApplyExternalPhysicsProfileTest(unittest.TestCase):
    def test_profile_selects_horizon_stable_observer(self):
        settings = {
            "ViewMode": "SpringArmChase",
            "Vehicles": {
                "Drone1": {
                    "Cameras": {"cam0": {"CaptureSettings": []}},
                    "Sensors": {"Imu": {}},
                }
            },
        }

        with tempfile.TemporaryDirectory() as temporary_directory:
            settings_path = Path(temporary_directory) / "settings.json"
            settings_path.write_text(json.dumps(settings), encoding="utf-8")

            result = subprocess.run(
                [
                    sys.executable,
                    str(PROFILE_SCRIPT),
                    "--settings",
                    str(settings_path),
                    "--no-backup",
                ],
                text=True,
                capture_output=True,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            applied = json.loads(settings_path.read_text(encoding="utf-8"))
            self.assertEqual(applied["ViewMode"], "SpringArmChase")
            self.assertEqual(applied["CameraDirector"]["FollowDistance"], -2.0)
            self.assertEqual(
                applied["PhysicsEngineName"], "ExternalPhysicsEngine"
            )


if __name__ == "__main__":
    unittest.main()
