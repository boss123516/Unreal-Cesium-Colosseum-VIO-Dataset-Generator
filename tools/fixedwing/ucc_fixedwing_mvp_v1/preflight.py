#!/usr/bin/env python3
"""Read-only local readiness report for the fixed-wing MVP."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import socket
import subprocess
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, default=Path.home() / "vio_sim_ws")
    parser.add_argument(
        "--repo",
        type=Path,
        default=Path.home() / "research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator",
    )
    parser.add_argument("--px4-root", type=Path, default=Path.home() / "PX4-Autopilot")
    parser.add_argument(
        "--settings",
        type=Path,
        default=Path.home() / "Documents/AirSim/settings.json",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--json-output", type=Path)
    return parser.parse_args()


def git_value(repo: Path, *arguments: str) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "-C", str(repo), *arguments],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def main() -> int:
    args = parse_args()
    workspace = args.workspace.expanduser().resolve()
    repo = args.repo.expanduser().resolve()
    px4_root = args.px4_root.expanduser().resolve()
    settings_path = args.settings.expanduser().resolve()
    checks: list[dict] = []

    def add(name: str, status: str, detail: str) -> None:
        checks.append({"name": name, "status": status, "detail": detail})
        print(f"[{status}] {name}: {detail}")

    branch = git_value(repo, "branch", "--show-current")
    commit = git_value(repo, "rev-parse", "--short", "HEAD")
    if branch and commit:
        add("UCC repository", "PASS", f"{branch} @ {commit}")
    else:
        add("UCC repository", "FAIL", f"not a Git repository: {repo}")

    python_client = workspace / "Colosseum/PythonClient"
    venv_python = workspace / "airsim_pyenv/bin/python"
    if python_client.is_dir() and venv_python.is_file():
        probe = (
            "import sys; "
            f"sys.path.insert(0, {str(python_client)!r}); "
            "import airsim; "
            "required={'position','orientation','linear_velocity','angular_velocity',"
            "'linear_acceleration','angular_acceleration'}; "
            "actual={name for name,_ in airsim.KinematicsState.attribute_order}; "
            "assert required == actual; "
            "assert hasattr(airsim.VehicleClient, 'simSetKinematics')"
        )
        result = subprocess.run(
            [str(venv_python), "-c", probe],
            text=True,
            capture_output=True,
        )
        if result.returncode == 0:
            add("Colosseum Python API", "PASS", "simSetKinematics and six state fields present")
        else:
            add("Colosseum Python API", "FAIL", result.stderr.strip() or "import failed")
    else:
        add("Colosseum Python API", "FAIL", "PythonClient or airsim_pyenv missing")

    if settings_path.is_file():
        try:
            settings = json.loads(settings_path.read_text(encoding="utf-8"))
            engine = settings.get("PhysicsEngineName")
            profile_status = "PASS" if engine == "ExternalPhysicsEngine" else "WARN"
            add("AirSim settings", profile_status, f"PhysicsEngineName={engine!r} in {settings_path}")
        except Exception as exc:
            add("AirSim settings", "FAIL", f"JSON parse error: {exc}")
    else:
        add("AirSim settings", "FAIL", f"not found: {settings_path}")

    try:
        with socket.create_connection((args.host, args.port), timeout=0.25):
            add("AirSim RPC", "PASS", f"TCP {args.host}:{args.port} open")
    except OSError:
        add("AirSim RPC", "WARN", "not running; start Unreal Play/PIE for runtime gates")

    px4_commit = git_value(px4_root, "describe", "--tags", "--always")
    if px4_commit:
        add("PX4 source", "PASS", f"{px4_commit} at {px4_root}")
    else:
        add("PX4 source", "FAIL", f"not found or incomplete: {px4_root}")

    gz_path = shutil.which("gz")
    if gz_path:
        add("Gazebo CLI", "PASS", gz_path)
    else:
        add("Gazebo CLI", "FAIL", "gz not found on PATH")

    summary = {
        "checks": checks,
        "counts": {
            status: sum(item["status"] == status for item in checks)
            for status in ("PASS", "WARN", "FAIL")
        },
    }
    print(
        "[SUMMARY] "
        + ", ".join(f"{key}={value}" for key, value in summary["counts"].items())
    )

    if args.json_output:
        output = args.json_output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"[REPORT] {output}")

    return 1 if summary["counts"]["FAIL"] else 0


if __name__ == "__main__":
    sys.exit(main())
