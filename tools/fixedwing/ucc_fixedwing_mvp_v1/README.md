# UCC fixed-wing MVP v1

This package implements the first runtime gates from the 2026-07-20 fixed-wing
execution plan without changing the validated quadrotor recorder.

## What is implemented

- read-only environment preflight;
- minimal, backup-first `ExternalPhysicsEngine` settings update;
- a no-noise IMU validation profile;
- explicit ENU/FLU to NED/FRD vector and quaternion conversion;
- a synthetic full-kinematics probe for AirSim ground truth and IMU;
- offline frame-conversion unit tests.

The Unreal visual pawn and live Gazebo state source are deliberately gated on
the synthetic IMU result. If this probe fails, inspect or patch the Colosseum
sensor update path before expanding the bridge.

## 1. Offline checks

```bash
cd tools/fixedwing/ucc_fixedwing_mvp_v1
python3 -m unittest -v test_fixedwing_frames.py
./00_preflight.sh
```

`AirSim RPC` is reported as a warning while Unreal Play/PIE is stopped. Missing
PX4 or `gz` is a failure for Gate A but does not prevent the offline tests.

## 2. Apply the synthetic validation profile

Stop Unreal Play/PIE before changing the profile.

```bash
./01_apply_external_physics_profile.sh --profile validation
```

The script preserves every existing camera, spawn, view and project-specific
setting. It changes only the external-physics/RPC requirements and the IMU noise
parameters needed for an axis test. The previous file is copied beside
`settings.json` with a timestamped `backup.fixedwing_*` suffix.

Restart Play/PIE after applying the profile. AirSim settings are not hot-loaded.

## 3. Prove full kinematics reaches AirSim IMU

With Play/PIE running:

```bash
mkdir -p "$HOME/vio_sim_ws/artifacts/fixedwing_mvp/synthetic_gate"
./02_run_synthetic_imu_probe.sh \
  --output "$HOME/vio_sim_ws/artifacts/fixedwing_mvp/synthetic_gate/imu_validation.json"
```

The probe injects and verifies four conditions:

1. static gyro near zero and accelerometer norm near gravity;
2. body-X angular rate appears on AirSim gyro X;
3. world-NED-X acceleration produces the expected body-X accelerometer delta;
4. `simGetGroundTruthKinematics()` returns the injected acceleration.

The original kinematics state is restored in a `finally` block. Run this only
with `ExternalPhysicsEngine`; the script refuses any other runtime engine.

## 4. Install the MAVLink bridge dependency

```bash
./03_install_bridge_python_deps.sh
```

The pinned `pymavlink` version matches the version selected by the PX4 v1.17.0
setup requirements at the time this package was created.

## 5. Install Gazebo and run the Cessna-only gate

The official PX4 Ubuntu setup installs system packages and therefore requires
the local user's sudo password:

```bash
./05_setup_px4_gazebo.sh
```

Restart the machine if the PX4 setup script requests it. Then run:

```bash
./06_run_gz_rc_cessna.sh
```

Set `HEADLESS=1` in the environment for a server-only check. Gate A still
requires a later visual or telemetry check that the plane can sustain controlled
forward flight.

## 6. Run the PX4-to-AirSim MVP bridge

Start `gz_rc_cessna` and UCC External Physics first. Then run:

```bash
RUN_DIR="$HOME/vio_sim_ws/artifacts/fixedwing_mvp/bridge_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
./04_run_px4_airsim_bridge.sh \
  --mavlink udpin:0.0.0.0:14540 \
  --duration-sec 30 \
  --summary "$RUN_DIR/bridge_summary.json" \
  --state-log "$RUN_DIR/bridge_state.csv"
```

PX4 telemetry is already NED/FRD, so this source path does not apply the
Gazebo ENU/FLU conversion. Acceleration is a low-pass-filtered finite
difference for MVP response testing. It is explicitly marked as unsuitable for
the final research dataset; the deterministic bridge must consume acceleration
from Gazebo physics directly.

The bridge terminates when source state age exceeds 0.5 seconds, after five
consecutive AirSim RPC failures, or on invalid runtime configuration. Its
summary includes source/injection rates, latency, drops, timestamp regressions,
invalid values and RPC failures. It also emits a 1 Hz MAVLink GCS heartbeat so
PX4 remains preflight-ready during an unattended bridge run.

## 7. Restore runtime sensor noise

After the frame and gravity checks pass, restore the known-good quad settings
backup or apply the external physics profile without changing existing IMU noise:

```bash
./01_apply_external_physics_profile.sh --profile runtime
```

If the current file already contains the zero-noise validation values, restore
the pre-validation backup first; the runtime profile intentionally preserves
whatever IMU values exist in its input.

## Frame contract

The conversion module accepts Gazebo state fields with these frames:

```text
position, linear velocity, linear acceleration: world ENU
orientation: body FLU -> world ENU, quaternion xyzw
angular velocity, angular acceleration: body FLU
```

It outputs the AirSim equivalent in world NED and body FRD. If a selected Gazebo
topic publishes angular velocity in world coordinates, convert it as an ENU
world vector before using the bridge contract; do not feed it to the FLU helper.
