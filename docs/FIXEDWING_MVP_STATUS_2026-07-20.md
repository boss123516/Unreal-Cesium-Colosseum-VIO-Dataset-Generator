# Fixed-wing MVP status — 2026-07-20

## Outcome

The fixed-wing kinematics MVP is a **Go**. PX4 v1.17.0 and Gazebo Harmonic
8.14.0 run the `gz_rc_cessna` dynamics, and the Python bridge injects PX4
NED/FRD state into the UCC `ExternalPhysicsEngine` at 100 Hz. The injected
ground truth reaches both AirSim IMU and camera motion in the current UE 5.6 /
Colosseum 2.3.0 runtime. A Colosseum C++ adapter is not required for this MVP.

The remaining work is dataset hardening: direct Gazebo acceleration instead of
finite differences, a fixed-wing Unreal visual pawn, explicit bank/axis visual
checks, and the 30-second mini-dataset timing and quality gate.

## Completed

- Preserved UCC baseline commit `4f7e01f` and created
  `feature/fixedwing-integration-20260720`.
- Backed up the quadrotor AirSim settings and applied an External Physics,
  no-noise validation profile.
- Installed Gazebo Harmonic 8.14.0 with the official PX4 Ubuntu setup.
- Cloned and built PX4 v1.17.0 at `$HOME/PX4-Autopilot`.
- Spawned `rc_cessna_0` with airframe `SYS_AUTOSTART=4003` and verified valid
  PX4 position, velocity and attitude estimates.
- Added and tested ENU/FLU to NED/FRD vector and quaternion conversion.
- Implemented the PX4 MAVLink to AirSim full-kinematics MVP bridge, including a
  1 Hz GCS heartbeat required for unattended PX4 preflight readiness.
- Added source timeout, invalid-state, quaternion, timestamp, duplicate, drop,
  latency and RPC-failure accounting.
- Proved automatic takeoff and controlled forward flight while the bridge drove
  live UCC ground truth, IMU and camera motion.

## Runtime evidence

Synthetic UCC runtime gate:

| Test | Injection | AirSim IMU result | Status |
|---|---:|---:|---|
| Static acceleration norm | 0 world acceleration | 9.806650 m/s² | Pass |
| Body X gyro | 0.1 rad/s | 0.1000000015 rad/s | Pass |
| World/body X acceleration delta | 1.0 m/s² | 1.0 m/s² | Pass |
| Ground-truth round trip | 1.0 m/s² | 1.0 m/s² | Pass |

Actual PX4/Gazebo Cessna to live UCC stationary gate:

| Metric | Result |
|---|---:|
| Duration / injections | 20.0 s / 2,000 |
| Source / AirSim RPC injection rate | 99.999 / 99.999 Hz |
| Mean / p95 / max receive-to-inject latency | 2.20 / 3.67 / 4.83 ms |
| Dropped states | 0 |
| Timestamp regressions | 0 |
| Invalid quaternion/numeric states | 0 / 0 |
| RPC failures | 0 |

Actual automatic-takeoff and forward-flight gate:

| Metric | Result |
|---|---:|
| Duration / AirSim RPC injections | 45.0 s / 4,500 |
| Source / AirSim RPC injection rate | 100.044 / 99.9998 Hz |
| Mean / p95 / max receive-to-inject latency | 4.32 / 5.54 / 6.96 ms |
| Dropped states | 1 at startup |
| Timestamp regressions / invalid values / RPC failures | 0 / 0 / 0 |
| Maximum displacement | 151.65 m |
| Maximum horizontal / 3D speed | 13.73 / 14.47 m/s |
| Maximum relative altitude | 34.41 m |

PX4 reported `Ready for takeoff`, `Preflight check: OK`, and `Takeoff
detected`. During the sampled flight its local position was approximately
`[-12.23, 141.03, -30.44] m` with about `13.0 m/s` horizontal speed, a valid
heading solution, and no active failsafe.

The final dynamic bridge row and UCC `simGetGroundTruthKinematics()` agreed for
position, orientation, velocity, angular velocity and acceleration within
float serialization precision. AirSim IMU angular velocity also matched the
injected ground truth. The live scene camera returned a valid 256x144 RGB frame
with standard deviation 33.70 and zero pixels above the all-white threshold,
so the earlier standalone white-frame smoke result is no longer reproduced.

Artifacts:

- `artifacts/fixedwing_mvp/synthetic_gate/imu_validation_20260720.json`
- `artifacts/fixedwing_mvp/px4_cessna_live_bridge/summary.json`
- `artifacts/fixedwing_mvp/px4_cessna_live_bridge/state.csv`
- `artifacts/fixedwing_mvp/px4_cessna_takeoff_bridge/summary.json`
- `artifacts/fixedwing_mvp/px4_cessna_takeoff_bridge/state.csv`

## Open gates

1. Consume linear and angular acceleration directly from Gazebo physics rather
   than finite-difference PX4 telemetry.
2. Run explicit one-axis and banked-turn visual checks and save their evidence.
3. Create or select a fixed-wing Unreal pawn/mesh and validate camera occlusion.
4. Restore the intended runtime IMU-noise profile and record the 30-second
   no-wind/no-noise camera/IMU/ground-truth mini dataset.
5. Validate timestamp alignment, frame counts and data quality before extending
   to the 180-second research scenario.

## Reproduction

From `tools/fixedwing/ucc_fixedwing_mvp_v1`, run PX4/Gazebo and UCC first, then:

```bash
RUN_DIR="$HOME/vio_sim_ws/artifacts/fixedwing_mvp/live_bridge_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
./04_run_px4_airsim_bridge.sh \
  --mavlink udpin:0.0.0.0:14540 \
  --duration-sec 30 \
  --summary "$RUN_DIR/bridge_summary.json" \
  --state-log "$RUN_DIR/bridge_state.csv"
```
