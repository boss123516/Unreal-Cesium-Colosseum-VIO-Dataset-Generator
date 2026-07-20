# Fixed-wing integration estimate

This estimate maps the 2026-07-20 execution plan to the current local UCC
baseline. It assumes one developer, an already working Unreal/Cesium/Colosseum
project, and the PX4 `gz_rc_cessna` model rather than a custom aircraft model.

## Current baseline findings

- UCC baseline commit: `4f7e01f`
- Preserved source branch: `ucc/dynamic-vio-3min-v8-success-20260718`
- Fixed-wing work branch: `feature/fixedwing-integration-20260720`
- Colosseum contains `ExternalPhysicsEngine`, the `simSetKinematics` RPC, and
  all six `KinematicsState` fields.
- Runtime testing proved that AirSim IMU reads the same ground-truth kinematics
  object updated by `simSetKinematics` in this fork.
- PX4 v1.17.0 and Gazebo Harmonic 8.14.0 are installed and `gz_rc_cessna`
  automatic takeoff and controlled forward flight have passed.
- UCC can be started either in Editor PIE or as `UnrealEditor -game`; both expose
  the same AirSim RPC contract used by the bridge.

## Effort estimate

| Work package | Expected effort | Exit condition |
|---|---:|---|
| Baseline backup and preflight | 1-2 h | Existing settings, commit, RPC contract recorded |
| External Physics profile and synthetic IMU gate | 2-4 h | Injected gyro and acceleration appear in AirSim IMU |
| PX4/Gazebo install and Cessna-only Gate A | 3-6 h | `gz_rc_cessna` runs and can sustain controlled flight |
| MAVSDK/Gazebo-to-AirSim Python bridge | 1-2 d | Pose, twist, acceleration, timeout and rate logs work |
| Frame/origin validation | 0.5-1 d | All position, attitude and IMU axis tests pass |
| Unreal fixed-wing visual pawn | 0.5-1 d | Mesh and camera mount render correctly |
| 30 s camera/IMU/GT mini dataset | 1-2 d | Timing and data-quality gates pass |

The original practical MVP estimate was **3-5 working days**; the kinematics
MVP and controlled-flight gate are now complete. The remaining mini-dataset and
visual-pawn hardening is estimated at **2-4 working days**. A deterministic C++ bridge,
simulation-clock synchronization, wind scenarios, and a validated 180-second
research dataset add approximately **5-10 working days**. A custom aircraft
aerodynamic model and PX4 tuning are a separate calibration project.

## Critical path

1. Prove `simSetKinematics -> ground truth -> ImuSimple` at runtime.
2. Prove `gz_rc_cessna` independently before coupling simulators.
3. Validate frame conversion with one-axis tests before free flight.
4. Record a 30-second no-wind/no-noise dataset before adding visual or physical
   disturbances.

If step 1 fails, the architecture remains viable but requires a Colosseum C++
adapter before the Python bridge is expanded.
