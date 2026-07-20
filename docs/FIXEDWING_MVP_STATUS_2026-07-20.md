# Fixed-wing integration status

## Outcome

The fixed-wing integration MVP is complete on branch `fixed_wing`. PX4 v1.17.0
and Gazebo Harmonic 8.14.0 run the `gz_rc_cessna` dynamics, a native Gazebo
plugin publishes direct link kinematics, and the UCC bridge injects converted
state into Colosseum `ExternalPhysicsEngine` at 100 Hz.

The final no-wind/no-noise 30-second mini dataset passed its camera, IMU,
ground-truth, timestamp, quaternion and content gates.

## Implemented path

```text
PX4 fixed-wing controller
        |
Gazebo rc_cessna dynamics
        |
UccKinematicsPublisher, 250 Hz ENU/FLU
        |
gazebo_airsim_bridge.py, 100 Hz NED/FRD
        |
Colosseum ExternalPhysicsEngine
        |----------------------|
 AirSim IMU/GT          Unreal/Cesium cam0
```

Implemented components:

- native Gazebo `gz::sim::System` link-state publisher;
- explicit 21-field transport contract and validation;
- ENU/FLU to NED/FRD conversion and first-state reanchoring;
- direct Gazebo linear and angular acceleration consumption;
- 1 Hz MAVLink GCS heartbeat for PX4 readiness;
- UCC injection rate, source age, latency, deadline, timestamp and RPC gates;
- `PhysicsBody` scoped lock in `ExternalPhysicsEngine` to prevent concurrent
  sensor/update access and the observed `SensorCollection` crash;
- fixed-wing Unreal Cessna visual with quad components hidden;
- built-in front-center SceneCapture reused as the `cam0` alias because
  runtime-spawned `BP_PIPCamera` Scene RGB captures remain blank in this map;
- camera warm-up gate requiring three consecutive detailed 640x480 frames;
- synchronized 10 Hz camera and 100 Hz IMU/ground-truth mini recorder;
- idempotent Colosseum source patch and UE rebuild script.

## Runtime evidence

### Synthetic UCC IMU gate

| Test | Injection | AirSim result | Status |
|---|---:|---:|---|
| Static acceleration norm | 0 world acceleration | 9.806650 m/s² | Pass |
| Body X gyro | 0.1 rad/s | 0.1000000015 rad/s | Pass |
| World/body X acceleration delta | 1.0 m/s² | 1.0 m/s² | Pass |
| Ground-truth round trip | 1.0 m/s² | 1.0 m/s² | Pass |

### Native automatic-takeoff stability gate

| Metric | Result |
|---|---:|
| Duration / UCC injections | 75 s / 7,500 |
| Gazebo source / UCC rate | 250 Hz / 100 Hz |
| Receive-to-inject p95 latency | 2.80 ms |
| Deadline miss / timestamp regression / RPC failure | 0 / 0 / 0 |
| Maximum displacement | 162.487 m |
| Maximum horizontal speed | 14.513 m/s |
| Maximum relative altitude | 33.929 m |
| Final CSV-to-UCC GT position error | 0.00000732 m |

PX4 reported `Ready for takeoff` and `Takeoff detected`. UCC remained stable
beyond the prior crash point.

### Final native bridge gate

| Metric | Result |
|---|---:|
| Duration | 100.000 s |
| Gazebo states | 25,001 |
| UCC injections | 10,000 |
| Gazebo source rate | 249.9997 Hz |
| UCC injection rate | 99.9999 Hz |
| Mean / p95 latency | 1.617 / 2.658 ms |
| Missed deadlines | 0 |
| Duplicate/regressed/invalid states | 0 |
| RPC failures | 0 |

### Final 30-second mini dataset

| Metric | Result |
|---|---:|
| Camera | 300 / 300, 640x480, 10 Hz |
| IMU | 3,000 / 3,000, 100 Hz |
| Ground truth | 3,000 / 3,000, 100 Hz |
| Blank frames / maximum white ratio | 0 / 0.0 |
| Timestamp duplicates / regressions | 0 / 0 |
| Camera source period mean / p95 | 99.333 / 108.002 ms |
| IMU source period mean / p95 | 9.931 / 12.000 ms |
| Maximum quaternion norm error | 4.12e-8 |
| Motion displacement / max horizontal speed | 160.928 m / 14.554 m/s |
| Recorder errors | 0 |

Validated artifacts are outside Git under:

```text
$HOME/vio_sim_ws/artifacts/fixedwing_mvp/final_bridge_summary.json
$HOME/vio_sim_ws/artifacts/fixedwing_mvp/final_bridge_state.csv
$HOME/vio_sim_ws/artifacts/fixedwing_mvp/mini_dataset_pass/
```

## Reproduction

Use `tools/fixedwing/ucc_fixedwing_mvp_v1/README.md`. The required runtime path
is `08_run_gz_rc_cessna_ucc.sh`, `09_run_gazebo_airsim_bridge.sh`, PX4
`commander arm` / `commander takeoff`, and
`11_run_fixedwing_mini_dataset.sh`.

The external AirSim plugin is intentionally ignored by repository policy.
`12_patch_build_colosseum_fixedwing.sh` applies the project-owned runtime
changes and rebuilds the plugin after Colosseum installation.
