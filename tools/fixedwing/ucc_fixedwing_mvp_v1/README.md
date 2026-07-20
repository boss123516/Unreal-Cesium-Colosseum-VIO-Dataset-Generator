# UCC fixed-wing MVP v1

This package runs PX4 fixed-wing control and Gazebo Cessna dynamics as the
physics source for UCC. A native Gazebo plugin publishes link kinematics at
250 Hz, the bridge converts ENU/FLU to NED/FRD and injects UCC at 100 Hz, and
the recorder writes synchronized 640x480 camera, IMU and ground truth data.

## Validated result

- Gazebo state: 250 Hz
- UCC `simSetKinematics`: 100 Hz
- camera: 640x480 at 10 Hz
- IMU and ground truth: 100 Hz
- fixed-wing visual: PX4 `rc_cessna` mesh imported into Unreal
- 100-second live bridge: 10,000 injections, no deadline miss or RPC failure
- 30-second mini dataset: 300 camera, 3,000 IMU and 3,000 GT samples, all gates pass

The recorder warms `cam0` until three consecutive detailed Cesium frames are
available. Recording never starts on AirSim's initial blank render target.

## 1. Preflight and Python dependencies

```bash
cd tools/fixedwing/ucc_fixedwing_mvp_v1
./00_preflight.sh
./03_install_bridge_python_deps.sh
```

`AirSim RPC` is only a warning while Unreal is stopped.

## 2. Install PX4/Gazebo and build the native state plugin

```bash
./05_setup_px4_gazebo.sh
./07_build_gazebo_kinematics_plugin.sh
```

The PX4 setup uses the official PX4 Ubuntu installer and may request the local
sudo password. The validated versions are PX4 v1.17.0 and Gazebo Harmonic
8.14.0.

## 3. Patch/build the installed Colosseum plugin

Install the external AirSim/Colosseum plugin into the Unreal project first,
then stop Unreal and run:

```bash
./12_patch_build_colosseum_fixedwing.sh
```

The patch is idempotent. It adds the External Physics body lock that prevents
the sensor-thread crash, reuses the proven built-in capture for the `cam0` API
alias, hides the carrier pawn from sensor captures, and activates the Cessna
visual in External Physics mode.

Set `UCC_FIXEDWING_VISUAL=0` only for a visual A/B diagnostic. The normal path
loads `/Game/FixedWing/SM_RCCessna`.

## 4. Import the fixed-wing visual

The imported Unreal assets are committed, so this step is only needed after
changing the source FBX:

```bash
./10_import_fixedwing_visual.sh
```

The source is `assets/rc_cessna_body.fbx`. The runtime visual is collision-free
and excluded from sensor SceneCapture while remaining visible to the player
and chase camera.

## 5. Apply the UCC profile

```bash
./01_apply_external_physics_profile.sh --profile validation
```

The profile selects `ExternalPhysicsEngine`, configures `Drone1/cam0` at
640x480 with X=1.0 m and Pitch=-30 degrees, disables motion blur and zeros IMU
noise for the integration gate. Restart Unreal after applying it.

To prove the UCC IMU path independently:

```bash
mkdir -p "$HOME/vio_sim_ws/artifacts/fixedwing_mvp/synthetic_gate"
./02_run_synthetic_imu_probe.sh \
  --output "$HOME/vio_sim_ws/artifacts/fixedwing_mvp/synthetic_gate/imu_validation.json"
```

## 6. Start the fixed-wing simulation

Start UCC on `HighAltitudeCity`, wait for AirSim RPC port 41451, then start
PX4/Gazebo in a second terminal:

```bash
./08_run_gz_rc_cessna_ucc.sh
```

Start the native bridge in a third terminal:

```bash
./09_run_gazebo_airsim_bridge.sh \
  --duration-sec 100 \
  --summary "$HOME/vio_sim_ws/artifacts/fixedwing_mvp/bridge_summary.json" \
  --state-log "$HOME/vio_sim_ws/artifacts/fixedwing_mvp/bridge_state.csv"
```

After PX4 reports `Ready for takeoff`, enter these commands at the `pxh>`
prompt:

```text
commander arm
commander takeoff
```

The bridge sends the GCS heartbeat required for unattended PX4 preflight and
stops on stale Gazebo state, invalid state, timestamp regression, repeated UCC
RPC failure or the requested duration.

## 7. Record the 30-second mini dataset

While the aircraft and bridge are running:

```bash
./11_run_fixedwing_mini_dataset.sh \
  --output "$HOME/vio_sim_ws/artifacts/fixedwing_mvp/mini_dataset" \
  --duration-sec 30
```

Success removes `.recording_incomplete` and writes `timing_report.json` with
`all_pass: true`. The output uses an EuRoC-like `mav0/cam0`, `mav0/imu0` and
`mav0/state_groundtruth_estimate0` layout.

## 8. Tests

```bash
export PYTHONPATH="/usr/lib/python3/dist-packages:$HOME/vio_sim_ws/Colosseum/PythonClient"
"$HOME/vio_sim_ws/airsim_pyenv/bin/python" -m unittest discover \
  -s . -p 'test_*.py' -v
```

## Frame contract

The Gazebo plugin publishes this ordered `gz.msgs.Double_V` contract:

```text
version, simulation timestamp,
position ENU,
body-FLU to world-ENU quaternion xyzw,
linear velocity ENU,
angular velocity FLU,
linear acceleration ENU,
angular acceleration FLU
```

The bridge outputs world NED/body FRD and reanchors the first received Gazebo
position to the UCC spawn origin. Linear acceleration comes directly from the
Gazebo link component; it is not a finite difference and does not duplicate
gravity.

`04_run_px4_airsim_bridge.sh` remains as the earlier MAVLink MVP path. Use the
native `08` + `09` path for dataset generation.
