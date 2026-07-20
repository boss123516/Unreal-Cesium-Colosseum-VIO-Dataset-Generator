# Unreal-Cesium-Colosseum VIO Dataset Generator

A public research repository integrating Unreal Engine,
Cesium for Unreal, and Colosseum to generate synchronized
high-altitude Camera, IMU, and Ground Truth datasets for
Visual-Inertial Odometry evaluation.

## System architecture

    Cesium 3D Tiles environment
                |
          Unreal Engine
                |
           Colosseum
     vehicle physics/sensors
                |
      Trajectory controller
                |
     Camera / IMU / Ground Truth
                |
            VIO dataset

## Core components

### Unreal Engine 5.6

- Simulation runtime
- Sensor-image rendering
- Project and level management

### Cesium for Unreal

- Georeferenced terrain
- 3D Tiles streaming
- High-altitude virtual environment

### Colosseum

Colosseum is an AirSim-compatible simulation layer providing:

- Multirotor vehicle dynamics
- SimpleFlight control
- Camera API
- IMU API
- Ground Truth state API

For the fixed-wing path, Colosseum runs `ExternalPhysicsEngine`: PX4/Gazebo
produces the aircraft motion while the existing `cam0`, AirSim IMU model, RPC
surface and dataset recorder remain in use. The current MVP tooling is under
`tools/fixedwing/ucc_fixedwing_mvp_v1`. The complete implementation status,
runtime procedure and troubleshooting guide are in
`docs/FIXEDWING_INTEGRATION_STATUS.md`.

The fixed-wing MVP has passed a 100-second native bridge gate and a synchronized
30-second dataset gate with 300 camera frames, 3,000 IMU samples and 3,000
ground-truth samples. See `docs/FIXEDWING_MVP_STATUS_2026-07-20.md`.

### Project-owned components

This repository will implement:

- Environment setup automation
- Unreal, Cesium, and Colosseum integration
- Flight trajectory generation
- Camera and IMU sampling
- Ground Truth recording
- Coordinate-frame conversion
- Calibration export
- Dataset integrity validation
- EuRoC and ROS 2 dataset export

## Target dataset

- Monocular Camera: 640 x 480 at 10 Hz
- IMU: 100 Hz
- Ground Truth pose and velocity
- Camera intrinsic parameters
- Camera-IMU extrinsic transformation
- Simulation and trajectory metadata

## Project status

- [x] Host environment verification
- [x] Unreal Engine 5.6 source acquisition
- [x] Unreal Engine dependency setup
- [x] Unreal project-file generation
- [ ] Unreal Editor build verification
- [ ] Cesium for Unreal integration
- [ ] Colosseum integration
- [ ] High-altitude environment validation
- [ ] Camera and IMU recorder
- [ ] Dataset validation pipeline

## Repository scope

This repository contains project-owned integration code,
configuration, documentation, Unreal project files, and
dataset-generation tools.

It does not contain:

- Unreal Engine source code or binaries
- Cesium ion access tokens
- Map-provider API credentials
- Downloaded 3D Tiles caches
- Generated large-scale datasets
- Restricted third-party assets

## Usage notice

This is a public research repository.

No open-source license is currently granted for project-owned
content unless explicitly stated otherwise.
