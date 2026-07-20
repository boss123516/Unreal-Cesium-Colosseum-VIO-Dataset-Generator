# System Architecture

## Overview

    Cesium environment
            |
      Unreal Engine
            |
       Colosseum
            |
   Trajectory runner
            |
    Dataset recorder
            |
  Camera / IMU / GT data

## Unreal Engine

Unreal Engine runs the simulation world and renders the
virtual Camera images.

## Cesium for Unreal

Cesium provides georeferenced terrain and 3D Tiles for
constructing high-altitude environments.

## Colosseum

Colosseum is the AirSim-compatible vehicle and sensor layer.

Responsibilities:

- Multirotor dynamics
- SimpleFlight control
- Camera data
- IMU data
- Ground Truth vehicle state

## Project-owned layer

Responsibilities:

- Controlled flight trajectories
- Sensor sampling schedules
- Timestamp synchronization
- Dataset recording
- Frame conversion
- Calibration export
- Dataset validation

## Fixed-wing external-physics path

The fixed-wing MVP keeps Colosseum's camera, IMU, RPC and recorder layers but
replaces SimpleFlight motion integration with PX4 and Gazebo:

    PX4 fixed-wing control
              |
       Gazebo gz_rc_cessna
              |
       full kinematics state
              |
       UCC bridge and frames
              |
    Colosseum ExternalPhysicsEngine
          /                 \
    AirSim IMU          Unreal cam0

The validated source path uses a native Gazebo system plugin to publish pose,
twist and direct link acceleration at 250 Hz in world ENU/body FLU. The bridge
applies the explicit basis change, reanchors position and injects UCC at 100 Hz
in world NED/body FRD. Gazebo is the physics source of truth; PX4 supplies the
fixed-wing control surfaces and flight mode logic.

Colosseum's built-in front-center SceneCapture is configured as the `cam0`
alias for this path. This keeps the proven Cesium render path while preserving
the dataset API name, resolution and fixed-wing camera mount.
