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

The MVP source path consumes PX4 telemetry in local NED/body FRD and estimates
acceleration by finite difference. The research path will consume Gazebo pose,
twist and acceleration in world ENU/body FLU, apply the explicit basis change,
and synchronize updates to simulation time. Gazebo remains the physics source
of truth in both paths.
