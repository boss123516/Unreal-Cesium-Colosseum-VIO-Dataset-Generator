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
