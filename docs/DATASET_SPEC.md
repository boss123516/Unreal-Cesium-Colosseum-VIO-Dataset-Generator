# Dataset Specification

## Camera

- Sensor name: cam0
- Camera model: pinhole
- Resolution: 640 x 480
- Target frequency: 10 Hz
- Image format: PNG
- Timestamp unit: nanoseconds
- Timestamp source: simulation time

## IMU

- Sensor name: Imu
- Target frequency: 100 Hz
- Angular velocity unit: rad/s
- Linear acceleration unit: m/s^2
- Timestamp unit: nanoseconds
- Timestamp source: simulation time

## Ground Truth

Each Ground Truth sample should contain:

- Position
- Orientation quaternion
- Linear velocity
- Angular velocity
- Simulation timestamp

## Required metadata

- Camera intrinsic matrix
- Distortion coefficients
- Camera-IMU extrinsic transformation
- Coordinate-frame definitions
- Geospatial origin
- Environment parameters
- Flight trajectory parameters
- Simulator dependency versions

## Timing requirements

- Camera timestamp intervals should be approximately 100 ms.
- IMU timestamp intervals should be approximately 10 ms.
- Timestamps must increase monotonically.
- IMU data must cover the entire Camera time interval.
