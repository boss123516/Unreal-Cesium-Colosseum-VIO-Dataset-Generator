# Unreal-Cesium-Colosseum VIO Dataset Generator

> [!IMPORTANT]
> 이 페이지는 **`fixed_wing` 브랜치**의 README다. 현재 브랜치는 PX4/Gazebo
> 고정익 동역학을 Unreal/Cesium과 Colosseum 센서 계층에 연결하고, 500 m
> 로컬 고도에서 180초 Dataset 3을 생성·검증한 상태다.

## Fixed-wing branch: start here

처음 보는 경우 아래 순서로 읽는다.

1. [고정익 디렉터리 README](tools/fixedwing/README.md) — 구조, 빠른 실행과 주요 파일
2. [PX4–Gazebo–Colosseum 파이프라인 인수인계](docs/FIXEDWING_PX4_GAZEBO_PIPELINE_HANDOVER_2026-07-21.md) — 설계 이유와 전체 데이터 흐름
3. [고정익 통합 현황 및 실행 가이드](docs/FIXEDWING_INTEGRATION_STATUS.md) — 설치·실행·문제 해결
4. [공력 모델 기준서 PDF](docs/FIXEDWING_AERODYNAMIC_MODEL_REFERENCE.pdf) — 현행 기체 분석과 한화 데이터 교체 지침
5. [500 m Dataset 3 실행 패키지](tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1/README.md) — 실제 기록 절차

### 실제 고정익 파이프라인

이 브랜치는 Unreal에 고정익 3D 모델만 로드해 움직이는 구성이 아니다.

```text
                   비행·물리 폐루프

Gazebo 가상 센서 ───────────────► PX4 SITL
       ▲                            │
       │                            │ 모터·조종면 명령
       └──── Gazebo 공력·6DoF ◄─────┘
                    │
                    │ base_link 계산 상태 250 Hz
                    ▼
          UccKinematicsPublisher
                    │ Gazebo Transport
                    │ /ucc/fixed_wing/kinematics
                    ▼
         ENU/FLU → NED/FRD 변환
            위치 재기준화·최신값 선택
                    │ AirSim RPC 100 Hz
                    ▼
       Colosseum ExternalPhysicsEngine
          ├── Unreal/Cesium 렌더링
          ├── Camera 640×480, 10 Hz
          ├── IMU 100 Hz
          └── Ground Truth 100 Hz
```

- PX4가 비행 제어를 담당한다.
- Gazebo가 공력·추진·중력·충돌·6-DoF 운동을 계산한다.
- 250 Hz 상태는 ROS가 아닌 Gazebo Transport로 전달된다.
- AirSim은 물리를 다시 계산하지 않고 Gazebo 상태를 센서와 화면에 사용한다.
- 상태 흐름은 Gazebo → AirSim 단방향이다.
- 250 Hz 입력을 100 Hz로 주입할 때 이전 상태가 최신 상태로 대체되는 것은
  의도된 latest-state 선택이며 패킷 손실이 아니다.

### 검증된 결과

| 지표 | 결과 |
|---|---:|
| Gazebo source / AirSim injection | 249.9985 / 100.0017 Hz |
| Bridge deadline miss / RPC failure | 0 / 0 |
| 180초 Camera | 1,800장, 640×480, 10 Hz |
| IMU / Ground Truth | 각 18,000개, 100 Hz |
| Camera source gap max | 165.004 ms |
| Camera–GT source skew max | 135.003 ms |
| 로컬 고도 min / mean / max | 483.039 / 497.183 / 511.836 m |
| 속도 max | 19.500 m/s |
| Roll min / max | -23.297 / +28.374° |
| Dataset 3 최종 판정 | `all_pass: true` |

최종 데이터셋은 약 1.1 GB이므로 Git에 포함하지 않으며 로컬
`$HOME/vio_sim_ws/datasets/` 아래에 보관한다.

---

## Base project overview

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

The fixed-wing MVP first passed a 100-second native bridge gate and a synchronized
30-second mini-dataset gate. The current branch additionally passes the 180-second
500 m Dataset 3 contract with 1,800 camera frames and 18,000 IMU / ground-truth
samples. See `docs/FIXEDWING_PX4_GAZEBO_PIPELINE_HANDOVER_2026-07-21.md`.

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

## Project status on `fixed_wing`

- [x] Host environment verification
- [x] Unreal Engine 5.6 source acquisition
- [x] Unreal Engine dependency setup
- [x] Unreal project-file generation
- [x] Unreal Editor build and runtime verification
- [x] Cesium for Unreal integration
- [x] Colosseum External Physics integration
- [x] PX4/Gazebo fixed-wing 250 Hz state bridge
- [x] High-altitude local environment validation
- [x] Camera, IMU and Ground Truth recorder
- [x] 500 m, 180-second Dataset 3 validation
- [ ] Hanwha aircraft aerodynamic model and PX4 tuning
- [ ] ROS 2 playback and VINS-Mono quantitative evaluation

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
