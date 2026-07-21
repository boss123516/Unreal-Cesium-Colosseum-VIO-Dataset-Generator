# Fixed-wing simulation and VIO dataset pipeline

이 디렉터리는 PX4 SITL과 Gazebo Harmonic에서 계산한 고정익 운동을
Colosseum/AirSim과 Unreal/Cesium에 전달하고, Camera·IMU·Ground Truth
데이터셋으로 기록하는 구현을 관리한다.

현재 `fixed_wing` 브랜치에서는 PX4 기본 RC Cessna를 사용한 통합 파이프라인과
500 m 로컬 고도 Dataset 3 생성까지 완료했다.

## 핵심 구조

이 구현은 Unreal에 고정익 메시만 올려 임의로 이동시키는 방식이 아니다.

```text
                    비행·물리 폐루프

Gazebo 가상 센서 ───────────────► PX4 SITL
       ▲                            │
       │                            │ 모터·조종면 명령
       └──── Gazebo 공력·6DoF ◄─────┘
                    │
                    │ base_link 상태 250 Hz
                    ▼
          UccKinematicsPublisher
                    │ Gazebo Transport
                    │ /ucc/fixed_wing/kinematics
                    ▼
         gazebo_airsim_bridge.py
          - 21-field 계약 검사
          - ENU/FLU → NED/FRD
          - 첫 상태 위치 재기준화
          - 최신 상태 선택
                    │ AirSim RPC 100 Hz
                    ▼
       Colosseum ExternalPhysicsEngine
          ├── Unreal/Cesium 렌더링
          ├── Camera 640×480, 10 Hz
          ├── IMU 100 Hz
          └── Ground Truth 100 Hz
```

반드시 구분해야 할 사항:

- 비행 제어는 PX4가 담당한다.
- 공력·추진·중력·충돌·6-DoF 적분은 Gazebo가 담당한다.
- 250 Hz 상태는 PX4가 ROS로 보내는 값이 아니라 Gazebo 링크 계산 결과다.
- `/ucc/fixed_wing/kinematics`는 ROS가 아닌 Gazebo Transport 토픽이다.
- AirSim은 `ExternalPhysicsEngine`에서 Gazebo 상태를 사용하며 물리를 다시
  계산하지 않는다.
- 실시간 상태 흐름은 Gazebo → AirSim 단방향이다.
- 250 Hz 입력 중 100 Hz 주입 전에 더 최신 상태로 교체된 값은 의도된
  latest-state 선택이며 패킷 손실이 아니다.

전체 설계와 판단 근거는
[고정익 PX4–Gazebo–Colosseum 파이프라인 인수인계](../../docs/FIXEDWING_PX4_GAZEBO_PIPELINE_HANDOVER_2026-07-21.md)를
먼저 참고한다.

## 디렉터리 구성

| 경로 | 역할 |
|---|---|
| [`ucc_fixedwing_mvp_v1/`](ucc_fixedwing_mvp_v1/) | PX4/Gazebo 실행, 250 Hz 네이티브 publisher, 좌표 변환, AirSim 브리지, Cessna 시각 패치 |
| [`../dataset_generation/ucc_fixedwing_dataset3_500m_v1/`](../dataset_generation/ucc_fixedwing_dataset3_500m_v1/) | 500 m 자동 이륙·안정화·mission·180초 데이터 기록 패키지 |
| [`../../docs/FIXEDWING_INTEGRATION_STATUS.md`](../../docs/FIXEDWING_INTEGRATION_STATUS.md) | 통합 현황과 기본 실행·문제 해결 가이드 |
| [`../../docs/FIXEDWING_AERODYNAMIC_MODEL_REFERENCE.md`](../../docs/FIXEDWING_AERODYNAMIC_MODEL_REFERENCE.md) | 현행 공력 모델 분석과 한화 기체 데이터 교체 지침 |
| [`../../docs/FIXEDWING_AERODYNAMIC_MODEL_REFERENCE.pdf`](../../docs/FIXEDWING_AERODYNAMIC_MODEL_REFERENCE.pdf) | 사진과 공력면 도해를 포함한 배포용 PDF |

## 현재 기준 모델

실행 환경은 다음 설정을 사용한다.

```text
PX4_SYS_AUTOSTART=4003
PX4_SIM_MODEL=gz_rc_cessna_ucc
PX4_GZ_WORLD=default
```

`rc_cessna_ucc`는 새로운 공력 모델이 아니다. PX4 기본 `rc_cessna`를 merge
include하고 `UccKinematicsPublisher`만 추가한다. 현행 질량·관성·공력·추진
값은 PX4의 `Tools/simulation/gz/models/rc_cessna/model.sdf`에서 가져온다.

상태 publisher는 Gazebo `PostUpdate`에서 다음 값을 직접 읽어 250 Hz로 보낸다.

- position과 quaternion
- linear/angular velocity
- linear/angular acceleration
- Gazebo simulation timestamp

가속도는 속도 유한차분으로 만들지 않고 Gazebo link component를 직접 사용한다.

## 500 m Dataset 3 빠른 실행

저장소 루트에서 실행한다.

```bash
cd /home/boss/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator
```

### 터미널 1: 500 m profile과 Unreal

```bash
./tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1/00_apply_500m_profile.sh

/home/boss/vio_sim_ws/UE_5.6/Engine/Binaries/Linux/UnrealEditor \
  "$PWD/sim/UCCVioDatasetSim/UCCVioDatasetSim.uproject" \
  /Game/Maps/HighAltitudeCity \
  -game -windowed -ResX=1280 -ResY=720 -nosplash -NoSound
```

### 터미널 2: PX4/Gazebo

```bash
./tools/fixedwing/ucc_fixedwing_mvp_v1/08_run_gz_rc_cessna_ucc.sh
```

이 단계에서 `commander arm`이나 `commander takeoff`를 직접 입력하지 않는다.

### 터미널 3: 자동 이륙과 500 m 브리지

```bash
./tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1/01_prepare_takeoff.sh
```

PX4 상대고도 약 100 m에서 안정화한 후 현재 Gazebo 위치를 Unreal의 500 m
스폰 위치에 재기준화한다. `[RATE] source=250.0Hz inject=100.0Hz`가 반복되는지
확인한다.

### 선택 터미널: Gazebo 추적 GUI

```bash
./tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1/02a_open_gazebo_follow_gui.sh
```

### 터미널 4: 180초 동적 기록

```bash
./tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1/03_run_dataset3.sh
```

상세한 실행 계약은
[Dataset 3 README](../dataset_generation/ucc_fixedwing_dataset3_500m_v1/README.md)를
참고한다.

## 최종 검증 결과

| 지표 | 결과 |
|---|---:|
| Gazebo source rate | 249.9985 Hz |
| AirSim injection rate | 100.0017 Hz |
| Bridge deadline miss / RPC failure | 0 / 0 |
| Camera | 1,800장, 640×480, 10 Hz |
| IMU / Ground Truth | 각 18,000개, 100 Hz |
| Camera source gap max | 165.004 ms |
| Camera–GT source skew max | 135.003 ms |
| 로컬 고도 min / mean / max | 483.039 / 497.183 / 511.836 m |
| 속도 max | 19.500 m/s |
| Roll min / max | -23.297 / +28.374° |
| 최종 판정 | `all_pass: true` |

최종 데이터셋은 크기 때문에 Git에 포함하지 않는다.

```text
$HOME/vio_sim_ws/datasets/ucc_fixedwing_dataset3_500m_20260721_172929/
$HOME/vio_sim_ws/datasets/fixed_wing_datasets_v1.zip
```

## 테스트

```bash
PYTHONPATH=/usr/lib/python3/dist-packages:$HOME/vio_sim_ws/Colosseum/PythonClient \
  $HOME/vio_sim_ws/airsim_pyenv/bin/python -m unittest discover \
  -s tools/fixedwing/ucc_fixedwing_mvp_v1 -p 'test_*.py' -v

PYTHONPATH=/usr/lib/python3/dist-packages:$HOME/vio_sim_ws/Colosseum/PythonClient \
  $HOME/vio_sim_ws/airsim_pyenv/bin/python -m unittest discover \
  -s tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1 \
  -p 'test_*.py' -v

shellcheck tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1/*.sh
```

현재 고정익 테스트 28개와 Dataset mission 테스트 5개가 통과한다.

## 한화 기체로 교체할 때

다음 파이프라인은 그대로 유지한다.

- PX4–Gazebo 폐루프
- Gazebo 250 Hz 물리 상태 publisher
- 21-field Gazebo Transport 계약
- ENU/FLU → NED/FRD 변환과 위치 재기준화
- AirSim External Physics 100 Hz 주입
- Camera/IMU/GT 기록 형식과 품질 gate

다음 항목은 한화 제공값으로 교체하고 다시 검증해야 한다.

- 질량, CG, 관성
- 날개 형상과 공력계수
- stall과 안정미계수
- 조종면 위치·제한·effectiveness
- 추진계 추력·토크 맵
- PX4 airframe, 속도 범위, trim과 gain
- Unreal 기체 외형 메시

현행 RC Cessna의 19 m/s나 공력계수를 한화 기체에 그대로 사용하면 안 된다.
상세 교체 항목과 검증 순서는
[공력 모델 기준서](../../docs/FIXEDWING_AERODYNAMIC_MODEL_REFERENCE.md)에 정리되어
있다.

## 현재 한계

- 현행 모델은 약 1.67 kg RC Cessna다.
- 단순 `LiftDrag` 기반 공력 모델을 사용한다.
- flap 출력은 현행 Gazebo flap joint controller에 연결되지 않았다.
- 무풍, IMU noise 비활성 조건에서 검증했다.
- 500 m는 Cesium 실제 지형 AGL이 아니라 PlayerStart 기준 로컬 고도다.
- Gazebo 250 Hz → AirSim 100 Hz 경로는 latest-state 방식이며 보간하지 않는다.

로컬 `sim/UCCVioDatasetSim/Content/CesiumSettings/`는 credential을 포함할 수 있어
Git에서 제외한다.
