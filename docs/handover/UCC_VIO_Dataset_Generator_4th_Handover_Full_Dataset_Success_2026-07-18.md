# Unreal–Cesium–Colosseum 기반 고고도 VIO Dataset Generator
## 4차 개발 인수인계 문서 — Full Dataset 1회 생성 성공 시점

- **작성일:** 2026-07-18
- **Repository:** `boss123516/Unreal-Cesium-Colosseum-VIO-Dataset-Generator`
- **Unreal Project:** `sim/UCCVioDatasetSim`
- **개발 환경:** Ubuntu 24.04 / Unreal Engine 5.6 / Cesium for Unreal 2.28.0 / Colosseum·AirSim / Python
- **현재 도달 상태:** Camera·IMU·Ground Truth가 포함된 EuRoC-like 전체 Dataset 1회 생성 성공
- **중요 구분:** Dataset 파일 구조와 timestamp 수집에는 성공했지만, 일부 Camera image에 파란색 Cesium missing-tile/background가 나타나므로 아직 최종 benchmark-quality Dataset은 아님
- **다음 큰 단계:** Cesium tile streaming 문제 해결 후 Dataset 재생성 → VINS-Mono ROS2/Jazzy 실행 및 GT 정량 평가

---

# 1. 프로젝트 목적

본 프로젝트의 목적은 실제 지형 기반의 synthetic 고고도 VIO Dataset을 생성하는 것이다.

전체 구성은 다음과 같다.

```text
Cesium 실제 Terrain
        ↓
Unreal Engine 5.6
        ↓
Colosseum / AirSim Plugin
        ↓
SimpleFlight Drone1
        ↓
Dynamic 3D Flight
        ↓
Camera + IMU + Ground Truth 수집
        ↓
Simulation timestamp 기반 resampling
        ↓
Camera 10 Hz / IMU 100 Hz / GT 100 Hz
        ↓
EuRoC-like Dataset
        ↓
ROS 2 playback
        ↓
VINS-Mono 및 후속 VIO 알고리즘 평가
```

최종 목표는 다음 조건에 가까운 Dataset을 안정적으로 반복 생성하는 것이다.

```text
Camera         : Monocular RGB, 640×480, 10 Hz
IMU            : Angular velocity + Linear acceleration, 100 Hz
Ground Truth   : Position, Orientation, Velocity 등, 100 Hz
Flight speed   : 최대 30 m/s
Flight time    : 180 s
Environment    : Cesium 실제 지형
Motion         : Forward / Turn / Slalom / Climb / Descent
Output format  : EuRoC-like
```

---

# 2. 왜 Unreal + Cesium + Colosseum을 연결했는가

## 2.1 Unreal Engine

Unreal Engine은 다음 역할을 담당한다.

- 현실적인 rendering
- AirSim vehicle simulation 실행
- Camera sensor image 생성
- Cesium plugin 구동
- 고속 3D trajectory 시각화

## 2.2 Cesium for Unreal

Cesium은 실제 위성·지형 기반 환경을 제공한다.

고고도 VIO에서는 일반적인 실내 또는 단순 Gazebo 환경보다 다음이 중요하다.

- 넓은 지형 texture
- 도로·건물·산지 등 실제적인 spatial pattern
- 고도에 따른 texture density 변화
- VPS와 연결 가능한 geospatial coordinate
- 장거리 비행 시 반복되지 않는 environment

## 2.3 Colosseum / AirSim

Colosseum/AirSim은 다음 역할을 담당한다.

- Drone1 spawn 및 control
- Python RPC
- Camera API
- IMU API
- Kinematics / Ground Truth API
- NED coordinate 기반 비행 명령
- Image timestamp 및 sensor timestamp 제공

즉 세 시스템의 역할은 다음과 같이 분리된다.

```text
Unreal      : Simulation runtime + Rendering
Cesium      : 실제 지형
AirSim      : Vehicle + Sensor + RPC
```

---

# 3. 전체 진행 상태

```text
[완료] Unreal Engine 5.6 source build
[완료] UCCVioDatasetSim C++ project 생성
[완료] Cesium for Unreal 2.28.0 설치
[완료] Cesium ion Terrain 연결
[완료] 한국항공대학교 인근 georeference 설정
[완료] Colosseum v2.3.0 core build
[완료] UE 5.6 AirSim Plugin compatibility patch
[완료] AirLib linking 문제 해결
[완료] rpclib / glibc ABI mismatch 해결
[완료] Unreal Editor target build 성공
[완료] AirSimGameMode 실행
[완료] Drone1 spawn
[완료] Python RPC 연결
[완료] cam0 Camera API 확인
[완료] IMU API 확인
[완료] Ground Truth API 확인
[완료] Camera 10 Hz resampling 검증
[완료] IMU 100 Hz resampling 검증
[완료] cam0 mount position·rotation 수정
[완료] EuRoC-like recorder 구현
[완료] Dataset validator 구현
[완료] Dynamic 3D trajectory 구현
[완료] 전체 Camera·IMU·GT Dataset 1회 생성
[진행] Camera image의 Cesium missing-tile 문제 해결
[대기] 수정 후 최종 Dataset 재생성
[대기] ROS 2 Jazzy Dataset player
[대기] VINS-Mono ROS2 실행
[대기] Ground Truth 기반 ATE/RPE 평가
```

---

# 4. UE 5.6 + AirSim build 문제와 해결

## 4.1 UE 5.6 API compatibility

Colosseum v2.3.0의 AirSim Unreal Plugin은 UE 5.6에서 그대로 compile되지 않았다.

적용한 대표 patch:

```text
UWorld::LineBatcher
→ UWorld::GetLineBatcher(UWorld::ELineBatcherType::World)

UWorld::PersistentLineBatcher
→ UWorld::GetLineBatcher(UWorld::ELineBatcherType::WorldPersistent)
```

관련 파일:

```text
Plugins/AirSim/Source/PawnSimApi.cpp
Plugins/AirSim/Source/WorldSimApi.cpp
```

`TObjectPtr` 대응:

```cpp
APawn* pawn = player_controller->GetPawn().Get();
```

관련 파일:

```text
Plugins/AirSim/Source/SimHUD/SimHUD.cpp
```

## 4.2 AirLib linking 문제

초기 설정:

```text
CppCompileWithRpc
```

최종 설정:

```text
HeaderOnlyWithRpc
```

관련 파일:

```text
Plugins/AirSim/Source/AirSim.Build.cs
```

이를 통해 최종 Unreal module에서 AirLib implementation이 정상 link되었다.

## 4.3 `__isoc23_strtol` ABI mismatch

문제 구조:

```text
Ubuntu 24.04 host glibc로 build한 librpc.a
            ↕
UE 5.6 RockyLinux8 bundled toolchain
```

최종 오류:

```text
undefined symbol: __isoc23_strtol
```

해결:

- UE 5.6 bundled clang/toolchain으로 rpclib 재빌드
- 동일한 `librpc.a`를 AirLib 및 project plugin 경로에 배치
- `__isoc23_*` symbol이 없는지 확인

최종 build 결과:

```text
Link libUnrealEditor-AirSim.so
WriteMetadata UCCVioDatasetSimEditor.target
Result: Succeeded
```

---

# 5. Runtime 구성

## 5.1 Unreal Level

```text
Content/Maps/HighAltitudeCity
```

World Settings:

```text
GameMode Override = AirSimGameMode
HUD Class         = SimHUD
```

## 5.2 AirSim RPC

기본 RPC port:

```text
TCP 41451
```

확인:

```bash
ss -ltnp | grep 41451
```

Python 환경:

```text
~/vio_sim_ws/airsim_pyenv
```

PythonClient:

```text
~/vio_sim_ws/Colosseum/PythonClient
```

환경 설정 예시:

```bash
source ~/vio_sim_ws/airsim_pyenv/bin/activate
export PYTHONPATH="$HOME/vio_sim_ws/Colosseum/PythonClient:$PYTHONPATH"
```

---

# 6. Camera 최종 설정

## 6.1 Camera contract

```text
Vehicle          : Drone1
Camera name      : cam0
Image type       : Scene RGB
Resolution       : 640 × 480
Final rate       : 10 Hz
Mount position   : X=0.0, Y=0.0, Z=0.45 m
Mount rotation   : Roll=0°, Pitch=-45°, Yaw=0°
Timestamp        : simulation timestamp
```

중요:

- `45°`는 FOV가 아니라 Camera mount Pitch다.
- Camera는 기체 중심보다 아래쪽으로 0.45 m 위치한다.
- Camera는 전방 아래쪽 지형을 보도록 Pitch=-45°로 설정한다.
- HUD의 `ExternalCamera`는 dataset sensor인 `cam0`와 별개의 camera다.

## 6.2 초기 Camera 문제

초기에는 다음과 같은 잘못된 설정이 있었다.

```text
Pitch = 0°
Camera가 drone mesh 내부에 가까움
FOV를 45° 요구사항으로 오해
```

이 상태에서는:

- 하늘 위주 영상
- drone arm/body 노출
- 지면 texture 부족

문제가 있었다.

수정 후 cam0의 최종 mount는 다음과 같다.

```text
Translation = [0.0, 0.0, 0.45] m
Rotation    = [0°, -45°, 0°]
```

---

# 7. Sensor timing 검증

## 7.1 Native sensor output을 그대로 사용하지 않은 이유

AirSim Camera와 IMU의 실제 API update는 설정값과 정확히 일치하지 않았다.

측정 예:

```text
ClockSpeed 0.5:
Camera 평균 약 10 Hz지만 큰 jitter
IMU 약 666 Hz

ClockSpeed 0.1:
Camera 약 95 Hz
IMU 약 3333 Hz
```

따라서 native output을 직접 10 Hz / 100 Hz라고 가정하지 않았다.

## 7.2 Timestamp-grid resampling

Camera:

```text
Raw Camera polling
→ 100 ms target grid
→ 가장 가까운 source frame 선택
→ 10 Hz output
```

IMU:

```text
Raw IMU polling
→ 10 ms target grid
→ 가장 가까운 source sample 선택
→ 100 Hz output
```

Ground Truth:

```text
Raw state polling
→ 10 ms target grid
→ 100 Hz output
```

검증 결과:

```text
Camera 10 Hz resampling : PASS
IMU 100 Hz resampling   : PASS
Timestamp monotonic     : PASS
Shared epoch            : 구현 완료
```

---

# 8. 통합 Dataset recorder

Recorder는 다음 worker를 병렬로 실행한다.

```text
Camera worker
IMU worker
Ground Truth worker
Trajectory worker
```

## 8.1 Camera output

```text
mav0/cam0/data.csv
mav0/cam0/mapping.csv
mav0/cam0/data/<timestamp_ns>.png
```

## 8.2 IMU output

```text
mav0/imu0/data.csv
mav0/imu0/mapping.csv
```

저장 값:

```text
timestamp
angular velocity x/y/z
linear acceleration x/y/z
```

## 8.3 Ground Truth output

```text
mav0/state_groundtruth_estimate0/data.csv
mav0/state_groundtruth_estimate0/mapping.csv
```

저장 값:

```text
position
orientation quaternion
linear velocity
angular velocity
linear acceleration
latitude
longitude
altitude
```

## 8.4 Timestamp 보존

각 sensor는 두 timestamp를 모두 보존한다.

```text
target_timestamp_ns
source_timestamp_ns
timestamp_error_ns
```

따라서 resampling된 결과와 AirSim 원본 sensor timestamp 사이 오차를 추적할 수 있다.

---

# 9. Dataset directory 구조

전체 Dataset은 다음 구조로 생성된다.

```text
ucc_dynamic_3min_YYYYMMDD_HHMMSS/
├── camera_calibration.json
├── settings.snapshot.json 또는 run_config.json
├── run_summary.json
├── validation_report.json
├── DRONE_LEFT_HOVERING.txt
└── mav0/
    ├── cam0/
    │   ├── data.csv
    │   ├── mapping.csv
    │   └── data/
    │       └── <target_timestamp_ns>.png
    ├── imu0/
    │   ├── data.csv
    │   └── mapping.csv
    └── state_groundtruth_estimate0/
        ├── data.csv
        └── mapping.csv
```

기록 중 중단된 Dataset에는 다음 marker가 남는다.

```text
.recording_incomplete
```

이 marker가 있는 Dataset은 평가에 사용하지 않는다.

확인:

```bash
find ~/vio_sim_ws/datasets \
  -maxdepth 2 \
  -name '.recording_incomplete' \
  -print
```

---

# 10. Dynamic 3-minute flight

최종 Dataset 수집용으로 설계한 v7 조건:

```text
ClockSpeed        : 1.0
Maximum speed     : 30 m/s
Recording time    : 180 s
Camera            : 10 Hz
IMU               : 100 Hz
Ground Truth      : 100 Hz
Landing           : 없음
End state         : hover
```

Trajectory에는 다음 motion이 포함된다.

```text
Takeoff
→ Forward acceleration
→ Climb
→ 3D Slalom
→ Broad left turn
→ Climb / Descent oscillation
→ Broad right turn
→ S-turn
→ 180 s 종료
→ cancelLastTask
→ hover
→ Dataset finalize
```

목표 sample count:

```text
Camera       : 1,800 frames
IMU          : 18,000 samples
Ground Truth : 18,000 samples
```

종료 시 의도된 동작:

```text
180초 도달
→ 현재 movement task 취소
→ hover
→ CSV/JSON flush
→ validation 수행
→ process 종료
```

Landing, disarm, API control release는 자동으로 하지 않는다.

나중에 안전 복구가 필요하면 별도 script를 사용한다.

```bash
source ~/vio_sim_ws/airsim_pyenv/bin/activate
export PYTHONPATH="$HOME/vio_sim_ws/Colosseum/PythonClient:$PYTHONPATH"

./02_safe_recover.py Drone1
```

---

# 11. 전체 Dataset 1회 생성 성공

현재 단계에서 다음 조건을 갖춘 전체 Dataset을 한 번 생성하는 데 성공했다.

```text
Camera PNG sequence 존재
Camera timestamp CSV 존재
IMU 100 Hz CSV 존재
Ground Truth CSV 존재
EuRoC-like directory 구조 생성
Run summary 생성
Validation report 생성
```

즉 다음 pipeline은 실제로 동작했다.

```text
AirSim dynamic flight
→ Camera capture
→ IMU polling
→ Ground Truth polling
→ Timestamp alignment
→ File serialization
→ Dataset finalize
```

이는 프로젝트에서 중요한 milestone이다.

하지만 이 성공은 두 종류로 구분해야 한다.

## 11.1 성공한 부분

```text
Sensor API 연결
Recorder concurrency
Timestamp grid 생성
Camera·IMU·GT 동기화
파일 생성
Dataset directory 규격
3분 실행 후 finalize
```

## 11.2 아직 통과하지 못한 부분

```text
모든 Camera frame의 Cesium terrain rendering 품질
파란색 missing-tile/background 제거
정확한 AGL 검증
정확한 Camera–IMU extrinsic 고정
VINS-Mono 결과 검증
GT 기반 ATE/RPE 평가
```

따라서 현재 Dataset은 다음 용도로는 사용할 수 있다.

```text
ROS topic playback smoke test
Feature tracker 연결 확인
IMU/Camera timestamp pipeline 확인
VINS 초기화 가능 여부 확인
Dataset loader 개발
```

하지만 다음 용도로는 아직 부적합하다.

```text
최종 VIO 성능 benchmark
정확한 ATE/RPE 비교
실제 Camera 환경을 대표하는 정식 Dataset
논문 실험 결과
```

---

# 12. 현재 Camera 영상의 파란색 배경 문제

## 12.1 관찰된 현상

생성된 일부 Camera frame에서:

- 화면의 넓은 영역이 파란색
- 경계가 자연스러운 하늘·지평선 형태가 아님
- 각진 Cesium tile 형태로 영역이 빠짐
- 지형 mesh의 side 또는 background가 노출됨

현상이 확인되었다.

이는 Camera exposure나 PNG encoding 문제가 아니라 다음 가능성이 높다.

```text
Cesium tile이 cam0 view 기준으로 load되지 않음
또는
30 m/s 이동 중 tile streaming이 Camera를 따라오지 못함
```

## 12.2 원인 가설

현재 Cesium은 기본적으로 Player Camera 또는 ExternalCamera를 중심으로 tile selection을 수행할 수 있다.

하지만 Dataset sensor는:

```text
Drone1 / cam0
```

이고 HUD의 camera는:

```text
ExternalCamera
```

이므로 둘은 다르다.

결과적으로:

```text
ExternalCamera 주변 tile은 load
cam0가 바라보는 지형 tile은 누락 또는 지연
```

될 수 있다.

## 12.3 고도를 높이면 해결되는가

근본 해결책은 아니다.

높은 고도에서는 low-LOD parent tile이 넓은 영역을 덮기 때문에 일시적으로 문제가 덜 보일 수 있다.

하지만:

```text
cam0가 Cesium tile-selection camera로 등록되지 않음
```

이라는 구조가 유지되면 다시 발생할 수 있다.

또한 고도를 높이면 Camera footprint가 넓어져 더 많은 tile이 필요할 수도 있다.

따라서 고도 변경보다 Camera registration과 tile preload가 우선이다.

---

# 13. Cesium cam0 bridge 해결 방향

현재 준비한 해결 패키지:

```text
ucc_cesium_cam0_bridge_v1.zip
```

목표:

```text
AirSim cam0 world pose
        ↓ 매 Unreal Tick
CesiumCameraManager.AdditionalCameras
        ↓
cam0 view 기반 tile selection
```

추가로 30 m/s 비행에 대응하기 위해:

```text
현재 cam0 camera
+ 진행 방향 90 m 앞 preload camera
```

를 등록한다.

계산:

```text
30 m/s × 3 s = 90 m
```

적용 예정 tileset 옵션:

```text
PreloadAncestors               = true
PreloadSiblings                = true
ForbidHoles                    = true
MaximumScreenSpaceError        = 16
MaximumSimultaneousTileLoads   = 64
MaximumCachedBytes             = 2 GiB
LoadingDescendantLimit         = 20
EnableFrustumCulling           = true
EnableFogCulling               = false
EnableOcclusionCulling         = false
```

패치 적용 흐름:

```bash
cd ~/Downloads
unzip -o ucc_cesium_cam0_bridge_v1.zip
cd ucc_cesium_cam0_bridge_v1

chmod +x *.sh

./01_apply_cam0_bridge.sh
./02_build_editor.sh
```

Editor를 다시 실행한 뒤 Output Log에서 확인:

```text
[CESIUM_CAM0_BRIDGE] selected actor=...
[CESIUM_CAM0_BRIDGE] READY
```

그 다음 정지 cam0 probe:

```bash
./03_run_static_cam0_probe.sh
```

Probe가 통과하면 30 m/s Dataset을 재생성한다.

---

# 14. AGL 문제

AirSim NED z와 Cesium 실제 지표면 AGL은 동일하지 않다.

관찰 예:

```text
Initial NED z = 약 74.9 m
```

이 값은 지표면 기준 74.9 m 고도라는 뜻이 아니다.

현재 비행 명령은 상대 상승량을 사용한다.

```text
target_z = current_z - climb
```

이는:

```text
현재 NED 위치에서 climb만큼 상승
```

만 보장한다.

원하는 목표는:

```text
실제 Cesium terrain 기준 AGL
```

이다.

향후 해결 후보:

1. Launch 지점 ground reference를 고정
2. Downward Distance Sensor
3. Unreal raycast
4. Cesium terrain height와 geodetic altitude 비교

현재 첫 Dataset은 trajectory·sensor pipeline 검증용이므로 상대 NED 상승을 사용했다.

---

# 15. VINS-Mono ROS2 연동 준비 상태

사용할 Repository:

```text
https://github.com/boss123516/VINS-MONO-ROS2
```

현재 PC 환경:

```text
Ubuntu 24.04
ROS 2 Jazzy
```

초기에 준비한 install script는 Humble을 기본값으로 작성되어 다음 오류가 발생했다.

```text
[ERROR] ROS 2 humble is not installed.
Expected: /opt/ros/humble/setup.bash
```

이 PC에서는 Humble을 설치하지 않고 Jazzy를 사용해야 한다.

변경:

```bash
cd ~/Downloads/ucc_vins_mono_ros2_full_run_v1

sed -i \
  's/ROS_DISTRO_EXPECTED:-humble/ROS_DISTRO_EXPECTED:-jazzy/g' \
  00_install_clone_patch_build.sh \
  02_run_vins.sh
```

향후 실행 순서:

```bash
./00_install_clone_patch_build.sh
./01_validate_prepare_dataset.sh <DATASET_ROOT>
./02_run_vins.sh <DATASET_ROOT>
```

첫 VINS test 설정:

```text
Camera topic       : /cam0/image_raw
IMU topic          : /imu0
Camera rate        : 10 Hz
IMU rate           : 100 Hz
Loop closure       : OFF
estimate_extrinsic : 2
estimate_td        : 0
```

현재 파란 tile Dataset은 smoke test에는 사용할 수 있지만 최종 성능 평가에는 사용하지 않는다.

---

# 16. 주요 script/package 이력

## Build / Runtime

```text
ucc_vio_next_step_scripts.zip
ucc_vio_runtime_next.zip
05_diagnose_airsim_rpc.sh
```

## Sensor timing

```text
ucc_vio_sensor_validation.zip
ucc_vio_resampling_validation.zip
```

## Camera contract

```text
ucc_vio_camera_contract.zip
ucc_vio_camera_contract_v2.zip
ucc_vio_camera_mount_fix.zip
ucc_vio_bottom_gimbal_camera.zip
```

## Recorder / Flight

```text
ucc_vio_dataset_integration.zip
ucc_vio_dataset_integration_v2.zip
ucc_vio_dataset_flight_test_v3.zip
ucc_vio_dataset_flight_test_v4_z045.zip
ucc_vio_dataset_flight_test_v5_visible_safe.zip
ucc_vio_dynamic_high_altitude_v6.zip
ucc_vio_dynamic_3min_v7.zip
```

## Cesium tile fix

```text
ucc_cesium_cam0_bridge_v1.zip
```

## VINS-Mono integration

```text
ucc_vins_mono_ros2_full_run_v1.zip
```

---

# 17. 현재 유효한 성과

## Runtime

```text
Unreal Editor build          : PASS
AirSim Plugin load           : PASS
Drone1 spawn                 : PASS
RPC port 41451               : PASS
Camera API                   : PASS
IMU API                      : PASS
Ground Truth API             : PASS
```

## Camera contract

```text
cam0 only                    : PASS
Resolution 640×480           : PASS
Mount Z=0.45 m               : PASS
Pitch=-45°                   : PASS
Timestamp recording          : PASS
```

## Timing

```text
Camera 10 Hz grid            : PASS
IMU 100 Hz grid              : PASS
GT 100 Hz grid               : PASS
Monotonic timestamp          : PASS
Source/target timestamp save : PASS
```

## Dataset

```text
Camera PNG output            : PASS
Camera CSV                   : PASS
IMU CSV                      : PASS
Ground Truth CSV             : PASS
Run summary                  : PASS
Validation report            : PASS
Full Dataset 1회 생성        : PASS
```

## 아직 미완료

```text
Camera blue missing-tile 제거
정확한 AGL
정확한 Camera–IMU extrinsic
VINS-Mono Jazzy build
VINS initialization
GT 기반 정량 평가
```

---

# 18. 다음 작업 순서

## Step 1. Cesium cam0 bridge 적용

```text
AirSim cam0를 CesiumCameraManager에 등록
90 m look-ahead camera 추가
ForbidHoles / preload / cache 설정
```

## Step 2. Static cam0 probe

```text
15초 정지 Camera test
파란색 영역 비율 확인
저장 frame 직접 시각 확인
```

## Step 3. 짧은 dynamic test

```text
30 m/s
20~30초
Camera+IMU+GT 기록
```

파란색 missing tile이 없는지 확인한다.

## Step 4. Full 3-minute Dataset 재생성

```text
30 m/s
180 s
Camera 1,800장
IMU 18,000개
GT 18,000개
```

## Step 5. VINS-Mono ROS2 Jazzy build

```text
Ubuntu 24.04
ROS 2 Jazzy
Repository main branch
```

## Step 6. UCC Dataset playback

```text
/cam0/image_raw
/imu0
```

를 original timestamp 순서로 publish한다.

## Step 7. VINS 결과 확인

```text
feature_tracker
vins_estimator
vins_result_no_loop.csv
```

## Step 8. Ground Truth 평가

GT CSV를 TUM format으로 변환하고 다음을 수행한다.

```text
evo_ape
evo_rpe
trajectory alignment
```

---

# 19. 유지해야 할 원칙

1. Camera의 `45°`는 FOV가 아니라 mount Pitch다.
2. Camera mount는 `[0.0, 0.0, 0.45] m`다.
3. Camera output은 `cam0 Scene` 하나만 사용한다.
4. HUD의 `ExternalCamera`와 Dataset `cam0`를 혼동하지 않는다.
5. Camera native frequency를 직접 신뢰하지 않는다.
6. IMU native frequency를 직접 100 Hz라고 가정하지 않는다.
7. Simulation timestamp grid로 resampling한다.
8. Source timestamp와 target timestamp를 모두 저장한다.
9. `.recording_incomplete` Dataset은 사용하지 않는다.
10. 파란색 missing tile이 있는 Dataset은 정식 benchmark로 사용하지 않는다.
11. Dataset 구조 성공과 영상 품질 성공을 구분한다.
12. VINS 첫 test에서는 loop closure를 끈다.
13. 최종 VINS 평가에서는 정확한 Camera–IMU extrinsic을 사용한다.
14. AirSim NED z를 실제 AGL로 단정하지 않는다.
15. 30 m/s 비행에서는 Cesium look-ahead tile preload가 필요하다.

---

# 20. 현재 상태 한 줄 요약

> **Unreal–Cesium–AirSim 환경에서 Camera 10 Hz, IMU 100 Hz, Ground Truth 100 Hz를 포함한 EuRoC-like 전체 Dataset을 한 번 생성하는 데 성공했으며, 현재 남은 핵심 문제는 cam0 영상의 Cesium missing-tile 파란색 배경을 제거한 뒤 동일 Dataset을 재생성하고 VINS-Mono ROS2/Jazzy로 검증하는 것이다.**
