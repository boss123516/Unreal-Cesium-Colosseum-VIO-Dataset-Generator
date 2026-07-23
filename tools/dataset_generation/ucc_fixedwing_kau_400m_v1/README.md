# KAU 3D Buildings Fixed-Wing Dataset at 400 m

항공대 중심 반경 5 km의 LoD1 3D 건물을 배경으로 PX4/Gazebo 고정익
동역학 비행을 수행하고 Unreal/AirSim에서 Camera·IMU·GT를 3분간 기록한다.

## 계산과 렌더링의 역할

```text
PX4 autopilot + Gazebo aerodynamics/physics (250 Hz)
                    ↓ Gazebo Transport
Gazebo→AirSim bridge (100 Hz, ENU/FLU→NED/FRD)
                    ↓
Unreal/Cesium KAU 3D buildings + cam0 rendering
                    ↓
Camera 10 Hz + IMU 100 Hz + Ground Truth 100 Hz
```

고정익의 위치·자세·속도·가속도는 Unreal의 단순 이동 명령으로 만드는
것이 아니다. PX4와 Gazebo의 RC Cessna 공력·추진·관성 모델이 계산하고,
Unreal은 그 상태를 받아 항공대 3D 환경에서 렌더링과 센서 생성을 담당한다.

## 비행 계약

- Unreal 로컬 고도: `400 m`
- PX4/Gazebo 준비 상대고도: `100 m`
- 대기속도: `19 m/s`
- 경로: 항공대 원점을 접점으로 하는 좌·우 반경 `260 m` figure-eight
- 경로 최대 범위: 원점에서 전후 `260 m`, 좌우 `520 m`
- 고도 변화: `±10 m`
- PX4 Roll limit: `28°`
- 기록: `180 s`
- cam0: `640×480`, `10 Hz`, pitch `-45°`
- IMU/GT: `100 Hz`

두 원의 반경 260 m에서 19 m/s로 선회할 때 이상적인 정상선회 뱅크는
약 8°다. 오른쪽 원과 왼쪽 원을 연속 비행하므로 양방향 선회가 데이터에
포함된다.

## 수동 실행 순서

아래 명령은 각각 별도 터미널에서 실행한다.

### 터미널 1: 400 m 외부물리 프로필 적용

Unreal Play/PIE를 완전히 종료한 상태에서:

```bash
cd ~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/dataset_generation/ucc_fixedwing_kau_400m_v1
./00_apply_400m_profile.sh
```

### 터미널 2: 항공대 건물 서버와 Unreal

```bash
~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/geodata/kau_5km_buildings/launch_kau_high_altitude.sh -KAUNoBuildingCollision
```

Unreal에서 `Play`를 누르고 다음 로그를 확인한다.

```text
[KAU_BUILDINGS] GEOREFERENCE_READY
[KAU_BUILDINGS] READY ... collision=false physics_meshes=false
```

고도 400 m 데이터셋에서는 건물 충돌이 필요 없으므로 physics mesh만
비활성화해 타일 로딩 부하를 줄인다. 3D 지붕과 벽 렌더링은 그대로 유지된다.

### 터미널 3: PX4 + Gazebo 서버

```bash
cd ~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/fixedwing/ucc_fixedwing_mvp_v1
./08_run_gz_rc_cessna_ucc.sh
```

PX4 콘솔은 그대로 열어 둔다. `commander arm`이나 `takeoff`를 직접 입력하지
않는다.

### 터미널 4: Gazebo 고정익 추적 GUI

```bash
cd ~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/dataset_generation/ucc_fixedwing_kau_400m_v1
./02a_open_gazebo_follow_gui.sh
```

Gazebo GUI는 `rc_cessna_ucc_0`를 뒤쪽에서 추적한다. 이 화면을 함께 보여주면
기체의 실제 뱅크·피치·선회가 Gazebo에서 계산되고 있음을 확인할 수 있다.
기본 카메라 오프셋은 기체 기준 `(-5, 0, 2) m`다. 더 가까이 보려면:

```bash
GAZEBO_FOLLOW_X=-3 GAZEBO_FOLLOW_Z=1.5 ./02a_open_gazebo_follow_gui.sh
```

### 터미널 5: PX4 이륙 후 400 m Unreal 브리지

```bash
cd ~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/dataset_generation/ucc_fixedwing_kau_400m_v1
./01_prepare_takeoff.sh
```

`[PREPARE_READY]`, `[PREPARE_COMPLETE]` 다음 브리지의 `[RATE]` 로그가
계속 출력되어야 한다. 이 터미널은 데이터셋 종료 때까지 열어 둔다.

### 터미널 6: figure-eight 비행과 데이터셋 기록

```bash
cd ~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/dataset_generation/ucc_fixedwing_kau_400m_v1
./03_run_kau_dataset.sh
```

결과:

```text
~/vio_sim_ws/datasets/ucc_fixedwing_kau_400m_YYYYMMDD_HHMMSS
```

주요 결과 파일:

```text
flight_mission.json
timing_report.json
run_config.json
mav0/cam0/data/*.png
mav0/cam0/data.csv
mav0/imu0/data.csv
mav0/state_groundtruth_estimate0/data.csv
```

### 터미널 7: cam0 실시간 rqt_image_view

Unreal Play/PIE가 켜진 뒤 언제든 실행할 수 있다. 데이터셋 기록과 동시에
실행하면 실제 저장 카메라와 같은 `Drone1/cam0` 영상을 확인한다.

AirSim 코어는 msgpack-RPC를 사용하며 ROS 2 토픽을 자동 발행하지 않는다.
현재 PX4는 MAVLink, Gazebo 상태는 Gazebo Transport를 사용하므로 ROS 노드가
없을 때 `ros2 topic list`에 카메라가 없는 것은 정상이다. 아래 실행기가
cam0에 필요한 최소 ROS 2 publisher를 함께 시작한다.

```bash
cd ~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/dataset_generation/ucc_fixedwing_kau_400m_v1
./04_run_cam0_rqt.sh
```

ROS 2 토픽:

```text
/ucc/cam0/image_raw
/ucc/cam0/camera_info
```

기본 표시 속도는 recorder 부하를 줄이기 위해 5 Hz다. 10 Hz로 보려면:

```bash
CAM0_VIEW_HZ=10 ./04_run_cam0_rqt.sh
```

화면 역할은 다음처럼 구분된다.

```text
Gazebo GUI     : 공력·추진·관성에 따른 기체 운동
Unreal         : 항공대 3D 건물 환경과 외부 추적 시점
rqt_image_view : 실제 Drone1/cam0 센서 영상
```

## 교수님께 보여줄 포인트

- Gazebo GUI에서 고정익이 실제로 좌·우 뱅크하며 figure-eight 비행
- 브리지 터미널에서 Gazebo 상태 250 Hz, Unreal 주입 약 100 Hz 로그
- Unreal에서는 같은 자세로 항공대 3D 건물 위를 비행
- rqt_image_view에서는 같은 시각의 cam0 영상을 실시간 확인
- 데이터셋의 IMU/GT 가속도는 Gazebo physics component에서 직접 취득
- `timing_report.json`으로 1,800 camera / 18,000 IMU / 18,000 GT 검증

## 종료

데이터셋 생성 후 각 터미널에서 `Ctrl+C`로 브리지와 PX4/Gazebo를 종료한다.
Unreal Play/PIE도 정지한다.
