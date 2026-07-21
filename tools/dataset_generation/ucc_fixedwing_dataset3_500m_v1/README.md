# 고정익 3번째 데이터셋: 500 m 동적 비행

이 패키지는 PX4/Gazebo `rc_cessna_ucc`를 사용해 500 m 로컬 고도에서 3분간
직선과 완만한 좌·우 선회를 섞어 비행하고, AirSim/Unreal에서 Camera·IMU·GT를
동기 기록한다.

전체 구조와 설계 이유는
[`docs/FIXEDWING_PX4_GAZEBO_PIPELINE_HANDOVER_2026-07-21.md`](../../../docs/FIXEDWING_PX4_GAZEBO_PIPELINE_HANDOVER_2026-07-21.md)를
먼저 읽는다. 핵심 상태 경로는 ROS가 아니라
`Gazebo physics 250 Hz → Gazebo Transport → AirSim RPC 100 Hz`이며,
AirSim은 `ExternalPhysicsEngine`에서 Gazebo 운동을 렌더링·센서 생성에 사용한다.

## 비행·기록 계약

- 데이터셋 시작 로컬 고도: 500 m
- 합격 고도 범위: 450~550 m
- 실제 waypoint 명령 변화: 기준 고도 대비 -15~+15 m
- 목표 대기속도: 19 m/s
- PX4 최대 뱅크 제한: 28°
- PX4 mission 착륙 강제: 실행 중 `MIS_TKO_LAND_REQ=0`
- 데이터셋 검증 최대 절대 Roll: 35°
- 경로: 직진, 우측 완만한 이동, 직진, 좌측 교차 선회, 직진, 우측 교차 선회
- 기록 시간: 180 s
- `cam0`: 640×480 PNG, 10 Hz, body X=1 m, Pitch=-30°
- `Imu`: 100 Hz
- Ground Truth: 100 Hz
- 출력: EuRoC 유사 `mav0/cam0`, `mav0/imu0`,
  `mav0/state_groundtruth_estimate0`

19 m/s는 현재 RC Cessna의 PX4 설정 `FW_AIRSPD_MIN/TRIM/MAX=10/15/20 m/s`
안에서 최대속도에 1 m/s 제어 여유를 둔 값이다. 한화 기체의 속도는 해당
공력·추진 데이터와 PX4 airframe을 적용할 때 별도로 올린다.

## 중요한 고도 기준

Gazebo 기체는 먼저 PX4 상대고도 100 m에서 안정화한다. 그 다음 브리지를
시작하면 그 순간의 Gazebo 상태가 Unreal의 500 m 스폰 위치에 재기준화된다.
따라서 반드시 아래 순서를 지킨다. 브리지를 지상에서 먼저 켜면 이륙 상승량이
500 m에 더해져 데이터셋 고도 계약이 깨진다.

## 실행 순서

### 1. 500 m AirSim 프로필 적용

Unreal Play/PIE를 완전히 중지한 상태에서 실행한다.

```bash
cd tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1
./00_apply_500m_profile.sh
```

적용 후 Unreal Play/PIE를 다시 시작하고 Cesium 영상이 로딩될 때까지 기다린다.

### 2. PX4/Gazebo 시작

두 번째 터미널에서:

```bash
cd tools/fixedwing/ucc_fixedwing_mvp_v1
./08_run_gz_rc_cessna_ucc.sh
```

이 단계에서는 `commander arm` 또는 `commander takeoff`를 직접 입력하지 않는다.

### 3. 기준고도 이륙·안정화 및 500 m 브리지

세 번째 터미널에서:

```bash
cd tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1
./01_prepare_takeoff.sh
```

`[PREPARE_READY]`와 `[PREPARE_COMPLETE]`가 출력될 때까지 기다린다. 기체는 PX4
상대고도 100 m에서 안정화된다. 명령은 곧바로 `02_run_500m_bridge.sh`를 이어서
실행해 GCS heartbeat가 끊기지 않게 하며, 현재 Gazebo 상태를 Unreal의 500 m
위치에 고정하고 이후 운동을 100 Hz로 주입한다. 이 터미널은 계속 열어 둔다.
브리지는 기본적으로 시간 제한 없이 실행되며 `Ctrl+C`로 종료한다.

`02_run_500m_bridge.sh`는 브리지만 다시 연결해야 할 때 사용하는 독립 실행
스크립트다. 정상 첫 실행에서는 따로 입력하지 않는다.

Gazebo 화면에서도 기체를 계속 추적하려면 별도 터미널에서 다음을 실행한다.

```bash
./02a_open_gazebo_follow_gui.sh
```

GUI 카메라는 `rc_cessna_ucc_0` 뒤쪽 `(-12, 0, 4) m` 오프셋으로 고정된다.

### 4. 동적 비행과 데이터셋 기록

새 터미널에서:

```bash
./03_run_dataset3.sh
```

결과 예시는 다음과 같다.

```text
~/vio_sim_ws/datasets/ucc_fixedwing_dataset3_500m_YYYYMMDD_HHMMSS
```

## 자동 합격 조건

`timing_report.json.all_pass`는 다음 조건을 모두 검사한다.

- Camera 1,800장, 640×480, blank frame 0
- Camera 실제 프레임 간격 300 ms 이하, 캡처 스케줄 지연 500 ms 이하
- 같은 목표 시각의 Camera-GT 소스 시각 차이 200 ms 이하
- IMU 18,000개
- GT 18,000개
- timestamp 역행 없음
- quaternion norm 오차 1e-4 미만
- 전체 GT 고도 샘플 450~550 m
- 좌·우 각각 5° 이상 bank 존재
- 절대 Roll 35° 이하
- 절대 Roll 3° 이하 직진 샘플 비율 10% 이상

`flight_mission.json`에는 실제 시작 위도·경도·상대고도·방위각, 각 waypoint의
전방/측방 거리, 고도 변화, 적용 속도와 뱅크 제한을 저장한다.

실행기는 기록 전에 AirSim 설정의 `Drone1.Z=-500`, 실제 브리지 운동량, 절대
로컬 고도를 다시 확인한다. 그래서 브리지를 지상에서 먼저 시작해 500 m 기준이
잘못 잡힌 경우에는 기록을 시작하지 않는다.

기록 종료 후 기체는 마지막 waypoint에서 loiter한다. Gazebo/PX4와 브리지는
각 실행 터미널에서 `Ctrl+C`로 종료한다.
