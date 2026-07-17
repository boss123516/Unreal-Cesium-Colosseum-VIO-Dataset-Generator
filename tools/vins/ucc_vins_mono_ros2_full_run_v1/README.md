# UCC dataset → VINS-MONO-ROS2 전체 실행 v1

대상 환경:

```text
Ubuntu 22.04
ROS 2 Humble
Repository: boss123516/VINS-MONO-ROS2
Dataset: EuRoC-like Camera 10 Hz + IMU 100 Hz
```

이 패키지는 다음을 자동화한다.

```text
의존성 설치
→ Workspace 생성
→ Repository clone
→ UCC 전용 launch 추가
→ colcon build
→ Dataset 검사
→ Camera calibration 기반 UCC YAML 생성
→ /cam0/image_raw + /imu0 timestamp 순서 publish
→ feature_tracker + vins_estimator 실행
→ loop closure 없이 trajectory CSV 저장
```

## 요구 Dataset 구조

```text
DATASET_ROOT/
├── camera_calibration.json
└── mav0/
    ├── cam0/
    │   ├── data.csv
    │   └── data/
    │       └── <timestamp_ns>.png
    ├── imu0/
    │   └── data.csv
    └── state_groundtruth_estimate0/
        └── data.csv                 # 선택
```

Camera CSV:

```text
#timestamp [ns],filename
```

IMU CSV는 EuRoC-style 열 이름을 기본으로 사용한다.

```text
#timestamp [ns]
w_RS_S_x [rad s^-1]
w_RS_S_y [rad s^-1]
w_RS_S_z [rad s^-1]
a_RS_S_x [m s^-2]
a_RS_S_y [m s^-2]
a_RS_S_z [m s^-2]
```

---

# 1. 설치·Clone·Build

```bash
cd ~/Downloads
unzip -o ucc_vins_mono_ros2_full_run_v1.zip
cd ucc_vins_mono_ros2_full_run_v1

chmod +x *.sh tools/*.py
./00_install_clone_patch_build.sh
```

기본 Workspace:

```text
~/vins_mono_ros2_ws
```

# 2. Dataset 검사

실제 전체 Dataset root를 넣는다.

```bash
./01_validate_prepare_dataset.sh \
  ~/vio_sim_ws/datasets/ucc_dynamic_3min_YYYYMMDD_HHMMSS
```

# 3. VINS 실행

```bash
./02_run_vins.sh \
  ~/vio_sim_ws/datasets/ucc_dynamic_3min_YYYYMMDD_HHMMSS
```

동작:

```text
feature_tracker 실행
vins_estimator 실행
RViz2 실행
Camera+IMU 원래 timestamp 기반 실시간 재생
Dataset 종료 후 estimator flush
VINS 종료
결과 검사
```

# 4. 결과

```text
~/vins_mono_ros2_ws/output/ucc_latest/vins_result_no_loop.csv
```

확인:

```bash
./03_check_result.sh
```

---

# 유용한 실행 옵션

RViz 없이 실행:

```bash
USE_RVIZ=false ./02_run_vins.sh /path/to/dataset
```

2배속 재생:

```bash
PLAYBACK_RATE=2.0 ./02_run_vins.sh /path/to/dataset
```

첫 테스트는 정확성과 message loss 방지를 위해 기본 `1.0`을 권장한다.

Workspace를 다르게 만들 때:

```bash
VINS_WS=~/other_vins_ws ./00_install_clone_patch_build.sh
VINS_WS=~/other_vins_ws ./02_run_vins.sh /path/to/dataset
```

---

# 현재 Config 정책

첫 smoke test:

```text
estimate_extrinsic = 2
loop_closure       = 0
estimate_td        = 0
rolling_shutter    = 0
Camera rate        = 10 Hz
```

`estimate_extrinsic=2`는 camera–IMU extrinsic을 모르는 상태에서 온라인 추정한다. 따라서 Dataset 초반에 회전 motion이 있어야 한다.

최종 평가에서는 AirSim camera optical frame과 IMU/body frame의 정확한 변환을 계산해:

```text
estimate_extrinsic = 0
```

으로 고정해야 한다.

# Intrinsic 처리

`camera_calibration.json`에서 다음 중 하나를 자동 탐색한다.

```text
fx/fy/cx/cy
camera matrix K
intrinsics list
horizontal FOV
```

찾지 못하면 smoke test에 한해 horizontal FOV 90°를 사용한다. 이 경우 terminal에 경고가 출력된다.

수동 override:

```bash
export UCC_FX=...
export UCC_FY=...
export UCC_CX=...
export UCC_CY=...

./02_run_vins.sh /path/to/dataset
```

# 현재 파란 배경 Dataset에 대한 해석

실행 자체와 ROS2/VINS 연결 검증에는 사용할 수 있다. 그러나 missing Cesium tile의 경계는 실제 환경에 없는 artificial edge이므로, 이 Dataset의 trajectory 정확도를 정식 성능으로 평가하면 안 된다.
