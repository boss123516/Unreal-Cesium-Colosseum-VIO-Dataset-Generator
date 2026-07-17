# UCC VIO Dynamic 3-Minute v7

## 확정 동작

- `ClockSpeed = 1.0`
- 비행 속도 상한 `30 m/s`
- Dataset recording wall time `180 s`
- Camera `640×480 Scene PNG @ 10 Hz`
- IMU `100 Hz`
- Ground Truth `100 Hz`
- Takeoff부터 180초 recording window에 포함
- 3D Slalom, 상승/하강, 좌/우 360° broad turn 포함
- 180초 종료 시 `cancelLastTask → hover`
- Landing 없음
- Disarm 없음
- API control release 없음
- 종료 전에 CSV/JSON/PNG finalize
- Dataset 위치: `~/vio_sim_ws/datasets/ucc_dynamic_3min_YYYYMMDD_HHMMSS`

기존 recorder/trajectory 파일을 덮어쓰지 않는 standalone v7 package다.

## 1. ClockSpeed 적용

```bash
cd ~/Downloads/ucc_vio_dynamic_3min_v7
chmod +x 00_set_clock_speed_1.sh 01_run_dynamic_3min.sh 02_safe_recover.py
./00_set_clock_speed_1.sh
```

그 다음 Unreal의 Play/PIE를 **완전히 Stop 후 다시 Start**한다.  
ClockSpeed는 실행 중 hot reload되지 않는다.

## 2. 3분 비행 및 데이터 생성

Unreal Play 상태에서:

```bash
cd ~/Downloads/ucc_vio_dynamic_3min_v7
./01_run_dynamic_3min.sh
```

정상 종료 시:

```text
=== Dataset finalized ===
Root   : /home/.../vio_sim_ws/datasets/ucc_dynamic_3min_...
Camera : 1800 / 1800
IMU    : 18000 / 18000
GT     : 18000 / 18000
Valid  : True
[STATE] Drone was not landed. Hover command was issued.
```

## 3. 결과 확인

```bash
LATEST="$(find ~/vio_sim_ws/datasets -maxdepth 1 -type d \
  -name 'ucc_dynamic_3min_*' | sort | tail -n 1)"

cat "$LATEST/run_summary.json"
cat "$LATEST/validation_report.json"
find "$LATEST/mav0/cam0/data" -type f | wc -l
wc -l "$LATEST/mav0/imu0/data.csv"
wc -l "$LATEST/mav0/state_groundtruth_estimate0/data.csv"
```

## 4. 나중에 착륙시킬 때만

```bash
cd ~/Downloads/ucc_vio_dynamic_3min_v7
source ~/vio_sim_ws/airsim_pyenv/bin/activate
export PYTHONPATH="$HOME/vio_sim_ws/Colosseum/PythonClient:$PYTHONPATH"
./02_safe_recover.py Drone1
```

## 주의

Camera 1800장과 IMU/GT 18000개가 모두 써졌는지는 `validation_report.json`으로 판정한다.  
Cesium rendering이나 disk I/O가 10 Hz를 못 따라가면 파일은 생성되지만 `Valid=false`가 된다. 누락 frame을 복제해서 채우지 않는다.
