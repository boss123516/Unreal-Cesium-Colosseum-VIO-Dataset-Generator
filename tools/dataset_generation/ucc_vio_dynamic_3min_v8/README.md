# UCC VIO Dynamic 3-Minute v8

v7의 검증된 180초 recorder와 종료 동작을 유지하면서 다음 고도 계약을 추가한 standalone 패키지다.

- AirSim `ClockSpeed = 1.0`
- Unreal 메인 화면: `SpringArmChase` 3인칭 추적 시점 (`FollowDistance=-6 m`)
- `Drone1` 초기 위치: PlayerStart 기준 NED `X=0, Y=0, Z=-400 m`
- 로컬 스폰 고도: `400 m`
- 허용 비행 고도: `300~500 m`
- 명령 경로 고도: 경계 여유 15m를 둔 약 `315~485 m`
- 비행 속도 상한: `30 m/s`
- Camera: `640×480 Scene PNG @ 10 Hz`
- IMU / Ground Truth: `100 Hz`
- 기록 시간: wall time `180 s`
- 종료: `cancelLastTask → hover`; 착륙·disarm·API release 없음

고도는 AirSim/Unreal PlayerStart 기준 로컬 NED 좌표 계약이다. 실제 Cesium terrain AGL과 동일하다고 가정하지 않는다.

## 1. 프로필 적용

Unreal Play/PIE를 완전히 정지한 상태에서:

```bash
cd tools/dataset_generation/ucc_vio_dynamic_3min_v8
./00_apply_400m_profile.sh
```

스크립트는 기존 `~/Documents/AirSim/settings.json`을 timestamp backup한 뒤 다음 값을 적용한다.

```text
ClockSpeed=1.0
ViewMode=SpringArmChase
CameraDirector.FollowDistance=-6
Drone1.X=0
Drone1.Y=0
Drone1.Z=-400
```

설정은 hot reload되지 않는다. 적용 후 Unreal Play/PIE를 새로 시작해야 한다.

실행 중 임시로 3인칭 추적 시점으로 전환하려면 PIE 화면을 클릭하고 `/` 키를 누른다.

## 2. 3분 데이터 생성

Unreal Play/PIE가 실행되고 RPC port 41451이 열린 상태에서:

```bash
./01_run_dynamic_3min.sh
```

실행기는 recorder를 시작하기 전에 RPC의 실제 runtime settings를 읽어 `ClockSpeed=1.0`과 `Drone1.Z=-400`을 검증한다. 이전 설정으로 실행 중이면 데이터 디렉터리를 만들지 않고 중단한다. 검증을 통과하면 AirSim `reset()` 직후 스폰 기준 상대 좌표 `(0,0,0)`으로 기체를 즉시 텔레포트하고 속도를 0으로 초기화한 뒤 hover를 잡는다. 별도의 `takeoff` 상승 없이 400m에서 수평으로 경로 비행을 시작하고, 처음 5초간 400m를 유지한 다음 10초에 걸쳐 3D 경로의 고도 변화를 부드럽게 적용한다. 따라서 직전 실행의 호버 위치나 reset 직후의 자유낙하가 다음 데이터셋 시작점에 영향을 주지 않는다.

## 3. 결과 계약

결과는 다음 위치에 생성된다.

```text
~/vio_sim_ws/datasets/ucc_dynamic_3min_YYYYMMDD_HHMMSS
```

`validation_report.json.all_pass`에는 다음 조건이 모두 포함된다.

- Camera 1800장
- IMU 18000개
- GT 18000개
- runtime ClockSpeed 1.0
- runtime spawn NED Z -400m
- GT 고도 샘플 수 일치
- 실제 로컬 고도 전 구간 300~500m
- recorder/flight error 없음
- worker 정상 종료 및 PNG/CSV 개수 일치

`run_summary.json.local_altitude_m`에 실제 시작·종료·최저·최고 로컬 고도를 기록한다.

## 4. 종료 후 착륙

기본 실행은 hover 상태를 유지한다. 착륙이 필요할 때만:

```bash
source ~/vio_sim_ws/airsim_pyenv/bin/activate
export PYTHONPATH="$HOME/vio_sim_ws/Colosseum/PythonClient:${PYTHONPATH:-}"
./02_safe_recover.py Drone1
```
