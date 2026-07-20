# UCC 고정익 통합 현황 및 실행 가이드

## 1. 현재 결론

`fixed_wing` 브랜치에서 PX4와 Gazebo의 고정익 동역학을 Unreal/Cesium 및
Colosseum 센서 계층에 연결하는 통합 MVP를 완료했다.

현재 구성은 다음 기능을 제공한다.

- PX4 `gz_rc_cessna` 자동 이륙 및 고정익 제어
- Gazebo Harmonic 기반 고정익 동역학
- Gazebo 기체 상태 250 Hz 발행
- Colosseum `ExternalPhysicsEngine`에 100 Hz 상태 주입
- Unreal/Cesium에서 Cessna 외형 표시
- 640x480, 10 Hz `cam0` 이미지 기록
- 100 Hz IMU 및 Ground Truth 기록
- 카메라, IMU, GT 타임스탬프 및 품질 검증
- 고정익 전용 후방 관찰 카메라

검증 범위는 **무풍, IMU 노이즈 비활성, PX4 기본 Cessna 파라미터**를 사용한
통합 MVP이다. 실제 난류, 돌풍, 센서 노이즈 및 기체별 공력 튜닝은 다음 단계다.

## 2. 브랜치와 커밋

작업 브랜치:

```text
fixed_wing
```

주요 커밋:

| 커밋 | 내용 |
|---|---|
| `818b0da` | Fixed-wing External Physics MVP 구성 |
| `488ebb0` | PX4/Gazebo 네이티브 상태 브리지와 데이터셋 통합 완료 |
| `1e1a703` | 수평 안정형 고정익 후방 추적 카메라 추가 |

`sim/UCCVioDatasetSim/Content/CesiumSettings/`는 로컬 Cesium 설정이므로 위
고정익 커밋에 포함하지 않았다.

## 3. 시스템 구성

```text
PX4 fixed-wing controller
        |
Gazebo Harmonic gz_rc_cessna dynamics
        |
UccKinematicsPublisher
250 Hz, world ENU / body FLU
        |
gazebo_airsim_bridge.py
100 Hz, world NED / body FRD
        |
Colosseum ExternalPhysicsEngine
        |
        +-- Unreal/Cesium player viewport
        +-- AirSim cam0, 640x480 at 10 Hz
        +-- AirSim IMU, 100 Hz
        +-- AirSim Ground Truth, 100 Hz
```

역할 구분은 다음과 같다.

- PX4: 고정익 비행 모드, 자세 제어 및 조종면 명령
- Gazebo: 기체 동역학 및 링크 상태의 기준값
- 네이티브 Gazebo 플러그인: 위치, 자세, 속도, 각속도, 가속도 및 각가속도 발행
- Python 브리지: 좌표계 변환, 위치 재기준화, 주입 주기 및 오류 감시
- Colosseum: AirSim RPC, IMU, GT 및 카메라 API 제공
- Unreal/Cesium: 고해상도 지형 렌더링과 플레이어 관찰 화면 제공

## 4. 좌표계와 상태 계약

Gazebo 플러그인은 다음 순서의 21개 필드를 발행한다.

```text
contract version
simulation timestamp
position ENU
body-FLU to world-ENU quaternion xyzw
linear velocity ENU
angular velocity FLU
linear acceleration ENU
angular acceleration FLU
```

브리지는 이를 AirSim이 사용하는 world NED/body FRD로 변환한다. 첫 Gazebo
위치는 Unreal의 기체 생성 위치에 재기준화되며 이후 운동은 Gazebo를 기준으로
유지된다.

주입 중 다음 조건을 감시한다.

- 소스 상태 정체 또는 타임아웃
- 타임스탬프 역행
- 잘못된 계약 버전 또는 필드 수
- NaN 및 비정상 수치
- 주입 마감 실패
- 연속 AirSim RPC 실패
- 수신부터 주입까지의 지연

## 5. 구현된 주요 변경

### 5.1 PX4/Gazebo 상태 경로

- PX4 `gz_rc_cessna` 모델을 기반으로 `rc_cessna_ucc` 모델을 구성했다.
- 네이티브 `gz::sim::System` 플러그인이 링크 상태를 250 Hz로 발행한다.
- 가속도와 각가속도는 위치/속도의 차분이 아니라 Gazebo 링크 컴포넌트 값을
  직접 사용한다.
- 브리지가 1 Hz MAVLink GCS heartbeat를 보내 PX4의 GCS 연결 조건을 만족한다.
- 브리지는 변환된 전체 kinematics를 `simSetKinematics`로 100 Hz 주입한다.

### 5.2 Colosseum 안정화

- `ExternalPhysicsEngine` 업데이트와 센서 업데이트 사이에 `PhysicsBody` 잠금을
  추가해 기존 `SensorCollection` 동시 접근 크래시를 제거했다.
- 변경은 설치된 AirSim 플러그인에 직접 의존하지 않고
  `apply_colosseum_fixedwing_patch.py`로 반복 적용할 수 있다.
- `12_patch_build_colosseum_fixedwing.sh`가 패치 적용과 Unreal 플러그인 빌드를
  함께 수행한다.

### 5.3 Unreal Cessna 외형

- External Physics 모드에서 기존 쿼드콥터 Body와 Prop 메시를 숨긴다.
- `/Game/FixedWing/SM_RCCessna` 정적 메시를 Pawn 루트에 부착한다.
- FBX 축을 AirSim/Unreal body 축에 맞추는 고정 회전을 적용한다.
- 시각 메시에는 충돌을 사용하지 않는다.
- 데이터셋 SceneCapture에서는 기체를 숨겨 `cam0` 자기 가림을 방지한다.
- 플레이어 관찰 화면에서는 Cessna가 보인다.

현재 시각 모델은 본체 정적 메시이며 조종면 애니메이션은 구현하지 않았다.
기체 자세와 이동은 실제 Gazebo/PX4 상태를 사용하지만, 에일러론·엘리베이터·
러더의 개별 시각 애니메이션은 별도 작업이다.

### 5.4 `cam0`, IMU 및 Ground Truth

- 런타임 생성 `BP_PIPCamera`가 이 Cesium 맵에서 빈 RGB를 반환하는 문제를
  피하기 위해 검증된 built-in front-center SceneCapture를 `cam0` 별칭으로
  재사용한다.
- `cam0` 장착값은 body 기준 X=1.0 m, Pitch=-30 deg이다.
- 이미지 해상도는 640x480이고 Motion Blur는 0이다.
- 기록 시작 전 세 프레임 연속으로 상세 Cesium 영상이 나오는지 확인한다.
- 기록기는 카메라 10 Hz, IMU 100 Hz, GT 100 Hz를 EuRoC 유사 구조로 저장한다.

### 5.5 플레이어 관찰 카메라

기존 `SpringArmChase`가 Roll, Pitch, Yaw를 모두 기체에서 상속해 자세 변화가
화면에서 상쇄되는 문제가 있었다.

현재 고정익 관찰 카메라는 다음 규칙을 사용한다.

| 축 | 카메라 동작 | 화면에서 보이는 정보 |
|---|---|---|
| Yaw | 기체를 추종 | 항상 기체 뒤에서 추적하며 지형 회전으로 선회 확인 |
| Roll | 상속하지 않음 | 날개 뱅크와 롤 진동 확인 |
| Pitch | 상속하지 않음 | 기수 들림/내림과 피치 진동 확인 |

Follow distance는 기존 6 m에서 2 m로 줄여 Cessna 자세를 더 크게 확인할 수
있도록 했다. 이 카메라는 플레이어 뷰포트 전용이며 body-mounted `cam0`의
포즈나 데이터셋 결과에는 영향을 주지 않는다.

Yaw 상속까지 끄면 카메라가 월드 한 방향에 고정되어 기체가 옆이나 앞에서
보이고 후진하는 것처럼 느껴진다. 현재 구현은 Yaw를 추종하므로 이 현상이 없다.

## 6. 검증 결과

### 6.1 단위 및 빌드 검사

- 고정익 Python 단위 테스트: 22개 통과
- Python 문법 컴파일: 통과
- JSON 문법 검사: 통과
- 패치 반복 적용: 통과
- Unreal `UCCVioDatasetSimEditor` Development 빌드: 성공

### 6.2 기준 네이티브 브리지 검증

| 항목 | 결과 |
|---|---:|
| 실행 시간 | 100.000 s |
| Gazebo 소스 속도 | 249.9997 Hz |
| UCC 주입 속도 | 99.9999 Hz |
| UCC 주입 수 | 10,000 |
| 수신-주입 평균 지연 | 1.617 ms |
| 수신-주입 p95 지연 | 2.658 ms |
| 주입 마감 실패 | 0 |
| 타임스탬프 역행 | 0 |
| 비정상 상태 | 0 |
| RPC 실패 | 0 |
| 종합 결과 | Pass |

독립 재검증에서도 90초 동안 소스 249.9997 Hz, 주입 99.9999 Hz, p95 지연
2.331 ms, 9,000회 주입, RPC 실패 0으로 통과했다.

### 6.3 30초 미니 데이터셋

| 항목 | 결과 |
|---|---:|
| 카메라 | 300 / 300, 640x480, 10 Hz |
| IMU | 3,000 / 3,000, 100 Hz |
| Ground Truth | 3,000 / 3,000, 100 Hz |
| 빈 이미지 | 0 |
| 최대 white ratio | 0.0 |
| 타임스탬프 중복/역행 | 0 / 0 |
| 최대 quaternion norm error | 4.12e-8 |
| 최대 이동 거리 | 160.928 m |
| 최대 수평 속도 | 14.554 m/s |
| 기록 오류 | 0 |
| 종합 결과 | Pass |

별도 재검증 데이터셋도 카메라 300, IMU 3,000, GT 3,000, 빈 이미지 0,
이동 거리 159.278 m, 최대 수평 속도 14.600 m/s로 통과했다.

### 6.4 카메라 수정 후 실제 비행 검증

- PX4 `Ready for takeoff` 확인
- `commander arm` 성공
- `commander takeoff` 및 `Takeoff detected` 확인
- Cessna가 화면 중앙 부근에서 뒤쪽 시점으로 유지됨
- 선회 시 카메라가 Yaw를 따라가므로 후진처럼 보이지 않음
- 지평선이 기체 Roll/Pitch와 함께 회전하지 않음
- 실제 상태 범위: Roll 약 28.9 deg, Pitch 약 34.6 deg
- 90초 브리지 실행 `all_pass: true`, RPC 실패 0

카메라 검증 런은 PX4/Gazebo 소스가 준비되기 전에 브리지를 먼저 실행한 초기
대기 시간을 통계에 포함했다. 성능 기준값은 초기 대기를 제외해 250/100 Hz를
달성한 위 기준 네이티브 브리지 결과를 사용한다.

## 7. 실행 전 준비

저장소 루트는 다음 경로다.

```bash
cd /home/boss/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator
```

모든 `./tools/fixedwing/...` 명령은 반드시 이 저장소 루트에서 실행하거나 절대
경로를 사용해야 한다. `~/vio_sim_ws`에서 상대 경로로 실행하면 `No such file or
directory`가 발생한다.

최초 설치 또는 설정을 다시 적용할 때만 다음을 실행한다.

```bash
./tools/fixedwing/ucc_fixedwing_mvp_v1/00_preflight.sh
./tools/fixedwing/ucc_fixedwing_mvp_v1/03_install_bridge_python_deps.sh
./tools/fixedwing/ucc_fixedwing_mvp_v1/07_build_gazebo_kinematics_plugin.sh
./tools/fixedwing/ucc_fixedwing_mvp_v1/12_patch_build_colosseum_fixedwing.sh
./tools/fixedwing/ucc_fixedwing_mvp_v1/01_apply_external_physics_profile.sh \
  --profile validation
```

`01_apply_external_physics_profile.sh` 실행 후에는 Unreal을 재시작해야 한다.

## 8. 실제 실행 순서

### 8.1 터미널 1: Unreal/Cesium

```bash
cd /home/boss/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator

/home/boss/vio_sim_ws/UE_5.6/Engine/Binaries/Linux/UnrealEditor \
  "$PWD/sim/UCCVioDatasetSim/UCCVioDatasetSim.uproject" \
  /Game/Maps/HighAltitudeCity \
  -game -windowed -ResX=1280 -ResY=720 -nosplash -NoSound
```

Unreal 창에서 Cesium 지형과 AirSim 메시지가 나타날 때까지 기다린다.

### 8.2 터미널 2: PX4/Gazebo

```bash
cd /home/boss/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator

./tools/fixedwing/ucc_fixedwing_mvp_v1/08_run_gz_rc_cessna_ucc.sh
```

`pxh>` 프롬프트가 나타나면 터미널을 유지한다. 브리지가 아직 없으면
`Preflight Fail: No connection to the GCS`가 나올 수 있으며 정상이다.

### 8.3 터미널 3: Gazebo-UCC 브리지

```bash
cd /home/boss/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator

./tools/fixedwing/ucc_fixedwing_mvp_v1/09_run_gazebo_airsim_bridge.sh
```

정상 상태에서는 다음과 유사한 메시지가 반복된다.

```text
Connected!
[RATE] source=250.0Hz inject=100.0Hz ...
```

### 8.4 이륙

브리지가 heartbeat를 보내면 PX4 터미널에 `Ready for takeoff!`가 나타난다.
같은 PX4 터미널에서 다음을 입력한다.

```text
commander arm
commander takeoff
```

정상 상태에서는 다음 메시지를 확인할 수 있다.

```text
Armed by internal command
Takeoff detected
```

### 8.5 30초 데이터셋 기록

기체와 브리지가 동작하는 동안 네 번째 터미널에서 실행한다.

```bash
cd /home/boss/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator

./tools/fixedwing/ucc_fixedwing_mvp_v1/11_run_fixedwing_mini_dataset.sh \
  --output "$HOME/vio_sim_ws/artifacts/fixedwing_mvp/mini_dataset_manual" \
  --duration-sec 30
```

성공하면 `.recording_incomplete`가 제거되고 `timing_report.json`의
`all_pass`가 `true`가 된다.

## 9. 종료 순서

각 프로세스를 실행한 터미널에서 다음 순서로 종료한다.

1. 브리지 터미널: `Ctrl+C`
2. PX4/Gazebo 터미널: `Ctrl+C`
3. Unreal 창 종료 또는 Unreal 터미널: `Ctrl+C`

같은 PX4/Gazebo를 중복 실행하지 않는다.

## 10. 자주 발생하는 오류

### `No such file or directory`

원인: `~/vio_sim_ws`에서 저장소 상대 경로를 실행했다.

해결:

```bash
cd /home/boss/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator
```

이후 `./tools/fixedwing/...` 명령을 실행한다.

### `PX4 server already running for instance 0`

원인: 이전 PX4/Gazebo 인스턴스가 실행 중이거나 같은 명령을 두 번 실행했다.

해결:

- 기존 PX4 터미널에서 `Ctrl+C`로 종료한다.
- Gazebo가 완전히 종료된 뒤 `08_run_gz_rc_cessna_ucc.sh`를 다시 실행한다.
- 기존 실행을 유지하려면 새 PX4를 실행하지 않고 브리지만 연결한다.

### `Preflight Fail: No connection to the GCS`

원인: `gazebo_airsim_bridge.py`가 아직 실행되지 않아 GCS heartbeat가 없다.

해결: Unreal RPC가 준비된 상태에서 터미널 3의
`09_run_gazebo_airsim_bridge.sh`를 실행한다. 브리지가 연결되면 경고가 사라지고
`Ready for takeoff!`가 나타난다.

### 브리지가 AirSim에 연결되지 않음

원인: Unreal이 꺼져 있거나 AirSim RPC 41451이 아직 준비되지 않았다.

해결: 터미널 1에서 Unreal을 먼저 실행하고 AirSim 초기화 후 브리지를 다시
실행한다.

### 카메라에서 기체가 후진하는 것처럼 보임

고정익 카메라 패치가 적용되지 않은 설치본이거나 Yaw 비추종 구성이 남아 있을
수 있다. Unreal을 종료한 뒤 다음을 실행하고 다시 시작한다.

```bash
./tools/fixedwing/ucc_fixedwing_mvp_v1/12_patch_build_colosseum_fixedwing.sh
./tools/fixedwing/ucc_fixedwing_mvp_v1/01_apply_external_physics_profile.sh \
  --profile validation
```

## 11. 주요 파일

| 파일 | 역할 |
|---|---|
| `config/simulator/settings.fixedwing.validation.json` | 고정익 검증용 AirSim 설정 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/08_run_gz_rc_cessna_ucc.sh` | PX4/Gazebo 실행 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/09_run_gazebo_airsim_bridge.sh` | Gazebo-UCC 브리지 실행 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/gazebo_airsim_bridge.py` | 네이티브 상태 수신, 변환 및 주입 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/fixedwing_frames.py` | ENU/FLU-NED/FRD 변환 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/gz_plugin/UccKinematicsPublisher.cpp` | Gazebo 상태 발행 플러그인 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/11_run_fixedwing_mini_dataset.sh` | 미니 데이터셋 기록 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/apply_colosseum_fixedwing_patch.py` | 설치된 Colosseum 패치 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/12_patch_build_colosseum_fixedwing.sh` | 패치 적용 및 UE 빌드 |

## 12. 검증 산출물

검증 파일은 Git 저장소 밖에 있다.

```text
$HOME/vio_sim_ws/artifacts/fixedwing_mvp/final_bridge_summary.json
$HOME/vio_sim_ws/artifacts/fixedwing_mvp/final_bridge_state.csv
$HOME/vio_sim_ws/artifacts/fixedwing_mvp/verify_bridge_summary.json
$HOME/vio_sim_ws/artifacts/fixedwing_mvp/verify_bridge_state.csv
$HOME/vio_sim_ws/artifacts/fixedwing_mvp/mini_dataset_pass/
$HOME/vio_sim_ws/artifacts/fixedwing_mvp/mini_dataset_verify/
```

## 13. 현재 한계

- PX4 기본 `gz_rc_cessna` 공력 모델과 파라미터를 사용한다.
- 무풍 및 검증용 IMU 노이즈 0 조건에서 통합을 검증했다.
- 고정익 전용 실험 궤적 생성기는 아직 연결하지 않았다.
- 현재 자동 이륙 후 PX4 기본 항법 동작을 사용한다.
- Cessna 조종면의 시각 애니메이션은 없다.
- 외부 관찰 카메라는 사람의 자세 확인용이며 데이터셋 `cam0`와 별개다.
- 진동은 상태와 화면에서 관찰할 수 있지만 PSD, 주파수 대역 및 RMS 기반의
  정량 진동 리포트는 아직 없다.

## 14. 다음 권장 작업

1. PX4/Gazebo 풍속, 난류 및 돌풍 조건을 설정한다.
2. Roll/Pitch/Yaw와 IMU 각속도에 대해 RMS, peak-to-peak 및 PSD를 계산한다.
3. 무풍/정상풍/난류 시나리오별 동일 궤적을 반복 실행한다.
4. 고정익 VIO에 필요한 선회, 상승, 하강 및 S-turn excitation 궤적을 연결한다.
5. IMU 노이즈와 bias를 목표 센서 사양에 맞게 복구한다.
6. 3분 이상 장시간 데이터셋을 기록하고 드롭, 지연 및 영상 품질을 재검증한다.
7. 필요하면 Cessna 조종면 애니메이션을 PX4 actuator 상태와 연결한다.

다음 단계의 핵심은 단순히 화면에서 흔들림을 확인하는 것을 넘어, 동일한
비행 조건에서 자세와 IMU 진동을 수치화해 데이터셋 메타데이터로 남기는 것이다.
