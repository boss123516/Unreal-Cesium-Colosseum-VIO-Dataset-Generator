# 고정익 PX4–Gazebo–Colosseum 파이프라인 인수인계

## 500 m 동적 비행 Dataset 3 구현 및 검증 기록

| 항목 | 내용 |
|---|---|
| 작성일 | 2026-07-21 |
| 작업 브랜치 | `fixed_wing` |
| 기준 기체 | PX4 Gazebo `rc_cessna` + UCC wrapper `rc_cessna_ucc` |
| 시뮬레이터 | PX4 SITL, Gazebo Harmonic 8.14, Unreal Engine 5.6, Colosseum/AirSim |
| 최종 결과 | 500 m 로컬 고도, 180초 동적 고정익 데이터셋 합격 |
| 보안 범위 | 한화 고유 제원이나 비공개 공력 데이터는 포함하지 않음 |

이 문서는 고정익 통합을 처음 보는 사람이 다음 내용을 한 번에 이해할 수 있도록
작성했다.

1. 실제 기체 운동과 공력은 어느 프로그램이 계산하는가?
2. PX4, Gazebo, Colosseum/AirSim, Unreal은 각각 무슨 역할을 하는가?
3. Gazebo의 250 Hz 계산 결과가 어떤 계약과 좌표 변환을 거쳐 전달되는가?
4. 왜 AirSim에 100 Hz로 다시 주입하며, 250 Hz 중 일부 상태를 사용하지 않는 것이
   왜 정상인가?
5. 500 m Dataset 3을 어떤 이유로 설계했고 무엇을 검증했는가?
6. 한화 기체 정보가 들어오면 무엇을 교체하고 무엇을 그대로 유지하는가?

---

## 1. 가장 중요한 결론

현재 시스템은 Unreal에 고정익 3D 모델만 로드해서 임의로 움직인 구성이 아니다.
기체의 자세와 궤적은 다음 폐루프에서 실제로 만들어진다.

```text
                         비행·물리 폐루프

     Gazebo 가상 센서 ───────────────► PX4 SITL
            ▲                           │
            │                           │ 모터·조종면 명령
            │                           ▼
     Gazebo 6-DoF 적분 ◄──── 공력·추진·조종면 모델
            │
            │ base_link 계산 결과, Gazebo Transport 250 Hz
            ▼
     UccKinematicsPublisher
            │ /ucc/fixed_wing/kinematics
            ▼
     gazebo_airsim_bridge.py
       - 계약 검사
       - ENU/FLU → NED/FRD 변환
       - 첫 상태 위치 재기준화
       - 최신 상태 선택
            │ AirSim RPC simSetKinematics(), 100 Hz
            ▼
     Colosseum ExternalPhysicsEngine
            ├── Unreal/Cesium 기체 표시
            ├── cam0 640×480, 10 Hz
            ├── IMU 100 Hz
            └── Ground Truth 100 Hz
```

핵심 원칙은 다음과 같다.

- **PX4는 비행 제어기**다. 가상 센서값을 보고 모터·에일러론·엘리베이터·러더
  명령을 만든다.
- **Gazebo가 물리 기준(source of truth)**이다. 질량, 관성, 중력, 공력, 추진,
  충돌과 6-DoF 운동을 계산하고 적분한다.
- **UCC 네이티브 플러그인은 물리를 만들지 않는다.** Gazebo가 계산한
  `base_link` 상태를 읽어 250 Hz로 발행할 뿐이다.
- **AirSim/Colosseum은 같은 기체를 다시 물리 계산하지 않는다.**
  `ExternalPhysicsEngine`에서 Gazebo 상태를 받아 렌더링과 센서 생성에 사용한다.
- **상태 흐름은 Gazebo → AirSim 단방향**이다. AirSim 상태를 Gazebo로 되돌려
  공력을 재계산하는 피드백 경로는 없다.
- `/ucc/fixed_wing/kinematics`는 **ROS 토픽이 아니라 Gazebo Transport 토픽**이다.
  현재 실시간 핵심 경로에는 ROS publish/subscribe가 없다. ROS는 추후 데이터셋
  재생이나 VIO 연동 단계에서 사용할 수 있다.

---

## 2. 컴포넌트별 책임

| 컴포넌트 | 책임 | 하지 않는 일 |
|---|---|---|
| PX4 SITL | mission, 자세·속도·고도 제어, actuator 명령 | Unreal 렌더링, VIO 센서 기록 |
| Gazebo Harmonic | 공력·추진·중력·충돌·6-DoF 적분, 가상 센서 | Cesium 영상 렌더링 |
| `rc_cessna_ucc` | PX4 기본 Cessna 포함, 상태 발행 플러그인 추가 | 새 공력 모델 정의 |
| `UccKinematicsPublisher` | Gazebo 링크 상태를 21-field 계약으로 250 Hz 발행 | 힘·모멘트 계산, 유한차분 가속도 계산 |
| `gazebo_airsim_bridge.py` | 계약 검사, 좌표 변환, 재기준화, 100 Hz RPC 주입 | 공력 계산, PX4 제어 |
| Colosseum/AirSim | RPC, External Physics 상태 수용, IMU·GT·카메라 API | 고정익 운동 적분 |
| Unreal/Cesium | 지형과 기체 외형 렌더링 | 비행 동역학의 기준값 생성 |
| Dataset recorder | Camera/IMU/GT 동기 기록과 자동 합격 판정 | 기체 제어 |

### 2.1 `rc_cessna_ucc`가 의미하는 것

UCC 모델은 PX4 기본 `rc_cessna`를 복사해 별도 공력을 만든 모델이 아니다.
다음처럼 upstream 모델을 merge include하고 상태 발행기만 추가한다.

```xml
<model name="rc_cessna_ucc">
  <include merge="true">
    <uri>rc_cessna</uri>
  </include>
  <plugin filename="libUccKinematicsPublisher.so"
          name="ucc::sim::systems::KinematicsPublisher">
    <link_name>base_link</link_name>
    <topic>/ucc/fixed_wing/kinematics</topic>
    <publish_rate_hz>250</publish_rate_hz>
  </plugin>
</model>
```

따라서 현행 질량·관성·공력면·추진력은 PX4의
`Tools/simulation/gz/models/rc_cessna/model.sdf`가 결정한다. UCC wrapper는
그 계산 결과를 외부 렌더링·센서 계층에 전달할 수 있게 만든 경계다.

---

## 3. 250 Hz Gazebo 상태 경로

### 3.1 250 Hz의 정확한 의미

Gazebo ODE 물리 스텝은 4 ms, 즉 250 Hz로 설정되어 있다. 네이티브 플러그인은
`ISystemPostUpdate`에서 물리 스텝이 끝난 뒤 `base_link`의 다음 값을 읽는다.

- world pose
- world linear velocity
- world angular velocity
- world linear acceleration
- world angular acceleration
- Gazebo simulation time

각속도와 각가속도는 world 값에서 body 값으로 회전한 후 발행한다. 선형·각가속도는
속도 차분으로 추정하지 않고 Gazebo의 링크 acceleration component를 직접 읽는다.
따라서 “PX4가 ROS로 250 Hz 결과를 보내는 구조”가 아니라 다음 구조다.

```text
Gazebo physics PostUpdate
  → native C++ plugin reads base_link
  → Gazebo Transport Double_V publish at 250 Hz
```

PX4는 이와 별도로 Gazebo 센서–actuator 브리지 안에서 폐루프 제어를 수행한다.
UCC 250 Hz 토픽은 그 폐루프의 결과 상태를 외부에 복제하는 관측 경로다.

### 3.2 21-field 전송 계약

메시지 형식은 `gz.msgs.Double_V`이며 총 21개 double 값으로 고정했다.

| 인덱스 | 개수 | 의미 | 좌표계/단위 |
|---:|---:|---|---|
| 0 | 1 | 계약 버전, 현재 `1.0` | scalar |
| 1 | 1 | Gazebo simulation timestamp | ns |
| 2–4 | 3 | position | world ENU, m |
| 5–8 | 4 | body-to-world quaternion `xyzw` | FLU → ENU |
| 9–11 | 3 | linear velocity | world ENU, m/s |
| 12–14 | 3 | angular velocity | body FLU, rad/s |
| 15–17 | 3 | linear acceleration | world ENU, m/s² |
| 18–20 | 3 | angular acceleration | body FLU, rad/s² |

브리지는 길이, 계약 버전, timestamp 정수성, NaN/Infinity를 검사한다. timestamp
역행이나 중복도 별도 카운터로 기록하며 역행 상태는 주입하지 않는다.

### 3.3 왜 ROS를 사용하지 않았는가

Gazebo 내부 상태를 가장 짧은 경로로 받기 위해 해당 버전의 네이티브 Gazebo
Transport와 `gz::sim::Link` component를 사용했다. 중간에 ROS 메시지 변환이나
별도 DDS 노드를 넣지 않아 다음 이점이 있다.

- Gazebo simulation timestamp와 링크 component를 그대로 보존한다.
- 위치·속도 차분이 아니라 Gazebo가 계산한 가속도를 직접 전달한다.
- 250 Hz 상태 경로의 의존성과 지연을 줄인다.
- ROS 배포판이나 bridge 설정과 무관하게 시뮬레이터 통합을 먼저 검증할 수 있다.

ROS가 필요 없다는 뜻은 아니다. 최종 데이터셋을 VINS-Mono 등에 넣을 때는
EuRoC-like 파일을 ROS 2 토픽으로 재생하는 계층을 별도로 사용한다.

---

## 4. 250 Hz 입력을 AirSim에 100 Hz로 주입하는 이유

브리지는 Gazebo callback으로 들어온 가장 최신 상태 하나를 보관하고, 독립적인
10 ms 주기로 `simSetKinematics()`를 호출한다.

```text
Gazebo source:     4 ms, 250 Hz  ─┬─┬─┬─┬─┬─┬─┬─┬─┬─
                                  │ latest state selection
AirSim injection: 10 ms, 100 Hz  ─────●─────●─────●─────
```

250 Hz 중 약 60%가 `superseded_source_states`로 집계되는 것은 패킷 손실이
아니다. 다음 100 Hz 주입 시각 전에 더 최신 Gazebo 상태가 도착해 이전 상태를
사용하지 않았다는 뜻이다. 브리지는 평균이나 보간 대신 가장 최근의 완전한
Gazebo 상태를 선택한다.

100 Hz를 사용한 이유는 다음과 같다.

- AirSim IMU와 Ground Truth 기록 계약이 100 Hz다.
- Unreal 시각·센서 계층에 250 Hz RPC를 강제할 실익이 작다.
- 100 Hz에서도 10 ms 간격으로 자세와 가속도를 갱신하므로 10 Hz 카메라보다
  충분히 높은 상태 해상도를 제공한다.
- RPC 지연과 렌더링 부하를 줄이면서 Gazebo의 고해상도 물리 결과를 최신값으로
  유지할 수 있다.

브리지는 다음 오류를 감시한다.

- 최초 Gazebo 상태 연결 timeout
- 최신 상태 age 0.5초 초과
- timestamp 역행·중복
- 잘못된 계약 길이·버전·숫자
- 주입 deadline miss
- 연속 AirSim RPC 실패 5회

최종 장시간 브리지 검증 결과는 다음과 같다.

| 지표 | 결과 |
|---|---:|
| 실행 시간 | 431.183 s |
| Gazebo source rate | 249.9985 Hz |
| AirSim injection rate | 100.0017 Hz |
| source period mean / p95 / max | 4.0 / 4.0 / 4.0 ms |
| receive-to-inject latency mean / p95 | 2.317 / 3.435 ms |
| injection deadline miss | 0 |
| timestamp regression | 0 |
| invalid state | 0 |
| RPC failure | 0 |

브리지는 1 Hz GCS heartbeat도 PX4로 보낸다. 이것은 상태 전달 경로가 아니라
PX4가 GCS data-link loss로 RTL에 들어가지 않도록 유지하는 운영용 side channel다.

---

## 5. 좌표계 변환과 500 m 재기준화

### 5.1 좌표계

| 계층 | world | body |
|---|---|---|
| Gazebo | ENU: East, North, Up | FLU: Forward, Left, Up |
| AirSim | NED: North, East, Down | FRD: Forward, Right, Down |

벡터 변환은 다음과 같다.

```text
world: [north, east, down] = [gazebo_y, gazebo_x, -gazebo_z]
body : [forward, right, down] = [gazebo_x, -gazebo_y, -gazebo_z]
```

자세는 Euler angle을 중간에 사용하지 않는다. world basis와 body basis를 각각
행렬로 바꾼 뒤 quaternion으로 되돌려 특이점과 축 혼동을 피한다.

### 5.2 첫 상태 재기준화

Gazebo와 Unreal은 원점이 다르다. 브리지가 받은 첫 Gazebo 위치를 source origin,
AirSim이 시작한 위치를 target origin으로 저장하고 다음 식을 사용한다.

```text
AirSim_position = AirSim_target_origin
                + Gazebo_current_position
                - Gazebo_first_position
```

이 때문에 절대 Gazebo 좌표를 Unreal에 그대로 복사하지 않으면서 이후의 상대
이동·자세·속도·가속도는 보존된다.

### 5.3 Dataset 3의 500 m가 의미하는 것

500 m는 Cesium 실제 지표면 AGL이나 해발고도를 자동 계산한 값이 아니다.
Unreal PlayerStart 기준 로컬 고도다.

실행 순서는 의도적으로 다음과 같이 구성했다.

1. AirSim profile의 `Drone1.Z`를 `-500`으로 설정한다.
2. Gazebo/PX4 기체를 상대고도 약 100 m까지 먼저 이륙·안정화한다.
3. 그 시점에 브리지를 시작해 현재 Gazebo 위치를 Unreal의 500 m 스폰 위치에
   재기준화한다.
4. 이후 Gazebo 고도 변화만 500 m를 중심으로 AirSim에 반영한다.

지상에서 브리지를 먼저 시작하면 Gazebo 이륙 상승량이 Unreal 500 m에 추가되어
고도 계약이 깨진다. 그래서 `01_prepare_takeoff.sh`가 이륙 안정화 직후
`02_run_500m_bridge.sh`를 `exec`해 사람의 지연 없이 연결한다.

---

## 6. Dataset 3 비행 설계와 선택 이유

### 6.1 비행 계약

| 항목 | 설정 | 이유 |
|---|---:|---|
| 기록 시간 | 180 s | 3분 연속 VIO 입력 확보 |
| 로컬 기준 고도 | 500 m | 요청된 고도 조건 |
| 합격 범위 | 450–550 m | 전체 샘플에 ±50 m 계약 적용 |
| waypoint 고도 변화 | -15–+15 m | 허용 범위 안에서 완만한 수직 excitation |
| 목표 대기속도 | 19 m/s | 현행 PX4 Cessna 최대 20 m/s에서 1 m/s 제어 여유 |
| PX4 Roll 제한 | 28° | 과격하지 않은 좌·우 선회 |
| 데이터 합격 Roll | 절대 35° 이하 | transient 여유를 포함한 안전 gate |
| 경로 | 직진 + 좌·우 S-turn | VIO에 회전·병진 excitation 제공 |
| 종료 | 무한 loiter | 착륙 없이 기록 종료 후 안전 유지 |

현재 PX4 `gz_rc_cessna`의 `FW_AIRSPD_MIN/TRIM/MAX`는 10/15/20 m/s다.
20 m/s 상한에 계속 붙이면 속도 제어 여유가 없으므로 19 m/s를 사용했다.
이 값은 Gazebo 엔진 자체의 속도 한계가 아니라 현행 RC Cessna 공력·추진·PX4
튜닝 범위다. 한화 기체 적용 시 성능 데이터와 함께 다시 정해야 한다.

### 6.2 mission 유효성 처리

PX4 기본 `MIS_TKO_LAND_REQ=2`는 mission에 착륙 지점을 요구했다. Dataset 3은
기록 종료 후 loiter하도록 설계했기 때문에 mission이 무효 처리됐다. 실행 중
`MIS_TKO_LAND_REQ=0`으로 설정해 착륙 없는 데이터 수집 mission을 허용했다.

`FW_R_LIM=28`을 runtime에 적용하고 실제 반환값을 `flight_mission.json`에
기록한다. mission은 현재 위치와 heading을 기준으로 생성하므로 특정 위도·경도에
고정된 경로가 아니다.

---

## 7. 센서 기록과 검증 기준

### 7.1 기록률

| 데이터 | 출력 | 주기 |
|---|---|---:|
| Camera | `mav0/cam0/data/*.png`, `data.csv`, `mapping.csv` | 10 Hz |
| IMU | `mav0/imu0/data.csv`, `mapping.csv` | 100 Hz |
| Ground Truth | `mav0/state_groundtruth_estimate0/data.csv`, `mapping.csv` | 100 Hz |

각 행에는 공통 target timestamp를 쓰고 `mapping.csv`에는 API가 반환한 source
timestamp를 별도로 저장한다. 따라서 정규 10/100 Hz 스케줄과 실제 센서 시각을
둘 다 추적할 수 있다.

### 7.2 자동 합격 gate

- Camera 1,800장, 640×480, blank frame 0
- IMU 18,000개, GT 18,000개
- Camera/IMU/GT timestamp 중복 및 역행 없음
- Camera source frame gap 300 ms 이하
- Camera capture schedule jitter 500 ms 이하
- 같은 target 시각의 Camera–GT source timestamp 차이 200 ms 이하
- quaternion norm 오차 `1e-4` 미만
- GT 전체 샘플 로컬 고도 450–550 m
- 좌·우 각각 5° 이상 bank 존재
- 절대 Roll 35° 이하
- 절대 Roll 3° 이하인 직진 샘플 비율 10% 이상

### 7.3 카메라 timing gate를 고친 이유

첫 180초 기록은 샘플 개수는 맞았지만 실제 Camera source timestamp 사이에
약 5.07초 공백이 있었다. VIO 입력으로 부적합하므로 삭제하지 않고 실패
산출물로 격리하고, 최대 source gap을 자동 실패 조건으로 추가했다.

두 번째 기록에서는 최대 Camera source gap이 165 ms로 정상화됐다. 그러나
초기 검증안은 Camera source timestamp와 wall-clock target의 절대 mapping drift
변화폭을 제한해, AirSim simulation clock과 wall clock의 정상적인 미세 속도
차이까지 실패로 판정했다.

최종 검증은 같은 target timestamp에서 Camera와 GT가 반환한 source timestamp의
차이를 비교한다. 두 센서가 공유하는 simulation-clock drift는 상쇄되고 실제
교차센서 불일치만 남는다. 이 기준은 다음 두 기록을 명확히 분리했다.

| 기록 | Camera–GT 최대 source 시각 차이 | 판정 |
|---|---:|---|
| 실패 기록 | 약 5,100 ms | 실패 |
| 최종 기록 | 135.003 ms | 통과, 제한 200 ms |

이전 판정 보고서는 최종 데이터셋의 `validation_history/`에 보존했다.

---

## 8. 최종 Dataset 3 결과

최종 데이터셋:

```text
$HOME/vio_sim_ws/datasets/ucc_fixedwing_dataset3_500m_20260721_172929
```

압축본:

```text
$HOME/vio_sim_ws/datasets/fixed_wing_datasets_v1.zip
```

| 지표 | 결과 |
|---|---:|
| `timing_report.json.all_pass` | `true` |
| Camera | 1,800 / 1,800, 10 Hz |
| IMU | 18,000 / 18,000, 100 Hz |
| Ground Truth | 18,000 / 18,000, 100 Hz |
| Camera 해상도 / blank | 640×480 / 0 |
| Camera source gap max | 165.004 ms |
| Camera schedule jitter max | 108.956 ms |
| Camera–GT source skew max | 135.003 ms |
| 로컬 고도 min / mean / max | 483.039 / 497.183 / 511.836 m |
| 수평속도 max | 19.500 m/s |
| Roll min / max | -23.297 / +28.374° |
| 직진 샘플 비율 | 75.43% |
| 좌선회 / 우선회 | 모두 존재 |
| quaternion norm 오차 max | `4.15e-8` |
| 오류 | 0 |

Git에는 1.1 GB 데이터셋과 실행 로그를 넣지 않는다. 코드, 검증 로직, 실행
절차와 결과 요약만 버전 관리한다.

---

## 9. 구현 중 발견하고 해결한 문제

### 9.1 Unreal 시각 메시의 기수·꼬리 반전

물리 자세와 카메라 방향은 정상이었지만 Unreal Cessna 외형만 180° 반대로
보였다. FBX 기수는 이미 body `+X`였는데 runtime patch가 Yaw 180°를 추가한 것이
원인이었다.

```text
변경 전: FRotator(0, 180, 90)
변경 후: FRotator(0,   0, 90)
```

이 문제는 시각 메시 축 문제였고 Gazebo 공력이나 브리지 좌표 변환 문제는
아니었다. 기존 설치본도 자동 교정하도록 patch migration과 회귀 테스트를
추가했다.

### 9.2 브리지 heartbeat를 PX4 heartbeat로 잘못 선택

mission 제어기가 forwarded GCS heartbeat를 먼저 받아 target system `0`을
선택하는 문제가 있었다. `MAV_AUTOPILOT_PX4`이며 source system이 0보다 큰
heartbeat만 선택하도록 수정했다.

### 9.3 GCS link loss와 RTL

이륙 준비가 끝난 뒤 브리지를 사람이 늦게 시작하면 PX4가 data-link loss로 RTL에
들어갈 수 있었다. 준비 스크립트가 브리지를 즉시 이어서 시작하도록 합치고,
브리지가 1 Hz GCS heartbeat를 지속해서 보내도록 했다.

### 9.4 브리지 600초 자동 종료

초기 Dataset 3 wrapper는 브리지를 600초 후 종료하도록 고정했다. 장시간 화면
확인이나 후속 기록에서 예기치 않게 끊길 수 있어 기본값을 무제한 `0`으로 바꾸고
필요한 경우에만 `BRIDGE_DURATION_SEC`로 제한하도록 정리했다.

### 9.5 Gazebo 추적 GUI

서버와 분리된 Gazebo GUI를 열고 `rc_cessna_ucc_0` 뒤쪽
`(-12, 0, 4) m`에서 자동 추적하는 `02a_open_gazebo_follow_gui.sh`를 추가했다.
이 GUI는 관찰용이며 물리나 데이터셋 Camera에 영향을 주지 않는다.

---

## 10. 재현 절차

저장소 루트:

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

`commander arm`이나 `commander takeoff`를 수동 입력하지 않는다.

### 터미널 3: 자동 이륙·안정화·500 m 브리지

```bash
./tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1/01_prepare_takeoff.sh
```

`[PREPARE_READY]`, `[PREPARE_COMPLETE]`, `[PREFLIGHT]`, 반복 `[RATE]`를 확인한다.

### 선택 터미널: Gazebo 기체 추적 화면

```bash
./tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1/02a_open_gazebo_follow_gui.sh
```

### 터미널 4: 180초 기록

```bash
./tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1/03_run_dataset3.sh
```

정상 종료 시 `[DATASET3_COMPLETE]`와 `timing_report.json.all_pass=true`를
확인한다.

종료는 브리지 → PX4/Gazebo → Unreal 순서로 각 터미널에서 `Ctrl+C`를 사용한다.

---

## 11. 주요 코드 위치

| 파일 | 역할 |
|---|---|
| `tools/fixedwing/ucc_fixedwing_mvp_v1/gz_models/rc_cessna_ucc/model.sdf` | PX4 Cessna include + 250 Hz publisher 등록 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/gz_plugin/UccKinematicsPublisher.cpp` | Gazebo 링크 상태 직접 발행 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/fixedwing_frames.py` | ENU/FLU → NED/FRD 변환 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/gazebo_airsim_bridge.py` | 250 Hz 구독, 최신 상태 100 Hz AirSim 주입 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/apply_colosseum_fixedwing_patch.py` | External Physics와 Cessna 시각 패치 재현 |
| `tools/fixedwing/ucc_fixedwing_mvp_v1/fixedwing_mini_dataset.py` | Camera/IMU/GT 기록과 품질 판정 |
| `tools/dataset_generation/ucc_fixedwing_dataset3_500m_v1/` | 500 m Dataset 3 전체 실행 패키지 |
| `docs/FIXEDWING_AERODYNAMIC_MODEL_REFERENCE.md` | 현행 공력 모델 분석과 한화 데이터 교체 지침 |
| `docs/FIXEDWING_INTEGRATION_STATUS.md` | 전체 고정익 통합 현황과 기본 실행 가이드 |

외부 Colosseum/AirSim plugin의 생성 binary와 설치본 소스는 Git에 직접 넣지
않는다. 저장소의 patch script가 필요한 변경을 반복 적용하고 빌드하도록 한다.

---

## 12. 한화 기체 데이터 적용 시 유지·교체 경계

### 그대로 유지할 부분

- PX4–Gazebo 폐루프 구조
- Gazebo 250 Hz 물리 스텝과 UCC 상태 계약
- Gazebo Transport 구독
- ENU/FLU → NED/FRD 변환
- 첫 상태 재기준화
- AirSim External Physics 100 Hz 주입
- Unreal/Cesium, Camera/IMU/GT 기록 형식
- timestamp·고도·선회·영상 품질 검증 틀

### 한화 값으로 교체할 부분

- 질량, CG, 관성 텐서
- 날개 면적, span, MAC와 collision geometry
- 양력·항력 polar, stall, 안정미계수와 rate derivative
- 조종면 위치·축·각도·속도 제한·effectiveness
- 프로펠러/모터/엔진 추력·토크 맵
- PX4 airframe, control allocation, trim, 속도 범위와 gain
- Unreal 실제 기체 메시, 축과 scale

현행 RC Cessna의 19 m/s, 28° Roll 제한, 공력계수를 한화 기체에 그대로 적용하면
안 된다. 기체 데이터를 바꾼 뒤 open-loop 공력 검증 → PX4 closed-loop 비행 →
250/100 Hz bridge 회귀 → 센서 데이터셋 검증 순서로 다시 승인해야 한다.

---

## 13. 현재 한계와 다음 우선순위

- 현행 모델은 약 1.67 kg RC Cessna이며 한화 실기체 모델이 아니다.
- 기본 `LiftDrag` 기반의 단순화된 공력 모델이다.
- PX4는 flap 출력을 만들지만 현행 SDF에는 flap joint controller 연결이 없다.
- 무풍, IMU noise 비활성 조건에서 검증했다.
- Cesium 500 m는 실제 지형 AGL 보장이 아닌 PlayerStart 기준 로컬 고도다.
- Gazebo 250 Hz → AirSim 100 Hz는 latest-state 방식이며 보간·평균은 하지 않는다.
- 정식 공력/성능 분석에는 한화 계수 적용과 별도 calibration이 필요하다.

다음 작업의 권장 순서는 다음과 같다.

1. 한화 질량·CG·관성·공력·추진·actuator 데이터 수령 및 단위/축 검토
2. upstream PX4 모델을 직접 수정하지 않는 `hanwha_fixedwing` 독립 모델 작성
3. flap 연결과 AdvancedLiftDrag 또는 동등 공력 모델 적용
4. open-loop force/moment와 trim/stall/glide 성능 검증
5. PX4 gain과 speed envelope 튜닝
6. 바람·난류·센서 noise 조건별 Dataset 재생성
7. ROS 2 재생과 VINS-Mono 정량 평가

---

## 14. 완료 판단

현재 완료된 것은 “현행 PX4 RC Cessna를 사용한 고정익 시뮬레이션·렌더링·센서
데이터 생성 파이프라인”과 “500 m Dataset 3”이다. Gazebo가 250 Hz로 계산한
실제 링크 상태가 계약·좌표 변환·재기준화를 거쳐 AirSim에 100 Hz로 전달되며,
Unreal 화면과 Camera/IMU/GT가 동일 운동을 사용한다는 것을 실행 결과로 검증했다.

앞으로 기체 자체가 한화 모델로 바뀌더라도 이 중간 파이프라인은 유지할 수 있다.
교체의 핵심은 Gazebo 물성·공력·추진 모델과 PX4 airframe이며, 교체 후 동일한
250/100 Hz 전송 및 데이터 품질 gate를 다시 통과시키는 것이 승인 기준이다.
