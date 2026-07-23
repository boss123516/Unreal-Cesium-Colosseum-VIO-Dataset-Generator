# 항공대 5 km 3D 건물 및 쿼드콥터 충돌 시연 계획

## 1. 목적

교수님께서 확인하려는 핵심은 건물 데이터가 단순한 2D 항공사진 또는
화면상의 외곽선이 아니라, 실제 위치·지면고도·건물높이를 갖는 3차원
기하 구조인지 여부다. 이를 다음 두 장면으로 검증한다.

1. 고도 약 30 m에서 건물 옆면과 지붕이 시점에 따라 달라지는 근접 순환 비행
2. 쿼드콥터가 건물 벽에 진입했을 때 Unreal 물리 엔진이 충돌을 반환하는 시연

## 2. 현재 3D 데이터 구성

- 범위: 한국항공대학교 중심 반경 5 km
- 중심: 위도 `37.60025`, 경도 `126.86519`
- 건물 수: `54,007`
- 3D 타일 수: `316 GLB`
- 전체 정점 수: `1,846,930`
- 전체 삼각형 수: `1,000,153`
- 지붕: 2D footprint를 삼각분할한 실제 면
- 벽: footprint의 각 변에 대해 지면부터 지붕까지 생성한 실제 수직 면
- 지면고도: SRTM 1 arc-second DEM과 EGM96→WGS84 높이 변환
- 건물높이:
  - VWorld 실측 높이: `16,695`동
  - 층수 × 3.2 m 추정: `11,344`동
  - 기본 9.6 m 적용: `25,968`동

따라서 각 건물은 `경도·위도 + 지면 타원체고 + 건물높이`로 배치된다.
카메라가 이동하면 지붕과 벽의 투영, 가림 관계, 특징점 위치가 함께
달라지므로 한 장의 2D 이미지를 평면에 붙인 방식과 구분된다.

## 3. 항공대 원점 정합

Unreal 런타임 시작 시 CesiumGeoreference를 다음 값으로 자동 보정한다.

```text
Longitude = 126.86519 deg
Latitude  = 37.60025 deg
Height    = 37.630 m (WGS84 ellipsoid)
```

`37.630 m`는 항공대 중심의 SRTM 정표고 `15.0 m`에 EGM96 지오이드
분리량 약 `22.630 m`를 더한 값이다. 이 보정으로 Unreal 원점,
쿼드콥터 로컬 좌표, 항공대 건물 타일이 같은 위치에 놓인다.

런타임 Output Log 확인 문자열:

```text
[KAU_BUILDINGS] GEOREFERENCE_READY
[KAU_BUILDINGS] READY ... collision=true physics_meshes=true
```

## 4. 30 m 근접 순환 비행

기본 시연 경로:

```text
물리 엔진   : FastPhysicsEngine
로컬 고도  : 30 m
순환 반경  : 90 m
속도       : 5 m/s
순환 횟수  : 1
기체 방향  : 계속 캠퍼스 중심을 향함
종료 상태  : hover
```

실행:

```bash
cd ~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/dataset_generation/ucc_kau_building_orbit_30m_v1
./00_apply_30m_profile.sh
```

Unreal을 완전히 다시 시작하고 Play를 누른 뒤:

```bash
./01_fly_building_orbit.sh
```

이 장면에서는 같은 건물의 지붕과 서로 다른 벽면이 연속적으로 보이는지,
건물 앞뒤 가림 순서가 카메라 위치에 맞게 변하는지 확인한다.

## 5. 실제 충돌 시연

Cesium 타일의 `CreatePhysicsMeshes`, 양면 충돌, Actor collision을
활성화한다. 충돌용 표적은 공식 데이터에서 확인한 항공대학교 건물이다.

```text
UFID              = 2004188062744552817400000000
건물높이          = 17.0 m
건물 중심         = 로컬 (37.103, 52.580) m
예상 첫 벽 교차점 = 로컬 (26.967, 38.215) m
충돌 고도         = 8.0 m
충돌 속도         = 2.0 m/s
```

실행:

```bash
./03_fly_collision_demo.sh
```

스크립트는 건물 물리 메시 로딩을 기다린 뒤 원점의 빈 공간에서 8 m로
하강하고, 17 m 높이 건물의 중심을 향해 저속 직진한다. 성공 기준은
AirSim `simGetCollisionInfo()`가 실제 충돌 객체를 반환하는 것이다.
SimpleFlight의 공중 스폰은 초기 landed flag를 가지므로, 스크립트가
짧은 takeoff 명령으로 비행 상태를 활성화한 뒤 목표 고도로 이동한다.

성공 로그:

```text
[COLLISION_CONFIRMED] object=...
[PASS] The quad produced a real physics collision with the 3D tile.
```

상세 결과:

```text
~/vio_sim_ws/datasets/kau_collision_demo_YYYYMMDD_HHMMSS.json
```

JSON에는 충돌 객체명, 충돌점, 표면 법선, 침투 깊이, 기체 위치가
저장된다. 이 기록은 “화면에 3D처럼 보이는 것”뿐 아니라 물리 엔진이
건물 표면을 공간상의 충돌면으로 사용했음을 보여주는 증거다.

## 6. 실제 런타임 검증 결과

2026-07-23 렌더 오프스크린 Unreal 런타임에서 충돌 시연을 실행했고
다음 결과를 얻었다.

```text
30 m 순환 제어 단축 시험 = PASS, 종료 후 hover
판정                    = PASS
collision_detected      = true
collision object        = KAU_Buildings_5km
기체 정지 위치 NED      = (26.549, 37.624, 21.468) m
계산한 첫 벽 XY         = (26.967, 38.215) m
벽 위치 수평 오차       = 0.724 m
실제 impact point       = (27.126, 37.830, 21.434) m
표면 normal             = (-0.909, -0.417, -0.000007)
penetration depth       = 0.0 m
```

검증 파일:

```text
~/vio_sim_ws/datasets/kau_collision_demo_20260723_210025.json
```

충돌 객체명이 Cesium World Terrain이 아니라 별도로 생성한
`KAU_Buildings_5km`로 기록되었고, 충돌 위치도 사전에 footprint로
계산한 벽 위치와 1 m 이내에서 일치했다. 따라서 이 결과는 쿼드가
지면에 닿은 것이 아니라 생성된 3D 건물 벽과 충돌했음을 보여준다.

## 7. 시연 순서

1. `00_apply_30m_profile.sh` 실행
2. 건물 서버와 Unreal 실행
3. Play 후 `GEOREFERENCE_READY`, `READY` 로그 확인
4. `01_fly_building_orbit.sh`로 외관·시차·가림 확인
5. Unreal Play를 재시작해 충돌 상태 초기화
6. `03_fly_collision_demo.sh`로 저속 벽 충돌 확인
7. 생성된 collision JSON을 교수님께 제시

## 8. 해석 시 주의점

- 30 m는 항공대 중심 지면 기준 로컬 고도이며 모든 지점에서 정확히
  30 m AGL을 유지하는 terrain-following 값은 아니다.
- 약 48.1%의 건물은 높이 정보가 없어 9.6 m 기본값을 쓴다.
- 창문·외벽 텍스처가 없는 LoD1 모델이다. 400 m VIO 연구에는 전체
  건물 형태와 높이가 핵심이므로 현재 단계에서는 의도된 단순화다.
- 충돌 시연은 물리 검증용이다. 정상 VIO 데이터셋 경로에는 의도적인
  충돌을 포함하지 않는다.
