# KAU 30 m Building Inspection Orbit v1

항공대 중심의 건물 LoD1 타일을 가까이 확인하기 위한 저속 쿼드콥터
순환 비행이다. 데이터셋 생성이 아니라 육안 검사용이며 기본값은 다음과 같다.

- Cesium 원점: 경도 `126.86519`, 위도 `37.60025`
- 원점 높이: WGS84 타원체고 `37.630 m`
- Drone1 스폰: 로컬 NED `Z=-30 m`
- 물리 엔진: `FastPhysicsEngine`
- 순환 반경: `90 m`
- 속도: `5 m/s`
- 순환 횟수: `1`
- 기체 방향: 계속 항공대 중심을 바라봄
- 종료: 자동 착륙 없이 호버
- 건물 타일: physics mesh 및 양면 충돌 활성화

`30 m`는 항공대 중심 지면을 기준으로 맞춘 로컬 고도다. 캠퍼스 안의
지형 기복 때문에 경로 전체에서 정확히 30 m AGL을 보장하는
terrain-following 경로는 아니다.

## 실행

Unreal Play/PIE를 완전히 종료한 상태에서 한 번 적용한다.

```bash
cd ~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/dataset_generation/ucc_kau_building_orbit_30m_v1
./00_apply_30m_profile.sh
```

건물 서버와 Unreal을 연다.

```bash
~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/geodata/kau_5km_buildings/launch_kau_high_altitude.sh
```

Unreal에서 `Play`를 누른 뒤 Output Log에 아래 두 줄이 나타날 때까지 기다린다.

```text
[KAU_BUILDINGS] GEOREFERENCE_READY
[KAU_BUILDINGS] READY
```

그 다음 순환 비행을 시작한다.

```bash
./01_fly_building_orbit.sh
```

중간에 멈추려면 `Ctrl+C`를 누른다. 스크립트는 기체를 호버시킨다.
호버 중인 기체를 착륙·disarm하려면 AirSim Python 환경에서 다음을 실행한다.

```bash
source ~/vio_sim_ws/airsim_pyenv/bin/activate
export PYTHONPATH="$HOME/vio_sim_ws/Colosseum/PythonClient:${PYTHONPATH:-}"
./02_safe_recover.py Drone1
```

더 천천히 또는 더 가까이 돌려면 최소 반경 30 m 범위에서 인자를 바꿀 수 있다.

```bash
./01_fly_building_orbit.sh --radius-m 60 --speed-mps 3 --loops 1
```

## 실제 3D 충돌 시연

순환 비행과 별도로, 항공대 중심에서 약 65 m 떨어진 높이 `17 m` 건물
벽을 고도 `8 m`, 속도 `2 m/s`로 천천히 충돌시키는 시연이 들어 있다.
이 경로는 의도적인 충돌 검증이므로 일반 데이터셋 비행에는 사용하지 않는다.

Unreal을 새로 시작하고 타일 로딩이 끝난 상태에서:

```bash
./03_fly_collision_demo.sh
```

성공하면 터미널에 `[COLLISION_CONFIRMED]`와 `[PASS]`가 출력되며,
충돌 객체명·충돌점·법선·침투 깊이가 다음 형식으로 저장된다.

```text
~/vio_sim_ws/datasets/kau_collision_demo_YYYYMMDD_HHMMSS.json
```
