# UCC Cesium cam0 Tile Bridge v1

## 목적

AirSim `Drone1/cam0`가 보는 방향을 Cesium tile-selection에 직접 전달해,
고속 비행 중 cam0 이미지에 파란색 missing-tile/background가 나타나는 문제를 막는다.

PIE 시작 시 자동으로:

1. `APIPCamera` Scene capture 후보 탐색
2. 640×480, attached, downward-facing 후보를 cam0로 우선 선택
3. 현재 cam0 view 등록
4. 진행 방향 90 m 앞의 preload view 등록
5. 모든 Cesium tileset에 다음 설정 적용

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
CameraManager                  = bridge가 사용하는 manager
```

Level이나 Blueprint를 수동 편집할 필요가 없다.

---

## 적용 순서

### 1. UnrealEditor 완전히 종료

PIE만 끄는 것이 아니라 editor도 닫는다.

### 2. 패치 적용

```bash
cd ~/Downloads/ucc_cesium_cam0_bridge_v1
chmod +x *.sh

./01_apply_cam0_bridge.sh
./02_build_editor.sh
```

프로젝트 위치가 기본 경로와 다르면:

```bash
./01_apply_cam0_bridge.sh /absolute/path/to/UCCVioDatasetSim
./02_build_editor.sh /absolute/path/to/UCCVioDatasetSim
```

### 3. Editor 실행 및 Play

`HighAltitudeCity`를 열고 Play한다.

Output Log에 다음이 나와야 한다.

```text
[CESIUM_CAM0_BRIDGE] selected actor=...
[CESIUM_CAM0_BRIDGE] READY: current cam0 + 90m look-ahead camera ...
```

### 4. 정지 cam0 15초 probe

```bash
cd ~/Downloads/ucc_cesium_cam0_bridge_v1
./03_run_static_cam0_probe.sh
```

결과:

```text
~/vio_sim_ws/validation_runs/cam0_cesium_probe_YYYYMMDD_HHMMSS/
├── frame_0000.ppm
├── frame_....ppm
├── samples.json
└── probe_summary.json
```

`automatic_pass=true`와 PPM 3장의 지형 표시를 모두 확인한다.

### 5. 기존 v7 3분 실행

bridge는 Unreal runtime에서 자동 동작하므로 기존 명령을 그대로 쓴다.

```bash
cd ~/Downloads/ucc_vio_dynamic_3min_v7
./01_run_dynamic_3min.sh
```

---

## 로그가 READY까지 가지 않는 경우

```bash
./04_check_bridge_log.sh
```

`selected actor`에 표시된 해상도와 parent를 확인한다.

정상 기대값:

```text
size=640x480
parent=Drone1 관련 actor
```

다른 camera를 선택한 경우 Output Log의 후보 정보와 현재 settings.json을 같이 확인해야 한다.

---

## Rollback

```bash
./05_rollback_cam0_bridge.sh
./02_build_editor.sh
```

적용 전 Build.cs와 기존 bridge source는 프로젝트 아래의 다음 폴더에도 백업된다.

```text
.backup_cesium_cam0_bridge_YYYYMMDD_HHMMSS
```
