# 한국항공대학교 반경 5 km 건물 데이터

한국항공대학교 캠퍼스 중심(`37.60025, 126.86519`)에서 반경 5 km 원과
교차하는 국토교통부 브이월드 `GIS건물통합정보` 건물을 수집하는 작업
디렉터리다.

## 범위와 결과

- 중심: 한국항공대학교 캠퍼스 중심
- 반경: 5,000 m
- WGS84 사각 경계: `126.808499, 37.555334, 126.921881, 37.645166`
- 원본 레이어: `lt_c_bldginfo`
- 선택 규칙: 건물 polygon 일부라도 5 km 원과 교차하면 포함
- 최종 좌표계: WGS84 경위도(`EPSG:4326`)

수집이 끝나면 `output/`에 다음 결과가 생긴다.

```text
output/
├── buildings.geojson     # 원본 속성, polygon, 높이 파생값
├── buildings.geojson.gz  # 동일 GeoJSON의 전송·보관용 압축본
├── quality_report.json   # 건물 수, 중복 제거 수, 높이 누락률과 분포
├── fetch_manifest.json   # 범위, 요청 계약, 타일별 수집 이력
├── checksums.sha256      # 최종 산출물 무결성 해시
└── raw_tiles/            # 중단 후 재개할 수 있는 원본 WFS 응답
```

`buildings.geojson`의 기존 브이월드 속성은 보존한다. `_kau5km` 속성에
`height_final_m`, `height_source`, `height_raw`, `floor_count_raw`와 중심까지의
거리를 추가한다. `height`가 유효하면 그대로 사용하고, 없으면
`grnd_flr × 3.2 m`, 두 값 모두 없으면 9.6 m를 임시값으로 사용한다.

## 실행

브이월드 WFS는 발급받은 인증키가 필수다. 키를 채팅이나 명령행 인자로
전달하지 않는다. 직접 실행할 때는 현재 셸의 환경변수를 사용한다.

```bash
cd ~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/geodata/kau_5km_buildings
export VWORLD_API_KEY='발급받은-키'
python3 fetch_vworld_buildings.py
```

이 작업을 Codex가 이어서 실행해야 한다면 키가 셸 기록에 남지 않도록 숨김
입력을 받아 Git 제외 경로에 저장한다.

```bash
cd ~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/geodata/kau_5km_buildings
mkdir -p .secret
read -rsp 'VWorld API key: ' VWORLD_KEY
printf '%s\n' "$VWORLD_KEY" > .secret/vworld_api_key
chmod 600 .secret/vworld_api_key
unset VWORLD_KEY
```

`.secret/`과 `output/`은 이 디렉터리의 `.gitignore`에 포함되어 있다.

키 발급 시 등록 도메인을 요청에 함께 보내야 한다면 다음 값도 설정한다.

```bash
export VWORLD_API_DOMAIN='등록한-도메인'
```

키를 설정하기 전에도 계산된 범위와 초기 요청 수를 검사할 수 있다.

```bash
python3 fetch_vworld_buildings.py --dry-run
```

API의 요청당 최대 1,000개 제한 때문에 500 m 타일로 요청한다. 한 타일에서
1,000개가 반환되거나 전체 개수가 반환 개수보다 크면 그 타일을 네 조각으로
자동 분할한다. 최소 타일에서도 제한에 닿으면 누락된 결과를 만들지 않고
실패시킨다. 다시 실행하면 완료한 `raw_tiles/`를 재사용한다.

## LoD1 3D Tiles 및 Unreal 연결

지면고도가 결합된 GeoJSON과 3D Tiles는 다음 명령으로 재생성하고 검증한다.

```bash
cd ~/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/tools/geodata/kau_5km_buildings
python3 -m venv .venv
.venv/bin/pip install -r requirements-lod1.txt
mkdir -p dem
curl --fail --location \
  https://s3.amazonaws.com/elevation-tiles-prod/skadi/N37/N37E126.hgt.gz \
  --output dem/N37E126.hgt.gz
.venv/bin/python build_lod1_tiles.py
.venv/bin/python validate_lod1_tiles.py
```

지형은 AWS Open Data의 Mapzen Terrain Tiles `N37E126` SRTM 1 arc-second
타일을 사용한다. 원본 EGM96 평균해수면고도에 시스템의
`/usr/share/proj/egm96_15.gtx` 지오이드 모델을 적용해 WGS84 타원체고도로
변환한다. SRTM은 약 30 m 해상도이고 건물·수목의 영향을 받을 수 있으므로,
건물 내부 대표점 주변 5×5 셀의 25백분위를 지면 추정값으로 사용한다.

Unreal을 실행하기 전에 로컬 타일 서버를 시작한다.

```bash
python3 serve_tiles.py
```

`UCCVioDatasetSim`은 `HighAltitudeCity` 실행 시
`http://127.0.0.1:8765/tileset.json`을 사용하는
`KAU_Buildings_5km` Cesium3DTileset Actor를 자동 생성한다. 설정은
`Config/DefaultGame.ini`의 `[KAUBuildings]`에서 바꿀 수 있다.

서버와 Unreal Editor를 한 번에 실행하려면 다음 명령을 사용한다.

```bash
./launch_kau_high_altitude.sh
```

이미 생성된 데이터가 다른 경로에 있다면 `KAU_GEODATA_DIR`로 지정할 수
있다. 기존 `~/vio_sim_ws/geodata/kau_5km_buildings` 데이터는 실행기가
자동으로 찾아 사용한다.

```bash
KAU_GEODATA_DIR=/path/to/kau_5km_buildings ./launch_kau_high_altitude.sh
```

## 공식 출처

- 브이월드 WFS: `https://api.vworld.kr/req/wfs`
- 브이월드 WMS/WFS API 안내:
  `https://www.vworld.kr/dev/v4dv_wmsguide2_s001.do`
- 공공데이터포털 GIS건물통합정보:
  `https://www.data.go.kr/data/15083092/fileData.do`

브이월드 원본 속성 중 이 작업에서 직접 사용하는 필드는 다음과 같다.

| 필드 | 의미 |
|---|---|
| `ufid` | UFID |
| `height` | 건물높이 |
| `grnd_flr` | 지상층수 |
| `bldrgst_pk` | 건축물대장 PK |
| `bd_mgt_sn` | 건물관리번호 |
