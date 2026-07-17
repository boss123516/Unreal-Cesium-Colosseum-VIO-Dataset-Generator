# UCC VIO Dataset Generator Handover

## Latest milestone

- Full EuRoC-like Camera / IMU / Ground Truth dataset generation completed once.
- Camera: 640×480 at 10 Hz
- IMU: 100 Hz
- Ground Truth: 100 Hz
- Dynamic flight: up to 30 m/s for 180 seconds
- Current blocker: blue Cesium missing-tile regions in some `cam0` frames

## Latest document

- `UCC_VIO_Dataset_Generator_4th_Handover_Full_Dataset_Success_2026-07-18.md`

## Included tools

- `tools/dataset_generation/ucc_vio_dynamic_3min_v7`
- `tools/cesium/ucc_cesium_cam0_bridge_v1`
- `tools/vins/ucc_vins_mono_ros2_full_run_v1`
