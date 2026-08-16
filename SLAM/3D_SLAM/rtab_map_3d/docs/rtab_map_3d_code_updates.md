# rtab_map_3d Code Updates

## 2026-02-18

### 00:00

- **추가** `docs/3d_lidar_main_rgbd_sub_design.md` - 3D LiDAR(Main) + RGB-D(Sub) RTAB-Map 설계 문서 (Reg/Strategy, 센서 구독, TF 구조, 토픽, LiDAR 선정 시 확인사항)

## 2026-02-17

### 20:10

- **수정** `launch/rtabmap_3d_astra_pro.launch.py` → `launch/rtabmap_3d_astra_pro_only.launch.py` - 파일명 변경(_only), astra_camera include 제거, rtabmap 파라미터 동기화 (approx_sync_max_interval: 0.5, queue_size: 30, Grid/Sensor: 1, Grid/RangeMax: 10.0)
- **수정** `launch/rtabmap_3d_astra_pro_scan_merged.launch.py` → `launch/rtabmap_3d_astra_pro_scan_merged_only.launch.py` - 파일명 변경(_only), docstring 업데이트

### 19:56

- **수정** `rviz/rtabmap_3d.rviz` - CloudMap Size (Pixels): 5 → 3

### 19:45

- **수정** `rviz/rtabmap_3d.rviz` - CloudMap Color Transformer: AxisColor → RGB8, Channel Name: intensity → rgb

### 19:24

- **추가** `rviz/rtabmap_3d_astra_pro_scan_merged.rviz` - 전용 rviz config (scan_merged QoS Best Effort, scan_front/rear 추가, RGB Image 패널)
- **수정** `launch/rtabmap_3d_astra_pro_scan_merged.launch.py` - rviz config를 전용 파일로 변경

### 15:19

- **추가** `launch/rtabmap_3d_astra_pro_scan_merged.launch.py`

### 15:48

- **수정** `launch/static_transforms.launch.py` - base_footprint 추가 (Z=0.30), camera_link Z=0.85
- **수정** `launch/rtabmap_2dlidar_rgbd.launch.py` - frame_id: base_link → base_footprint
- **수정** `launch/rtabmap_2dlidar_rgbd_gazebo.launch.py` - frame_id: base_link → base_footprint
- **수정** `launch/rtabmap_2dlidar_rgbd_loc.launch.py` - frame_id: base_link → base_footprint
- **수정** `launch/rtabmap_2dlidar_rgbd_loc_gazebo.launch.py` - frame_id: base_link → base_footprint
- **수정** `launch/rtabmap_3d_astra_pro.launch.py` - frame_id: base_link → base_footprint
- **수정** `launch/rtabmap_3d_astra_pro_scan_merged.launch.py` - frame_id: base_link → base_footprint
- **수정** `launch/rtabmap_3d_localization.launch.py` - frame_id: base_link → base_footprint
- **수정** `launch/rtabmap_3d_mapping.launch.py` - frame_id: base_link → base_footprint
- **수정** `launch/rtabmap_rgbd.launch.py` - frame_id: base_link → base_footprint
- **수정** `launch/rtabmap_rgbd_gazebo.launch.py` - frame_id: base_link → base_footprint
- **수정** `launch/rtabmap_rgbd_loc.launch.py` - frame_id: base_link → base_footprint
- **수정** `launch/rtabmap_rgbd_loc_gazebo.launch.py` - frame_id: base_link → base_footprint
