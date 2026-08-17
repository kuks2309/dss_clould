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

## 2026-08-17 — 첫 실구동 결함 2건 수정 (rtabmap_dss_3dlidar_slam.launch.py · rtabmap_3d.rviz)

1. **ICP 키프레임 미생성 → 지도 0장** — `Icp/RangeMax` 20→80 m, `Icp/PointToPlaneMinComplexity` 0.0 추가.
   실측 근거: DSS 스캔은 근거리 점의 62%가 지면 링이라 20 m 로 자르면 지면 평면만 남아
   PointToPlane complexity 가 ~0.00002 로 떨어지고, icp_odometry 가 "Scan complexity too low
   (0.000018) to init first keyframe" 을 무한 반복 — odom→base_link TF 미발행로 rtabmap 이
   "Tf has two or more unconnected trees" 경고와 함께 지도를 조립하지 못했다.
   수정 후 ratio 0.89~1.0, 표준편차 ~2 cm, 점유 격자 161×162@0.1 m 생성 확인.
2. **RViz 스캔 디스플레이 무표시** — PointCloud2 토픽이 원본 워크스페이스 이름(/scan/points·
   Reliable)으로 남아 있던 것을 /dss/sensor/lidar3d·Best Effort 로 정정.

정본 병기: docs/code_updates/2026-08-16-3d-slam-port.md 추기 참조.

## 2026-08-17 (2차) — 주행 중 지도 갱신 정지·오돔 상실 수정

`Icp/MaxTranslation` 미설정(기본 0.2 m/frame)이 원인: 10 Hz 스캔에서 2 m/s 이상 주행 시
정합이 성공(ratio 0.85+)해도 프레임당 이동이 상한을 넘어 기각 → 노드 추가 정지(WM 고정)
→ "Odometry lost". 3.0 (30 m/s 상당) 으로 확대 + `Odom/ResetCountdown` 3→1 (상실 시
즉시 재초기화). 실측: 정지·저속 정상, 51→62 m 고속 구간에서 WM 28 고정과 lost 2회 재현.

## 2026-08-17 (3차) — z(피치) 드리프트 수정: Reg/Force3DoF

무데스큐 ICP 가 주행 중 지면 링 변형으로 피치 계통 편향을 얻어 그래프 z 가 141 m 에
+4.51 m 누적 상승(실측), 기울어진 회랑 구간들이 겹쳐 3D 지도가 뒤엉킴. 루프 클로저는
0건으로 무관 확인. DSS 도시 맵은 평지이므로 icp_odometry·rtabmap 양쪽에
`Reg/Force3DoF true` (x·y·yaw 만 추정) 적용.

## 2026-08-17 (4차) — localization launch 에 SLAM 실측 수정 3종 이식

rtabmap_dss_3dlidar_localization.launch.py 의 icp_odometry·rtabmap 파라미터가 원본
(Gazebo 저속) 그대로라 SLAM launch 에서 실측으로 확정한 수정을 동일 적용:
`Icp/RangeMax` 20→80 · `Icp/PointToPlaneMinComplexity` 0.0 · `Icp/MaxTranslation` 3.0 ·
`Odom/ResetCountdown` 3→1 · `Reg/Force3DoF` true(오돔·SLAM 양쪽). 미적용 시 localization
도 동일 결함(키프레임 거부·주행 기각·z 드리프트) 재현이 자명해 사전 이식.

## 2026-08-17 (5차) — localization 초기자세·표시 결함 3건 수정

1. `RGBD/StartAtOrigin` false→true (localization launch) — false 는 DB 마지막 자세에서
   시작해 DSS 리셋(=지도 원점 복귀) 워크플로와 어긋남. 리셋 직후 추정이 매핑 종료점
   (x=284.7)에 있던 실측 결함 → true 후 원점 (0.02,-0.04) 정상.
2. MapCloud3D 디스플레이를 rtabmap_localization.rviz 에도 추가(Cloud from scan true·
   Download map true) — SLAM rviz 에만 넣고 짝 자산에 누락했던 반복 실수의 정정.
3. `Download namespace: /rtabmap/rtabmap` (rviz 양쪽) — 노드가 ns+노드명 이중 경로에
   서비스를 등록해 기본 호출(/rtabmap/get_map_data)이 실패하던 것("Cannot call" 팝업).
4. Map(2D) 디스플레이 구독 Durability Volatile→Transient Local (localization rviz) —
   rtabmap 이 래치 1회 발행이라 늦게 뜬 RViz 가 놓치던 "No map received" 해소.

### 짝 자산 목록 (SLAM ↔ Localization — 한쪽 수정 시 반드시 대조)

| SLAM 쪽 | Localization 쪽 |
| --- | --- |
| launch/slam/rtabmap_dss_3dlidar_slam.launch.py | launch/localization/rtabmap_dss_3dlidar_localization.launch.py |
| rviz2/rtabmap_3d.rviz | rviz2/rtabmap_localization.rviz |

ICP·오돔 파라미터와 디스플레이 구성은 두 쪽이 같은 결함을 공유한다 — 실측 수정은
**양쪽 동시 적용**이 기본값이고, 한쪽만 고치는 것이 이 모듈의 반복 실수 패턴이었다
(mistake 2026-08-17-004).
