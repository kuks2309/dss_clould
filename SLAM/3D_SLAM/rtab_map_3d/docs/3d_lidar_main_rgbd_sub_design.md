# 3D LiDAR(Main) + RGB-D(Sub) RTAB-Map 설계 문서

> 작성일: 2026-02-17
> 상태: 설계 단계 (3D LiDAR 미선정)

## 1. 개요

3D LiDAR를 메인 센서, RGB-D 카메라를 보조 센서로 사용하는 RTAB-Map SLAM/Localization 구성.

### 목표
- Odometry: 3D LiDAR 포인트클라우드 기반 ICP
- Registration(루프 클로저): ICP 메인 + Visual 보조
- 맵 생성: 3D LiDAR 포인트클라우드 + RGB-D depth로 밀도 보강

### 현재 프로젝트 기존 구성과의 비교

| 항목 | 기존 3d_mapping | 기존 2dlidar_rgbd | **신규 (본 문서)** |
|------|----------------|-------------------|-------------------|
| Odometry | ICP (3D LiDAR) | RGBD (Visual) | **ICP (3D LiDAR)** |
| Registration | ICP only (`Reg/Strategy: 1`) | Visual only (`Reg/Strategy: 0`) | **ICP + Visual (`Reg/Strategy: 2`)** |
| 3D LiDAR | scan_cloud 구독 | 미사용 | **scan_cloud 구독 (메인)** |
| RGB-D depth | 미사용 | depth 구독 | **depth 구독 (보조)** |
| RGB 이미지 | rgb만 구독 | rgb 구독 | **rgb 구독** |
| 2D LiDAR | 미사용 | scan 구독 | 미사용 |
| Launch 파일 | `rtabmap_3d_mapping.launch.py` | `rtabmap_2dlidar_rgbd.launch.py` | 신규 작성 예정 |

## 2. RTAB-Map 핵심 파라미터

### 2.1 Reg/Strategy (Registration 전략)

루프 클로저 검출 시 노드 간 매칭 방법을 결정하는 핵심 파라미터.

| 값 | 이름 | 동작 | 용도 |
|----|------|------|------|
| `0` | Visual | RGB 특징점(ORB, SIFT 등) 매칭 | RGB-D 메인 구성 |
| `1` | ICP | 포인트클라우드 ICP 매칭 | 3D LiDAR 단독 구성 |
| **`2`** | **Visual + ICP** | **ICP로 매칭 후 Visual로 보정** | **본 문서의 목표 구성** |

### 2.2 센서 구독 설정

```python
# SLAM 노드 파라미터
'subscribe_depth': True,        # RGB-D depth 활용
'subscribe_rgb': True,          # RGB 이미지 (Visual 특징)
'subscribe_rgbd': False,        # rgbd 토픽 미사용 (개별 rgb+depth 사용)
'subscribe_scan': False,        # 2D LiDAR 미사용
'subscribe_scan_cloud': True,   # 3D LiDAR 포인트클라우드 (메인)
'subscribe_odom_info': True,    # ICP odometry 정보 동기화
```

### 2.3 Odometry 노드

`icp_odometry` 사용 (3D LiDAR 포인트클라우드 기반):

```python
# rtabmap_odom/icp_odometry
'frame_id': 'base_footprint',
'odom_frame_id': 'odom_rtabmap',
'publish_tf': True,

# ICP 파라미터
'Icp/VoxelSize': '0.1',
'Icp/MaxCorrespondenceDistance': '0.5',
'Icp/Iterations': '30',
'Icp/PointToPlane': 'true',
'Icp/PointToPlaneK': '20',
```

입력 토픽: 3D LiDAR의 `PointCloud2` (`/scan/points` 등 - LiDAR에 따라 변경)

### 2.4 SLAM 노드 Registration

```python
# Reg/Strategy: 2 = ICP(메인) + Visual(보조)
'Reg/Strategy': '2',
'Reg/Force3DoF': 'false',       # 6DoF (3D SLAM)

# ICP 관련
'Icp/VoxelSize': '0.1',
'Icp/PointToPlane': 'true',

# Visual 보조 관련
'Vis/MinInliers': '15',
'Vis/InlierDistance': '0.1',
'Vis/FeatureType': '6',         # ORB
'Vis/MaxFeatures': '1000',
```

### 2.5 Grid/맵 생성

```python
'Grid/Sensor': '0',             # 0=both (LiDAR + depth 모두 사용)
'Grid/RangeMax': '20.0',        # 3D LiDAR 범위에 맞게 조정
'Grid/RangeMin': '0.3',
'Grid/CellSize': '0.1',
'Grid/3D': 'true',
'Grid/RayTracing': 'true',
```

`Grid/Sensor` 옵션:
- `0`: LiDAR + depth 모두 사용 (권장)
- `1`: depth만 사용
- `2`: LiDAR만 사용

## 3. 토픽 구조

### 3.1 필요 입력 토픽

| 토픽 | 타입 | 소스 | 용도 |
|------|------|------|------|
| `/scan/points` (가칭) | `sensor_msgs/PointCloud2` | 3D LiDAR | ICP Odom + SLAM 메인 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB-D 카메라 | Visual 특징 보조 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RGB-D 카메라 | 카메라 내부 파라미터 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | RGB-D 카메라 | depth 맵 보강 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | RGB-D 카메라 | depth 카메라 파라미터 |

> 3D LiDAR 토픽명은 선정 후 확정 필요

### 3.2 Remapping 구성 (예상)

```python
# icp_odometry 노드
remappings=[
    ('scan_cloud', '/scan/points'),  # 3D LiDAR 토픽 (LiDAR 선정 후 확정)
]

# rtabmap SLAM 노드
remappings=[
    ('scan_cloud', '/scan/points'),                    # 3D LiDAR
    ('odom', '/rtabmap/odom'),                         # ICP odom 출력
    ('rgb/image', '/camera/color/image_raw'),          # RGB
    ('rgb/camera_info', '/camera/color/camera_info'),  # RGB info
    ('depth/image', '/camera/depth/image_raw'),        # Depth
    ('depth/camera_info', '/camera/depth/camera_info'),# Depth info
]
```

## 4. TF 프레임 구조

```
map
 └── odom_rtabmap (ICP odometry 발행)
      └── base_footprint
           ├── base_link
           │    └── lidar_3d_link (3D LiDAR - TF 위치 미정)
           └── camera_link
                └── camera_color_optical_frame
                └── camera_depth_optical_frame
```

> `lidar_3d_link` 프레임명과 TF는 3D LiDAR 선정 및 장착 위치 확정 후 결정

## 5. 3D LiDAR 선정 시 확인 사항

3D LiDAR 선정 시 아래 항목을 확인하여 launch 파라미터에 반영해야 함:

| 항목 | 확인 내용 | 영향 파라미터 |
|------|----------|--------------|
| PointCloud2 토픽명 | 드라이버 기본 출력 토픽 | `scan_cloud` remapping |
| 프레임 ID | 드라이버 기본 frame_id | static_transform 설정 |
| 최대 측정 거리 | 스펙시트 확인 | `Grid/RangeMax` |
| 포인트 밀도 | 채널 수, 회전 속도 | `Icp/VoxelSize`, `OdomF2M/ScanMaxSize` |
| FOV | 수직/수평 시야각 | LiDAR 장착 위치 결정 |
| ROS2 드라이버 | 공식 패키지 존재 여부 | launch 파일 include |

## 6. Reg/Strategy: 1 vs 2 선택 가이드

### Strategy 1 (ICP only) 권장 환경
- 시각적 특징이 부족한 환경 (창고, 빈 복도)
- RGB-D 카메라가 불안정하거나 화각이 좁은 경우
- 연산 자원이 제한적인 경우

### Strategy 2 (ICP + Visual) 권장 환경
- 시각적 특징이 풍부한 실내/사무실 환경
- 넓은 공간에서 루프 클로저 검출 성공률을 높이고 싶은 경우
- RGB-D 카메라가 안정적으로 동작하는 경우
- AMR이 동일 경로를 반복 주행하는 경우 (Visual로 추가 검증)

## 7. 구현 계획 (3D LiDAR 선정 후)

1. 3D LiDAR ROS2 드라이버 설치 및 토픽 확인
2. static_transform에 3D LiDAR 프레임 추가
3. 신규 launch 파일 작성 (mapping + localization)
4. 신규 rviz 설정 파일 작성
5. 파라미터 튜닝 (ICP VoxelSize, Grid/RangeMax 등)
6. 테스트 및 검증
