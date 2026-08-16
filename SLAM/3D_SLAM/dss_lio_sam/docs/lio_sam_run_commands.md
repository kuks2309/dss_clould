# LIO-SAM (Livox) 실행 명령어

패키지명: `dss_lio_sam`

---

## 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select dss_lio_sam
source install/setup.bash
```

---

## Gazebo 시뮬레이션 (Pioneer2dx 모델)

LIO-SAM은 이제 Gazebo 환경에서 완전히 지원됩니다. Pioneer2dx 모델에는 16채널 3D LiDAR와 IMU 센서가 탑재되어 있습니다.

### 센서 사양 (Gazebo Pioneer2dx)

| 센서 | 토픽 | 타입 | 주파수 | 프레임 |
|------|------|------|--------|--------|
| 3D LiDAR | `/scan/points` | PointCloud2 | 20 Hz | lidar_link |
| IMU | `/imu/data` | Imu | 200 Hz | base_link |

### 1단계: Gazebo 시뮬레이터 기동 (Gazebo 브리지 포함)

> **odom_tf:=false 필수!** LIO-SAM은 자체적으로 `odom → base_link` TF를 발행합니다.
> `odom_tf:=false`를 빼면 Gazebo의 odom_to_tf와 충돌하여 **맵이 회전/드리프트**합니다.

```bash
ros2 launch tm_gazebo gazebo_no_odom.launch.py odom_tf:=false
```

**Gazebo 브리지 확인**: 다음 토픽들이 활성화되어야 합니다.
```bash
ros2 topic list | grep -E "(scan/points|imu/data)"
```

### 2단계: SLAM 모드 기동 (새 지도 생성)

별도의 터미널에서 SLAM을 실행합니다.

```bash
ros2 launch dss_lio_sam run_slam_gazebo.launch.py
```

실행되는 주요 노드:
- `dss_lio_sam_imuPreintegration`: IMU 사전적분 및 초기 자세 추정
- `dss_lio_sam_imageProjection`: LiDAR 포인트 투영 및 왜곡 보정
- `dss_lio_sam_featureExtraction`: 엣지/평면 특징 추출
- `dss_lio_sam_mapOptimization`: ICP + 루프 클로저 + 지도 최적화
- `rviz2`: 3D 포인트클라우드 및 경로 시각화

### 3단계: 지도 저장

SLAM 모드 실행 중 로봇을 충분히 이동시킨 후 지도를 저장합니다.

```bash
ros2 service call /lio_sam/save_map dss_lio_sam/srv/SaveMap "{}"
```

저장 위치: `~/Study/ros2_3dslam_ws/maps/lio_sam/`

저장되는 파일:
- `GlobalMap.pcd` - 전체 맵 (Localization에서 사용)
- `CornerMap.pcd` - 엣지 특징 맵
- `SurfMap.pcd` - 평면 특징 맵
- `trajectory.pcd` - 로봇 경로
- `transformations.pcd` - 키프레임 변환

### 4단계: Localization 모드 기동 (저장된 지도 사용)

사전에 생성된 지도를 사용하여 로봇의 위치를 추정합니다.

> **주의:** Gazebo를 먼저 종료 후 재시작해야 합니다 (SLAM과 동시 실행 불가).

```bash
# 터미널 1: Gazebo 재시작
ros2 launch tm_gazebo gazebo_no_odom.launch.py odom_tf:=false

# 터미널 2: LIO-SAM Localization
ros2 launch dss_lio_sam run_localization_gazebo.launch.py
```

이 모드에서는 전역 맵이 고정되며, 로봇은 맵 내에서의 자신의 위치만 추정합니다.

---

## 검증 명령어

SLAM/Localization 실행 중 별도 터미널에서 확인할 수 있습니다.

```bash
# 센서 주파수 확인
ros2 topic hz /scan/points      # ~20Hz 정상
ros2 topic hz /imu/data         # ~200Hz 정상

# TF 확인 (roll ≈ 0, pitch ≈ 0 이어야 정상)
ros2 run tf2_ros tf2_echo odom base_link

# 맵핑 오도메트리 위치 확인
ros2 topic echo /lio_sam/mapping/odometry --field pose.pose.position

# 노드 상태
ros2 node list | grep dss_lio_sam

# TF 트리 확인
ros2 run tf2_tools view_frames
```

---

## 파라미터 설명: Gazebo vs 실제 하드웨어

### 주요 차이점

Gazebo 시뮬레이션용 파라미터 파일과 실제 Livox 하드웨어용 파라미터 파일은 센서 사양이 다릅니다.

#### params_slam_gazebo.yaml (Gazebo Pioneer2dx)

```yaml
# Topics (Gazebo 시뮬레이션)
pointCloudTopic: "/scan/points"              # Gazebo 3D LiDAR
imuTopic: "/imu/data"                        # Gazebo IMU

# Sensor Settings (Pioneer2dx 16-channel LiDAR)
sensor: velodyne                             # 16-channel 센서 포맷
N_SCAN: 16                                   # 수직 채널 16개
Horizon_SCAN: 1800                           # 수평 샘플 1800개

# LiDAR - IMU 외부 파라미터 (Pioneer2dx SDF 기준)
extrinsicTrans: [ 0.0, 0.0, 0.19 ]          # lidar_link는 base_link 위 0.19m

# IMU 잡음 파라미터 (시뮬레이션용 - 낮은 잡음)
imuAccNoise: 0.01
imuGyrNoise: 0.001
```

#### params_slam.yaml (실제 Livox 하드웨어)

```yaml
# Topics (실제 Livox)
pointCloudTopic: "/livox/lidar"              # Livox MID-360
imuTopic: "/livox/imu"                       # Livox 내장 IMU

# Sensor Settings (Livox MID-360)
sensor: livox                                # Livox 센서 타입
N_SCAN: 6                                    # 수직 채널 6개
Horizon_SCAN: 4000                           # 수평 샘플 4000개

# LiDAR - IMU 외부 파라미터 (실제 하드웨어 기준)
extrinsicTrans: [ 1.07, 0.0, 0.15 ]

# IMU 잡음 파라미터 (실제 센서 - 높은 잡음)
imuAccNoise: 3.9939570888238808e-03
imuGyrNoise: 1.5636343949698187e-03
```

### params_localization_gazebo.yaml (Gazebo Localization)

SLAM 파라미터와 동일하지만 다음 설정이 다릅니다:

```yaml
localizationMode: true
globalMapPath: "~/Study/ros2_3dslam_ws/maps/lio_sam/GlobalMap.pcd"
savePCD: false                               # 지도 저장 안 함 (읽기만)
```

---

## bag 파일 재생

녹화된 ROS 2 bag 파일에서 SLAM을 실행합니다.

```bash
# 1단계: SLAM 기동 (use_sim_time 활성화)
ros2 launch dss_lio_sam run_slam.launch.py use_sim_time:=true

# 2단계: bag 재생 (별도 터미널)
ros2 bag play <bag_file> --clock
```

---

## 필요 Gazebo 브리지 설정

Gazebo에서 ROS 2로의 센서 데이터 변환은 다음과 같이 구성됩니다:

| 센서 | ROS 메시지 | Gazebo 메시지 |
|------|-----------|---------------|
| LiDAR | sensor_msgs/PointCloud2 | ignition.msgs.PointCloud |
| IMU | sensor_msgs/Imu | ignition.msgs.IMU |

이 변환은 `gazebo_no_odom.launch.py`의 Gazebo ROS 2 브리지에서 처리됩니다.

---

## 주요 노드

| 노드 | 설명 |
|------|------|
| `dss_lio_sam_imuPreintegration` | IMU 사전적분 → 초기 자세 추정 |
| `dss_lio_sam_imageProjection` | LiDAR 포인트 투영 및 왜곡 보정 |
| `dss_lio_sam_featureExtraction` | 엣지/평면 특징 추출 |
| `dss_lio_sam_mapOptimization` | ICP + 루프 클로저 + 지도 최적화 |
| `rviz2` | 3D 포인트클라우드 + 경로 시각화 |

---

## 시뮬레이션 환경 설정 요약

Gazebo 지원을 위해 다음이 구현되었습니다:

1. **Pioneer2dx SDF 모델**: 16채널 3D LiDAR + IMU 센서 탑재
2. **IMU 센서**: 200 Hz, `/imu/data` 토픽, base_link 프레임
3. **Gazebo 브리지**: `/scan/points` (PointCloud2) 및 `/imu/data` (Imu) 변환
4. **Gazebo 전용 파라미터**: `params_slam_gazebo.yaml`, `params_localization_gazebo.yaml`
5. **launch 파일**: `run_slam_gazebo.launch.py`, `run_localization_gazebo.launch.py`
