# RTAB-Map 3D SLAM 실행 명령어

패키지명: `rtab_map_3d_config`

> 본 프로젝트는 Gazebo 시뮬레이션 환경에서 실행합니다.

## 빌드

```bash
cd ~/Study/ros2_3dslam_ws
colcon build --packages-select rtab_map_3d_config
source install/setup.bash
```

## 실행 순서

### 1단계: Gazebo 시뮬레이터 기동

> RTAB-Map 3D는 자체 odom 프레임(`odom_rtabmap`)을 사용하므로 Gazebo odom TF와 충돌하지 않습니다.
> 단, 3D LiDAR 전용 모드에서는 `/odom` 토픽이 불필요하므로 `gazebo_no_odom` 사용을 권장합니다.

```bash
ros2 launch tm_gazebo gazebo_no_odom.launch.py
```

### 2단계: SLAM 기동

---

#### RGB-D 전용 (Depth Camera)

```bash
# 매핑
ros2 launch rtab_map_3d_config rtabmap_rgbd_slam_gazebo.launch.py

# 위치추정
ros2 launch rtab_map_3d_config rtabmap_rgbd_localization_gazebo.launch.py
```

---

#### 2D LiDAR (주) + RGB-D (보조)

```bash
# 매핑
ros2 launch rtab_map_3d_config rtabmap_2dlidar_rgbd_slam_gazebo.launch.py

# 위치추정
ros2 launch rtab_map_3d_config rtabmap_2dlidar_rgbd_localization_gazebo.launch.py
```

---

#### 3D LiDAR 전용 (카메라 없음)

```bash
# 매핑
ros2 launch rtab_map_3d_config rtabmap_3dlidar_only_slam_gazebo.launch.py

# 위치추정
ros2 launch rtab_map_3d_config rtabmap_3dlidar_only_localization_gazebo.launch.py
```

---

#### 3D LiDAR + RGB 카메라 (Depth 없음, 시각적 루프 클로저)

```bash
# 매핑
ros2 launch rtab_map_3d_config rtabmap_3dlidar_rgb_slam_gazebo.launch.py

# 위치추정
ros2 launch rtab_map_3d_config rtabmap_3dlidar_rgb_localization_gazebo.launch.py
```

---

#### 3D LiDAR + RGB-D 카메라 (ICP + 시각적 루프 클로저)

```bash
# 매핑
ros2 launch rtab_map_3d_config rtabmap_3dlidar_rgbd_slam_gazebo.launch.py

# 위치추정
ros2 launch rtab_map_3d_config rtabmap_3dlidar_rgbd_localization_gazebo.launch.py
```

---

#### 3D LiDAR + RGB-D (Livox Mid-360, ICP + RGBD odometry)

```bash
# 매핑
ros2 launch rtab_map_3d_config rtabmap_livox_rgbd_slam_gazebo.launch.py

# 위치추정
ros2 launch rtab_map_3d_config rtabmap_livox_rgbd_localization_gazebo.launch.py
```

---

#### 전체 스택 (mapping / localization 전환 지원)

```bash
# 매핑 모드
ros2 launch rtab_map_3d_config rtabmap_3d_full_gazebo.launch.py mode:=mapping

# 위치추정 모드
ros2 launch rtab_map_3d_config rtabmap_3d_full_gazebo.launch.py mode:=localization
```

---

> **미지원 (Gazebo 브리지 미설정)**
> - `rtabmap_astra_3d_slam_gazebo.launch.py` — Astra Pro 카메라 드라이버 불필요 (Gazebo 카메라 사용)

---

## 실행 옵션

| 인수 | 기본값 | 설명 |
|------|--------|------|
| `use_sim_time` | `true` | 시뮬레이션 시간 사용 |
| `rviz` | `true` | RViz 실행 여부 |
| `database_path` | `~/.ros/rtabmap.db` | DB 파일 경로 (localization 전용) |

```bash
# RViz 없이 실행
ros2 launch rtab_map_3d_config rtabmap_rgbd_slam_gazebo.launch.py rviz:=false

# 커스텀 DB 경로 (localization)
ros2 launch rtab_map_3d_config rtabmap_rgbd_localization_gazebo.launch.py \
  database_path:=$HOME/Study/ros2_3dslam_ws/maps/rtabmap_3d/rtabmap.db
```

## DB 파일 위치

```
~/.ros/rtabmap.db
```

매핑 완료 후 자동 저장됩니다. 위치추정 시 동일 경로를 사용합니다.

## 기동되는 주요 노드

| 노드 | 설명 |
|------|------|
| `rgbd_odometry` 또는 `icp_odometry` | Odometry 추정 (센서 모드에 따라 다름) |
| `rtabmap` | SLAM 메인 노드 (지도 생성 / 위치추정) |
| `rviz2` | 3D 포인트클라우드 + 지도 시각화 |

## 센서 정보 (Gazebo Pioneer2dx)

| 센서 | 토픽 | 타입 | 주기 |
|------|------|------|------|
| 3D LiDAR | `/scan/points` | PointCloud2 (16ch) | 20 Hz |
| IMU | `/imu/data` | Imu | 200 Hz |
| RGB 카메라 | `/camera/color/image_raw` | Image | 30 Hz |
| Depth 카메라 | `/camera/depth/image_raw` | Image | 30 Hz |

## 주의사항

- Gazebo를 먼저 기동한 후 SLAM을 실행해야 센서 토픽이 수신됩니다.
- Gazebo 브리지 QoS는 BEST_EFFORT → `_gazebo` 런치 파일에 `qos_*: 2` 설정 포함.
- 3D LiDAR 모드는 `/scan/points` (PointCloud2) 토픽을 사용합니다.
- ICP odometry는 `/scan/points` 토픽에서 포인트클라우드를 수신합니다.
- 3D LiDAR + RGB-D 모드는 ICP odometry와 RGBD odometry를 함께 사용합니다.
