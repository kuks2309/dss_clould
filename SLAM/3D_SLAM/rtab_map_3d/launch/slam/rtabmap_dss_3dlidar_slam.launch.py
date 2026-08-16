"""RTAB-Map SLAM — DSS 3D 라이다 단독 (카메라 없음).

원본 `rtabmap_3dlidar_only_slam_gazebo.launch.py` 를 DSS 브리지 스트림으로 개조한 것.

구성:
    icp_odometry  →  rtabmap (Reg/Strategy=1, ICP 전용)

입력 (dss_ros2_bridge 발행):
    /dss/sensor/lidar3d   sensor_msgs/PointCloud2  frame=lidar_link  10.00 Hz
                          x,y,z,intensity,ring(uint16),time(float32) · 6750 pt/scan
    /clock                rosgraph_msgs/Clock      (use_sim_time 필수)

TF: 본 launch 가 dss_static_transforms 를 include 해 base_link→lidar_link 를 채운다.
    나머지(map→odom→base_link)는 rtabmap·icp_odometry 가 발행한다.

DSS 개조 내역 (원본 대비):
  - scan_cloud 리맵 /scan/points → /dss/sensor/lidar3d
  - frame_id base_footprint → base_link (DSS 에는 base_footprint 개념이 없다)
  - odom_frame_id odom_rtabmap → odom (Divine 표준 REP-105 이름)
  - launch_utils.setup_gpu_offload() 제거 — 원본 워크스페이스 전용 패키지
  - rviz 설정 경로의 ~/Study/... 하드코딩 → get_package_share_directory
  - static TF include 추가 — 브리지가 TF 를 발행하지 않는다

카메라를 쓰지 않는 이유: DSS 브리지는 Image 는 내지만 **CameraInfo 를 발행하지 않는다**.
RTAB-Map RGB-D 계열은 CameraInfo 없이 동작하지 않는다 (ADR 2026-08-16 D8).

사용:
    ros2 launch rtab_map_3d_config rtabmap_dss_3dlidar_slam.launch.py
    ros2 launch rtab_map_3d_config rtabmap_dss_3dlidar_slam.launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# DSS 라이다 실측 특성 (2026-08-16). ICP 파라미터의 근거이므로 값과 함께 남긴다.
DSS_SCAN_HZ = 10.0        # Hz — 스탬프 간격 0.1000 s
DSS_RANGE_MAX_M = 100.0   # m  — 실측 최대 99.69, 브리지 역양자화 상한과 동일


def generate_launch_description():
    pkg_share = get_package_share_directory('rtab_map_3d_config')
    rviz_config = os.path.join(pkg_share, 'rviz2', 'rtabmap_3d.rviz')

    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_cloud_topic = LaunchConfiguration('scan_cloud_topic')

    # --- 0. DSS 센서 static TF (브리지가 발행하지 않으므로 여기서 채운다) ---
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'dss_static_transforms.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # --- 1. ICP Odometry (3D 라이다 기반) ---
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform': 0.2,       # s
            'approx_sync': True,
            'queue_size': 10,
            'Odom/ResetCountdown': '3',
            'Icp/VoxelSize': '0.1',                  # m
            'Icp/MaxCorrespondenceDistance': '0.5',  # m
            'Icp/RangeMax': '20.0',                  # m — 16채널 0.72° 간격이라 원거리는 희박
            'Icp/RangeMin': '0.3',                   # m
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0.0',
            'Odom/ScanKeyFrameThr': '0.6',
            'OdomF2M/ScanSubtractRadius': '0.1',     # m
            'OdomF2M/ScanMaxSize': '15000',
            # QoS 2 = BEST_EFFORT — 브리지가 SensorDataQoS(BEST_EFFORT) 로 발행한다.
            # RELIABLE 로 요구하면 offered<requested 가 되어 구독이 아예 연결되지 않는다.
            'qos_scan_cloud': 2,
        }],
        remappings=[('scan_cloud', scan_cloud_topic)],
        output='screen',
    )

    # --- 2. RTAB-Map SLAM (3D 라이다 전용) ---
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            'qos_scan_cloud': 2,             # BEST_EFFORT — 위와 같은 이유
            # SLAM 모드 (증분 매핑)
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            # 루프 클로저 / 근접 탐색
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',    # rad
            'RGBD/LinearUpdate': '0.05',     # m
            # 정합: ICP 전용 (카메라 없음)
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'false',
            # 3D 점유 격자
            'Grid/RangeMax': '20.0',         # m
            'Grid/RangeMin': '0.3',          # m
            'Grid/CellSize': '0.1',          # m
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            'Icp/VoxelSize': '0.1',          # m
            'Icp/PointToPlane': 'true',
        }],
        remappings=[
            ('scan_cloud', scan_cloud_topic),
            ('odom', '/rtabmap/odom'),
            ('odom_info', '/rtabmap/odom_info'),
        ],
        arguments=['-d'],   # 기존 DB 삭제 후 새 지도 생성
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='RViz2 실행'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='DSS /clock 사용. 센서 스탬프가 sim time 이라 false 면 TF 시각이 어긋난다'),
        DeclareLaunchArgument(
            'scan_cloud_topic', default_value='/dss/sensor/lidar3d',
            description='DSS 브리지 3D 라이다 토픽'),
        static_tf,
        icp_odometry,
        rtabmap_slam,
        rviz_node,
    ])
