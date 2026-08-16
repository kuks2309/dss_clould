"""RTAB-Map Localization — DSS 3D 라이다 단독 (카메라 없음).

원본 `rtabmap_3dlidar_only_localization_gazebo.launch.py` 를 DSS 브리지 스트림으로 개조한 것.
`rtabmap_dss_3dlidar_slam.launch.py` 로 만든 데이터베이스가 **먼저 있어야** 한다.

구성:
    icp_odometry  →  rtabmap (Mem/IncrementalMemory=false, 기존 DB 로드)

SLAM launch 와의 차이는 세 곳뿐이다:
  - `Mem/IncrementalMemory: false` — 지도를 늘리지 않고 기존 지도 위에서 위치만 찾는다
  - `arguments=['-d']` 없음 — DB 를 지우지 않는다
  - `Optimizer/Robust` · `RGBD/ProximityBySpace` 활성 — 재방문 정합 강화

사용:
    ros2 launch rtab_map_3d_config rtabmap_dss_3dlidar_localization.launch.py
    ros2 launch rtab_map_3d_config rtabmap_dss_3dlidar_localization.launch.py \\
        database_path:=/home/amap/Project/Divine/maps/rtabmap_3d/rtabmap.db
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# SLAM launch 가 -d 없이 쓰는 기본 DB 위치와 같아야 한다 (rtabmap 기본값).
DEFAULT_DB = os.path.join(os.path.expanduser('~'), '.ros', 'rtabmap.db')


def generate_launch_description():
    pkg_share = get_package_share_directory('rtab_map_3d_config')
    rviz_config = os.path.join(pkg_share, 'rviz2', 'rtabmap_localization.rviz')

    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    scan_cloud_topic = LaunchConfiguration('scan_cloud_topic')

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'dss_static_transforms.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

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
            'Icp/RangeMax': '20.0',                  # m
            'Icp/RangeMin': '0.3',                   # m
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0.0',
            'Odom/ScanKeyFrameThr': '0.6',
            'OdomF2M/ScanSubtractRadius': '0.1',     # m
            'OdomF2M/ScanMaxSize': '15000',
            'qos_scan_cloud': 2,             # BEST_EFFORT — 브리지 SensorDataQoS 와 맞춘다
        }],
        remappings=[('scan_cloud', scan_cloud_topic)],
        output='screen',
    )

    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{'database_path': database_path}, {
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
            'qos_scan_cloud': 2,
            # Localization 모드 — 지도를 늘리지 않는다
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'RGBD/StartAtOrigin': 'false',
            # 재방문 정합
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityOdomGuess': 'true',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '10',
            'RGBD/AngularUpdate': '0.05',    # rad
            'RGBD/LinearUpdate': '0.05',     # m
            'Reg/Strategy': '1',             # ICP 전용
            'Reg/Force3DoF': 'false',
            'Grid/RangeMax': '20.0',         # m
            'Grid/RangeMin': '0.3',          # m
            'Grid/CellSize': '0.1',          # m
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            'Icp/VoxelSize': '0.1',          # m
            'Icp/PointToPlane': 'true',
            'Optimizer/Robust': 'true',
            'RGBD/OptimizeMaxError': '0.0',
        }],
        remappings=[
            ('scan_cloud', scan_cloud_topic),
            ('odom', '/rtabmap/odom'),
            ('odom_info', '/rtabmap/odom_info'),
        ],
        # '-d' 없음 — 기존 DB 를 지우지 않고 로드한다
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
            description='DSS /clock 사용 — 센서 스탬프가 sim time'),
        DeclareLaunchArgument(
            'database_path', default_value=DEFAULT_DB,
            description='SLAM 단계에서 만든 RTAB-Map DB 경로'),
        DeclareLaunchArgument(
            'scan_cloud_topic', default_value='/dss/sensor/lidar3d',
            description='DSS 브리지 3D 라이다 토픽'),
        static_tf,
        icp_odometry,
        rtabmap_localization,
        rviz_node,
    ])
