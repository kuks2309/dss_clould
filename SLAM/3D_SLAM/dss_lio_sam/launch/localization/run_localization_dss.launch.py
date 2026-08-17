"""LIO-SAM Localization — DSS 시뮬레이터 (기존 지도 위 위치 인식).

`run_slam_dss.launch.py` 로 만든 지도(maps/lio_sam/)가 **먼저 있어야** 한다.
구성은 SLAM launch 와 동일(4노드 + 중력 릴레이 + static TF 3 + rviz)하고,
파라미터만 params_dss_localization.yaml(지도 로드·증분 매핑 off·루프클로저 off).

초기 포즈:
  - 차량이 지도 원점(스폰 위치)에서 시작하면 그대로 동작한다.
  - 다른 위치에서 시작하면 RViz "2D Pose Estimate" 로 /initialpose 를 찍는다
    (mapOptimization 이 큐 리셋 후 그 포즈에서 재정합 — mapOptmization.cpp:474).

사용:
    ros2 launch dss_lio_sam run_localization_dss.launch.py
    ros2 launch dss_lio_sam run_localization_dss.launch.py \\
        map_path:=/home/amap/Project/Divine/maps/lio_sam/GlobalMap.pcd rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MAP = '/home/amap/Project/Divine/maps/lio_sam/GlobalMap.pcd'


def generate_launch_description():
    share_dir = get_package_share_directory('dss_lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_imu_gravity = LaunchConfiguration('use_imu_gravity')
    rviz = LaunchConfiguration('rviz')
    rviz_config_file = os.path.join(share_dir, 'rviz2', 'lio_sam.rviz')

    common = {'use_sim_time': use_sim_time}
    imu_topic_override = LaunchConfiguration('imu_topic_resolved')
    map_override = {'globalMapPath': LaunchConfiguration('map_path')}

    lio_nodes = [
        Node(package='dss_lio_sam', executable=f'dss_lio_sam_{exe}',
             name=f'dss_lio_sam_{exe}',
             parameters=[parameter_file, common, {'imuTopic': imu_topic_override},
                         map_override],
             output='screen')
        for exe in ('imuPreintegration', 'imageProjection',
                    'featureExtraction', 'mapOptimization')
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'params_dss_localization.yaml'),
            description='LIO-SAM localization 파라미터 (기본: DSS 지도 로드 구성)'),
        DeclareLaunchArgument(
            'map_path', default_value=DEFAULT_MAP,
            description='기준 지도 GlobalMap.pcd 경로 (같은 폴더의 Corner/SurfMap.pcd 동반)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='DSS /clock 사용 — 센서 스탬프가 sim time'),
        DeclareLaunchArgument(
            'use_imu_gravity', default_value='true',
            description='DSS IMU 중력 합성 릴레이 (ADR D5). false=브리지 원본 사용'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz2 실행'),

        SetLaunchConfiguration('imu_topic_resolved', '/dss/sensor/imu_gravity',
                               condition=IfCondition(use_imu_gravity)),
        SetLaunchConfiguration('imu_topic_resolved', '/dss/sensor/imu',
                               condition=UnlessCondition(use_imu_gravity)),

        # static TF — SLAM launch 와 동일 (값은 가정, debt-002)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_map_to_odom',
             arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
             parameters=[common], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_base_to_lidar',
             arguments='0.0 0.0 0.0 0.0 0.0 0.0 base_link lidar_link'.split(' '),
             parameters=[common], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_base_to_imu',
             arguments='0.0 0.0 0.0 0.0 0.0 0.0 base_link imu_link'.split(' '),
             parameters=[common], output='screen'),

        Node(package='dss_lio_sam', executable='dss_imu_gravity.py',
             name='dss_imu_gravity',
             parameters=[common, {'gravity': 9.80511}],
             condition=IfCondition(use_imu_gravity),
             output='screen'),

        *lio_nodes,

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config_file],
             parameters=[common],
             condition=IfCondition(rviz),
             output='screen'),
    ])
