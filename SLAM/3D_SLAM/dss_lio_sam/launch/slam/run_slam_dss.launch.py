"""LIO-SAM SLAM — DSS 시뮬레이터 (dss_ros2_bridge 스트림).

원본 `run_slam_gazebo.launch.py` 를 DSS 로 개조한 것.

기동 그래프:

    dss_ros2_bridge (별도 launch, 선행)
        /dss/sensor/lidar3d ─────────────────→ imageProjection
        /dss/sensor/imu ──→ dss_imu_gravity ──→ imuPreintegration · imageProjection
        /clock ────────────────────────────────→ 전 노드 (use_sim_time)

    static TF: map→odom (원본과 동일), base_link→lidar_link/imu_link (DSS 전용 신설)

DSS 개조 내역 (원본 run_slam_gazebo 대비):
  - 파라미터 params_slam_gazebo.yaml → params_dss.yaml (실측 기반, 파일 헤더 참조)
  - dss_imu_gravity 노드 추가 — DSS IMU 가 중력을 안 실어 preintegration 이 발산한다
    (ADR 2026-08-16 D5). use_imu_gravity:=false 로 끄면 원본 IMU 토픽을 그대로 쓴다.
  - base_link→lidar_link/imu_link static TF 추가 — 브리지가 TF 를 발행하지 않는다
  - launch_utils.setup_gpu_offload() 제거 — 원본 워크스페이스 전용
  - 지도 저장 경로 ~/Study/... → <Divine>/maps/lio_sam/ (인자 save_pcd_dir 로 덮기 가능)
  - rviz 설정을 소스 트리 역산(src_dir) 대신 설치본(share)에서 읽는다

사용:
    ros2 launch dss_lio_sam run_slam_dss.launch.py
    ros2 launch dss_lio_sam run_slam_dss.launch.py rviz:=false use_imu_gravity:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 원본 LIO-SAM 은 savePCDDirectory 앞에 $HOME 을 무조건 붙인다 (mapOptmization.cpp:602).
# 따라서 HOME-상대 경로로 준다 — 실제 저장 위치는 $HOME + 이 값.
DEFAULT_SAVE_DIR = '/Project/Divine/maps/lio_sam/'


def generate_launch_description():
    share_dir = get_package_share_directory('dss_lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_imu_gravity = LaunchConfiguration('use_imu_gravity')
    rviz = LaunchConfiguration('rviz')
    rviz_config_file = os.path.join(share_dir, 'rviz2', 'lio_sam.rviz')

    common = {'use_sim_time': use_sim_time}
    # LIO-SAM 4개 노드가 공유하는 파라미터 묶음. imuTopic 은 use_imu_gravity 에 따라
    # 합성 스트림(기본, params_dss.yaml 값) 또는 브리지 원본으로 정해진다.
    imu_topic_override = LaunchConfiguration('imu_topic_resolved')

    lio_nodes = [
        Node(package='dss_lio_sam', executable=f'dss_lio_sam_{exe}',
             name=f'dss_lio_sam_{exe}',
             parameters=[parameter_file, common, {'imuTopic': imu_topic_override},
                         *extra],
             output='screen')
        for exe, extra in [
            ('imuPreintegration', []),
            ('imageProjection', []),
            ('featureExtraction', []),
            ('mapOptimization', [{'savePCDDirectory': LaunchConfiguration('save_pcd_dir')}]),
        ]
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'params_dss.yaml'),
            description='LIO-SAM 파라미터 파일 (기본: DSS 실측 기반 params_dss.yaml)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='DSS /clock 사용 — 센서 스탬프가 sim time 이라 false 면 TF 가 어긋난다'),
        DeclareLaunchArgument(
            'use_imu_gravity', default_value='true',
            description='DSS IMU 에 중력을 합성하는 릴레이 사용 (ADR D5). false=브리지 원본 사용'),
        DeclareLaunchArgument(
            'save_pcd_dir', default_value=DEFAULT_SAVE_DIR,
            description='지도 PCD 저장 디렉토리'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz2 실행'),

        # use_imu_gravity 에 따라 LIO-SAM 이 구독할 IMU 토픽을 정한다.
        SetLaunchConfiguration('imu_topic_resolved', '/dss/sensor/imu_gravity',
                               condition=IfCondition(use_imu_gravity)),
        SetLaunchConfiguration('imu_topic_resolved', '/dss/sensor/imu',
                               condition=UnlessCondition(use_imu_gravity)),

        # ── static TF ───────────────────────────────────────────────
        # map→odom: 원본 launch 와 동일 (LIO-SAM 은 odom 기준으로 궤적을 낸다)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_map_to_odom',
             arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
             parameters=[common], output='screen'),
        # base_link→lidar_link/imu_link: DSS 전용 신설. 단위 변환은 측정값이 아니라
        # 가정이다 (ADR D4·debt). extrinsicTrans(params_dss.yaml)와 함께 바꿔야 한다.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_base_to_lidar',
             arguments='0.0 0.0 0.0 0.0 0.0 0.0 base_link lidar_link'.split(' '),
             parameters=[common], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_base_to_imu',
             arguments='0.0 0.0 0.0 0.0 0.0 0.0 base_link imu_link'.split(' '),
             parameters=[common], output='screen'),

        # ── DSS IMU 중력 합성 (opt-in, 기본 ON) ─────────────────────
        Node(package='dss_lio_sam', executable='dss_imu_gravity.py',
             name='dss_imu_gravity',
             parameters=[common, {'gravity': 9.80511}],   # params_dss.yaml imuGravity 와 동일
             condition=IfCondition(use_imu_gravity),
             output='screen'),

        # ── LIO-SAM 4개 노드 ────────────────────────────────────────
        *lio_nodes,

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config_file],
             parameters=[common],
             condition=IfCondition(rviz),
             output='screen'),
    ])
