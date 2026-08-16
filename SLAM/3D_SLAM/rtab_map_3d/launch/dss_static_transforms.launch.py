"""DSS 센서 프레임의 static TF 발행.

`dss_ros2_bridge` 는 센서 메시지의 `frame_id` 로 `lidar_link`·`imu_link` 를 쓰지만
**TF 는 한 건도 발행하지 않는다**. RTAB-Map `icp_odometry` 는 LIO-SAM 과 달리 센서
외부 파라미터를 파라미터가 아니라 **TF 로 조회**하므로, 이 TF 가 없으면
`wait_for_transform` 타임아웃으로 스캔을 한 장도 처리하지 못한다.

TF 트리 (본 파일이 채우는 구간은 base_link 아래 두 줄):

    map ── odom ── base_link ── lidar_link      (odom 이하 SLAM 노드가 발행)
                            └─ imu_link

기본값 (0,0,0 / 0,0,0) 은 **측정값이 아니라 가정**이다. DSS 는 센서 장착 오프셋을
어떤 채널로도 내보내지 않으며(protobuf 에 외부 파라미터 필드 없음), 제어포트(8886)가
불통이라 캘리브레이션 기동도 불가하다. 실제 값이 확인되면 launch 인자로 덮는다:

  ros2 launch rtab_map_3d_config dss_static_transforms.launch.py \\
      lidar_z:=0.19 imu_z:=0.05

→ 부채: docs/debt/registry.md
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (인자 이름, 기본값, 설명) — base_link 기준 센서 장착 위치·자세
_SENSOR_ARGS = [
    ('lidar_x', '0.0', 'base_link→lidar_link 전방 오프셋 (m)'),
    ('lidar_y', '0.0', 'base_link→lidar_link 좌측 오프셋 (m)'),
    ('lidar_z', '0.0', 'base_link→lidar_link 상방 오프셋 (m)'),
    ('lidar_roll', '0.0', 'base_link→lidar_link roll (rad)'),
    ('lidar_pitch', '0.0', 'base_link→lidar_link pitch (rad)'),
    ('lidar_yaw', '0.0', 'base_link→lidar_link yaw (rad)'),
    ('imu_x', '0.0', 'base_link→imu_link 전방 오프셋 (m)'),
    ('imu_y', '0.0', 'base_link→imu_link 좌측 오프셋 (m)'),
    ('imu_z', '0.0', 'base_link→imu_link 상방 오프셋 (m)'),
    ('imu_roll', '0.0', 'base_link→imu_link roll (rad)'),
    ('imu_pitch', '0.0', 'base_link→imu_link pitch (rad)'),
    ('imu_yaw', '0.0', 'base_link→imu_link yaw (rad)'),
]


def _static_tf(name, parent, child, prefix, use_sim_time):
    """launch 인자에서 값을 읽는 static_transform_publisher 노드 하나."""
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', LaunchConfiguration(f'{prefix}_x'),
            '--y', LaunchConfiguration(f'{prefix}_y'),
            '--z', LaunchConfiguration(f'{prefix}_z'),
            '--roll', LaunchConfiguration(f'{prefix}_roll'),
            '--pitch', LaunchConfiguration(f'{prefix}_pitch'),
            '--yaw', LaunchConfiguration(f'{prefix}_yaw'),
            '--frame-id', parent,
            '--child-frame-id', child,
        ],
        output='screen',
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='DSS 는 /clock 을 발행한다 — 센서 스탬프가 sim time 이므로 true 가 기본'),
        *[DeclareLaunchArgument(n, default_value=d, description=h)
          for n, d, h in _SENSOR_ARGS],
        _static_tf('dss_tf_base_to_lidar', 'base_link', 'lidar_link', 'lidar', use_sim_time),
        _static_tf('dss_tf_base_to_imu', 'base_link', 'imu_link', 'imu', use_sim_time),
    ])
