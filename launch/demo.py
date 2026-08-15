from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    #  모든 노드가 사용할 공통 파라미터
    common_params = {
        "use_sim_time": True,   #sim time 사용
        #"nats_server": "nats://172.25.96.1:4222",
        #"dss_server": "172.25.96.1",
        #"dss_port": 8886,
        #"nats_port": 4222,
    }

    return LaunchDescription([
        # DSS Demo
        Node(
           package='dss_ros2_bridge',
           executable='DSSDemoNode',
           name='Demo',
           output='screen',
           parameters=[common_params],
        ),
    ])
