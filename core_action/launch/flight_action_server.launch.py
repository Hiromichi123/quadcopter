from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 Action Server
        Node(
            package='core_action',
            executable='flight_action_server',
            name='flight_action_server',
            output='screen',
            parameters=[],
        ),
    ])
