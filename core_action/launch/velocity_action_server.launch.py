from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 Velocity Action Server
        Node(
            package='core_action',
            executable='velocity_action_server',
            name='velocity_action_server',
            output='screen',
            parameters=[],
        ),
    ])
