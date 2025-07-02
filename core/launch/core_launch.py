import os
import launch
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取所有包的share目录路径
    livox_share_dir = get_package_share_directory('livox_ros_driver2')
    fast_lio_share_dir = get_package_share_directory('fast_lio')

    ros2_tools_nodes = [
        'lidar_data_node',
        'lidar_to_px4_node',
        'ground_camera_node'
    ]

    return launch.LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            parameters=[{
                'fcu_url': 'serial:///dev/ttyACM0:921600',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0'
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch/msg_MID360_launch.py')
            ])
        ),
        TimerAction(
            period=10.0, # 延迟10s启动fastlio
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('fast_lio'), 'launch/mapping.launch.py')
                    ]),
                    launch_arguments={'rviz': 'false'}.items()
                )
            ]
        ),
        *[Node(
            package='ros2_tools',
            executable=node
            ) for node in ros2_tools_nodes],
        '''Node(
            package='vision',
            executable='vision_node.py',
        ),'''
        Node(
            package='servo_control',
            executable='servo_control',
        ),
        Node(
            package='core',
            executable='quad_node',
        )
    ])
