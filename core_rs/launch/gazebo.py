import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_tools',
            executable='lidar_data_node',
        ),
        launch_ros.actions.Node(
            package='core_rs',
            executable='main_node',
        )
    ])
