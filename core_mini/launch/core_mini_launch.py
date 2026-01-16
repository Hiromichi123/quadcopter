"""
这个launch文件用于启动所有必需的节点：
1. MAVROS节点：与飞控通信
2. 激光雷达相关节点：获取位置信息
3. 四旋翼控制节点：执行飞行任务

教学要点：
- Launch文件用于同时启动多个ROS2节点
- 可以设置节点的参数、命名空间等
- 使用TimerAction可以延迟启动某些节点
"""

import os
import launch
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    生成launch描述，最后返回一个LaunchDescription对象，包含所有要启动的节点
    """
    # 定义ros2_tools包中需要启动的节点列表
    ros2_tools_nodes = [
        'lidar_data_node',      # 激光雷达数据处理节点
        'lidar_to_px4_node',    # 激光雷达数据转换节点
    ]
    
    return LaunchDescription([
        # 1.启动MAVROS节点（用于与飞控PX4通信）
        # MAVROS是ROS与PX4飞控之间的桥梁
        Node(
            package='mavros',              # 包名
            executable='mavros_node',      # 可执行文件名
            parameters=[{
                'fcu_url': 'serial:///dev/ttyACM0:921600',   # 飞控连接方式（串口）
                'tgt_system': 1,                             # 目标系统ID
                'tgt_component': 1,                          # 目标组件ID
                'fcu_protocol': 'v2.0'                       # MAVLink协议版本
            }],
            output='screen'  # 输出到屏幕
        ),
        
        # 2.启动激光雷达驱动（Livox MID360）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('livox_ros_driver2'), 
                    'launch/msg_MID360_launch.py'
                )
            ])
        ),
        
        # 3.延迟10秒后启动SLAM（激光雷达惯性里程计）
        TimerAction(
            period=10.0,  # 延迟10秒
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(
                            get_package_share_directory('fast_lio'), 
                            'launch/mapping.launch.py'
                        )
                    ]),
                    launch_arguments={'rviz': 'false'}.items()  # 不启动RViz可视化
                )
            ]
        ),
        
        # 4.启动ros2_tools中的辅助节点，处理雷达数据
        *[Node(
            package='ros2_tools',
            executable=node,
            output='screen'
        ) for node in ros2_tools_nodes],
        
        # 5.延迟20秒后启动四旋翼控制节点
        # 确保所有传感器和定位系统都已就绪
        TimerAction(
            period=20.0,  # 延迟20秒
            actions=[
                Node(
                    package='core_mini',              # 我们的简化版控制包
                    executable='quad_mini_node',      # 四旋翼控制节点
                    output='screen',                  # 输出到屏幕
                    emulate_tty=True                  # 模拟终端（保留颜色输出）
                )
            ]
        )
    ])
