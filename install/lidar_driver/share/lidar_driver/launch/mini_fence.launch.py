import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. RPLidar C1 Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[
                {'serial_port': '/dev/ttyTHS1'},
                # The RPLidar C1 specifically requires a 460800 baudrate
                {'serial_baudrate': 460800}, 
                {'frame_id': 'laser'},
                {'inverted': False},
                {'angle_compensate': True}
            ],
            output='screen'
        ),
        
        # 2. Your Mini Fence Node
        Node(
            package='mini_fence',
            executable='mini_fence_node',
            name='mini_fence_node',
            output='screen',
            remappings=[
                ('/mini_fence/scan', '/mavros/obstacle/send')
            ]
        )
    ])