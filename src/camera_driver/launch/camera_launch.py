from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Create a launch configuration variable
    camera_type_arg = LaunchConfiguration('camera_type')

    # 2. Declare the argument so the user can set it from the terminal
    declare_camera_type_cmd = DeclareLaunchArgument(
        'camera_type',
        default_value='rgb',
        description='Type of camera to launch: "rgb" or "thermal"')

    # 3. Pass that configuration into your node's parameters
    gst_node = Node(
        package='camera_driver',
        executable='gst_node', # The single entry point you defined in setup.py
        name='gst_yolo_node',
        output='screen',
        parameters=[{
            'camera_type': camera_type_arg,
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        declare_camera_type_cmd,
        gst_node
    ])