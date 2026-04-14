from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Create launch configuration variables
    camera_type_arg = LaunchConfiguration('camera_type')
    node_executable_arg = LaunchConfiguration('node_executable')

    # 2. Declare the arguments
    declare_camera_type_cmd = DeclareLaunchArgument(
        'camera_type',
        default_value='rgb',
        description='Type of camera to launch: "rgb" or "thermal"'
    )

    declare_node_executable_cmd = DeclareLaunchArgument(
        'node_executable',
        default_value='gst_node', # Default falls back to your standard node
        description='Choose the node to run: "gst_node" or "survey_gst_node"'
    )

    # 3. Pass that configuration into your node
    gst_node = Node(
        package='camera_driver',
        executable=node_executable_arg,
        name='gst_yolo_node', 
        output='screen',
        parameters=[{
            'camera_type': camera_type_arg,
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        declare_camera_type_cmd,
        declare_node_executable_cmd,
        gst_node
    ])
