import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 1a. Declare the user input argument for the camera type
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='rgb', # Defaults to rgb if the user doesn't specify
        description='Type of camera to launch: "rgb" or "thermal"'
    )
    camera_type_config = LaunchConfiguration('camera_type')

    # 1b. Declare the user input argument for the camera usage
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Set to false to run the drone with LiDAR only'
    )
    use_camera_config = LaunchConfiguration('use_camera')
    
    # 1c. Declare the argument to choose the node executable
    camera_node_arg = DeclareLaunchArgument(
        'camera_node',
        default_value='gst_node',
        description='Node to run: "gst_node" or "survey_gst_node"'
    )
    camera_node_exe_config = LaunchConfiguration('camera_node_exe')

    # 2. MAVROS: Connect Orin to Cube Black
    # This mimics running `ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600`
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')
        ),
        launch_arguments={
            'fcu_url': '///dev/ttyACM0:57600',
            'tgt_system': '3'
        }.items()
    )

    # 3a. RPLidar Motor Driver (Hardware)
    sllidar_hardware_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[
            {'serial_port': '/dev/ttyTHS1'},
            {'serial_baudrate': 460800}, 
            {'frame_id': 'laser'},
            {'inverted': False},
            {'angle_compensate': True}
        ],
        output='screen'
    )

    # 3b. Mini Fence Math Node (from your lidar_driver package)
    mini_fence_math_node = Node(
        package='lidar_driver',
        executable='mini_fence_node',
        name='mini_fence_node',
        output='screen',
        remappings=[
            ('/mini_fence/scan', '/mavros/obstacle/send')
        ]
    )

    # 4. Camera Driver (Your GStreamer/YOLO Node)
    camera_node = Node(
        package='camera_driver',
        executable='gst_node', 
        name='gst_yolo_node',
        output='screen',
        # Pass the launch argument into the node as a ROS parameter
        condition=IfCondition(use_camera_config), 
        parameters=[{
            'camera_node': LaunchConfiguration('camera_node'),
            'camera_type': LaunchConfiguration('camera_type'),
        }]
    )

    # 5. Your existing Fusion Node
    fusion_node = Node(
        package='sensor_fusion',
        executable='fusion_visualizer',
        name='fusion_node',
        output='screen',
        condition=IfCondition(use_camera_config)
    )

    return LaunchDescription([
        camera_type_arg,
        use_camera_arg,
        camera_node_arg,
        mavros_launch,
        #sllidar_hardware_node,
        #mini_fence_math_node,
        camera_node,
        fusion_node
    ])
