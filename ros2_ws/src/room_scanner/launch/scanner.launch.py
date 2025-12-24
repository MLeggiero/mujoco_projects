"""
Main launch file for 3D Room Scanner
Starts RealSense camera, RTABMap, and RViz2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    use_rtabmap = LaunchConfiguration('use_rtabmap')
    use_octomap = LaunchConfiguration('use_octomap')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2')

    declare_use_rtabmap_cmd = DeclareLaunchArgument(
        'use_rtabmap',
        default_value='true',
        description='Whether to use RTABMap for mapping (recommended)')

    declare_use_octomap_cmd = DeclareLaunchArgument(
        'use_octomap',
        default_value='false',
        description='Whether to use Octomap (alternative to RTABMap)')

    # RealSense camera node
    camera_params_file = PathJoinSubstitution([
        FindPackageShare('room_scanner'),
        'config',
        'camera_params.yaml'
    ])

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        parameters=[camera_params_file],
        output='screen',
        emulate_tty=True,
    )

    # RTABMap node for SLAM and mapping
    rtabmap_params_file = PathJoinSubstitution([
        FindPackageShare('room_scanner'),
        'config',
        'rtabmap_params.yaml'
    ])

    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params_file],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
        ],
        arguments=['-d'],  # Delete previous database on start
        condition=IfCondition(use_rtabmap)
    )

    # RGBD Odometry for RTABMap
    rgbd_odometry_node = Node(
        package='rtabmap_ros',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[rtabmap_params_file],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
        ],
        condition=IfCondition(use_rtabmap)
    )

    # Octomap server (alternative to RTABMap)
    octomap_params_file = PathJoinSubstitution([
        FindPackageShare('room_scanner'),
        'config',
        'octomap_params.yaml'
    ])

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[octomap_params_file],
        remappings=[
            ('cloud_in', '/camera/depth/color/points'),
        ],
        condition=IfCondition(use_octomap)
    )

    # RViz2 for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('room_scanner'),
        'rviz',
        'scanner_config.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_rtabmap_cmd)
    ld.add_action(declare_use_octomap_cmd)

    # Add nodes
    ld.add_action(realsense_node)
    ld.add_action(rgbd_odometry_node)
    ld.add_action(rtabmap_node)
    ld.add_action(octomap_node)
    ld.add_action(rviz_node)

    return ld
