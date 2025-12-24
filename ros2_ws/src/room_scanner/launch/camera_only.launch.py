"""
Launch file for RealSense camera only (for testing)
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Camera parameters
    camera_params_file = PathJoinSubstitution([
        FindPackageShare('room_scanner'),
        'config',
        'camera_params.yaml'
    ])

    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        parameters=[camera_params_file],
        output='screen',
        emulate_tty=True,
    )

    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(realsense_node)

    return ld
