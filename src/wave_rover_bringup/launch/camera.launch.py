"""CSI-camera (IMX219) -> /camera/image/compressed via camera_bridge."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wave_rover_base',
            executable='camera_bridge',
            output='screen',
        ),
    ])
