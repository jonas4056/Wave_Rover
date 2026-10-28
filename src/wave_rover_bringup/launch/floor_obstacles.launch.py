"""Lage obstakels op de vloer met de camera -> /floor_scan (vereist de module camera)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('wave_rover_bringup'), 'config',
                          'vision.yaml')
    return LaunchDescription([
        Node(
            package='wave_rover_vision',
            executable='floor_obstacles',
            parameters=[config],
            output='screen',
        ),
    ])
