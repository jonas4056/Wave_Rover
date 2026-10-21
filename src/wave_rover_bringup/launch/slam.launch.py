"""Kaart maken met slam_toolbox (online, asynchroon) -> /map en TF map -> odom."""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('wave_rover_bringup'), 'config', 'slam.yaml'])

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[config],
            output='screen',
        ),
    ])
