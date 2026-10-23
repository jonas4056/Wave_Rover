"""Kaart maken met slam_toolbox (online, asynchroon) -> /map en TF map -> odom.

slam_toolbox is in Jazzy een lifecycle-node; het officiële launchbestand configureert
en activeert hem automatisch.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('wave_rover_bringup'), 'config', 'slam.yaml'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']),
            launch_arguments={
                'slam_params_file': config,
                'use_sim_time': 'false',
            }.items(),
        ),
        # /robot_pose voor het dashboard (zonder kaartnaam: niets bewaren).
        Node(
            package='wave_rover_base',
            executable='pose_saver',
            output='screen',
        ),
    ])
