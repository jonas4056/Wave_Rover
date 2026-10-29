"""Zelf verkennen (module explore): een nieuwe ruimte in kaart brengen zonder te sturen.

- slam_toolbox maakt de kaart (slam.launch.py, met pose_saver voor /robot_pose);
- Nav2 rijdt op die groeiende kaart (nav2.launch.py; plannen door onbekende ruimte mag);
- explorer kiest telkens een rand tussen bekend en onbekend, en slaat de kaart op het einde
  op. Starten met de knop "Verken" in het dashboard (/explore/start met de kaartnaam).
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('wave_rover_bringup'), 'launch'])
    return LaunchDescription([
        IncludeLaunchDescription(PathJoinSubstitution([launch_dir, 'slam.launch.py'])),
        IncludeLaunchDescription(PathJoinSubstitution([launch_dir, 'nav2.launch.py'])),
        Node(
            package='wave_rover_base',
            executable='explorer',
            output='screen',
        ),
    ])
