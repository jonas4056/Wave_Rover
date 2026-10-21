"""Autonoom navigeren met Nav2 op een opgeslagen kaart.

Start map_server + AMCL (lokalisatie op de kaart) en de Nav2-navigatiestack.
De snelheidscommando's gaan naar /cmd_vel_nav2 (ingang van twist_mux).
De startpositie uit RViz komt binnen via /initialpose_rviz (zie initialpose_relay).

Argument `map`: pad naar het .yaml-bestand van de kaart.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('wave_rover_bringup'), 'config', 'nav2.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='/home/ubuntu/wave_rover/maps/living_v2.yaml',
            description='Kaart (.yaml) om op te navigeren'),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': params,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),
        Node(
            package='wave_rover_base',
            executable='initialpose_relay',
            output='screen',
        ),
    ])
