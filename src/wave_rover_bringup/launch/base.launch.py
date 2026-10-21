"""Basis van de Wave Rover: ESP32-bridge, RC-zender, teleop, twist_mux en robotmodel."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('wave_rover_bringup'), 'config', 'base.yaml'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('wave_rover_description'), 'launch', 'description.launch.py'])
        ),
        Node(
            package='wave_rover_base',
            executable='esp32_bridge',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='wave_rover_rc',
            executable='crsf_joy',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config],
            remappings=[('cmd_vel', 'cmd_vel_rc')],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[config],
            remappings=[('cmd_vel_out', 'cmd_vel')],
        ),
    ])
