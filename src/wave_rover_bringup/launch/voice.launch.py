"""Yahboom-spraakmodule: spraakcommando's, batterijmeldingen en noodstop."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wave_rover_base',
            executable='yahboom_voice',
            output='screen',
        ),
    ])
