"""Objectherkenning op de GPU -> /detections (vereist de module camera)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wave_rover_vision',
            executable='object_detector',
            output='screen',
        ),
    ])
