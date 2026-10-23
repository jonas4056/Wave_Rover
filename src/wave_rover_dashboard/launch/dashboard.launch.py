"""Dashboard: rosbridge (WebSocket, poort 9090) en een statische webserver (poort 8080)."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    web = PathJoinSubstitution([FindPackageShare('wave_rover_dashboard'), 'web'])

    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8080', '--directory', web],
            name='dashboard_web',
            output='screen',
        ),
    ])
