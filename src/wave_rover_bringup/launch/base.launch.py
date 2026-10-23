"""Basis van de Wave Rover: ESP32-bridge, RC-zender, twist_mux, robotmodel, batterijbewaking, module-manager en dashboard."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('wave_rover_bringup'), 'config', 'base.yaml'])
    modules = PathJoinSubstitution(
        [FindPackageShare('wave_rover_bringup'), 'config', 'modules.yaml'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('wave_rover_description'), 'launch', 'description.launch.py'])
        ),
        # Alleen twist_mux mag de motoren aansturen: de bridge luistert naar /cmd_vel_safe.
        # (Sommige Nav2-nodes, zoals de docking_server, publiceren rechtstreeks op /cmd_vel.)
        Node(
            package='wave_rover_base',
            executable='esp32_bridge',
            parameters=[config],
            remappings=[('cmd_vel', 'cmd_vel_safe')],
            output='screen',
        ),
        # Enige eigenaar van /e_stop (lock in twist_mux); service /e_stop/set, vrijgeven met RC kanaal 8.
        Node(
            package='wave_rover_base',
            executable='e_stop',
            output='screen',
        ),
        # Kaarten opslaan/kiezen vanuit het dashboard (doet niets zonder verzoek).
        Node(
            package='wave_rover_base',
            executable='map_manager',
            output='screen',
        ),
        Node(
            package='wave_rover_base',
            executable='battery_monitor',
            output='screen',
        ),
        # Jetson netjes afsluiten als de UPS-batterij onder 10% zakt (aftelling 60 s).
        Node(
            package='wave_rover_base',
            executable='power_guard',
            output='screen',
        ),
        Node(
            package='wave_rover_rc',
            executable='crsf_joy',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='wave_rover_rc',
            executable='rc_teleop',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[config],
            remappings=[('cmd_vel_out', 'cmd_vel_safe')],
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('wave_rover_dashboard'), 'launch', 'dashboard.launch.py'])
        ),
        Node(
            package='wave_rover_manager',
            executable='module_manager',
            parameters=[{'modules_file': modules}],
            output='screen',
        ),
    ])
