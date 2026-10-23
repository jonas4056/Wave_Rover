"""RPLidar C1 op /dev/lidar -> /scan (frame "laser"), via de Slamtec-driver sllidar_ros2."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/lidar',
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'scan_mode': 'Standard',
                'angle_compensate': True,
            }],
            output='screen',
            # De C1 geeft soms een time-out (80008004) bij het verbinden vlak na een stop:
            # dan gewoon opnieuw proberen.
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
