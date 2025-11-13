from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('esp32_serial_bridge')
    urdf_file = os.path.join(pkg_share, 'urdf', 'wave_rover.urdf')

    serial_port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port to communicate with ESP32'
    )

    esp32_bridge_node = Node(
        package='esp32_serial_bridge',
        executable='esp32_serial_bridge_node',
        name='wave_rover_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': 115200
        }]
    )

    # Optional: robot_state_publisher for visualization in RViz
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': False}],
        arguments=[urdf_file]
    )

    # Launch RViz2 to visualize topics
    rviz_config = os.path.join(pkg_share, 'rviz', 'wave_rover.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        serial_port_arg,
        esp32_bridge_node,
        robot_state_pub,
        rviz2
    ])

