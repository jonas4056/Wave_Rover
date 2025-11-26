from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crsf_receiver',
            executable='crsf_receiver_node',
            name='crsf_receiver_node',
            output='screen',
            parameters=[{'baud_rate': 420000, 'link_stats': True}]
        ),
        Node(
            package='crsf_to_joy_bridge',
            executable='crsf_to_joy',
            name='crsf_to_joy_node',
            output='screen'
        ),
        Node(
            package='simple_joy_to_cmd',
            executable='simple_joy_to_cmd',
            name='simple_joy_to_cmd_node',
            output='screen'
        )
    ])
