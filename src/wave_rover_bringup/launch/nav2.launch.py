"""De Nav2-navigatiestack (zonder lokalisatie), gebruikt door navigation.launch.py en
explore.launch.py.

Alle Nav2-nodes (zonder AMCL en map_server) in één proces (`nav2_container`, composition)
om CPU en geheugen te sparen. De kaart komt van slam_toolbox (/map), in lokalisatie- of
kaartmodus. De snelheidscommando's gaan naar /cmd_vel_nav2 (ingang van twist_mux).
"""
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_params = PathJoinSubstitution([FindPackageShare('wave_rover_bringup'), 'config',
                                        'nav2.yaml'])

    return LaunchDescription([
        # Eén proces voor alle Nav2-nodes (i.p.v. ±12): minder geheugen en DDS-overhead.
        Node(
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[nav2_params, {'autostart': True}],
            output='screen',
        ),
        # Bond (hartslag tussen de lifecycle-manager en elke server) uit: 10 servers x 10 Hz
        # op één gedeeld topic dat elke server ontvangt, 200 berichten/s x 20 ontvangers =
        # ~0,5 kern bij stilstand (gemeten 29-10-2026). Alles draait in één proces: crasht een
        # server, dan valt het hele proces weg en ziet de module-manager dat. Via SetParameter,
        # want de lifecycle-manager krijgt nav2.yaml niet mee.
        GroupAction([
            SetParameter('bond_heartbeat_period', 0.0),
            SetParameter('bond_timeout', 0.0),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
                launch_arguments={
                    'params_file': nav2_params,
                    'use_sim_time': 'false',
                    'autostart': 'true',
                    'use_composition': 'True',
                    'container_name': 'nav2_container',
                }.items(),
            ),
        ]),
    ])
