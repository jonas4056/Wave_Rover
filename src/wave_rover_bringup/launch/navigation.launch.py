"""Autonoom navigeren met Nav2 op een opgeslagen kaart.

Lokalisatie: slam_toolbox in lokalisatiemodus op de opgeslagen posegraph. Die
vergelijkt scans met de kaart (zoals bij het mappen) en is veel minder afhankelijk
van goede odometrie dan AMCL; dat is belangrijk zonder wielencoders. slam_toolbox
publiceert /map en de TF map -> odom.

Navigatie: de Nav2-navigatiestack (zonder AMCL en map_server). De snelheidscommando's
gaan naar /cmd_vel_nav2 (ingang van twist_mux).

Startpositie: de laatst bekende positie uit `<map>.lastpose` (bijgehouden door pose_saver),
anders het beginpunt van de kaart. Corrigeren kan met 2D Pose Estimate in RViz; die komt
binnen via /initialpose_rviz (zie initialpose_relay).

Argument `map`: pad naar de opgeslagen kaart, zonder extensie (posegraph van slam_toolbox).
"""
import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, \
    OpaqueFunction, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def start_pose_params(map_path):
    """Startpositie voor slam_toolbox: laatst bekende positie, anders het beginpunt."""
    try:
        with open(map_path + '.lastpose') as f:
            pose = json.load(f)
        return {'map_start_at_dock': False,
                'map_start_pose': [float(pose['x']), float(pose['y']), float(pose['yaw'])]}
    except (OSError, ValueError, KeyError):
        return {'map_start_at_dock': True}


def localization_actions(context):
    map_path = LaunchConfiguration('map').perform(context)
    slam_params = os.path.join(FindPackageShare('wave_rover_bringup').perform(context),
                               'config', 'slam_localization.yaml')

    # slam_toolbox is een lifecycle-node: na het starten configureren en activeren.
    localization = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[slam_params, {'map_file_name': map_path, 'use_sim_time': False,
                                  **start_pose_params(map_path)}],
        output='screen',
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(localization),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=localization, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(localization),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    return [localization, configure, activate]


def generate_launch_description():
    share = FindPackageShare('wave_rover_bringup')
    nav2_params = PathJoinSubstitution([share, 'config', 'nav2.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='/home/ubuntu/wave_rover/maps/living_v3',
            description='Opgeslagen kaart (posegraph van slam_toolbox), zonder extensie'),
        OpaqueFunction(function=localization_actions),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),
        Node(
            package='wave_rover_base',
            executable='initialpose_relay',
            output='screen',
        ),
        Node(
            package='wave_rover_base',
            executable='pose_saver',
            parameters=[{'map': LaunchConfiguration('map')}],
            output='screen',
        ),
    ])
