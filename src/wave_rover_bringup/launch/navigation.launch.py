"""Autonoom navigeren met Nav2 op een opgeslagen kaart.

Lokalisatie (argument `localization`), publiceert /map en de TF map -> odom:
- `amcl` (standaard): deeltjesfilter dat elke scan met de opgeslagen kaart (.pgm/.yaml)
  vergelijkt, via map_server + amcl in het Nav2-proces. Een fout bij het draaien stapelt
  zich zo niet op.
  Met het omni-bewegingsmodel en rechtgezette scans (scan_deskew): na 2 rondes ter plaatse
  draaien +8/-13 graden (met het differentiële model sprong hij tot 158 graden ernaast).
- `slam`: slam_toolbox in lokalisatiemodus op de posegraph (+1/-10 graden in dezelfde proef). Was de keuze toen de
  odometrie nog slecht was (gyro scheef, geen draaibijsturing), maar vergelijkt vooral met
  zijn eigen recente scans: na ter plaatse draaien stond de richting 10-20 graden scheef en
  dat bleef zo (gemeten 29-10-2026).

Navigatie: de Nav2-navigatiestack uit nav2.launch.py (één proces, composition). De
snelheidscommando's gaan naar /cmd_vel_nav2 (ingang van twist_mux).

Startpositie: de laatst bekende positie uit `<map>.lastpose` (bijgehouden door pose_saver),
anders het beginpunt van de kaart. Corrigeren kan met "Zoek mezelf" in de app (map_manager)
of 2D Pose Estimate in RViz (via /initialpose_rviz, zie initialpose_relay).

Argument `map`: pad naar de opgeslagen kaart, zonder extensie (posegraph van slam_toolbox).
Standaard de kaart uit `maps/current` (gekozen in het dashboard via map_manager).
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, \
    IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def last_pose(map_path):
    """Laatst bekende positie (x, y, yaw) uit <map>.lastpose, of None."""
    try:
        with open(map_path + '.lastpose') as f:
            pose = json.load(f)
        return float(pose['x']), float(pose['y']), float(pose['yaw'])
    except (OSError, ValueError, KeyError):
        return None


def start_pose_params(map_path):
    """Startpositie voor slam_toolbox: laatst bekende positie, anders het beginpunt."""
    pose = last_pose(map_path)
    if pose is None:
        return {'map_start_at_dock': True}
    return {'map_start_at_dock': False, 'map_start_pose': list(pose)}


def localization_actions(context):
    map_path = LaunchConfiguration('map').perform(context)
    share = FindPackageShare('wave_rover_bringup').perform(context)
    if LaunchConfiguration('localization').perform(context) == 'amcl':
        return amcl_actions(map_path, share)
    slam_params = os.path.join(share, 'config', 'slam_localization.yaml')

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


def amcl_actions(map_path, share):
    """map_server + amcl in het Nav2-proces, startpositie uit <map>.lastpose."""
    x, y, yaw = last_pose(map_path) or (0.0, 0.0, 0.0)
    return [GroupAction([
        # Zelfde redenen als in nav2.launch.py (bond); startpositie via SetParameter omdat
        # de lokalisatie-launch van Nav2 nav2.yaml zonder aanpassingen inleest.
        SetParameter('bond_heartbeat_period', 0.0),
        SetParameter('bond_timeout', 0.0),
        SetParameter('set_initial_pose', True),
        SetParameter('initial_pose.x', x),
        SetParameter('initial_pose.y', y),
        SetParameter('initial_pose.z', 0.0),
        SetParameter('initial_pose.yaw', yaw),
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                         'localization_launch.py'),
            launch_arguments={
                'map': map_path + '.yaml',
                'params_file': os.path.join(share, 'config', 'nav2.yaml'),
                'use_sim_time': 'false',
                'autostart': 'true',
                'use_composition': 'True',
                'container_name': 'nav2_container',
            }.items(),
        ),
    ])]


MAPS_DIR = '/home/ubuntu/wave_rover/maps'
FALLBACK_MAP = 'living_v3'


def default_map():
    """De kaart uit maps/current, anders FALLBACK_MAP."""
    try:
        with open(os.path.join(MAPS_DIR, 'current')) as f:
            name = f.read().strip()
    except OSError:
        name = ''
    return os.path.join(MAPS_DIR, name or FALLBACK_MAP)


def generate_launch_description():
    share = FindPackageShare('wave_rover_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map(),
            description='Opgeslagen kaart (posegraph van slam_toolbox), zonder extensie'),
        DeclareLaunchArgument(
            'localization', default_value='amcl', choices=['amcl', 'slam'],
            description='amcl (vergelijkt met de opgeslagen kaart) of slam (slam_toolbox)'),
        OpaqueFunction(function=localization_actions),
        IncludeLaunchDescription(PathJoinSubstitution([share, 'launch', 'nav2.launch.py'])),
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
