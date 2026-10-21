"""Odometrie zonder wielencoders: lidar-odometrie (rf2o) + IMU-gyroscoop in een EKF.

- rf2o leidt de beweging af uit opeenvolgende scans -> /odom_rf2o (zonder TF).
- lidar_odom_fix berekent daaruit de snelheid in het frame van de robot -> /odom_lidar
  (rf2o publiceert de snelheid in het frame van de lidar, en die is 180 graden gedraaid).
- robot_localization (EKF) combineert dat met /imu/data_raw -> /odometry/filtered
  en publiceert de TF odom -> base_footprint.
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('wave_rover_bringup'), 'config', 'odometry.yaml'])

    return LaunchDescription([
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='wave_rover_base',
            executable='lidar_odom_fix',
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[config],
            output='screen',
        ),
    ])
