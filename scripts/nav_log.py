#!/usr/bin/env python3
"""Logt tijdens het navigeren de correcties van AMCL en de draaisnelheden naar een CSV.
In de container uitvoeren (stopt na [seconden], standaard 1800):

    docker compose exec ros python3 scripts/nav_log.py [seconden] [bestand]

Regels (kolom 'bron'):
  amcl : map -> odom van AMCL (x, y, yaw); een sprong hierin is een sprong van de robot op de kaart
  ekf  : odom -> base_footprint (x, y, yaw) en de draaisnelheid van de EKF
  rf2o : draaisnelheid uit de lidar (/odom_lidar)
  gyro : draaisnelheid van de gyroscoop (/imu/data_raw)
  cmd  : gevraagde snelheid (/cmd_vel_safe)
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 1800.0
    path = sys.argv[2] if len(sys.argv) > 2 else '/home/ubuntu/wave_rover/run/nav_log.csv'
    rclpy.init()
    node = rclpy.create_node('nav_log')
    out = open(path, 'w', buffering=1)
    out.write('t,bron,x,y,yaw_deg,wz_deg_s,vx\n')
    t0 = time.monotonic()

    def row(src, x='', y='', yaw='', wz='', vx=''):
        fmt = lambda v: '' if v == '' else f'{v:.4f}'  # noqa: E731
        out.write(f'{time.monotonic() - t0:.3f},{src},{fmt(x)},{fmt(y)},{fmt(yaw)},{fmt(wz)},{fmt(vx)}\n')

    def on_tf(msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                tr = t.transform.translation
                row('amcl', tr.x, tr.y, math.degrees(yaw_of(t.transform.rotation)))

    def on_ekf(msg):
        p = msg.pose.pose
        row('ekf', p.position.x, p.position.y, math.degrees(yaw_of(p.orientation)),
            math.degrees(msg.twist.twist.angular.z), msg.twist.twist.linear.x)

    node.create_subscription(TFMessage, '/tf', on_tf, 50)
    node.create_subscription(Odometry, '/odometry/filtered', on_ekf, 10)
    node.create_subscription(
        Odometry, '/odom_lidar',
        lambda m: row('rf2o', wz=math.degrees(m.twist.twist.angular.z), vx=m.twist.twist.linear.x), 10)
    node.create_subscription(
        Imu, '/imu/data_raw', lambda m: row('gyro', wz=math.degrees(m.angular_velocity.z)),
        qos_profile_sensor_data)
    node.create_subscription(
        TwistStamped, '/cmd_vel_safe',
        lambda m: row('cmd', wz=math.degrees(m.twist.angular.z), vx=m.twist.linear.x), 10)

    while rclpy.ok() and time.monotonic() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.5)
    out.close()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
