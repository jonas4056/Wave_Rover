#!/usr/bin/env python3
"""Vergelijkt de draaisnelheid volgens IMU, lidar-odometrie en EKF tijdens het draaien.
In de container uitvoeren en ondertussen de robot op de plaats laten draaien:

    docker compose exec ros python3 scripts/turn_check.py [seconden]

Bij een correcte opstelling hebben alle drie hetzelfde teken en ongeveer dezelfde grootte.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = rclpy.create_node('turn_check')
    latest = {'imu': 0.0, 'rf2o': 0.0, 'ekf': 0.0}
    sums = {k: 0.0 for k in latest}
    samples = 0

    node.create_subscription(Imu, 'imu/data_raw',
                             lambda m: latest.__setitem__('imu', m.angular_velocity.z),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, 'odom_rf2o',
                             lambda m: latest.__setitem__('rf2o', m.twist.twist.angular.z), 10)
    node.create_subscription(Odometry, 'odometry/filtered',
                             lambda m: latest.__setitem__('ekf', m.twist.twist.angular.z), 10)

    print('tijd   imu °/s  rf2o °/s  ekf °/s', flush=True)
    start = time.time()
    next_print = start
    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() >= next_print:
            row = {k: math.degrees(v) for k, v in latest.items()}
            for k in sums:
                sums[k] += row[k]
            samples += 1
            print(f'{time.time() - start:4.1f}  {row["imu"]:+8.1f} {row["rf2o"]:+8.1f} {row["ekf"]:+8.1f}',
                  flush=True)
            next_print += 0.5
    if samples:
        print('gem.  ' + ' '.join(f'{sums[k] / samples:+8.1f}' for k in ('imu', 'rf2o', 'ekf')))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
