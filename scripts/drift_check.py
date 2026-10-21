#!/usr/bin/env python3
"""Meet de drift van de odometrie terwijl de robot stilstaat. In de container uitvoeren:

    docker compose exec ros python3 scripts/drift_check.py [seconden]

Vergelijkt de lidar-odometrie (/odom_rf2o), het EKF-resultaat (/odometry/filtered)
en de gyroscoop van de IMU (/imu/data_raw). Bij stilstand horen alle drie ~0 te blijven.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = rclpy.create_node('drift_check')
    first, last = {}, {}
    gyro_z = []

    def on_odom(name):
        def callback(msg):
            p = msg.pose.pose
            sample = (p.position.x, p.position.y, yaw_of(p.orientation))
            first.setdefault(name, sample)
            last[name] = sample
        return callback

    node.create_subscription(Odometry, 'odom_rf2o', on_odom('rf2o'), 10)
    node.create_subscription(Odometry, 'odometry/filtered', on_odom('ekf'), 10)
    node.create_subscription(Imu, 'imu/data_raw',
                             lambda m: gyro_z.append(math.degrees(m.angular_velocity.z)),
                             qos_profile_sensor_data)

    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'Drift over {duration:.0f} s bij stilstand:')
    for name in ('rf2o', 'ekf'):
        if name not in first:
            print(f'  {name:5s}: geen data')
            continue
        (x0, y0, a0), (x1, y1, a1) = first[name], last[name]
        dist = math.hypot(x1 - x0, y1 - y0) * 100
        turn = (a1 - a0 + 180) % 360 - 180
        print(f'  {name:5s}: {dist:6.1f} cm verschoven, {turn:+7.1f}° gedraaid')
    if gyro_z:
        mean = sum(gyro_z) / len(gyro_z)
        print(f'  gyro : gemiddeld {mean:+.3f} °/s ({len(gyro_z)} metingen) '
              f'-> {mean * duration:+.1f}° over {duration:.0f} s')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
