#!/usr/bin/env python3
"""Volgt een draaicommando door de hele keten tot aan de motoren. In de container uitvoeren
en ondertussen een navigatiedoel geven:

    docker compose exec ros python3 scripts/cmd_trace.py [seconden]

Kolommen (rad/s): controller (cmd_vel_nav), smoother (cmd_vel_smoothed),
collision monitor (cmd_vel_nav2), twist_mux -> bridge (cmd_vel_safe), en de
werkelijke draaisnelheid volgens de IMU.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

STAGES = ['cmd_vel_nav', 'cmd_vel_smoothed', 'cmd_vel_nav2', 'cmd_vel_safe']


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = rclpy.create_node('cmd_trace')
    latest = {name: (0.0, 0.0) for name in STAGES}
    latest['imu'] = (0.0, 0.0)

    def on_cmd(name):
        def callback(msg):
            latest[name] = (msg.twist.linear.x, msg.twist.angular.z)
        return callback

    for name in STAGES:
        node.create_subscription(TwistStamped, name, on_cmd(name), 10)
    node.create_subscription(Imu, 'imu/data_raw',
                             lambda m: latest.__setitem__('imu', (0.0, m.angular_velocity.z)),
                             qos_profile_sensor_data)

    print('tijd  controller   smoother  collision  ->motoren   imu (wz rad/s, vx m/s)', flush=True)
    start = time.time()
    next_print = start
    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() >= next_print:
            cols = ' '.join(f'{latest[n][1]:+5.2f}/{latest[n][0]:+4.2f}' for n in STAGES)
            print(f'{time.time() - start:4.1f}  {cols}  {latest["imu"][1]:+5.2f}', flush=True)
            for n in STAGES:
                latest[n] = (0.0, 0.0)
            next_print += 0.5
    rclpy.shutdown()


if __name__ == '__main__':
    main()
