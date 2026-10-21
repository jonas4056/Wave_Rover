#!/usr/bin/env python3
"""Controleert of vooruitrijden ook als vooruit gemeten wordt. In de container uitvoeren
en ondertussen enkele seconden rechtdoor vooruit rijden met de zender:

    docker compose exec ros python3 scripts/forward_check.py [seconden]
"""
import sys
import time

import rclpy
from nav_msgs.msg import Odometry


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    node = rclpy.create_node('forward_check')
    peaks = {'odom_rf2o': 0.0, 'odom_lidar': 0.0, 'odometry/filtered': 0.0}

    def track(name):
        def callback(msg):
            v = msg.twist.twist.linear.x
            if abs(v) > abs(peaks[name]):
                peaks[name] = v
        return callback

    for topic in peaks:
        node.create_subscription(Odometry, topic, track(topic), 10)
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    print('Grootste snelheid vooruit (positief = vooruit):')
    for topic, v in peaks.items():
        print(f'  {topic:18s} {v:+.2f} m/s')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
