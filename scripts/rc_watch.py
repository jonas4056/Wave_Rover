#!/usr/bin/env python3
"""Toont welke RC-kanalen veranderen. In de container uitvoeren:

    docker compose exec ros python3 scripts/rc_watch.py [seconden]
"""
import sys
import time

import rclpy
from sensor_msgs.msg import Joy

THRESHOLD = 0.4


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
    rclpy.init()
    node = rclpy.create_node('rc_watch')
    ref = None
    lo = [9.0] * 16
    hi = [-9.0] * 16

    def on_joy(msg):
        nonlocal ref
        axes = list(msg.axes)
        if ref is None:
            ref = axes
            print('start:', ' '.join(f'{i + 1}:{a:+.2f}' for i, a in enumerate(axes)), flush=True)
            return
        for i, value in enumerate(axes):
            lo[i] = min(lo[i], value)
            hi[i] = max(hi[i], value)
            if abs(value - ref[i]) > THRESHOLD:
                print(f'kanaal {i + 1}: {ref[i]:+.2f} -> {value:+.2f}', flush=True)
                ref[i] = value

    node.create_subscription(Joy, 'joy', on_joy, 10)
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    ranges = [f'{i + 1}:[{lo[i]:+.1f},{hi[i]:+.1f}]' for i in range(16) if hi[i] - lo[i] > 0.3]
    print('bereik:', ' '.join(ranges), flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
