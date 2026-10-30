#!/usr/bin/env python3
"""Hoe goed liggen de lidarpunten op de muren van de kaart? In de container uitvoeren:

    docker compose exec ros python3 scripts/map_fit.py

Neemt één scan en de huidige TF map -> lidar, en zoekt de draaiing/verschuiving waarbij de
scan het best op de bezette cellen van /map valt. Toont hoeveel procent van de punten nu op
een muur valt en hoeveel graden/cm de lokalisatie ernaast zit (± 20°, ± 30 cm).
"""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = rclpy.create_node('map_fit')
    buf = Buffer()
    TransformListener(buf, node)
    got = {}
    node.create_subscription(OccupancyGrid, '/map', lambda m: got.setdefault('map', m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(LaserScan, '/scan', lambda m: got.__setitem__('scan', m),
                             qos_profile_sensor_data)
    start = time.time()
    tf = None
    while time.time() - start < 20 and tf is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if 'map' in got and 'scan' in got:
            try:
                tf = buf.lookup_transform('map', got['scan'].header.frame_id, rclpy.time.Time())
            except Exception:  # noqa: BLE001 - TF nog niet binnen
                pass
    if tf is None:
        print('Geen kaart, scan of TF (staat navigation aan?)')
        return
    m, s = got['map'], got['scan']
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    lx, ly = tf.transform.translation.x, tf.transform.translation.y

    r = np.array(s.ranges)
    a = s.angle_min + np.arange(len(r)) * s.angle_increment
    ok = np.isfinite(r) & (r > s.range_min) & (r < s.range_max)
    r, a = r[ok], a[ok]

    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    occ = np.array(m.data, np.int16).reshape(m.info.height, m.info.width) >= 65
    occ = occ | np.roll(occ, 1, 0) | np.roll(occ, -1, 0) | np.roll(occ, 1, 1) | np.roll(occ, -1, 1)

    def score(dyaw, dx, dy):
        x = lx + dx + r * np.cos(a + yaw + dyaw)
        y = ly + dy + r * np.sin(a + yaw + dyaw)
        c = ((x - ox) / res).astype(int)
        rr = ((y - oy) / res).astype(int)
        inside = (c >= 0) & (c < occ.shape[1]) & (rr >= 0) & (rr < occ.shape[0])
        return occ[rr[inside], c[inside]].sum() / len(r)

    now = score(0, 0, 0)
    best = max((score(math.radians(d), dx, dy), d, dx, dy)
               for d in np.arange(-20, 20.1, 0.5)
               for dx in np.arange(-0.3, 0.31, 0.05)
               for dy in np.arange(-0.3, 0.31, 0.05))
    print(f'Op de muren nu: {now:.0%}; best haalbaar {best[0]:.0%} bij {best[1]:+.1f}°, '
          f'({best[2] * 100:+.0f}, {best[3] * 100:+.0f}) cm')
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
