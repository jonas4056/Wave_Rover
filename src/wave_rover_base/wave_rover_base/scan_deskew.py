"""Lidarscans rechtzetten voor het draaien van de robot tijdens één omwenteling ("deskew").

/scan_raw (driver) -> /scan. De RPLidar C1 doet ~97 ms over een omwenteling; draait de robot
ondertussen (70 graden/s), dan is de scan 7 graden "uitgesmeerd". Bovendien stempelt de driver
het begin van de omwenteling. slam_toolbox en AMCL corrigeren geen van beide en raakten bij ter
plaatse draaien de richting kwijt (29-10-2026).

- Tijdstempel = het midden van de omwenteling. Gemeten tijdens een draai: met het stempel van de
  driver stond de scan 4 graden naast de kaart (teken wisselt met de draairichting), met het
  midden 0.
- Elk punt wordt met de gyro (/imu/data_raw) teruggedraaid naar dat midden: de scan past beter
  op de kaart (0,578 -> 0,631). De punten zitten in de tijd in omgekeerde volgorde van de index
  (`time_order` -1, gemeten; +1 maakte het slechter).
Alleen draaien wordt gecorrigeerd, verschuiven niet (1-2 cm bij 0,2 m/s). Zonder verse gyro of
bij stilstand gaat de scan ongewijzigd door, wel met het tijdstempel op het midden.
"""
from collections import deque

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

MIN_RATE = 0.05  # rad/s; trager draaien geeft minder dan een halve stap uitsmeren


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def to_stamp(t):
    sec = int(t)
    return TimeMsg(sec=sec, nanosec=int((t - sec) * 1e9))


class ScanDeskew(Node):

    def __init__(self):
        super().__init__('scan_deskew')
        self.declare_parameter('time_order', -1)
        self.time_order = self.get_parameter('time_order').value
        self.gyro = deque(maxlen=40)  # (tijd, rad/s), ~2 s bij 20 Hz
        self.pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)

    def on_imu(self, msg):
        self.gyro.append((seconds(msg.header.stamp), msg.angular_velocity.z))

    def rate(self, t0, t1):
        """Gemiddelde draaisnelheid tijdens de omwenteling, of None zonder verse gyro."""
        samples = [w for t, w in self.gyro if t0 - 0.05 <= t <= t1 + 0.05]
        return sum(samples) / len(samples) if samples else None

    def on_scan(self, msg):
        n = len(msg.ranges)
        start = seconds(msg.header.stamp)
        duration = msg.time_increment * (n - 1) if msg.time_increment > 0 else msg.scan_time
        msg.header.stamp = to_stamp(start + duration / 2)
        wz = self.rate(start, start + duration)
        if wz is not None and abs(wz) > MIN_RATE and n > 1:
            ranges = np.asarray(msg.ranges, np.float32)
            ranges[~np.isfinite(ranges)] = np.inf
            # Hoek die de robot na het midden nog draaide, per punt; terug naar het midden.
            turned = wz * duration * np.linspace(-0.5, 0.5, n) * self.time_order
            idx = np.round(np.arange(n) + turned / msg.angle_increment).astype(int) % n
            out = np.full(n, np.inf, np.float32)
            np.minimum.at(out, idx, ranges)
            msg.ranges = out.tolist()
            msg.intensities = []  # horen niet meer bij de verschoven punten
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanDeskew()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
