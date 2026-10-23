#!/usr/bin/env python3
"""Meet hoe de robot werkelijk draait en rijdt tegenover wat gevraagd wordt.

In de container uitvoeren, met de odometriemodule aan, de robot op de grond met
ongeveer 1 m vrije ruimte rondom en de arm-schakelaar van de zender UIT:

    docker compose exec ros python3 scripts/calibrate_motion.py

Stuurt via /cmd_vel_nav2 (laagste prioriteit in twist_mux: de zender kan altijd ingrijpen).
Draaisnelheid gemeten met de gyroscoop (/imu/data_raw), rijsnelheid met /odometry/filtered.
Per test wordt het gemiddelde genomen na de eerste seconde (optrekken).
"""
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

SETTLE = 1.0


class Calibrator:

    def __init__(self):
        self.node = rclpy.create_node('calibrate_motion')
        self.pub = self.node.create_publisher(TwistStamped, 'cmd_vel_nav2', 10)
        self.gyro_z = 0.0
        self.vx = 0.0
        self.node.create_subscription(Imu, 'imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, 'odometry/filtered', self.on_odom, 10)

    def on_imu(self, msg):
        self.gyro_z = msg.angular_velocity.z

    def on_odom(self, msg):
        self.vx = msg.twist.twist.linear.x

    def drive(self, vx, wz, duration):
        """Rijdt `duration` s met (vx, wz); geeft het gemiddelde (vx, wz) na het optrekken."""
        samples = []
        start = time.time()
        while time.time() - start < duration:
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.twist.linear.x = vx
            msg.twist.angular.z = wz
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if time.time() - start > SETTLE:
                samples.append((self.vx, self.gyro_z))
        self.stop(1.0)
        n = max(1, len(samples))
        return sum(s[0] for s in samples) / n, sum(s[1] for s in samples) / n

    def stop(self, duration):
        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(self.node, timeout_sec=0.05)


def main():
    rclpy.init()
    cal = Calibrator()
    cal.stop(1.0)
    rows = []

    print('Draaien op de plaats', flush=True)
    for wz in (0.5, 1.0, 1.5, 2.0):
        for sign in (1, -1):
            _, actual = cal.drive(0.0, sign * wz, 3.0)
            rows.append(('draaien', 0.0, sign * wz, 0.0, actual))
            print(f'  gevraagd {sign * wz:+.1f} rad/s -> echt {actual:+.2f} rad/s '
                  f'({actual / (sign * wz) * 100:.0f}%)', flush=True)

    print('Bocht (vooruit + draaien)', flush=True)
    for vx, wz in ((0.2, 0.8), (0.2, -0.8)):
        avx, awz = cal.drive(vx, wz, 3.0)
        cal.drive(-vx, -wz, 3.0)  # terug naar ongeveer het startpunt
        rows.append(('bocht', vx, wz, avx, awz))
        print(f'  gevraagd {vx:.2f} m/s, {wz:+.1f} rad/s -> echt {avx:.2f} m/s, {awz:+.2f} rad/s '
              f'({awz / wz * 100:.0f}% draaien)', flush=True)

    print('Rechtdoor', flush=True)
    for vx in (0.15, 0.3):
        avx, _ = cal.drive(vx, 0.0, 2.0)
        bvx, _ = cal.drive(-vx, 0.0, 2.0)
        rows.append(('rechtdoor', vx, 0.0, avx, 0.0))
        print(f'  gevraagd {vx:.2f} m/s -> vooruit {avx:.2f}, achteruit {-bvx:.2f} m/s '
              f'({(avx - bvx) / 2 / vx * 100:.0f}%)', flush=True)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
