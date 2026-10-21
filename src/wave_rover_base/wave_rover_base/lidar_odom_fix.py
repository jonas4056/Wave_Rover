"""Corrigeert de snelheid van rf2o_laser_odometry voor een gedraaid gemonteerde lidar.

rf2o rekent de positie correct om naar base_footprint, maar publiceert de snelheid
in het frame van de lidar (lin_speed = verplaatsing langs de x-as van de lidar).
Bij onze lidar, 180 graden gedraaid, wordt vooruit daardoor achteruit. rf2o geeft
bovendien geen covariantie mee.

Deze node leidt de snelheid af uit twee opeenvolgende (correcte) posities van rf2o,
uitgedrukt in het frame van de robot, en publiceert die met een covariantie op
/odom_lidar. De EKF gebruikt dat topic.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class LidarOdomFix(Node):

    def __init__(self):
        super().__init__('lidar_odom_fix')
        # Standaardafwijking van de snelheden (m/s en rad/s).
        self.declare_parameter('linear_std', 0.05)
        self.declare_parameter('angular_std', 0.05)
        lin_var = self.get_parameter('linear_std').value ** 2
        ang_var = self.get_parameter('angular_std').value ** 2
        self.twist_cov = [0.0] * 36
        self.twist_cov[0] = lin_var
        self.twist_cov[7] = lin_var
        self.twist_cov[35] = ang_var

        self.prev = None
        self.pub = self.create_publisher(Odometry, 'odom_lidar', 10)
        self.create_subscription(Odometry, 'odom_rf2o', self.on_odom, 10)

    def on_odom(self, msg):
        p = msg.pose.pose
        stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        current = (stamp, p.position.x, p.position.y, yaw_of(p.orientation))
        if self.prev is None:
            self.prev = current
            return

        t0, x0, y0, a0 = self.prev
        t1, x1, y1, a1 = current
        dt = t1 - t0
        if dt <= 0.0:
            return
        self.prev = current

        # Verplaatsing in het odom-frame -> frame van de robot op het vorige tijdstip.
        dx, dy = x1 - x0, y1 - y0
        cos_a, sin_a = math.cos(a0), math.sin(a0)
        forward = cos_a * dx + sin_a * dy
        sideways = -sin_a * dx + cos_a * dy
        turn = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist.twist.linear.x = forward / dt
        out.twist.twist.linear.y = sideways / dt
        out.twist.twist.angular.z = turn / dt
        out.twist.covariance = self.twist_cov
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarOdomFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
