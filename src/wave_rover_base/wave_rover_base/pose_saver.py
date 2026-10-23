"""Positie van de robot op de kaart: publiceren voor het dashboard en bewaren.

- Publiceert /robot_pose (PoseStamped in `map`, `publish_rate` Hz), zodat het dashboard
  geen TF hoeft te verwerken.
- Schrijft elke `period` seconden `<map>.lastpose` (JSON: x, y, yaw) naast de kaart, als
  de parameter `map` gezet is. navigation.launch.py gebruikt dat bestand als
  startpositie, zodat 2D Pose Estimate alleen nog nodig is als de robot verplaatst werd
  terwijl hij uit stond.
"""
import json
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class PoseSaver(Node):

    def __init__(self):
        super().__init__('pose_saver')
        self.declare_parameter('map', '')
        self.declare_parameter('period', 2.0)
        self.declare_parameter('publish_rate', 5.0)
        map_path = self.get_parameter('map').value
        self.path = map_path + '.lastpose' if map_path else None
        self.last = None
        self.transform = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.publish)
        if self.path:
            self.create_timer(self.get_parameter('period').value, self.save)

    def publish(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return
        self.transform = t
        msg = PoseStamped()
        msg.header = t.header
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.orientation = t.transform.rotation
        self.pub.publish(msg)

    def save(self):
        t = self.transform
        if t is None:
            return
        q = t.transform.rotation
        pose = {
            'x': round(t.transform.translation.x, 3),
            'y': round(t.transform.translation.y, 3),
            'yaw': round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 3),
        }
        if pose == self.last:
            return
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(pose, f)
        os.replace(tmp, self.path)  # atomair: nooit een half geschreven bestand
        self.last = pose


def main(args=None):
    rclpy.init(args=args)
    node = PoseSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
