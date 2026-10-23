"""Bewaart de positie van de robot op de kaart, zodat de lokalisatie daar kan herstarten.

Leest elke `period` seconden de TF map -> base_footprint en schrijft die naar
`<map>.lastpose` (JSON: x, y, yaw) naast de kaart. navigation.launch.py gebruikt dat
bestand als startpositie, zodat 2D Pose Estimate alleen nog nodig is als de robot
verplaatst werd terwijl hij uit stond.
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class PoseSaver(Node):

    def __init__(self):
        super().__init__('pose_saver')
        self.declare_parameter('map', '')
        self.declare_parameter('period', 2.0)
        self.path = self.get_parameter('map').value + '.lastpose'
        self.last = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(self.get_parameter('period').value, self.save)

    def save(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
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
