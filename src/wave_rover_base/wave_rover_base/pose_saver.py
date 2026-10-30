"""Positie van de robot op de kaart: publiceren voor het dashboard en bewaren.

- Publiceert /robot_pose (PoseStamped in `map`, `publish_rate` Hz), zodat het dashboard
  geen TF hoeft te verwerken.
- Schrijft elke `period` seconden `<map>.lastpose` (JSON: x, y, yaw) naast de kaart, als
  de parameter `map` gezet is. navigation.launch.py gebruikt dat bestand als
  startpositie, zodat 2D Pose Estimate alleen nog nodig is als de robot verplaatst werd
  terwijl hij uit stond.
- Zet de obstakels rond de robot om naar `map` voor de kaart in het dashboard
  (`obstacle_rate` Hz), als std_msgs/Int16MultiArray [x1, y1, x2, y2, ...] in cm:
  /scan (lidar, live) -> /dashboard/lidar_points, /floor_points (camera, met geheugen) ->
  /dashboard/floor_points. Hier en niet in een aparte node: TF ontvangen (40 berichten/s)
  kost in Python ~8% van een kern, en deze node doet dat toch al.
"""
import json
import math
import os

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int16MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener


class PoseSaver(Node):

    def __init__(self):
        super().__init__('pose_saver')
        self.declare_parameter('map', '')
        self.declare_parameter('period', 2.0)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('obstacle_rate', 2.0)
        self.declare_parameter('lidar_max_range', 4.0)
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
            # map_manager wisselde van kaart (robot stond in een andere vaste kaart).
            self.create_subscription(String, 'maps/active', self.on_active_map, 10)

        self.lidar_max_range = self.get_parameter('lidar_max_range').value
        self.scan = None
        self.floor = None
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'floor_points', self.on_floor, 10)
        self.lidar_pub = self.create_publisher(Int16MultiArray, 'dashboard/lidar_points', 1)
        self.floor_pub = self.create_publisher(Int16MultiArray, 'dashboard/floor_points', 1)
        self.create_timer(1.0 / self.get_parameter('obstacle_rate').value,
                          self.publish_obstacles)

    def on_active_map(self, msg):
        self.path = msg.data + '.lastpose'
        self.last = None
        self.get_logger().info(f'Laatste positie voortaan in {self.path}')

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

    def on_scan(self, msg):
        self.scan = msg

    def on_floor(self, msg):
        self.floor = msg

    def to_map_cm(self, header, xy):
        """Punten (n x 2, m) uit het frame van `header` naar `map`, in cm, één per 5 cm.

        Met de positie op het moment van de meting: met de laatste positie stonden de punten
        tijdens het draaien scheef (69 graden/s x ~0,2 s vertraging).
        """
        try:
            tf = self.tf_buffer.lookup_transform('map', header.frame_id, Time.from_msg(header.stamp))
        except TransformException:
            try:  # nog niet in de buffer of te oud: dan toch de laatste positie
                tf = self.tf_buffer.lookup_transform('map', header.frame_id, Time())
            except TransformException:
                return None
        tf = tf.transform
        q = tf.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        x = tf.translation.x + c * xy[:, 0] - s * xy[:, 1]
        y = tf.translation.y + s * xy[:, 0] + c * xy[:, 1]
        cells = np.unique(np.round(np.stack([x, y], 1) / 0.05).astype(np.int16), axis=0)
        return Int16MultiArray(data=(cells * 5).ravel().tolist())

    def publish_obstacles(self):
        if self.scan is not None:
            s, self.scan = self.scan, None
            r = np.asarray(s.ranges, np.float32)
            a = s.angle_min + np.arange(len(r)) * s.angle_increment
            ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, self.lidar_max_range))
            msg = self.to_map_cm(s.header,
                                 np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1))
            if msg is not None:
                self.lidar_pub.publish(msg)
        if self.floor is not None:
            f, self.floor = self.floor, None  # module gestopt: niets meer sturen
            pts = point_cloud2.read_points_numpy(f, field_names=('x', 'y'), skip_nans=True)
            msg = self.to_map_cm(f.header, pts.reshape(-1, 2))
            if msg is not None:
                self.floor_pub.publish(msg)

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
