"""Kaarten opslaan, oplijsten en kiezen (voor het dashboard).

Topics (gewone berichttypes, zodat rosbridge geen eigen interfaces nodig heeft):
- /maps/save (std_msgs/String): naam -> de lopende SLAM-kaart opslaan als
  `<maps_dir>/<naam>` (.posegraph/.data voor slam_toolbox, .pgm/.yaml als afbeelding)
  plus `<naam>.lastpose` met de huidige positie, zodat de navigatie daar meteen start.
- /maps/select (std_msgs/String): naam -> wordt de kaart voor de navigatie
  (`<maps_dir>/current`; navigation.launch.py leest dit bij het starten).
- /maps/list (std_msgs/String, JSON, latched): {"maps": [...], "current": "...", "message": "..."}
- /localization/find (std_msgs/Empty): zoek waar de robot op de huidige kaart staat (één
  lidarscan tegen de opgeslagen kaart, zie relocalize.py) en geef dat als startpositie aan de
  lokalisatie. Resultaat op /localization/status (std_msgs/String, JSON, latched):
  {"busy": bool, "ok": bool, "message": "..."}.

Doet niets zolang er geen verzoek komt.
"""
import json
import math
import os
import re
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

from wave_rover_base import relocalize

VALID_NAME = re.compile(r'^[A-Za-z0-9_-]{1,40}$')
# Zelf lokaliseren: zo goed moet de scan passen, en zoveel beter dan de beste andere plek
# (score per straal, zie relocalize.beam_score; living_v3: 0,62 tegen 0,33).
FIND_MIN_SCORE = 0.35
FIND_MIN_MARGIN = 0.12


class MapManager(Node):

    def __init__(self):
        super().__init__('map_manager')
        self.declare_parameter('maps_dir', '/home/ubuntu/wave_rover/maps')
        self.maps_dir = self.get_parameter('maps_dir').value
        os.makedirs(self.maps_dir, exist_ok=True)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.list_pub = self.create_publisher(String, 'maps/list', latched)
        self.create_subscription(String, 'maps/save', self.on_save, 10)
        self.create_subscription(String, 'maps/select', self.on_select, 10)
        self.create_subscription(PoseStamped, 'robot_pose', self.on_pose, 10)
        self.serialize = self.create_client(SerializePoseGraph, 'slam_toolbox/serialize_map')

        self.robot_pose = None
        self.busy = False
        self.publish_list('')

        self.find_pub = self.create_publisher(String, 'localization/status', latched)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_subscription(Empty, 'localization/find', self.on_find, 10)
        self.finding = False
        self.laser_offset = None  # (x, y, yaw) van de lidar op de robot, eenmalig uit TF
        self.publish_find(False, '')

    # --- lijst ----------------------------------------------------------------------

    def maps(self):
        return sorted(f[:-len('.posegraph')] for f in os.listdir(self.maps_dir)
                      if f.endswith('.posegraph'))

    def current(self):
        try:
            with open(os.path.join(self.maps_dir, 'current')) as f:
                return f.read().strip()
        except OSError:
            return ''

    def publish_list(self, message):
        data = {'maps': self.maps(), 'current': self.current(), 'message': message,
                'busy': self.busy}
        self.list_pub.publish(String(data=json.dumps(data)))

    # --- kiezen ---------------------------------------------------------------------

    def on_select(self, msg):
        name = msg.data.strip()
        if name not in self.maps():
            self.publish_list(f'Kaart "{name}" bestaat niet')
            return
        with open(os.path.join(self.maps_dir, 'current'), 'w') as f:
            f.write(name + '\n')
        self.get_logger().info(f'Kaart voor navigatie: {name}')
        self.publish_list(f'"{name}" gekozen; (her)start de navigatie')

    # --- opslaan --------------------------------------------------------------------

    def on_pose(self, msg):
        q = msg.pose.orientation
        self.robot_pose = {
            'x': round(msg.pose.position.x, 3),
            'y': round(msg.pose.position.y, 3),
            'yaw': round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 3),
        }

    def on_save(self, msg):
        name = msg.data.strip()
        if not VALID_NAME.match(name):
            self.publish_list('Ongeldige naam: gebruik letters, cijfers, - of _')
            return
        if self.busy:
            self.publish_list('Er wordt al een kaart opgeslagen')
            return
        if not self.serialize.service_is_ready():
            self.publish_list('Geen kaart om op te slaan: zet eerst de module "slam" aan')
            return
        self.busy = True
        self.publish_list(f'"{name}" opslaan…')
        threading.Thread(target=self.save, args=(name,), daemon=True).start()

    def save(self, name):
        path = os.path.join(self.maps_dir, name)
        try:
            # Afbeelding (.pgm/.yaml) met de map_saver van Nav2.
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path,
                 '--ros-args', '-p', 'map_subscribe_transient_local:=true'],
                capture_output=True, text=True, timeout=60)
            if 'saved successfully' not in (result.stdout + result.stderr):
                raise RuntimeError('afbeelding opslaan mislukt')

            # Posegraph voor de lokalisatie van slam_toolbox.
            done = threading.Event()
            future = self.serialize.call_async(SerializePoseGraph.Request(filename=path))
            future.add_done_callback(lambda _f: done.set())
            if not done.wait(60) or future.result() is None or future.result().result != 0:
                raise RuntimeError('posegraph opslaan mislukt')

            # Huidige positie: de navigatie start daar, zonder startpositie te zetten.
            if self.robot_pose is not None:
                with open(path + '.lastpose', 'w') as f:
                    json.dump(self.robot_pose, f)

            self.get_logger().info(f'Kaart opgeslagen: {path}')
            message = f'"{name}" opgeslagen'
        except (RuntimeError, subprocess.TimeoutExpired) as e:
            self.get_logger().error(f'Kaart {name}: {e}')
            message = f'Fout bij opslaan van "{name}": {e}'
        finally:
            self.busy = False
        self.publish_list(message)

    # --- zelf lokaliseren -------------------------------------------------------------

    def publish_find(self, ok, message):
        data = {'busy': self.finding, 'ok': ok, 'message': message}
        self.find_pub.publish(String(data=json.dumps(data)))

    def on_find(self, _msg):
        if self.finding:
            return
        if self.count_subscribers('initialpose') == 0:
            self.publish_find(False, 'Zet eerst de module "navigation" aan')
            return
        self.finding = True
        self.publish_find(False, 'Zoeken…')
        threading.Thread(target=self.find, daemon=True).start()

    def wait_for_scan(self, timeout=3.0):
        got, done = [], threading.Event()

        def on_scan(msg):
            got.append(msg)
            done.set()

        sub = self.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)
        done.wait(timeout)
        self.destroy_subscription(sub)
        return got[0] if got else None

    def lookup_laser_offset(self, frame, timeout=3.0):
        buffer = Buffer()
        listener = TransformListener(buffer, self)
        try:
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                try:
                    t = buffer.lookup_transform('base_footprint', frame, Time()).transform
                    q = t.rotation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                    return (t.translation.x, t.translation.y, yaw)
                except Exception:
                    time.sleep(0.1)
            return None
        finally:
            listener.unregister()

    def find(self):
        ok, message = False, ''
        try:
            name = self.current()
            yaml_path = os.path.join(self.maps_dir, name + '.yaml')
            if not name or not os.path.exists(yaml_path):
                raise RuntimeError('geen kaart gekozen')
            scan = self.wait_for_scan()
            if scan is None:
                raise RuntimeError('geen lidarscan (staat de lidar aan?)')
            if self.laser_offset is None:
                self.laser_offset = self.lookup_laser_offset(scan.header.frame_id)
                if self.laser_offset is None:
                    raise RuntimeError('positie van de lidar op de robot onbekend (TF)')
            started = time.monotonic()
            beams = relocalize.Scan(scan.ranges, scan.angle_min, scan.angle_increment,
                                    scan.range_min, scan.range_max, self.laser_offset)
            pose, best, second = relocalize.localize(relocalize.OccupancyMap(yaml_path), beams)
            took = time.monotonic() - started
            self.get_logger().info(
                f'Zelf lokaliseren op {name}: {pose} score {best:.2f} (tweede {second:.2f}, '
                f'{took:.1f} s)')
            if pose is None or best < FIND_MIN_SCORE:
                raise RuntimeError('de scan past nergens goed; staat de robot op deze kaart?')
            if best - second < FIND_MIN_MARGIN:
                raise RuntimeError('twee plekken lijken op elkaar; draai de robot een beetje '
                                   'of zet hem ergens anders en probeer opnieuw')
            self.send_initial_pose(pose)
            ok, message = True, f'Gevonden (zekerheid {round(100 * best)}%)'
        except RuntimeError as e:
            message = f'Niet gevonden: {e}'
        finally:
            self.finding = False
        self.publish_find(ok, message)

    def send_initial_pose(self, pose):
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = Time().to_msg()  # 0 = laatst bekende TF, zie initialpose_relay
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05 ** 2
        msg.pose.covariance[35] = math.radians(3) ** 2
        self.initial_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
