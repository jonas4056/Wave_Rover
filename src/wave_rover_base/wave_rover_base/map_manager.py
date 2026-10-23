"""Kaarten opslaan, oplijsten en kiezen (voor het dashboard).

Topics (gewone berichttypes, zodat rosbridge geen eigen interfaces nodig heeft):
- /maps/save (std_msgs/String): naam -> de lopende SLAM-kaart opslaan als
  `<maps_dir>/<naam>` (.posegraph/.data voor slam_toolbox, .pgm/.yaml als afbeelding)
  plus `<naam>.lastpose` met de huidige positie, zodat de navigatie daar meteen start.
- /maps/select (std_msgs/String): naam -> wordt de kaart voor de navigatie
  (`<maps_dir>/current`; navigation.launch.py leest dit bij het starten).
- /maps/list (std_msgs/String, JSON, latched): {"maps": [...], "current": "...", "message": "..."}

Doet niets zolang er geen verzoek komt.
"""
import json
import math
import os
import re
import subprocess
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import String

VALID_NAME = re.compile(r'^[A-Za-z0-9_-]{1,40}$')


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
