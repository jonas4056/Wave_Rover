"""Kaarten opslaan, oplijsten en kiezen (voor het dashboard).

Topics (gewone berichttypes, zodat rosbridge geen eigen interfaces nodig heeft):
- /maps/save (std_msgs/String): naam -> de lopende SLAM-kaart opslaan als
  `<maps_dir>/<naam>` (.posegraph/.data voor slam_toolbox, .pgm/.yaml als afbeelding)
  plus `<naam>.lastpose` met de huidige positie, zodat de navigatie daar meteen start.
- /maps/select (std_msgs/String): naam -> wordt de kaart voor de navigatie
  (`<maps_dir>/current`; navigation.launch.py leest dit bij het starten).
- /maps/list (std_msgs/String, JSON, latched): {"maps": [...], "current": "...", "message": "..."}
- /localization/find (std_msgs/Empty, knop "Zoek mij"): zoek in welke kaart en waar de robot
  staat (één lidarscan tegen de vaste kaarten `fixed_maps` en de gekozen kaart, zie
  relocalize.py) en geef dat als startpositie aan de lokalisatie. Staat hij in een andere
  kaart dan de navigatie gebruikt, dan laadt map_server die (nav2 LoadMap; AMCL en de
  costmaps volgen /map), wordt ze de gekozen kaart en meldt /maps/active (std_msgs/String,
  pad zonder extensie) het aan pose_saver. Resultaat op /localization/status (std_msgs/String,
  JSON, latched): {"busy": bool, "ok": bool, "message": "..."}.
- Hetzelfde gebeurt vanzelf zodra de module navigation aan staat (/modules/status): de robot
  start meestal in het bureau, soms beneden; eigen kaarten (bv. buiten) kies je met de hand.

Rekent alleen bij een zoekopdracht (±1-3 s CPU per kaart).
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
from nav2_msgs.srv import LoadMap
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
# Zoveel beter moet de beste kaart passen dan de beste andere kaart.
MAP_MIN_MARGIN = 0.08


class MapManager(Node):

    def __init__(self):
        super().__init__('map_manager')
        self.declare_parameter('maps_dir', '/home/ubuntu/wave_rover/maps')
        # Vaste plaatsen: daartussen kiest hij zelf (zie locate); eigen kaarten met de hand.
        self.declare_parameter('fixed_maps', ['bureau', 'beneden'])
        self.maps_dir = self.get_parameter('maps_dir').value
        self.fixed_maps = list(self.get_parameter('fixed_maps').value)
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
        self.load_map = self.create_client(LoadMap, 'map_server/load_map')
        self.active_pub = self.create_publisher(String, 'maps/active', 10)
        self.create_subscription(String, 'modules/status', self.on_modules, latched)
        self.nav_state = None
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
                 '--ros-args', '-p', 'map_subscribe_transient_local:=true',
                 # Standaard 2 s: te kort als de Jetson druk is (verkennen, 29-10-2026).
                 '-p', 'save_map_timeout:=10.0'],
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
        threading.Thread(target=self.locate, daemon=True).start()

    def on_modules(self, msg):
        """Navigation net aangezet: zelf zoeken in welke kaart de robot staat."""
        state = json.loads(msg.data).get('navigation', {}).get('state')
        if state == 'aan' and self.nav_state != 'aan' and not self.finding:
            self.finding = True
            self.publish_find(False, 'Navigatie start: zoeken waar ik ben…')
            threading.Thread(target=self.auto_locate, daemon=True).start()
        self.nav_state = state

    def auto_locate(self):
        # Nav2 komt pas na de module op (±20-40 s): wachten op AMCL en map_server.
        end = time.monotonic() + 90.0
        while time.monotonic() < end:
            if self.count_subscribers('initialpose') > 0 and self.load_map.service_is_ready():
                time.sleep(2.0)  # eerste scans en TF
                self.locate()
                return
            time.sleep(1.0)
        self.finding = False
        self.publish_find(False, 'Navigatie niet klaar; probeer later "Zoek mij"')

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

    def map_path(self, name):
        return os.path.join(self.maps_dir, name)

    def locate(self):
        """In welke kaart en waar staat de robot? Vaste kaarten + de gekozen kaart."""
        ok, message = False, ''
        try:
            current = self.current()
            names = [n for n in dict.fromkeys(self.fixed_maps + [current])
                     if n and os.path.exists(self.map_path(n) + '.yaml')]
            if not names:
                raise RuntimeError('geen kaart om in te zoeken')
            scan = self.wait_for_scan()
            if scan is None:
                raise RuntimeError('geen lidarscan (staat de lidar aan?)')
            if self.laser_offset is None:
                self.laser_offset = self.lookup_laser_offset(scan.header.frame_id)
                if self.laser_offset is None:
                    raise RuntimeError('positie van de lidar op de robot onbekend (TF)')
            beams = relocalize.Scan(scan.ranges, scan.angle_min, scan.angle_increment,
                                    scan.range_min, scan.range_max, self.laser_offset)
            results = []
            for name in names:
                started = time.monotonic()
                pose, best, second = relocalize.localize(
                    relocalize.OccupancyMap(self.map_path(name) + '.yaml'), beams)
                self.get_logger().info(
                    f'Zelf lokaliseren op {name}: {pose} score {best:.2f} (tweede {second:.2f}, '
                    f'{time.monotonic() - started:.1f} s)')
                results.append({'name': name, 'pose': pose, 'best': best, 'second': second})
            results.sort(key=lambda r: r['best'], reverse=True)
            found = results[0]
            if found['pose'] is None or found['best'] < FIND_MIN_SCORE:
                raise RuntimeError('de scan past op geen enkele kaart; staat de robot op een '
                                   'plek zonder kaart?')
            if found['best'] - found['second'] < FIND_MIN_MARGIN:
                raise RuntimeError('twee plekken lijken op elkaar; draai de robot een beetje '
                                   'of zet hem ergens anders en probeer opnieuw')
            if len(results) > 1 and found['best'] - results[1]['best'] < MAP_MIN_MARGIN:
                raise RuntimeError(f'twijfel tussen "{found["name"]}" en '
                                   f'"{results[1]["name"]}"; kies de kaart zelf')
            if found['name'] != current:
                self.switch_map(found['name'])
            self.send_initial_pose(found['pose'])
            ok = True
            message = f'In "{found["name"]}" gevonden (zekerheid {round(100 * found["best"])}%)'
        except RuntimeError as e:
            message = f'Niet gevonden: {e}'
        finally:
            self.finding = False
        self.publish_find(ok, message)

    def switch_map(self, name):
        """Andere kaart in map_server laden zonder Nav2 te herstarten, en kiezen."""
        if not self.load_map.wait_for_service(timeout_sec=2.0):
            # bv. localization:=slam (geen map_server)
            raise RuntimeError(f'de robot staat in "{name}": kies die kaart en herstart de '
                               f'navigatie')
        done = threading.Event()
        future = self.load_map.call_async(LoadMap.Request(map_url=self.map_path(name) + '.yaml'))
        future.add_done_callback(lambda _f: done.set())
        if (not done.wait(30.0) or future.result() is None
                or future.result().result != LoadMap.Response.RESULT_SUCCESS):
            raise RuntimeError(f'kaart "{name}" laden mislukt')
        with open(os.path.join(self.maps_dir, 'current'), 'w') as f:
            f.write(name + '\n')
        self.active_pub.publish(String(data=self.map_path(name)))
        self.get_logger().info(f'Kaart gewisseld naar {name}')
        self.publish_list(f'"{name}" geladen (zelf gevonden)')
        time.sleep(1.5)  # AMCL en de costmaps verwerken eerst de nieuwe /map

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
