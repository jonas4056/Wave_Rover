"""Zelf verkennen: de robot rijdt naar de randen van de kaart tot alles verkend is.

Onderdeel van de module explore (explore.launch.py: slam_toolbox maakt de kaart, Nav2 rijdt).
1. Wacht op /explore/start (std_msgs/String: naam van de nieuwe kaart), van het dashboard.
2. Zoekt op /map de "randen": vrije cellen naast onbekende cellen, niet tegen een muur,
   gegroepeerd. Kiest de beste rand (dichtbij en lang; mislukte doelen niet opnieuw) en rijdt
   erheen met Nav2, trager dan gewoon (`speed`). Is de rand onderweg al verkend, dan meteen
   de volgende; komt de robot `stuck_time` s niet vooruit, dan wordt die rand overgeslagen.
   Met `max_radius` (m, ook tijdens het verkennen aan te passen) blijft de robot binnen die
   straal rond het startpunt: een verdere rand wordt tot op de straal benaderd (bv. aan de
   voedingskabel, of om in één kamer te blijven).
3. Geen bereikbare rand meer of `max_time` voorbij: terug naar het startpunt, kaart opslaan
   onder de naam (via map_manager) en kiezen voor de navigatie. Bij /explore/stop
   (std_msgs/Empty) of de stopknop van de kaart: meteen stoppen en opslaan, zonder terugrijden.
Staat het chassis uit (schakelaar, /chassis/battery < 6 V), dan wacht hij en meldt dat; doelen
die dan mislukken tellen niet als onbereikbaar.
Status op /explore/status (std_msgs/String, JSON, latched):
  {"state": "wacht|verkennen|terug|stoppen|opslaan|klaar|fout", "message": "...", "name": "..."}
Rekent alleen bij het kiezen en controleren van een doel (numpy, enkele ms). De positie komt
van /robot_pose (pose_saver), zonder eigen TF-listener.
"""
import json
import math
import re
import time
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty, String

FREE_MAX = 25      # bezettingskans (%) tot waar een cel vrij is
OCCUPIED_MIN = 65  # vanaf hier een muur of obstakel
CHASSIS_MIN_V = 6.0  # daaronder staat de schakelaar van het chassis uit (gemeten ~0,2 V)
VALID_NAME = re.compile(r'^[A-Za-z0-9_-]{1,40}$')  # zoals map_manager


def grow(mask, cells):
    """`mask` uitbreiden met `cells` cellen in alle richtingen."""
    out = mask.copy()
    for _ in range(cells):
        grown = out.copy()
        grown[1:] |= out[:-1]
        grown[:-1] |= out[1:]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        out = grown
    return out


def find_frontiers(grid, clearance_cells, min_cells):
    """Randen tussen vrije en onbekende ruimte: lijst van groepen [(rij, kolom), ...]."""
    unknown = grid < 0
    near_unknown = grow(unknown, 1)
    free = (grid >= 0) & (grid < FREE_MAX)
    # Alleen randen aan een vrije ruimte waar de robot in past: geen dunne "stralen" waar de
    # lidar door een smalle opening of onder een kast keek.
    roomy = ~grow(~free, clearance_cells)
    frontier = (free & near_unknown & ~grow(grid >= OCCUPIED_MIN, clearance_cells)
                & grow(roomy, clearance_cells + 1))
    cells = set(map(tuple, np.argwhere(frontier).tolist()))
    groups = []
    while cells:
        first = cells.pop()
        group, queue = [first], deque([first])
        while queue:
            r, c = queue.popleft()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    n = (r + dr, c + dc)
                    if n in cells:
                        cells.remove(n)
                        group.append(n)
                        queue.append(n)
        if len(group) >= min_cells:
            groups.append(group)
    return groups


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')
        self.declare_parameter('speed', 0.2)            # m/s tijdens het verkennen
        self.declare_parameter('max_time', 900.0)       # s, daarna terug en opslaan
        self.declare_parameter('min_frontier', 0.25)    # m, kortere randen negeren
        self.declare_parameter('clearance', 0.15)       # m, rand niet vlak tegen een muur
        # m afstand die 1 m rand waard is. Klein: dichtbij eerst (test 29-10: met 1,0 koos hij
        # een lange rand op 4,3 m i.p.v. kleine randen op 1 m).
        self.declare_parameter('size_gain', 0.3)
        self.declare_parameter('stuck_time', 20.0)      # s zonder 15 cm vooruit = overslaan
        self.declare_parameter('failed_radius', 0.4)    # m rond een mislukt doel overslaan
        self.declare_parameter('max_radius', 0.0)       # m rond het startpunt, 0 = geen grens
        p = {n: self.get_parameter(n).value for n in (
            'speed', 'max_time', 'min_frontier', 'clearance', 'size_gain', 'stuck_time',
            'failed_radius')}
        self.p = p

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'explore/status', latched)
        self.save_pub = self.create_publisher(String, 'maps/save', 10)
        self.select_pub = self.create_publisher(String, 'maps/select', 10)
        self.speed_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.create_subscription(String, 'explore/start', self.on_start, 10)
        self.create_subscription(Empty, 'explore/stop', lambda _m: self.stop('gestopt'), 10)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, latched)
        self.create_subscription(PoseStamped, 'robot_pose', self.on_pose, 10)
        self.create_subscription(String, 'maps/list', self.on_maps, latched)
        self.create_subscription(Bool, 'e_stop', self.on_e_stop, latched)
        self.create_subscription(BatteryState, 'chassis/battery', self.on_chassis, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.grid = None      # numpy (rijen = y, kolommen = x)
        self.map_info = None  # (resolutie, oorsprong x, oorsprong y)
        self.pose = None      # (x, y, yaw) in map
        self.maps = []
        self.e_stop = False
        self.chassis_v = None

        self.state, self.name, self.started = 'wacht', '', 0.0
        self.pending = ''         # naam van een start die wacht tot kaart en Nav2 klaar zijn
        self.start_pose = None
        self.return_sent = False
        self.failed = []          # mislukte doelen (x, y)
        self.goal = None          # (x, y) van het huidige Nav2-doel
        self.target = None        # (x, y) van de rand waarvoor we rijden (zonder leiband)
        self.goal_handle = None
        self.cancel_reason = None  # 'verkend' / 'vast' / 'stop' als wij annuleren
        self.progress = (None, 0.0)  # (positie, tijd) van de laatste 15 cm vooruitgang
        self.goals_done = 0
        self.next_goal_at = 0.0
        self.paused = False       # chassis uit
        self.publish_status('Wacht op start (knop "Verken" in het dashboard)')
        self.create_timer(1.0, self.tick)

    # --- ontvangen ------------------------------------------------------------------

    def on_map(self, msg):
        info = msg.info
        self.grid = np.asarray(msg.data, np.int8).reshape(info.height, info.width)
        self.map_info = (info.resolution, info.origin.position.x, info.origin.position.y)

    def on_pose(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def on_maps(self, msg):
        data = json.loads(msg.data)
        self.maps = data.get('maps', [])
        message = data.get('message', '')
        if self.state != 'opslaan':
            return
        if message == f'"{self.name}" opgeslagen':
            self.select_pub.publish(String(data=self.name))
            self.state = 'klaar'
            self.publish_status(f'Klaar: kaart "{self.name}" opgeslagen en gekozen; zet '
                                f'"navigation" aan om ze te gebruiken')
        elif message.startswith(f'Fout bij opslaan van "{self.name}"'):
            self.state = 'fout'
            self.publish_status(message)

    def on_e_stop(self, msg):
        self.e_stop = msg.data

    def on_chassis(self, msg):
        self.chassis_v = msg.voltage

    def motors_off(self):
        return self.chassis_v is not None and self.chassis_v < CHASSIS_MIN_V

    def on_start(self, msg):
        name = msg.data.strip()
        if self.state not in ('wacht', 'klaar', 'fout') or name == self.pending:
            return  # bezig; het dashboard herhaalt de start tot de status verandert
        if not VALID_NAME.match(name):
            self.fail(f'Ongeldige naam {name!r}: gebruik letters, cijfers, - of _')
        elif name in self.maps:
            self.fail(f'Er bestaat al een kaart "{name}"; kies een andere naam')
        else:
            self.pending, self.state = name, 'wacht'
            self.publish_status('Wachten tot de kaart en Nav2 klaar zijn…')

    def begin(self):
        self.name, self.pending, self.started = self.pending, '', time.monotonic()
        self.start_pose, self.failed, self.goals_done = self.pose, [], 0
        self.state = 'verkennen'
        self.get_logger().info(f'Verkennen gestart, kaart "{self.name}"')
        self.publish_status('Verkennen…')

    # --- elke seconde ---------------------------------------------------------------

    def tick(self):
        if (self.state == 'wacht' and self.pending and self.grid is not None
                and self.pose is not None and self.nav.server_is_ready()):
            self.begin()
        if self.state != 'verkennen':
            return
        now = time.monotonic()
        if self.motors_off():
            if not self.paused:
                self.paused = True
                self.publish_status('Het chassis staat uit: zet de schakelaar aan, dan rijdt hij verder')
            self.progress = (None, 0.0)  # telt niet als vastzitten
            return
        if self.paused:
            self.paused = False
            self.publish_status('Verkennen…')
        if now - self.started > self.p['max_time']:
            self.finish(f'Tijd om ({self.p["max_time"] / 60:.0f} min)')
        elif self.goal_handle is not None:
            self.check_goal(now)
        elif self.goal is None and now >= self.next_goal_at:
            self.next_goal()

    def check_goal(self, now):
        """Rand onderweg al verkend, of komt de robot niet vooruit? Dan dit doel stoppen."""
        if self.cancel_reason is not None or self.pose is None:
            return
        if not self.frontier_left(self.target):
            self.cancel('verkend')
            return
        last, since = self.progress
        if last is None or self.e_stop or math.dist(last, self.pose[:2]) > 0.15:
            self.progress = (self.pose[:2], now)  # noodstop telt niet als vastzitten
        elif now - since > self.p['stuck_time']:
            self.cancel('vast')

    # --- doelen ---------------------------------------------------------------------

    def next_goal(self):
        if self.grid is None or self.pose is None:
            return
        res, ox, oy = self.map_info
        groups = find_frontiers(self.grid, max(1, round(self.p['clearance'] / res)),
                                max(2, round(self.p['min_frontier'] / res)))
        best = None
        for group in groups:
            cells = np.array(group)
            # Doel: de randcel het dichtst bij het midden van de groep.
            r, c = cells[np.argmin(((cells - cells.mean(axis=0)) ** 2).sum(axis=1))]
            x, y = ox + (c + 0.5) * res, oy + (r + 0.5) * res
            if any(math.dist((x, y), f) < self.p['failed_radius'] for f in self.failed):
                continue
            cost = math.dist((x, y), self.pose[:2]) - self.p['size_gain'] * len(group) * res
            if best is None or cost < best[0]:
                best = (cost, x, y)
        if best is None:
            self.finish('Alles verkend')
            return
        self.publish_status(f'Verkennen… ({len(groups)} randen, {self.goals_done} bezocht)')
        _, x, y = best
        self.target = (x, y)
        limit = self.get_parameter('max_radius').value
        sx, sy, _ = self.start_pose
        if limit > 0 and math.dist((x, y), (sx, sy)) > limit:
            scale = limit / math.dist((x, y), (sx, sy))
            x, y = sx + (x - sx) * scale, sy + (y - sy) * scale
        if not self.send_goal(x, y):
            self.fail('Nav2 is niet klaar (navigate_to_pose)')

    def frontier_left(self, goal):
        """Is er rond het doel (±10 cm) nog onbekende ruimte naast vrije?"""
        res, ox, oy = self.map_info
        c, r = int((goal[0] - ox) / res), int((goal[1] - oy) / res)
        k = max(1, round(0.10 / res))
        window = self.grid[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1]
        return bool((window < 0).any())

    def send_goal(self, x, y):
        # Snelheid begrenzen (RPP); opnieuw bij elk doel, mocht de controller herstart zijn.
        self.speed_pub.publish(SpeedLimit(percentage=False, speed_limit=self.p['speed']))
        yaw = math.atan2(y - self.pose[1], x - self.pose[0])
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not self.nav.server_is_ready():
            return False
        self.goal, self.cancel_reason, self.progress = (x, y), None, (None, 0.0)
        self.get_logger().info(f'Doel ({x:.2f}, {y:.2f})')
        self.nav.send_goal_async(msg).add_done_callback(self.on_goal_response)
        return True

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.goal_done(GoalStatus.STATUS_ABORTED)
            return
        self.goal_handle = handle
        if self.cancel_reason is not None:  # intussen gestopt
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f: self.goal_done(f.result().status))

    def cancel(self, reason):
        self.cancel_reason = reason
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def goal_done(self, status):
        goal, reason = self.target, self.cancel_reason
        self.goal = self.target = self.goal_handle = self.cancel_reason = None
        if self.state == 'terug':
            # Eerst het lopende doel geannuleerd, dan terugrijden; daarna (gelukt of niet) opslaan.
            if self.return_sent:
                self.save()
            else:
                self.send_return()
            return
        if self.state == 'stoppen':
            self.save()
            return
        if self.state != 'verkennen' or goal is None:
            return
        if status == GoalStatus.STATUS_CANCELED and reason is None:
            self.stop('gestopt met de stopknop')  # iemand anders annuleerde: stoppen
            return
        self.goals_done += 1
        if self.motors_off():
            return  # mislukt omdat de motoren geen stroom hadden: later opnieuw proberen
        # Niet bereikt, vast, of bereikt maar de rand blijft onbekend: niet opnieuw proberen.
        if (status == GoalStatus.STATUS_ABORTED or reason == 'vast'
                or (status == GoalStatus.STATUS_SUCCEEDED and self.frontier_left(goal))):
            self.failed.append(goal)
        self.next_goal_at = time.monotonic() + 1.0  # kaart even laten bijwerken

    # --- einde ----------------------------------------------------------------------

    def finish(self, why):
        """Klaar met verkennen: terug naar het startpunt en dan opslaan."""
        self.get_logger().info(f'{why}: terug naar het startpunt')
        self.state, self.return_sent = 'terug', False
        self.publish_status(f'{why}; terug naar het startpunt…')
        if self.goal is not None:
            self.cancel('stop')  # goal_done stuurt daarna het terugdoel
        else:
            self.send_return()

    def send_return(self):
        self.return_sent = True
        x, y, _ = self.start_pose
        if not self.send_goal(x, y):
            self.save()  # niet terug kunnen rijden: de kaart toch bewaren

    def stop(self, why):
        if self.state == 'wacht' and self.pending:
            self.pending = ''
            self.publish_status('Niet gestart')
        if self.state not in ('verkennen', 'terug'):
            return
        self.get_logger().info(f'Verkennen {why}')
        self.state = 'stoppen'
        self.publish_status(f'Verkennen {why}; kaart opslaan…')
        if self.goal is not None:
            self.cancel('stop')  # goal_done slaat daarna op
        else:
            self.save()

    def save(self):
        self.state = 'opslaan'
        self.publish_status(f'Kaart "{self.name}" opslaan…')
        self.save_pub.publish(String(data=self.name))

    def fail(self, message):
        self.get_logger().warn(message)
        self.state = 'fout'
        self.publish_status(message)

    def publish_status(self, message):
        data = {'state': self.state, 'message': message, 'name': self.name}
        self.status_pub.publish(String(data=json.dumps(data)))


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
