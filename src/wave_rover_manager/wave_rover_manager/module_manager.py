"""Start en stopt modules van de Wave Rover op vraag.

Een module is een launchbestand, beschreven in een YAML-bestand (parameter
`modules_file`):

    modules:
      lidar:
        description: RPLidar C1 -> /scan
        package: wave_rover_bringup
        launch: lidar.launch.py
      slam:
        package: wave_rover_bringup
        launch: slam.launch.py
        requires: [lidar]
        conflicts: [navigation]
        on_stop: [ros2, run, wave_rover_base, lidar_stop]
        health: {topic: /scan, type: sensor_msgs/msg/LaserScan, timeout: 3.0}

`health`: komt er bij een draaiende module langer dan `timeout` seconden niets binnen op
`topic`, dan wordt de module herstart (zonder de modules die ervan afhangen te stoppen).
Nodig voor drivers die vastlopen in plaats van te stoppen, bv. als een USB-apparaat even
wegvalt.

`on_stop` is een commando dat na het stoppen van de module wordt uitgevoerd, ook na
een crash, en eenmaal bij het opstarten van de manager (opruimen na een herstart).

Interface (bruikbaar vanaf de terminal en later vanaf het dashboard):
- service  /modules/<naam>/enable  (std_srvs/SetBool): true = starten, false = stoppen.
  Bij starten worden de modules uit `conflicts` gestopt en die uit `requires` eerst
  gestart; bij stoppen worden de modules die ervan afhangen eerst gestopt.
- topic    /modules/status  (std_msgs/String, JSON, transient local):
  {"lidar": {"state": "aan", "description": "..."}, ...}
  Mogelijke states: uit, start, aan, stopt, fout.

Elke module draait als aparte `ros2 launch`-procesgroep; de uitvoer komt in
~/.ros/modules/<naam>.log.
"""
import json
import os
import signal
import subprocess
import threading
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from std_srvs.srv import SetBool

STOP_TIMEOUT = 10.0
# Zo lang moet een module draaien voor ze als "aan" geldt.
STARTUP_GRACE = 2.0
# Na een (her)start zo lang wachten voor de gezondheidscontrole meetelt (driver moet
# verbinden, lidarmotor op toeren komen).
HEALTH_GRACE = 10.0


class Module:

    def __init__(self, name, spec):
        self.name = name
        self.description = spec.get('description', '')
        self.package = spec['package']
        self.launch = spec['launch']
        self.args = [f'{k}:={v}' for k, v in spec.get('args', {}).items()]
        self.requires = list(spec.get('requires', []))
        self.conflicts = list(spec.get('conflicts', []))
        self.on_stop = list(spec.get('on_stop', []))
        self.health = spec.get('health')
        self.last_health = 0.0
        self.restarting = False
        self.process = None
        self.started_at = 0.0
        self.state = 'uit'


class ModuleManager(Node):

    def __init__(self):
        super().__init__('module_manager')
        self.declare_parameter('modules_file', '')
        path = self.get_parameter('modules_file').value
        with open(path) as f:
            specs = yaml.safe_load(f)['modules']

        self.modules = {name: Module(name, spec) for name, spec in specs.items()}
        self.lock = threading.RLock()
        self.log_dir = os.path.expanduser('~/.ros/modules')
        os.makedirs(self.log_dir, exist_ok=True)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'modules/status', qos)
        for name in self.modules:
            self.create_service(SetBool, f'modules/{name}/enable',
                                lambda req, resp, n=name: self.on_enable(n, req, resp))

        for module in self.modules.values():
            self.run_on_stop(module)
            if module.health:
                self.create_subscription(
                    get_message(module.health['type']), module.health['topic'],
                    lambda _msg, m=module: setattr(m, 'last_health', time.monotonic()),
                    qos_profile_sensor_data)
        self.create_timer(1.0, self.update_states)
        self.get_logger().info(f'Modules: {", ".join(self.modules)}')
        self.publish_status()

    # --- services -----------------------------------------------------------------

    def on_enable(self, name, request, response):
        try:
            if request.data:
                self.start(name)
                response.message = f'{name} gestart'
            else:
                self.stop(name)
                response.message = f'{name} gestopt'
            response.success = True
        except Exception as e:  # noqa: BLE001 - fout teruggeven aan de aanroeper
            response.success = False
            response.message = str(e)
        self.publish_status()
        return response

    # --- starten en stoppen -------------------------------------------------------

    def start(self, name, chain=()):
        if name in chain:
            raise RuntimeError(f'Kringverwijzing in requires: {" -> ".join(chain + (name,))}')
        module = self.modules.get(name)
        if module is None:
            raise RuntimeError(f'Onbekende module: {name}')

        for other in module.conflicts:
            if other in self.modules:
                self.stop(other)
        for dep in module.requires:
            self.start(dep, chain + (name,))

        with self.lock:
            if module.process and module.process.poll() is None:
                return
            self.launch(module)

    def launch(self, module, log_mode='w'):
        with self.lock:
            name = module.name
            log = open(os.path.join(self.log_dir, f'{name}.log'), log_mode)
            cmd = ['ros2', 'launch', module.package, module.launch, *module.args]
            module.process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True)
            module.started_at = time.monotonic()
            module.last_health = module.started_at
            module.state = 'start'
            self.get_logger().info(f'Start {name}: {" ".join(cmd)}')

    def stop(self, name):
        module = self.modules[name]
        for other in self.modules.values():
            if name in other.requires and other.process and other.process.poll() is None:
                self.stop(other.name)

        with self.lock:
            process = module.process
            if not process or process.poll() is not None:
                module.state = 'uit'
                return
            module.state = 'stopt'
        self.get_logger().info(f'Stop {name}')
        self.publish_status()
        self.kill(module, process)
        self.run_on_stop(module)
        with self.lock:
            module.state = 'uit'

    def kill(self, module, process):
        # ros2 launch sluit netjes af op SIGINT; daarna desnoods harder.
        pgid = os.getpgid(process.pid)
        os.killpg(pgid, signal.SIGINT)
        try:
            process.wait(STOP_TIMEOUT)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'{module.name} reageert niet, wordt geforceerd gestopt')
            os.killpg(pgid, signal.SIGKILL)
            process.wait()

    def restart_unhealthy(self, module):
        """Herstart alleen deze module; modules die ervan afhangen blijven draaien."""
        self.get_logger().error(
            f'{module.name}: niets ontvangen op {module.health["topic"]} sinds '
            f'{module.health["timeout"]} s, module wordt herstart')
        try:
            self.kill(module, module.process)
            self.run_on_stop(module)
            self.launch(module, log_mode='a')
        finally:
            module.restarting = False
        self.publish_status()

    def run_on_stop(self, module):
        if not module.on_stop:
            return
        try:
            result = subprocess.run(module.on_stop, capture_output=True, text=True, timeout=15)
            output = (result.stdout + result.stderr).strip()
            self.get_logger().info(f'on_stop {module.name}: {output or result.returncode}')
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().warn(f'on_stop {module.name} mislukt: {e}')

    def stop_all(self):
        for name in self.modules:
            self.stop(name)

    # --- status -------------------------------------------------------------------

    def update_states(self):
        now = time.monotonic()
        for module in self.modules.values():
            if (module.health and module.state == 'aan' and not module.restarting
                    and now - module.started_at > HEALTH_GRACE
                    and now - module.last_health > module.health['timeout']):
                module.restarting = True
                module.state = 'start'
                threading.Thread(target=self.restart_unhealthy, args=(module,), daemon=True).start()
        changed = False
        with self.lock:
            for module in self.modules.values():
                if module.restarting:
                    continue
                process = module.process
                if module.state == 'start' and process.poll() is None \
                        and time.monotonic() - module.started_at > STARTUP_GRACE:
                    module.state = 'aan'
                    changed = True
                elif module.state in ('start', 'aan') and process.poll() is not None:
                    module.state = 'fout'
                    self.get_logger().error(
                        f'{module.name} is gestopt met code {process.returncode}; '
                        f'zie {self.log_dir}/{module.name}.log')
                    threading.Thread(target=self.run_on_stop, args=(module,), daemon=True).start()
                    changed = True
        if changed:
            self.publish_status()

    def publish_status(self):
        with self.lock:
            status = {m.name: {'state': m.state, 'description': m.description,
                               'requires': m.requires, 'conflicts': m.conflicts}
                      for m in self.modules.values()}
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = ModuleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.try_shutdown()
