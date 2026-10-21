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
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool

STOP_TIMEOUT = 10.0
# Zo lang moet een module draaien voor ze als "aan" geldt.
STARTUP_GRACE = 2.0


class Module:

    def __init__(self, name, spec):
        self.name = name
        self.description = spec.get('description', '')
        self.package = spec['package']
        self.launch = spec['launch']
        self.args = [f'{k}:={v}' for k, v in spec.get('args', {}).items()]
        self.requires = list(spec.get('requires', []))
        self.conflicts = list(spec.get('conflicts', []))
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
            log = open(os.path.join(self.log_dir, f'{name}.log'), 'w')
            cmd = ['ros2', 'launch', module.package, module.launch, *module.args]
            module.process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True)
            module.started_at = time.monotonic()
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

        # ros2 launch sluit netjes af op SIGINT; daarna desnoods harder.
        pgid = os.getpgid(process.pid)
        os.killpg(pgid, signal.SIGINT)
        try:
            process.wait(STOP_TIMEOUT)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'{name} reageert niet, wordt geforceerd gestopt')
            os.killpg(pgid, signal.SIGKILL)
            process.wait()

        with self.lock:
            module.state = 'uit'

    def stop_all(self):
        for name in self.modules:
            self.stop(name)

    # --- status -------------------------------------------------------------------

    def update_states(self):
        changed = False
        with self.lock:
            for module in self.modules.values():
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
