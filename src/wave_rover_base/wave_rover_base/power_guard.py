"""Sluit de Jetson netjes af: op vraag (knop in de app) of voor de UPS-batterij te diep ontlaadt.

- Zakt /ups/battery onder `shutdown_percentage` terwijl ze ontlaadt, en dat `confirm_time`
  seconden lang (korte spanningsdips bij het optrekken tellen niet), dan start een
  aftelling van `countdown` seconden.
- Laadt de batterij weer (adapter ingestoken), dan wordt de aftelling geannuleerd.
- Bij 0, of bij de service /power/shutdown (std_srvs/Trigger, knop "Robot afsluiten"):
  alle modules stoppen (/modules/stop_all, zet ook de lidarmotor stil), via de speaker
  zeggen dat de schakelaars uit moeten, en dan het verzoekbestand `request_file` aanmaken.
  Een systemd-path-unit op de Jetson (scripts/install_power_guard.sh) ziet dat en zet de
  Jetson uit. Voorheen zette Jonas de UPS-schakelaar uit terwijl de Jetson nog draaide.

Publiceert /power/shutdown_in (std_msgs/Int32): seconden tot afsluiten, -1 = geen aftelling.
"""
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from wave_rover_base import speech

DISCHARGING = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
GOODBYE = ('Ik sluit af. Wacht tot ik helemaal uit ben, en zet dan de schakelaar van de UPS '
           'en het knopje op het chassis uit.')


class PowerGuard(Node):

    def __init__(self):
        super().__init__('power_guard')
        # 20% i.p.v. 10%: de UPS meet alleen het hele pakket en balanceert de cellen niet.
        # Op 28-10-2026 bleek na een afsluiting op 10% één cel op 2,66 V te staan (andere 4,2 V).
        self.declare_parameter('shutdown_percentage', 20.0)
        self.declare_parameter('confirm_time', 30.0)
        self.declare_parameter('countdown', 60)
        self.declare_parameter('request_file', '/home/ubuntu/wave_rover/run/shutdown_request')
        self.threshold = self.get_parameter('shutdown_percentage').value
        self.confirm_time = self.get_parameter('confirm_time').value
        self.countdown = self.get_parameter('countdown').value
        self.request_file = self.get_parameter('request_file').value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Int32, 'power/shutdown_in', latched)
        self.create_subscription(BatteryState, 'ups/battery', self.on_battery, 10)
        self.create_service(Trigger, 'power/shutdown', self.on_shutdown)
        self.stop_modules = self.create_client(Trigger, 'modules/stop_all')
        self.create_timer(1.0, self.tick)

        self.shutting_down = False
        self.low_since = None
        self.deadline = None
        self.pub.publish(Int32(data=-1))

    def on_battery(self, msg):
        low = (msg.present and msg.power_supply_status == DISCHARGING
               and msg.percentage * 100.0 < self.threshold)
        if low:
            if self.low_since is None:
                self.low_since = time.monotonic()
        else:
            self.low_since = None
            if self.deadline is not None and msg.power_supply_status != DISCHARGING:
                self.get_logger().info('Adapter ingestoken: afsluiten geannuleerd')
                self.deadline = None
                self.pub.publish(Int32(data=-1))

    def tick(self):
        now = time.monotonic()
        if (self.deadline is None and self.low_since is not None
                and now - self.low_since >= self.confirm_time):
            self.deadline = now + self.countdown
            self.get_logger().warn(
                f'Batterij Jetson onder {self.threshold:.0f}%: afsluiten over {self.countdown} s')
            self.say_async('Mijn batterij is bijna leeg. Steek de adapter in, anders sluit ik '
                           'over een minuut af.')
        if self.deadline is None:
            return
        remaining = max(0, int(round(self.deadline - now)))
        self.pub.publish(Int32(data=remaining))
        if remaining == 0:
            self.get_logger().error('Batterij Jetson leeg: Jetson wordt afgesloten')
            self.deadline = None
            self.begin_shutdown('batterij leeg')

    # --- afsluiten --------------------------------------------------------------------

    def on_shutdown(self, _request, response):
        response.success = not self.shutting_down
        response.message = 'Afsluiten gestart' if response.success else 'Al bezig met afsluiten'
        if response.success:
            self.begin_shutdown('knop in de app')
        return response

    def begin_shutdown(self, reason):
        if self.shutting_down:
            return
        self.shutting_down = True
        self.get_logger().warn(f'Robot afsluiten ({reason})')
        # Modules stoppen duurt seconden: niet in de ROS-thread (die moet het antwoord ontvangen).
        threading.Thread(target=self.shutdown_sequence, args=(reason,), daemon=True).start()

    def shutdown_sequence(self, reason):
        if self.stop_modules.wait_for_service(timeout_sec=2.0):
            future = self.stop_modules.call_async(Trigger.Request())
            end = time.monotonic() + 60.0
            while not future.done() and time.monotonic() < end:
                time.sleep(0.2)
        else:
            self.get_logger().warn('module_manager niet bereikbaar: modules niet gestopt')
        self.say(GOODBYE)
        os.makedirs(os.path.dirname(self.request_file), exist_ok=True)
        with open(self.request_file, 'w') as f:
            f.write(reason + '\n')

    def say(self, text):
        try:
            speech.say(text)
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as e:
            self.get_logger().warn(f'Speaker: {e}')

    def say_async(self, text):
        threading.Thread(target=self.say, args=(text,), daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = PowerGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
