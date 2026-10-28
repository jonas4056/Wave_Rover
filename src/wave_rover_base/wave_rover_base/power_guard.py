"""Sluit de Jetson netjes af voor de batterij van de UPS te diep ontlaadt.

- Zakt /ups/battery onder `shutdown_percentage` terwijl ze ontlaadt, en dat `confirm_time`
  seconden lang (korte spanningsdips bij het optrekken tellen niet), dan start een
  aftelling van `countdown` seconden.
- Laadt de batterij weer (adapter ingestoken), dan wordt de aftelling geannuleerd.
- Bij 0: verzoekbestand `request_file` aanmaken. Een systemd-path-unit op de Jetson
  (scripts/install_power_guard.sh) ziet dat en zet de Jetson uit.

Publiceert /power/shutdown_in (std_msgs/Int32): seconden tot afsluiten, -1 = geen aftelling.
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32

DISCHARGING = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING


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
        self.create_timer(1.0, self.tick)

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
        if self.deadline is None:
            return
        remaining = max(0, int(round(self.deadline - now)))
        self.pub.publish(Int32(data=remaining))
        if remaining == 0:
            self.get_logger().error('Batterij Jetson leeg: Jetson wordt afgesloten')
            os.makedirs(os.path.dirname(self.request_file), exist_ok=True)
            with open(self.request_file, 'w') as f:
                f.write('batterij leeg\n')
            self.deadline = None


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
