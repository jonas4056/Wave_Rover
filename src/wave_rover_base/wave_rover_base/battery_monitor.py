"""Bewaakt de batterij van de Jetson (Waveshare UPS Power Module C) en die van het chassis.

- Jetson: INA219 van de UPS via I2C (bus 7, adres 0x41) -> /ups/battery
- Chassis: /battery (van esp32_bridge) wordt aangevuld met een percentage -> /chassis/battery

Beide als sensor_msgs/BatteryState met spanning, stroom (alleen UPS), percentage (0-1)
en laadstatus. Onder `warn_percentage` volgt een waarschuwing in het log.

Laadkabel: zolang de adapter in zit, mag de robot niet rijden (hij zou de kabel meetrekken).
/charging_lock (std_msgs/Bool, latched) is een lock in twist_mux, net als de noodstop, en
blokkeert ook de RC. Kabel in = de batterij ontlaadt niet (stroom > UNPLUGGED_CURRENT; op
batterij trekt de Jetson alleen al >0,5 A). Meteen blokkeren, pas na 2 metingen (4 s)
vrijgeven. Bewust toch aan de kabel rijden: ros2 param set /battery_monitor
block_when_charging false.
"""
import fcntl
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from wave_rover_base.battery import percentage

I2C_SLAVE = 0x0703
REG_SHUNT = 0x01
REG_BUS = 0x02
SHUNT_OHM = 0.01
# Boven deze spanning meet de UPS de adapter, niet de batterij.
MAX_PACK_VOLTAGE = 12.8
# Zo lang na een meting boven MAX_PACK_VOLTAGE blijft het 'geen batterij' (zie publish_ups).
ADAPTER_HOLD = 30.0
UNPLUGGED_CURRENT = -0.15  # A; daaronder ontlaadt de batterij: de kabel is eruit


class Ina219:

    def __init__(self, bus, address):
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, address)

    def read_register(self, register):
        os.write(self.fd, bytes([register]))
        high, low = os.read(self.fd, 2)
        return (high << 8) | low

    def voltage(self):
        return (self.read_register(REG_BUS) >> 3) * 0.004

    def current(self):
        raw = self.read_register(REG_SHUNT)
        if raw > 32767:
            raw -= 65536
        return raw * 10e-6 / SHUNT_OHM


class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('battery_monitor')
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_address', 0x41)
        self.declare_parameter('warn_percentage', 30.0)
        self.warn_pct = self.get_parameter('warn_percentage').value

        self.ina = Ina219(self.get_parameter('i2c_bus').value,
                          self.get_parameter('i2c_address').value)
        self.ups_pub = self.create_publisher(BatteryState, 'ups/battery', 10)
        self.chassis_pub = self.create_publisher(BatteryState, 'chassis/battery', 10)
        self.create_subscription(BatteryState, 'battery', self.on_chassis, 10)
        self.create_timer(2.0, self.publish_ups)
        self.declare_parameter('block_when_charging', True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.lock_pub = self.create_publisher(Bool, 'charging_lock', latched)
        self.unplugged_count = 0
        self.locked = None
        self.warned = {'jetson': False, 'chassis': False}
        self.adapter_seen = None  # laatste meting van de adapter (geen batterij), monotonic

    def check_low(self, name, pct, discharging):
        low = discharging and pct < self.warn_pct
        if low and not self.warned[name]:
            self.get_logger().warn(f'Batterij {name} bijna leeg: {pct:.0f}%')
        self.warned[name] = low

    def update_charging_lock(self, current):
        self.unplugged_count = self.unplugged_count + 1 if current < UNPLUGGED_CURRENT else 0
        locked = self.get_parameter('block_when_charging').value and self.unplugged_count < 2
        if locked != self.locked:
            self.get_logger().info('Laadkabel in: rijden geblokkeerd' if locked
                                   else 'Op batterij: rijden mag')
        self.locked = locked
        self.lock_pub.publish(Bool(data=locked))

    def publish_ups(self):
        try:
            voltage, current = self.ina.voltage(), self.ina.current()
        except OSError as e:
            self.get_logger().error(f'UPS niet leesbaar via I2C: {e}', throttle_duration_sec=60)
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = voltage
        msg.current = current
        # Een 3S-pakket haalt hoogstens ~12,6 V. Meet de UPS meer, dan meet hij de adapter:
        # geen batterij geplaatst, of de beveiliging heeft ze losgekoppeld (bv. één cel te leeg).
        # Zonder batterij schommelt die meting (12,4-13,6 V): na een meting boven
        # MAX_PACK_VOLTAGE nog ADAPTER_HOLD s 'geen batterij', anders wisselde de app telkens
        # tussen een percentage en 'geen batterij' (30-10-2026).
        now = time.monotonic()
        if voltage >= MAX_PACK_VOLTAGE:
            self.adapter_seen = now
        recent_adapter = self.adapter_seen is not None and now - self.adapter_seen < ADAPTER_HOLD
        msg.present = 5.0 < voltage < MAX_PACK_VOLTAGE and not recent_adapter
        msg.percentage = percentage(voltage) / 100.0 if msg.present else float('nan')
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        # Zonder batterij schommelt de meting (12,4-13,6 V); niet elke dip is betrouwbaar te
        # herkennen, maar boven MAX_PACK_VOLTAGE is het zeker de adapter.
        if not msg.present and voltage >= MAX_PACK_VOLTAGE:
            self.get_logger().warn(
                f'UPS meet {voltage:.2f} V (adapter): geen batterij of batterij losgekoppeld',
                throttle_duration_sec=300)
        if current > 0.05:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif current < -0.05:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        self.ups_pub.publish(msg)
        self.update_charging_lock(current)
        if msg.present:
            self.check_low('jetson', msg.percentage * 100, current < -0.05)

    def on_chassis(self, msg):
        msg.percentage = percentage(msg.voltage) / 100.0 if msg.present else float('nan')
        self.chassis_pub.publish(msg)
        if msg.present:
            self.check_low('chassis', msg.percentage * 100, True)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
