"""Leest een Crossfire-ontvanger (CRSF) op een UART en publiceert /joy.

Elk van de 16 kanalen wordt een as in Joy.axes, genormaliseerd naar -1…1.
Kanalen 5–16 (schakelaars) worden ook als knop gepubliceerd: 1 als de as > 0,5.

Valt de verbinding weg (link quality 0 of geen kanaaldata meer), dan wordt er
geen /joy meer gepubliceerd. rc_teleop en twist_mux stoppen de robot dan vanzelf.
"""
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

CRSF_SYNC = 0xC8
TYPE_LINK_STATISTICS = 0x14
TYPE_RC_CHANNELS = 0x16

CRSF_MID = 992
CRSF_HALF_RANGE = 820


def _crc8_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return bytes(table)


CRC8_TABLE = _crc8_table()


def crc8_dvb_s2(data):
    # Met tabel: 150 pakketten/s bit per bit narekenen kostte merkbaar CPU.
    crc = 0
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc


def unpack_channels(payload):
    """16 kanalen van 11 bits, little-endian aaneengeschoven."""
    bits = int.from_bytes(payload[:22], 'little')
    return [(bits >> (11 * i)) & 0x7FF for i in range(16)]


class CrsfJoy(Node):

    def __init__(self):
        super().__init__('crsf_joy')

        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 420000)
        self.declare_parameter('link_timeout', 0.3)
        # De ontvanger stuurt 150 pakketten/s; 50 Hz is ruim genoeg om te rijden en spaart CPU
        # in crsf_joy, rc_teleop, e_stop en twist_mux. 0 = elk pakket doorsturen.
        self.declare_parameter('publish_rate', 50.0)
        rate = self.get_parameter('publish_rate').value
        # 25% marge: pakketten komen elke 6,7 ms; zonder marge viel het ritme terug naar ~37 Hz.
        self.min_period = 0.75 / rate if rate > 0 else 0.0
        self.last_publish = 0.0

        self.link_timeout = self.get_parameter('link_timeout').value
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.get_logger().info(f'CRSF-ontvanger op {port} ({baud} baud)')

        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.connected_pub = self.create_publisher(Bool, 'rc/connected', 10)

        self.link_quality = 0
        self.last_channels_time = 0.0
        self.create_timer(0.2, self.publish_connected)

        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def connected(self):
        fresh = time.monotonic() - self.last_channels_time < self.link_timeout
        return fresh and self.link_quality > 0

    def publish_connected(self):
        self.connected_pub.publish(Bool(data=self.connected()))

    def read_loop(self):
        buf = bytearray()
        while rclpy.ok():
            try:
                # Lezen wat er al binnen is (minstens 1 byte). read(256) wachtte tot de
                # time-out van 50 ms: dan kwamen de pakketten per ~7 tegelijk en 50 ms te laat.
                buf += self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                self.get_logger().error(f'Seriële fout: {e}')
                return

            while len(buf) >= 2:
                if buf[0] != CRSF_SYNC:
                    del buf[0]
                    continue
                length = buf[1]
                if length < 2 or length > 62:
                    del buf[0]
                    continue
                if len(buf) < length + 2:
                    break
                frame = bytes(buf[2:length + 2])
                del buf[:length + 2]
                if crc8_dvb_s2(frame[:-1]) == frame[-1]:
                    self.handle_frame(frame[0], frame[1:-1])

    def handle_frame(self, frame_type, payload):
        if frame_type == TYPE_LINK_STATISTICS and len(payload) >= 3:
            self.link_quality = payload[2]
        elif frame_type == TYPE_RC_CHANNELS and len(payload) >= 22:
            now = time.monotonic()
            self.last_channels_time = now
            if not self.connected() or now - self.last_publish < self.min_period:
                return
            self.last_publish = now
            axes = [max(-1.0, min(1.0, (ch - CRSF_MID) / CRSF_HALF_RANGE))
                    for ch in unpack_channels(payload)]
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = axes
            msg.buttons = [1 if a > 0.5 else 0 for a in axes[4:]]
            self.joy_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CrsfJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
