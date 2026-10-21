"""Yahboom-spraakmodule (ASR-TTS, CI1302) op /dev/yahboom.

Protocol (115200 baud, frames van 5 bytes):
- herkend commando:     AA 55 <type> <id> FB  (zie COMMANDS)
- zin laten afspelen:   AA 55 FF <id> FB      (zie PHRASES)

De module kent alleen vaste zinnen uit de firmware; vrije tekst kan niet.

Topics:
- /voice/command (String): naam van elk herkend commando, bv. "GO-AHEAD"
- /voice/say (String): naam uit PHRASES om af te spelen, bv. "I-AM-READY"
- /e_stop (Bool): "robot stop"/"stop" -> true (noodstop via twist_mux), "go ahead" -> false

Batterijstatus als kleur ("this" = chassis, "there" = Jetson):
groen >= 50%, geel 20-50%, rood < 20%. Wordt uitgesproken na "display power" en
automatisch elke `low_battery_repeat` seconden zolang een batterij bijna leeg is.
"""
import threading
import time

import rclpy
import serial
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

# (type, id) zoals de module ze stuurt -> naam uit de protocollijst van Yahboom.
COMMANDS = {
    (0x01, 0x00): 'WELCOME', (0x02, 0x00): 'BYE', (0x03, 0x00): 'HI-YAHBOOM',
    (0x04, 0x00): 'VOLUME-UP', (0x05, 0x00): 'VOLUME-DOWN', (0x06, 0x00): 'MAX-VOLUME',
    (0x07, 0x00): 'MID-VOLUME', (0x08, 0x00): 'MINI-VOLUME',
    (0x00, 0x01): 'ROBOT-STOP', (0x00, 0x02): 'STOP', (0x00, 0x03): 'ROBOT-SLEEP',
    (0x00, 0x04): 'GO-AHEAD', (0x00, 0x05): 'BACK', (0x00, 0x06): 'TURN-LEFT',
    (0x00, 0x07): 'TURN-RIGHT', (0x00, 0x08): 'ENTER-A-MODE', (0x00, 0x09): 'ENTER-B-MODE',
    (0x00, 0x12): 'DISPLAY-POWER',
    (0x00, 0x13): 'GO-TO-THE-POINT-A', (0x00, 0x14): 'GO-TO-THE-POINT-B',
    (0x00, 0x15): 'GO-TO-THE-POINT-C', (0x00, 0x20): 'GO-TO-THE-POINT-D',
    (0x00, 0x21): 'BACK-TO-ORIGINAL',
    (0x00, 0x1B): 'MODE-ON', (0x00, 0x1C): 'MODE-OFF', (0x00, 0x26): 'WARNING',
}

# Zinnen die de host kan laten afspelen (AA 55 FF <id> FB).
PHRASES = {
    'THIS-IS-RED': 0x5F, 'THIS-IS-BLUE': 0x60, 'THIS-IS-GREEN': 0x61, 'THIS-IS-YELLOW': 0x62,
    'THERE-IS-YELLOW': 0x63, 'THERE-IS-GREEN': 0x64, 'THERE-IS-BLUE': 0x65, 'THERE-IS-RED': 0x66,
    'I-AM-READY': 0x67,
}

COLOR_PHRASES = {
    'chassis': {'green': 'THIS-IS-GREEN', 'yellow': 'THIS-IS-YELLOW', 'red': 'THIS-IS-RED'},
    'jetson': {'green': 'THERE-IS-GREEN', 'yellow': 'THERE-IS-YELLOW', 'red': 'THERE-IS-RED'},
}

# Pauze tussen twee zinnen, zodat de module de vorige kan uitspreken.
PHRASE_GAP = 2.0


def color_for(pct):
    if pct >= 50.0:
        return 'green'
    return 'yellow' if pct >= 20.0 else 'red'


class YahboomVoice(Node):

    def __init__(self):
        super().__init__('yahboom_voice')
        self.declare_parameter('port', '/dev/yahboom')
        self.declare_parameter('low_battery_repeat', 300.0)

        self.ser = serial.Serial(self.get_parameter('port').value, 115200, timeout=0.1)
        self.write_lock = threading.Lock()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.command_pub = self.create_publisher(String, 'voice/command', 10)
        self.estop_pub = self.create_publisher(Bool, 'e_stop', latched)
        self.create_subscription(String, 'voice/say', lambda m: self.say(m.data), 10)

        self.battery_pct = {}
        self.create_subscription(BatteryState, 'chassis/battery',
                                 lambda m: self.on_battery('chassis', m), 10)
        self.create_subscription(BatteryState, 'ups/battery',
                                 lambda m: self.on_battery('jetson', m), 10)
        self.last_low_alarm = 0.0
        self.create_timer(10.0, self.check_low_battery)

        self.cancel_nav = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')

        self.estop_pub.publish(Bool(data=False))
        threading.Thread(target=self.read_loop, daemon=True).start()
        self.say('I-AM-READY')
        self.get_logger().info('Yahboom-spraakmodule klaar')

    # --- uitspreken -----------------------------------------------------------------

    def say(self, *names, delay=0.0):
        """Speelt een of meer zinnen af, met een pauze ertussen (in een aparte thread)."""
        def worker():
            time.sleep(delay)
            for i, name in enumerate(names):
                phrase_id = PHRASES.get(name)
                if phrase_id is None:
                    self.get_logger().warn(f'Onbekende zin: {name}')
                    continue
                if i:
                    time.sleep(PHRASE_GAP)
                with self.write_lock:
                    self.ser.write(bytes([0xAA, 0x55, 0xFF, phrase_id, 0xFB]))
        threading.Thread(target=worker, daemon=True).start()

    # --- batterij -------------------------------------------------------------------

    def on_battery(self, name, msg):
        if msg.present and msg.percentage == msg.percentage:  # niet NaN
            self.battery_pct[name] = msg.percentage * 100.0

    def battery_phrases(self, only_low=False):
        phrases = []
        for name in ('chassis', 'jetson'):
            pct = self.battery_pct.get(name)
            if pct is None:
                continue
            color = color_for(pct)
            if not only_low or color == 'red':
                phrases.append(COLOR_PHRASES[name][color])
        return phrases

    def check_low_battery(self):
        repeat = self.get_parameter('low_battery_repeat').value
        low = self.battery_phrases(only_low=True)
        if low and time.monotonic() - self.last_low_alarm > repeat:
            self.get_logger().warn(f'Batterij bijna leeg: {", ".join(low)}')
            self.say(*low)
            self.last_low_alarm = time.monotonic()

    # --- herkende commando's --------------------------------------------------------

    def read_loop(self):
        buf = b''
        while rclpy.ok():
            try:
                buf += self.ser.read(32)
            except serial.SerialException as e:
                self.get_logger().error(f'Seriële fout: {e}')
                return
            while True:
                start = buf.find(b'\xaa\x55')
                if start < 0:
                    buf = buf[-1:]
                    break
                if len(buf) - start < 5:
                    buf = buf[start:]
                    break
                frame, buf = buf[start:start + 5], buf[start + 5:]
                if frame[4] == 0xFB:
                    self.on_frame(frame[2], frame[3])

    def on_frame(self, kind, ident):
        name = COMMANDS.get((kind, ident), f'ONBEKEND-{kind:02X}-{ident:02X}')
        self.get_logger().info(f'Spraakcommando: {name}')
        self.command_pub.publish(String(data=name))

        if name in ('ROBOT-STOP', 'STOP'):
            self.estop_pub.publish(Bool(data=True))
            if self.cancel_nav.service_is_ready():
                self.cancel_nav.call_async(CancelGoal.Request())  # lege id = alle doelen
        elif name == 'GO-AHEAD':
            self.estop_pub.publish(Bool(data=False))
        elif name == 'DISPLAY-POWER':
            # De module zegt eerst zelf "ok the battery level is displayed".
            self.say(*self.battery_phrases(), delay=PHRASE_GAP + 1.0)


def main(args=None):
    rclpy.init(args=args)
    node = YahboomVoice()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
