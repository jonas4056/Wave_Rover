"""Bridge tussen ROS 2 en het ESP32 general driver board van de Wave Rover.

- /cmd_vel (TwistStamped) wordt omgezet naar wielcommando's `{"T":1,"L":..,"R":..}`.
  De motoren hebben geen encoders: dit is open-loop sturing.
- Komt er langer dan `cmd_timeout` geen /cmd_vel binnen, dan worden de motoren gestopt.
- De IMU (`T:126` → antwoord `T:1002`) en de chassisfeedback (`T:130` → `T:1001`)
  worden periodiek opgevraagd en gepubliceerd als /imu/data_raw en /battery.
"""
import json
import math
import threading

import rclpy
import serial
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu

G = 9.80665


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Esp32Bridge(Node):

    def __init__(self):
        super().__init__('esp32_bridge')

        self.declare_parameter('port', '/dev/esp32')
        self.declare_parameter('baud', 115200)
        # Effectieve wielafstand (m). Bij skid-steer schuiven de wielen zijdelings bij het
        # draaien; daardoor is meer verschil tussen links en rechts nodig dan de echte
        # wielafstand (0,12 m) doet vermoeden.
        self.declare_parameter('track_width', 0.3)
        # Snelheid (m/s) die overeenkomt met wielwaarde `max_wheel_cmd`; te kalibreren in fase 2.
        self.declare_parameter('speed_at_max_cmd', 1.25)
        self.declare_parameter('max_wheel_cmd', 0.5)
        # Kleinste wielwaarde waarbij de motoren op gang komen; kleinere commando's
        # worden lineair geschaald naar [min_wheel_cmd, max_wheel_cmd].
        self.declare_parameter('min_wheel_cmd', 0.1)
        # Skid-steer heeft in bochten meer kracht nodig (de wielen schuiven zijdelings):
        # het minimum loopt op tot deze waarde naarmate de bocht scherper is.
        self.declare_parameter('min_turn_wheel_cmd', 0.25)
        # Minimum bij draaien op de plaats: lager, anders draait de robot te snel voor lidar/IMU.
        self.declare_parameter('min_spin_wheel_cmd', 0.17)
        # Correctie van de draaisnelheid, gemeten met scripts/calibrate_motion.py: de robot
        # draait zonder correctie ~60% van wat gevraagd wordt op de plaats. In bochten draait hij
        # door het bochtduwtje (min_turn_wheel_cmd) al voldoende of te veel.
        self.declare_parameter('spin_gain', 1.6)
        self.declare_parameter('curve_gain', 0.9)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('imu_rate', 20.0)
        self.declare_parameter('imu_frame', 'imu_link')

        self.track_width = self.get_parameter('track_width').value
        self.speed_at_max_cmd = self.get_parameter('speed_at_max_cmd').value
        self.max_wheel_cmd = self.get_parameter('max_wheel_cmd').value
        self.min_wheel_cmd = self.get_parameter('min_wheel_cmd').value
        self.min_turn_wheel_cmd = self.get_parameter('min_turn_wheel_cmd').value
        self.min_spin_wheel_cmd = self.get_parameter('min_spin_wheel_cmd').value
        self.spin_gain = self.get_parameter('spin_gain').value
        self.curve_gain = self.get_parameter('curve_gain').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.imu_frame = self.get_parameter('imu_frame').value

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.write_lock = threading.Lock()
        self.get_logger().info(f'Verbonden met ESP32 op {port} ({baud} baud)')

        # Continue feedbackstroom uit: we vragen zelf op, zodat de seriële lijn niet overloopt.
        self.send({'T': 131, 'cmd': 0})
        self.stop_motors()

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', qos_profile_sensor_data)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.create_subscription(TwistStamped, 'cmd_vel', self.on_cmd_vel, 10)

        self.last_cmd_time = None
        self.create_timer(1.0 / self.get_parameter('imu_rate').value,
                          lambda: self.send({'T': 126}))
        self.create_timer(1.0, lambda: self.send({'T': 130}))
        self.create_timer(0.1, self.check_cmd_timeout)

        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def send(self, msg):
        line = json.dumps(msg, separators=(',', ':')) + '\n'
        with self.write_lock:
            self.ser.write(line.encode())

    def stop_motors(self):
        self.send({'T': 1, 'L': 0, 'R': 0})

    def on_cmd_vel(self, stamped):
        msg = stamped.twist
        half = self.track_width / 2.0
        wz = msg.angular.z * self.angular_gain(msg.linear.x, msg.angular.z * half)
        left = msg.linear.x - wz * half
        right = msg.linear.x + wz * half

        left, right = self.to_wheel_cmds(left, right)
        self.send({'T': 1, 'L': round(left, 3), 'R': round(right, 3)})
        self.last_cmd_time = self.get_clock().now()

    def angular_gain(self, forward, turn):
        """Vloeiend van `curve_gain` (vooral vooruit) naar `spin_gain` (op de plaats draaien)."""
        total = abs(forward) + abs(turn)
        if total < 1e-6:
            return 1.0
        # Kwadratisch: de grote correctie geldt pas bij (bijna) zuiver draaien op de plaats.
        spin = (abs(turn) / total) ** 2
        return self.curve_gain * (1.0 - spin) + self.spin_gain * spin

    def to_wheel_cmds(self, left, right):
        """Wielsnelheden (m/s) -> wielwaarden voor T:1.

        Het snelste wiel krijgt minstens een minimum om de motoren op gang te krijgen:
        `min_wheel_cmd` rechtdoor, oplopend tot `min_turn_wheel_cmd` in een scherpe bocht
        (één wiel stil) en `min_spin_wheel_cmd` bij draaien op de plaats. Beide wielen worden met dezelfde factor geschaald, zodat de
        verhouding links/rechts, en dus de bochtstraal, behouden blijft.
        """
        scale = self.max_wheel_cmd / self.speed_at_max_cmd
        left, right = left * scale, right * scale
        fastest = max(abs(left), abs(right))
        if fastest < 1e-3:
            return 0.0, 0.0
        # 0 = rechtdoor, 1 = één wiel staat stil, 2 = draaien op de plaats.
        turn = abs(left - right) / fastest
        if turn <= 1.0:
            minimum = self.min_wheel_cmd + (self.min_turn_wheel_cmd - self.min_wheel_cmd) * turn
        else:
            minimum = (self.min_turn_wheel_cmd
                       + (self.min_spin_wheel_cmd - self.min_turn_wheel_cmd) * min(1.0, turn - 1.0))
        fastest_cmd = min(self.max_wheel_cmd, fastest)
        fastest_cmd = minimum + (self.max_wheel_cmd - minimum) * fastest_cmd / self.max_wheel_cmd
        factor = fastest_cmd / fastest
        return left * factor, right * factor

    def check_cmd_timeout(self):
        if self.last_cmd_time is None:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age > self.cmd_timeout:
            self.stop_motors()
            self.last_cmd_time = None

    def read_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except serial.SerialException as e:
                self.get_logger().error(f'Seriële fout: {e}')
                return
            if not line.startswith('{'):
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            if data.get('T') == 1002:
                self.publish_imu(data)
            elif data.get('T') == 1001:
                self.publish_battery(data)

    def publish_imu(self, d):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame

        q = quaternion_from_rpy(math.radians(d['r']), math.radians(d['p']),
                                math.radians(d['y']))
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        # Yaw komt van de magnetometer en is binnenshuis onbetrouwbaar: grote onzekerheid.
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0]

        # Gyroscoop in °/s, versnelling in mg.
        msg.angular_velocity.x = math.radians(d['gx'])
        msg.angular_velocity.y = math.radians(d['gy'])
        msg.angular_velocity.z = math.radians(d['gz'])
        # Gemeten ruis bij stilstand: ±3 °/s met pieken tot 16 °/s -> std ~0.1 rad/s.
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.linear_acceleration.x = d['ax'] / 1000.0 * G
        msg.linear_acceleration.y = d['ay'] / 1000.0 * G
        msg.linear_acceleration.z = d['az'] / 1000.0 * G
        msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]

        self.imu_pub.publish(msg)

    def publish_battery(self, d):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(d.get('v', math.nan))
        msg.present = msg.voltage > 5.0
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery_pub.publish(msg)

    def destroy_node(self):
        try:
            self.stop_motors()
            self.ser.close()
        except serial.SerialException:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
