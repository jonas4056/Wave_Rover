#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
import threading
import math

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class WaveRoverBridge(Node):
    def __init__(self):
        super().__init__('wave_rover_bridge')

        # Declare parameters
        self.declare_parameter('port', '/dev/esp32')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # Connect to serial port
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f'✅ Connected to {port} at {baud} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Failed to open serial port {port}: {e}')
            rclpy.shutdown()
            return
        # Enable continuous telemetry streaming from ESP32
        self.ser.write(b'{"T":131,"cmd":1}\r\n')
        self.get_logger().info("Sent continuous feedback command to ESP32 (T131).")
        qos = QoSProfile(depth=10)

        # Publishers
        self.battery_pub = self.create_publisher(Float32, '/battery', qos)
        self.temp_pub = self.create_publisher(Float32, '/temperature', qos)
        self.imu_pub = self.create_publisher(Imu, '/imu', qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber for velocity commands
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, qos)

        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def cmd_callback(self, msg: Twist):
        """Send CMD_SPEED_CTRL (T:1) for non-encoder WAVE ROVER control."""
        MAX_SPEED = 0.5          # firmware PWM full scale
        MAX_SPEED_ROS = 1.0      # teleop's m/s max

        scale = MAX_SPEED / MAX_SPEED_ROS

        # Differential drive mixing
        left = (msg.linear.x - msg.angular.z) * scale
        right = (msg.linear.x + msg.angular.z) * scale

        # Clamp to [-0.5, 0.5]
        left = max(-MAX_SPEED, min(MAX_SPEED, left))
        right = max(-MAX_SPEED, min(MAX_SPEED, right))

        # Send JSON
        cmd = {
            "T": 1,
            "L": round(left, 3),
            "R": round(right, 3)
        }
        try:
            self.ser.write((json.dumps(cmd) + '\n').encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Write error: {e}')

    def read_serial(self):
        """Continuously read and parse JSON data from serial."""
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line or not line.startswith('{') or not line.endswith('}'):
                    continue

                data = json.loads(line)
                if "T" in data and data["T"] == 1001:
                    self.publish_feedback(data)

            except json.JSONDecodeError:
                continue
            except serial.SerialException as e:
                self.get_logger().error(f"Serial port error: {e}")
                break

    def publish_feedback(self, data):
        """Publish IMU, battery, temperature, and odom + TF data."""

        # --- Battery voltage ---
        if "v" in data:
            msg = Float32()
            msg.data = float(data["v"])
            self.battery_pub.publish(msg)

        # --- Temperature ---
        if "temp" in data:
            msg = Float32()
            msg.data = float(data["temp"])
            self.temp_pub.publish(msg)

        # --- IMU data ---
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"

        if all(k in data for k in ("r", "p", "y")):
            roll = math.radians(float(data["r"]))
            pitch = math.radians(float(data["p"]))
            yaw = math.radians(float(data["y"]))
            q = quaternion_from_euler(roll, pitch, yaw)

            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]

        self.imu_pub.publish(imu_msg)

        # --- Odometry ---
        odom_msg = Odometry()
        odom_msg.header.stamp = imu_msg.header.stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position unknown → keep (0,0,0)
        odom_msg.pose.pose.orientation = imu_msg.orientation
        self.odom_pub.publish(odom_msg)

        # --- TF Broadcast (odom → base_link) ---
        t = TransformStamped()
        t.header.stamp = imu_msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = imu_msg.orientation

        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        """Clean up serial port on shutdown."""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaveRoverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
