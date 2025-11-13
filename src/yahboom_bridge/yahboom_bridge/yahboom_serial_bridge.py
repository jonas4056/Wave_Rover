import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, binascii

COMMAND_MAP = {
    "aa550300fb": "wake_up",
    "aa550400fb": "volume_up",
    "aa550500fb": "volume_down",
    "aa550001fb": "robot_stop",
    "aa550014fb": "robot_explore",
}

class YahboomBridge(Node):
    def __init__(self):
        super().__init__('yahboom_bridge')
        self.publisher_ = self.create_publisher(String, 'yahboom_command', 10)
        self.port = '/dev/ttyCH341USB0'
        try:
            self.serial = serial.Serial(self.port, 115200, timeout=0.2)
            self.get_logger().info(f"✅ Yahboom bridge connected at {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Could not open {self.port}: {e}")
            raise SystemExit

        self.timer = self.create_timer(0.1, self.read_loop)

    def read_loop(self):
        data = self.serial.read(5)
        if len(data) == 5:
            hexed = binascii.hexlify(data).decode()
            cmd = COMMAND_MAP.get(hexed, None)
            if cmd:
                msg = String()
                msg.data = cmd
                self.publisher_.publish(msg)
                self.get_logger().info(f"📦 {hexed} → {cmd}")
            else:
                self.get_logger().info(f"❔ Unmapped: {hexed}")

def main(args=None):
    rclpy.init(args=args)
    node = YahboomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
