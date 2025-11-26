import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SimpleJoyToCmd(Node):
    def __init__(self):
        super().__init__('simple_joy_to_cmd')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # match CRSF bridge QoS
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to /joy
        self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile)

        # Publisher for /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Scaling factors
        self.linear_scale = 1.0
        self.angular_scale = 1.0

        self.get_logger().info('SimpleJoyToCmd started - mapping /joy to /cmd_vel')

    def joy_callback(self, joy):
        twist = Twist()

        if len(joy.axes) >= 2:
            # Left stick forward/back → linear.x
            twist.linear.x = self.linear_scale * joy.axes[0]
            # Right stick left/right → angular.z
            twist.angular.z = -self.angular_scale * joy.axes[1]

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleJoyToCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down cleanly...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
