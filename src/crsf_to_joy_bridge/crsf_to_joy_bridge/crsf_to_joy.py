import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from crsf_receiver_msg.msg import CRSFChannels16
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class CRSFToJoyNode(Node):
    def __init__(self):
        super().__init__('crsf_to_joy_node')

        # Match publisher QoS: Best Effort and Keep Last
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            CRSFChannels16,
            '/rc/channels',
            self.channels_callback,
            qos_profile
        )
        
        self.pub = self.create_publisher(Joy, '/joy', 10)

    def channels_callback(self, msg):
        joy_msg = Joy()

        # Normalize values from CRSF (midpoint ~992, range ±820)
        def norm(val):
            return (val - 992) / 820.0

        # Map channel 1 -> angular (steering), channel 2 -> linear (forward/backward)
        joy_msg.axes = [
            norm(msg.ch1),  # Steering stick
            norm(msg.ch2)   # Throttle stick
        ]
        joy_msg.buttons = []

        self.pub.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CRSFToJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
