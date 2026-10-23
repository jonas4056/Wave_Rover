"""Noodstop met één eigenaar.

- Enige publisher van /e_stop (std_msgs/Bool, latched): de lock in twist_mux
  (prioriteit 255) die alle rijcommando's blokkeert, ook die van de RC-zender.
- Service /e_stop/set (std_srvs/SetBool): true = noodstop, false = vrijgeven.
  Gebruikt door het dashboard en de spraakmodule.
- RC-zender: schakelaar op `release_axis` (kanaal 8) omzetten naar boven geeft de
  noodstop vrij, zodat je niet vastzit als de iPhone leeg of buiten bereik is.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class EStop(Node):

    def __init__(self):
        super().__init__('e_stop')
        self.declare_parameter('release_axis', 7)
        self.release_axis = self.get_parameter('release_axis').value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Bool, 'e_stop', latched)
        self.create_service(SetBool, 'e_stop/set', self.on_set)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)

        self.active = False
        self.switch_high = None
        self.publish()

    def publish(self):
        self.pub.publish(Bool(data=self.active))

    def set_active(self, active, source):
        if active != self.active:
            self.get_logger().warn(f'Noodstop {"AAN" if active else "vrijgegeven"} ({source})')
        self.active = active
        self.publish()

    def on_set(self, request, response):
        self.set_active(request.data, 'service')
        response.success = True
        response.message = 'noodstop aan' if self.active else 'noodstop vrijgegeven'
        return response

    def on_joy(self, msg):
        if len(msg.axes) <= self.release_axis:
            return
        high = msg.axes[self.release_axis] > 0.5
        # Alleen bij omzetten naar boven, zodat een schakelaar die al boven staat de
        # noodstop niet meteen weer opheft.
        if high and self.switch_high is False and self.active:
            self.set_active(False, 'RC-zender kanaal 8')
        self.switch_high = high


def main(args=None):
    rclpy.init(args=args)
    node = EStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
