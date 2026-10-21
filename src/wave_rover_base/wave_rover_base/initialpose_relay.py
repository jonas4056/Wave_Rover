"""Geeft de startpositie uit RViz door aan AMCL, met tijdstempel 0 ("laatst bekende TF").

RViz op de pc stempelt /initialpose met het huidige tijdstip. AMCL zoekt de TF dan
exact op dat moment op, en dat ligt bijna altijd net na de laatste odometrie
(30 Hz) -> "extrapolation into the future" en de startpositie wordt genegeerd.

RViz publiceert daarom op /initialpose_rviz; deze node stuurt het door naar /initialpose.
"""
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Time


class InitialPoseRelay(Node):

    def __init__(self):
        super().__init__('initialpose_relay')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose_rviz', self.on_pose, 10)

    def on_pose(self, msg):
        msg.header.stamp = Time().to_msg()
        self.pub.publish(msg)
        p = msg.pose.pose.position
        self.get_logger().info(f'Startpositie doorgegeven: x={p.x:.2f} y={p.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
