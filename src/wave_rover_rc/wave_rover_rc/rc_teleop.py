"""Zet /joy van de RC-zender om naar /cmd_vel_rc.

- Arm-schakelaar uit: er wordt niets gepubliceerd, zodat Nav2 of het dashboard
  via twist_mux mogen rijden. Bij het uitzetten volgt eerst één stopcommando.
- Arm-schakelaar aan: rijden met de rechterstick.
- Snelheidsschakelaar (3 standen): traag / normaal / snel.

Assen zijn indexen in Joy.axes (kanaal - 1).
"""
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


class RcTeleop(Node):

    def __init__(self):
        super().__init__('rc_teleop')

        self.declare_parameter('axis_linear', 2)
        self.declare_parameter('axis_angular', 1)
        self.declare_parameter('axis_arm', 4)
        self.declare_parameter('axis_speed', 5)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', True)
        self.declare_parameter('deadband', 0.05)
        self.declare_parameter('max_linear', [0.15, 0.3, 0.6])
        self.declare_parameter('max_angular', [1.5, 3.0, 5.0])

        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.axis_arm = self.get_parameter('axis_arm').value
        self.axis_speed = self.get_parameter('axis_speed').value
        self.sign_linear = -1.0 if self.get_parameter('invert_linear').value else 1.0
        self.sign_angular = -1.0 if self.get_parameter('invert_angular').value else 1.0
        self.deadband = self.get_parameter('deadband').value
        self.max_linear = list(self.get_parameter('max_linear').value)
        self.max_angular = list(self.get_parameter('max_angular').value)

        self.armed = False
        self.speed_level = None
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_rc', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)

    def apply_deadband(self, value):
        return 0.0 if abs(value) < self.deadband else value

    def publish(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.pub.publish(msg)

    def on_joy(self, joy):
        axes = joy.axes
        armed = axes[self.axis_arm] > 0.5
        if armed != self.armed:
            self.get_logger().info('GEARMD: handmatig rijden' if armed else 'Ontwapend')
            if not armed:
                self.publish(0.0, 0.0)
            self.armed = armed
        if not armed:
            return

        speed = axes[self.axis_speed]
        level = 0 if speed < -0.5 else (2 if speed > 0.5 else 1)
        if level != self.speed_level:
            self.get_logger().info(f'Snelheid {level + 1}/3: max {self.max_linear[level]} m/s')
            self.speed_level = level

        linear = self.apply_deadband(axes[self.axis_linear]) * self.sign_linear
        angular = self.apply_deadband(axes[self.axis_angular]) * self.sign_angular
        self.publish(linear * self.max_linear[level], angular * self.max_angular[level])


def main(args=None):
    rclpy.init(args=args)
    node = RcTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
