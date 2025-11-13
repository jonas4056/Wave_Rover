import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotCommandHandler(Node):
    def __init__(self):
        super().__init__('robot_command_handler')
        self.subscription = self.create_subscription(
            String,
            'yahboom_command',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("🤖 Robot Command Handler Ready")

    def listener_callback(self, msg):
        cmd = msg.data
        self.get_logger().info(f"🎤 Received voice command: {cmd}")

        if cmd == "robot_stop":
            self.stop_robot()
        elif cmd == "robot_explore":
            self.start_exploration()
        elif cmd == "wake_up":
            self.say("Hello! Ready to assist.")
        elif cmd == "volume_up":
            self.say("Increasing volume.")
        elif cmd == "volume_down":
            self.say("Decreasing volume.")
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    def stop_robot(self):
        self.get_logger().info("🛑 Stopping robot motors (simulated).")
        # TODO: publish to /cmd_vel = 0, or stop your motor topic

    def start_exploration(self):
        self.get_logger().info("🗺️ Starting exploration mode (simulated).")
        # TODO: call your exploration start service or launch command

    def say(self, text):
        # This is where you can integrate Yahboom speaker output later.
        self.get_logger().info(f"🔊 Speaking: {text}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotCommandHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
