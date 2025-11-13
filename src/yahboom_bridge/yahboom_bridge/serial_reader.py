import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import binascii

class YahboomSerialReader(Node):
    def __init__(self):
        super().__init__('yahboom_serial_reader')
        self.declare_parameter('port', '/dev/ttyCH341USB0')
        self.declare_parameter('baudrate', 9600)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f'✅ Connected to Yahboom board at {self.port}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to open {self.port}: {e}')
            rclpy.shutdown()
            return

        self.publisher = self.create_publisher(String, 'yahboom_command', 10)
        self.timer = self.create_timer(0.05, self.read_serial)

        # Example mapping table (extend this from the Yahboom Protocol doc)
        self.command_map = {
            0x09: "Open light",
            0x0A: "Close light",
            0x10: "Robot stop",
            0x11: "Robot start exploring",
            0x67: "I am ready"
        }

        self.buffer = bytearray()

    def read_serial(self):
        if self.ser.in_waiting:
            data = self.ser.read(self.ser.in_waiting)
            self.buffer.extend(data)

            # parse full frames
            while len(self.buffer) >= 5:
                if self.buffer[0] == 0xAA and self.buffer[1] == 0xFF and self.buffer[4] == 0x55:
                    frame = self.buffer[:5]
                    self.buffer = self.buffer[5:]

                    cmd_type = frame[2]
                    cmd_id = frame[3]
                    hex_frame = ' '.join(f'{b:02X}' for b in frame)

                    # decode known commands
                    cmd_text = self.command_map.get(cmd_id, f"Unknown (0x{cmd_id:02X})")

                    self.get_logger().info(f'📦 Frame: {hex_frame} → Command: {cmd_text}')

                    msg = String()
                    msg.data = cmd_text
                    self.publisher.publish(msg)
                else:
                    # discard one byte until we find a header
                    self.buffer.pop(0)

def main(args=None):
    rclpy.init(args=args)
    node = YahboomSerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
