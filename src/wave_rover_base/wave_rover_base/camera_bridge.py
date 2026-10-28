"""Brengt de CSI-camera naar ROS als /camera/image/compressed (JPEG).

De camera draait via NVIDIA Argus op de Jetson zelf (host/wave_rover_camera.sh): die
codeert beelden met de hardware-JPEG-encoder en stuurt ze over TCP naar 127.0.0.1:5600.
Deze node:
- maakt bij het starten `request_file` aan; de systemd-path-unit start daarop de camera,
  en bij het stoppen verwijdert hij het bestand weer (camera uit);
- knipt de TCP-stroom in losse JPEG-beelden (begin FFD8, einde FFD9) en publiceert ze
  zonder te decoderen, dus bijna zonder CPU.
"""
import os
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

SOI = b'\xff\xd8'
EOI = b'\xff\xd9'


class CameraBridge(Node):

    def __init__(self):
        super().__init__('camera_bridge')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 5600)
        self.declare_parameter('frame_id', 'camera_link')
        # 30 fps: vloeiend en beter voor beeldherkenning; 15 fps halveert de CPU van nvargus.
        self.declare_parameter('fps', 30)
        self.declare_parameter('request_file', '/home/ubuntu/wave_rover/run/camera_on')
        self.address = (self.get_parameter('host').value, self.get_parameter('port').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.request_file = self.get_parameter('request_file').value

        self.pub = self.create_publisher(CompressedImage, 'camera/image/compressed',
                                         qos_profile_sensor_data)
        os.makedirs(os.path.dirname(self.request_file), exist_ok=True)
        with open(self.request_file, 'w') as f:
            f.write(f"fps={self.get_parameter('fps').value}\n")
        threading.Thread(target=self.receive_loop, daemon=True).start()

    def receive_loop(self):
        while rclpy.ok():
            try:
                with socket.create_connection(self.address, timeout=5) as sock:
                    self.get_logger().info(f'Verbonden met camera op {self.address[0]}:{self.address[1]}')
                    self.read_frames(sock)
            except OSError:
                time.sleep(1.0)  # camera start nog op (Argus heeft enkele seconden nodig)

    def read_frames(self, sock):
        buf = b''
        while rclpy.ok():
            chunk = sock.recv(65536)
            if not chunk:
                return
            buf += chunk
            while True:
                start = buf.find(SOI)
                if start < 0:
                    buf = b''
                    break
                end = buf.find(EOI, start + 2)
                if end < 0:
                    buf = buf[start:]
                    break
                self.publish(buf[start:end + 2])
                buf = buf[end + 2:]

    def publish(self, jpeg):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = jpeg
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            os.remove(self.request_file)  # camera op de host stopt
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
