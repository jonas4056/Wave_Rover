"""Brengt de CSI-camera naar ROS als /camera/image/compressed (JPEG).

De camera draait via NVIDIA Argus op de Jetson zelf (host/wave_rover_camera.sh): die
codeert beelden met de hardware-JPEG-encoder en stuurt ze over TCP naar 127.0.0.1:5600.
Deze node:
- maakt bij het starten `request_file` aan; de systemd-path-unit start daarop de camera,
  en bij het stoppen verwijdert hij het bestand weer (camera uit);
- knipt de TCP-stroom in losse JPEG-beelden (begin FFD8, einde FFD9) en publiceert ze
  zonder te decoderen, dus bijna zonder CPU;
- publiceert ook camera/image_low/compressed met een lager ritme (`low_rate`), voor
  beeldherkenning die niet elk beeld nodig heeft. Een topic zonder luisteraars slaat hij over;
- publiceert elke seconde het aantal beelden op camera/frames (voor de health-controle,
  zodat die niet de hele beeldstroom hoeft te ontvangen);
- geeft het nieuwste voorbeeldbeeld (640x360, aparte stroom op `preview_port`) via HTTP
  (`http_port`, /latest.jpg?after=<nr>) aan het dashboard.
  Het dashboard vraagt pas een volgend beeld als het vorige op het scherm staat, dus er kan
  geen achterstand ontstaan. Via rosbridge liep die bij een trage gsm op tot vele seconden.
"""
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt32

SOI = b'\xff\xd8'
EOI = b'\xff\xd9'


class CameraBridge(Node):

    def __init__(self):
        super().__init__('camera_bridge')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 5600)
        self.declare_parameter('preview_port', 5601)
        self.declare_parameter('frame_id', 'camera_link')
        # 30 fps: vloeiend en beter voor beeldherkenning; 15 fps halveert de CPU van nvargus.
        self.declare_parameter('fps', 30)
        self.declare_parameter('low_rate', 10.0)
        self.declare_parameter('http_port', 8081)
        self.declare_parameter('request_file', '/home/ubuntu/wave_rover/run/camera_on')
        fps = self.get_parameter('fps').value
        self.low_every = max(1, round(fps / self.get_parameter('low_rate').value))
        self.frame_count = 0
        self.frames_reported = 0
        host = self.get_parameter('host').value
        self.address = (host, self.get_parameter('port').value)
        self.preview_address = (host, self.get_parameter('preview_port').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.request_file = self.get_parameter('request_file').value

        self.pub = self.create_publisher(CompressedImage, 'camera/image/compressed',
                                         qos_profile_sensor_data)
        self.low_pub = self.create_publisher(CompressedImage, 'camera/image_low/compressed',
                                             qos_profile_sensor_data)
        self.frames_pub = self.create_publisher(UInt32, 'camera/frames', 10)
        self.create_timer(1.0, self.publish_frames)
        self.frame_ready = threading.Condition()
        self.latest_jpeg = None
        self.latest_number = 0
        self.http = ThreadingHTTPServer(('', self.get_parameter('http_port').value),
                                        self.make_handler())
        self.http.daemon_threads = True
        threading.Thread(target=self.http.serve_forever, daemon=True).start()

        os.makedirs(os.path.dirname(self.request_file), exist_ok=True)
        with open(self.request_file, 'w') as f:
            f.write(f'fps={fps}\n')
        threading.Thread(target=self.receive_loop, args=(self.address, self.publish),
                         daemon=True).start()
        threading.Thread(target=self.receive_loop,
                         args=(self.preview_address, self.set_preview), daemon=True).start()

    def receive_loop(self, address, handle):
        while rclpy.ok():
            try:
                with socket.create_connection(address, timeout=5) as sock:
                    self.get_logger().info(f'Verbonden met camera op {address[0]}:{address[1]}')
                    self.read_frames(sock, handle)
            except OSError:
                time.sleep(1.0)  # camera start nog op (Argus heeft enkele seconden nodig)

    def read_frames(self, sock, handle):
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
                handle(buf[start:end + 2])
                buf = buf[end + 2:]

    def publish_frames(self):
        # Alleen bij nieuwe beelden: blijft de camera hangen, dan merkt de health-controle het.
        if self.frame_count != self.frames_reported:
            self.frames_reported = self.frame_count
            self.frames_pub.publish(UInt32(data=self.frame_count % 2**32))

    def make_handler(self):
        bridge = self

        class FrameHandler(BaseHTTPRequestHandler):

            def do_GET(self):
                url = urlparse(self.path)
                if url.path != '/latest.jpg':
                    self.send_error(404)
                    return
                after = int(parse_qs(url.query).get('after', ['0'])[0])
                # Wachten tot er een nieuwer beeld is dan het vorige dat de klant kreeg.
                with bridge.frame_ready:
                    bridge.frame_ready.wait_for(lambda: bridge.latest_number > after, 2.0)
                    number, jpeg = bridge.latest_number, bridge.latest_jpeg
                if jpeg is None or number <= after:
                    self.send_response(204)
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Expose-Headers', 'X-Frame')
                self.send_header('X-Frame', str(number))
                self.end_headers()
                self.wfile.write(jpeg)

            def log_message(self, *args):
                pass

        return FrameHandler

    def set_preview(self, jpeg):
        with self.frame_ready:
            self.latest_jpeg = jpeg
            self.latest_number += 1
            self.frame_ready.notify_all()

    def publish(self, jpeg):
        self.frame_count += 1
        full = self.pub.get_subscription_count() > 0
        low = (self.frame_count % self.low_every == 0
               and self.low_pub.get_subscription_count() > 0)
        if not (full or low):
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = jpeg
        if full:
            self.pub.publish(msg)
        if low:
            self.low_pub.publish(msg)

    def destroy_node(self):
        self.http.shutdown()
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
