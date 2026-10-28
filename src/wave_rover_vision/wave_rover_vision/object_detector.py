"""Objectherkenning op de camerabeelden met YOLO26n op de GPU (TensorRT).

Abonneert op /camera/image_low/compressed (10 Hz, zie camera_bridge) en verwerkt `rate` keer per seconde het nieuwste
beeld; tussenliggende beelden worden overgeslagen. Publiceert /detections
(vision_msgs/Detection2DArray) met kaders in pixels van het camerabeeld en de klasse als
tekst (COCO: person, tv, chair, ...).

Het model (ONNX, ingangsmaat 640x384, "end2end": NMS zit al in het model) staat in
~/wave_rover/models; het TensorRT-engine wordt de eerste keer zelf gebouwd. Het neemt ruwe
RGB-bytes (1 x H x W x 3); de omzetting naar float doet de GPU (scripts/export_yolo.sh).
"""
import io
import time

import numpy as np
import rclpy
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from wave_rover_vision.coco import COCO_NAMES
from wave_rover_vision.trt_model import TrtModel, build_engine

PAD_VALUE = 114  # zelfde grijze rand als bij het trainen (ultralytics letterbox)


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('model', '/home/ubuntu/wave_rover/models/yolo26n_rgb8.onnx')
        # 10 Hz is vlot genoeg om iets te zoeken; elk extra beeld kost CPU voor het decoderen.
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('threshold', 0.4)
        self.threshold = self.get_parameter('threshold').value

        engine = build_engine(self.get_parameter('model').value, self.get_logger().info)
        self.model = TrtModel(engine)
        self.input = next(iter(self.model.inputs.values()))
        self.output = next(iter(self.model.outputs.values()))
        _, self.in_h, self.in_w, _ = self.input.shape
        self.get_logger().info(f'Model geladen: {engine} (ingang {self.in_w}x{self.in_h})')

        self.latest = None
        self.create_subscription(CompressedImage, 'camera/image_low/compressed', self.on_image,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.process)

        self.stats_start = time.monotonic()
        self.stats_frames = 0
        self.stats_busy = [0.0, 0.0, 0.0]

    def on_image(self, msg):
        self.latest = msg  # alleen het nieuwste beeld bewaren

    def prepare(self, jpeg):
        """JPEG decoderen op ingangsmaat en in het midden van de ingang plaatsen."""
        img = Image.open(io.BytesIO(jpeg))
        width, height = img.size
        scale = min(self.in_w / width, self.in_h / height)
        size = (round(width * scale), round(height * scale))
        img.draft('RGB', size)  # JPEG decodeert meteen op 1/2, 1/4, ...: veel minder werk
        img = img.convert('RGB')
        if img.size != size:
            img = img.resize(size, Image.BILINEAR)
        left, top = (self.in_w - size[0]) // 2, (self.in_h - size[1]) // 2
        self.input.fill(PAD_VALUE)
        self.input[0, top:top + size[1], left:left + size[0]] = np.asarray(img)
        return scale, left, top

    def process(self):
        msg, self.latest = self.latest, None
        if msg is None:
            return
        t0 = time.monotonic()
        scale, left, top = self.prepare(bytes(msg.data))
        t1 = time.monotonic()
        self.model.run()
        t2 = time.monotonic()

        out = Detection2DArray()
        out.header = msg.header
        rows = self.output[0]
        for x1, y1, x2, y2, score, cls in rows[rows[:, 4] >= self.threshold].tolist():
            det = Detection2D()
            det.header = msg.header
            # Terug naar pixels van het camerabeeld.
            det.bbox.center.position.x = float((x1 + x2) / 2 - left) / scale
            det.bbox.center.position.y = float((y1 + y2) / 2 - top) / scale
            det.bbox.size_x = float(x2 - x1) / scale
            det.bbox.size_y = float(y2 - y1) / scale
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = COCO_NAMES[int(cls)]
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            out.detections.append(det)
        self.pub.publish(out)
        self.log_stats((t1 - t0, t2 - t1, time.monotonic() - t2))

    def log_stats(self, parts):
        self.stats_frames += 1
        self.stats_busy = [a + b for a, b in zip(self.stats_busy, parts)]
        elapsed = time.monotonic() - self.stats_start
        if elapsed >= 60.0:
            ms = [1000 * t / self.stats_frames for t in self.stats_busy]
            self.get_logger().info(
                f'{self.stats_frames / elapsed:.1f} beelden/s; per beeld: decoderen '
                f'{ms[0]:.1f} ms, GPU {ms[1]:.1f} ms, publiceren {ms[2]:.1f} ms')
            self.stats_start, self.stats_frames = time.monotonic(), 0
            self.stats_busy = [0.0, 0.0, 0.0]

    def destroy_node(self):
        self.model.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
