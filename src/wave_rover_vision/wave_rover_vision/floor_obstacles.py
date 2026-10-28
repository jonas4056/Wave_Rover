"""Lage obstakels op de vloer met de camera: alles wat uit de vloer steekt, zonder te weten wat.

Werkwijze per beeld (`rate` keer per seconde):
1. Depth Anything V2 Small (TensorRT) schat de relatieve inverse diepte van elk pixel.
2. De camera staat vast, dus voor elk pixel onder de horizon is bekend waar zijn straal de
   vloer raakt (`camera_height`, `focal_length`, `pitch`; zie scripts/calibrate_floor.py).
   Een vlakke vloer heeft een inverse diepte die daarmee evenredig is; die verhouding (schaal
   en verschuiving) wordt per beeld robuust gepast, want het model geeft geen meters.
3. Pixels die duidelijk dichterbij zijn dan de vloer daar zou zijn = obstakel. Per kolom
   telt het laagste obstakelpixel: daar raakt het obstakel de vloer, en dat punt ligt vast.
4. Publiceert /floor_scan (sensor_msgs/LaserScan in camera_link): een virtuele lidar op
   vloerhoogte. Nav2 zet die in een eigen costmap-laag (floor_layer in nav2.yaml).
"""
import io
import math
import time

import numpy as np
import rclpy
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan

from wave_rover_vision.trt_model import TrtModel, build_engine

IMAGE_W, IMAGE_H = 1280, 720  # camerabeeld waarop de kalibratie slaat


class FloorObstacles(Node):

    def __init__(self):
        super().__init__('floor_obstacles')
        self.declare_parameter('model',
                               '/home/ubuntu/wave_rover/models/depth_anything_v2_small_rgb8.onnx')
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('frame_id', 'camera_link')
        # Kalibratie (pixels van het 1280x720-beeld; pitch in rad, positief = naar beneden).
        self.declare_parameter('camera_height', 0.085)
        self.declare_parameter('focal_length', 1000.0)
        self.declare_parameter('pitch', 0.0)
        # Verder dan max_range is de vloer te weinig pixels om betrouwbaar te zijn.
        self.declare_parameter('max_range', 1.5)
        # Obstakel = minstens zoveel (relatief) dichterbij dan de vloer op die plek.
        self.declare_parameter('threshold', 0.15)
        # Zoveel pixels moet een obstakel minstens hoog zijn (tegen ruis).
        self.declare_parameter('min_height_px', 3)
        self.declare_parameter('columns_per_beam', 4)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.frame_id = p('frame_id')
        self.max_range = p('max_range')
        self.threshold = p('threshold')
        self.min_height_px = p('min_height_px')

        engine = build_engine(p('model'), self.get_logger().info)
        self.model = TrtModel(engine)
        self.input = next(iter(self.model.inputs.values()))
        self.output = next(iter(self.model.outputs.values()))
        _, self.in_h, self.in_w, _ = self.input.shape
        self.get_logger().info(f'Model geladen: {engine} (ingang {self.in_w}x{self.in_h})')

        self.setup_geometry(p('camera_height'), p('focal_length'), p('pitch'),
                            p('columns_per_beam'))

        self.latest = None
        self.create_subscription(CompressedImage, 'camera/image_low/compressed', self.on_image,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, 'floor_scan', 10)
        self.create_timer(1.0 / p('rate'), self.process)

        self.stats_start = time.monotonic()
        self.stats_frames = 0
        self.stats_skipped = 0
        self.stats_busy = [0.0, 0.0, 0.0]

    def setup_geometry(self, height, focal, pitch, columns_per_beam):
        """Per pixel van de modeluitgang: waar raakt zijn straal de vloer?"""
        h, w = self.output.shape[-2:]
        u = (np.arange(w) + 0.5) * IMAGE_W / w
        v = (np.arange(h) + 0.5) * IMAGE_H / h
        uu, vv = np.meshgrid(u, v)
        # Straal in camera_link (x vooruit, y links, z omhoog), dan de camera laten kantelen.
        x, y, z = np.ones_like(uu), -(uu - IMAGE_W / 2) / focal, -(vv - IMAGE_H / 2) / focal
        xr = x * math.cos(pitch) + z * math.sin(pitch)
        zr = -x * math.sin(pitch) + z * math.cos(pitch)
        down = zr < -1e-6
        t = np.where(down, height / np.where(down, -zr, 1.0), np.inf)  # afstand langs de straal
        gx, gy = t * xr, t * y
        rng = np.hypot(gx, gy)
        self.valid = down & (rng <= self.max_range)
        self.ground_range = np.where(self.valid, rng, np.inf).astype(np.float32)
        self.ground_angle = np.arctan2(gy, gx).astype(np.float32)
        # Inverse diepte van de vloer (op een schaal na): 1 / afstand langs de optische as.
        self.floor_g = np.where(self.valid, 1.0 / t, 0.0).astype(np.float32)
        # Startschatting van de vloer: de onderste helft van de bruikbare pixels.
        rows = np.nonzero(self.valid.any(axis=1))[0]
        self.seed = self.valid.copy()
        if len(rows):
            self.seed[:rows[len(rows) // 2]] = False

        self.beams = max(1, w // columns_per_beam)
        bottom = self.ground_angle[-1]
        self.angle_min, self.angle_max = float(bottom.min()), float(bottom.max())
        self.angle_increment = (self.angle_max - self.angle_min) / max(1, self.beams - 1)

    def on_image(self, msg):
        self.latest = msg  # alleen het nieuwste beeld bewaren

    def prepare(self, jpeg):
        img = Image.open(io.BytesIO(jpeg))
        img.draft('RGB', (self.in_w, self.in_h))  # JPEG decodeert meteen op 1/2: minder werk
        img = img.convert('RGB').resize((self.in_w, self.in_h), Image.BILINEAR)
        self.input[0] = np.asarray(img)

    def fit_floor(self, depth):
        """Past depth ~= s * floor_g + c op de vloerpixels; geeft None als dat niet lukt."""
        g, d = self.floor_g, depth
        mask = self.seed
        for _ in range(3):
            n = mask.sum()
            if n < 500:
                return None
            gm, dm = g[mask], d[mask]
            gmean, dmean = gm.mean(), dm.mean()
            var = ((gm - gmean) ** 2).mean()
            if var <= 0:
                return None
            s = ((gm - gmean) * (dm - dmean)).mean() / var
            c = dmean - s * gmean
            if s <= 0:
                return None
            pred = s * g + c
            mask = self.valid & (np.abs(d - pred) < 0.1 * np.abs(pred))
        # Te weinig vloer gezien (bv. vlak voor een muur): geen uitspraak doen.
        if mask.sum() < 0.25 * self.valid.sum():
            return None
        return pred

    def process(self):
        msg, self.latest = self.latest, None
        if msg is None:
            return
        t0 = time.monotonic()
        self.prepare(bytes(msg.data))
        t1 = time.monotonic()
        self.model.run()
        t2 = time.monotonic()
        depth = self.output.reshape(self.output.shape[-2:])

        pred = self.fit_floor(depth)
        if pred is None:
            self.stats_skipped += 1
            self.log_stats((t1 - t0, t2 - t1, time.monotonic() - t2))
            return
        obstacle = self.valid & (depth - pred > self.threshold * pred)
        # Pas een obstakel als het ook in de pixels erboven zit (ruis weg).
        solid = obstacle.copy()
        for k in range(1, self.min_height_px):
            solid[k:] &= obstacle[:-k]
        solid[:self.min_height_px - 1] = False
        # Per kolom het laagste obstakelpixel = waar het de vloer raakt.
        has = solid.any(axis=0)
        bottom_row = solid.shape[0] - 1 - np.argmax(solid[::-1], axis=0)
        cols = np.nonzero(has)[0]
        rows = bottom_row[cols]

        ranges = np.full(self.beams, np.inf, dtype=np.float32)
        beam = np.round((self.ground_angle[rows, cols] - self.angle_min)
                        / max(self.angle_increment, 1e-9)).astype(int)
        keep = (beam >= 0) & (beam < self.beams)
        np.minimum.at(ranges, beam[keep], self.ground_range[rows, cols][keep])

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = 0.05
        scan.range_max = float(self.max_range)
        # inf = vrije vloer tot max_range (Nav2: inf_is_valid wist dan de cellen).
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)
        self.log_stats((t1 - t0, t2 - t1, time.monotonic() - t2))

    def log_stats(self, parts):
        self.stats_frames += 1
        self.stats_busy = [a + b for a, b in zip(self.stats_busy, parts)]
        elapsed = time.monotonic() - self.stats_start
        if elapsed >= 60.0:
            ms = [1000 * t / self.stats_frames for t in self.stats_busy]
            self.get_logger().info(
                f'{self.stats_frames / elapsed:.1f} beelden/s ({self.stats_skipped} zonder '
                f'vloer); per beeld: decoderen {ms[0]:.1f} ms, GPU {ms[1]:.1f} ms, '
                f'vloer {ms[2]:.1f} ms')
            self.stats_start, self.stats_frames, self.stats_skipped = time.monotonic(), 0, 0
            self.stats_busy = [0.0, 0.0, 0.0]

    def destroy_node(self):
        self.model.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FloorObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
