"""Kalibreert de camera t.o.v. de vloer voor de module floor_obstacles.

Leg een bal (COCO "sports ball") recht voor de camera op enkele afstanden, bv. 30, 60 en
100 cm, gemeten over de vloer van de lens tot de voorkant van de bal. Module detection aan.

1. Per afstand, in de container (geeft de beeldrij waar de bal de vloer raakt):
     python3 scripts/calibrate_floor.py measure
2. Daarna (waar dan ook):
     python3 scripts/calibrate_floor.py fit --height 0.105 --ball 0.10 0.30:675 0.60:520 1.00:470
   en focal_length en pitch overnemen in src/wave_rover_bringup/config/vision.yaml.
"""
import argparse
import math
import statistics
import time

IMAGE_H = 720


def measure(seconds=4.0):
    import rclpy
    from vision_msgs.msg import Detection2DArray

    rclpy.init()
    node = rclpy.create_node('calibrate_floor')
    rows = []

    def on_detections(msg):
        for det in msg.detections:
            if det.results[0].hypothesis.class_id == 'sports ball':
                rows.append(det.bbox.center.position.y + det.bbox.size_y / 2)

    node.create_subscription(Detection2DArray, 'detections', on_detections, 10)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if rows:
        print(f'onderkant bal: rij {statistics.median(rows):.1f} ({len(rows)} metingen)')
    else:
        print('geen bal gezien (staat de module detection aan?)')


def fit(height, ball, points):
    """Zoekt focal_length en pitch zodat de contactpunten op de juiste rijen vallen."""
    cy = IMAGE_H / 2

    def error(f, pitch):
        return sum((cy + f * math.tan(math.atan(height / (d + ball / 2)) - pitch) - v) ** 2
                   for d, v in points)

    best = None
    for f in range(300, 3001, 5):  # grof rooster, daarna fijn: geen extra pakketten nodig
        for p in range(-300, 301, 5):
            e = error(f, p / 1000)
            if best is None or e < best[0]:
                best = (e, f, p / 1000)
    _, f0, p0 = best
    for f in [f0 + i * 0.5 for i in range(-20, 21)]:
        for p in [p0 + i * 0.0002 for i in range(-30, 31)]:
            e = error(f, p)
            if e < best[0]:
                best = (e, f, p)
    e, f, pitch = best
    print(f'focal_length: {f:.1f}')
    print(f'pitch: {pitch:.4f}  ({math.degrees(pitch):.1f} graden, + = naar beneden)')
    print(f'horizontale beeldhoek: {math.degrees(2 * math.atan(640 / f)):.0f} graden, '
          f'horizon op rij {cy + f * math.tan(-pitch):.0f}')
    for d, v in points:
        pred = cy + f * math.tan(math.atan(height / (d + ball / 2)) - pitch)
        print(f'  {d:.2f} m: gemeten rij {v:.1f}, berekend {pred:.1f}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    sub.add_parser('measure')
    p = sub.add_parser('fit')
    p.add_argument('--height', type=float, required=True, help='lens boven de vloer (m)')
    p.add_argument('--ball', type=float, required=True, help='diameter van de bal (m)')
    p.add_argument('points', nargs='+', help='afstand:rij, bv. 0.30:675')
    args = parser.parse_args()
    if args.cmd == 'measure':
        measure()
    else:
        points = [tuple(float(x) for x in pt.split(':')) for pt in args.points]
        fit(args.height, args.ball, points)


if __name__ == '__main__':
    main()
