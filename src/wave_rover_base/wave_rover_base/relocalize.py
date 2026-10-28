"""Globale lokalisatie: waar op de kaart staat de robot, zonder startpositie?

Vergelijkt één lidarscan met de opgeslagen kaart (.pgm/.yaml) op alle vrije plekken en
richtingen, van grof naar fijn:
1. "Waarschijnlijkheidsveld": hoe dicht bij een muur ligt elke cel (vervaagde muren).
2. Grof: elke vrije cel op 10 cm, elke 5 graden; score = gemiddelde veldwaarde onder de
   scanpunten. Daarna de beste kandidaten fijn bijstellen (2,5 cm, 1 graad).
3. Controle per straal: voor de beste kandidaten wordt elke lidarstraal door de kaart
   getrokken. Klopt de gemeten afstand met de eerste muur op die lijn, of "kijkt" de straal
   door een muur? Het veld alleen ziet dat laatste niet; deze controle wel, en die maakt
   het verschil tussen twee plekken die op elkaar lijken.
4. Zeker genoeg als de beste plek goed past en duidelijk beter is dan de beste andere plek
   (in een symmetrische ruimte lijken twee plekken soms op elkaar).
Enkel numpy: ~1 s CPU per zoekopdracht, niets als er niet gezocht wordt.
"""
import math
import os

import numpy as np
import yaml
from PIL import Image

MAX_RANGE = 10.0  # m; verder gebruiken we de lidar niet


class OccupancyMap:

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        image = os.path.join(os.path.dirname(yaml_path), meta['image'])
        pixels = np.asarray(Image.open(image).convert('L'), np.float32)
        p = pixels / 255.0 if meta.get('negate', 0) else (255.0 - pixels) / 255.0
        # Rij 0 = kleinste y (de afbeelding staat met de bovenkant = grootste y).
        p = p[::-1]
        self.occupied = p > meta.get('occupied_thresh', 0.65)
        self.free = p < meta.get('free_thresh', 0.25)
        self.resolution = float(meta['resolution'])
        self.origin = np.array(meta['origin'][:2], np.float64)


def blur(mask, sigma_cells):
    """Gaussisch vervagen (scheidbaar), genormaliseerd op 1 voor een muur."""
    radius = max(1, int(math.ceil(3 * sigma_cells)))
    k = np.exp(-0.5 * (np.arange(-radius, radius + 1) / sigma_cells) ** 2).astype(np.float32)
    out = mask.astype(np.float32)
    for axis in (0, 1):
        padded = np.pad(out, [(radius, radius) if a == axis else (0, 0) for a in (0, 1)])
        acc = np.zeros_like(out)
        for i, w in enumerate(k):
            sl = [slice(None), slice(None)]
            sl[axis] = slice(i, i + out.shape[axis])
            acc += w * padded[tuple(sl)]
        out = acc
    return np.minimum(out / out.max(), 1.0) if out.max() > 0 else out


def downsample(mask, factor, mode):
    h, w = (mask.shape[0] // factor) * factor, (mask.shape[1] // factor) * factor
    blocks = mask[:h, :w].reshape(h // factor, factor, w // factor, factor)
    return blocks.any(axis=(1, 3)) if mode == 'any' else blocks.all(axis=(1, 3))


class Field:
    """Waarschijnlijkheidsveld op een bepaalde resolutie."""

    def __init__(self, occupied, resolution, origin, sigma):
        self.values = blur(occupied, sigma / resolution)
        self.resolution = resolution
        self.origin = origin

    def score(self, xs, ys, cos_t, sin_t, points):
        """Gemiddelde veldwaarde van `points` (n x 2, robotframe) voor elke pose.

        xs, ys, cos_t, sin_t: arrays met één waarde per pose. Punten buiten de kaart tellen 0.
        """
        px = xs[:, None] + cos_t[:, None] * points[None, :, 0] - sin_t[:, None] * points[None, :, 1]
        py = ys[:, None] + sin_t[:, None] * points[None, :, 0] + cos_t[:, None] * points[None, :, 1]
        col = ((px - self.origin[0]) / self.resolution).astype(np.int32)
        row = ((py - self.origin[1]) / self.resolution).astype(np.int32)
        h, w = self.values.shape
        inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)
        vals = np.where(inside, self.values[np.clip(row, 0, h - 1), np.clip(col, 0, w - 1)], 0.0)
        return vals.mean(axis=1)


class Scan:
    """Lidarscan, uitgedund tot hoogstens `count` geldige stralen.

    angles/ranges: in het lidarframe; offset: (x, y, yaw) van de lidar op de robot;
    points: de gemeten punten (n x 2) in het robotframe.
    """

    def __init__(self, ranges, angle_min, angle_increment, range_min, range_max, offset,
                 count=180):
        r = np.asarray(ranges, np.float32)
        a = angle_min + np.arange(len(r)) * angle_increment
        ok = np.isfinite(r) & (r > max(range_min, 0.05)) & (r < min(range_max, MAX_RANGE))
        r, a = r[ok], a[ok]
        if len(r) > count:
            idx = np.linspace(0, len(r) - 1, count).astype(int)
            r, a = r[idx], a[idx]
        self.angles, self.ranges, self.offset = a, r, offset
        ox, oy, oyaw = offset
        x, y = r * np.cos(a + oyaw), r * np.sin(a + oyaw)
        self.points = np.stack([ox + x, oy + y], 1).astype(np.float32)


def localize(occ_map, scan, robot_radius=0.12):
    """Zoekt de pose (x, y, yaw) van de robot. Geeft (pose, score, tweede_score)."""
    points = scan.points
    res = occ_map.resolution
    factor = max(1, int(round(0.10 / res)))
    coarse_res = res * factor
    fine = Field(occ_map.occupied, res, occ_map.origin, sigma=0.05)
    coarse = Field(downsample(occ_map.occupied, factor, 'any'), coarse_res, occ_map.origin,
                   sigma=0.15)

    # Kandidaten: vrije cellen waar de robot past (niet tegen een muur).
    clearance = blur(occ_map.occupied, robot_radius / res / 2) < 0.05
    free = downsample(occ_map.free & clearance, factor, 'all')
    rows, cols = np.nonzero(free)
    if len(rows) == 0 or len(points) == 0:
        return None, 0.0, 0.0
    cx = occ_map.origin[0] + (cols + 0.5) * coarse_res
    cy = occ_map.origin[1] + (rows + 0.5) * coarse_res

    coarse_pts = points[:: max(1, len(points) // 60)]
    angles = np.radians(np.arange(0, 360, 5))
    scores = np.empty((len(angles), len(cx)), np.float32)
    for i, t in enumerate(angles):
        c, s = np.full(len(cx), math.cos(t)), np.full(len(cx), math.sin(t))
        scores[i] = coarse.score(cx, cy, c, s, coarse_pts)

    # Beste kandidaten, onderling verschillend (ander plek of andere richting).
    order = np.argsort(scores, axis=None)[::-1]
    candidates = []
    for flat in order[:4000]:
        ai, ci = np.unravel_index(flat, scores.shape)
        pose = (cx[ci], cy[ci], angles[ai])
        if all(not similar(pose, c) for c in candidates):
            candidates.append(pose)
            if len(candidates) == 25:
                break

    poses = [refine(fine, points, c)[0] for c in candidates]
    checked = sorted(((p, beam_score(occ_map, scan, p)) for p in poses), key=lambda r: -r[1])
    best_pose, best = checked[0]
    second = max((s for p, s in checked[1:] if not similar(p, best_pose)), default=0.0)
    return best_pose, best, second


def beam_score(occ_map, scan, pose, sigma=0.10, through=0.20):
    """Deel van de stralen waarvan de gemeten afstand klopt met de eerste muur op de kaart,
    min het deel dat "door een muur kijkt" (verder meet dan die muur + `through`).

    Iets vóór de muur (meubel, persoon) telt gewoon niet mee; door een muur kijken kan niet
    op de juiste plek. Dat onderscheidt plekken waar het veld (enkel de eindpunten) geen
    verschil ziet.
    """
    x, y, t = pose
    ox, oy, oyaw = scan.offset
    lx = x + math.cos(t) * ox - math.sin(t) * oy
    ly = y + math.sin(t) * ox + math.cos(t) * oy
    a = scan.angles + oyaw + t
    step = occ_map.resolution / 2
    d = np.arange(step, MAX_RANGE, step, dtype=np.float32)
    col = ((lx + np.cos(a)[:, None] * d - occ_map.origin[0]) / occ_map.resolution).astype(np.int32)
    row = ((ly + np.sin(a)[:, None] * d - occ_map.origin[1]) / occ_map.resolution).astype(np.int32)
    h, w = occ_map.occupied.shape
    inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)
    wall = inside & occ_map.occupied[np.clip(row, 0, h - 1), np.clip(col, 0, w - 1)]
    expected = np.where(wall.any(axis=1), d[wall.argmax(axis=1)], np.inf)
    match = np.exp(-0.5 * ((scan.ranges - expected) / sigma) ** 2)
    return float(match.mean() - (scan.ranges > expected + through).mean())


def similar(a, b, dist=0.4, angle=math.radians(25)):
    d_angle = abs((a[2] - b[2] + math.pi) % (2 * math.pi) - math.pi)
    return math.hypot(a[0] - b[0], a[1] - b[1]) < dist and d_angle < angle


def refine(field, points, pose):
    """Lokaal bijstellen rond `pose`: twee rondes, telkens fijner."""
    x, y, t = pose
    for step_xy, step_t in ((0.05, math.radians(2.5)), (0.0125, math.radians(0.5))):
        d = np.arange(-4, 5)
        dx, dy, dt = np.meshgrid(d * step_xy, d * step_xy, d * step_t, indexing='ij')
        xs, ys, ts = x + dx.ravel(), y + dy.ravel(), t + dt.ravel()
        s = field.score(xs, ys, np.cos(ts), np.sin(ts), points)
        i = int(np.argmax(s))
        x, y, t, best = xs[i], ys[i], ts[i], float(s[i])
    return (float(x), float(y), float((t + math.pi) % (2 * math.pi) - math.pi)), best
