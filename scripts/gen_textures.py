#!/usr/bin/env python3
"""Generate the arena textures (run once; outputs are committed).

  python3 scripts/gen_textures.py            # needs numpy + opencv-contrib (cv2.aruco)

Writes models/arena_assets/materials/textures/:
  tag36h11_<id>.png   real AprilTag 36h11 markers (ids 0..N-1) with a white quiet zone,
                      usable with apriltag_ros / cv2.aruco for landmark localization
  tag36h11_<id>_m.png horizontally mirrored copy (see arena.landmarks.tag_mirror)
  poster_<k>.jpg      random high-contrast shapes: texture for stereo / visual SLAM features
  floor_<k>.jpg       multi-scale speckle "gym floor": texture for the optical-flow camera
  wall.jpg            low-contrast concrete-ish wall
"""
import os
import sys

import cv2
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUT = os.path.join(ROOT, "models", "arena_assets", "materials", "textures")
N_TAGS = 40
N_POSTERS = 8
N_FLOORS = 2


def tags(rng):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for i in range(N_TAGS):
        inner = cv2.aruco.generateImageMarker(d, i, 400)  # 8x8 cells incl. black border -> 50 px/cell
        img = cv2.copyMakeBorder(inner, 56, 56, 56, 56, cv2.BORDER_CONSTANT, value=255)  # 512 px total
        cv2.imwrite(os.path.join(OUT, f"tag36h11_{i}.png"), img)
        # mirrored copy: use if Gazebo's plane UVs end up flipping the image (a mirrored tag won't decode)
        cv2.imwrite(os.path.join(OUT, f"tag36h11_{i}_m.png"), cv2.flip(img, 1))


def posters(rng):
    for k in range(N_POSTERS):
        img = np.full((512, 512, 3), rng.integers(180, 255, 3), np.uint8)
        for _ in range(int(rng.integers(25, 45))):
            col = tuple(int(c) for c in rng.integers(0, 255, 3))
            kind = rng.integers(0, 3)
            p = rng.integers(0, 512, 2)
            s = int(rng.integers(15, 120))
            if kind == 0:
                cv2.circle(img, tuple(int(v) for v in p), s // 2, col, -1)
            elif kind == 1:
                cv2.rectangle(img, tuple(int(v) for v in p), tuple(int(v) for v in p + s), col, -1)
            else:
                pts = rng.integers(-s, s, (3, 2)) + p
                cv2.fillPoly(img, [pts.astype(np.int32)], col)
        cv2.putText(img, f"P{k}", (20, 490), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (0, 0, 0), 6)
        cv2.imwrite(os.path.join(OUT, f"poster_{k}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 85])


def floors(rng):
    n = 1024
    for k in range(N_FLOORS):
        acc = np.zeros((n, n), np.float32)
        for scale, w in ((4, 0.5), (16, 0.3), (64, 0.2)):  # fine grain + blotches
            small = rng.random((n // scale, n // scale)).astype(np.float32)
            acc += w * cv2.resize(small, (n, n), interpolation=cv2.INTER_CUBIC)
        acc = (acc - acc.min()) / (acc.max() - acc.min())
        base = np.array([150, 170, 190], np.float32)  # warm wood-ish (BGR)
        img = (base[None, None, :] * (0.55 + 0.6 * acc[..., None])).clip(0, 255).astype(np.uint8)
        for y in range(0, n, 128):  # plank seams
            cv2.line(img, (0, y), (n, y), (60, 70, 80), 2)
        cv2.imwrite(os.path.join(OUT, f"floor_{k}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 80])


def wall(rng):
    n = 512
    acc = cv2.GaussianBlur(rng.random((n, n)).astype(np.float32), (0, 0), 3)
    acc = (acc - acc.min()) / (acc.max() - acc.min())
    img = (170 + 40 * acc).astype(np.uint8)
    cv2.imwrite(os.path.join(OUT, "wall.jpg"), cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), [cv2.IMWRITE_JPEG_QUALITY, 80])


def main():
    if not hasattr(cv2, "aruco"):
        sys.exit("cv2.aruco missing: pip install opencv-contrib-python")
    os.makedirs(OUT, exist_ok=True)
    rng = np.random.default_rng(1234)
    tags(rng); posters(rng); floors(rng); wall(rng)
    print(f"wrote textures to {OUT}")


if __name__ == "__main__":
    main()
