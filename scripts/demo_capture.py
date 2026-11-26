#!/usr/bin/env python3
"""Record a short demo of the arena + full sensor stack to files (no GUI needed, works over SSH).

Writes to out_dir:
  overview.mp4   corner spectator camera (needs models/demo_cameras spawned; demo.sh does it)
  topdown.mp4    camera looking straight down on the arena
  onboard.mp4    the blimp's left stereo camera, with the optical-flow camera inset
  scan_map.png   every 2D lidar scan placed with the ground-truth pose: the arena outline, built by the blimp
  altitude.png   height from ground truth vs ToF vs barometer
  summary.png    the four pictures in one image, for a slide
  sensors.txt    message count and rate for each sensor topic

Params: agent, info_file (generated/<scenario>/scenario_info.yaml), out_dir, duration (sim s), max_wall (s).
Uses only rclpy, numpy and OpenCV (no cv_bridge, no matplotlib).
"""
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.clock import Clock
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Image, LaserScan
from rcl_interfaces.msg import ParameterDescriptor

FPS = 10.0
RHO_G = 1.225 * 9.81  # Pa per metre of height


def param(node, name, default):
    """Declare (or re-read) a parameter. Tolerates 150 for a float default, "true" for a bool, etc."""
    if node.has_parameter(name):
        v = node.get_parameter(name).value
    else:
        v = node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
    if isinstance(default, bool):
        return v if isinstance(v, bool) else str(v).lower() in ("1", "true", "yes")
    if isinstance(default, float) and isinstance(v, int):
        return float(v)
    if isinstance(default, str) and v is not None and not isinstance(v, str):
        return str(v)
    return v


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def to_bgr(m):
    a = np.frombuffer(m.data, np.uint8)
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1, "8UC1": 1, "8UC3": 3}.get(m.encoding)
    if ch is None:
        return None
    a = a.reshape(m.height, m.step)[:, : m.width * ch].reshape(m.height, m.width, ch)
    if m.encoding in ("rgb8",):
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    if m.encoding in ("rgba8",):
        return cv2.cvtColor(a, cv2.COLOR_RGBA2BGR)
    if m.encoding in ("bgra8",):
        return cv2.cvtColor(a, cv2.COLOR_BGRA2BGR)
    if ch == 1:
        return cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
    return a.copy()


def rpy_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def label(img, text, org=(10, 28), scale=0.7):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def line_plot(series, title, ylabel, w=900, h=420):
    """series: list of (name, ts, vs, bgr). Plain OpenCV line chart."""
    img = np.full((h, w, 3), 255, np.uint8)
    L, R, T, B = 70, 20, 40, 50
    ts = [t for _, tt, _, _ in series for t in tt]
    vs = [v for _, _, vv, _ in series for v in vv]
    cv2.putText(img, title, (L, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (30, 30, 30), 1, cv2.LINE_AA)
    if not ts:
        cv2.putText(img, "no data", (w // 2 - 40, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 200), 2)
        return img
    t0, t1 = min(ts), max(ts) + 1e-6
    v0, v1 = min(vs), max(vs)
    pad = max(0.1, 0.1 * (v1 - v0))
    v0, v1 = v0 - pad, v1 + pad
    X = lambda t: int(L + (t - t0) / (t1 - t0) * (w - L - R))  # noqa: E731
    Y = lambda v: int(h - B - (v - v0) / (v1 - v0) * (h - T - B))  # noqa: E731
    for i in range(6):  # grid + ticks
        v = v0 + i * (v1 - v0) / 5
        cv2.line(img, (L, Y(v)), (w - R, Y(v)), (230, 230, 230), 1)
        cv2.putText(img, f"{v:.1f}", (8, Y(v) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1, cv2.LINE_AA)
        t = t0 + i * (t1 - t0) / 5
        cv2.putText(img, f"{t:.0f}s", (X(t) - 12, h - B + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1,
                    cv2.LINE_AA)
    cv2.rectangle(img, (L, T), (w - R, h - B), (120, 120, 120), 1)
    cv2.putText(img, ylabel, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1, cv2.LINE_AA)
    for k, (name, tt, vv, col) in enumerate(series):
        pts = np.array([[X(t), Y(v)] for t, v in zip(tt, vv)], np.int32)
        if len(pts) > 1:
            cv2.polylines(img, [pts], False, col, 2 if k == len(series) - 1 else 1, cv2.LINE_AA)
        cv2.line(img, (L + 10 + 180 * k, h - 14), (L + 40 + 180 * k, h - 14), col, 3)
        cv2.putText(img, name, (L + 46 + 180 * k, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1, cv2.LINE_AA)
    return img


class Video:
    """Writes frames at a fixed sim-time rate, so the mp4 plays back in real (sim) time."""

    def __init__(self, path):
        self.path, self.w, self.last, self.n, self.keep = path, None, -1e9, 0, []

    def add(self, t, img):
        if img is None or t < self.last + 1.0 / FPS - 1e-3:
            return
        if self.w is None:
            self.w = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (img.shape[1], img.shape[0]))
        self.w.write(img)
        self.last, self.n = t, self.n + 1
        if self.n % int(FPS * 5) == 1:  # keep a still every 5 s for the summary
            self.keep.append(img)

    def close(self):
        if self.w is not None:
            self.w.release()

    def still(self, frac=0.5):
        return self.keep[min(len(self.keep) - 1, int(frac * len(self.keep)))] if self.keep else None


class DemoCapture(Node):
    def __init__(self):
        super().__init__("demo_capture")
        P = lambda n, d: param(self, n, d)  # noqa: E731
        a = self.a = P("agent", "blimp")
        info_file = P("info_file", "generated/arena_solo/scenario_info.yaml")
        self.out = P("out_dir", "demo_out")
        self.duration = P("duration", 150.0)
        self.max_wall = P("max_wall", 900.0)
        os.makedirs(self.out, exist_ok=True)
        gen = os.path.dirname(os.path.abspath(info_file))
        with open(info_file) as f:
            self.info = yaml.safe_load(f)
        self.arena = self.info.get("arena") or dict(length=20.0, width=12.0)
        ag = next(x for x in self.info["agents"] if x["name"] == a)
        self.stack = ag.get("stack", "?")
        lm_file = os.path.join(gen, "landmarks.yaml")
        self.landmarks = yaml.safe_load(open(lm_file))["landmarks"] if os.path.isfile(lm_file) else []
        tfs = {}
        tf_file = os.path.join(gen, "static_tf.yaml")
        if os.path.isfile(tf_file):
            tfs = {t["child"]: t for t in yaml.safe_load(open(tf_file))["transforms"]}
        t = tfs.get(f"{a}/scan_link")
        self.scan_R = rpy_R(*t["rpy"]) if t else np.eye(3)
        self.scan_t = np.array(t["xyz"]) if t else np.zeros(3)
        t = tfs.get(f"{a}/tof_down_link")
        self.tof_z = -float(t["xyz"][2]) if t else 0.0  # ToF sits this far below base_link

        self.vid = {k: Video(os.path.join(self.out, f"{k}.mp4")) for k in ("overview", "topdown", "onboard")}
        self.flow_img = None
        self.pose = None  # (t, position np, R)
        self.t0 = None
        self.gt, self.tof, self.baro = [], [], []
        self.baro_p0 = None
        self.scan_pts, self.n_scans = [], 0
        self.counts, self.first, self.last = {}, {}, {}
        self.wall0 = time.time()
        self.done = False

        S = lambda typ, topic, cb: self.create_subscription(typ, topic, cb, qos_profile_sensor_data)  # noqa: E731
        S(Odometry, f"/{a}/odom_gt", self.on_odom)
        S(Image, "/demo/overview/image", lambda m: self.on_cam("overview", m))
        S(Image, "/demo/topdown/image", lambda m: self.on_cam("topdown", m))
        S(Image, f"/{a}/stereo/left/image_raw", self.on_onboard)
        S(Image, f"/{a}/flow/camera/image_raw", self.on_flow)
        S(LaserScan, f"/{a}/scan", self.on_scan)
        S(LaserScan, f"/{a}/tof_down", self.on_tof)
        S(FluidPressure, f"/{a}/baro", self.on_baro)
        self.create_timer(1.0, self.check_wall, clock=Clock())  # wall-clock guard
        self.get_logger().info(f"recording {self.duration:.0f} sim-s of '{self.info['scenario']}' "
                               f"(stack {self.stack}) to {self.out}")

    def count(self, topic, t):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.first.setdefault(topic, t)
        self.last[topic] = t

    def rel(self, t):
        return t - (self.t0 if self.t0 is not None else t)

    def on_odom(self, m):
        t = stamp(m)
        self.count("odom_gt", t)
        p = m.pose.pose.position
        self.pose = (t, np.array([p.x, p.y, p.z]), quat_R(m.pose.pose.orientation))
        if self.t0 is None:
            self.t0 = t
        self.gt.append((self.rel(t), p.x, p.y, p.z))
        if self.rel(t) > self.duration and not self.done:
            self.finish(f"{self.duration:.0f} sim-s recorded")

    def on_cam(self, name, m):
        self.count(name, stamp(m))
        img = to_bgr(m)
        if img is not None and name == "topdown":
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)  # -> world +x right, +y up
        if img is not None:
            label(img, f"{name}  t={self.rel(stamp(m)):.0f}s")
        self.vid[name].add(stamp(m), img)

    def on_flow(self, m):
        self.count("flow_camera", stamp(m))
        self.flow_img = to_bgr(m)

    def on_onboard(self, m):
        self.count("stereo_left", stamp(m))
        img = to_bgr(m)
        if img is None:
            return
        if self.flow_img is not None:  # inset the 64x64 flow camera, enlarged
            s = max(96, img.shape[0] // 3)
            inset = cv2.resize(self.flow_img, (s, s), interpolation=cv2.INTER_NEAREST)
            img[-s - 10:-10, -s - 10:-10] = inset
            cv2.rectangle(img, (img.shape[1] - s - 11, img.shape[0] - s - 11),
                          (img.shape[1] - 10, img.shape[0] - 10), (255, 255, 255), 1)
            label(img, "flow cam", (img.shape[1] - s - 6, img.shape[0] - s - 16), 0.45)
        label(img, f"onboard stereo (left)  t={self.rel(stamp(m)):.0f}s")
        self.vid["onboard"].add(stamp(m), img)

    def on_scan(self, m):
        self.count("scan", stamp(m))
        self.n_scans += 1
        if self.pose is None or self.n_scans % 2:
            return
        r = np.asarray(m.ranges, float)
        ang = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max * 0.99)
        if not ok.any():
            return
        pl = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]), np.zeros(ok.sum())], 1)
        _, pos, R = self.pose
        pw = (R @ (self.scan_R @ pl.T + self.scan_t[:, None])).T + pos
        self.scan_pts.append(pw[:, :2])

    def on_tof(self, m):
        self.count("tof_down", stamp(m))
        r = [x for x in m.ranges if math.isfinite(x) and m.range_min < x < m.range_max]
        if r:
            self.tof.append((self.rel(stamp(m)), min(r) + self.tof_z))

    def on_baro(self, m):
        self.count("baro", stamp(m))
        if self.pose is None:
            return
        if self.baro_p0 is None:  # anchor to ground-truth height at the first reading
            self.baro_p0 = (m.fluid_pressure, self.pose[1][2])
        p0, z0 = self.baro_p0
        self.baro.append((self.rel(stamp(m)), z0 + (p0 - m.fluid_pressure) / RHO_G))

    def check_wall(self):
        if not self.done and time.time() - self.wall0 > self.max_wall:
            self.finish(f"wall-clock limit {self.max_wall:.0f}s hit (sim slower than expected?)")

    # ---------- outputs ----------
    def scan_map(self):
        L, W = float(self.arena["length"]), float(self.arena["width"])
        s, m = 45.0, 40  # px per metre, margin px
        w, h = int(L * s) + 2 * m, int(W * s) + 2 * m + 40
        img = np.full((h, w, 3), 255, np.uint8)
        P = lambda x, y: (int(m + (x + L / 2) * s), int(m + 40 + (W / 2 - y) * s))  # noqa: E731
        cv2.rectangle(img, P(-L / 2, W / 2), P(L / 2, -W / 2), (200, 200, 200), 1)
        for gx in range(int(-L // 2), int(L // 2) + 1, 2):
            cv2.line(img, P(gx, -W / 2), P(gx, W / 2), (240, 240, 240), 1)
        pts = np.concatenate(self.scan_pts) if self.scan_pts else np.zeros((0, 2))
        for x, y in pts[:: max(1, len(pts) // 60000)]:
            cv2.circle(img, P(x, y), 1, (40, 40, 40), -1)
        team = {"blue": (200, 90, 20), "red": (40, 40, 210)}
        for lmk in self.landmarks:
            x, y = lmk["xyz"][:2]
            if lmk["type"] == "apriltag":
                cv2.rectangle(img, (P(x, y)[0] - 4, P(x, y)[1] - 4), (P(x, y)[0] + 4, P(x, y)[1] + 4), (0, 160, 0), -1)
            elif lmk["type"] == "pillar":
                cv2.circle(img, P(x, y), max(3, int(lmk.get("radius", 0.25) * s)), (0, 140, 255), 2)
            elif lmk["type"] == "goal":
                cv2.circle(img, P(x, y), 7, team.get(lmk.get("team"), (120, 120, 120)), 2)
        path = np.array([P(x, y) for _, x, y, _ in self.gt[::5]], np.int32)
        if len(path) > 1:
            cv2.polylines(img, [path], False, (230, 120, 0), 2, cv2.LINE_AA)
            cv2.circle(img, tuple(path[0]), 6, (230, 120, 0), -1)
        cv2.putText(img, f"2D lidar scans placed with ground-truth pose ({len(self.scan_pts)} scans, no SLAM)",
                    (m, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 30, 30), 1, cv2.LINE_AA)
        leg = [("lidar hits", (40, 40, 40)), ("flight path", (230, 120, 0)), ("AprilTags", (0, 160, 0)),
               ("pillars", (0, 140, 255)), ("goals", (200, 90, 20))]
        for k, (n, c) in enumerate(leg):
            x0 = m + 150 * k
            cv2.rectangle(img, (x0, h - 22), (x0 + 14, h - 10), c, -1)
            cv2.putText(img, n, (x0 + 20, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 30, 30), 1, cv2.LINE_AA)
        return img

    def altitude(self):
        return line_plot([("barometer", [x[0] for x in self.baro], [x[1] for x in self.baro], (120, 120, 235)),
                          ("ToF (down)", [x[0] for x in self.tof], [x[1] for x in self.tof], (230, 120, 0)),
                          ("ground truth", [g[0] for g in self.gt], [g[3] for g in self.gt], (0, 0, 0))],
                         "Height above floor: ground truth vs onboard sensors", "z [m]")

    def finish(self, why):
        self.done = True
        self.get_logger().info(f"stopping: {why}")
        for v in self.vid.values():
            v.close()
        smap, alt = self.scan_map(), self.altitude()
        cv2.imwrite(os.path.join(self.out, "scan_map.png"), smap)
        cv2.imwrite(os.path.join(self.out, "altitude.png"), alt)

        tile = (800, 450)

        def fit(img, name):
            if img is None:
                img = np.full((tile[1], tile[0], 3), 60, np.uint8)
                label(img, f"{name}: no frames received", (20, tile[1] // 2))
            fh, fw = img.shape[:2]
            k = min(tile[0] / fw, tile[1] / fh)
            r = cv2.resize(img, (int(fw * k), int(fh * k)), interpolation=cv2.INTER_AREA)
            out = np.full((tile[1], tile[0], 3), 255, np.uint8)
            y, x = (tile[1] - r.shape[0]) // 2, (tile[0] - r.shape[1]) // 2
            out[y:y + r.shape[0], x:x + r.shape[1]] = r
            return out

        grid = np.vstack([np.hstack([fit(self.vid["overview"].still(0.6), "overview"),
                                     fit(self.vid["onboard"].still(0.4), "onboard")]),
                          np.hstack([fit(smap, "scan map"), fit(alt, "altitude")])])
        bar = np.full((50, grid.shape[1], 3), 35, np.uint8)
        label(bar, f"scenario {self.info['scenario']}  |  stack {self.stack}: stereo, 2D lidar, ToF, barometer, IMU, "
                   f"optical flow", (12, 33), 0.7)
        cv2.imwrite(os.path.join(self.out, "summary.png"), np.vstack([bar, grid]))

        lines = [f"{'topic':<14}{'msgs':>8}{'rate [Hz]':>12}"]
        for k in sorted(self.counts):
            span = self.last[k] - self.first[k]
            lines.append(f"{k:<14}{self.counts[k]:>8}{(self.counts[k] - 1) / span if span > 0 else 0:>12.1f}")
        for k in ("overview", "topdown", "stereo_left", "scan", "tof_down", "baro", "flow_camera", "odom_gt"):
            if k not in self.counts:
                lines.append(f"{k:<14}{'0':>8}   <- nothing received")
        lines.append(f"sim time recorded: {self.gt[-1][0] if self.gt else 0:.1f} s, "
                     f"wall time: {time.time() - self.wall0:.0f} s")
        open(os.path.join(self.out, "sensors.txt"), "w").write("\n".join(lines) + "\n")
        print("\n".join(lines))


def main():
    rclpy.init()
    n = DemoCapture()
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
