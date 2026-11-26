#!/usr/bin/env python3
"""Closed-loop waypoint flight on ground-truth odometry: a repeatable trajectory for SLAM benchmarks,
and scripted behaviour for opponents.

  python3 scripts/waypoint_flight.py --ros-args -p use_sim_time:=true \
      -p info_file:=generated/arena_solo/scenario_info.yaml -p agent:=blimp -p pattern:=lawnmower

The controller flies on GROUND TRUTH on purpose, so every SLAM method sees the same trajectory; the
estimator under test is only observed (see traj_recorder.py).

Patterns (built from the arena size in scenario_info.yaml, margin `margin` m from the walls):
  lawnmower  rows along x covering the arena, then back to start (loop closure)  ~4-6 min
  loop       2 laps of the perimeter rectangle                                   ~3 min
  loop3d     perimeter lap with altitude changing 1.5 <-> 3.5 m                   ~2 min
  figure8    two circles through the centre                                      ~2 min
  spin       hover at the start and yaw 2 turns each way (hard case for VO)      ~1 min
  patrol     back and forth between own half and centre forever (opponents)
  hover      hold the spawn pose forever
Exits 0 when a finite pattern is done (run_benchmark.sh relies on that).
"""
import math

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import ParameterDescriptor


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


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def make_pattern(name, L, W, start, z, margin):
    x0, x1, y0, y1 = -L / 2 + margin, L / 2 - margin, -W / 2 + margin, W / 2 - margin
    sx, sy, sz = start[:3]
    if name == "lawnmower":
        n_rows = max(2, int((y1 - y0) // 2.5) + 1)
        ys = [y0 + i * (y1 - y0) / (n_rows - 1) for i in range(n_rows)]
        wps = []
        for i, y in enumerate(ys):
            xa, xb = (x0, x1) if i % 2 == 0 else (x1, x0)
            wps += [(xa, y, z), (xb, y, z)]
        return [(sx, sy, z)] + wps + [(sx, sy, z)], False
    if name == "loop":
        rect = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
        return [(sx, sy, z)] + rect * 2 + [(x0, y0, z), (sx, sy, z)], False
    if name == "loop3d":
        rect = [(x0, y0, 1.5), (0, y0, 3.5), (x1, y0, 2.0), (x1, 0, 3.0), (x1, y1, 1.5), (0, y1, 3.5), (x0, y1, 2.0)]
        return [(sx, sy, z)] + rect + [(x0, y0, z), (sx, sy, z)], False
    if name == "figure8":
        r = min(L / 4 - margin / 2, W / 2 - margin)
        pts = []
        for cx, d in ((-r, 1), (r, -1)):
            for k in range(13):
                a = math.pi * 2 * k / 12 * d
                pts.append((cx + r * math.cos(a) * (-d), r * math.sin(a), z))
        return [(sx, sy, z)] + pts + [(sx, sy, z)], False
    if name == "spin":
        return [("spin", sx, sy, z)], False
    if name == "patrol":
        side = 1 if sx > 0 else -1
        return [(side * (L / 2 - margin - 1), sy, z), (side * 1.0, -sy if sy else 2.0, z + 0.5)], True
    if name == "hover":
        return [(sx, sy, sz)], True
    raise SystemExit(f"unknown pattern '{name}'")


class Flight(Node):
    def __init__(self):
        super().__init__("waypoint_flight")
        P = lambda n, d: param(self, n, d)  # noqa: E731
        self.agent = P("agent", "blimp")
        info = {}
        if P("info_file", ""):
            with open(P("info_file", "")) as f:
                info = yaml.safe_load(f)
        ag = next((a for a in info.get("agents", []) if a["name"] == self.agent), {})
        arena = info.get("arena", dict(length=20, width=12))
        self.trim = P("trim", float(ag.get("trim_thrust", 0.02)))
        start = ag.get("pose", [0, 0, 1.5, 0])
        self.wps, self.repeat = make_pattern(P("pattern", "lawnmower"), arena["length"], arena["width"], start,
                                             P("altitude", 2.0), P("margin", 2.5))
        self.radius = P("arrive_radius", 0.6)
        self.f_max = P("max_forward", 0.06)  # N per side
        self.timeout = P("timeout", 900.0)
        self.i = 0
        self.state = None
        self.prev = None
        self.z_int = 0.0
        self.spin_start = None
        self.t0 = None
        self.done = False
        self.pub = {k: self.create_publisher(Float64, f"/{self.agent}/thrust/{k}", 10) for k in ("left", "right", "vertical")}
        self.create_subscription(Odometry, f"/{self.agent}/odom_gt", self.on_odom, 20)
        self.get_logger().info(f"{self.agent}: {len(self.wps)} waypoints, trim {self.trim:.3f} N, repeat={self.repeat}")

    def send(self, l, r, v):
        for k, val in (("left", l), ("right", r), ("vertical", v)):
            self.pub[k].publish(Float64(data=float(val)))

    def on_odom(self, m):
        if self.done:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        yaw = yaw_of(m.pose.pose.orientation)
        if self.t0 is None:
            self.t0 = t
        if self.prev is None or t <= self.prev[0]:
            self.prev = (t, p.x, p.y, p.z, yaw)
            return
        dt = t - self.prev[0]
        vx, vy, vz = (p.x - self.prev[1]) / dt, (p.y - self.prev[2]) / dt, (p.z - self.prev[3]) / dt
        r = wrap(yaw - self.prev[4]) / dt
        self.prev = (t, p.x, p.y, p.z, yaw)

        wp = self.wps[self.i]
        if wp[0] == "spin":
            _, tx, ty, tz = wp
            self.spin_start = self.spin_start or t
            el = t - self.spin_start
            yaw_rate_cmd = 0.35 if el < 36 else (-0.35 if el < 72 else 0.0)
            u = max(-0.04, min(0.04, 0.06 * (yaw_rate_cmd - r)))
            fwd, arrived = 0.0, el > 80
        else:
            tx, ty, tz = wp
            dx, dy = tx - p.x, ty - p.y
            dist = math.hypot(dx, dy)
            e = wrap(math.atan2(dy, dx) - yaw) if dist > 0.3 else 0.0
            u = max(-0.04, min(0.04, 0.03 * e - 0.06 * r))
            v_fwd = math.cos(yaw) * vx + math.sin(yaw) * vy
            final = self.i == len(self.wps) - 1
            scale = min(1.0, dist / 2.0) if final else max(0.4, min(1.0, dist / 1.5))
            fwd = max(0.0, self.f_max * scale * max(0.0, math.cos(e)) ** 2 - (0.05 * v_fwd if final else 0.0))
            arrived = dist < self.radius and abs(tz - p.z) < 0.4

        ez = tz - p.z
        self.z_int = max(-2.0, min(2.0, self.z_int + ez * dt))
        vert = max(-0.2, min(0.3, self.trim + 0.08 * ez - 0.10 * vz + 0.01 * self.z_int))
        self.send(fwd - u, fwd + u, vert)  # R > L yaws left (+z)

        if arrived:
            self.i += 1
            self.spin_start = None
            if self.i >= len(self.wps):
                if self.repeat:
                    self.i = 0
                else:
                    self.finish("pattern complete")
            else:
                self.get_logger().info(f"waypoint {self.i}/{len(self.wps)} t={t - self.t0:.0f}s")
        if t - self.t0 > self.timeout:
            self.finish(f"timeout after {self.timeout:.0f}s at waypoint {self.i}/{len(self.wps)}")

    def finish(self, why):
        self.send(0, 0, self.trim)
        self.done = True
        self.get_logger().info(why)


def main():
    rclpy.init()
    n = Flight()
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.send(0, 0, n.trim)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
