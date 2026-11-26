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


def make_pattern(name, L, W, start, z, margin, margin_x=None):
    # margin_x keeps turns away from the goal hoops at both ends (they sit ~1.5 m from the end walls)
    mx = margin if margin_x is None else margin_x
    x0, x1, y0, y1 = -L / 2 + mx, L / 2 - mx, -W / 2 + margin, W / 2 - margin
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
    if name == "track":
        # rounded "running track": two straights joined by half circles. No sharp corners, so a slow-turning
        # blimp can follow it; laps until the timeout. Starts on the lower straight heading +x.
        rad = min((y1 - y0) / 2, 2.2)  # flown path sits ~0.4 m outside this, i.e. y = +-2.6: clear of the pillars
        xa, xb, yc = x0 + rad, x1 - rad, (y0 + y1) / 2
        pts = [(xa, y0, z), (xb, y0, z)]  # straights are single segments (line-of-sight guidance follows them)
        pts += [(xb + rad * math.sin(a), yc - rad * math.cos(a), z) for a in (math.pi * k / 6 for k in range(1, 6))]
        pts += [(xb, y1, z), (xa, y1, z)]
        pts += [(xa - rad * math.sin(a), yc + rad * math.cos(a), z) for a in (math.pi * k / 6 for k in range(1, 6))]
        return pts, True
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
                                             P("altitude", 2.0), P("margin", 3.0),
                                             P("margin_x", 3.5))
        self.radius = P("arrive_radius", 0.6)
        self.f_max = P("max_forward", 0.06)  # N per side
        self.v_cruise = P("cruise_speed", 0.45)  # m/s
        self.lookahead = P("lookahead", 1.5)  # m
        self.k_yaw, self.k_rate = P("k_yaw", 0.05), P("k_rate", 0.08)  # N per side per rad, per rad/s
        self.v_corner = P("corner_speed", 0.12)  # m/s at a 90+ deg turn; straight-through waypoints keep cruise
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

    def corner_speed(self, i):
        """Allowed speed at waypoint i from the turn angle to the next one."""
        n = len(self.wps)
        if i + 1 >= n and not self.repeat or i == 0:
            return self.v_corner
        a, b, c = self.wps[i - 1], self.wps[i], self.wps[(i + 1) % n]
        if "spin" in (a[0], b[0], c[0]):
            return 0.0
        turn = abs(wrap(math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(b[1] - a[1], b[0] - a[0])))
        k = min(1.0, turn / (math.pi / 3))  # 60 deg or more: corner speed
        return self.v_cruise * (1 - k) + self.v_corner * k

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
            prev = self.wps[self.i - 1] if (self.i > 0 or self.repeat) else None
            if prev is not None and prev[0] == "spin":
                prev = None
            # line-of-sight guidance: steer at a point `lookahead` ahead on the segment prev -> wp, so the blimp
            # follows the line instead of bowing out after every turn
            ax, ay = tx, ty
            if prev is not None:
                sx_, sy_ = tx - prev[0], ty - prev[1]
                seg = math.hypot(sx_, sy_)
                if seg > 1e-3:
                    along = ((p.x - prev[0]) * sx_ + (p.y - prev[1]) * sy_) / seg
                    k = min(seg, max(0.0, along) + self.lookahead) / seg
                    ax, ay = prev[0] + sx_ * k, prev[1] + sy_ * k
            e = wrap(math.atan2(ay - p.y, ax - p.x) - yaw) if math.hypot(ax - p.x, ay - p.y) > 0.3 else 0.0
            u = max(-0.04, min(0.04, self.k_yaw * e - self.k_rate * r))
            v_fwd = math.cos(yaw) * vx + math.sin(yaw) * vy
            final = self.i == len(self.wps) - 1 and not self.repeat
            # speed we may have at this waypoint: slow for sharp turns, stop at the last one
            v_wp = 0.0 if final else self.corner_speed(self.i)
            v_des = min(self.v_cruise, v_wp + 0.2 * dist) * max(0.0, math.cos(e)) ** 2
            # drag feed-forward (per side) + speed feedback; negative thrust brakes
            ff = (0.05 * v_des + 0.15 * v_des * v_des) / 2
            fwd = max(-0.04, min(self.f_max, ff + 0.12 * (v_des - v_fwd)))
            passed = (not final and prev is not None and prev[0] != "spin" and dist < 1.5
                      and (tx - prev[0]) * dx + (ty - prev[1]) * dy < 0)  # already beyond it along the path
            arrived = (dist < self.radius and abs(tz - p.z) < 0.4) or passed

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
