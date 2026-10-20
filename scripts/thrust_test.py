#!/usr/bin/env python3
"""Scripted flight test for the sim blimp.

Phases (thrust in N):
  0  hold      0-5 s   no thrust       -> checks ~neutral buoyancy (expect slow sink, < ~0.1 m)
  1  trim      5-10 s  vertical trim   -> should hold altitude
  2  forward  10-18 s  L=R=+0.06       -> accelerates forward, damping caps speed
  3  yaw      18-26 s  L=+0.04 R=-0.04 -> turns in place
  4  climb    26-32 s  vertical +0.05  -> climbs
  5  stop     32-38 s  trim only       -> settles

Also counts messages on every sensor topic and reports rates.
Writes thrust_test_<stamp>.csv (t, phase, x, y, z, yaw, vx, vy, vz).

Run while the sim is up:
  python3 scripts/thrust_test.py --ros-args -p use_sim_time:=true
"""
import csv, math, time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, LaserScan, PointCloud2

TRIM = 0.02  # N, offsets the ~2 g negative buoyancy in model.sdf
PHASES = [  # (name, duration_s, left, right, vertical)
    ("hold", 5, 0.0, 0.0, 0.0),
    ("trim", 5, 0.0, 0.0, TRIM),
    ("forward", 8, 0.06, 0.06, TRIM),
    ("yaw", 8, 0.04, -0.04, TRIM),
    ("climb", 6, 0.0, 0.0, TRIM + 0.05),
    ("stop", 6, 0.0, 0.0, TRIM),
]
SENSORS = {
    "/blimp/imu": Imu, "/blimp/front_camera/image_raw": Image,
    "/blimp/down_camera/image_raw": Image, "/blimp/range": LaserScan,
    "/blimp/lidar/points": PointCloud2,
}


class ThrustTest(Node):
    def __init__(self):
        super().__init__("blimp_thrust_test")
        self.pub = {k: self.create_publisher(Float64, f"/blimp/thrust/{k}", 10) for k in ("left", "right", "vertical")}
        self.odom = None
        self.create_subscription(Odometry, "/blimp/odom_gt", self._odom, 10)
        self.counts = {t: 0 for t in SENSORS}
        for t, typ in SENSORS.items():
            self.create_subscription(typ, t, lambda m, t=t: self.counts.__setitem__(t, self.counts[t] + 1), qos_profile_sensor_data)
        self.range_last = None
        self.create_subscription(LaserScan, "/blimp/range", self._range, qos_profile_sensor_data)

    def _odom(self, m): self.odom = m
    def _range(self, m): self.range_last = m.ranges[0] if m.ranges else None

    def send(self, l, r, v):
        for k, val in (("left", l), ("right", r), ("vertical", v)):
            self.pub[k].publish(Float64(data=float(val)))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    n = ThrustTest()
    print("waiting for /blimp/odom_gt ...")
    t0 = time.time()
    while n.odom is None and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.odom is None:
        raise SystemExit("No odometry — is the sim running and the bridge up? (ros2 topic list)")

    out = f"thrust_test_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    summary = []
    sensor_t0 = time.time(); n.counts = {t: 0 for t in SENSORS}
    with open(out, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "phase", "x", "y", "z", "yaw", "vx", "vy", "vz", "range"])
        start = time.time()
        for name, dur, l, r, v in PHASES:
            p0 = n.odom.pose.pose; y0 = yaw_of(p0.orientation)
            x0, yy0, z0 = p0.position.x, p0.position.y, p0.position.z
            end = time.time() + dur
            print(f"-- {name:8s} L={l:+.3f} R={r:+.3f} V={v:+.3f} N for {dur}s")
            while time.time() < end:
                n.send(l, r, v)
                rclpy.spin_once(n, timeout_sec=0.02)
                o = n.odom; p = o.pose.pose.position; tw = o.twist.twist.linear
                w.writerow([f"{time.time()-start:.3f}", name, f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}",
                            f"{yaw_of(o.pose.pose.orientation):.4f}", f"{tw.x:.4f}", f"{tw.y:.4f}", f"{tw.z:.4f}",
                            n.range_last])
            p = n.odom.pose.pose
            dyaw = math.degrees((yaw_of(p.orientation) - y0 + math.pi) % (2 * math.pi) - math.pi)
            summary.append((name, p.position.x - x0, p.position.y - yy0, p.position.z - z0, dyaw))
        n.send(0, 0, 0)
    el = time.time() - sensor_t0

    print("\n=== RESULTS ===")
    print(f"{'phase':9s} {'dx[m]':>7s} {'dy[m]':>7s} {'dz[m]':>7s} {'dyaw[deg]':>9s}")
    for s in summary:
        print(f"{s[0]:9s} {s[1]:7.2f} {s[2]:7.2f} {s[3]:7.2f} {s[4]:9.1f}")
    print("\nsensor rates (wall clock):")
    for t, c in n.counts.items():
        print(f"  {t:35s} {c/el:6.1f} Hz {'<-- NO DATA' if c == 0 else ''}")
    print(f"\nlog: {out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
