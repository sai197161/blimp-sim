#!/usr/bin/env python3
"""Record ground truth + an estimator's pose to TUM files for eval_traj.py (or evo).

  python3 scripts/traj_recorder.py --ros-args -p use_sim_time:=true -p out_dir:=results/run1 \
      -p est_mode:=tf -p map_frame:=map -p base_frame:=blimp/base_link

est_mode:
  tf    look up map_frame -> base_frame at `rate` Hz (works for any SLAM that publishes TF)
  odom  subscribe nav_msgs/Odometry on est_topic
  pose  subscribe geometry_msgs/PoseStamped or PoseWithCovarianceStamped on est_topic (set est_type)
Writes gt.tum, est.tum (t x y z qx qy qz qw) and, if psutil is installed and cpu_pattern is set,
cpu.csv (t, cpu %, rss MB summed over processes whose command line contains cpu_pattern).
Stop with Ctrl-C / SIGINT; files are flushed as it goes.
"""
import os
import signal
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from rcl_interfaces.msg import ParameterDescriptor

try:
    import psutil
except ImportError:
    psutil = None


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


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def line(t, p, q):
    return f"{t:.6f} {p.x:.5f} {p.y:.5f} {p.z:.5f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"


class Recorder(Node):
    def __init__(self):
        super().__init__("traj_recorder")
        P = lambda n, d: param(self, n, d)  # noqa: E731
        out = P("out_dir", "results/run")
        os.makedirs(out, exist_ok=True)
        self.f_gt = open(os.path.join(out, "gt.tum"), "w")
        self.f_est = open(os.path.join(out, "est.tum"), "w")
        self.n_gt = self.n_est = 0
        self.create_subscription(Odometry, P("gt_topic", "/blimp/odom_gt"), self.on_gt, 50)
        mode = P("est_mode", "tf")
        self.last_est_t = None
        if mode == "tf":
            self.map, self.base = P("map_frame", "map"), P("base_frame", "blimp/base_link")
            self.buf = Buffer()
            self.tl = TransformListener(self.buf, self)
            self.create_timer(1.0 / P("rate", 20.0), self.poll_tf)
        elif mode == "odom":
            self.create_subscription(Odometry, P("est_topic", "/blimp/odometry/filtered"),
                                     lambda m: self.write_est(stamp(m.header), m.pose.pose), 50)
        elif mode == "pose":
            typ = PoseWithCovarianceStamped if P("est_type", "PoseStamped") == "PoseWithCovarianceStamped" else PoseStamped
            self.create_subscription(typ, P("est_topic", "/pose"), lambda m: self.write_est(
                stamp(m.header), m.pose.pose if typ is PoseWithCovarianceStamped else m.pose), 50)
        self.cpu_pat = P("cpu_pattern", "")
        if self.cpu_pat and psutil:
            self.f_cpu = open(os.path.join(out, "cpu.csv"), "w")
            self.f_cpu.write("t,cpu_percent,rss_mb\n")
            self.procs = {}
            self.create_timer(1.0, self.poll_cpu)
        self.create_timer(10.0, lambda: self.get_logger().info(f"gt={self.n_gt} est={self.n_est} poses"))

    def on_gt(self, m):
        self.f_gt.write(line(stamp(m.header), m.pose.pose.position, m.pose.pose.orientation))
        self.n_gt += 1

    def write_est(self, t, pose):
        if self.last_est_t is not None and t <= self.last_est_t:
            return
        self.last_est_t = t
        self.f_est.write(line(t, pose.position, pose.orientation))
        self.n_est += 1
        if self.n_est % 50 == 0:
            self.f_est.flush(); self.f_gt.flush()

    def poll_tf(self):
        try:
            tf = self.buf.lookup_transform(self.map, self.base, Time())
        except Exception:
            return
        tr = tf.transform
        self.write_est(stamp(tf.header), SimpleNamespace(position=tr.translation, orientation=tr.rotation))

    def poll_cpu(self):
        me = os.getpid()
        for p in psutil.process_iter(["pid", "cmdline"]):
            if p.info["pid"] != me and p.info["pid"] not in self.procs and self.cpu_pat in " ".join(p.info["cmdline"] or []):
                self.procs[p.info["pid"]] = p
                p.cpu_percent(None)
        cpu = rss = 0.0
        for pid, p in list(self.procs.items()):
            try:
                cpu += p.cpu_percent(None)
                rss += p.memory_info().rss / 1e6
            except psutil.Error:
                self.procs.pop(pid)
        self.f_cpu.write(f"{self.get_clock().now().nanoseconds * 1e-9:.2f},{cpu:.1f},{rss:.1f}\n")
        self.f_cpu.flush()

    def close(self):
        for f in (self.f_gt, self.f_est, getattr(self, "f_cpu", None)):
            if f:
                f.close()


def main():
    rclpy.init()
    n = Recorder()
    def on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.close()
        print(f"recorded gt={n.n_gt} est={n.n_est}")


if __name__ == "__main__":
    main()
