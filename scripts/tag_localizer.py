#!/usr/bin/env python3
"""Known-map localization from AprilTag detections (the "do we even need SLAM?" candidate).

apriltag_ros publishes TF <camera optical frame> -> "<family>:<id>" for each detected tag. This node
takes the detected tag CENTRES (it ignores tag orientation, which avoids any tag-frame convention or
texture-orientation mismatch), puts them in the gravity-aligned body frame using the static camera
mount + IMU roll/pitch, and solves x, y, z, yaw against the known positions in landmarks.yaml
(>= 2 tags; 2D Umeyama for yaw). Publishes PoseWithCovarianceStamped on /<agent>/tag_pose in `map`.

Params: agent, landmarks_file, static_tf_file, camera_frame (optical), imu_topic, min_tags, max_range.
"""
import math
import re

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
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


def rpy_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def quat_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class TagLocalizer(Node):
    def __init__(self):
        super().__init__("tag_localizer")
        P = lambda n, d: param(self, n, d)  # noqa: E731
        a = P("agent", "blimp")
        self.cam = P("camera_frame", f"{a}/stereo_left_optical")
        self.min_tags = P("min_tags", 2)
        self.max_range = P("max_range", 12.0)
        with open(P("landmarks_file", "generated/arena_solo/landmarks.yaml")) as f:
            self.tags = {l["id"]: np.array(l["xyz"]) for l in yaml.safe_load(f)["landmarks"] if l["type"] == "apriltag"}
        # chain base_link -> ... -> camera optical frame from the generated static TFs
        with open(P("static_tf_file", "generated/arena_solo/static_tf.yaml")) as f:
            tfs = {t["child"]: t for t in yaml.safe_load(f)["transforms"]}
        T, frame = np.eye(4), self.cam
        while frame in tfs:
            t = tfs[frame]
            Ti = np.eye(4)
            Ti[:3, :3], Ti[:3, 3] = rpy_R(*t["rpy"]), t["xyz"]
            T, frame = Ti @ T, t["parent"]
        self.T_b_c = T
        self.R_rp = np.eye(3)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, f"/{a}/tag_pose", 10)
        self.create_subscription(Imu, P("imu_topic", f"/{a}/imu"), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        self.n_pub = 0
        self.get_logger().info(f"{len(self.tags)} known tags, camera {self.cam} (mount resolved to {frame})")

    def on_imu(self, m):
        q = m.orientation
        R = quat_R(q.x, q.y, q.z, q.w)
        yaw = math.atan2(R[1, 0], R[0, 0])
        self.R_rp = rpy_R(0, 0, -yaw) @ R  # strip yaw: IMU yaw is not trusted (no magnetometer)

    def on_tf(self, msg):
        by_stamp = {}
        for t in msg.transforms:
            mt = re.search(r":(\d+)$", t.child_frame_id)
            if not mt or t.header.frame_id != self.cam or int(mt.group(1)) not in self.tags:
                continue
            p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, 1.0])
            if np.linalg.norm(p[:3]) > self.max_range:
                continue
            key = (t.header.stamp.sec, t.header.stamp.nanosec)
            by_stamp.setdefault(key, []).append((int(mt.group(1)), self.R_rp @ (self.T_b_c @ p)[:3]))
        for (sec, nsec), dets in by_stamp.items():
            if len(dets) >= self.min_tags:
                self.solve(sec, nsec, dets)

    def solve(self, sec, nsec, dets):
        src = np.array([d[1] for d in dets])  # gravity-aligned body frame
        dst = np.array([self.tags[d[0]] for d in dets])
        ms, md = src.mean(0), dst.mean(0)
        H = (src - ms)[:, :2].T @ (dst - md)[:, :2]
        yaw = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        Rz = rpy_R(0, 0, yaw)
        t = md - Rz @ ms
        res = np.linalg.norm((Rz @ src.T).T + t - dst, axis=1)
        if res.max() > 0.5:  # inconsistent detections (mis-ID or bad geometry)
            return
        R = Rz @ self.R_rp
        w = math.sqrt(max(1e-9, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
        q = ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
        m = PoseWithCovarianceStamped()
        m.header.stamp.sec, m.header.stamp.nanosec, m.header.frame_id = sec, nsec, "map"
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = t
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        var = max(0.01, float(np.mean(res ** 2))) / max(1, len(dets) - 1)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = var
        cov[21] = cov[28] = 1e-3
        cov[35] = var / max(0.5, float(np.linalg.norm(src[:, :2] - ms[:2], axis=1).mean())) ** 2
        m.pose.covariance = cov
        self.pub.publish(m)
        self.n_pub += 1
        if self.n_pub % 100 == 1:
            self.get_logger().info(f"pose from {len(dets)} tags, residual max {res.max():.3f} m")


def main():
    rclpy.init()
    try:
        rclpy.spin(TagLocalizer())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # normal when the launch file shuts down


if __name__ == "__main__":
    main()
