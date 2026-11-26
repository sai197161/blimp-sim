#!/usr/bin/env python3
"""Optical-flow sensor emulator (PMW3901-style) for the downward flow camera.

Gazebo has no optical-flow sensor, so the stack renders a small downward camera and this node turns
consecutive frames into body-frame velocity, the way a flow sensor + rangefinder would:

  pixel shift (phase correlation) -> angular flow (rad/s) -> subtract gyro rotation -> x range = m/s

Publishes geometry_msgs/TwistWithCovarianceStamped (vx, vy in <agent>/base_link) for the EKF, and
geometry_msgs/Vector3Stamped raw angular flow (x, y rad/s; z = quality 0..1) on <twist>_raw.

Params: image_topic, imu_topic, range_topic, twist_topic, frame_id, hfov, min_quality,
        flip_x / flip_y (only if your mount differs from the generated one), compare_gt + gt_topic:
        log RMSE vs ground truth every 5 s (use this to sanity-check signs and scale).

Image geometry (generated mount = pitch +90 deg, i.e. looking down, image top = forward):
  image right = body -y, image down = body -x. Ground features therefore move image-down when the
  blimp moves +x and image-right when it moves +y.
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
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


def to_gray(msg):
    buf = np.frombuffer(msg.data, np.uint8)
    if msg.encoding in ("mono8", "8UC1"):
        return buf.reshape(msg.height, msg.step)[:, : msg.width].astype(np.float32)
    img = buf.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).astype(np.float32)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class OpticalFlow(Node):
    def __init__(self):
        super().__init__("optical_flow")
        P = lambda n, d: param(self, n, d)  # noqa: E731
        self.hfov = P("hfov", 0.733)
        self.min_q = P("min_quality", 0.15)
        self.fx = -1.0 if P("flip_x", False) else 1.0
        self.fy = -1.0 if P("flip_y", False) else 1.0
        self.frame = P("frame_id", "blimp/base_link")
        self.pub = self.create_publisher(TwistWithCovarianceStamped, P("twist_topic", "/blimp/flow/twist"), 10)
        self.pub_raw = self.create_publisher(Vector3Stamped, P("twist_topic", "/blimp/flow/twist") + "_raw", 10)
        self.create_subscription(Image, P("image_topic", "/blimp/flow/camera/image_raw"), self.on_image, qos_profile_sensor_data)
        self.create_subscription(Imu, P("imu_topic", "/blimp/imu"), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, P("range_topic", "/blimp/tof_down"), self.on_range, qos_profile_sensor_data)
        self.prev = None
        self.prev_t = None
        self.window = None
        self.gyro_acc = np.zeros(3)  # integrated body rates since last frame (rad)
        self.last_imu_t = None
        self.range = None
        self.compare = P("compare_gt", False)
        if self.compare:
            self.err, self.gt_prev, self.gt_v = [], None, None
            self.create_subscription(Odometry, P("gt_topic", "/blimp/odom_gt"), self.on_gt, 20)
            self.create_timer(5.0, self.report)

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.last_imu_t is not None and 0 < t - self.last_imu_t < 0.1:
            self.gyro_acc += np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]) * (t - self.last_imu_t)
        self.last_imu_t = t

    def on_range(self, m):
        r = m.ranges[0] if m.ranges else float("nan")
        self.range = r if math.isfinite(r) and m.range_min < r < m.range_max else None

    def on_image(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        img = to_gray(m)
        if self.window is None or self.window.shape != img.shape:
            self.window = cv2.createHanningWindow(img.shape[::-1], cv2.CV_32F)
        if self.prev is None:
            self.prev, self.prev_t, self.gyro_acc[:] = img, t, 0
            return
        dt = t - self.prev_t
        if dt <= 0:
            return
        (du, dv), quality = cv2.phaseCorrelate(self.prev, img, self.window)
        rot = self.gyro_acc.copy()
        self.prev, self.prev_t, self.gyro_acc[:] = img, t, 0
        ifov = self.hfov / img.shape[1]  # rad per pixel
        flow_x = self.fx * dv * ifov / dt  # rad/s, "as if moving +x"
        flow_y = self.fy * du * ifov / dt
        p, q = rot[0] / dt, rot[1] / dt  # roll / pitch rate over the frame interval
        raw = Vector3Stamped()
        raw.header = m.header
        raw.vector.x, raw.vector.y, raw.vector.z = flow_x, flow_y, float(quality)
        self.pub_raw.publish(raw)
        if self.range is None or quality < self.min_q:
            return
        h = self.range
        vx, vy = h * (flow_x + q), h * (flow_y - p)  # de-rotate: pitch-up looks like moving -x, roll-right like +y
        out = TwistWithCovarianceStamped()
        out.header.stamp = m.header.stamp
        out.header.frame_id = self.frame
        out.twist.twist.linear.x, out.twist.twist.linear.y = vx, vy
        sig = 0.03 + 0.1 * math.hypot(vx, vy) + 0.05 * (1 - quality)
        cov = [0.0] * 36
        cov[0] = cov[7] = sig * sig
        cov[14] = cov[21] = cov[28] = cov[35] = 1e3
        out.twist.covariance = cov
        self.pub.publish(out)
        if self.compare and self.gt_v is not None:
            self.err.append((vx - self.gt_v[0], vy - self.gt_v[1], self.gt_v[0], self.gt_v[1], vx, vy))

    def on_gt(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        if self.gt_prev is not None and t > self.gt_prev[0]:
            dt = t - self.gt_prev[0]
            wx, wy = (p.x - self.gt_prev[1]) / dt, (p.y - self.gt_prev[2]) / dt
            yaw = yaw_of(m.pose.pose.orientation)
            c, s = math.cos(yaw), math.sin(yaw)
            self.gt_v = (c * wx + s * wy, -s * wx + c * wy)  # world -> body (yaw only)
        self.gt_prev = (t, p.x, p.y)

    def report(self):
        if len(self.err) < 10:
            self.get_logger().info("compare_gt: not enough flow samples yet (moving? range valid?)")
            return
        e = np.array(self.err)
        rmse = np.sqrt((e[:, :2] ** 2).mean(axis=0))
        corr = [np.corrcoef(e[:, 2 + i], e[:, 4 + i])[0, 1] if e[:, 2 + i].std() > 1e-3 else float("nan") for i in (0, 1)]
        self.get_logger().info(f"flow vs GT over {len(e)} samples: rmse vx={rmse[0]:.3f} vy={rmse[1]:.3f} m/s, "
                               f"corr vx={corr[0]:+.2f} vy={corr[1]:+.2f} (negative corr -> set flip_x/flip_y)")
        self.err.clear()


def main():
    rclpy.init()
    try:
        rclpy.spin(OpticalFlow())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # normal when the launch file shuts down


if __name__ == "__main__":
    main()
