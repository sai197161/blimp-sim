#!/usr/bin/env python3
"""Barometer + downward ToF -> base_link height above the floor, as PoseWithCovarianceStamped (z only)
for robot_localization.

  baro: relative altitude from the first `baro_init_samples` readings (hypsometric formula), anchored to
        the first valid ToF height (or `initial_z` if there is no ToF). Publishes <agent>/altitude/baro.
  tof : range x cos(tilt) - sensor z offset. Publishes <agent>/altitude/tof. It measures height above
        whatever is under the blimp (a ball, a goal, an opponent), so the EKF config sets a rejection threshold.
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu, LaserScan


class Altitude(Node):
    def __init__(self):
        super().__init__("altitude")
        P = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        ns = P("agent", "blimp")
        self.frame = P("frame_id", f"{ns}/odom")
        self.tof_z = P("tof_z_offset", -0.42)  # ToF z in base_link (negative = below)
        self.baro_z = P("baro_z_offset", -0.39)
        self.initial_z = P("initial_z", 1.5)
        self.n_init = P("baro_init_samples", 50)
        self.baro_std = P("baro_std_m", 0.1)
        self.tilt_c = 1.0
        self.p0, self.p_acc, self.z0 = None, [], None
        self.tof_h = None
        self.pub_b = self.create_publisher(PoseWithCovarianceStamped, f"/{ns}/altitude/baro", 10)
        self.pub_t = self.create_publisher(PoseWithCovarianceStamped, f"/{ns}/altitude/tof", 10)
        if P("baro_topic", ""):
            self.create_subscription(FluidPressure, P("baro_topic", ""), self.on_baro, qos_profile_sensor_data)
        if P("tof_topic", ""):
            self.create_subscription(LaserScan, P("tof_topic", ""), self.on_tof, qos_profile_sensor_data)
        if P("imu_topic", ""):
            self.create_subscription(Imu, P("imu_topic", ""), self.on_imu, qos_profile_sensor_data)

    def msg(self, stamp, z, var):
        m = PoseWithCovarianceStamped()
        m.header.stamp, m.header.frame_id = stamp, self.frame
        m.pose.pose.position.z = z
        m.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        for i in (0, 7, 21, 28, 35):
            cov[i] = 1e3
        cov[14] = var
        m.pose.covariance = cov
        return m

    def on_imu(self, m):
        q = m.orientation
        self.tilt_c = max(0.5, 1 - 2 * (q.x * q.x + q.y * q.y))  # R33: cos of tilt from vertical

    def on_tof(self, m):
        r = m.ranges[0] if m.ranges else float("nan")
        if not (math.isfinite(r) and m.range_min < r < m.range_max):
            return
        self.tof_h = r * self.tilt_c - self.tof_z
        self.pub_t.publish(self.msg(m.header.stamp, self.tof_h, 0.02 ** 2))

    def on_baro(self, m):
        p = m.fluid_pressure
        if self.p0 is None:
            self.p_acc.append(p)
            if len(self.p_acc) >= self.n_init:
                self.p0 = sum(self.p_acc) / len(self.p_acc)
                self.z0 = self.tof_h if self.tof_h is not None else self.initial_z
                self.get_logger().info(f"baro reference {self.p0:.1f} Pa anchored at z={self.z0:.2f} m "
                                       f"({'ToF' if self.tof_h is not None else 'initial_z'})")
            return
        rel = 44330.0 * (1.0 - (p / self.p0) ** (1 / 5.255))
        self.pub_b.publish(self.msg(m.header.stamp, self.z0 + rel, self.baro_std ** 2))


def main():
    rclpy.init()
    rclpy.spin(Altitude())


if __name__ == "__main__":
    main()
