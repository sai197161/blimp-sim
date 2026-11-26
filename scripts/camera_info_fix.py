#!/usr/bin/env python3
"""Republish a CameraInfo with the stereo baseline term set: P[3] = Tx = -fx * baseline.

Gazebo renders each camera of the pair on its own and leaves Tx = 0 on the right camera, which stereo
consumers (rtabmap, stereo_image_proc, ...) read as zero baseline. build_scenario.py wires this in
for every stereo_camera: params input, output, tx.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
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


class Fix(Node):
    def __init__(self):
        super().__init__("camera_info_fix")
        self.tx = float(param(self, "tx", 0.0))
        self.pub = self.create_publisher(CameraInfo, param(self, "output", "camera_info"),
                                         qos_profile_sensor_data)
        self.create_subscription(CameraInfo, param(self, "input", "camera_info_gz"), self.cb,
                                 qos_profile_sensor_data)

    def cb(self, m):
        p = list(m.p)
        p[3] = self.tx
        m.p = p
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Fix())


if __name__ == "__main__":
    main()
