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


class Fix(Node):
    def __init__(self):
        super().__init__("camera_info_fix")
        self.tx = float(self.declare_parameter("tx", 0.0).value)
        self.pub = self.create_publisher(CameraInfo, self.declare_parameter("output", "camera_info").value,
                                         qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.declare_parameter("input", "camera_info_gz").value, self.cb,
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
