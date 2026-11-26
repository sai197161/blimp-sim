#!/usr/bin/env python3
"""Broadcast an Odometry topic as TF parent -> child (renaming the frames).

Used for odom_source:=gt: /blimp/odom_gt -> TF blimp/odom -> blimp/base_link, so a SLAM method can map
with perfect odometry (an upper bound on map quality, separate from odometry drift).
Params: input, parent_frame, child_frame.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTF(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        self.parent = self.declare_parameter("parent_frame", "blimp/odom").value
        self.child = self.declare_parameter("child_frame", "blimp/base_link").value
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.declare_parameter("input", "/blimp/odom_gt").value, self.cb, 20)

    def cb(self, m):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = m.header.stamp, self.parent, self.child
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = q
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomTF())


if __name__ == "__main__":
    main()
