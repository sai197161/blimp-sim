#!/usr/bin/env python3
"""Broadcast an Odometry topic as TF parent -> child (renaming the frames).

Used for odom_source:=gt: /blimp/odom_gt -> TF blimp/odom -> blimp/base_link, so a SLAM method can map
with perfect odometry (an upper bound on map quality, separate from odometry drift).
Params: input, parent_frame, child_frame.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
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


class OdomTF(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        self.parent = param(self, "parent_frame", "blimp/odom")
        self.child = param(self, "child_frame", "blimp/base_link")
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, param(self, "input", "/blimp/odom_gt"), self.cb, 20)

    def cb(self, m):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = m.header.stamp, self.parent, self.child
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = q
        self.br.sendTransform(t)


def main():
    rclpy.init()
    try:
        rclpy.spin(OdomTF())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # normal when the launch file shuts down
    except Exception:  # Humble shutdown race: RCLError 'context is not valid' after Ctrl-C
        if rclpy.ok():
            raise


if __name__ == "__main__":
    main()
