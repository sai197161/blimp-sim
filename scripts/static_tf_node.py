#!/usr/bin/env python3
"""Publish every static transform in a generated static_tf.yaml (base_link -> sensor frames).

  python3 scripts/static_tf_node.py generated/<scenario>/static_tf.yaml [agent] --ros-args -p use_sim_time:=true

With [agent], only transforms whose parent starts with "<agent>/" are published.
"""
import math
import sys

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if not args:
        sys.exit(__doc__)
    with open(args[0]) as f:
        tfs = yaml.safe_load(f)["transforms"]
    if len(args) > 1:
        tfs = [t for t in tfs if t["parent"].startswith(args[1] + "/")]
    rclpy.init()
    node = Node("static_tf_" + (args[1] if len(args) > 1 else "all"))
    br = StaticTransformBroadcaster(node)
    msgs = []
    for t in tfs:
        m = TransformStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = t["parent"], t["child"]
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (float(v) for v in t["xyz"])
        q = quat(*t["rpy"])
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
        msgs.append(m)
    br.sendTransform(msgs)
    node.get_logger().info(f"published {len(msgs)} static transforms")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
