"""Start one SLAM / localization method against a running scenario (see config/slam/methods.yaml).

  ros2 launch launch/slam.launch.py method:=slam_toolbox
  ros2 launch launch/slam.launch.py method:=rtabmap_stereo gen_dir:=generated/arena_solo__vision_only

The scenario must be running with the odom_source the method expects (methods.yaml), e.g.
  ros2 launch launch/scenario.launch.py scenario:=arena_solo odom_source:=none   # for rtabmap_stereo
scripts/run_benchmark.sh does the pairing for you.

Args: method, agent (default blimp), gen_dir (generated scenario dir, for apriltag), rviz (false).
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CFG = os.path.join(ROOT, "config", "slam")


def rtabmap_nodes(a, stereo, scan, vo):
    """rtabmap_ros >= 0.21 (split packages: rtabmap_odom / rtabmap_slam). Params are strings, as rtabmap expects."""
    st = f"/{a}/stereo"
    cam_remaps = [("left/image_rect", f"{st}/left/image_raw"), ("right/image_rect", f"{st}/right/image_raw"),
                  ("left/camera_info", f"{st}/left/camera_info"), ("right/camera_info", f"{st}/right/camera_info")]
    common = dict(frame_id=f"{a}/base_link", use_sim_time=True, approx_sync=False, qos=1)
    nodes = []
    if vo:
        nodes.append(Node(package="rtabmap_odom", executable="stereo_odometry", name="stereo_odometry", output="screen",
                          parameters=[dict(common, odom_frame_id=f"{a}/odom", publish_tf=True, wait_imu_to_init=True,
                                           **{"Odom/Strategy": "0", "Vis/MinInliers": "12", "GFTT/MinDistance": "7",
                                              "Odom/ResetCountdown": "1", "Reg/Force3DoF": "false"})],
                          remappings=cam_remaps + [("imu", f"/{a}/imu"), ("odom", f"/{a}/rtabmap/odom")]))
    p = dict(common, map_frame_id="map", subscribe_stereo=stereo, subscribe_depth=False, subscribe_rgb=False,
             subscribe_scan=scan, subscribe_odom_info=vo, wait_imu_to_init=True,
             **{"Mem/IncrementalMemory": "true", "Rtabmap/DetectionRate": "1", "RGBD/OptimizeMaxError": "3.0",
                "Grid/RangeMax": "12", "Reg/Force3DoF": "true" if not stereo else "false"})
    remaps = [("imu", f"/{a}/imu")]
    if vo:
        remaps.append(("odom", f"/{a}/rtabmap/odom"))
    else:
        p["odom_frame_id"] = f"{a}/odom"  # take odometry from TF (EKF), so only images need syncing
    if stereo:
        remaps += cam_remaps
    if scan:
        remaps.append(("scan", f"/{a}/scan"))
        p.update({"Reg/Strategy": "2" if stereo else "1", "RGBD/NeighborLinkRefining": "true",
                  "RGBD/ProximityBySpace": "true", "Icp/VoxelSize": "0.05", "Icp/MaxCorrespondenceDistance": "0.3",
                  "Icp/PointToPlane": "false", "Grid/Sensor": "0" if not stereo else "2"})
        if not stereo:
            p["approx_sync"] = True
    nodes.append(Node(package="rtabmap_slam", executable="rtabmap", name="rtabmap", output="screen", parameters=[p],
                      remappings=remaps, arguments=["-d"]))  # -d: fresh database every run
    return nodes


def setup(context):
    arg = lambda n: context.launch_configurations[n]  # noqa: E731
    method, a = arg("method"), arg("agent")
    with open(os.path.join(CFG, "methods.yaml")) as f:
        methods = yaml.safe_load(f)
    if method not in methods:
        raise RuntimeError(f"unknown method '{method}'. Known: {', '.join(methods)}")
    m = methods[method]
    print(f"[slam.launch] {method}: {m['description']}  (expects scenario odom_source:={m['odom_source']})")
    n = []
    if method == "ekf_only":
        pass  # the scenario's EKF is the estimate
    elif method == "slam_toolbox":
        n.append(Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox", output="screen",
                      parameters=[os.path.join(CFG, "slam_toolbox.yaml"),
                                  dict(use_sim_time=True, odom_frame=f"{a}/odom", base_frame=f"{a}/base_link",
                                       map_frame="map", scan_topic=f"/{a}/scan")]))
    elif method.startswith("cartographer_2d"):
        lua = "cartographer_2d_odom.lua" if method == "cartographer_2d" else "cartographer_2d_imu.lua"
        n.append(Node(package="cartographer_ros", executable="cartographer_node", name="cartographer_node", output="screen",
                      parameters=[dict(use_sim_time=True)],
                      arguments=["-configuration_directory", CFG, "-configuration_basename", lua],
                      remappings=[("scan", f"/{a}/scan"), ("imu", f"/{a}/imu"), ("odom", f"/{a}/odometry/filtered")]))
        n.append(Node(package="cartographer_ros", executable="cartographer_occupancy_grid_node", output="screen",
                      parameters=[dict(use_sim_time=True)], arguments=["-resolution", "0.05"]))
    elif method == "rtabmap_stereo":
        n += rtabmap_nodes(a, stereo=True, scan=False, vo=True)
    elif method == "rtabmap_stereo_ekf":
        n += rtabmap_nodes(a, stereo=True, scan=False, vo=False)
    elif method == "rtabmap_stereo_scan":
        n += rtabmap_nodes(a, stereo=True, scan=True, vo=True)
    elif method == "rtabmap_scan":
        n += rtabmap_nodes(a, stereo=False, scan=True, vo=False)
    elif method == "apriltag":
        lm = yaml.safe_load(open(os.path.join(arg("gen_dir"), "landmarks.yaml")))["landmarks"]
        size = next((x["size"] for x in lm if x["type"] == "apriltag"), 0.4688)
        n.append(Node(package="apriltag_ros", executable="apriltag_node", name="apriltag", output="screen",
                      parameters=[dict(use_sim_time=True, family="36h11", size=size, max_hamming=0, detector=dict(threads=2))],
                      remappings=[("image_rect", f"/{a}/stereo/left/image_raw"),
                                  ("camera_info", f"/{a}/stereo/left/camera_info")]))
        n.append(ExecuteProcess(output="screen", cmd=[
            "python3", os.path.join(ROOT, "scripts", "tag_localizer.py"), "--ros-args", "-p", "use_sim_time:=true",
            "-p", f"agent:={a}", "-p", f"landmarks_file:={os.path.join(arg('gen_dir'), 'landmarks.yaml')}",
            "-p", f"static_tf_file:={os.path.join(arg('gen_dir'), 'static_tf.yaml')}"]))
    if arg("rviz").lower() == "true":
        n.append(Node(package="rviz2", executable="rviz2", parameters=[dict(use_sim_time=True)]))
    return n


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("method", default_value="slam_toolbox"),
        DeclareLaunchArgument("agent", default_value="blimp"),
        DeclareLaunchArgument("gen_dir", default_value=os.path.join(ROOT, "generated", "arena_solo")),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=setup),
    ])
