"""Generate a scenario and launch it: Gazebo world + bridge + TFs + sensor helpers + odometry.

  ros2 launch launch/scenario.launch.py scenario:=arena_1v1
  ros2 launch launch/scenario.launch.py scenario:=arena_solo ego_stack:=full_stack_lidar_top headless:=true
  ros2 launch launch/scenario.launch.py scenario:=arena_solo odom_source:=gt     # map with perfect odometry

Args:
  scenario     name in config/scenarios/ or a YAML path                         (default arena_solo)
  ego_stack    override the ego's sensor stack (placement / ablation sweeps)    (default: as in scenario)
  headless     no Gazebo GUI                                                    (default false)
  odom_source  ekf  -> robot_localization fuses IMU + flow + baro + ToF -> TF <agent>/odom -> base_link
               gt   -> ground truth republished as that TF (upper bound)
               none -> nothing; the SLAM method provides odometry itself (e.g. rtabmap visual odometry)
  others       behaviour of non-ego agents: patrol | hover | idle               (default patrol)
  flow_compare log optical flow vs ground truth every 5 s                        (default false)
"""
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch_ros.actions import Node

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPTS = os.path.join(ROOT, "scripts")


def _gen():
    spec = importlib.util.spec_from_file_location("build_scenario", os.path.join(SCRIPTS, "build_scenario.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def py(script, params=None, args=(), name=None):
    cmd = ["python3", os.path.join(SCRIPTS, script), *args, "--ros-args", "-p", "use_sim_time:=true"]
    for k, v in (params or {}).items():
        cmd += ["-p", f"{k}:={str(v).lower() if isinstance(v, bool) else v}"]
    if name:
        cmd += ["-r", f"__node:={name}"]
    return ExecuteProcess(cmd=cmd, output="screen")


def _append(var, path):
    old = os.environ.get(var, "")
    return path + (os.pathsep + old if old else "")


def launch_setup(context):
    arg = lambda n: context.launch_configurations[n]  # noqa: E731
    info = _gen().build(arg("scenario"), None, arg("ego_stack") or None)
    out = info["out_dir"]
    world = os.path.join(out, "world.sdf")
    headless = arg("headless").lower() == "true"
    odom_source = arg("odom_source")
    res = os.path.join(ROOT, "models") + os.pathsep + os.path.join(ROOT, "worlds")

    acts = [
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", _append("IGN_GAZEBO_RESOURCE_PATH", res)),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _append("GZ_SIM_RESOURCE_PATH", res)),
        ExecuteProcess(cmd=["ign", "gazebo", "-r", "-v", "3"] + (["-s", "--headless-rendering"] if headless else []) + [world],
                       output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="scenario_bridge",
             parameters=[{"config_file": os.path.join(out, "bridge.yaml"), "use_sim_time": True}], output="screen"),
    ]

    for ag in info["agents"]:
        a, sens = ag["name"], ag["sensors"]
        has_sensors = bool(ag["expected_topics"]) and any(sens.values())
        if has_sensors:
            acts.append(py("static_tf_node.py", args=[os.path.join(out, "static_tf.yaml"), a], name=f"static_tf_{a}"))
        for h in ag.get("helper_nodes", []):
            acts.append(py(h["script"], h["params"], name=f"{a}_{os.path.splitext(h['script'])[0]}"))

        if ag["role"] != "ego":
            beh = arg("others")
            if beh != "idle":
                acts.append(py("waypoint_flight.py", dict(info_file=os.path.join(out, "scenario_info.yaml"), agent=a,
                                                          pattern=beh, altitude=float(ag["pose"][2])), name=f"{a}_flight"))
            continue

        flow, tofs = sens.get("flow"), sens.get("tofs") or {}
        down_tof = next((n for n, t in tofs.items() if abs(t["pose"][4] - 1.5708) < 0.2), None)
        if flow:
            rng = flow.get("range_source") or down_tof
            imu = flow.get("imu_source") or (sens["imu"] or {}).get("name", "imu")
            acts.append(py("optical_flow_node.py", dict(
                image_topic=flow["image"], imu_topic=f"/{a}/{imu}", range_topic=f"/{a}/{rng}", twist_topic=flow["twist"],
                frame_id=f"{a}/base_link", hfov=flow["hfov"], compare_gt=arg("flow_compare").lower() == "true",
                gt_topic=f"/{a}/odom_gt"), name=f"{a}_optical_flow"))
        if sens.get("baro") or down_tof:
            p = dict(agent=a, initial_z=float(ag["pose"][2]))
            if sens.get("baro"):
                p.update(baro_topic=sens["baro"]["topic"], baro_z_offset=sens["baro"]["pose"][2],
                         baro_std_m=0.083 * sens["baro"]["noise_std_pa"])
            if down_tof:
                p.update(tof_topic=tofs[down_tof]["topic"], tof_z_offset=tofs[down_tof]["pose"][2])
            if sens.get("imu"):
                p["imu_topic"] = sens["imu"]["topic"]
            acts.append(py("altitude_node.py", p, name=f"{a}_altitude"))

        if odom_source == "ekf":
            ekf = os.path.join(out, f"ekf_{a}.yaml")
            if os.path.isfile(ekf):
                acts.append(Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
                                 namespace=a, parameters=[ekf], output="screen"))
            else:
                print(f"[scenario.launch] {a}: no IMU in stack, cannot run EKF (use odom_source:=gt)")
        elif odom_source == "gt":
            acts.append(py("odom_to_tf.py", dict(input=f"/{a}/odom_gt", parent_frame=f"{a}/odom",
                                                 child_frame=f"{a}/base_link"), name=f"{a}_gt_tf"))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="arena_solo"),
        DeclareLaunchArgument("ego_stack", default_value=""),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("odom_source", default_value="ekf", choices=["ekf", "gt", "none"]),
        DeclareLaunchArgument("others", default_value="patrol", choices=["patrol", "hover", "idle"]),
        DeclareLaunchArgument("flow_compare", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
