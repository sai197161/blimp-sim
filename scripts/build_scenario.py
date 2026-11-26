#!/usr/bin/env python3
"""Scenario generator: scenario YAML (+ sensor-stack YAMLs) -> runnable Gazebo world + ROS glue.

  python3 scripts/build_scenario.py config/scenarios/arena_1v1.yaml      # or just: arena_1v1
  python3 scripts/build_scenario.py arena_1v1 --out /tmp/gen

The launch file (launch/scenario.launch.py) calls this automatically. Outputs, in
generated/<scenario>/ :
  world.sdf              arena (or blimp_test room) + every agent inlined as its own blimp model
  bridge.yaml            ros_gz_bridge config for all agents
  static_tf.yaml         <agent>/base_link -> every sensor frame (incl. camera optical frames)
  scenario_info.yaml     arena size, agents, trim thrust, mass budget, expected topics + rates
  landmarks.yaml         ground-truth pose of every AprilTag / goal / pillar (for localization + eval)
  ekf_<agent>.yaml       robot_localization config matching that agent's stack
  <agent>_camchain.yaml  Kalibr-format stereo+IMU calibration (OpenVINS / ORB-SLAM3 / VINS)
  <agent>_imu.yaml       Kalibr-format IMU noise

Only depends on PyYAML (ships with ROS 2). No Gazebo needed to run it.
"""
import argparse
import copy
import math
import os
import random
import re
import sys

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
STACK_DIR = os.path.join(ROOT, "config", "sensor_stacks")
SCEN_DIR = os.path.join(ROOT, "config", "scenarios")
RHO_AIR = 1.225
RHO_HE = 0.1786
G = 9.81
TEX = "model://arena_assets/materials/textures"

# ---------------------------------------------------------------- pose math (SDF: R = Rz*Ry*Rx)


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def R_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2][0])))
    return [math.atan2(R[2][1], R[2][2]), pitch, math.atan2(R[1][0], R[0][0])]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def compose(a, b):
    """Pose b expressed in frame a -> pose in a's parent. Poses are [x y z r p y]."""
    Ra, Rb = rpy_to_R(*a[3:]), rpy_to_R(*b[3:])
    t = [x + y for x, y in zip(matvec(Ra, b[:3]), a[:3])]
    return t + R_to_rpy(matmul(Ra, Rb))


def fmt(v, nd=4):
    return " ".join(f"{x:.{nd}f}".rstrip("0").rstrip(".") if abs(x) > 1e-12 else "0" for x in v)


def mat_rgba(rgb, a=1.0):
    c = fmt(list(rgb) + [a], 3)
    return f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>"


def tex_material(uri, tint=(1, 1, 1)):
    c = fmt(list(tint) + [1], 3)
    return (f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse><specular>0.05 0.05 0.05 1</specular>"
            f"<pbr><metal><albedo_map>{uri}</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material>")


def noise(std, kind="gaussian"):
    return f'<noise type="{kind}"><mean>0</mean><stddev>{std}</stddev></noise>'


# ---------------------------------------------------------------- sensor stacks

SENSOR_DEFAULTS = {
    "imu": dict(rate=200, gyro_std=0.002, accel_std=0.02, gyro_bias_std=0.0, accel_bias_std=0.0, mass=0.001),
    "camera": dict(rate=30, width=640, height=480, hfov=1.3963, format="R8G8B8", noise_std=0.007,
                   near=0.05, far=50.0, mass=0.003),
    "stereo_camera": dict(rate=20, width=640, height=400, hfov=1.5708, format="R8G8B8", noise_std=0.007,
                          baseline=0.06, near=0.05, far=40.0, mass=0.015),
    "optical_flow": dict(rate=60, width=64, height=64, hfov=0.733, format="L8", near=0.05, far=30.0,
                         mass=0.002, range_source=None, imu_source=None),
    "tof": dict(rate=50, zones=[1, 1], fov=0.47, range=[0.04, 4.0], noise_std=0.005, mass=0.002),
    "lidar_2d": dict(rate=10, samples=450, fov=2 * math.pi, range=[0.1, 12.0], noise_std=0.01, mass=0.042),
    "lidar_3d": dict(rate=10, samples=900, channels=16, vfov=0.5236, range=[0.3, 30.0], noise_std=0.01,
                     mass=0.007),
    "barometer": dict(rate=50, noise_std_pa=1.0, mass=0.001),
}
STACK_DEFAULTS = dict(gondola_base_mass=0.025,
                      hull=dict(scale=1.0, auto_scale=False, target_net_mass=-0.002))
BASE_HULL = dict(radius=0.30, length=1.00, envelope_mass=0.303 - 0.2827 * RHO_HE, prop_mass=0.002)


def deep_update(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        out[k] = deep_update(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def load_raw_stack(name):
    path = name if os.path.isfile(name) else os.path.join(STACK_DIR, f"{name}.yaml")
    with open(path) as f:
        st = yaml.safe_load(f)
    return apply_variant(load_raw_stack(st["extends"]), st) if st.get("extends") else st


def apply_variant(base, var):
    """A stack with `extends: <other>` may set: modify: {sensor_name: {fields...}}, remove: [names], add: [sensors],
    plus any top-level key (hull, gondola_base_mass, ...)."""
    out = copy.deepcopy(base)
    for k, v in var.items():
        if k not in ("extends", "modify", "remove", "add", "sensors"):
            out[k] = deep_update(out.get(k), v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    names = [s["name"] for s in out.get("sensors", [])]
    for n, fields in (var.get("modify") or {}).items():
        if n not in names:
            sys.exit(f"[stack {var.get('name')}] modify: no sensor '{n}' in base (has: {', '.join(names)})")
        i = names.index(n)
        out["sensors"][i] = deep_update(out["sensors"][i], fields)
    out["sensors"] = [s for s in out.get("sensors", []) if s["name"] not in (var.get("remove") or [])]
    out["sensors"] += var.get("add") or []
    return out


def load_stack(name):
    if name in (None, "none"):
        return deep_update(STACK_DEFAULTS, dict(name="none", description="body only, no sensors", sensors=[],
                                                hull=dict(auto_scale=True)))
    path = name if os.path.isfile(name) else os.path.join(STACK_DIR, f"{name}.yaml")
    with open(path) as f:
        st = yaml.safe_load(f)
    st.setdefault("name", os.path.splitext(os.path.basename(path))[0])
    if st.get("extends"):
        st = apply_variant(load_raw_stack(st["extends"]), st)
    st = deep_update(STACK_DEFAULTS, st)
    sensors = []
    for s in st.get("sensors", []):
        if s.get("enabled", True) is False:
            continue
        if s["type"] not in SENSOR_DEFAULTS:
            sys.exit(f"[stack {st['name']}] unknown sensor type '{s['type']}' (known: {', '.join(SENSOR_DEFAULTS)})")
        full = deep_update(SENSOR_DEFAULTS[s["type"]], s)
        full.setdefault("mount", "gondola")
        full.setdefault("pose", [0, 0, 0, 0, 0, 0])
        sensors.append(full)
    st["sensors"] = sensors
    return st


# ---------------------------------------------------------------- hull sizing / mass budget


def hull_geom(s):
    r, L = BASE_HULL["radius"] * s, BASE_HULL["length"] * s
    vol = math.pi * r * r * L  # the Buoyancy system only sees the cylinder collision (+ gondola box)
    return r, L, vol


def mass_budget(stack, s):
    r, L, vol = hull_geom(s)
    gond_vol = 0.12 * 0.08 * 0.05
    hull_mass = BASE_HULL["envelope_mass"] * s * s + vol * RHO_HE
    sensor_mass = sum(x["mass"] for x in stack["sensors"])
    gondola_mass = stack["gondola_base_mass"] + sensor_mass
    total = hull_mass + gondola_mass + 3 * BASE_HULL["prop_mass"]
    lift = (vol + gond_vol) * RHO_AIR
    return dict(scale=s, hull_radius=r, hull_length=L, hull_mass=hull_mass, gondola_mass=gondola_mass,
                sensor_mass=sensor_mass, total_mass=total, buoyant_mass=lift, net_mass=lift - total)


def solve_scale(stack):
    h = stack["hull"]
    if not h.get("auto_scale"):
        return float(h.get("scale", 1.0))
    target = float(h.get("target_net_mass", -0.002))
    lo, hi = 0.5, 3.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if mass_budget(stack, mid)["net_mass"] < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def mounts(s):
    r, L, _ = hull_geom(s)
    gz = -(r + 0.06)
    return {"hull": [0, 0, 0], "hull_nose": [L / 2 + r, 0, 0], "hull_top": [0, 0, r], "hull_bottom": [0, 0, -r],
            "hull_tail": [-(L / 2 + r), 0, 0], "gondola": [0, 0, gz], "gondola_bottom": [0, 0, gz - 0.025],
            "gondola_front": [0.06, 0, gz]}


# ---------------------------------------------------------------- sensor -> SDF + bridge + TF

OPTICAL = [0, 0, 0, -math.pi / 2, 0, -math.pi / 2]


class AgentOut:
    def __init__(self, agent):
        self.a = agent
        self.sdf, self.bridges, self.tfs, self.topics = [], [], [], {}
        self.cameras = {}  # name -> intrinsics, for calib files
        self.ros_nodes = []  # helper nodes the launch file should start

    def frame(self, n):
        return f"{self.a}/{n}"

    def bridge(self, ros, gz, ros_t, gz_t, direction="GZ_TO_ROS", rate=None):
        self.bridges.append(dict(ros_topic_name=ros, gz_topic_name=gz, ros_type_name=ros_t,
                                 gz_type_name=gz_t, direction=direction))
        if direction == "GZ_TO_ROS" and rate:
            self.topics[ros] = dict(type=ros_t, rate=rate)

    def tf(self, parent, child, pose):
        self.tfs.append(dict(parent=parent, child=child, xyz=[round(v, 6) for v in pose[:3]],
                             rpy=[round(v, 6) for v in pose[3:]]))

    def body_visual(self, name, pose, size, offset, rgb=(0.1, 0.1, 0.1)):
        p = compose(pose, offset)
        self.sdf.append(f'<visual name="{name}_body"><pose>{fmt(p)}</pose><geometry><box><size>{fmt(size)}</size></box>'
                        f'</geometry>{mat_rgba(rgb)}</visual>')


def camera_sdf(out, sname, pose, c, topic_base, rgb=(0.05, 0.05, 0.05)):
    """One pinhole camera. ROS: /<agent>/<topic_base>/image_raw + camera_info, frame <..>_optical."""
    link = out.frame(f"{sname}_link")
    optical = out.frame(f"{sname}_optical")
    gz_topic = f"{out.a}/{topic_base}/image"  # Fortress derives camera_info as <dirname(topic)>/camera_info
    out.sdf.append(f"""<sensor name="{sname}" type="camera">
  <pose>{fmt(pose)}</pose><always_on>1</always_on><update_rate>{c['rate']}</update_rate>
  <topic>{gz_topic}</topic><ignition_frame_id>{optical}</ignition_frame_id>
  <camera>
    <horizontal_fov>{c['hfov']}</horizontal_fov>
    <image><width>{c['width']}</width><height>{c['height']}</height><format>{c['format']}</format></image>
    <clip><near>{c['near']}</near><far>{c['far']}</far></clip>
    {f"<noise><type>gaussian</type><mean>0</mean><stddev>{c['noise_std']}</stddev></noise>" if c.get('noise_std') else ""}
  </camera>
</sensor>""")
    out.body_visual(sname, pose, [0.015, 0.025, 0.02], [-0.012, 0, 0, 0, 0, 0], rgb)
    out.tf(out.frame("base_link"), link, pose)
    out.tf(link, optical, OPTICAL)
    out.bridge(f"/{out.a}/{topic_base}/image_raw", gz_topic, "sensor_msgs/msg/Image", "ignition.msgs.Image",
               rate=c["rate"])
    out.bridge(f"/{out.a}/{topic_base}/camera_info", f"{out.a}/{topic_base}/camera_info",
               "sensor_msgs/msg/CameraInfo", "ignition.msgs.CameraInfo", rate=c["rate"])
    fx = (c["width"] / 2) / math.tan(c["hfov"] / 2)
    out.cameras[topic_base] = dict(fx=fx, fy=fx, cx=c["width"] / 2, cy=c["height"] / 2, width=c["width"],
                                   height=c["height"], pose=pose, frame=optical,
                                   image=f"/{out.a}/{topic_base}/image_raw")


def lidar_sdf(out, s, pose, h_samples, h_min, h_max, v_samples, v_min, v_max, rmin, rmax):
    return f"""<sensor name="{s['name']}" type="gpu_lidar">
  <pose>{fmt(pose)}</pose><always_on>1</always_on><update_rate>{s['rate']}</update_rate>
  <topic>{out.a}/{s['name']}</topic><ignition_frame_id>{out.frame(s['name'] + '_link')}</ignition_frame_id>
  <lidar>
    <scan>
      <horizontal><samples>{h_samples}</samples><resolution>1</resolution><min_angle>{h_min:.5f}</min_angle><max_angle>{h_max:.5f}</max_angle></horizontal>
      <vertical><samples>{v_samples}</samples><resolution>1</resolution><min_angle>{v_min:.5f}</min_angle><max_angle>{v_max:.5f}</max_angle></vertical>
    </scan>
    <range><min>{rmin}</min><max>{rmax}</max><resolution>0.001</resolution></range>
    <noise><type>gaussian</type><mean>0</mean><stddev>{s['noise_std']}</stddev></noise>
  </lidar>
</sensor>"""


def add_sensor(out, s, mnt):
    if s["mount"] not in mnt:
        sys.exit(f"sensor {s['name']}: unknown mount '{s['mount']}' (known: {', '.join(mnt)})")
    pose = [m + p for m, p in zip(mnt[s["mount"]], s["pose"][:3])] + list(s["pose"][3:])
    n, t, a = s["name"], s["type"], out.a
    link = out.frame(f"{n}_link")

    if t == "imu":
        g, ac = s["gyro_std"], s["accel_std"]
        gb, ab = s.get("gyro_bias_std", 0), s.get("accel_bias_std", 0)

        def ax(std, bias):
            extra = f"<bias_mean>0</bias_mean><bias_stddev>{bias}</bias_stddev>" if bias else ""
            return f'<noise type="gaussian"><mean>0</mean><stddev>{std}</stddev>{extra}</noise>'
        out.sdf.append(f"""<sensor name="{n}" type="imu">
  <pose>{fmt(pose)}</pose><always_on>1</always_on><update_rate>{s['rate']}</update_rate>
  <topic>{a}/{n}</topic><ignition_frame_id>{link}</ignition_frame_id>
  <imu>
    <angular_velocity><x>{ax(g, gb)}</x><y>{ax(g, gb)}</y><z>{ax(g, gb)}</z></angular_velocity>
    <linear_acceleration><x>{ax(ac, ab)}</x><y>{ax(ac, ab)}</y><z>{ax(ac, ab)}</z></linear_acceleration>
  </imu>
</sensor>""")
        out.tf(out.frame("base_link"), link, pose)
        out.bridge(f"/{a}/{n}", f"{a}/{n}", "sensor_msgs/msg/Imu", "ignition.msgs.IMU", rate=s["rate"])
        out.imu = dict(name=n, topic=f"/{a}/{n}", frame=link, pose=pose, rate=s["rate"], gyro_std=g, accel_std=ac,
                       gyro_bias_std=gb, accel_bias_std=ab)

    elif t == "barometer":
        out.sdf.append(f"""<sensor name="{n}" type="air_pressure">
  <pose>{fmt(pose)}</pose><always_on>1</always_on><update_rate>{s['rate']}</update_rate>
  <topic>{a}/{n}</topic><ignition_frame_id>{link}</ignition_frame_id>
  <air_pressure><reference_altitude>0</reference_altitude><pressure>{noise(s['noise_std_pa'])}</pressure></air_pressure>
</sensor>""")
        out.tf(out.frame("base_link"), link, pose)
        out.bridge(f"/{a}/{n}", f"{a}/{n}", "sensor_msgs/msg/FluidPressure", "ignition.msgs.FluidPressure",
                   rate=s["rate"])
        out.baro = dict(name=n, topic=f"/{a}/{n}", pose=pose, noise_std_pa=s["noise_std_pa"])

    elif t == "camera":
        camera_sdf(out, n, pose, s, n)

    elif t == "stereo_camera":
        b = s["baseline"]
        for side, dy in (("left", b / 2), ("right", -b / 2)):
            camera_sdf(out, f"{n}_{side}", compose(pose, [0, dy, 0, 0, 0, 0]), s, f"{n}/{side}")
        out.body_visual(n + "_bar", pose, [0.01, b + 0.03, 0.01], [-0.02, 0, 0, 0, 0, 0], (0.2, 0.2, 0.2))
        out.stereo = dict(name=n, baseline=b, rate=s["rate"])
        # right camera_info from Gazebo has P[3] (Tx) = 0; republish with Tx = -fx*b for stereo consumers
        out.bridges[-1]["ros_topic_name"] = f"/{a}/{n}/right/camera_info_gz"
        out.topics.pop(f"/{a}/{n}/right/camera_info", None)
        fx = out.cameras[f"{n}/right"]["fx"]
        out.ros_nodes.append(dict(script="camera_info_fix.py",
                                  params=dict(input=f"/{a}/{n}/right/camera_info_gz",
                                              output=f"/{a}/{n}/right/camera_info", tx=-fx * b)))
        out.topics[f"/{a}/{n}/right/camera_info"] = dict(type="sensor_msgs/msg/CameraInfo", rate=s["rate"])

    elif t == "optical_flow":
        camera_sdf(out, n, pose, s, f"{n}/camera", rgb=(0.1, 0.4, 0.1))
        out.flow = dict(name=n, image=f"/{a}/{n}/camera/image_raw", hfov=s["hfov"], width=s["width"],
                        range_source=s.get("range_source"), imu_source=s.get("imu_source"),
                        twist=f"/{a}/{n}/twist")
        out.topics[f"/{a}/{n}/twist"] = dict(type="geometry_msgs/msg/TwistWithCovarianceStamped", rate=s["rate"])

    elif t == "tof":
        zx, zy = s["zones"]
        half = s["fov"] / 2
        hmin, hmax = (-half, half) if zx > 1 else (0.0, 0.0)
        vmin, vmax = (-half, half) if zy > 1 else (0.0, 0.0)
        out.sdf.append(lidar_sdf(out, s, pose, zx, hmin, hmax, zy, vmin, vmax, *s["range"]))
        out.body_visual(n, pose, [0.004, 0.015, 0.012], [-0.004, 0, 0, 0, 0, 0], (0.6, 0.1, 0.6))
        out.tf(out.frame("base_link"), link, pose)
        out.bridge(f"/{a}/{n}", f"{a}/{n}", "sensor_msgs/msg/LaserScan", "ignition.msgs.LaserScan", rate=s["rate"])
        if zx * zy > 1:
            out.bridge(f"/{a}/{n}/points", f"{a}/{n}/points", "sensor_msgs/msg/PointCloud2",
                       "ignition.msgs.PointCloudPacked", rate=s["rate"])
        out.tofs[n] = dict(topic=f"/{a}/{n}", pose=pose, zones=[zx, zy])

    elif t == "lidar_2d":
        half = s["fov"] / 2
        out.sdf.append(lidar_sdf(out, s, pose, s["samples"], -half, half, 1, 0, 0, *s["range"]))
        out.body_visual(n, pose, [0.04, 0.04, 0.025], [0, 0, -0.02, 0, 0, 0], (0.15, 0.15, 0.15))
        out.tf(out.frame("base_link"), link, pose)
        out.bridge(f"/{a}/{n}", f"{a}/{n}", "sensor_msgs/msg/LaserScan", "ignition.msgs.LaserScan", rate=s["rate"])
        out.scan = dict(name=n, topic=f"/{a}/{n}", frame=link)

    elif t == "lidar_3d":
        hv = s["vfov"] / 2
        out.sdf.append(lidar_sdf(out, s, pose, s["samples"], -math.pi, math.pi, s["channels"], -hv, hv, *s["range"]))
        out.body_visual(n, pose, [0.07, 0.07, 0.05], [0, 0, -0.035, 0, 0, 0], (0.15, 0.15, 0.15))
        out.tf(out.frame("base_link"), link, pose)
        out.bridge(f"/{a}/{n}/points", f"{a}/{n}/points", "sensor_msgs/msg/PointCloud2",
                   "ignition.msgs.PointCloudPacked", rate=s["rate"])


# ---------------------------------------------------------------- blimp model

TEAM_COLORS = {"red": (0.85, 0.15, 0.15), "blue": (0.15, 0.35, 0.9), "green": (0.2, 0.7, 0.3),
               "yellow": (0.95, 0.8, 0.1), "none": (0.9, 0.5, 0.1)}


def team_rgb(team):
    return tuple(team) if isinstance(team, (list, tuple)) else TEAM_COLORS.get(team, TEAM_COLORS["none"])


def blimp_model(agent, stack, pose, team):
    s = solve_scale(stack)
    mb = mass_budget(stack, s)
    r, L = mb["hull_radius"], mb["hull_length"]
    mnt = mounts(s)
    gz = mnt["gondola"][2]
    out = AgentOut(agent)
    out.imu = out.baro = out.flow = out.scan = out.stereo = None
    out.tofs = {}
    for sen in stack["sensors"]:
        add_sensor(out, sen, mnt)

    hm, gm = mb["hull_mass"], mb["gondola_mass"]
    ixx, iyy = 0.5 * hm * r * r, hm * (3 * r * r + L * L) / 12
    gi = [gm / 12 * (0.08 ** 2 + 0.05 ** 2), gm / 12 * (0.12 ** 2 + 0.05 ** 2), gm / 12 * (0.12 ** 2 + 0.08 ** 2)]
    white = mat_rgba((0.85, 0.85, 0.9))
    tc = mat_rgba(team_rgb(team))
    sensors_xml = "\n".join(out.sdf)
    ns = agent

    def prop(name, xyz, axis, rgb, rot):
        return f"""<link name="{name}">
      <pose>{fmt(xyz + [0, 0, 0])}</pose>
      <inertial><mass>{BASE_HULL['prop_mass']}</mass><inertia><ixx>2e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <visual name="v"><pose>{rot}</pose><geometry><cylinder><radius>0.05</radius><length>0.005</length></cylinder></geometry>{mat_rgba(rgb)}</visual>
    </link>
    <joint name="{name}_joint" type="revolute">
      <parent>gondola</parent><child>{name}</child>
      <axis><xyz>{axis}</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>"""

    def thruster(joint):
        return f"""<plugin filename="ignition-gazebo-thruster-system" name="ignition::gazebo::systems::Thruster">
      <namespace>{ns}</namespace><joint_name>{joint}</joint_name>
      <thrust_coefficient>0.004</thrust_coefficient><fluid_density>{RHO_AIR}</fluid_density><propeller_diameter>0.1</propeller_diameter>
      <p_gain>0.0001</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>
      <max_thrust_cmd>0.5</max_thrust_cmd><min_thrust_cmd>-0.5</min_thrust_cmd>
    </plugin>"""

    sdf = f"""<model name="{agent}">
    <!-- generated: stack={stack['name']} team={team} hull_scale={s:.3f} net_mass={mb['net_mass'] * 1000:+.1f} g -->
    <pose>{fmt(pose)}</pose>
    <link name="hull">
      <inertial><mass>{hm:.5f}</mass>
        <inertia><ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{iyy:.6f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="hull_collision"><pose>0 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>{r:.4f}</radius><length>{L:.4f}</length></cylinder></geometry></collision>
      <visual name="hull_visual"><pose>0 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>{r:.4f}</radius><length>{L:.4f}</length></cylinder></geometry>{white}</visual>
      <visual name="nose"><pose>{L / 2:.4f} 0 0 0 0 0</pose><geometry><sphere><radius>{r:.4f}</radius></sphere></geometry>{white}</visual>
      <visual name="tail"><pose>{-L / 2:.4f} 0 0 0 0 0</pose><geometry><sphere><radius>{r:.4f}</radius></sphere></geometry>{white}</visual>
      <visual name="fin_v"><pose>{-0.75 * s:.4f} 0 {0.25 * s:.4f} 0 0 0</pose><geometry><box><size>{fmt([0.2 * s, 0.01, 0.2 * s])}</size></box></geometry>{tc}</visual>
      <visual name="fin_h"><pose>{-0.75 * s:.4f} 0 0 0 0 0</pose><geometry><box><size>{fmt([0.2 * s, 0.6 * s, 0.01])}</size></box></geometry>{tc}</visual>
      <visual name="team_band"><pose>0 0 0 0 1.5708 0</pose><geometry><cylinder><radius>{r + 0.003:.4f}</radius><length>0.08</length></cylinder></geometry>{tc}</visual>
      {sensors_xml}
    </link>
    <link name="gondola">
      <pose>0 0 {gz:.4f} 0 0 0</pose>
      <inertial><mass>{gm:.5f}</mass><inertia><ixx>{gi[0]:.3e}</ixx><iyy>{gi[1]:.3e}</iyy><izz>{gi[2]:.3e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="gondola_collision"><geometry><box><size>0.12 0.08 0.05</size></box></geometry></collision>
      <visual name="gondola_visual"><geometry><box><size>0.12 0.08 0.05</size></box></geometry>{mat_rgba((0.2, 0.2, 0.2))}</visual>
      <visual name="arm"><geometry><box><size>0.02 0.54 0.01</size></box></geometry>{mat_rgba((0.3, 0.3, 0.3))}</visual>
    </link>
    <joint name="hull_gondola" type="fixed"><parent>hull</parent><child>gondola</child></joint>
    {prop("left_prop", [0, 0.27, gz], "1 0 0", (0.1, 0.4, 0.9), "0 0 0 0 1.5708 0")}
    {prop("right_prop", [0, -0.27, gz], "1 0 0", (0.1, 0.4, 0.9), "0 0 0 0 1.5708 0")}
    {prop("vert_prop", [-0.10, 0, gz - 0.04], "0 0 1", (0.9, 0.2, 0.2), "0 0 0 0 0 0")}
    {thruster("left_prop_joint")}
    {thruster("right_prop_joint")}
    {thruster("vert_prop_joint")}
    <plugin filename="ignition-gazebo-hydrodynamics-system" name="ignition::gazebo::systems::Hydrodynamics">
      <link_name>hull</link_name><water_density>{RHO_AIR}</water_density>
      <xDotU>0</xDotU><yDotV>0</yDotV><zDotW>0</zDotW><kDotP>0</kDotP><mDotQ>0</mDotQ><nDotR>0</nDotR>
      <xUabsU>-0.15</xUabsU><xU>-0.05</xU><yVabsV>-0.40</yVabsV><yV>-0.10</yV><zWabsW>-0.40</zWabsW><zW>-0.10</zW>
      <kPabsP>-0.005</kPabsP><kP>-0.005</kP><mQabsQ>-0.02</mQabsQ><mQ>-0.01</mQ><nRabsR>-0.02</nRabsR><nR>-0.01</nR>
    </plugin>
    <plugin filename="ignition-gazebo-odometry-publisher-system" name="ignition::gazebo::systems::OdometryPublisher">
      <odom_frame>{agent}/gt_odom</odom_frame><robot_base_frame>{agent}/gt_base_link</robot_base_frame>
      <dimensions>3</dimensions><odom_publish_frequency>50</odom_publish_frequency>
    </plugin>
  </model>"""

    out.bridge(f"/{agent}/odom_gt", f"/model/{agent}/odometry", "nav_msgs/msg/Odometry",
               "ignition.msgs.Odometry", rate=50)
    for k in ("left", "right", "vertical"):
        j = {"vertical": "vert"}.get(k, k)
        out.bridge(f"/{agent}/thrust/{k}", f"/model/{agent}/joint/{j}_prop_joint/cmd_thrust", "std_msgs/msg/Float64",
                   "ignition.msgs.Double", direction="ROS_TO_GZ")
    out.model_sdf, out.mass, out.scale = sdf, mb, s
    return out


# ---------------------------------------------------------------- arena world


def box_model(name, pose, size, material, collide=True, static=True):
    col = f'<collision name="c"><geometry><box><size>{fmt(size)}</size></box></geometry></collision>' if collide else ""
    return (f'<model name="{name}"><static>{str(static).lower()}</static><pose>{fmt(pose)}</pose><link name="l">{col}'
            f'<visual name="v"><geometry><box><size>{fmt(size)}</size></box></geometry>{material}</visual></link></model>')


def panel_pose(center, yaw, roll_deg=0.0):
    """Plane visual (normal +z) turned to face horizontal direction `yaw`, rotated in-plane by roll_deg."""
    R = matmul(rpy_to_R(0, 0, yaw), matmul(rpy_to_R(0, math.pi / 2, 0), rpy_to_R(0, 0, math.radians(roll_deg))))
    return list(center) + R_to_rpy(R)


def hoop_model(name, center, shape, size, thick, rgb):
    """Goal hoop in the y-z plane (balls go through along x) on a pole down to the floor."""
    n, R, off = {"circle": (20, size / 2, 0.0), "square": (4, size / math.sqrt(2), math.pi / 4),
                 "triangle": (3, size / math.sqrt(3), math.pi / 2)}[shape]
    verts = [(R * math.cos(off + 2 * math.pi * k / n), R * math.sin(off + 2 * math.pi * k / n)) for k in range(n)]
    parts, m = [], mat_rgba(rgb)
    for k in range(n):
        (y0, z0), (y1, z1) = verts[k], verts[(k + 1) % n]
        ln = math.hypot(y1 - y0, z1 - z0) + (thick if n <= 4 else 0.01)
        p = fmt([0, (y0 + y1) / 2, (z0 + z1) / 2, math.atan2(z1 - z0, y1 - y0), 0, 0])
        g = f"<geometry><box><size>{fmt([thick, ln, thick])}</size></box></geometry>"
        parts.append(f'<collision name="c{k}"><pose>{p}</pose>{g}</collision><visual name="v{k}"><pose>{p}</pose>{g}{m}</visual>')
    zb = center[2] + min(z for _, z in verts)  # pole runs up to the lowest vertex
    pole_h = max(zb, 0.05)
    g = f"<geometry><cylinder><radius>0.03</radius><length>{pole_h:.3f}</length></cylinder></geometry>"
    pp = fmt([0, 0, -(center[2] - pole_h / 2), 0, 0, 0])
    parts.append(f'<collision name="pole_c"><pose>{pp}</pose>{g}</collision><visual name="pole_v"><pose>{pp}</pose>{g}'
                 f'{mat_rgba((0.3, 0.3, 0.3))}</visual>')
    return (f'<model name="{name}"><static>true</static><pose>{fmt(list(center) + [0, 0, 0])}</pose>'
            f'<link name="l">{"".join(parts)}</link></model>')


def ball_model(name, pos, radius, rgb, net_mass):
    vol = 4 / 3 * math.pi * radius ** 3
    m = vol * RHO_AIR + net_mass  # neutral by default; net_mass>0 -> slowly sinks
    I = 0.4 * m * radius * radius
    g = f"<geometry><sphere><radius>{radius}</radius></sphere></geometry>"
    return f"""<model name="{name}"><pose>{fmt(list(pos) + [0, 0, 0])}</pose>
  <link name="l"><inertial><mass>{m:.6f}</mass><inertia><ixx>{I:.3e}</ixx><iyy>{I:.3e}</iyy><izz>{I:.3e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <collision name="c">{g}<surface><bounce><restitution_coefficient>0.4</restitution_coefficient></bounce></surface></collision>
    <visual name="v">{g}{mat_rgba(rgb)}</visual></link>
  <plugin filename="ignition-gazebo-hydrodynamics-system" name="ignition::gazebo::systems::Hydrodynamics">
    <link_name>l</link_name><water_density>{RHO_AIR}</water_density>
    <xDotU>0</xDotU><yDotV>0</yDotV><zDotW>0</zDotW><kDotP>0</kDotP><mDotQ>0</mDotQ><nDotR>0</nDotR>
    <xUabsU>-0.02</xUabsU><xU>-0.004</xU><yVabsV>-0.02</yVabsV><yV>-0.004</yV><zWabsW>-0.02</zWabsW><zW>-0.004</zW>
    <kPabsP>-1e-6</kPabsP><kP>-1e-6</kP><mQabsQ>-1e-6</mQabsQ><mQ>-1e-6</mQ><nRabsR>-1e-6</nRabsR><nR>-1e-6</nR>
  </plugin>
</model>"""


ARENA_DEFAULTS = dict(
    length=20.0, width=12.0, wall_height=6.0, ceiling=False, seed=7, floor_tile=2.5, court_lines=True,
    balls=dict(green=4, purple=4, radius=0.12, z_range=[1.0, 3.5], net_mass=0.0, keepout_x=3.0),
    goals=dict(shapes=["circle", "square", "triangle"], size=1.0, thickness=0.06, heights=[2.5, 3.5, 3.0],
               y_positions=[-3.0, 0.0, 3.0], inset=1.5, teams=["blue", "red"]),
    landmarks=dict(apriltags=True, tag_size=0.6, tag_spacing=3.0, tag_heights=[1.4, 2.6], tag_mirror=False,
                   tag_rotation_deg=0.0, posters=True, poster_size=1.2, poster_height=2.0,
                   pillars=[[-4.0, 4.0], [4.0, -4.0]], pillar_radius=0.25),
)

SYSTEMS = """<physics name="4ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-air-pressure-system" name="ignition::gazebo::systems::AirPressure"/>
    <plugin filename="ignition-gazebo-buoyancy-system" name="ignition::gazebo::systems::Buoyancy"><uniform_fluid_density>1.225</uniform_fluid_density></plugin>
    <gravity>0 0 -9.81</gravity>"""


def arena_world(name, cfg):
    A = deep_update(ARENA_DEFAULTS, cfg)
    L, W, H = A["length"], A["width"], A["wall_height"]
    rng = random.Random(A["seed"])
    m, landmarks = [], []

    # lighting: soft sun + ceiling lamps
    m.append("""<scene><ambient>0.55 0.55 0.55 1</ambient><background>0.75 0.78 0.82 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 20 0 0 0</pose>
      <diffuse>0.7 0.7 0.7 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.3 0.2 -0.9</direction></light>""")
    for i, (x, y) in enumerate([(sx * L / 4, sy * W / 4) for sx in (-1, 1) for sy in (-1, 1)]):
        m.append(f'<light type="point" name="lamp_{i}"><pose>{x} {y} {H - 0.3} 0 0 0</pose><diffuse>0.35 0.35 0.33 1</diffuse>'
                 f'<attenuation><range>{max(L, W)}</range><constant>0.6</constant><linear>0.02</linear><quadratic>0.0</quadratic></attenuation>'
                 f'<cast_shadows>false</cast_shadows></light>')

    # floor: one collision plane + textured tiles (randomly rotated so the flow camera doesn't see a repeating grid)
    tile = A["floor_tile"]
    nx, ny = max(1, round(L / tile)), max(1, round(W / tile))
    tx, ty = L / nx, W / ny
    vis = []
    for i in range(nx):
        for j in range(ny):
            cx, cy = -L / 2 + (i + 0.5) * tx, -W / 2 + (j + 0.5) * ty
            yaw = rng.choice([0, 1, 2, 3]) * math.pi / 2
            sz = (tx, ty) if yaw in (0, math.pi) else (ty, tx)
            vis.append(f'<visual name="t{i}_{j}"><pose>{fmt([cx, cy, 0.001, 0, 0, yaw])}</pose><geometry><plane><normal>0 0 1</normal>'
                       f'<size>{fmt(sz)}</size></plane></geometry>{tex_material(f"{TEX}/floor_{rng.randint(0, 1)}.jpg")}</visual>')
    if A["court_lines"]:
        lm = mat_rgba((0.95, 0.95, 0.95))
        for k, (p, sz) in enumerate([([0, 0], [0.06, W]), ([-L / 2 + 0.3, 0], [0.06, W - 0.6]),
                                     ([L / 2 - 0.3, 0], [0.06, W - 0.6]), ([0, W / 2 - 0.3], [L - 0.6, 0.06]),
                                     ([0, -W / 2 + 0.3], [L - 0.6, 0.06])]):
            vis.append(f'<visual name="line{k}"><pose>{fmt(p + [0.003, 0, 0, 0])}</pose><geometry><box><size>{fmt(sz + [0.002])}</size></box></geometry>{lm}</visual>')
        vis.append(f'<visual name="center_circle"><pose>0 0 0.0025 0 0 0</pose><geometry><cylinder><radius>1.5</radius><length>0.002</length></cylinder></geometry>{lm}</visual>')
        vis.append(f'<visual name="center_circle_in"><pose>0 0 0.003 0 0 0</pose><geometry><cylinder><radius>1.44</radius><length>0.002</length></cylinder></geometry>{tex_material(f"{TEX}/floor_0.jpg")}</visual>')
    m.append(f'<model name="floor"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal>'
             f'<size>{L + 2} {W + 2}</size></plane></geometry></collision>{"".join(vis)}</link></model>')

    # walls
    wt = 0.2
    wall_mat = tex_material(f"{TEX}/wall.jpg")
    for nm, pose, size in (("wall_px", [L / 2 + wt / 2, 0, H / 2], [wt, W + 2 * wt, H]),
                           ("wall_nx", [-L / 2 - wt / 2, 0, H / 2], [wt, W + 2 * wt, H]),
                           ("wall_py", [0, W / 2 + wt / 2, H / 2], [L, wt, H]),
                           ("wall_ny", [0, -W / 2 - wt / 2, H / 2], [L, wt, H])):
        m.append(box_model(nm, pose + [0, 0, 0], size, wall_mat))
    if A["ceiling"]:
        m.append(box_model("ceiling", [0, 0, H + 0.05, 0, 0, 0], [L + 0.4, W + 0.4, 0.1], mat_rgba((0.8, 0.8, 0.8))))

    # landmarks on the inside face of each wall: AprilTags + feature posters, alternating
    lmk = A["landmarks"]
    walls = [  # (start point, along-wall direction, inward yaw, wall length)
        ((L / 2 - 0.01, -W / 2), (0, 1), math.pi, W), ((-L / 2 + 0.01, W / 2), (0, -1), 0.0, W),
        ((L / 2, W / 2 - 0.01), (-1, 0), -math.pi / 2, L), ((-L / 2, -W / 2 + 0.01), (1, 0), math.pi / 2, L)]
    tag_id, poster_k, vis = 0, 0, []
    for (sx, sy), (dx, dy), yaw, wl in walls:
        n_slots = max(1, int(wl // lmk["tag_spacing"]))
        step = wl / n_slots
        for k in range(n_slots):
            d = (k + 0.5) * step
            px, py = sx + dx * d, sy + dy * d
            if lmk["apriltags"] and tag_id < 40:
                z = lmk["tag_heights"][k % len(lmk["tag_heights"])]
                c = [px + 0.004 * math.cos(yaw), py + 0.004 * math.sin(yaw), z]
                p = panel_pose(c, yaw, lmk["tag_rotation_deg"])
                suffix = "_m" if lmk["tag_mirror"] else ""
                vis.append(f'<visual name="tag{tag_id}"><pose>{fmt(p)}</pose><geometry><plane><normal>0 0 1</normal>'
                           f'<size>{lmk["tag_size"]} {lmk["tag_size"]}</size></plane></geometry>'
                           f'{tex_material(f"{TEX}/tag36h11_{tag_id}{suffix}.png")}</visual>')
                # printed texture is 512 px with a 56 px quiet zone; the black-border tag edge is 400/512 of it
                landmarks.append(dict(type="apriltag", family="tag36h11", id=tag_id, size=round(lmk["tag_size"] * 400 / 512, 4),
                                      xyz=[round(v, 4) for v in c], facing_yaw=round(yaw, 4)))
                tag_id += 1
            if lmk["posters"] and k < n_slots - 1:
                d2 = d + step / 2
                c = [sx + dx * d2 + 0.004 * math.cos(yaw), sy + dy * d2 + 0.004 * math.sin(yaw), lmk["poster_height"]]
                ps = lmk["poster_size"]
                vis.append(f'<visual name="poster{poster_k}"><pose>{fmt(panel_pose(c, yaw))}</pose><geometry><plane><normal>0 0 1</normal>'
                           f'<size>{ps} {ps}</size></plane></geometry>{tex_material(f"{TEX}/poster_{poster_k % 8}.jpg")}</visual>')
                poster_k += 1
    if vis:
        m.append(f'<model name="wall_landmarks"><static>true</static><link name="l">{"".join(vis)}</link></model>')

    for i, (x, y) in enumerate(lmk["pillars"] or []):
        rr = lmk["pillar_radius"]
        g = f"<geometry><cylinder><radius>{rr}</radius><length>{H}</length></cylinder></geometry>"
        m.append(f'<model name="pillar_{i}"><static>true</static><pose>{x} {y} {H / 2} 0 0 0</pose><link name="l">'
                 f'<collision name="c">{g}</collision><visual name="v">{g}{tex_material(f"{TEX}/poster_{(i + 3) % 8}.jpg")}</visual></link></model>')
        landmarks.append(dict(type="pillar", xyz=[x, y, H / 2], radius=rr))

    # goals: one row of hoops near each end wall, colored by the team that DEFENDS it
    gc = A["goals"]
    for side, team in zip((1, -1), gc["teams"]):
        x = side * (L / 2 - gc["inset"])
        for k, shape in enumerate(gc["shapes"]):
            y = gc["y_positions"][k % len(gc["y_positions"])]
            z = gc["heights"][k % len(gc["heights"])]
            m.append(hoop_model(f"goal_{team}_{shape}", [x, y, z], shape, gc["size"], gc["thickness"], team_rgb(team)))
            landmarks.append(dict(type="goal", team=team, shape=shape, xyz=[x, y, z], size=gc["size"]))

    # neutrally buoyant game balls in the middle zone
    bc = A["balls"]
    placed = []
    for color, rgb in (("green", (0.1, 0.8, 0.2)), ("purple", (0.55, 0.15, 0.75))):
        for i in range(int(bc.get(color, 0))):
            for _ in range(200):
                p = [rng.uniform(-L / 2 + bc["keepout_x"], L / 2 - bc["keepout_x"]), rng.uniform(-W / 2 + 1.5, W / 2 - 1.5),
                     rng.uniform(*bc["z_range"])]
                if all(math.dist(p, q) > 1.0 for q in placed) and \
                        all(math.dist(p[:2], q[:2]) > 2.0 for q in A.get("_avoid", [])):
                    break
            placed.append(p)
            m.append(ball_model(f"ball_{color}_{i}", [round(v, 3) for v in p], bc["radius"], rgb, bc["net_mass"]))

    body = "\n    ".join(m)
    world = f"""<?xml version="1.0"?>
<!-- GENERATED by scripts/build_scenario.py - edit the scenario YAML, not this file. -->
<sdf version="1.8">
  <world name="{name}">
    {SYSTEMS}
    {body}
    @@AGENTS@@
  </world>
</sdf>
"""
    return world, landmarks, A


def blimp_test_world():
    with open(os.path.join(ROOT, "worlds", "blimp_test.sdf")) as f:
        w = f.read()
    w = re.sub(r"<include>\s*<uri>model://blimp</uri>.*?</include>", "", w, flags=re.S)
    if "air-pressure-system" not in w:
        w = w.replace('<plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>',
                      '<plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>\n'
                      '    <plugin filename="ignition-gazebo-air-pressure-system" name="ignition::gazebo::systems::AirPressure"/>')
    w = w.replace("</world>", "  @@AGENTS@@\n  </world>")
    return w, [], dict(length=12.0, width=12.0, wall_height=3.0)


# ---------------------------------------------------------------- per-agent ROS configs


def ekf_yaml(out):
    a = out.a
    p = dict(frequency=30.0, sensor_timeout=0.2, two_d_mode=False, publish_tf=True, use_sim_time=True,
             map_frame="map", odom_frame=f"{a}/odom", base_link_frame=f"{a}/base_link", world_frame=f"{a}/odom",
             print_diagnostics=False)
    F = False
    if out.imu:
        # roll/pitch absolute (gravity-observable), yaw only via gyro rate: a real IMU without a magnetometer
        # cannot observe absolute yaw, and Gazebo's IMU orientation is near-perfect, which would hide drift.
        p.update(imu0=out.imu["topic"], imu0_config=[F, F, F, True, True, F, F, F, F, True, True, True, F, F, F],
                 imu0_remove_gravitational_acceleration=True, imu0_queue_size=20)
    if out.flow:
        p.update(twist0=out.flow["twist"], twist0_config=[F, F, F, F, F, F, True, True, F, F, F, F, F, F, F],
                 twist0_rejection_threshold=5.0)
    k = 0
    if out.baro:
        p.update({f"pose{k}": f"/{a}/altitude/baro", f"pose{k}_config": [F, F, True] + [F] * 12})
        k += 1
    if any(abs(t["pose"][4] - math.pi / 2) < 0.2 for t in out.tofs.values()):
        p.update({f"pose{k}": f"/{a}/altitude/tof", f"pose{k}_config": [F, F, True] + [F] * 12,
                  f"pose{k}_rejection_threshold": 3.0})
    return {"/**": {"ros__parameters": p}}  # wildcard: works whatever namespace/name the launch file gives the node


def T_of(pose):
    R = rpy_to_R(*pose[3:])
    return [R[0] + [pose[0]], R[1] + [pose[1]], R[2] + [pose[2]], [0, 0, 0, 1]]


def T_inv(T):
    R = [[T[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(R[i][k] * T[k][3] for k in range(3)) for i in range(3)]
    return [R[0] + [t[0]], R[1] + [t[1]], R[2] + [t[2]], [0, 0, 0, 1]]


def T_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def calib_yaml(out):
    if not (out.stereo and out.imu):
        return None, None
    n = out.stereo["name"]
    T_b_imu = T_of(out.imu["pose"])
    chain = {}
    for idx, side in enumerate(("left", "right")):
        c = out.cameras[f"{n}/{side}"]
        T_b_opt = T_mul(T_of(c["pose"]), T_of(OPTICAL))
        T_cam_imu = T_mul(T_inv(T_b_opt), T_b_imu)
        chain[f"cam{idx}"] = dict(
            T_cam_imu=[[round(v, 6) for v in row] for row in T_cam_imu], camera_model="pinhole",
            intrinsics=[round(c["fx"], 3), round(c["fy"], 3), c["cx"], c["cy"]], distortion_model="radtan",
            distortion_coeffs=[0.0, 0.0, 0.0, 0.0], resolution=[c["width"], c["height"]], rostopic=c["image"],
            timeshift_cam_imu=0.0)
    i = out.imu
    rate = i["rate"]
    imu = dict(imu0=dict(rostopic=i["topic"], update_rate=float(rate),
                         gyroscope_noise_density=i["gyro_std"] / math.sqrt(rate),
                         accelerometer_noise_density=i["accel_std"] / math.sqrt(rate),
                         gyroscope_random_walk=max(i["gyro_bias_std"], 1e-5),
                         accelerometer_random_walk=max(i["accel_bias_std"], 1e-4),
                         T_i_b=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], time_offset=0.0))
    return chain, imu


# ---------------------------------------------------------------- main


def build(scenario_path, out_dir=None, ego_stack=None):
    if not os.path.isfile(scenario_path):
        scenario_path = os.path.join(SCEN_DIR, f"{scenario_path}.yaml")
    with open(scenario_path) as f:
        sc = yaml.safe_load(f)
    scen_name = sc.get("name", os.path.splitext(os.path.basename(scenario_path))[0])
    if ego_stack:  # sweep helper: swap the ego's sensor stack without writing a new scenario
        egos = [a for a in sc["agents"] if a.get("role", "ego") == "ego"] or sc["agents"][:1]
        egos[0]["stack"] = ego_stack
        scen_name = f"{scen_name}__{os.path.splitext(os.path.basename(ego_stack))[0]}"
    out_dir = out_dir or os.path.join(ROOT, "generated", scen_name)
    os.makedirs(out_dir, exist_ok=True)

    world_type = sc.get("world", "arena")
    if world_type == "arena":
        world_name = sc.get("world_name", "arena")
        avoid = [list(a.get("pose", [0, 0]))[:2] for a in sc["agents"]]  # keep balls off spawn points
        world, landmarks, arena = arena_world(world_name, dict(sc.get("arena") or {}, _avoid=avoid))
    elif world_type == "blimp_test":
        world_name = "blimp_test"
        world, landmarks, arena = blimp_test_world()
    else:
        sys.exit(f"unknown world '{world_type}' (arena | blimp_test)")

    bridges = [dict(ros_topic_name="/clock", gz_topic_name=f"/world/{world_name}/clock",
                    ros_type_name="rosgraph_msgs/msg/Clock", gz_type_name="ignition.msgs.Clock", direction="GZ_TO_ROS"),
               dict(ros_topic_name="/sim/dynamic_poses", gz_topic_name=f"/world/{world_name}/dynamic_pose/info",
                    ros_type_name="tf2_msgs/msg/TFMessage", gz_type_name="ignition.msgs.Pose_V", direction="GZ_TO_ROS")]
    tfs, info_agents, models, report = [], [], [], []
    for ag in sc["agents"]:
        stack = load_stack(ag.get("stack", "none"))
        x, y, z, yaw = (list(ag.get("pose", [0, 0, 1.5, 0])) + [0, 0, 1.5, 0][len(ag.get("pose", [])):])[:4]
        o = blimp_model(ag["name"], stack, [x, y, z, 0, 0, yaw], ag.get("team", "none"))
        models.append(o.model_sdf)
        bridges += o.bridges
        tfs += o.tfs
        mb = o.mass
        trim = max(0.0, -mb["net_mass"] * G)
        if mb["net_mass"] * G < -0.15:
            print(f"  WARNING {ag['name']}: {mb['net_mass'] * 1000:.0f} g heavy -> needs {trim:.2f} N of continuous "
                  f"vertical thrust (max 0.5). Set hull.auto_scale: true in stack '{stack['name']}'.")
        report.append(f"  {ag['name']:<10} team={ag.get('team', 'none'):<6} stack={stack['name']:<22} hull_scale={o.scale:.3f} "
                      f"sensors={mb['sensor_mass'] * 1000:5.1f} g total={mb['total_mass'] * 1000:6.1f} g "
                      f"lift={mb['buoyant_mass'] * 1000:6.1f} g net={mb['net_mass'] * 1000:+6.1f} g trim={trim:.3f} N")
        info_agents.append(dict(
            name=ag["name"], team=ag.get("team", "none"), stack=stack["name"], role=ag.get("role", "ego"),
            pose=[x, y, z, yaw], hull_scale=round(o.scale, 4), trim_thrust=round(trim, 4),
            mass_budget={k: round(v, 5) for k, v in mb.items()},
            frames=dict(base=f"{ag['name']}/base_link", odom=f"{ag['name']}/odom"),
            sensors=dict(imu=o.imu, baro=o.baro, flow=o.flow, scan=o.scan, stereo=o.stereo, tofs=o.tofs),
            helper_nodes=o.ros_nodes, expected_topics=o.topics))
        if o.imu:
            with open(os.path.join(out_dir, f"ekf_{ag['name']}.yaml"), "w") as f:
                yaml.safe_dump(ekf_yaml(o), f, sort_keys=False)
        chain, imu = calib_yaml(o)
        if chain:
            with open(os.path.join(out_dir, f"{ag['name']}_camchain.yaml"), "w") as f:
                f.write("# Kalibr camchain (cam0 = left). Pinhole, no distortion (Gazebo renders ideal images).\n")
                yaml.safe_dump(chain, f, sort_keys=False, default_flow_style=None)
            with open(os.path.join(out_dir, f"{ag['name']}_imu.yaml"), "w") as f:
                f.write("# Kalibr IMU noise, derived from the Gazebo per-sample stddev at the sensor rate.\n")
                yaml.safe_dump(imu, f, sort_keys=False, default_flow_style=None)

    world = world.replace("@@AGENTS@@", "\n    ".join(models))
    with open(os.path.join(out_dir, "world.sdf"), "w") as f:
        f.write(world)
    with open(os.path.join(out_dir, "bridge.yaml"), "w") as f:
        f.write("# GENERATED by scripts/build_scenario.py\n")
        yaml.safe_dump(bridges, f, sort_keys=False)
    with open(os.path.join(out_dir, "static_tf.yaml"), "w") as f:
        yaml.safe_dump(dict(transforms=tfs), f, sort_keys=False, default_flow_style=None)
    with open(os.path.join(out_dir, "landmarks.yaml"), "w") as f:
        f.write("# Ground-truth landmark map (world frame). tag 'size' = black-border edge length in metres.\n")
        yaml.safe_dump(dict(landmarks=landmarks), f, sort_keys=False, default_flow_style=None)
    info = dict(scenario=scen_name, world_name=world_name, world_type=world_type, out_dir=out_dir,
                arena=dict(length=arena["length"], width=arena["width"], height=arena["wall_height"]),
                agents=info_agents)
    with open(os.path.join(out_dir, "scenario_info.yaml"), "w") as f:
        yaml.safe_dump(info, f, sort_keys=False, default_flow_style=None)
    print(f"[build_scenario] {scen_name}: world '{world_name}', {len(sc['agents'])} agent(s) -> {out_dir}")
    print("\n".join(report))
    return info


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", help="scenario name in config/scenarios/ or a path to a YAML file")
    ap.add_argument("--out", help="output dir (default generated/<scenario>)")
    ap.add_argument("--ego-stack", help="override the ego agent's sensor stack (name or path)")
    a = ap.parse_args()
    build(a.scenario, a.out, a.ego_stack)


if __name__ == "__main__":
    main()
