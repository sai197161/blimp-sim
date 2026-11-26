# blimp-sim

Gazebo Fortress simulation of the VIP indoor blimp, bridged to ROS 2 Humble.

- Lighter-than-air physics: buoyancy from displaced air, near-neutral by design
- 3 thrusters: left/right forward (differential yaw) + 1 vertical
- Aerodynamic damping
- Ground-truth odometry for evaluating SLAM / state estimation
- **Scenario generator**: competition-style arena (goal hoops, floating balls, AprilTag + poster landmarks),
  multiple agents (team / opponents), swappable **sensor stacks** described in YAML (what and where)
- Proposed stack: stereo camera, 2D lidar, ToF, barometer, IMU, optical flow (emulated)
- **SLAM benchmark**: slam_toolbox, Cartographer, RTAB-Map variants, AprilTag localization, EKF baseline,
  scored against ground truth (ATE / RPE / coverage / CPU)

The original single-blimp setup (steps 1-8 below) still works unchanged. The new workflow is in
[Arena, sensor stacks & SLAM benchmark](#arena-sensor-stacks--slam-benchmark).

**No build step.** It runs straight from the cloned folder.

---

## Requirements

| What | Version | Check with |
|---|---|---|
| Ubuntu | 22.04 | `lsb_release -a` |
| ROS 2 | Humble | `ls /opt/ros/humble` |
| Gazebo | Fortress (`ign gazebo` 6.x) | `ign gazebo --versions` |
| ros_gz bridge | `ros-humble-ros-gz` | `ros2 pkg prefix ros_gz_bridge` |

If something is missing (needs sudo, so ask the lab admin):
```bash
sudo apt install ros-humble-ros-gz        # pulls in ros_gz_bridge + Fortress if missing
```

---

## Step-by-step (lab workstation)

### 1. Clone into the workspace
```bash
mkdir -p ~/sai/sim_ws/src
cd ~/sai/sim_ws/src
git clone https://github.com/sai197161/blimp-sim.git
cd blimp-sim
```

### 2. Check the environment
```bash
bash scripts/check_env.sh
```
Every line should say `[OK]`. Fix any `[MISS]` before continuing.

### 3. Launch the sim (Terminal 1)
```bash
cd ~/sai/sim_ws/src/blimp-sim
bash scripts/run_sim.sh
```
Expected:
- Gazebo opens with a gray floor, two walls, a green pillar, an orange box, and the white blimp with a dark gondola floating at 1.5 m.
- The sim is already running (`-r` flag). The blimp should drift down **very slowly**, because it's ~2 g heavy on purpose.
- The terminal shows `ros_gz_bridge` creating bridges for each topic.

Equivalent manual command:
```bash
source /opt/ros/humble/setup.bash
ros2 launch ~/sai/sim_ws/src/blimp-sim/launch/blimp_sim.launch.py
```

### 4. Check topics (Terminal 2)
```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep blimp
```
You should see:
```
/blimp/down_camera/image_raw
/blimp/front_camera/camera_info
/blimp/front_camera/image_raw
/blimp/imu
/blimp/lidar/points
/blimp/odom_gt
/blimp/range
/blimp/thrust/left
/blimp/thrust/right
/blimp/thrust/vertical
```
Spot-check rates (Ctrl-C each after a few seconds):
```bash
ros2 topic hz /blimp/imu                 # ~200 Hz
ros2 topic hz /blimp/front_camera/image_raw   # ~30 Hz
ros2 topic hz /blimp/lidar/points        # ~10 Hz
ros2 topic echo /blimp/range --once      # ranges[0] ~ 1.1 m (height minus gondola offset)
```

### 5. Manual thrust check (Terminal 2)
```bash
ros2 topic pub --once /blimp/thrust/vertical std_msgs/msg/Float64 "{data: 0.08}"   # should rise
ros2 topic pub --once /blimp/thrust/vertical std_msgs/msg/Float64 "{data: 0.02}"   # ~hover
ros2 topic pub --once /blimp/thrust/left  std_msgs/msg/Float64 "{data: 0.06}"
ros2 topic pub --once /blimp/thrust/right std_msgs/msg/Float64 "{data: 0.06}"      # should move forward (+x)
```
The thruster holds the last command, so send `0.0` to stop.

### 6. Run the scripted flight test (Terminal 2)
```bash
cd ~/sai/sim_ws/src/blimp-sim
python3 scripts/thrust_test.py --ros-args -p use_sim_time:=true
```
It flies hold → trim → forward → yaw → climb → stop (~38 s), then prints:
- per-phase displacement (dx, dy, dz, dyaw)
- the rate of every sensor topic (`<-- NO DATA` flags a broken one)
- a CSV log `thrust_test_<timestamp>.csv`

Expected results:
| Phase | Command (N) | Expected |
|---|---|---|
| hold | none | slow sink (dz slightly negative) |
| trim | V=0.02 | dz ≈ 0 |
| forward | L=R=0.06 | dx clearly positive |
| yaw | L=+0.04, R=−0.04 | large dyaw, small dx/dy |
| climb | V=0.07 | dz positive |
| stop | V=0.02 | settles |

### 7. Visualize in RViz (optional, Terminal 3)
```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id lidar_link &
rviz2
```
In RViz: set **Fixed Frame** = `map`, then add **PointCloud2** → `/blimp/lidar/points` and **Image** → `/blimp/front_camera/image_raw` and `/blimp/down_camera/image_raw`.

### 8. Record a bag (optional)
```bash
ros2 bag record -s mcap -o sim_run1 /clock /blimp/imu /blimp/odom_gt /blimp/range \
  /blimp/lidar/points /blimp/front_camera/image_raw /blimp/down_camera/image_raw
```

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Unable to find uri[model://blimp]` | Launch via `scripts/run_sim.sh` (it sets `IGN_GAZEBO_RESOURCE_PATH`). Manual: `export IGN_GAZEBO_RESOURCE_PATH=~/sai/sim_ws/src/blimp-sim/models:$IGN_GAZEBO_RESOURCE_PATH` |
| `ign: command not found` | Fortress isn't installed or isn't on PATH; run `bash scripts/check_env.sh` |
| `package 'ros_gz_bridge' not found` | `sudo apt install ros-humble-ros-gz` (ask the admin) |
| Topics exist in Gazebo but not in ROS | Compare `ign topic -l` with `config/bridge.yaml`; gz topic names must match exactly |
| Thrust does nothing | Run `ign topic -l \| grep thrust`. If the topic differs from `/model/blimp/joint/<joint>/cmd_thrust`, update `config/bridge.yaml` |
| Blimp moves backwards on positive thrust | Flip the sign in `scripts/thrust_test.py` (PHASES) or flip the joint `<axis>` in `model.sdf` |
| Blimp rockets up / falls fast | Buoyancy plugin not loaded; check the world file has the `Buoyancy` system and the Gazebo console for plugin errors |
| Camera/lidar topics have no data | Needs a GPU/OpenGL display; over ssh, use `bash scripts/run_sim.sh headless` |
| ROS topics from another team member's machine show up | Set a unique `ROS_DOMAIN_ID` (run_sim.sh defaults to 42) |
| Odometry never arrives in thrust_test | Make sure the sim is running (not paused) and `ros2 topic echo /blimp/odom_gt --once` works |

---

## Repo layout
```
blimp-sim/
├── models/blimp/model.sdf     # blimp: hull, gondola, props, sensors, plugins
├── models/blimp/model.config
├── worlds/blimp_test.sdf      # test room + physics/sensor/buoyancy systems
├── config/bridge.yaml         # ROS <-> Gazebo topic mapping
├── launch/blimp_sim.launch.py
└── scripts/
    ├── check_env.sh           # verifies dependencies
    ├── run_sim.sh             # launches sim + bridge
    └── thrust_test.py         # scripted flight + sensor-rate test
```

## Model design

| Item | Value | Why |
|---|---|---|
| Hull | cylinder r=0.30 m, L=1.0 m (+ sphere end caps, visual only) | Buoyancy system computes volume from collision shapes |
| Fluid | air, 1.225 kg/m³ | Lighter-than-air |
| Mass | 0.349 kg vs 0.347 kg displaced → −0.02 N | Slightly heavy; vertical prop trims at 0.02 N |
| CoM vs center of buoyancy | gondola 0.36 m below hull | Passive roll/pitch stability |
| Thrusters | `Thruster` system, command = thrust in N | Topic `/model/blimp/joint/<joint>/cmd_thrust` |
| Drag | `Hydrodynamics` system with air density, linear + quadratic damping | Coefficients are first guesses |
| Ground truth | `OdometryPublisher` → `/blimp/odom_gt` | For evaluating estimators |

## Sensor stack

| Sensor | ROS topic | Rate | Real-hardware analog |
|---|---|---|---|
| IMU | `/blimp/imu` | 200 Hz | IMU on Teensy / flight controller |
| Front RGB 640×480 | `/blimp/front_camera/image_raw` | 30 Hz | VIO / visual SLAM camera |
| Downward mono 160×160 | `/blimp/down_camera/image_raw` | 30 Hz | PMW3901 optical flow |
| Downward rangefinder | `/blimp/range` | 50 Hz | VL53L1X ToF |
| 3D lidar, 16 ch | `/blimp/lidar/points` | 10 Hz | Candidate, payload TBD |

## Known limitations / next steps
- Damping and thrust coefficients are placeholders; tune with real blimp dimensions, mass, and props.
- No added-mass model yet (significant for blimps); add `xDotU` etc. after basic flight is verified.
- Real hull is an ellipsoid; the cylinder slightly overestimates volume.
- Next: realistic indoor environment, then run an off-the-shelf SLAM (e.g. ORB-SLAM3 / FAST-LIO) on sim bags against `/blimp/odom_gt`.

---

## Quick demo over SSH (no GUI)

One command runs the arena headless, flies the blimp around a loop with the full sensor stack, and
saves videos and pictures you can copy to your laptop:

```bash
bash scripts/demo.sh                      # arena_solo, 150 sim-seconds, rounded laps (track)
bash scripts/demo.sh arena_1v1 120 track # with an opponent blimp patrolling
```

It writes `demo_out/<stamp>_<scenario>/` and prints the `scp` line to run on your laptop:

| File | What it shows |
|---|---|
| `summary.png` | everything below in one picture (for a slide) |
| `overview.mp4`, `topdown.mp4` | spectator cameras: the arena, landmarks, goals, balls and the blimp flying |
| `onboard.mp4` | the blimp's left stereo camera, with the optical-flow camera inset |
| `scan_map.png` | every 2D lidar scan placed with the ground-truth pose: the arena outline, with tags, pillars and goals marked |
| `altitude.png` | height from ground truth vs the ToF and the barometer |
| `sensors.txt` | messages received and rate for each sensor |

Headless mode still renders the cameras, so the lab machine needs a GPU with EGL. If the videos are
empty, check `sim.log`. With a desktop session (e.g. VNC), `HEADLESS=false bash scripts/demo.sh`
also opens the Gazebo GUI.

## Arena, sensor stacks & SLAM benchmark

### How it fits together
```
config/scenarios/<name>.yaml ──┐   (world + arena knobs + list of agents)
config/sensor_stacks/<s>.yaml ─┤   (sensors, mounts, poses, rates, noise, masses)
                               ▼
              scripts/build_scenario.py ──► generated/<scenario>/   (git-ignored)
                                             world.sdf  bridge.yaml  static_tf.yaml  scenario_info.yaml
                                             landmarks.yaml  ekf_<agent>.yaml  <agent>_camchain.yaml / _imu.yaml
                               ▼
     launch/scenario.launch.py  (runs the generator, Gazebo, bridge, TFs, flow/altitude nodes, EKF)
     launch/slam.launch.py      (one method from config/slam/methods.yaml)
     scripts/run_benchmark.sh   (scenario + method + scripted flight + recorder + scoring, headless)
```
The generator also prints a **mass budget** for every agent. With `hull.auto_scale: true` it grows the
envelope until the blimp is ~2 g heavy again. That shows what each sensor costs in envelope size
(the full stack needs a hull ~11 % bigger than the original).

### Extra packages
```bash
sudo apt install ros-humble-robot-localization ros-humble-slam-toolbox ros-humble-cartographer-ros \
                 ros-humble-rtabmap-ros ros-humble-apriltag-ros
pip3 install --user psutil matplotlib     # optional: CPU logging, trajectory plots
```

### Scenarios (`config/scenarios/`)
| Scenario | What |
|---|---|
| `arena_solo` | 20×12×6 m arena, ego blimp with `full_stack`. The standard benchmark world |
| `arena_1v1` | + one opponent. **Every arena knob is documented in this file** |
| `arena_2v2` | ego + teammate (`minimal` stack) vs two opponents |
| `test_room_full_stack` | the original small room with the full stack, for a quick sensor check |

The arena has three goal hoops (circle / square / triangle) at each end in team colours, neutrally buoyant
green and purple balls, court lines, a textured floor (so optical flow has something to track), and walls
carrying real AprilTag 36h11 markers (ids in `generated/<s>/landmarks.yaml`) plus feature posters.
Non-ego agents fly a `patrol` pattern by default (`others:=hover|idle` to change).

### Sensor stacks (`config/sensor_stacks/`)
| Stack | What |
|---|---|
| `full_stack` | **the proposed stack**: stereo (6 cm, 20° down), 2D lidar under the gondola, down ToF, baro, IMU, optical flow |
| `full_stack_lidar_top` | lidar on top of the envelope |
| `full_stack_lidar_gondola` | lidar level with the gondola (prop arm in the scan plane: shows self-occlusion) |
| `full_stack_stereo_nose` | stereo on the envelope nose, level |
| `full_stack_wide_baseline` | 12 cm stereo baseline |
| `full_stack_multizone_tof` | + forward 8×8 ToF |
| `vision_only` | no lidar (saves 42 g) |
| `baseline` | the original model's sensors (mono, down cam, rangefinder, 3D lidar) |
| `minimal`, `none` | cheap stacks for other agents |

A variant only lists what changes (`extends: full_stack` + `modify` / `remove` / `add`). Mount points
(`hull_nose`, `hull_top`, `gondola_bottom`, ...) move with the hull when it's auto-scaled. Field reference:
the comments in `full_stack.yaml` and `SENSOR_DEFAULTS` in `scripts/build_scenario.py`.

ROS topics for agent `blimp` with the full stack:
```
/blimp/imu                       /blimp/baro (FluidPressure)        /blimp/tof_down (LaserScan)
/blimp/scan (LaserScan)          /blimp/stereo/{left,right}/{image_raw,camera_info}
/blimp/flow/camera/image_raw     /blimp/flow/twist (TwistWithCovarianceStamped, from optical_flow_node.py)
/blimp/altitude/{baro,tof}       /blimp/odometry/filtered (EKF)     /blimp/odom_gt
/blimp/thrust/{left,right,vertical}                                 /sim/dynamic_poses (all model poses)
```
TF: `blimp/odom → blimp/base_link` (EKF, ground truth, or the SLAM method) and `blimp/base_link →` every
sensor frame. Camera images use `*_optical` frames.

### Run a scenario
```bash
source /opt/ros/humble/setup.bash
cd ~/sai/sim_ws/src/blimp-sim
ros2 launch launch/scenario.launch.py scenario:=arena_1v1
ros2 launch launch/scenario.launch.py scenario:=arena_solo ego_stack:=full_stack_lidar_top   # placement variant
python3 scripts/check_sensors.py generated/arena_1v1/scenario_info.yaml      # every topic + rate
python3 scripts/waypoint_flight.py --ros-args -p use_sim_time:=true \
    -p info_file:=generated/arena_1v1/scenario_info.yaml -p pattern:=lawnmower   # fly it
```
`odom_source:=ekf|gt|none` picks what provides `blimp/odom → base_link`. Add `flow_compare:=true` to log
optical-flow error against ground truth.

### Run the SLAM comparison
```bash
bash scripts/run_benchmark.sh slam_toolbox,rtabmap_stereo,apriltag        # arena_solo, loop pattern
bash scripts/run_benchmark.sh all arena_solo lawnmower full_stack,vision_only
python3 scripts/eval_traj.py --table results/summary.csv
```
Or by hand: start the scenario with the method's `odom_source` (see `config/slam/methods.yaml`), then
`ros2 launch launch/slam.launch.py method:=<m> rviz:=true`, `traj_recorder.py`, `waypoint_flight.py`,
and `eval_traj.py`. The full plan, how to pick a winner, and the sim-to-real caveats are in
**[docs/SLAM_COMPARISON.md](docs/SLAM_COMPARISON.md)**.

### Things to check on the first real run
These were written without a Gazebo install, so check them once and adjust:
- **Tag textures**: if `apriltag` never detects anything, set `landmarks.tag_mirror: true` (Gazebo may flip
  plane UVs). Tags are positioned and used by their centre only, so their in-plane rotation doesn't matter.
- **Flow sign**: run with `flow_compare:=true`. Negative correlation means set `flip_x` / `flip_y` on
  `optical_flow_node.py`.
- **camera_info**: Fortress publishes it at `<dirname(image topic)>/camera_info`. The generator relies on
  this (and the original model now does too). Confirm with `ign topic -l | grep camera_info`.
- **rtabmap parameter names** target rtabmap_ros 0.21+ (split `rtabmap_odom` / `rtabmap_slam` packages).
