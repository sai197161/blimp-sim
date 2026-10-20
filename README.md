# blimp-sim

Gazebo Fortress simulation of the VIP indoor blimp with the proposed vision/state-estimation sensor stack, bridged to ROS 2 Humble.

- Lighter-than-air physics: buoyancy from displaced air, near-neutral by design
- 3 thrusters: left/right forward (differential yaw) + 1 vertical
- Aerodynamic damping
- Sensors: IMU, front RGB camera, downward camera (optical-flow stand-in), downward rangefinder, 16-channel 3D lidar
- Ground-truth odometry for evaluating SLAM / state estimation
- Scripted thrust test that checks flight behavior and sensor rates

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
