# SLAM / localization comparison plan

Goal: find which estimator gives the blimp a usable pose **in the arena frame** (where the goals are)
with the sensors it can actually lift and the computer it actually carries.

## 1. What makes this use case unusual

| Property | Consequence for SLAM |
|---|---|
| Payload is tens of grams (every 10 g ≈ +3–4 % hull size, see the generator's mass report) | sensors are cheap/low-res; a method that needs the lidar has to beat `vision_only` by enough to justify 42 g |
| Small onboard computer | CPU/RAM per method is a first-class metric (`cpu_mean`, `rss_max`) |
| Slow, smooth motion; roll/pitch small (CoM under CoB); yaw oscillates | 2D lidar SLAM is plausible; pure-yaw rotations hurt stereo VO (`spin` pattern) |
| Large open volume, walls may be > 10 m away | a 12 m 2D lidar sees little from mid-arena; stereo depth is weak at range (baseline 6 cm) |
| Moving objects: balls, opponents, teammates | scan matching / loop closure can lock onto the wrong things (`arena_2v2`) |
| **The arena is known** (goal positions, could be tagged) | localization against a known map may beat SLAM entirely (`apriltag` method); or map once, then localize |

## 2. Candidates

All runnable from `config/slam/methods.yaml` via `launch/slam.launch.py` + `scripts/run_benchmark.sh`:

| Method | Sensors | Odometry it relies on | What it tests |
|---|---|---|---|
| `ekf_only` | IMU, flow, baro, ToF | itself (robot_localization) | dead-reckoning baseline: how fast does xy drift? |
| `slam_toolbox` | 2D lidar | EKF | standard 2D graph SLAM |
| `cartographer_2d` | 2D lidar, IMU | EKF | alternative 2D SLAM, better IMU use |
| `cartographer_2d_imu` | 2D lidar, IMU | none | is the flow/EKF needed at all for lidar SLAM? |
| `rtabmap_stereo` | stereo, IMU | its own stereo VO | vision-only SLAM with loop closure |
| `rtabmap_stereo_ekf` | stereo | EKF | stereo mapping/loop closure on top of the EKF |
| `rtabmap_stereo_scan` | stereo, 2D lidar | its own VO | does adding the lidar to vision help? |
| `rtabmap_scan` | 2D lidar | EKF | lidar-only graph SLAM, RTAB-Map flavour |
| `apriltag` | one camera, IMU | (none) | known-map localization: accuracy **and** coverage (needs ≥ 2 tags in view) |

Built from source (not wired into the launch file; calibration is generated for them):

| Method | Notes |
|---|---|
| OpenVINS (stereo-inertial MSCKF) | ROS 2 support upstream. Use `generated/<scenario>/blimp_camchain.yaml` + `blimp_imu.yaml` (Kalibr format). Record with `traj_recorder est_mode:=odom est_topic:=/ov_msckf/odomimu`. |
| ORB-SLAM3 stereo-inertial | community ROS 2 wrappers; build the settings file from the camchain (fx, fy, cx, cy, baseline, T_b_c). |
| VINS-Fusion | ROS 2 ports exist; same calibration. |

Apt packages (Humble): `ros-humble-robot-localization ros-humble-slam-toolbox ros-humble-cartographer-ros ros-humble-rtabmap-ros ros-humble-apriltag-ros`.

## 3. Procedure

**Step 0: sensors are sane.**
```bash
ros2 launch launch/scenario.launch.py scenario:=arena_solo flow_compare:=true
python3 scripts/check_sensors.py generated/arena_solo/scenario_info.yaml
python3 scripts/waypoint_flight.py --ros-args -p use_sim_time:=true -p info_file:=generated/arena_solo/scenario_info.yaml -p pattern:=loop
```
The optical-flow node logs RMSE and correlation against ground truth every 5 s. Negative correlation means flip
`flip_x` / `flip_y`. Correlation near 0 means the image orientation differs from what the node assumes.
In RViz, check that `/blimp/scan` lines up with the walls and isn't hitting the blimp itself.

**Step 1: each method works at all.** Run it with perfect odometry first (`odom_source:=gt`, where
the method takes odometry). That separates "the config is wrong" from "the odometry is bad".

**Step 2: the main matrix.** Run each method on each trajectory, then on each stack variant:
```bash
bash scripts/run_benchmark.sh all arena_solo loop
bash scripts/run_benchmark.sh all arena_solo lawnmower
bash scripts/run_benchmark.sh all arena_solo loop3d
bash scripts/run_benchmark.sh all arena_solo spin
bash scripts/run_benchmark.sh all arena_solo loop full_stack,vision_only,full_stack_lidar_top,full_stack_stereo_nose,full_stack_wide_baseline
python3 scripts/eval_traj.py --table results/summary.csv
```
`all` skips methods whose sensors the stack lacks. Repeat each cell 3× (opponent patrol and sensor noise
change between runs) and compare medians.

**Step 3: stress the winners.** Copy `arena_1v1.yaml` and change one thing at a time:
- `agents`: 2v2 with patrolling opponents (dynamic objects in scans and images)
- `arena.length/width`: the real venue size (lidar range vs wall distance)
- `landmarks.posters: false`: bare walls (texture-poor, like a real gym)
- `landmarks.apriltags: false`: no tags (what if we can't tag the venue?)
- `balls.green/purple: 15`: clutter

## 4. How to pick

Suggested scoring, highest weight first:
1. **coverage** ≥ 0.98. Tracking loss mid-match is worse than slow drift.
2. **ATE** on `loop` and `lawnmower` (SE3-aligned). For goal-scoring you probably want < 0.3 m; set your own bar.
3. **final_err / rpe_trans**: drift. `ekf_only` sets the floor every method must beat.
4. **cpu_mean / rss_max** on the benchmark machine. Scale by how much slower the onboard computer is.
5. **Sensor cost**: the mass the method requires. Each gram means a bigger envelope (see the `build_scenario` report).

A likely outcome (an untested guess to check against data): a fused design. EKF for smooth short-term
odometry; known landmarks (tags, or a map made once with slam_toolbox / RTAB-Map then used in localization
mode) to remove drift. The `apriltag` row shows how often tag fixes are available. If coverage is low but
ATE is excellent, add `/blimp/tag_pose` as an absolute pose input to a second, map-frame
robot_localization instance.

## 5. Sim-to-real gaps (read before trusting the numbers)

- **No air currents.** HVAC drafts push a real blimp around; here only thrust moves it.
- **Ideal cameras.** No motion blur, rolling shutter, auto-exposure, lens distortion, or low-light noise.
  Visual methods will look better than they will be.
- **Instant lidar scans.** A real LD06 spins at ~10 Hz, so yaw during a scan skews it. The blimp's yaw
  wobble makes this matter more than on a ground robot.
- **Optical flow** is phase correlation on a rendered 64×64 image, not the PMW3901's internals, and the
  rendered floor texture is generous.
- **Barometer** has white noise but no drift or door-slam pressure steps. **IMU** bias walk is set in the
  stack (`gyro_bias_std`, `accel_bias_std`).
- **Buoyancy** sees only the cylinder collision (the nose and tail spheres are visual only), and the damping
  coefficients are untuned guesses. Tune them from real flight logs before trusting closed-loop behaviour.
