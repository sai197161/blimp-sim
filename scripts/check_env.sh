#!/usr/bin/env bash
# Checks the lab machine has everything the sim needs. Run: bash scripts/check_env.sh
ok=0
check() { if eval "$2" >/dev/null 2>&1; then echo "  [OK]   $1"; else echo "  [MISS] $1  -> $3"; ok=1; fi; }

[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
echo "Checking environment..."
check "ROS 2 Humble"               "[ -f /opt/ros/humble/setup.bash ]"        "install ros-humble-desktop"
check "ros2 CLI"                   "command -v ros2"                          "source /opt/ros/humble/setup.bash"
check "Gazebo Fortress (ign)"      "command -v ign && ign gazebo --versions"  "sudo apt install ignition-fortress"
check "ros_gz_bridge"              "ros2 pkg prefix ros_gz_bridge"            "sudo apt install ros-humble-ros-gz"
check "rclpy (python)"             "python3 -c 'import rclpy'"                "source /opt/ros/humble/setup.bash"
check "nav_msgs / sensor_msgs"     "python3 -c 'import nav_msgs.msg, sensor_msgs.msg'" "sudo apt install ros-humble-ros-base"
echo
[ $ok -eq 0 ] && echo "All good. Next: bash scripts/run_sim.sh" || echo "Fix the [MISS] items above (apt needs sudo -> ask the lab admin)."
exit $ok
