#!/usr/bin/env bash
# Launch the blimp sim (Gazebo Fortress + ROS 2 bridge).
#   bash scripts/run_sim.sh            # with Gazebo GUI
#   bash scripts/run_sim.sh headless   # no GUI (e.g. over ssh)
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
HEADLESS=false; [ "$1" = "headless" ] && HEADLESS=true
exec ros2 launch "$ROOT/launch/blimp_sim.launch.py" headless:=$HEADLESS
