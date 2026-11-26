#!/usr/bin/env bash
# One-command demo, made for SSH: runs the arena headless, flies the blimp, saves videos and pictures.
#
#   bash scripts/demo.sh [scenario] [sim_seconds] [pattern]
#   bash scripts/demo.sh                          # arena_solo, 150 s, track (rounded laps)
#   bash scripts/demo.sh arena_1v1 120 track      # with an opponent patrolling
#   HEADLESS=false bash scripts/demo.sh           # also open the Gazebo GUI (only if you have a display, e.g. VNC)
#
# Output: demo_out/<stamp>_<scenario>/  overview.mp4 topdown.mp4 onboard.mp4 scan_map.png altitude.png
#         summary.png sensors.txt (+ logs). Copy it to your laptop with the scp line printed at the end.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="${1:-arena_solo}"
DURATION="${2:-150}"
PATTERN="${3:-track}"
AGENT="${AGENT:-blimp}"
HEADLESS="${HEADLESS:-true}"
OUT="$ROOT/demo_out/$(date +%Y%m%d_%H%M%S)_${SCENARIO}"
GEN="$ROOT/generated/$SCENARIO"
mkdir -p "$OUT"

set +u; source /opt/ros/humble/setup.bash; set -u   # ROS setup scripts read unset variables
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-44}"
export IGN_PARTITION="${IGN_PARTITION:-blimp_demo_$USER}"
export GZ_PARTITION="$IGN_PARTITION"

PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
  pkill -f "ign gazebo.*$GEN/world.sdf" 2>/dev/null
  true
}
trap cleanup EXIT

echo "== starting $SCENARIO (headless=$HEADLESS), logs in $OUT"
setsid ros2 launch "$ROOT/launch/scenario.launch.py" scenario:="$SCENARIO" headless:="$HEADLESS" \
    odom_source:=none others:=patrol >"$OUT/sim.log" 2>&1 &
PIDS+=($!)

echo -n "== waiting for the sim"
end=$((SECONDS + 120))
until timeout 3 ros2 topic echo --once "/$AGENT/odom_gt" >/dev/null 2>&1; do
  echo -n "."
  if [ $SECONDS -gt $end ]; then echo; echo "sim did not come up, last lines of $OUT/sim.log:"; tail -20 "$OUT/sim.log"; exit 1; fi
done
echo " up"

WORLD="$(python3 -c "import yaml; print(yaml.safe_load(open('$GEN/scenario_info.yaml'))['world_name'])")"
echo "== spawning spectator cameras into world '$WORLD'"
ign service -s "/world/$WORLD/create" --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean \
    --timeout 5000 --req "sdf_filename: \"$ROOT/models/demo_cameras/model.sdf\" name: \"demo_cameras\"" \
    >>"$OUT/sim.log" 2>&1 || echo "   (spawn failed: overview/topdown videos will be empty)"
setsid ros2 run ros_gz_bridge parameter_bridge \
    /demo/overview/image@sensor_msgs/msg/Image[ignition.msgs.Image \
    /demo/topdown/image@sensor_msgs/msg/Image[ignition.msgs.Image \
    --ros-args -p use_sim_time:=true >"$OUT/demo_bridge.log" 2>&1 &
PIDS+=($!)

echo "== flying '$PATTERN' and recording $DURATION sim-seconds"
setsid python3 "$ROOT/scripts/waypoint_flight.py" --ros-args -p use_sim_time:=true -p agent:="$AGENT" \
    -p info_file:="$GEN/scenario_info.yaml" -p pattern:="$PATTERN" -p timeout:="$DURATION" >"$OUT/flight.log" 2>&1 &
PIDS+=($!)
python3 "$ROOT/scripts/demo_capture.py" --ros-args -p use_sim_time:=true -p agent:="$AGENT" \
    -p info_file:="$GEN/scenario_info.yaml" -p out_dir:="$OUT" -p duration:="$DURATION" \
    -p max_wall:="$((DURATION * 6 + 120))" 2>&1 | tee "$OUT/capture.log"

echo
echo "== done. Files in $OUT:"
ls -lh "$OUT"
echo
echo "Copy to your laptop (run this ON THE LAPTOP):"
echo "  scp -r $USER@$(hostname -f 2>/dev/null || hostname):$OUT ."
