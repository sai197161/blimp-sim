#!/usr/bin/env bash
# Headless SLAM benchmark: for each (stack, method), launch the scenario, start the method, fly the same
# ground-truth-controlled trajectory, record the estimate, score it against ground truth.
#
#   bash scripts/run_benchmark.sh <methods> [scenario] [pattern] [stacks]
#     methods  comma list from config/slam/methods.yaml, or "all" (skips ones the stack can't support)
#     scenario default arena_solo
#     pattern  lawnmower | loop | loop3d | figure8 | spin        (default loop)
#     stacks   comma list of sensor stacks for the ego             (default: the scenario's own)
#
#   bash scripts/run_benchmark.sh slam_toolbox,rtabmap_stereo
#   bash scripts/run_benchmark.sh all arena_solo lawnmower full_stack,vision_only,full_stack_stereo_nose
#   python3 scripts/eval_traj.py --table results/summary.csv
#
# Each run goes to results/<stamp>_<scenario>_<stack>_<method>_<pattern>/ (logs, gt.tum, est.tum, cpu.csv, traj.png).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
METHODS="${1:?usage: run_benchmark.sh <methods|all> [scenario] [pattern] [stacks]}"
SCENARIO="${2:-arena_solo}"
PATTERN="${3:-loop}"
STACKS="${4:-}"
AGENT="${AGENT:-blimp}"
TIMEOUT="${TIMEOUT:-900}"   # sim seconds the flight may take
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
export IGN_PARTITION="${IGN_PARTITION:-sai_blimp_bench}"
export GZ_PARTITION="$IGN_PARTITION"
SUMMARY="$ROOT/results/summary.csv"
mkdir -p "$ROOT/results"

meth() {  # meth <method> <field.subfield> : read config/slam/methods.yaml ("__list__" lists methods)
  ROOT="$ROOT" python3 -c '
import os, sys, yaml
m = yaml.safe_load(open(os.path.join(os.environ["ROOT"], "config/slam/methods.yaml")))
name, field, agent = sys.argv[1:4]
if name == "__list__":
    print(" ".join(m)); sys.exit()
v = m[name]
for k in field.split("."):
    v = v.get(k, "") if isinstance(v, dict) else ""
print((" ".join(v) if isinstance(v, list) else str(v)).replace("{agent}", agent))
' "$1" "$2" "$AGENT"
}

stack_has() {  # stack_has <scenario_info.yaml> <sensor kinds...> -> exit 0 if the ego has all of them
  python3 -c '
import sys, yaml
info = yaml.safe_load(open(sys.argv[1])); ag = next(a for a in info["agents"] if a["name"] == sys.argv[2])
s = ag["sensors"]
ok = all((s.get(k) if k != "tof" else s.get("tofs")) for k in sys.argv[3:])
sys.exit(0 if ok else 1)' "$@"
}

stop_group() { [ -n "${1:-}" ] && kill -INT -- "-$1" 2>/dev/null; sleep 3; kill -TERM -- "-$1" 2>/dev/null; sleep 1; kill -KILL -- "-$1" 2>/dev/null; true; }

wait_topic() {  # wait_topic <topic> <seconds>
  local end=$((SECONDS + $2))
  while [ $SECONDS -lt $end ]; do
    timeout 3 ros2 topic echo --once "$1" >/dev/null 2>&1 && return 0
  done
  return 1
}

run_one() {
  local method="$1" stack="$2"
  local odom; odom="$(meth "$method" odom_source)"
  local stack_arg="" gen="$ROOT/generated/$SCENARIO"
  if [ -n "$stack" ]; then stack_arg="ego_stack:=$stack"; gen="$ROOT/generated/${SCENARIO}__${stack}"; fi
  python3 "$ROOT/scripts/build_scenario.py" "$SCENARIO" ${stack:+--ego-stack "$stack"} >/dev/null || return 1
  local stack_name; stack_name="$(python3 -c "import yaml,sys; i=yaml.safe_load(open('$gen/scenario_info.yaml')); print(next(a['stack'] for a in i['agents'] if a['name']=='$AGENT'))")"
  # shellcheck disable=SC2046
  if ! stack_has "$gen/scenario_info.yaml" "$AGENT" $(meth "$method" needs); then
    echo "== skip $method on $stack_name: needs [$(meth "$method" needs)]"; return 0
  fi
  local run="$ROOT/results/$(date +%Y%m%d_%H%M%S)_${SCENARIO}_${stack_name}_${method}_${PATTERN}"
  mkdir -p "$run"
  echo "== $method | stack $stack_name | $SCENARIO | $PATTERN -> $run"

  setsid ros2 launch "$ROOT/launch/scenario.launch.py" scenario:="$SCENARIO" $stack_arg headless:=true \
      odom_source:="$odom" others:=patrol >"$run/scenario.log" 2>&1 &
  local scen_pid=$!
  if ! wait_topic "/$AGENT/odom_gt" 90; then
    echo "   sim did not come up (see $run/scenario.log)"; stop_group $scen_pid; return 1
  fi
  setsid ros2 launch "$ROOT/launch/slam.launch.py" method:="$method" agent:="$AGENT" gen_dir:="$gen" \
      >"$run/slam.log" 2>&1 &
  local slam_pid=$!
  sleep 8

  local mode; mode="$(meth "$method" est.mode)"
  local rec_args=(-p use_sim_time:=true -p out_dir:="$run" -p gt_topic:="/$AGENT/odom_gt" -p est_mode:="$mode"
                  -p cpu_pattern:="$(meth "$method" cpu_pattern)")
  if [ "$mode" = tf ]; then rec_args+=(-p map_frame:="$(meth "$method" est.map_frame)" -p base_frame:="$AGENT/base_link")
  else rec_args+=(-p est_topic:="$(meth "$method" est.topic)" -p est_type:="$(meth "$method" est.type)"); fi
  setsid python3 "$ROOT/scripts/traj_recorder.py" --ros-args "${rec_args[@]}" >"$run/recorder.log" 2>&1 &
  local rec_pid=$!

  python3 "$ROOT/scripts/waypoint_flight.py" --ros-args -p use_sim_time:=true -p agent:="$AGENT" \
      -p info_file:="$gen/scenario_info.yaml" -p pattern:="$PATTERN" -p timeout:="$TIMEOUT" >"$run/flight.log" 2>&1
  tail -1 "$run/flight.log"

  stop_group $rec_pid; stop_group $slam_pid; stop_group $scen_pid
  pkill -f "ign gazebo.*$gen/world.sdf" 2>/dev/null || true
  sleep 2

  if [ -s "$run/est.tum" ]; then
    python3 "$ROOT/scripts/eval_traj.py" "$run/gt.tum" "$run/est.tum" --label "$method" --stack "$stack_name" \
        --scenario "$SCENARIO" --pattern "$PATTERN" --summary "$SUMMARY" \
        $(python3 -c "import matplotlib" 2>/dev/null && echo --plot) | tee "$run/metrics.txt"
  else
    echo "   no estimate recorded: method failed or never localized (see $run/slam.log)"
    printf 'label,stack,scenario,pattern,ate_rmse,coverage,run_dir\n%s,%s,%s,%s,,0,%s\n' "$method" "$stack_name" \
        "$SCENARIO" "$PATTERN" "$run" > "$run/metrics_failed.csv"
  fi
}

[ "$METHODS" = all ] && METHODS="$(meth __list__ x | tr ' ' ',')"
IFS=',' read -ra MLIST <<< "$METHODS"
IFS=',' read -ra SLIST <<< "${STACKS:-}"
[ ${#SLIST[@]} -eq 0 ] && SLIST=("")
for s in "${SLIST[@]}"; do
  for m in "${MLIST[@]}"; do run_one "$m" "$s"; done
done
echo
[ -f "$SUMMARY" ] && python3 "$ROOT/scripts/eval_traj.py" --table "$SUMMARY"
