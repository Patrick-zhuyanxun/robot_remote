#!/usr/bin/env bash
# RViz view of the Quest teleop: the Isaac rig, or with --source real the physical arms + Quest controllers.
# Read-only: sends no commands,
# so it can run next to run_dual_arm_sim.sh and run_quest_dual_arm.sh.
#   --yaw-deg N   use the same value you pass to the teleop script (default 0)
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../ros_env.sh"  # private ROS 2 domain, localhost only (see ros_env.sh)
source /opt/ros/humble/setup.bash
source "$HERE/../igus_ws/install/setup.bash"

XACRO="$(ros2 pkg prefix igus_rebel_description)/share/igus_rebel_description/urdf/igus_rebel_xeg32_wrapper.urdf.xacro"
PARAMS="$(mktemp /tmp/teleop_viz_rsp_XXXX.yaml)"
# the URDF is too long for -p on the command line; hand it over as a params file
xacro "$XACRO" include_ros2_control:=false | python3 -c 'import sys, yaml; yaml.safe_dump({"/**": {"ros__parameters": {"robot_description": sys.stdin.read()}}}, sys.stdout)' > "$PARAMS"

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; wait 2>/dev/null || true; rm -f "$PARAMS"; }
trap cleanup EXIT INT TERM

for side in left right cmd_left cmd_right; do  # cmd_* = translucent teleop command arms
  ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/$side \
    --params-file "$PARAMS" -p frame_prefix:="$side/" &
  pids+=($!)
done
python3 "$HERE/teleop_viz.py" "$@" &
pids+=($!)
if [[ "${NO_RVIZ:-0}" == "1" ]]; then wait; else rviz2 -d "$HERE/teleop_viz.rviz"; fi
