#!/usr/bin/env bash
# Meta Quest -> BOTH arms of the Isaac dual-arm rig (left hand -> left arm, right hand -> right arm).
# Needs only:  robot_remote/dual_arm_sim/run_dual_arm_sim.sh   (Isaac, no /opt/ros sourced)
# Do NOT run dual_arm_sim/start_right_arm_moveit.sh at the same time: its ros2_control also writes the
# right arm and drags it back. Extra args pass through, e.g. --arms right, --yaw-deg 180, --swap-hands, --scale 0.5
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../ros_env.sh"  # private ROS 2 domain, localhost only (see ros_env.sh)
source /opt/ros/humble/setup.bash
source "$HERE/../igus_ws/install/setup.bash"
if pgrep -f "ros2_control_node" >/dev/null; then
  echo "WARNING: ros2_control_node is running (start_right_arm_moveit.sh?) - it will fight the right arm. Stop it first." >&2
fi
cd "$HERE"
exec "$HERE/../.venv_quest/bin/python" "$HERE/quest_dual_arm_teleop.py" "$@"
