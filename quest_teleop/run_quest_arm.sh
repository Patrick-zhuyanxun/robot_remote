#!/usr/bin/env bash
# Meta Quest -> igus ReBeL arm (default: right arm of the Isaac dual-arm rig).
# Needs, already running:
#   1. robot_remote/dual_arm_sim/run_dual_arm_sim.sh          (Isaac, no /opt/ros sourced)
#   2. robot_remote/dual_arm_sim/start_right_arm_moveit.sh    (relay + ros2_control + MoveIt/RViz)
# Extra args are passed through, e.g.  --yaw-deg 180  --scale 0.5  --gripper-open -0.0116631 --gripper-closed 0.00483688
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../ros_env.sh"  # private ROS 2 domain, localhost only (see ros_env.sh)
source /opt/ros/humble/setup.bash
source "$HERE/../igus_ws/install/setup.bash"
exec "$HERE/../.venv_quest/bin/python" "$HERE/quest_arm_teleop.py" "$@"
