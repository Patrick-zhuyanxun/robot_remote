#!/usr/bin/env bash
# Meta Quest -> ONE real igus ReBeL (default: the left arm, driven by the left controller).
# The robot's driver must already be running on its PC (ROS_DOMAIN_ID 11, Fast DDS). Hold GRIP to move,
# release to stop. Slow defaults; extra args pass through, e.g. --arms right, --max-lin-vel 0.1, --dry-run
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT_REMOTE_TARGET=real
source "$HERE/../ros_env.sh"  # domain 11, Fast DDS, LAN (see ros_env.sh)
source /opt/ros/humble/setup.bash
source "$HERE/../igus_ws/install/setup.bash"
cd "$HERE"
args=("$@")
[[ " ${args[*]} " == *" --arms "* ]] || args=(--arms left "${args[@]}")
exec "$HERE/../.venv_quest/bin/python" "$HERE/quest_dual_arm_teleop.py" --backend real "${args[@]}"
