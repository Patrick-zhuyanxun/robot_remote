#!/usr/bin/env bash
# Quest -> the physical ReBeLs through the igus ROS driver (default, like the Isaac sim: commands go out on topics).
# Start the drivers first (other terminal):  ros2 launch real_robot/dual_arm_driver.launch.py [arms:=left]
#   ./quest_teleop/run_quest_real_arm.sh --arms left right [--dry-run] [--no-start-home] [--no-park]
# --backend real = the direct Ethernet CRI client instead (no ROS driver; experimental).
# --check-only uses system Python and sends only CRI queries / zero-jog heartbeats.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ " $* " == *" --check-only "* ]]; then
  exec /usr/bin/python3 "$HERE/ethernet_rebel.py" "$@"
fi
export ROBOT_REMOTE_TARGET=ethernet
source "$HERE/../ros_env.sh"  # private local ROS graph (domain 42, localhost) - same as dual_arm_driver.launch.py
source /opt/ros/humble/setup.bash
source "$HERE/../igus_ws/install/setup.bash"
cd "$HERE"
exec "$HERE/../.venv_quest/bin/python" "$HERE/quest_dual_arm_teleop.py" --backend real-ros "$@"
