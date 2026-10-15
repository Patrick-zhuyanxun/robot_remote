#!/usr/bin/env bash
# Quest -> left/right physical ReBeLs over Ethernet CRI + Position Interface.
# --check-only uses system Python and sends only queries / zero-jog heartbeats.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ " $* " == *" --check-only "* ]]; then
  exec /usr/bin/python3 "$HERE/ethernet_rebel.py" "$@"
fi
export ROBOT_REMOTE_TARGET=ethernet
source "$HERE/../ros_env.sh"  # local ROS diagnostics only; arm commands use TCP
source /opt/ros/humble/setup.bash
source "$HERE/../igus_ws/install/setup.bash"
cd "$HERE"
exec "$HERE/../.venv_quest/bin/python" "$HERE/quest_dual_arm_teleop.py" --backend real "$@"
