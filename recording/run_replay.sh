#!/usr/bin/env bash
# Replay a recorded episode into the Isaac dual-arm rig and compare with the recorded states.
# Runs with a LeRobot 0.5.x venv (Python 3.12, see setup/install_record_env.sh) and Isaac Sim 6.1's bundled
# Python-3.12 ROS 2 Humble rclpy. Do NOT source /opt/ros in this terminal (that rclpy is Python 3.10).
#   run_replay.sh --name dual_rebel_pick --episode 0
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../ros_env.sh"  # private ROS 2 domain, localhost only (see ros_env.sh)
ISAAC_SIM="${ISAAC_SIM:-$HOME/isaac-sim-6.1}"
# LeRobot 0.5.x Python: $LEROBOT_PY, else the repo venv from setup/install_record_env.sh, else ~/Robotic_infra/lerobot
if [ -z "${LEROBOT_PY:-}" ]; then
  LEROBOT_PY="$HERE/../.venv_record/bin/python"
  [ -x "$LEROBOT_PY" ] || LEROBOT_PY="$HOME/Robotic_infra/lerobot/.venv/bin/python"
fi
ROS="$ISAAC_SIM/exts/isaacsim.ros2.core/humble"

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONPATH="$ROS/rclpy"
export LD_LIBRARY_PATH="$ROS/lib:${LD_LIBRARY_PATH:-}"
export HF_HUB_OFFLINE=1   # local dataset only - never fall back to the Hub
exec "$LEROBOT_PY" "$HERE/replay_episode.py" "$@"
