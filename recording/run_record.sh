#!/usr/bin/env bash
# Record the Isaac dual-arm Quest teleop into a LeRobotDataset.
# Runs with a LeRobot 0.5.x venv (Python 3.12, see setup/install_record_env.sh) and Isaac Sim 6.1's bundled
# Python-3.12 ROS 2 Humble rclpy. Do NOT source /opt/ros in this terminal (that rclpy is Python 3.10).
#   run_record.sh --name dual_rebel_pick --task "pick up the box" [--cameras head wrist_left wrist_right]
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../ros_env.sh"  # private ROS 2 domain, localhost only (see ros_env.sh)
ISAAC_SIM="${ISAAC_SIM:-$HOME/isaac-sim-6.1}"
ROS="$ISAAC_SIM/exts/isaacsim.ros2.core/humble"
export HF_HUB_OFFLINE=1   # local dataset only - never fall back to the Hub

if [ ! -d "$ROS/rclpy" ] || [ "${ROBOT_REMOTE_RECORD_PY:-}" = system ]; then
  # No Isaac Sim 6.1 here (e.g. a PC that only drives the real arms): system python3 (3.10) + /opt/ros/humble rclpy,
  # like run_record_mocap.sh. A LeRobot without LeRobotDataset.resume (0.4.x) works but cannot append to an existing
  # dataset (record_dataset.py refuses that before opening ROS) - use a new dataset name per session.
  # $LEROBOT_PY must then be a Python 3.10 with lerobot (default python3); the 3.12 .venv_record cannot load this rclpy.
  LEROBOT_PY="${LEROBOT_PY:-python3}"
  set +u
  source /opt/ros/humble/setup.bash
  source "$HERE/../ros_env.sh"   # again: keep the domain / RMW / localhost settings chosen above
  echo "[run_record] Isaac Sim 6.1 rclpy not found at $ROS -> system ROS 2 Humble + $LEROBOT_PY" \
       "(ROS_DOMAIN_ID=$ROS_DOMAIN_ID)" >&2
  exec "$LEROBOT_PY" "$HERE/record_dataset.py" "$@"
fi

# LeRobot 0.5.x Python: $LEROBOT_PY, else the repo venv from setup/install_record_env.sh, else ~/Robotic_infra/lerobot
if [ -z "${LEROBOT_PY:-}" ]; then
  LEROBOT_PY="$HERE/../.venv_record/bin/python"
  [ -x "$LEROBOT_PY" ] || LEROBOT_PY="$HOME/Robotic_infra/lerobot/.venv/bin/python"
fi

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONPATH="$ROS/rclpy"
export LD_LIBRARY_PATH="$ROS/lib:${LD_LIBRARY_PATH:-}"
exec "$LEROBOT_PY" "$HERE/record_dataset.py" "$@"
