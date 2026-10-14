#!/usr/bin/env bash
# Start the ROS 2-driven dual-arm ReBeL rig in Isaac Sim (6.1 default, 5.1 supported) using Isaac Sim's bundled ROS 2 Humble libraries.
# Run this in a terminal where /opt/ros/humble is NOT sourced (system ROS is Python 3.10, Isaac Sim is 3.11).
set -euo pipefail

ISAAC_SIM="${ISAAC_SIM:-$HOME/isaac-sim-6.1}"  # ISAAC_SIM=~/isaac-sim for 5.1
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../ros_env.sh"  # private ROS 2 domain, localhost only (see ros_env.sh)

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# bundled Humble libs: isaacsim.ros2.core (6.x) or isaacsim.ros2.bridge (5.x)
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$ISAAC_SIM/exts/isaacsim.ros2.core/humble/lib:$ISAAC_SIM/exts/isaacsim.ros2.bridge/humble/lib"
unset PYTHONPATH AMENT_PREFIX_PATH

exec "$ISAAC_SIM/python.sh" "$HERE/run_dual_arm_sim.py" "$@"
