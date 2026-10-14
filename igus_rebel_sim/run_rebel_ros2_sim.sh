#!/usr/bin/env bash
# Start the ROS 2-driven ReBeL in Isaac Sim 5.1 using Isaac Sim's bundled ROS 2 Humble libraries.
# Run this in a terminal where /opt/ros/humble is NOT sourced (system ROS is Python 3.10, Isaac Sim is 3.11).
set -euo pipefail

ISAAC_SIM="${ISAAC_SIM:-$HOME/isaac-sim}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../ros_env.sh"  # private ROS 2 domain, localhost only (see ros_env.sh)

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$ISAAC_SIM/exts/isaacsim.ros2.bridge/humble/lib"
unset PYTHONPATH AMENT_PREFIX_PATH

exec "$ISAAC_SIM/python.sh" "$HERE/rebel_ros2_sim.py" "$@"
