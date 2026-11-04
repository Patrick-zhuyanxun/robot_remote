#!/usr/bin/env bash
# Raw recorder of the cutting runs: one rosbag2 (sqlite3) per execution, full resolution, no conversion. Plain system
# python3 + /opt/ros/humble (no LeRobot). READ-ONLY (subscriptions only). The cutting launches of igus_rebel_ros2 start
# record_raw_cut.py themselves (cut_recorder_backend:=raw); this script is for running it by hand.
#   run_record_raw_cut.sh --trigger dual-cut                 # dual cell
#   run_record_raw_cut.sh --trigger fjt-status               # single arm (every executed trajectory)
#   run_record_raw_cut.sh --trigger dual-cut --camera-domain 11
# ROS network: CUT_DOMAIN_ID (default 11, the cutting stack's domain), RMW Fast DDS, localhost-only off unless
# CUT_LOCALHOST_ONLY=1. IGUS_WS (default ~/igus_rebel_ros2) provides dual_cut_msgs for --trigger dual-cut.
# Ctrl-C / SIGTERM closes the open bag cleanly. Output: see recording/record_raw_cut.py.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
IGUS_WS="${IGUS_WS:-$HOME/igus_rebel_ros2}"
[ -f "$IGUS_WS/install/setup.bash" ] && source "$IGUS_WS/install/setup.bash"
export ROS_DOMAIN_ID="${CUT_DOMAIN_ID:-11}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [ "${CUT_LOCALHOST_ONLY:-0}" = 1 ]; then export ROS_LOCALHOST_ONLY=1; else unset ROS_LOCALHOST_ONLY; fi
echo "[run_record_raw_cut] ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset} (read-only recorder)"
exec python3 "$HERE/record_raw_cut.py" "$@"
