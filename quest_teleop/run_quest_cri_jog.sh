#!/usr/bin/env bash
# Quest -> one or BOTH real igus ReBeLs through the CONTROLLER'S OWN Cartesian jog (--backend cri-jog): the controller
# does the IK, this PC only streams a Cartesian velocity (translation only) to the igus_rebel driver(s), with a reactive
# arm-to-arm guard. Fake-tested only; the driver's %-to-m/s mapping is unmeasured.
#
# ONE ARM
#   1. Driver (other terminal, SAME ROS_DOMAIN_ID / localhost setting as printed below), from an igus_rebel_ros2 install
#      that has the cart_jog_* interface:
#        ros2 launch igus_rebel rebel.launch.py hardware_protocol:=rebel robot_ip:=<ip>
#      then deactivate the trajectory controller (the driver refuses the mode otherwise):
#        ros2 control switch_controllers --deactivate rebel_arm_trajectory_controller
#   2. Start with the e-stop in reach, first without moving anything:
#        ./quest_teleop/run_quest_cri_jog.sh --arms right --dry-run
#      then:  ./quest_teleop/run_quest_cri_jog.sh --arms right [--cri-max-lin-vel 0.01] [--cri-signs 1,1,1]
#      (topics /igus_rebel/...; the other arm is assumed at --other-arm-deg, or --watch-other reads
#       /igus_rebel_{side}/robot_cart_pose read-only)
#
# TWO ARMS (dual stack: one driver instance per arm, node igus_rebel_left / igus_rebel_right, topics /igus_rebel_{side}/...)
#   1. Launch the dual stack driver of igus_rebel_ros2 (rebel_dual_table_description, hardware_protocol:=rebel) on this
#      ROS_DOMAIN_ID, then deactivate BOTH trajectory controllers:
#        ros2 control switch_controllers --deactivate arm_left_trajectory_controller arm_right_trajectory_controller
#   2. ./quest_teleop/run_quest_cri_jog.sh --arms left right --dry-run    (then without --dry-run, low speed)
#      Each hand drives its arm (--swap-hands swaps); a fault of one arm stops both; the arm guard needs the URDF
#      (igus_rebel_description from the sourced install) and refuses to start without it unless --no-arm-guard
#      (then NOTHING keeps the arms apart). The guard is reactive; stand, table and cameras are not modelled.
#      Other topic names: --cri-cmd-topic /x_{side}/cart_jog_cmd (also --cri-enable-service / --cri-state-topic /
#      --cri-pose-topic); two arms must resolve to different names.
#   Exit (Ctrl+C / Stop all): zero twist to every arm, then cart_jog_enable false on each. No home / park motion.
#
# Overridable: IGUS_REBEL_INSTALL (path to the igus_rebel_ros2 install/setup.bash that has igus_rebel_msgs
# RobotCartPose / HardwareState), QUEST_PYTHON (python with isaacteleop + scipy; default .venv_quest),
# ROBOT_REMOTE_TARGET / ROBOT_REMOTE_DOMAIN_ID (ROS graph, see ros_env.sh; default: local domain 42, localhost only).
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT_REMOTE_TARGET="${ROBOT_REMOTE_TARGET:-ethernet}"   # local graph unless told otherwise (ros_env.sh)
source "$HERE/../ros_env.sh"
source /opt/ros/humble/setup.bash
IGUS_SETUP="${IGUS_REBEL_INSTALL:-}"
if [[ -z "$IGUS_SETUP" ]]; then
  # Only an install whose igus_rebel_msgs has RobotCartPose has the cart_jog_* driver (2026-10: the jtc-tracking
  # worktree; the main ~/igus_rebel_ros2 install does not yet).
  for cand in "$HOME/igus_rebel_ros2/install" "$HOME/orca/workspaces/igus_rebel_ros2/jtc-tracking/install"; do
    if [[ -f "$cand/setup.bash" && -f "$cand/igus_rebel_msgs/share/igus_rebel_msgs/msg/RobotCartPose.msg" ]]; then
      IGUS_SETUP="$cand/setup.bash"; break
    fi
  done
fi
if [[ -z "$IGUS_SETUP" || ! -f "$IGUS_SETUP" ]]; then
  echo "ERROR: igus_rebel_ros2 install not found. Set IGUS_REBEL_INSTALL=/path/to/install/setup.bash (a build that has" >&2
  echo "       igus_rebel_msgs RobotCartPose / HardwareState.cart_jog_*)." >&2
  exit 1
fi
source "$IGUS_SETUP"
PY="${QUEST_PYTHON:-$HERE/../.venv_quest/bin/python}"
if [[ ! -x "$PY" ]]; then
  echo "WARNING: $PY not found (run setup/install_quest_env.sh); using python3 - the Quest reader needs isaacteleop." >&2
  PY=python3
fi
echo "cri-jog: ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}; driver install: $IGUS_SETUP" >&2
cd "$HERE"
exec "$PY" "$HERE/quest_dual_arm_teleop.py" --backend cri-jog "$@"
