#!/usr/bin/env bash
# Record the REAL ReBeL arm while a MoveIt / scripted waypoint program (running on ANOTHER PC) drives it through the
# arm's JointTrajectoryController, into a LeRobotDataset (v3.0, 14-D state/action like the Isaac datasets).
#   observation.state <- /joint_states (joint1..6)
#   action            <- the trajectory controller's SETPOINT: control_msgs/JointTrajectoryControllerState on
#                        /rebel_arm_trajectory_controller/controller_state (`reference`, or `desired` on an older JTC)
#   images            <- --camera KEY=TOPIC / --camera-preset (same cameras as the mocap recorder)
# Grippers are 0.0; an undriven arm is the constant --other-arm-deg. No observation.follower_state feature.
# READ-ONLY: the recorder only subscribes, it never publishes, calls services or sends trajectories. The waypoint
# program on the other PC must send to the controller that runs here. Runs with system python3 (3.10) +
# /opt/ros/humble rclpy. Needs a LeRobot python ($LEROBOT_PY, default python3; user-site lerobot 0.4.4 works - then an
# existing dataset cannot be resumed, use a new --name each time).
# ROS network: ROBOT_REMOTE_TARGET=real -> ROS_DOMAIN_ID=${ROBOT_REMOTE_REAL_DOMAIN_ID:-12}, Fast DDS, localhost-only
# OFF. Set ROBOT_REMOTE_REAL_DOMAIN_ID to use another domain (e.g. for tests).
#
#   run_record_moveit.sh --list-cameras
#   run_record_moveit.sh --name box_open_01 --task "open the box" --camera-preset cell-right
#   # dual-arm stack (one /joint_states with arm_*_joint names, one controller per side):
#   run_record_moveit.sh --name box_open_both --task "open the box" --arms left right --camera-preset cell \
#       --state-names "arm_{side}_joint{i}" --action-names "arm_{side}_joint{i}" \
#       --controller-state-topic "/arm_{side}_trajectory_controller/controller_state"
# Keyboard: Enter = start/save episode, d = discard, q = quit.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
ROBOT_REMOTE_TARGET=real source "$HERE/../ros_env.sh"
echo "[run_record_moveit] ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset} (read-only recorder)"
export HF_HUB_OFFLINE=1   # local dataset only - never fall back to the Hub
exec "${LEROBOT_PY:-python3}" "$HERE/record_dataset.py" --source moveit-real "$@"
