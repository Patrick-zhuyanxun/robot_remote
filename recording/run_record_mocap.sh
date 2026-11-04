#!/usr/bin/env bash
# Record the REAL right ReBeL arm driven by the OptiTrack mocap pipeline into a LeRobotDataset (v3.0, 14-D
# state/action like the Isaac datasets). READ-ONLY: the recorder only subscribes, it never publishes or calls services.
# Runs with system python3 (3.10) + /opt/ros/humble rclpy (NOT Isaac's rclpy). Needs a LeRobot python
# ($LEROBOT_PY, default python3; user-site lerobot 0.4.4 works - then an existing dataset cannot be resumed,
# use a new --name each time).
# ROS network: ROBOT_REMOTE_TARGET=real -> ROS_DOMAIN_ID=${ROBOT_REMOTE_REAL_DOMAIN_ID:-12}, Fast DDS, localhost-only OFF
# (the mocap PC publishes over the LAN). Set ROBOT_REMOTE_REAL_DOMAIN_ID to use another domain (e.g. for tests).
#
#   run_record_mocap.sh --list-cameras
#   # cameras from ~/igus_rebel_ros2 `ros2 launch rebel_cameras cameras.launch.py`: cell = all 5,
#   # cell-right / cell-left = head + that arm's gripper and knife cameras
#   run_record_mocap.sh --name right_pick --task "pick up the box" --camera-preset cell-right
#   run_record_mocap.sh --name right_pick --task "pick up the box" \
#       --camera head=/camera/color/image_raw --camera wrist_right_gripper=/arm_right/gripper_camera/image_raw/compressed
#   run_record_mocap.sh --name right_pick --task "pick up the box" --pick-cameras
#   # dual-arm stack (one /joint_states with arm_*_joint names, one status topic per side):
#   run_record_mocap.sh --name both --task "..." --arms left right --state-names "arm_{side}_joint{i}" \
#       --follower-status-topic "/mocap_follower_{side}/status" --camera head=/camera/color/image_raw
# Recommended camera keys (5-camera real cell): head, wrist_left_gripper, wrist_left_knife, wrist_right_gripper,
# wrist_right_knife. Keyboard: Enter = start/save episode, d = discard, q = quit.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
ROBOT_REMOTE_TARGET=real source "$HERE/../ros_env.sh"
echo "[run_record_mocap] ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset} (read-only recorder)"
export HF_HUB_OFFLINE=1   # local dataset only - never fall back to the Hub
exec "${LEROBOT_PY:-python3}" "$HERE/record_dataset.py" --source mocap-real "$@"
