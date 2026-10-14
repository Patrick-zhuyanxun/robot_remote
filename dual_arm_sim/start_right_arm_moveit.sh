#!/usr/bin/env bash
# MoveIt (RViz) -> right arm of the Isaac dual-arm rig.
# Start the simulator first, in another terminal WITHOUT /opt/ros sourced:
#   robot_remote/dual_arm_sim/run_dual_arm_sim.sh
# then run this script. Ctrl+C stops everything it started.
#
# Chain: RViz/move_group -> rebel_arm_trajectory_controller (ros2_control, topic_based isaac_topic)
#        -> /isaac_joint_commands -> arm_relay.py -> /dual_arm/isaac_joint_commands -> Isaac
set -eo pipefail

SIDE="${SIDE:-right}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../ros_env.sh"  # private ROS 2 domain, localhost only (see ros_env.sh)
WS="$HERE/../igus_ws"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

python3 "$HERE/arm_relay.py" --side "$SIDE" &
pids+=($!)

# ros2_control + controllers + robot_state_publisher; hardware = topic_based_ros2_control
ros2 launch igus_rebel rebel.launch.py hardware_protocol:=isaac_topic &
pids+=($!)
sleep 5

# move_group + RViz. hardware_protocol here only selects the URDF variant MoveIt loads
# (move_group.launch.py does not accept isaac_topic); motion goes through the controller above.
ros2 launch igus_rebel_moveit_config move_group.launch.py \
  hardware_protocol:=mock_hardware use_gui:=true \
  run_teleop:=false run_box_h_cut_executor:=false run_camera_pole:=false
