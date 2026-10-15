# Sourced by every robot_remote launcher. Picks the ROS 2 network for the target.
#
# Simulation (default): a private graph on this PC only. ROS 2 discovers every node with the same
# ROS_DOMAIN_ID on the LAN; another lab machine runs the same compal_robot dual-arm Isaac rig on domain 0,
# so with the defaults our /dual_arm/* commands also drove ITS arms and its /clock leaked into ours.
#
# Real robot (ROBOT_REMOTE_TARGET=real): the igus ReBeL driver PC publishes on ROS_DOMAIN_ID 11 over the
# LAN with Fast DDS (Cyclone DDS does not see it), so localhost-only must be off.
#
# Use the same values in any extra terminal where you run `ros2 ...` by hand:
#   source <path-to>/robot_remote/ros_env.sh                          # simulation
#   ROBOT_REMOTE_TARGET=real source <path-to>/robot_remote/ros_env.sh # real arm
# The new ROBOT_REMOTE_TARGET=ethernet path uses the local graph below for diagnostics only.
# Hardware commands / feedback go directly over TCP; "real" above is the legacy single-arm ROS path.
if [ "${ROBOT_REMOTE_TARGET:-sim}" = "real" ]; then
  export ROS_DOMAIN_ID="${ROBOT_REMOTE_REAL_DOMAIN_ID:-11}"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  unset ROS_LOCALHOST_ONLY
else
  export ROS_DOMAIN_ID="${ROBOT_REMOTE_DOMAIN_ID:-42}"
  export ROS_LOCALHOST_ONLY=1
fi
