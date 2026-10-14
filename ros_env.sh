# Sourced by every robot_remote launcher. Keeps this machine's ROS 2 graph private.
#
# ROS 2 discovers every node with the same ROS_DOMAIN_ID on the LAN. Another machine on 192.168.50.x runs
# the same compal_robot dual-arm Isaac rig (DualArmRigROS2Bridge, /clock ~37 h) on domain 0, so with the
# defaults our /dual_arm/* commands also drove ITS arms and its /clock + joint states leaked into ours.
#
# Use the same values in any extra terminal where you run `ros2 ...` by hand:
#   source <path-to>/robot_remote/ros_env.sh
export ROS_DOMAIN_ID="${ROBOT_REMOTE_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
