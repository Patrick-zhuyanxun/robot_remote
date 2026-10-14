#!/usr/bin/env bash
# Build the ROS 2 Humble workspace robot_remote/igus_ws (igus ReBeL description + MoveIt config).
# Clones the pinned upstream commits, applies our local patches, installs apt deps via rosdep
# (Gazebo / Orbbec camera deps are skipped - not used here) and runs colcon.
#   setup/install_ros_ws.sh            # Igus_rebel only (all the dual-arm teleop needs)
#   setup/install_ros_ws.sh --with-irc # also the legacy CommonplaceRobotics iRC_ROS (single-arm demos)
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HERE/../igus_ws"
IGUS_REBEL_URL=https://github.com/fani8j/Igus_rebel.git
IGUS_REBEL_BRANCH=hcut-path-consistency
IGUS_REBEL_COMMIT=cc0e2b61eaa34f35d75e1ff9aeb2d2bf3081c6b5
IRC_URL=https://github.com/CommonplaceRobotics/iRC_ROS.git
IRC_COMMIT=41701e6df3398087d217f0f478622e60f694de41

clone_patch() {  # url branch commit dir patch
  local dir="$WS/src/$4"
  if [ ! -d "$dir/.git" ]; then
    git clone -b "$2" "$1" "$dir"
    git -C "$dir" checkout -q "$3"
  fi
  if git -C "$dir" apply --check "$WS/patches/$5" 2>/dev/null; then
    git -C "$dir" apply "$WS/patches/$5" && echo "[ws] applied $5"
  else
    echo "[ws] $5 already applied (or does not apply) - skipped"
  fi
}

mkdir -p "$WS/src"
source /opt/ros/humble/setup.bash
clone_patch "$IGUS_REBEL_URL" "$IGUS_REBEL_BRANCH" "$IGUS_REBEL_COMMIT" Igus_rebel Igus_rebel-open_loop_control.patch
PKGS=(igus_rebel igus_rebel_moveit_config)
if [ "${1:-}" = "--with-irc" ]; then
  clone_patch "$IRC_URL" humble "$IRC_COMMIT" iRC_ROS iRC_ROS-rviz_params.patch
  PKGS+=(irc_ros_description irc_ros_bringup irc_ros_moveit_config)
fi

cd "$WS"
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "gz_ros2_control orbbec_camera image_transport_plugins" || \
  echo "[ws] rosdep reported problems (see above); continuing with the build"
colcon build --symlink-install --packages-up-to "${PKGS[@]}"
echo "[ws] done: source $WS/install/setup.bash"
