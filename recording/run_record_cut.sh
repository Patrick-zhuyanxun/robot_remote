#!/usr/bin/env bash
# Record the robot's CUTTING runs (~/igus_rebel_ros2) into a LeRobotDataset (v3.0, 14-D state/action like the other
# real-arm datasets): ONE EPISODE PER EXECUTE CALL. READ-ONLY - the recorder only subscribes, it never publishes,
# calls services or sends trajectories (the cut is started from igus_rebel_ros2 as usual).
#
#   dual cell   (default)  dual_cut_coordinator: episode starts when it enters WAITING_BARRIER / EXECUTING, spans every
#                          phase (A, barrier, B), is SAVED when the execution ends normally and DISCARDED when it is
#                          STOPPED / aborted (--keep-stopped saves it, task suffixed " [stopped: <reason>]").
#     observation.state <- /joint_states (arm_{left,right}_joint1..6)
#     action            <- the JTC setpoints, /arm_{side}_trajectory_controller/controller_state (`reference`)
#     status            <- /dual_cut_coordinator/status (dual_cut_msgs) + /dual_cut_coordinator/exec_report
#     robot_type igus_rebel_dual_arm_real_cut; extra feature observation.cut_progress = [coordinator state, phase]
#   --single-arm           box_h_cut_executor T-cut sequence (/box_h_cut_executor/t_cut_status): joints are plain
#                          joint1..6 and /rebel_arm_trajectory_controller; mapped to the side of --arms (default right);
#                          the other arm is the constant --other-arm-deg. robot_type igus_rebel_single_arm_real_cut,
#                          cut_progress = [step index, status].
#   Codes: see recording/cut_trigger.py and README "Record cutting runs".
#
# Cameras are read over ROS topics (never the in-process USB reader: it would fight the running camera drivers).
# Default --camera-preset cut = head + both gripper cameras (what `ros2 launch rebel_cameras cameras.launch.py` runs by
# default). Others: --camera-preset cell, --camera wrist_left_knife=/arm_left/wrist_camera/color/image_raw/compressed
# (wrist 305s: start the launch with wrist:=true right_wrist:=true), --camera knife=/knife_camera/image_raw/compressed
# (ros2 launch knife_camera knife_camera.launch.py). --list-cameras shows what is publishing.
#
# ROS network: the cutting stack of igus_rebel_ros2 runs on ROS_DOMAIN_ID 11 with the default RMW (Fast DDS) and
# localhost-only OFF, so that is what this script uses (this is the ROBOT's domain; the recorder only listens).
#   CUT_DOMAIN_ID=<n>        another domain (tests / mock stacks)
#   CUT_LOCALHOST_ONLY=1     export ROS_LOCALHOST_ONLY=1 (mock stack started localhost-only)
#   IGUS_WS=<dir>            igus_rebel_ros2 workspace (default ~/igus_rebel_ros2; dual cell needs its install/ for dual_cut_msgs)
#
# Keyboard: Enter = start/save by hand (the trigger keeps working), d = discard, q = quit. During a CRI MoveJoint transit
# (opt-in, off by default) the arm's JTC is inactive and its controller_state stops: the action goes stale, such frames
# are counted in the log ("N with a stale action"), no action is invented.
#
#   run_record_cut.sh --list-cameras
#   run_record_cut.sh --name cut_dual_001
#   run_record_cut.sh --name cut_single_001 --single-arm --arms right --camera knife=/knife_camera/image_raw/compressed
#   run_record_cut.sh --name cut_dual_001 --keep-stopped --task "cut the carton tape"
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
trigger=dual-cut
args=()
for a in "$@"; do
  if [ "$a" = "--single-arm" ]; then trigger=h-cut; else args+=("$a"); fi
done
set +u
source /opt/ros/humble/setup.bash
if [ "$trigger" = dual-cut ]; then
  IGUS_WS="${IGUS_WS:-$HOME/igus_rebel_ros2}"
  if [ ! -f "$IGUS_WS/install/setup.bash" ]; then
    echo "[run_record_cut] ERROR: $IGUS_WS/install/setup.bash not found (dual_cut_msgs is needed; set IGUS_WS)" >&2
    exit 2
  fi
  source "$IGUS_WS/install/setup.bash"
fi
export ROS_DOMAIN_ID="${CUT_DOMAIN_ID:-11}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [ "${CUT_LOCALHOST_ONLY:-0}" = 1 ]; then export ROS_LOCALHOST_ONLY=1; else unset ROS_LOCALHOST_ONLY; fi
echo "[run_record_cut] trigger=$trigger ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset} (read-only recorder)"
export HF_HUB_OFFLINE=1   # local dataset only - never fall back to the Hub
exec "${LEROBOT_PY:-python3}" "$HERE/record_dataset.py" --source moveit-real --episode-trigger "$trigger" \
  --camera-preset cut "${args[@]}"
