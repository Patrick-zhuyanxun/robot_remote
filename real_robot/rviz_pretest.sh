#!/usr/bin/env bash
# Pre-test of ONE physical igus ReBeL from RViz / MoveIt, before any Quest teleop.
# One command: starts the igus ROS driver (CRI, Igus_rebel rebel.launch.py), waits until it is connected,
# asks you before enabling the motors, then opens MoveIt + RViz. Ctrl+C (or closing RViz) stops everything.
#
#   ./real_robot/rviz_pretest.sh            # left arm (config/real_arms.json -> 192.168.3.11)
#   ./real_robot/rviz_pretest.sh right      # right arm
#   ./real_robot/rviz_pretest.sh left --yes # do not ask before enabling the motors
#   ./real_robot/rviz_pretest.sh --mock     # no robot: mock hardware, to try the RViz workflow
#
# The ROS graph stays on this PC (ROS_DOMAIN_ID 91, localhost only); the arm itself is reached over CRI/TCP.
# Do not run it together with the Quest teleop or another driver for the same arm: only one CRI client
# can control an arm, and this driver takes control (SetActive) when it connects.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
R="$(cd "$HERE/.." && pwd)"

SIDE=left YES=0 MOCK=0
for a in "$@"; do
  case "$a" in
    left|right) SIDE=$a ;;
    --yes|-y) YES=1 ;;
    --mock) MOCK=1 ;;
    -h|--help) sed -n 2,14p "$0"; exit 0 ;;
    *) echo "unknown argument: $a (use left|right, --yes, --mock)" >&2; exit 2 ;;
  esac
done
IP=$(python3 -c "import json,sys; print(json.load(open('$R/config/real_arms.json'))['$SIDE']['host'])")
PROTO=rebel; [ $MOCK = 1 ] && PROTO=mock_hardware

# Refuse to start next to another controller of the arm.
if pgrep -f "^[^ ]*python[0-9.]* [^ ]*quest_dual_arm_teleop.py --backend real" >/dev/null; then
  echo "The Quest real-arm teleop is running. Stop it first (only one program may control the arm)." >&2; exit 1
fi
if pgrep -f "^[^ ]*/controller_manager/ros2_control_node" >/dev/null; then
  echo "A ros2_control_node is already running (another driver / earlier pre-test?). Stop it first." >&2; exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$R/igus_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROBOT_REMOTE_RVIZ_DOMAIN_ID:-91}" ROS_LOCALHOST_ONLY=1

mkdir -p "$R/logs"
LOG="$R/logs/rviz_pretest_${SIDE}_$(date +%Y%m%d_%H%M%S).log"
DRV=""
cleanup() {
  trap - EXIT INT TERM
  if [ -n "$DRV" ] && kill -0 "$DRV" 2>/dev/null; then
    echo "[pretest] stopping the driver..."
    kill -INT -- "-$DRV" 2>/dev/null || true   # whole process group: launch + driver + spawners + RSP
    for _ in $(seq 1 20); do pgrep -g "$DRV" >/dev/null || break; sleep 0.5; done
    kill -TERM -- "-$DRV" 2>/dev/null || true
  fi
  echo "[pretest] done. Driver log: $LOG"
}
trap cleanup EXIT INT TERM

echo "[pretest] $SIDE arm, $([ $MOCK = 1 ] && echo 'MOCK hardware (no robot)' || echo "robot $IP:3920"), ROS_DOMAIN_ID $ROS_DOMAIN_ID (local only)"
echo "[pretest] 1/3 starting the igus driver (log: $LOG)"
setsid ros2 launch igus_rebel rebel.launch.py hardware_protocol:=$PROTO robot_ip:=$IP > "$LOG" 2>&1 &
DRV=$!

for _ in $(seq 1 60); do
  kill -0 "$DRV" 2>/dev/null || { echo "[pretest] the driver exited:"; tail -20 "$LOG"; exit 1; }
  grep -q "Configured and activated.*rebel_arm_trajectory_controller" "$LOG" && break
  sleep 0.5
done
grep -q "Configured and activated.*rebel_arm_trajectory_controller" "$LOG" || {
  echo "[pretest] driver not ready after 30 s - is the arm on (button ring red) and reachable at $IP?"; tail -20 "$LOG"; exit 1; }
grep -m1 -E "First STATUS" "$LOG" | sed 's/^.*igus_rebel\]: /[pretest]   /' || true

if [ $MOCK = 0 ]; then
  echo "[pretest] 2/3 enable the motors of the $SIDE arm ($IP)"
  if [ $YES = 0 ]; then
    echo "          The arm will hold its pose (it does not move by itself). Someone at the e-stop?"
    read -r -p "          Enable now? [y/N] " ok
    [[ "$ok" =~ ^[Yy]$ ]] || { echo "[pretest] not enabled - stopping."; exit 1; }
  fi
  # The driver also sends Reset + Enable when it connects, but that first attempt has not stuck on our arm;
  # this service re-sends the start-up sequence and reports whether the drives really came up.
  OUT=$(ros2 service call /igus_rebel/reset_and_enable std_srvs/srv/Trigger 2>&1 || true)
  if ! grep -q "success=True" <<<"$OUT"; then
    echo "[pretest] enabling failed:"; grep -E "message=|error" <<<"$OUT" | head -5
    echo "          Check: e-stop released, button ring red/yellow (not blue), errors in $LOG"; exit 1
  fi
  echo "[pretest]   motors enabled (button ring should be GREEN)"
else
  echo "[pretest] 2/3 mock hardware: nothing to enable"
fi

echo "[pretest] 3/3 MoveIt + RViz. In RViz: Velocity/Acceleration Scaling 0.05, Plan, check, Execute."
echo "          Close RViz or press Ctrl+C here to stop everything."
ros2 launch igus_rebel_moveit_config move_group.launch.py hardware_protocol:=$PROTO \
  use_gui:="${ROBOT_REMOTE_RVIZ_GUI:-true}" run_teleop:=false run_box_h_cut_executor:=false run_camera_pole:=false
