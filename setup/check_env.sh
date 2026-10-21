#!/usr/bin/env bash
# Check that everything robot_remote needs is installed. Prints one line per item; exit code = number of failures.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
R="$(cd "$HERE/.." && pwd)"
# What to check follows what setup/install.sh was told to install (config/install_profile.env); no file = everything.
# The profile only switches parts on/off. Paths come from the environment, exactly as the launchers see them.
RR_SIM=1; RR_RECORD=1
[ -f "$R/config/install_profile.env" ] && . "$R/config/install_profile.env"
for a in "$@"; do case "$a" in --no-sim) RR_SIM=0 ;; --no-record) RR_RECORD=0 ;; --all) RR_SIM=1; RR_RECORD=1 ;;
  *) echo "usage: check_env.sh [--no-sim] [--no-record] [--all]" >&2; exit 2 ;; esac; done
ISAAC_SIM="${ISAAC_SIM:-$HOME/isaac-sim-6.1}"
fails=0
ok()  { printf '  \033[32mOK\033[0m    %s\n' "$1"; }
bad() { printf '  \033[31mFAIL\033[0m  %s\n        -> %s\n' "$1" "$2"; fails=$((fails+1)); }
skip() { printf '  \033[2mskip\033[0m  %s (not selected at install)\n' "$1"; }
chk() { if eval "$2" >/dev/null 2>&1; then ok "$1"; else bad "$1" "$3"; fi; }

echo "robot_remote environment check"
chk "Ubuntu 22.04"                  'grep -q "22.04" /etc/os-release'                       "tested on Ubuntu 22.04 only"
chk "NVIDIA driver"                 'nvidia-smi'                                              "install the NVIDIA driver (580+)"
chk "ROS 2 Humble"                  'test -f /opt/ros/humble/setup.bash'                      "install ros-humble-desktop (Install page, step 2)"
chk "xacro, colcon, rosdep"         'test -x /opt/ros/humble/bin/xacro && command -v colcon && command -v rosdep' "sudo apt install ros-humble-xacro python3-colcon-common-extensions python3-rosdep"
chk "uv"                            'command -v uv'                                           "curl -LsSf https://astral.sh/uv/install.sh | sh"
if [ "$RR_SIM" = 1 ]; then
chk "Isaac Sim at $ISAAC_SIM"       'test -x "$ISAAC_SIM/python.sh"'                          "setup/install.sh --isaac download, or export ISAAC_SIM=/your/path"
chk "Isaac bundled ROS 2 rclpy"     'test -d "$ISAAC_SIM/exts/isaacsim.ros2.core/humble/rclpy"' "needs Isaac Sim 6.x"
else skip "Isaac Sim (simulation, and the rclpy the recorder borrows)"; fi
chk "igus_ws built"                 'test -f "$R/igus_ws/install/igus_rebel_description/share/igus_rebel_description/urdf/igus_rebel_xeg32_wrapper.urdf.xacro"' "setup/install_ros_ws.sh"
chk "open_loop_control patch"       'grep -q "open_loop_control: true" "$R/igus_ws/src/Igus_rebel/src/igus_rebel/config/controller_mock.yaml"' "setup/install_ros_ws.sh (applies igus_ws/patches)"
chk ".venv_quest (isaacteleop+rclpy)" 'bash -c "source /opt/ros/humble/setup.bash && \"$R/.venv_quest/bin/python\" -c \"import isaacteleop, rclpy, scipy\""' "setup/install_quest_env.sh"
LP="${LEROBOT_PY:-$R/.venv_record/bin/python}"; [ -x "$LP" ] || LP="$HOME/Robotic_infra/lerobot/.venv/bin/python"
if [ "$RR_RECORD" = 1 ]; then
chk "LeRobot 0.5.x for recording ($LP)" '"$LP" -c "import importlib.metadata as m, sys; sys.exit(not m.version(\"lerobot\").startswith(\"0.5\"))"' "setup/install_record_env.sh, or export LEROBOT_PY=..."
else skip "LeRobot recorder environment"; fi
[ "$RR_RECORD" = 1 ] && [ "$RR_SIM" = 0 ] && echo "  note  recording and replay need Isaac Sim's rclpy: they will not start without it"
chk "CloudXR EULA accepted"         'ls "$HOME/.cloudxr"'                                     "$R/.venv_quest/bin/python -m isaacteleop.cloudxr --accept-eula"
chk "tkinter for the launcher"      '/usr/bin/python3 -c "import tkinter"'                    "sudo apt install python3-tk"
echo
[ $fails -eq 0 ] && echo "All good. Start with: $R/launcher/run_gui.sh" || echo "$fails item(s) to fix."
exit $fails
