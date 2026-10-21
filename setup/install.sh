#!/usr/bin/env bash
# robot_remote installer: asks in plain text what to install, then runs the setup/install_*.sh steps.
# Safe to re-run: finished parts are detected and skipped. It never runs sudo itself (it prints the commands for you);
# the rosdep call inside install_ros_ws.sh may ask for your password for missing apt packages.
#
#   ./setup/install.sh                      # asks every question
#   ./setup/install.sh --dry-run            # asks, prints the plan, changes nothing
#   ./setup/install.sh --isaac download --record new --no-irc      # answer on the command line (agents / scripts)
#
# Answers can also be piped in, one per line, in the order the questions appear.
#   --isaac download|skip|<path>   Isaac Sim 6.1: download to $ISAAC_SIM (default ~/isaac-sim-6.1), skip, or use <path>
#   --record new|skip|<python>     recorder: new .venv_record (LeRobot 0.5.1), skip, or an existing LeRobot 0.5.x python
#   --with-irc | --no-irc          also build the legacy single-arm iRC_ROS packages (default: no)
#   --accept-cloudxr-eula          run the CloudXR licence command (only a person who has read the licence may pass this)
#   --write-bashrc                 append export lines for a non-default ISAAC_SIM / LEROBOT_PY to ~/.bashrc
#   --yes                          take the recommended answer to every question not given as an option
#   --dry-run                      print the plan and stop
#   --detect                       only list what is already installed, with versions (changes nothing, asks nothing)
# Exit codes: 0 done, 2 a question had no answer (pass the option), 3 you must run the printed sudo commands first.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
R="$(cd "$HERE/.." && pwd)"
PROFILE="$R/config/install_profile.env"
DEFAULT_ISAAC="${ISAAC_SIM:-$HOME/isaac-sim-6.1}"
ISAAC_URL=https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.1.0-linux-x86_64.zip
ISAAC_ZIP="$HOME/Downloads/isaac-sim-6.1.0.zip"
APT_PKGS=(ros-humble-desktop ros-humble-xacro python3-colcon-common-extensions python3-rosdep python3-tk
          ros-humble-moveit ros-humble-moveit-servo
          ros-humble-moveit-task-constructor-capabilities ros-humble-moveit-task-constructor-visualization)

ISAAC_OPT=""; RECORD_OPT=""; IRC_OPT=""; EULA=0; DRY=0; YES=0; BASHRC=0; ALL_GIVEN=0; DETECT=0
while [ $# -gt 0 ]; do
  case "$1" in
    --isaac) ISAAC_OPT="${2:?--isaac needs download|skip|<path>}"; shift ;;
    --record) RECORD_OPT="${2:?--record needs new|skip|<python>}"; shift ;;
    --with-irc) IRC_OPT=1 ;;
    --no-irc) IRC_OPT=0 ;;
    --accept-cloudxr-eula) EULA=1 ;;
    --write-bashrc) BASHRC=1 ;;
    --yes|-y) YES=1 ;;
    --dry-run) DRY=1 ;;
    --detect) DETECT=1 ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
  esac
  shift
done

[ -n "$ISAAC_OPT" ] && [ -n "$RECORD_OPT" ] && [ -n "$IRC_OPT" ] && ALL_GIVEN=1   # nothing to confirm: the options were the answers

say()  { printf '%s\n' "$*"; }
step() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
ask() {  # ask "question" default -> prints the answer (the default on Enter, or with --yes)
  if [ "$YES" = 1 ]; then echo "$2"; return; fi
  local ans
  printf '%s [%s] ' "$1" "$2" >&2
  if ! IFS= read -r ans; then
    printf '\n' >&2
    echo "No answer for: $1" >&2
    echo "Answer interactively, pipe the answers in, or pass the matching option (see --help)." >&2
    exit 2
  fi
  echo "${ans:-$2}"
}
lerobot_ok() { [ -x "${1:-}" ] && "$1" -c 'import importlib.metadata as m, sys; sys.exit(not m.version("lerobot").startswith("0.5"))' >/dev/null 2>&1; }
run() { if [ "$DRY" = 1 ]; then say "  [dry-run] $*"; else "$@"; fi; }

# ------------------------------------------------------------------ 0. what is already here
isaac_version() { head -1 "$1/VERSION" 2>/dev/null | cut -d+ -f1; }
lerobot_version() { "$1" -c 'import importlib.metadata as m; print(m.version("lerobot"))' 2>/dev/null; }
inventory() {  # one line per component: what is installed, which version, and whether this project can use it
  local v p found
  grep -q "22.04" /etc/os-release && v=ok || v="NOT 22.04 (tested on 22.04 only)"
  printf '  %-22s %s  [%s]\n' "Ubuntu" "$(lsb_release -ds 2>/dev/null || echo unknown)" "$v"
  if command -v nvidia-smi >/dev/null 2>&1; then
    printf '  %-22s %s  [driver 580+ needed]\n' "NVIDIA GPU" "$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -1)"
  else printf '  %-22s %s\n' "NVIDIA GPU" "none found (needed by the CloudXR runtime and Isaac Sim)"; fi
  printf '  %-22s %s\n' "free space in \$HOME" "$(df -h "$HOME" | awk 'NR==2{print $4}')  (about 40 GB with Isaac Sim, 15 GB without)"
  if [ -f /opt/ros/humble/setup.bash ]; then
    printf '  %-22s %s\n' "ROS 2 Humble" "installed, ros-humble-desktop $(dpkg-query -W -f='${Version}' ros-humble-desktop 2>/dev/null || echo '(package not found)')"
  else printf '  %-22s %s\n' "ROS 2 Humble" "not installed"; fi
  printf '  %-22s %s\n' "uv" "$(command -v uv >/dev/null 2>&1 && uv --version || echo 'not installed')"
  found=0
  for p in "$DEFAULT_ISAAC" "$HOME/isaac-sim-6.1" "$HOME/isaac-sim"; do
    [ -x "$p/python.sh" ] || continue
    case " ${seen:-} " in *" $p "*) continue ;; esac; seen="${seen:-} $p"
    v="$(isaac_version "$p")"; found=1
    printf '  %-22s %s at %s  [%s]\n' "Isaac Sim" "${v:-version unknown}" "$p" "$(case "$v" in 6.1*) echo 'usable';; *) echo 'this project needs 6.1.x';; esac)"
  done
  [ $found = 1 ] || printf '  %-22s %s\n' "Isaac Sim" "not found (looked in \$ISAAC_SIM, ~/isaac-sim-6.1, ~/isaac-sim)"
  found=0
  for p in "${LEROBOT_PY:-}" "$R/.venv_record/bin/python" "$HOME/Robotic_infra/lerobot/.venv/bin/python"; do
    [ -n "$p" ] && [ -x "$p" ] || continue
    v="$(lerobot_version "$p")"; found=1
    printf '  %-22s lerobot %s at %s  [%s]\n' "Recorder env" "${v:-not installed}" "$p" "$(case "$v" in 0.5*) echo usable;; *) echo 'needs 0.5.x';; esac)"
  done
  [ $found = 1 ] || printf '  %-22s %s\n' "Recorder env" "not found"
  if [ -x "$R/.venv_quest/bin/python" ]; then
    printf '  %-22s %s\n' "Quest env" "isaacteleop $("$R/.venv_quest/bin/python" -c 'import importlib.metadata as m; print(m.version("isaacteleop"))' 2>/dev/null || echo '? (broken)')"
  else printf '  %-22s %s\n' "Quest env" "not created"; fi
  printf '  %-22s %s\n' "ROS workspace igus_ws" "$([ -d "$R/igus_ws/install/igus_rebel_description" ] && echo built || echo 'not built')"
  printf '  %-22s %s\n' "CloudXR licence" "$([ -d "$HOME/.cloudxr" ] && echo accepted || echo 'not accepted yet')"
}

step "This machine"
inventory
if [ "$DETECT" = 1 ]; then exit 0; fi

# ------------------------------------------------------------------ 1. questions
step "What do you want to install?"
say "Always installed: ROS 2 workspace (igus ReBeL + MoveIt), Quest teleop environment, CloudXR runtime."
say "Asked below: Isaac Sim, the recorder environment, the legacy single-arm packages."

# --- Isaac Sim
ISAAC_MODE=""; ISAAC_PATH=""
case "$ISAAC_OPT" in
  "") ;;
  download) ISAAC_MODE=download; ISAAC_PATH="$DEFAULT_ISAAC" ;;
  skip) ISAAC_MODE=skip ;;
  *) [ -x "$ISAAC_OPT/python.sh" ] || { echo "--isaac: $ISAAC_OPT/python.sh not found" >&2; exit 2; }
     ISAAC_MODE=use; ISAAC_PATH="$ISAAC_OPT" ;;
esac
if [ -z "$ISAAC_MODE" ]; then
  say ""
  say "Isaac Sim 6.1 runs the dual-arm simulation. It is ALSO needed to record datasets, real robot included:"
  say "the recorder borrows Isaac's bundled ROS 2 (rclpy). Skip it only for teleop of the real arms without recording."
  if [ -x "$DEFAULT_ISAAC/python.sh" ] && isaac_version "$DEFAULT_ISAAC" | grep -q '^6\.1'; then
    a=$(ask "Isaac Sim $(isaac_version "$DEFAULT_ISAAC") found at $DEFAULT_ISAAC. Use it? (y/n)" y)
    case "$a" in y*|Y*) ISAAC_MODE=use; ISAAC_PATH="$DEFAULT_ISAAC" ;; esac
  fi
fi
if [ -z "$ISAAC_MODE" ]; then
  say "  1) Download and unzip Isaac Sim 6.1 into $DEFAULT_ISAAC  (about 9 GB download, 20 GB on disk)"
  say "  2) I already have Isaac Sim 6.1 in another folder"
  say "  3) Skip: no simulation and no dataset recording"
  c=$(ask "Isaac Sim: choose 1, 2 or 3" 1)
  case "$c" in
    1) ISAAC_MODE=download; ISAAC_PATH="$DEFAULT_ISAAC" ;;
    2) p=$(ask "Folder that contains python.sh" "$DEFAULT_ISAAC")
       [ -x "${p/#\~/$HOME}/python.sh" ] || { echo "$p/python.sh not found" >&2; exit 2; }
       ISAAC_MODE=use; ISAAC_PATH="${p/#\~/$HOME}" ;;
    3) ISAAC_MODE=skip ;;
    *) echo "choose 1, 2 or 3" >&2; exit 2 ;;
  esac
fi

# --- recorder environment
REC_MODE=""; REC_PY=""
case "$RECORD_OPT" in
  "") ;;
  new) REC_MODE=new; REC_PY="$R/.venv_record/bin/python" ;;
  skip) REC_MODE=skip ;;
  *) lerobot_ok "$RECORD_OPT" || { echo "--record: $RECORD_OPT is not a Python with lerobot 0.5.x" >&2; exit 2; }
     REC_MODE=use; REC_PY="$RECORD_OPT" ;;
esac
if [ -z "$REC_MODE" ]; then
  say ""
  say "The recorder environment holds LeRobot 0.5.x. It writes the datasets, replays them and shows them in the playback window."
  for cand in "${LEROBOT_PY:-}" "$R/.venv_record/bin/python" "$HOME/Robotic_infra/lerobot/.venv/bin/python"; do
    if lerobot_ok "$cand"; then
      a=$(ask "LeRobot $(lerobot_version "$cand") found: $cand. Use it? (y/n)" y)
      case "$a" in y*|Y*) REC_MODE=use; REC_PY="$cand" ;; esac
      break
    fi
  done
fi
if [ -z "$REC_MODE" ]; then
  say "  1) Create a new environment in $R/.venv_record  (LeRobot 0.5.1 + torch, several GB)"
  say "  2) Use my own LeRobot 0.5.x environment (the one I train with)"
  say "  3) Skip: no recording, replay or playback"
  c=$(ask "Recorder: choose 1, 2 or 3" 1)
  case "$c" in
    1) REC_MODE=new; REC_PY="$R/.venv_record/bin/python" ;;
    2) p=$(ask "Path of that environment's python (e.g. ~/lerobot/.venv/bin/python)" "")
       p="${p/#\~/$HOME}"; lerobot_ok "$p" || { echo "$p is not a Python with lerobot 0.5.x" >&2; exit 2; }
       REC_MODE=use; REC_PY="$p" ;;
    3) REC_MODE=skip ;;
    *) echo "choose 1, 2 or 3" >&2; exit 2 ;;
  esac
fi
[ "$REC_MODE" != skip ] && [ "$ISAAC_MODE" = skip ] && \
  say "NOTE: without Isaac Sim, LeRobot can view and train on datasets but recording and replay will not start."

# --- legacy packages
if [ -z "$IRC_OPT" ]; then
  say ""
  a=$(ask "Also build the legacy single-arm iRC_ROS packages (only the demos in igus_rebel_sim/ use them)? (y/n)" n)
  case "$a" in y*|Y*) IRC_OPT=1 ;; *) IRC_OPT=0 ;; esac
fi

# --- plan
step "Plan"
case "$ISAAC_MODE" in
  download) say "  Isaac Sim 6.1   download $ISAAC_URL -> $ISAAC_ZIP, unzip to $ISAAC_PATH" ;;
  use)      say "  Isaac Sim 6.1   use $ISAAC_PATH" ;;
  skip)     say "  Isaac Sim 6.1   skipped (no simulation, no recording)" ;;
esac
case "$REC_MODE" in
  new)  say "  Recorder        new environment $R/.venv_record" ;;
  use)  say "  Recorder        use $REC_PY" ;;
  skip) say "  Recorder        skipped" ;;
esac
say "  ROS workspace   igus_ws (clone, patch, colcon)$([ "$IRC_OPT" = 1 ] && echo ' + iRC_ROS')"
say "  Quest env       .venv_quest (isaacteleop + CloudXR)"
say "  uv              $(command -v uv >/dev/null 2>&1 && echo present || echo 'will be installed to ~/.local/bin')"
if [ "$DRY" != 1 ] && [ "$YES" != 1 ] && [ "$ALL_GIVEN" != 1 ]; then
  a=$(ask "Go ahead? (y/n)" y)
  case "$a" in y*|Y*) ;; *) say "Nothing changed."; exit 0 ;; esac
fi

# ------------------------------------------------------------------ 2. profile (check_env.sh reads it)
if [ "$DRY" != 1 ]; then
  mkdir -p "$R/config"
  {
    echo "# written by setup/install.sh; read by setup/check_env.sh"
    echo "RR_SIM=$([ "$ISAAC_MODE" = skip ] && echo 0 || echo 1)"
    echo "RR_ISAAC=\"$([ "$ISAAC_MODE" = skip ] || echo "$ISAAC_PATH")\""
    echo "RR_RECORD=$([ "$REC_MODE" = skip ] && echo 0 || echo 1)"
    echo "RR_LEROBOT_PY=\"$([ "$REC_MODE" = skip ] || echo "$REC_PY")\""
    echo "RR_IRC=$IRC_OPT"
  } > "$PROFILE"
fi

# ------------------------------------------------------------------ 3. ROS 2 and system packages (needs sudo: printed, never run)
step "ROS 2 Humble and system packages"
missing=()
for p in "${APT_PKGS[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p"); done
if [ ${#missing[@]} -gt 0 ]; then
  say "Missing apt packages: ${missing[*]}"
  say "This needs administrator rights, so run these commands yourself in a terminal, then re-run ./setup/install.sh:"
  say ""
  if [ ! -f /opt/ros/humble/setup.bash ]; then
    cat <<'EOF'
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
EOF
  fi
  say "sudo apt update && sudo apt install -y ${missing[*]}"
  say "sudo rosdep init || true"
  say "sudo apt install --only-upgrade -y 'ros-humble-*'   # keep every ROS package on one release (MoveIt/RViz need it)"
  [ "$DRY" = 1 ] && say "(dry run: continuing the plan without it)" || exit 3
else
  say "All ROS 2 / apt packages are installed."
fi
if [ "$DRY" != 1 ] && [ ! -f "$HOME/.ros/rosdep/sources.cache/index" ] && command -v rosdep >/dev/null 2>&1; then
  rosdep update || say "rosdep update failed (network?); the workspace build may report missing keys"
fi

# ------------------------------------------------------------------ 4. uv
step "uv"
export PATH="$HOME/.local/bin:$PATH"
if command -v uv >/dev/null 2>&1; then say "uv $(uv --version | awk '{print $2}') present"
elif [ "$DRY" = 1 ]; then say "  [dry-run] curl -LsSf https://astral.sh/uv/install.sh | sh"
else curl -LsSf https://astral.sh/uv/install.sh | sh; fi

# ------------------------------------------------------------------ 5. Isaac Sim
step "Isaac Sim 6.1"
case "$ISAAC_MODE" in
  skip) say "skipped" ;;
  use)  say "using $ISAAC_PATH" ;;
  download)
    if [ -x "$ISAAC_PATH/python.sh" ]; then say "already unzipped at $ISAAC_PATH"
    else
      run mkdir -p "$HOME/Downloads" "$ISAAC_PATH"
      run curl -L -C - --retry 5 -o "$ISAAC_ZIP" "$ISAAC_URL"
      run unzip -q -o "$ISAAC_ZIP" -d "$ISAAC_PATH"
      say "The zip ($ISAAC_ZIP, about 9 GB) is no longer needed; delete it when you like."
    fi ;;
esac
if [ "$DRY" != 1 ] && [ "$ISAAC_MODE" != skip ]; then
  head -c 5 "$ISAAC_PATH/VERSION" | grep -q '^6\.1' || { echo "$ISAAC_PATH/VERSION is not 6.1.x" >&2; exit 1; }
fi

# ------------------------------------------------------------------ 6. workspace, Quest env, recorder env
step "ROS workspace (about 5 minutes)"
if [ "$DRY" = 1 ]; then say "  [dry-run] setup/install_ros_ws.sh$([ "$IRC_OPT" = 1 ] && echo ' --with-irc')"
else "$HERE/install_ros_ws.sh" $([ "$IRC_OPT" = 1 ] && echo --with-irc) 2>&1 | tee "$HERE/install_ros_ws.log"; fi
step "Quest teleop environment"
run "$HERE/install_quest_env.sh"
step "Recorder environment"
case "$REC_MODE" in
  new)  run "$HERE/install_record_env.sh" ;;
  use)  say "using $REC_PY" ;;
  skip) say "skipped" ;;
esac

# ------------------------------------------------------------------ 7. paths the launchers need, licence, firewall
step "Paths and licence"
persist() {  # persist NAME VALUE: the launchers read these two variables from the environment
  local line="export $1=\"$2\""
  if grep -qsF "$line" "$HOME/.bashrc"; then return; fi
  if [ "$BASHRC" = 1 ] && [ "$DRY" != 1 ]; then echo "$line" >> "$HOME/.bashrc"; say "added to ~/.bashrc: $line"
  else say "Add this line to ~/.bashrc (new terminals need it):  $line"; fi
}
[ "$ISAAC_MODE" != skip ] && [ "$ISAAC_PATH" != "$HOME/isaac-sim-6.1" ] && [ "$ISAAC_PATH" != "${ISAAC_SIM:-}" ] && persist ISAAC_SIM "$ISAAC_PATH"
[ "$REC_MODE" = use ] && [ "$REC_PY" != "$R/.venv_record/bin/python" ] && \
  [ "$REC_PY" != "$HOME/Robotic_infra/lerobot/.venv/bin/python" ] && persist LEROBOT_PY "$REC_PY"

CMD=("$R/.venv_quest/bin/python" -m isaacteleop.cloudxr --accept-eula)
if [ -d "$HOME/.cloudxr" ]; then say "CloudXR licence already accepted."
elif [ "$EULA" = 1 ] && [ "$DRY" != 1 ]; then "${CMD[@]}"
else
  say "The CloudXR licence must be accepted by a person. Read it and accept it once per machine:"
  say "  ${CMD[*]}"
fi
say ""
say "If the firewall is on (sudo ufw status), open the CloudXR ports for the Quest:"
say "  sudo ufw allow 47998/udp && sudo ufw allow 48322/tcp && sudo ufw allow 49100/tcp"

# ------------------------------------------------------------------ 8. check
step "Check"
if [ "$DRY" = 1 ]; then say "dry run finished; nothing was installed."; exit 0; fi
# check what the launchers will see: they read these two paths from the environment
[ "$ISAAC_MODE" != skip ] && export ISAAC_SIM="$ISAAC_PATH"
[ "$REC_MODE" != skip ] && export LEROBOT_PY="$REC_PY"
"$HERE/check_env.sh"
