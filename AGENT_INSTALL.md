# AGENT_INSTALL.md — install robot_remote on a new workstation

This file is written for a coding agent (Claude Code, Codex, Cursor, ...). A person can hand it over with:

> Install robot_remote on this machine by following AGENT_INSTALL.md in https://github.com/Patrick-zhuyanxun/robot_remote

The human-oriented version of the same steps is the **Install** page of `docs/operator_manual.html`.

## Goal and definition of done

Install everything needed to run the Meta Quest → igus ReBeL dual-arm teleop in Isaac Sim 6.1 and record
LeRobot datasets. You are done when **`./setup/check_env.sh` exits 0** (every line `OK`).
Do not start the simulator, the teleop or the GUI as part of the install unless the human asks.

## Rules for the agent

1. **Work from the repository root** (the folder that contains this file). All paths below are relative to it.
2. **Never run `sudo` yourself.** Every step marked **[HUMAN · sudo]** must be shown to the human as one
   copy-paste block, and you wait until they say it finished. Then re-run that step's check yourself.
3. **Run the check after every step** and continue only when it passes. If a check fails, read the error, fix
   the cause, and re-run the step. The `setup/*.sh` scripts are idempotent; re-running them is safe.
4. **Ask before large downloads** (Isaac Sim is about 9 GB) if the human has not already said to go ahead.
   Say the file name, source URL and size.
5. **Do not** install Gazebo (`ros-humble-gz-*`), change `ros_env.sh`, edit `igus_ws/src/*` by hand,
   or `source /opt/ros/humble/setup.bash` in your shell profile. Isaac Sim must run without system ROS sourced.
6. The CloudXR licence (step 7) must be **accepted by the human**. Show them the command; do not accept it
   on their behalf.
7. Long-running commands (downloads, `colcon build`, `uv pip install` of torch) can take 5–20 minutes.
   Use generous timeouts or run them in the background and poll their log.

## Step 0 — inspect the machine (no changes)

```bash
lsb_release -ds
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
df -h ~ | tail -1
ls /opt/ros 2>/dev/null; command -v uv; ls -d ~/isaac-sim-6.1 2>/dev/null; echo "ISAAC_SIM=${ISAAC_SIM:-unset}"
./setup/check_env.sh || true
```

Requirements: **Ubuntu 22.04**, an NVIDIA RTX GPU with driver **≥ 580**, **≥ 40 GB** free in `$HOME`.
If the OS is not 22.04 or there is no NVIDIA driver, stop and tell the human; this setup is not supported there.
Use the output of `check_env.sh` to skip steps that are already `OK`.

## Step 1 — ROS 2 Humble and packages  **[HUMAN · sudo]**

Skip if `test -f /opt/ros/humble/setup.bash` and the check below already pass.
If `/opt/ros/humble` does not exist, the ROS 2 apt repository must be added first
(https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Give the human this block:

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop ros-humble-xacro \
  python3-colcon-common-extensions python3-rosdep python3-tk \
  ros-humble-moveit ros-humble-moveit-servo \
  ros-humble-moveit-task-constructor-capabilities ros-humble-moveit-task-constructor-visualization
sudo rosdep init || true
sudo apt install --only-upgrade -y 'ros-humble-*'
```

Then, without sudo: `rosdep update`.

**Check:**
```bash
bash -c 'source /opt/ros/humble/setup.bash && ros2 pkg prefix moveit_ros_move_group && command -v xacro colcon rosdep' && python3 -c "import tkinter"
```

The `--only-upgrade 'ros-humble-*'` line matters: mixed MoveIt releases (e.g. 2.5.9 core with 2.5.10 plugins)
make RViz fail to load the MTC panel.

## Step 2 — uv

```bash
command -v uv || curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
```

**Check:** `uv --version`

## Step 3 — Isaac Sim 6.1 (≈9 GB download, ≈20 GB unzipped)

Skip if `~/isaac-sim-6.1/python.sh` exists (or `$ISAAC_SIM/python.sh` with `ISAAC_SIM` set).

```bash
mkdir -p ~/Downloads ~/isaac-sim-6.1
curl -L -C - --retry 5 -o ~/Downloads/isaac-sim-6.1.0.zip \
  https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.1.0-linux-x86_64.zip
unzip -q ~/Downloads/isaac-sim-6.1.0.zip -d ~/isaac-sim-6.1
```

If the human wants it elsewhere, unzip there and add `export ISAAC_SIM=/that/path` to `~/.bashrc`;
every launcher reads `ISAAC_SIM`. Ask before deleting the zip.

**Check:** `head -c 5 ~/isaac-sim-6.1/VERSION` prints `6.1.0`, and
`test -d ~/isaac-sim-6.1/exts/isaacsim.ros2.core/humble/rclpy`.

## Step 4 — ROS workspace (`igus_ws`)

Clones `fani8j/Igus_rebel` (branch `hcut-path-consistency`, pinned commit) into `igus_ws/src`, applies
`igus_ws/patches/*.patch`, runs `rosdep install` and `colcon build`. About 5 minutes.

```bash
./setup/install_ros_ws.sh 2>&1 | tee setup/install_ros_ws.log
```

`rosdep install` may need sudo for missing apt packages. If its output lists packages it could not install,
give the human `sudo apt install -y <those packages>` (skip any `gz` / Gazebo / orbbec package), then re-run
the script. Compiler warnings in `igus_rebel_moveit_config` are normal.

**Check:** the log ends with `[ws] done`, and
```bash
test -f igus_ws/install/igus_rebel_description/share/igus_rebel_description/urdf/igus_rebel_xeg32_wrapper.urdf.xacro
grep -q "open_loop_control: true" igus_ws/src/Igus_rebel/src/igus_rebel/config/controller_mock.yaml
```

## Step 5 — Quest teleop environment (`.venv_quest`)

Python 3.10 venv with `--system-site-packages` (so it can import `/opt/ros/humble`'s rclpy) plus
`isaacteleop[cloudxr]==1.3.131`. It does not accept the CloudXR licence (that is step 7).

```bash
./setup/install_quest_env.sh
```

**Check:**
```bash
bash -c 'source /opt/ros/humble/setup.bash && .venv_quest/bin/python -c "import isaacteleop, rclpy, scipy, numpy; assert numpy.__version__.startswith(\"1.\")"'
```

Do not add `placo` or `pinocchio` to this venv: with `/opt/ros` sourced it aborts the process.

## Step 6 — recorder environment (`.venv_record`)

Ask the human whether they already have a **LeRobot 0.5.x** Python environment they train with.

- **Yes:** do not create a new one. Add `export LEROBOT_PY=/path/to/that/venv/bin/python` to `~/.bashrc`.
- **No:** create the repo-local one (downloads torch, several GB):
  ```bash
  ./setup/install_record_env.sh
  ```

**Check:** the script prints `[record] lerobot + av OK`, or
`"$LEROBOT_PY" -c "import importlib.metadata as m; print(m.version('lerobot'))"` prints `0.5.x`.

The recorder needs Python **3.12** (it imports the rclpy bundled with Isaac Sim 6.1, which is built for 3.12).

## Step 7 — CloudXR licence  **[HUMAN]**

Skip if `~/.cloudxr` already exists. Otherwise ask the human to run, read and accept:

```bash
.venv_quest/bin/python -m isaacteleop.cloudxr --accept-eula
```

## Step 8 — network  **[HUMAN · sudo, only if the firewall is on]**

```bash
sudo ufw status
```

If it is `active`, give the human:
```bash
sudo ufw allow 47998/udp && sudo ufw allow 48322/tcp && sudo ufw allow 49100/tcp
```

No change is needed for ROS: every launcher sources `ros_env.sh` (`ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`).
Tell the human that any manual `ros2 ...` command needs `source ./ros_env.sh` first.

## Step 9 — final check

```bash
./setup/check_env.sh
```

Done when it exits 0 and prints `All good`. Report to the human:
- what you installed and which steps you skipped because they were already done,
- the paths used for Isaac Sim and the LeRobot Python,
- how to start: `./launcher/run_gui.sh` (from the repository root),
- that the Quest headset setup is manual: manual page **Set up the Quest**
  (`docs/operator_manual.html#quest`).

## Optional

- **Legacy single-arm demos** (`igus_rebel_sim/`): `./setup/install_ros_ws.sh --with-irc` also builds
  CommonplaceRobotics `iRC_ROS`. Not needed for the dual-arm teleop.
- **Smoke test** (only if the human asks; opens no window, takes about a minute):
  ```bash
  ./dual_arm_sim/run_dual_arm_sim.sh --headless > /tmp/rr_sim.log 2>&1 &
  timeout 300 bash -c 'until grep -q "\[dual_arm\] running" /tmp/rr_sim.log; do sleep 2; done' && echo SIM_OK
  kill -INT %1
  ```
  The first start compiles shaders and can take several minutes.

## Troubleshooting for agents

| Symptom | Cause / fix |
| --- | --- |
| `check_env.sh` FAIL on `.venv_quest` with `ModuleNotFoundError: rclpy` | ROS 2 not installed, or the venv was created without `--system-site-packages`. Delete `.venv_quest` and redo step 5. |
| `colcon build` fails on a missing package | rosdep could not install it without sudo. Give the human the `sudo apt install` line for it, then re-run step 4. |
| `colcon` errors mention `gz_ros2_control` | Gazebo is not used; the script skips that key. Make sure you build only with `./setup/install_ros_ws.sh`. |
| Isaac Sim python crashes importing ROS | `/opt/ros/humble` is sourced in that shell. Use a fresh shell; `run_dual_arm_sim.sh` sets everything it needs. |
| `uv` not found right after install | `export PATH="$HOME/.local/bin:$PATH"` in the current shell. |
| LeRobot tries to reach huggingface.co | Expected only without `HF_HUB_OFFLINE=1`; the repo's launchers set it. Datasets are local. |
