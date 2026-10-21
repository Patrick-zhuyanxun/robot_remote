# AGENT_INSTALL.md — install robot_remote on a new workstation

This file is written for a coding agent (Claude Code, Codex, Cursor, ...). A person can hand it over with:

> Install robot_remote on this machine by following AGENT_INSTALL.md in https://github.com/Patrick-zhuyanxun/robot_remote

The human-oriented version is the **Install** page of `docs/operator_manual.html`.

## Goal and definition of done

Install what the human chose, then `./setup/check_env.sh` exits 0 (every selected item `OK`; unselected items show `skip`).
Do not start the simulator, the teleop or the GUI as part of the install unless the human asks.

## Rules for the agent

1. **Work from the repository root.** All paths are relative to it.
2. **Ask first, install second.** Do steps 0 and 1 before running anything that downloads or changes the machine.
   Ask in plain text in the chat (a question tool with the listed options if you have one). Do not guess the answers.
3. **Never run `sudo` yourself.** `./setup/install.sh` never runs it either: when apt packages are missing it prints
   the commands and exits with code 3. Show that block to the human as one copy-paste block, wait until they say it
   finished, then re-run the same `install.sh` command.
4. **The CloudXR licence is accepted by the human.** Do not pass `--accept-cloudxr-eula` unless the human has read it
   and told you to; otherwise show them the command `install.sh` prints.
5. Do not install Gazebo (`ros-humble-gz-*`), change `ros_env.sh`, edit `igus_ws/src/*` by hand, or
   `source /opt/ros/humble/setup.bash` in a shell profile. Isaac Sim must run without system ROS sourced.
6. Long steps (Isaac Sim download, `colcon build`, torch install) take 5–20 minutes: use a long timeout or run in
   the background and poll `setup/install_ros_ws.log`.

## Step 0 — look at the machine first (no changes, no questions)

```bash
./setup/install.sh --detect
```

It lists, with versions: Ubuntu, NVIDIA driver, free disk, ROS 2 Humble, uv, every Isaac Sim it finds
(`$ISAAC_SIM`, `~/isaac-sim-6.1`, `~/isaac-sim`), every LeRobot Python (`$LEROBOT_PY`, `.venv_record`,
`~/Robotic_infra/lerobot/.venv`), `.venv_quest` (isaacteleop), `igus_ws`, the CloudXR licence. Each Isaac Sim /
LeRobot line says `usable` or what is wrong: this project needs **Isaac Sim 6.1.x** and **LeRobot 0.5.x**; an older
Isaac Sim (e.g. 5.1) is not usable and must not be reused. Also run `./setup/check_env.sh || true`.

Requirements: **Ubuntu 22.04**, NVIDIA driver **≥ 580**, **≥ 40 GB** free in `$HOME` (about 15 GB without Isaac Sim).
If the OS is not 22.04 or there is no NVIDIA driver, stop and tell the human.

## Step 1 — tell the human what you found, then ask only what is still open

Summarise the Step 0 result in a few lines (what is already installed and its version, what is missing or
unusable). Do not ask about anything that is already `usable`: say you will reuse it and let them object.
Always installed when missing (do not ask): ROS 2 Humble workspace `igus_ws`, the Quest teleop environment
`.venv_quest`, the CloudXR runtime. Ask these, one at a time, in plain text:

1. **Isaac Sim 6.1** — only if no usable 6.1.x was found, or the human may want a different one. About 9 GB
   download, 20 GB on disk. It runs the simulation **and** it is needed to record datasets, real robot included
   (the recorder borrows Isaac's bundled rclpy). Options: download it / it is already in another folder (ask for the
   path) / skip (real-arm teleop only, no simulation, no recording).
2. **Recorder environment** (LeRobot 0.5.x; writes, replays and shows datasets) — only if no usable one was found.
   Options: create a new `.venv_record` (several GB) / use the environment they train with (ask for its `python`
   path) / skip. If more than one usable environment was found, ask which one.
3. **Legacy single-arm iRC_ROS packages** (only the demos in `igus_rebel_sim/`)? Default no.

Also tell them: the CloudXR licence has to be accepted by them, and a `sudo` block may follow.

## Step 2 — run the installer with their answers

Show the plan first (changes nothing):

```bash
./setup/install.sh --dry-run --isaac <download|skip|/path> --record <new|skip|/path/to/python> <--with-irc|--no-irc>
```

Then run the same command without `--dry-run`. Examples:

```bash
./setup/install.sh --isaac download --record new --no-irc           # simulation + recording
./setup/install.sh --isaac skip --record skip --no-irc              # real-arm teleop only
./setup/install.sh --isaac ~/isaac-sim-6.1 --record ~/lerobot/.venv/bin/python --no-irc
```

- **Exit 3**: apt packages are missing. Give the human the printed block, wait, re-run.
- **Exit 2**: a question had no answer or an option is wrong; fix the option.
- Re-running is safe: finished parts are detected and skipped. It writes your choices to
  `config/install_profile.env` (gitignored), which `check_env.sh` follows.
- A non-default Isaac Sim or LeRobot path needs an `export ISAAC_SIM=...` / `export LEROBOT_PY=...` line in
  `~/.bashrc` (the launchers read them); the installer prints it, or appends it with `--write-bashrc` if the human agrees.
- `rosdep` inside `install_ros_ws.sh` may ask for apt packages it cannot install without sudo: give the human
  `sudo apt install -y <those packages>` (skip any `gz` / Gazebo / orbbec package), then re-run.

## Step 3 — licence and firewall (human)

- CloudXR licence: if `~/.cloudxr` does not exist, ask the human to run, read and accept:
  `.venv_quest/bin/python -m isaacteleop.cloudxr --accept-eula`
- If `sudo ufw status` says `active`, give them:
  `sudo ufw allow 47998/udp && sudo ufw allow 48322/tcp && sudo ufw allow 49100/tcp`

Any manual `ros2 ...` command needs `source ./ros_env.sh` first (`ROS_DOMAIN_ID=42`, localhost only).

## Step 4 — final check and report

```bash
./setup/check_env.sh
```

Done when it exits 0 and prints `All good`. Report to the human: what you installed, what you skipped (and
why), the Isaac Sim / LeRobot paths, and how to start — `./launcher/run_gui.sh`. The Quest headset setup is
manual (manual page **Set up the Quest**, `docs/operator_manual.html#quest`); the physical arms and cameras are
described on the **Real robot** page.

## Optional

- **Smoke test** (only if asked; Isaac Sim installed; opens no window, about a minute):
  ```bash
  ./dual_arm_sim/run_dual_arm_sim.sh --headless > /tmp/rr_sim.log 2>&1 &
  timeout 300 bash -c 'until grep -q "\[dual_arm\] running" /tmp/rr_sim.log; do sleep 2; done' && echo SIM_OK
  kill -INT %1
  ```
  The first start compiles shaders and can take several minutes.
- **Real-robot cameras** (after install, with the cameras plugged in): `./recording/run_cameras.sh --list`.

## Troubleshooting for agents

| Symptom | Cause / fix |
| --- | --- |
| `check_env.sh` FAIL on `.venv_quest` with `ModuleNotFoundError: rclpy` | ROS 2 not installed, or the venv was created without `--system-site-packages`. Delete `.venv_quest` and re-run `install.sh`. |
| `colcon build` fails on a missing package | rosdep could not install it without sudo. Give the human the `sudo apt install` line, then re-run. |
| `colcon` errors mention `gz_ros2_control` | Gazebo is not used; the script skips that key. Build only with `install.sh` / `install_ros_ws.sh`. |
| Isaac Sim python crashes importing ROS | `/opt/ros/humble` is sourced in that shell. Use a fresh shell; `run_dual_arm_sim.sh` sets everything it needs. |
| `uv` not found right after install | `export PATH="$HOME/.local/bin:$PATH"` in the current shell. |
| Recorder says rclpy / Isaac not found | Isaac Sim was skipped. Re-run `install.sh --isaac download` (or a path). |
| LeRobot tries to reach huggingface.co | Expected only without `HF_HUB_OFFLINE=1`; the repo's launchers set it. Datasets are local. |

The individual scripts (`install_ros_ws.sh`, `install_quest_env.sh`, `install_record_env.sh`) still work on their own.
