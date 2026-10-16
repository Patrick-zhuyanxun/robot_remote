# robot_remote — Quest teleop & data recording for the igus ReBeL dual arm

Teleoperate the dual-arm igus ReBeL rig in **Isaac Sim 6.1** with a **Meta Quest 3/3S** and record
**LeRobot v3.0** datasets for VLA training. Two physical ReBeL arms can also be connected directly
over wired Ethernet (CRI + Position Interface); physical grippers and real-episode recording are not implemented.

**Codex / Claude shared context:** read [`AGENTS.md`](AGENTS.md) first (`CLAUDE.md` links to the same file).

**Manual:** open [`docs/operator_manual.html`](docs/operator_manual.html) in a browser (or press **? Manual** in the launcher).
It has one short page per task: Install, Set up the Quest, Run a session, Controls, Record & check, Data flow,
Troubleshooting, Real robot.

## Install (new machine)

**With a coding agent:** tell it *"Install robot_remote on this machine by following [AGENT_INSTALL.md](AGENT_INSTALL.md)"*.
It runs the steps below, checks each one, and asks you to run the `sudo` parts.

Ubuntu 22.04, NVIDIA RTX GPU (driver 580+), ROS 2 Humble, about 40 GB free. Details for each step are in the manual's Install page.

```bash
# 1. ROS 2 Humble (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) + extras
sudo apt install -y ros-humble-desktop ros-humble-xacro python3-colcon-common-extensions python3-rosdep python3-tk \
  ros-humble-moveit ros-humble-moveit-servo \
  ros-humble-moveit-task-constructor-capabilities ros-humble-moveit-task-constructor-visualization
# 2. uv
curl -LsSf https://astral.sh/uv/install.sh | sh
# 3. this repo
git clone https://github.com/Patrick-zhuyanxun/robot_remote.git ~/robot_remote && cd ~/robot_remote   # all commands below run from here
# 4. Isaac Sim 6.1 -> ~/isaac-sim-6.1 (or export ISAAC_SIM=...)
curl -L -o ~/Downloads/isaac-sim-6.1.0.zip https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.1.0-linux-x86_64.zip
mkdir -p ~/isaac-sim-6.1 && unzip -q ~/Downloads/isaac-sim-6.1.0.zip -d ~/isaac-sim-6.1
# 5-7. ROS workspace (clone + patch + build), Quest env (+ CloudXR EULA), recorder env
./setup/install_ros_ws.sh
./setup/install_quest_env.sh
.venv_quest/bin/python -m isaacteleop.cloudxr --accept-eula   # read and accept once per machine
./setup/install_record_env.sh      # or: export LEROBOT_PY=<your lerobot 0.5.x python>
# 8. check
./setup/check_env.sh
```

## Quick start (GUI)

```bash
./launcher/run_gui.sh
```

1. Pick the arms, cameras, task text, dataset name and save location, then press **Start**.
   The GUI starts Isaac → Quest teleop → recorder in that order (about 1 minute).
2. On the Quest: open **Browser** → `https://nvidia.github.io/IsaacTeleop/client/v1.3.131/` →
   enter the **Server IP** shown in the GUI → tap the *accept cert* link and accept → back → **Connect**.
3. Hold **Grip** to move an arm, **Trigger** closes its gripper, **A/X** starts / saves an episode,
   **B/Y** discards it. **Stop all** in the GUI saves any open episode and shuts everything down.

New datasets go to `robot_remote/datasets/` by default (the GUI and `ROBOT_REMOTE_DATASETS` can override this).
Settings live in `config/launcher.json`, physical endpoints in `config/real_arms.json`, and logs in `logs/`.
Old launcher settings are copied into the project on first use; the former default `~/datasets` is replaced
with the project default for future sessions. Existing datasets are left in place; custom dataset paths are preserved.

## Quest controls

| Input | Effect |
| --- | --- |
| Grip (middle finger), hold | The arm follows the hand. Release = freeze. Re-grip anywhere: no jump. |
| Move the hand | Tool translates along the arm-base axes (forward / left / up). |
| Twist the controller handle | Tool spins about its own Z (flange) axis. |
| Nod / tilt the controller | Tool tilts about its own X / Y axes. |
| Trigger (index finger) | Gripper closes proportionally (0 = open, 1 = closed). |
| A or X | Start recording → press again to save the episode. |
| B or Y | Discard the current episode. |

Left controller drives the left arm, right controller the right arm. "Facing the robot" in the GUI
mirrors the mapping (`--yaw-deg 180 --swap-hands`).

## Data flow

```
Quest 3/3S ──Wi-Fi/WebRTC──► CloudXR runtime ─┐
                                               ▼
                          quest_dual_arm_teleop.py  (py3.10 + rclpy, IK per arm)
                            │ /dual_arm/isaac_joint_commands  (joint targets, gripper carriages)
                            │ /quest_teleop/action            (14-D commanded action, every tick)
                            │ /quest_teleop/events            ("toggle" / "discard" from A/X, B/Y)
                            ▼
              Isaac Sim 6.1 (run_dual_arm_sim.py, OmniGraph ROS 2 bridge)
                            │ /dual_arm/isaac_joint_states    (21 joints)
                            │ /dual_arm/{head,wrist_left,wrist_right}/rgb
                            │ /clock                          (sim time)
                            ▼
              record_dataset.py (.venv_record: lerobot 0.5.1, Isaac's py3.12 rclpy)
                            ▼
              robot_remote/datasets/<name>/   LeRobotDataset v3.0
```

All launchers source `ros_env.sh` (**ROS_DOMAIN_ID=42, ROS_LOCALHOST_ONLY=1**). Another machine on the
LAN runs the same rig on domain 0; without this our commands drive its arms and its `/clock` corrupts
recordings. Run `source ./ros_env.sh` before any manual `ros2 …` command.

## Dataset format

| Feature | Shape | Content |
| --- | --- | --- |
| `observation.state` | 14 float32 | `arm_left_joint1..6, arm_left_gripper, arm_right_joint1..6, arm_right_gripper` (rad; gripper closedness 0–1) |
| `action` | 14 float32 | same names/order, the teleop's commanded joints + trigger |
| `observation.images.head` / `wrist_left` / `wrist_right` | video H×W×3 | RGB, default 640×480 |
| `task` | string | the task text from the GUI |

Frames are sampled every 1/fps of **simulation time** (`/clock`), so timestamps match the simulated
motion even though Isaac runs below real time (≈0.5× with three cameras).

## Tools

| Script | Purpose |
| --- | --- |
| `launcher/run_gui.sh` | Settings window + process manager (recommended entry point). |
| `dual_arm_sim/run_dual_arm_sim.sh [--cameras head wrist_left wrist_right] [--headless]` | Isaac Sim rig + ROS 2 bridge. Do not source `/opt/ros` in this terminal. |
| `quest_teleop/run_quest_dual_arm.sh [--arms left right] [--yaw-deg 180 --swap-hands]` | Quest → both arms, direct to Isaac. |
| `recording/run_record.sh --name N --task "…"` | Recorder. Keyboard fallback: Enter = start/save, d = discard, q = quit. |
| `launcher/run_replay_gui.sh [dataset folder]` | Playback window: choose a dataset folder + episode, then **View** (LeRobot viewer / Rerun) or **Replay in Isaac**. |
| `recording/run_replay.sh --path <dataset folder> --episode K` | Replays recorded actions into Isaac and compares with recorded states (alignment check). |
| `quest_teleop/run_teleop_viz.sh` | RViz view of the teleop frames (arm bases, control point, hand-motion axes). |
| `dual_arm_sim/start_right_arm_moveit.sh` | MoveIt/RViz planning for the right arm (do not run together with Quest teleop). |

## Pre-test of a physical arm in RViz (igus ROS driver)

```bash
./real_robot/rviz_pretest.sh left     # or right; --yes skips the enable question; --mock without a robot
```

Starts the igus driver for that arm (`config/real_arms.json`), asks before Reset + Enable, then opens
MoveIt + RViz (ROS_DOMAIN_ID 91, localhost only). Use Velocity/Acceleration Scaling 0.05; start from straight up
with a joint move. Goal states `home` = (0, 0, 65, 0, 50, 0)° and `zero`. Closing RViz / Ctrl+C stops everything.
Do not run it together with the Quest teleop: one CRI client controls an arm.

## Physical dual arms over Ethernet (teleop only)

| Arm | Controller | Address | Communication |
| --- | --- | --- | --- |
| Left | Left Quest controller | `192.168.3.11` | CRI TCP 3920 + Position Interface (port queried from CRI) |
| Right | Right Quest controller | `192.168.3.12` | Independent CRI TCP 3920 + Position Interface |

Connect this PC and both robot controllers to the same wired subnet, using a switch or suitable wired
interfaces. This PC was observed at `192.168.3.54`; avoid duplicate IPs. Quest still connects to CloudXR over
Wi-Fi. The physical arm link uses TCP directly; a remote ROS driver, ROS discovery and command topics are
not needed. Local ROS Humble and the existing `igus_ws` description remain dependencies for IK and visualization.

Run from this project directory:

```bash
# 1. Read-only check, no ROS / Quest required. Saves logs/real_connection_check.json.
./quest_teleop/run_quest_real_arm.sh --check-only

# 2. Observe both robots and exercise Quest IK without changing robot configuration or sending targets.
./quest_teleop/run_quest_real_arm.sh --arms left right --dry-run

# 3. Operator at the e-stop, motors enabled and bent poses prepared using the robot's own controls:
./quest_teleop/run_quest_real_arm.sh --arms left right
```

Alternatively launch `./launcher/run_gui.sh`, select **Real igus ReBeL dual arms — wired Ethernet**, select
both arms, and check the two IP fields. **Dry run** is enabled on a fresh configuration. One arm can be
selected with `--arms left` or `--arms right`. CLI endpoints come from `config/real_arms.json`; override with
`--config <file>`, `--left-ip <ip>` or `--right-ip <ip>`. The GUI remembers explicit IP overrides locally.

Both selected robots must have healthy feedback, enabled motors, an active CRI connection and a stopped
robot program before either Position Interface is configured. With `--enable-motors` (launcher: *Enable motors
at start*, default on) an arm whose only problem is 'motor not enabled' gets Reset + Enable first; e-stop, a running
program or any other error is refused. The client never homes, references joints or takes control away from
another client. Close other control clients after
preparing the arms. `--real-home` is rejected on Ethernet. Start from the measured physical pose and hold
each controller's **GRIP** to move its arm; release to deselect that position source. Tracking loss requires
release and re-grip. Default limits: 0.35 rad/s per joint, 0.15 m/s TCP translation, 30°/s TCP rotation at 30 Hz.

**Return to the home pose on exit:** on Stop all / Ctrl+C the selected arms move on their own back to the home pose (0, 0, 65, 0, 50, 0)° (`--park-deg`)
at 10°/s (`--park-vel-deg`), arriving together, then the program closes. A second Stop all / Ctrl+C stops them where
they are. Skipped on a dry run, after a fault, or with `--no-park` (launcher checkbox). Stops early if the arms come
within `--min-arm-dist`. Keep the path to the home pose clear.

The transport sends six joint angles in degrees, converted from the IK's radians. It leaves external axes,
tool axes and digital outputs alone. Live sessions temporarily enable the Position Interface if needed
(this controller setting is persistent), connect its queried port, seed the measured pose, and select it
only on grip. On normal cleanup, the client requests restoration of an interface it enabled. After a hard
process kill or network loss, check the controller configuration; restoration cannot be guaranteed.

A joint error, stale feedback (0.5 s), following error (15°), command gap (0.25 s), invalid target, rejected
command or TCP disconnect closes both selected transports. The watchdog runs independently of the Quest
control loop. Restart after faults; no automatic motion recovery. Socket close stops Position Interface use
as defined by the [manufacturer's specification](https://wiki.cpr-robots.com/images/7/70/CPR_PositionInterface_V1.pdf).
These software checks are not a safety controller; keep a person at the hardware e-stop.

The existing point-distance guard uses the simulated rig's base/tool geometry. Physical base poses, TCPs,
joint zero conventions and controller directions require on-site validation before meaningful motion.
It is not a full stand/table/self-collision checker. Physical gripper control, cameras and real recordings
remain unsupported.

**Verified 2026-10-15:** both controllers answered read-only CRI queries, reporting RobotControl
`V980-15-000-2` and Position Interface port 3940. Left: no joint errors; right: `_MNE` / `KINSTATE 99`
(motors disabled). No hardware movement, motor enable or Position Interface configuration was performed.
Socket-level mock tests cover streaming and fault handling; actual Quest-to-hardware motion remains unverified.
Current evidence and handoff are in `AGENTS.md`; rerun `--check-only` for current robot state.

The older one-arm ROS backend remains available through `ROBOT_REMOTE_TARGET=real` + `ros_env.sh`,
then `quest_dual_arm_teleop.py --backend real-ros --arms left` in the sourced Quest environment.
Do not run it alongside Ethernet control of that arm.

```bash
# Tests use two local fake controllers; they do not contact the real robots.
python3 -m unittest discover -s tests -v
```

## Known limitations

- The wrist-camera images are half black: the gripper/camera-mount geometry in the upstream USD occludes the lens.
- The inter-arm guard (12 cm) only checks arm vs arm (wrist, flange, jaw, knife points), not the stand.
- Wrist pitch is limited to about ±40° from home by joint 5.
- Not yet verified with a real headset end to end: gripper channel in a recorded episode, the left controller,
  and the A/B/X/Y buttons driving the launcher.

## Third-party material

See [`THIRD_PARTY.md`](THIRD_PARTY.md).
