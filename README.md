# robot_remote — Quest teleop & data recording for the igus ReBeL dual arm

Teleoperate the dual-arm igus ReBeL rig in **Isaac Sim 6.1** with a **Meta Quest 3/3S** and record
**LeRobot v3.0** datasets for VLA training. A real-robot backend is reserved but not implemented yet.

**Manual:** open [`docs/operator_manual.html`](docs/operator_manual.html) in a browser (or press **? Manual** in the launcher).
It has one short page per task: Install, Set up the Quest, Run a session, Controls, Record & check, Data flow,
Troubleshooting, Real robot (reserved).

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

Datasets go to `~/datasets` by default (choose another folder in the GUI or set `ROBOT_REMOTE_DATASETS`).
Settings are remembered in `~/.config/robot_remote_launcher.json`; process logs go to `robot_remote/logs/`.

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
              ~/datasets/<name>/   LeRobotDataset v3.0
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

## Real robot (reserved)

Not implemented. To be filled in when the hardware backend exists:
hardware driver (`igus_rebel` CRI, `hardware_protocol:=rebel`), network/IP per arm, safety limits and
e-stop procedure, calibration of the two arm bases, mapping of the same 14-D state/action to the real
joints and gripper, and the camera drivers that replace the Isaac image topics.

## Known limitations

- The wrist-camera images are half black: the gripper/camera-mount geometry in the upstream USD occludes the lens.
- The inter-arm guard (12 cm) only checks arm vs arm (wrist, flange, jaw, knife points), not the stand.
- Wrist pitch is limited to about ±40° from home by joint 5.
- Not yet verified with a real headset end to end: gripper channel in a recorded episode, the left controller,
  and the A/B/X/Y buttons driving the launcher.

## Third-party material

See [`THIRD_PARTY.md`](THIRD_PARTY.md).
