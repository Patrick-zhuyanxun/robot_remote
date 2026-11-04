# robot_remote — Quest teleop & data recording for the igus ReBeL dual arm

Teleoperate the dual-arm igus ReBeL rig in **Isaac Sim 6.1** with a **Meta Quest 3/3S** and record
**LeRobot v3.0** datasets for VLA training. Two physical ReBeL arms can also be connected directly
over wired Ethernet (igus ROS driver) and recorded with four USB cameras (see *Real-robot recording*); physical
grippers are not implemented.

**Codex / Claude shared context:** read [`AGENTS.md`](AGENTS.md) first (`CLAUDE.md` links to the same file).

**Manual:** open [`docs/operator_manual.html`](docs/operator_manual.html) in a browser (or press **? Manual** in the launcher).
It has one short page per task: Install, Set up the Quest, Run a session, Controls, Record & check, Data flow,
Troubleshooting, Real robot.

## Install (new machine)

Ubuntu 22.04, NVIDIA driver 580+, about 40 GB free (about 15 GB without Isaac Sim).

```bash
git clone https://github.com/Patrick-zhuyanxun/robot_remote.git ~/robot_remote && cd ~/robot_remote
./setup/install.sh
```

The installer first **lists what is already on the machine with versions** (`./setup/install.sh --detect` shows only that), then **asks in plain text what to install**, shows the plan, then does it (it never runs `sudo` itself: if apt
packages are missing it prints the commands and stops, you run them and start it again; the workspace build's `rosdep` may still ask for your password for missing apt packages):

| Question | Why it matters |
| --- | --- |
| Isaac Sim 6.1: download (about 9 GB) / already in another folder / skip | Runs the simulation **and** is needed to record datasets, real robot included (the recorder borrows its rclpy). Skip only for teleop of the real arms without recording. |
| Recorder: new `.venv_record` / my own LeRobot 0.5.x python / skip | Writes, replays and shows datasets. |
| Legacy single-arm iRC_ROS packages | Only for the demos in `igus_rebel_sim/`. Default no. |

Always installed: ROS 2 workspace `igus_ws`, Quest teleop environment `.venv_quest`, CloudXR runtime (its licence is
accepted by you: the installer prints the command). Options for scripts and agents: `./setup/install.sh --help`
(for example `--isaac skip --record skip --no-irc`, `--dry-run` to see the plan only). The choices are saved in
`config/install_profile.env`; `./setup/check_env.sh` follows them.

**With a coding agent:** tell it *"Install robot_remote on this machine by following [AGENT_INSTALL.md](AGENT_INSTALL.md)"*.
It first runs `--detect` and reports what is installed (and which versions are unusable), asks only what is still open, runs the installer with your answers and asks you to run the `sudo` parts.
Manual step-by-step commands are on the Install page of the manual.

## Quick start (GUI)

```bash
./launcher/run_gui.sh
```

1. Pick the arms, cameras, task text, dataset name and save location, then press **Start**.
   The GUI starts Isaac → Quest teleop → recorder in that order (about 1 minute).
2. On the Quest: open **Browser** → `https://nvidia.github.io/IsaacTeleop/client/v1.3.131/` →
   enter the **Server IP** shown in the GUI → tap the *accept cert* link and accept → back → **Connect**.
3. Hold **Grip** to move an arm, **Trigger** closes its gripper, **A** starts / saves an episode, **X** switches the rotation frame,
   **B** discards it, **Y** resets the scene (sim: arms home, box flaps closed; an open episode is discarded). **Stop all** in the GUI saves any open episode and shuts everything down.
   The run page's **Rotation: TOOL / BASE frame** button switches how hand rotation is applied
   (tool = about the tool's own axes, base = about the fixed arm-base axes; translation always follows the base axes).
   It takes effect when an arm is next gripped (`--rot-frame`, or topic `/quest_teleop/set_rot_frame` "tool"/"base").

New datasets go to `robot_remote/dataset/` by default (the GUI and `ROBOT_REMOTE_DATASETS` can override this).
Under the save location the launcher shows what Start will do with `<location>/<name>` and blocks what cannot work:
new name or empty folder → new dataset; an existing dataset → episodes are appended only if it has the same
source (sim / real), fps, cameras and image size; a dataset that never saved an episode → re-created; an incomplete
dataset (recorder killed) or a folder that is not a dataset → refused, nothing is written or deleted. **Browse...** on a
dataset folder itself selects it for continuing (save location = its parent, name = its folder).
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
| Thumbstick, while gripping | Slow tool rotation without twisting the wrist: left/right spins the tool about its own Z axis (`--stick-x yaw` = about the base vertical), up/down tilts the tip up/down. 15 deg/s at full deflection (`--stick-rot-vel`, 0 = off; dead zone `--stick-deadzone 0.15`, `--stick-signs` flips an axis). `--rot-input hand+stick|stick|hand` (launcher: *Rotation input*) chooses what rotates the tool; the stick is held while the wrist cannot follow (joint limit). IK backends only (not `cri-jog`). |
| Trigger (index finger) | Gripper closes proportionally (0 = open, 1 = closed). |
| A (right) | Start recording → press again to save the episode. |
| X (left) | Switch the rotation frame TOOL ↔ BASE (takes effect at the next grip; same as the run page button). |
| B | Discard the current episode. |
| Y (left) | Reset: an open episode is discarded; sim: arms home, grippers open, box flaps closed; real arms: **hold Y for 1 s** (`--y-hold-s`, so a brushed thumb does nothing), then both move home at 10°/s (any button, a grip or Stop all stops them). The launcher's reset button is immediate. |

Left controller drives the left arm, right controller the right arm. "Facing the robot" in the GUI
mirrors the mapping (`--yaw-deg 180 --swap-hands`).

## Data flow

```
Quest 3/3S ──Wi-Fi/WebRTC──► CloudXR runtime ─┐
                                               ▼
                          quest_dual_arm_teleop.py  (py3.10 + rclpy, IK per arm)
                            │ /dual_arm/isaac_joint_commands  (joint targets, gripper carriages)
                            │ /quest_teleop/action            (14-D commanded action, every tick)
                            │ /quest_teleop/events            ("toggle" / "discard" / "reset" from A, B, Y)
                            ▼
              Isaac Sim 6.1 (run_dual_arm_sim.py, OmniGraph ROS 2 bridge)
                            │ /dual_arm/isaac_joint_states    (21 joints)
                            │ /dual_arm/{head,wrist_left,wrist_right}/rgb
                            │ /clock                          (sim time)
                            ▼
              record_dataset.py (.venv_record: lerobot 0.5.1, Isaac's py3.12 rclpy)
                            ▼
              robot_remote/dataset/<name>/    LeRobotDataset v3.0
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

## Real-robot recording

**Video:** cameras are encoded to MP4 while recording (LeRobot `streaming_encoding`, `--no-streaming-encode` for the old PNG-then-encode path), so saving takes seconds; keyframe every `--video-gop` 30 frames (LeRobot default 2 = about 9x larger files, faster random access in training; `--video-gop 0` restores it). If the encoder falls behind and drops frames the episode is discarded (it would be out of sync).

**PC without Isaac Sim 6.1:** `run_record.sh` then runs the recorder on system `python3` + `/opt/ros/humble` (override the python with `LEROBOT_PY`, force it with `ROBOT_REMOTE_RECORD_PY=system`). With LeRobot 0.4.x an existing dataset cannot be continued: use a **new dataset name for every session** (the launcher says so under the save location and blocks Start).

In the launcher pick **Real igus ReBeL arms**, tick **Record** and the real cameras, then Start: drivers → teleop →
recorder (`run_record.sh --source real`) → RViz. Terminal equivalent, after the drivers and
`run_quest_real_arm.sh` are running: `recording/run_record.sh --source real --name N --task "…" [--cameras …]`.

| Feature | Real-robot content |
| --- | --- |
| `observation.state` | arm joints from `/{left,right}/joint_states` (igus drivers); a one-arm session takes the other arm's assumed pose from `/quest_teleop/joint_states`. Gripper slots = 0 (no physical gripper). |
| `action` | `/quest_teleop/action` (commanded joints; gripper slots = Quest trigger, operator intent only) |
| `observation.images.wrist_left` / `wrist_right` / `head` / `knife` | USB cameras of `config/real_cameras.json`, 640×480 RGB, read in the recorder process (`recording/real_cameras.py`) |
| robot_type | `igus_rebel_dual_arm_real` (a dataset is never mixed with sim episodes or other cameras: the recorder refuses) |

Sampling clock = wall time. Holding Y for 1 s (`--y-hold-s`; a short press does nothing) discards the open episode and moves both real arms home at 10°/s (any Quest button or grip, or Stop all, stops them; dry run: no motion); B only discards.
Stop all saves an open episode **before** the arms return to 0°.

Cameras (`recording/run_cameras.sh --list | --snapshot | --test 10`): the Orbbecs are found by serial + colour
interface (any port); the two ARC wrist cameras have no serial and are found by **USB port** — keep them in their ports
or update `usb_port` in the config. **Cabling:** a USB 3 hub is two hubs in one (here Genesys 05e3:0626 on the
5 Gbit/s bus + 05e3:0610 on the 480 Mbit/s bus); USB 2 devices such as the ARC cameras always use the shared 480 Mbit/s
half. Working setup (2026-10-21, 30 s test: ARC ~47 decoded fps each, Orbbecs 30 fps): all four on one USB 3 hub on a
USB 3 PC port, `--list` showing both Orbbecs at 5000 Mbit/s. When the hub sat on a USB 2-only port all four shared
480 Mbit/s: the second ARC got "Not enough bandwidth for altsetting" and the Gemini 336 kept resetting (error -71).

## Record real-arm mocap episodes

Records the **real** right ReBeL arm while the OptiTrack mocap pipeline (`rebel_mocap_follower`) drives it, in the
same 14-value layout as the Isaac datasets (LeRobot v3.0, 30 fps, AV1 video). The recorder is **read-only**: it only
subscribes (no publishers, services or clients besides rclpy's automatic `/rosout` and `/parameter_events`). It runs
with system `python3` + `/opt/ros/humble` (not Isaac's rclpy) on the real ROS network (domain 12, Fast DDS, localhost
off; `recording/run_record_mocap.sh` sets this via `ROBOT_REMOTE_TARGET=real ros_env.sh`). Unlike `run_record.sh` it
does **not** need Isaac Sim. `--source` values of `record_dataset.py`: `sim` (alias `isaac`, the default), `real` (USB
cameras read in-process, Quest teleop action), `mocap-real`, `moveit-real` (both ROS-topic only). Datasets go to
`robot_remote/dataset/<name>/` (`--root` / `ROBOT_REMOTE_DATASETS`), a new LeRobot dataset folder per `--name`.

```bash
recording/run_record_mocap.sh --list-cameras        # image topics: resolution, encoding, measured Hz
recording/run_record_mocap.sh --name right_pick --task "pick up the box" --camera-preset cell-right
recording/run_record_mocap.sh --name right_pick --task "pick up the box" --pick-cameras   # asks a key per topic
recording/run_record_mocap.sh --name right_pick --task "pick up the box" \
    --camera head=/camera/color/image_raw --camera wrist_right_gripper=/arm_right/gripper_camera/image_raw/compressed
```

| Flag | Default | Meaning |
| --- | --- | --- |
| `--arms` | `right` | driven sides; the other arm is constant |
| `--state-topic` / `--state-names` | `/joint_states` / `joint{i}` | arm state (dual stack: `arm_{side}_joint{i}`) |
| `--action-topic` / `--action-names` | `/dual_arm/isaac_joint_commands` / `arm_{side}_joint{i}` | **raw mocap target** (not the follower-filtered command); extra names (e.g. grippers) are ignored |
| `--follower-status-topic` | `/mocap_follower/status` | JSON status; `{side}` is replaced (dual launch: `/mocap_follower_{side}/status`) |
| `--other-arm-deg` | `0 0 0 0 0 0` | constant pose of the undriven arm (state **and** action) |
| `--camera KEY=TOPIC` | none | repeatable, any number of cameras, becomes `observation.images.KEY`. Recommended keys for the 5-camera cell: `head`, `wrist_left_gripper`, `wrist_left_knife`, `wrist_right_gripper`, `wrist_right_knife` |
| `--camera-preset` | none | `cell` = all 5 cameras of `ros2 launch rebel_cameras cameras.launch.py` (`~/igus_rebel_ros2`), `cell-right` / `cell-left` = `head` + that arm's gripper and knife cameras; `--camera` adds or overrides a key |
| `--list-cameras` / `--pick-cameras` / `--discover-s` | | topic discovery (3 s) |
| `--width/--height` | 640 / 480 | frames of another size are resized (logged once per camera) |

Preset topics (all JPEG): `head` = `/camera/color/image_raw/compressed` (Gemini 336, 1920x1080, 15 fps),
`wrist_{side}_knife` = `/arm_{side}/wrist_camera/color/image_raw/compressed` (Gemini 305, 1280x800, 15 fps),
`wrist_{side}_gripper` = `/arm_{side}/gripper_camera/image_raw/compressed` (USB camera, 1280x720, 30 fps). The Orbbecs'
raw `color/image_raw` (3-6 MB per frame) arrived at only 5-10 Hz on domain 11, so the preset uses `.../compressed`.
Each camera is decoded in its own thread (newest frame only), so the 30 fps sampling loop is not slowed down. All
frames are stretched to `--width/--height` (aspect ratio changes); a 15 fps camera repeats each image for two frames.
Recording waits until every chosen camera has sent a frame, so leave out cameras that are not running. Saving encodes
the videos one after another (AV1): a 20 s episode with 5 cameras took 30-125 s to save.

Cameras: `sensor_msgs/Image` (`rgb8 bgr8 rgba8 bgra8 mono8 yuv422_yuy2/yuyv uyvy`) and `CompressedImage` (jpeg/png).
An unsupported encoding is flagged `UNSUPPORTED` by `--list-cameras` and the episode is discarded with an error naming
the topic and encoding if it is recorded anyway. Grippers: state and action are 0.0 (open) on both arms (the mocap
path has no gripper). The mocap clutch is a foot pedal driving the follower deadman, already visible in `follower_state`.

Frames are sampled on the **wall clock** at `--fps` (no `/clock`); `--max-episode-s` is wall time. State/camera data
older than max(2/fps, 0.1 s) and action older than 0.2 s still produce a frame but are counted as stale in the log.
Extra feature `observation.follower_state` (float32, 2 = `[left, right]`):

| Code | Meaning |
| --- | --- |
| 0 | arm not driven, or no follower status within 0.5 s |
| 1 / 2 / 3 / 4 / 5 | `DISABLED` / `HOLD` / `CATCHUP` / `ENABLED` (following) / `REFUSED` |
| 6 | any other state string |

`robot_type` is `igus_rebel_dual_arm_real_mocap`. Keyboard: Enter = start/save, `d` = discard, `q` = quit. With
LeRobot 0.4.x (no `LeRobotDataset.resume`) an existing dataset folder is refused: use a new `--name`.

## Record real-arm MoveIt / scripted episodes

Same recorder, but for a real arm that a **MoveIt waypoint program on another PC** drives through its
`JointTrajectoryController` (only the controller stack runs on this PC). `--source moveit-real` is identical to the
mocap mode (14-value layout, cameras / `--camera-preset` / `--list-cameras` / `--pick-cameras`, wall-clock 30 fps,
Enter / `d` / `q`, grippers 0.0, undriven arm = `--other-arm-deg`, same domain-12 network setup) except for the action:

- **action** = the trajectory controller's **setpoint**, from `control_msgs/JointTrajectoryControllerState` on
  `--controller-state-topic` (default `/rebel_arm_trajectory_controller/controller_state`; `{side}` is replaced, one
  subscription per distinct topic). The `reference` positions are used; an older controller that leaves `reference`
  empty falls back to `desired`; a message with neither is ignored. Joint names come from `--action-names` (default
  `joint{i}`; with mocap it is `arm_{side}_joint{i}`), matched by name, so one topic can serve both arms.
- `observation.state` = `/joint_states` as before; **no** `observation.follower_state` feature;
  `robot_type` is `igus_rebel_dual_arm_real_moveit`. `--action-topic` and `--follower-status-topic` are mocap-only and
  are rejected with `moveit-real`; `--controller-state-topic` is rejected with the other sources.
- The recorder is **read-only** (subscriptions only, best-effort QoS so it also matches reliable publishers). The
  waypoint program has to send its trajectories to the controller that runs here; the recorder never commands anything.

```bash
recording/run_record_moveit.sh --list-cameras
recording/run_record_moveit.sh --name box_open_01 --task "open the box" --camera-preset cell-right
# dual-arm stack: one /joint_states with arm_*_joint names, one controller per side
recording/run_record_moveit.sh --name box_open_both --task "open the box" --arms left right --camera-preset cell \
    --state-names "arm_{side}_joint{i}" --action-names "arm_{side}_joint{i}" \
    --controller-state-topic "/arm_{side}_trajectory_controller/controller_state"
```

## Record cutting runs

Records the **cutting** runs of `~/igus_rebel_ros2` (carton tape cutting) with one episode per execute call. Same
read-only recorder as the MoveIt mode (`--source moveit-real`, 14-value layout, wall-clock sampling, system `python3` +
`/opt/ros/humble`): it only subscribes, it never publishes, calls services or sends trajectories; the cut is started from
`igus_rebel_ros2` as usual. Start the recorder first (it ignores an execution that is already running).

```bash
recording/run_record_cut.sh --list-cameras                         # what is publishing
recording/run_record_cut.sh --name cut_dual_001                    # dual cell: dual_cut_coordinator, both arms
recording/run_record_cut.sh --name cut_dual_001 --keep-stopped     # also keep stopped runs (task + " [stopped: <reason>]")
recording/run_record_cut.sh --name cut_single_001 --single-arm --arms right   # box_h_cut_executor T-cut sequence
```

| | dual cell (default) | `--single-arm` |
| --- | --- | --- |
| episode starts | the coordinator's first `WAITING_BARRIER` / `EXECUTING` after a non-executing status (`/dual_cut_coordinator/status`) | `t_cut_status`: `{step_index 0, "sequence", "running"}` (`execute_t_cut` dispatched) |
| episode ends, **saved** | status `PLANNED` after at least one phase report `completed=true` (`/dual_cut_coordinator/exec_report`, none `false`) | `{"sequence", "done"}` (all 4 steps) |
| episode ends, **discarded** (`--keep-stopped`: saved, task suffixed) | status `STOPPED` or `stop_latched` (`" [stopped: <stop_reason>]"`); back to `IDLE`/`PLANNING`, or `PLANNED` without any completed phase (barrier / goal failure) = `" [aborted: ...]"` | any `"failed"` step status (a step failed, was refused, or `abort_t_cut`) |
| state | `/joint_states` `arm_{left,right}_joint1..6` | `/joint_states` `joint1..6`, put in the slots of the side given by `--arms` (default right); the other arm = `--other-arm-deg` |
| action | `reference` of `/arm_{side}_trajectory_controller/controller_state` (`desired` as fallback) | `reference` of `/rebel_arm_trajectory_controller/controller_state` |
| `robot_type` | `igus_rebel_dual_arm_real_cut` | `igus_rebel_single_arm_real_cut` |

One episode spans all phases of one `~/execute` (A, barrier, B; or the whole sequence schedule). Extra feature
`observation.cut_progress` (float32, 2; **absent in manual mode**):

| | element 0 | element 1 |
| --- | --- | --- |
| dual-cut | coordinator state: 0 IDLE, 1 PLANNING, 2 PLANNED, 3 WAITING_BARRIER, 4 EXECUTING, 5 STOPPED | phase: 0 none, 1 `A`, 2 `B` |
| h-cut | step: 0 sequence start, 1 `approach_standoff`, 2 `side_cut_penetrate`, 3 `continue_side_cut`, 4 `continue_top_cut`, 5 sequence done | status: 0 none, 1 running, 2 done, 3 failed |

Keyboard Enter / `d` / `q` work in every mode as an override: Enter starts an episode by hand or saves the open one early
(a hand-started episode is only closed by hand), `d` discards, `q` quits and saves the open episode. The trigger options
are `--episode-trigger {manual,dual-cut,h-cut}` (default `manual` = the MoveIt mode), `--keep-stopped`, `--trigger-topic`,
`--exec-report-topic`, `--task` (default `cut the carton tape`), `--max-episode-s` (default 600 here; an auto episode that
reaches it is saved and the rest of that execution is not recorded). A silent status topic (> 2 s) while an episode is open
is only logged: the coordinator status is low-rate and the episode is ended by a message, never by a timeout.

### Automatic mode: raw rosbag2 per execution (`record_raw_cut.py`, started by the cutting launches)

The launches of `igus_rebel_ros2` (`dual_cut_system.launch.py`, `move_group.launch.py`; `cut_recorder_backend:=raw`, the default;
`video` = the old passive `tools/cut_recorder` mp4 recorder, also the automatic fallback when `~/robot_remote` or `rosbag2_py`
is missing) start `recording/record_raw_cut.py` next to the stack. It stores the **raw data at full resolution** (no LeRobot, no
numpy, no image decoding; plain system `python3` + `/opt/ros/humble`; converting to LeRobot is a later, separate step). Read-only:
serialized subscriptions only.

* **One rosbag2 (storage `sqlite3`; the mcap plugin is not installed) per execution**, written by the recorder itself
  (`rosbag2_py.SequentialWriter`) from the serialized messages: original bytes and type, bag timestamp = receive time (wall),
  the header stamps inside the messages are untouched. Cameras are the `.../compressed` `CompressedImage` topics as published
  (full-resolution JPEG, bit-identical), `/joint_states`, `/tf`, `/tf_static` (all static transforms written at bag open), both
  `/arm_{left,right}_trajectory_controller/controller_state` and the single-arm `/rebel_arm_trajectory_controller/controller_state`,
  `/dual_cut_coordinator/{status,exec_report,plan_report}` (the latest `plan_report` is written at bag open), `/box_h_cut_executor/t_cut_status`,
  the FJT action status topics, the `left_vision` / `right_vision` overlays and every other topic below `/side_cut_vision/` once it
  appears, `--extra-topic TOPIC=TYPE`. Only topics that exist are in a bag. With `--camera-domain N` the camera topics come from a second rclpy
  context in that ROS domain into the **same** bag (the overlays and everything else stay in the robot domain).
* **Episode** = the same triggers as above (`--trigger dual-cut | h-cut | fjt-status`, `--fjt-hold-s` 5 s for `fjt-status`) plus
  `--pre-roll-s` (default 2, a ring buffer of the messages before the trigger) and `--post-roll-s` (default 2). Outcome `finished` is
  kept; `stopped` / `aborted` are **kept too** and marked in `clip.json` (`--discard-stopped` deletes their bag); a SIGINT / SIGTERM (launch
  Ctrl-C) closes the open bag at once (outcome `shutdown`, kept) and exits 0 within seconds.
* **Folder = the side_cut_vision sidecar folder**: `<clip-dir>/<YYYYmmdd_HHMMSS>/clip.json` + `bag/` + a copy of the seam snapshot
  (`--snapshot`, best effort). `clip.json` has `"recording": true` while the episode is open (rewritten every 20 s; side_cut_vision's
  `open_clip_dir` finds it unchanged and its probe files land in this folder), then `false` with `outcome`, `reason`, `stop_reason`, `kept`,
  the coordinator's `exec_reports`, `topics` (messages, bytes, rate, longest gap per topic), `camera_status` (every camera of the list: messages,
  recorded or absent), `dropped_camera_messages`, `messages_written`. `<clip-dir>/index.jsonl` gets one line per episode (folder, path, bag,
  start / end, outcome, trigger, topics) for the converter. `--clip-dir` default = tools/cut_recorder's out dir: the **external drive**
  `/media/farhan/My Passport/igus_nvidia/cut_videos` if mounted, else `/home/farhan/Igus_rebel/outputs/cut_videos`; the recorder warns loudly
  at start-up when the drive is not mounted (`/` is nearly full; expect about 15-20 MB/s with all cameras) and refuses to start an episode below
  `--min-free-gb` (5).
* **The writer never blocks the subscriptions**: a queue + writer thread; when more than `--queue-mb` (512) is queued, the oldest *camera*
  messages are dropped first (counted in `clip.json`); joint, status and tf messages are never dropped.
* **Cameras**: `--camera-keys` (default all of `head knife wrist_left_knife wrist_right_knife wrist_left_gripper wrist_right_gripper left_vision
  right_vision`), `--camera KEY=TOPIC`. A camera that is not publishing is simply absent from the bag (it is recorded as soon as it appears); after
  `--camera-discover-s` (10) the log says which ones deliver and `clip.json` lists the others as not recorded.

```bash
recording/run_record_raw_cut.sh --trigger dual-cut --camera-domain 11     # by hand (domain 11 by default, read-only)
python3 recording/record_raw_cut.py --now --clip-dir /tmp/clips           # one episode until Ctrl-C
```

A bag whose recorder was killed with SIGKILL has no `metadata.yaml` (`ros2 bag reindex` repairs it). The LeRobot recorder described
before (`record_dataset.py`, `--headless`, `--camera-domain`, sidecar, `fjt-status`) stays as the manual tool; the launches no longer use it.

**Cameras** are read over ROS topics only, never the in-process USB reader (it would fight the running camera drivers for
the devices). `--camera-preset cut` (the default of `run_record_cut.sh`) = `head` (`/camera/color/image_raw/compressed`, Gemini
336) + `wrist_left_gripper` + `wrist_right_gripper` (`/arm_{side}/gripper_camera/image_raw/compressed`): the cameras that
`ros2 launch rebel_cameras cameras.launch.py` runs by default. The wrist Gemini 305s are off by default since 2026-10-26
(`wrist:=true right_wrist:=true`), then add `--camera wrist_left_knife=/arm_left/wrist_camera/color/image_raw/compressed`
(`--camera-preset cell` = all 5); the Logitech knife webcam (`ros2 launch knife_camera knife_camera.launch.py`) is
`--camera knife=/knife_camera/image_raw/compressed`. Recording waits for every chosen camera, so leave out cameras that are not running.

**ROS network.** The cutting stack of `igus_rebel_ros2` runs on **ROS_DOMAIN_ID 11**, default RMW (Fast DDS), localhost-only off,
so `run_record_cut.sh` uses that (it prints the domain / RMW; `ros_env.sh`'s real default 12 is the teleop network and is not
changed). `CUT_DOMAIN_ID=<n>` overrides it, `CUT_LOCALHOST_ONLY=1` exports `ROS_LOCALHOST_ONLY=1` (mock stacks), `IGUS_WS`
points to the workspace whose `install/setup.bash` provides `dual_cut_msgs` (dual cell only, imported lazily; default `~/igus_rebel_ros2`).
Domain 11 is the real robot's domain: the recorder is read-only, but never run tests there.

**CRI MoveJoint transits** (`cri_transit_enabled`, opt-in, off by default): while the robot runs a joint move itself the arm's
JTC is inactive and its `controller_state` stops, so the action goes stale (older than 0.2 s). Those frames keep the last
setpoint and are counted in the save line ("N with a stale action"); no action is invented. `observation.state` keeps following
`/joint_states`.

Limits and caveats: a barrier or goal failure of phase **B** after phase A completed looks like a normal finish (the
coordinator publishes no failure status for it; the recorder cannot know how many phases were meant), so such an episode is
saved. Saving encodes the videos one after another (AV1): a 130 s episode with 3 cameras took about 1 min, and while it is saved
no new episode can start (an execution that starts meanwhile is recorded from when the save is done: the log shows `execution N started` first and `RECORDING` later; one that is already over is skipped and logged; by design, not exercised end-to-end). Verified in mock only (unit + integration
tests, and the dual mock stack with fake cameras: `~/igus_rebel_ros2/scan_results/cut_recorder_20261026/`); **not** verified on
the real robot, the real `t_cut_status` of a running single-arm stack, the real cameras, or with a CRI run.

## Tools

| Script | Purpose |
| --- | --- |
| `setup/install.sh` | Asks what to install (Isaac Sim, recorder, legacy packages), then installs; `setup/check_env.sh` checks the result. |
| `launcher/run_gui.sh` | Settings window + process manager (recommended entry point). |
| `dual_arm_sim/run_dual_arm_sim.sh [--cameras head wrist_left wrist_right] [--headless]` | Isaac Sim rig + ROS 2 bridge. Do not source `/opt/ros` in this terminal. |
| `quest_teleop/run_quest_dual_arm.sh [--arms left right] [--yaw-deg 180 --swap-hands]` | Quest → both arms, direct to Isaac. |
| `recording/run_record.sh --name N --task "…" [--source real]` | Recorder. Keyboard fallback: Enter = start/save, d = discard, q = quit. |
| `recording/run_record_mocap.sh --name N --task "…" --camera KEY=TOPIC` | Read-only recorder for the real mocap-driven arm (see "Record real-arm mocap episodes"). |
| `recording/run_record_moveit.sh --name N --task "…" --camera-preset cell-right` | Read-only recorder for the real arm driven by a MoveIt program through its trajectory controller (action = controller setpoint). |
| `recording/run_record_raw_cut.sh --trigger dual-cut` | Raw rosbag2 recorder of the cutting runs (what the launches start). |
| `recording/run_record_cut.sh --name N [--single-arm] [--keep-stopped]` | Read-only recorder of the cutting runs of `~/igus_rebel_ros2`: one episode per execute call (see "Record cutting runs"). |
| `latest_run/` | Newest session only, emptied on every launcher Start: `<process>.log` copies and `teleop_trace.csv` (per tick and arm: Quest hand, grip, target, IK, joint command sent, measured joints, lag, which limit acted, log messages). Terminal runs overwrite the trace too (`--trace ''` = off). Timestamped logs stay in `logs/`. |
| `recording/run_cameras.sh --list / --snapshot / --test 10` | Real rig USB cameras: find nodes, save one JPEG per configured camera (`logs/camera_snapshots/`), measure fps together. |
| `launcher/run_replay_gui.sh [dataset folder]` | Playback window: choose a dataset folder + episode, then **View** (LeRobot viewer / Rerun) or **Replay in Isaac**. |
| `recording/run_replay.sh --path <dataset folder> --episode K` | Replays recorded actions into Isaac and compares with recorded states (alignment check). |
| `quest_teleop/run_teleop_viz.sh` | RViz view of the teleop frames (arm bases, control point, hand-motion axes). |
| `dual_arm_sim/start_right_arm_moveit.sh` | MoveIt/RViz planning for the right arm (do not run together with Quest teleop). |

## RViz of the physical arms

`./quest_teleop/run_teleop_viz.sh --source real` (the launcher opens it by default in Real mode): both arms from the
measured joints the teleop publishes on `/quest_teleop/joint_states` (bases from the hand-eye calibrations, world =
left base_link), each Quest controller as TF `{side}/controller` under the frame `quest` (green sphere = grip
driving the arm), and `/quest_teleop/{side}/target`. Read-only.

## Inter-arm collision guard

Each arm is modelled as capsules along its links and tooling (radius 6 cm links, 4 cm flange/XEG-32/knife). A
teleop step that would bring the two arms' surfaces closer than `--min-arm-dist` (5 cm) while reducing the
clearance is refused (both arms hold; release and re-grip). On the real arms the two bases are placed from the
hand-eye calibrations in `config/arm_calibration.json` (camera_link in each base_link; right base 0.448 m from the
left one; sources in `config/calibration/`); the simulation uses the USD poses. With **one** real arm selected the
other arm is **assumed to stay at all joints 0°** (straight up; `--other-arm-deg` for another pose) and its
controller is not contacted. If someone is moving that arm, add `--watch-other`: its live pose is then read
**read-only** over CRI (STATUS only; the link hands control back with `SetActive false`), and the driven arm holds
if that state goes stale. Not modelled: stand, table, cameras; the calibrations are marked unverified by their authors.

## Pre-test of a physical arm in RViz (igus ROS driver)

```bash
source /opt/ros/humble/setup.bash && source igus_ws/install/setup.bash
ros2 launch real_robot/rviz_pretest.launch.py arm:=left   # arm:=right, enable_motors:=false, hardware_protocol:=mock_hardware
```

Starts the igus driver for that arm (`config/real_arms.json`), enables the motors about 6 s later
(`/igus_rebel/reset_and_enable`), and opens MoveIt + RViz, all on ROS_DOMAIN_ID 91, localhost only. Use
Velocity/Acceleration Scaling 0.05; start from straight up with a joint move. Goal states `home` = (0, 0, 65, 0, 50, 0)°
and `zero`. Ctrl+C stops everything.
Do not run it together with the Quest teleop: one CRI client controls an arm.

## Physical arms through the igus ROS driver (teleop only)

Like the Isaac sim, the Quest teleop publishes on ROS topics; the igus ROS driver (`Igus_rebel`, CRI) drives each arm.
Verified on both real arms (2026-10-16): enable, home, back to 0°.

```bash
# terminal 1: one driver per arm in /left and /right (IPs from config/real_arms.json), motors enabled after ~6 s
source /opt/ros/humble/setup.bash && source igus_ws/install/setup.bash
ros2 launch real_robot/dual_arm_driver.launch.py arms:=left,right
# terminal 2: Quest teleop -> /<side>/rebel_arm_trajectory_controller/joint_trajectory
./quest_teleop/run_quest_real_arm.sh --arms left right
```

The launcher's Real mode does both (and RViz). After connecting the arms move to home (0, 0, 65, 0, 50, 0)°, on
exit back to 0°. The direct CRI client (`--backend real`, below) is kept as an experimental alternative.

Arm controller (`gains:=`, GUI "Arm gains", `config/real_controller_gains.json`): the default `tuned` is the igus
driver's controller from Igus_rebel `fani8j/jtc-tracking` 287b0186 (pinned in `setup/install_ros_ws.sh`): trajectory
gains p 3 / ff 0.6 and `jog_velocity_scale` 1.65, validated on both real arms with planned (MoveIt-style) moves on
2026-10-31. `standard` (p 1 / ff 0.4) and `responsive` are the older profiles; they pin the old scale 2.0 they were
measured with. `upstream` = the driver's own `controller.yaml` and xacro default.

## Controller-side Cartesian jog (`--backend cri-jog`, fake-tested only)

The controller does the IK: no IK on this PC (the URDF is used for the arm-to-arm guard's forward kinematics only). One
or both arms, translation only. The teleop maps the hand motion
since the grip (same yaw / scale as the IK backends) to a target position, reads the robot's own position
(`robot_cart_pose`) and streams `v = clamp(kp * (target - position), 3 cm/s)` as a `TwistStamped` (base frame,
stamped now(), >= 20 Hz) to the igus driver's `cart_jog_cmd`, which turns it into CRI ALIVEJOG X/Y/Z. Needs the
`igus_rebel_ros2` build that has `cart_jog_enable`, `cart_jog_cmd`, `hardware_state.cart_jog_*` and `RobotCartPose`
(see its README, "Teleoperation through the controller's Cartesian jog"). **The driver's %-to-m/s mapping and the A/B/C
axes are unmeasured; this path has only run against a fake driver.**

```bash
# driver terminal (same ROS_DOMAIN_ID as the teleop; the script prints it, default: domain 42, localhost only)
ros2 launch igus_rebel rebel.launch.py hardware_protocol:=rebel robot_ip:=<ip>
ros2 control switch_controllers --deactivate rebel_arm_trajectory_controller   # the driver refuses the mode otherwise
# teleop terminal: first --dry-run (computes + traces the twist, publishes nothing, no service call), e-stop in reach
IGUS_REBEL_INSTALL=/path/to/igus_rebel_ros2/install/setup.bash ./quest_teleop/run_quest_cri_jog.sh --arms right --dry-run
IGUS_REBEL_INSTALL=... ./quest_teleop/run_quest_cri_jog.sh --arms right --cri-max-lin-vel 0.01
```

- `--arms left` / `--arms right` / `--arms left right` (the default is both). Options: `--cri-max-lin-vel` (0.03 m/s), `--cri-kp`, `--cri-deadband`,
  `--cri-max-offset` (0.30 m around the grip point), `--cri-signs 1,1,1` (mirror an axis if the controller base frame is
  mirrored against the URDF base_link - not yet checked on the real arm, see `tools/igus_fk_check` in igus_rebel_ros2),
  `--cri-pose-units mm|m`, `--cri-cmd-topic / --cri-enable-service / --cri-state-topic / --cri-pose-topic` (`{side}`
  allowed; defaults `/igus_rebel/...` for one arm, `/igus_rebel_{side}/...` for two; two arms must resolve to different
  names), `--cri-no-auto-enable`, `--no-arm-guard`, `--cri-guard-margin`.
- Start: `cart_jog_enable true` (refused -> the driver's message is printed and the teleop exits with code 2). Exit
  (Ctrl+C / SIGTERM / error): zero twist, then `cart_jog_enable false`. No home / park motion: the robot moves only
  while the grip is held (open the hand once before the first grip).
- Zero twist at once on: grip released, tracking lost, robot pose older than 0.2 s, the driver leaving the mode,
  controller KINSTATE != 0, another publisher on `cart_jog_cmd` (also refused at start), an arm that moves against or
  does not follow the command (axis sign / mapping wrong). Then open and close the grip again; moving against the
  command latches until the teleop is restarted.
- Buttons: A / B = record events, Y (left hand, held 1 s) = discard event only (no motion), X (rotation frame) has no effect.
  Rotation is not implemented (TODO in `quest_teleop/cri_jog.py`).
- `/quest_teleop/action` is NOT published in this backend (there are no joint commands and none are invented); the
  operator's command is `/igus_rebel/cart_jog_cmd` and `latest_run/teleop_trace.csv` (cri-jog columns: robot / target
  position, twist, jog state, pose age). `/quest_teleop/{side}/hand`, `/state`, `/events`, `/joint_states` (measured, for
  RViz) are published.
- Needs on the teleop PC: `.venv_quest` (isaacteleop, scipy) and CloudXR as for the other backends; the `igus_rebel_ros2`
  install (for `igus_rebel_msgs`) only has to be sourced. `igus_ws` / the URDF are not used.
- Tests: `tests/test_cri_jog_teleop.py` (fake drivers; `source` the igus_rebel_ros2 install first).

### Both arms and the arm-to-arm guard (`--arms left right`, fake-tested only)

```bash
# dual stack driver (one instance per arm: /igus_rebel_left/..., /igus_rebel_right/...), then deactivate BOTH controllers
ros2 control switch_controllers --deactivate arm_left_trajectory_controller arm_right_trajectory_controller
./quest_teleop/run_quest_cri_jog.sh --arms left right --dry-run     # then without --dry-run, low --cri-max-lin-vel
```

- Each hand drives its own arm (`--swap-hands` swaps) with its own clutch and re-grip state. Releasing or losing one
  hand stops only that arm. A fault of ONE arm stops BOTH (zero twist, both grips must be opened and closed again), like
  the IK path: stale pose, latched / disconnected / stale driver, KINSTATE != 0, another publisher on a jog topic, the
  driver leaving the Cartesian mode by itself (it most likely latched, and the other arm would otherwise keep working
  while its partner is dead), an arm moving against its command (that arm then stays latched until a restart), the guard
  not knowing where an arm is.
- Start: the mode is entered on every arm; if any refuses, the arms that entered are left again and the teleop exits
  with code 2 and each driver's message. Exit: zero twist to all arms, then `cart_jog_enable false` on each.
- **Arm guard** (`quest_teleop/arm_collision.py` capsules, `--min-arm-dist` 5 cm surface gap, radii as in the IK path,
  FK from the URDF of the sourced `igus_rebel_description`, joints from each driver's `robot_cart_pose`): REACTIVE, it
  only sees the newest measured pose. Within `min_arm_dist + margin` (margin = arms driven x `--cri-max-lin-vel` x
  (0.2 s pose timeout + one tick + 0.15 s driver stop) ~ 2.3 cm at 3 cm/s with two arms) an arm is stopped when its
  commanded velocity points toward the other arm along the closest-point direction (closest point on the flange /
  gripper / knife, which move like the commanded TCP), or when the measured clearance shrinks by more than 2 mm/s
  (forearm / elbow contacts, mirrored axes). A stopped arm needs a new grip; moving away is allowed at once. The guard
  assumes the controller base axes equal the URDF `base_link`. One arm: the other is assumed at `--other-arm-deg`
  (default straight up), or read live from `/igus_rebel_{side}/robot_cart_pose` with `--watch-other` (a stale pose stops
  the driven arm).
- The guard needs the URDF: without it the teleop refuses to start unless `--no-arm-guard` is given (loud warning, nothing
  keeps the arms apart). The stand, table, cameras and the tool beyond the capsule model are not modelled.
- The trace has one row per arm per tick with `clearance_mm` and `guard`; the log shows the clearance.

## Direct Ethernet CRI client (experimental, `--backend real`)

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

**Home at start, 0° at exit:** after connecting, the arms first move slowly to the home pose (0, 0, 65, 0, 50, 0)°
(`--no-start-home` skips it); on Stop all / Ctrl+C they move on their own back to all joints 0° (`--park-deg`)
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
It is not a full stand/table/self-collision checker. Physical gripper control remains unsupported (real
cameras and real recording are on the igus ROS driver path, see *Real-robot recording*).

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
- The inter-arm guard (5 cm between capsule surfaces) only checks arm vs arm, not the stand, table, cameras or box.
- Wrist pitch is limited to about ±40° from home by joint 5.
- Not yet verified with a real headset end to end: gripper channel in a recorded episode, the left controller,
  and the A/B/X/Y buttons driving the launcher.

## Third-party material

See [`THIRD_PARTY.md`](THIRD_PARTY.md).
