#!/usr/bin/env python3
"""Record the dual-arm Quest teleop (Isaac sim or the real ReBeLs) into a LeRobotDataset (v3.0, HFRVLA lerobot).

Inputs (ROS 2, read through Isaac Sim's bundled Python-3.12 rclpy - see run_record.sh):
  /dual_arm/isaac_joint_states     -> observation.state (14): arm_{left,right}_joint1..6 + gripper closedness
  /quest_teleop/action             -> action            (14): same names, commanded by the teleop
  /dual_arm/{head,wrist_left,wrist_right}/rgb -> observation.images.{head,wrist_left,wrist_right}
  /quest_teleop/events             <- Quest buttons: "toggle" (A: start / save), "discard" (B), "reset" (Y: scene reset,
                                      discards an open episode)
  /clock                           -> sampling clock (sim time)

Frames are sampled every 1/fps of SIM time (Isaac runs below real time, ~0.77x), so timestamps in the
dataset match the simulated motion. Keyboard fallback in this terminal: Enter = toggle, d = discard,
q = save/finish and quit.

--source real (the physical arms, quest_teleop/run_quest_real_arm.sh = --backend real-ros):
  /{left,right}/joint_states       -> observation.state arm joints (igus ROS driver of each arm); an arm without a
                                      driver (one-arm session) takes the pose the teleop assumes for it from
                                      /quest_teleop/joint_states. Gripper slots are 0 (no real gripper).
  /quest_teleop/action             -> action (gripper slots = the Quest trigger, i.e. operator intent only)
  USB cameras of config/real_cameras.json (recording/real_cameras.py, read in this process, not over ROS)
                                   -> observation.images.{wrist_left,wrist_right,head,knife}
  sampling clock = wall time (time.monotonic()); robot_type igus_rebel_dual_arm_real.

  robot_remote/recording/run_record.sh --name dual_rebel_pick --task "pick up the box"

--source mocap-real (REAL arm driven by the OptiTrack mocap pipeline; run_record_mocap.sh, system python3 + /opt/ros/humble)
  Same 14-value observation.state / action layout, but the sources are the real robot's topics and the sampling
  clock is the WALL clock (time.monotonic) at --fps. The recorder is READ-ONLY: it only subscribes (no publishers,
  services or clients of its own besides rclpy's automatic /rosout and /parameter_events).
    observation.state  <- --state-topic (default /joint_states, names --state-names, default joint{i}) for the --arms
                          driven sides (default right); the undriven arm is the constant --other-arm-deg
    action             <- --action-topic (default /dual_arm/isaac_joint_commands, names --action-names, default
                          arm_{side}_joint{i}): the RAW mocap target, NOT the follower-filtered command;
                          undriven arm = --other-arm-deg
    grippers           state and action are 0.0 (open) on both arms - the mocap path has no gripper
    observation.images.KEY <- every --camera KEY=TOPIC (any number of cameras, any KEY of [A-Za-z0-9_]; Image
                          rgb8/bgr8/rgba8/bgra8/mono8/yuv422_yuy2(yuyv)/uyvy or CompressedImage jpeg/png, resized
                          to --width x --height). Recommended keys for the real cell (5 cameras): head,
                          wrist_left_gripper, wrist_left_knife, wrist_right_gripper, wrist_right_knife.
                          --list-cameras shows what is available, --pick-cameras asks
    observation.follower_state (2,) float32 [left, right] <- JSON on --follower-status-topic ({side} is replaced)
                          0 = arm not driven, or no status within 0.5 s
                          1 = DISABLED  2 = HOLD  3 = CATCHUP  4 = ENABLED (following)  5 = REFUSED  6 = other string
  robot_type is "igus_rebel_dual_arm_real_mocap". Enter / d / q on the keyboard and /quest_teleop/events still
  start-save / discard / quit.

  robot_remote/recording/run_record_mocap.sh --name right_pick --task "pick up the box" \\
      --camera head=/camera/color/image_raw --camera wrist_right=/right_wrist_camera/color/image_raw

--source moveit-real (REAL arm driven by a MoveIt / scripted waypoint program on another PC; run_record_moveit.sh)
  Identical to mocap-real (same 14-D layout, cameras, wall clock, keys, read-only node) except for the action:
    action             <- control_msgs/JointTrajectoryControllerState on --controller-state-topic (default
                          /rebel_arm_trajectory_controller/controller_state, {side} is replaced), joint names from
                          --action-names (default joint{i}): the trajectory controller's SETPOINT (`reference`, or
                          `desired` for an older controller that leaves `reference` empty), not the measured position
    no observation.follower_state feature; robot_type is "igus_rebel_dual_arm_real_moveit"
  robot_remote/recording/run_record_moveit.sh --name box_open_01 --task "open the box" --camera-preset cell-right

--source moveit-real --episode-trigger dual-cut | h-cut (CUTTING runs of ~/igus_rebel_ros2; run_record_cut.sh; read-only)
  The episode follows the cutting stack instead of the keyboard: one episode per execute call, saved when the execution
  ends normally, discarded when it is STOPPED / fails / aborts (--keep-stopped saves it, task suffixed
  " [stopped: <reason>]"). Enter / d / q still start / save / discard / quit. Cameras over ROS topics only (--camera-preset cut).
    dual-cut  /dual_cut_coordinator/status (dual_cut_msgs) + /dual_cut_coordinator/exec_report; robot_type
              igus_rebel_dual_arm_real_cut; observation.cut_progress = [coordinator state (0 IDLE 1 PLANNING 2 PLANNED
              3 WAITING_BARRIER 4 EXECUTING 5 STOPPED), phase (0 none, 1 A, 2 B)]
    h-cut     /box_h_cut_executor/t_cut_status; robot_type igus_rebel_single_arm_real_cut;
              observation.cut_progress = [step (0 start, 1 approach_standoff, 2 side_cut_penetrate, 3 continue_side_cut,
              4 continue_top_cut, 5 done), status (0 none, 1 running, 2 done, 3 failed)]
  Details and the start / end rules: recording/cut_trigger.py.
"""

import argparse
import collections
import inspect
import io
import json
import math
import queue
import re
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import rclpy.executors

import cut_sidecar
import cut_trigger
import dataset_check
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_action_status_default,
                       qos_profile_sensor_data)
from rosgraph_msgs.msg import Clock
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import String

import os

DEBUG = os.environ.get("REC_DEBUG") == "1"

SIDES = ("left", "right")
ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
NAMES = [f"arm_{s}_{j}" for s in SIDES for j in ARM_JOINTS + ["gripper"]]  # 14, fixed order
CAMERAS = {"head": "/dual_arm/head/rgb", "wrist_left": "/dual_arm/wrist_left/rgb",
           "wrist_right": "/dual_arm/wrist_right/rgb"}
REAL_STATE_TOPIC = "/{side}/joint_states"            # igus ROS driver (real_robot/dual_arm_driver.launch.py)
REAL_FALLBACK_TOPIC = "/quest_teleop/joint_states"   # teleop: measured joints + assumed pose of an undriven arm
ROBOT_TYPE = dataset_check.ROBOT_TYPE
GRIPPER_OPEN, GRIPPER_CLOSED = 0.00483688, -0.0116631  # XEG-32 carriage [m] (same as the teleop)


def stamp_s(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def image_to_np(msg: Image) -> np.ndarray:
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
    if ch is None:
        raise ValueError(f"unsupported image encoding {msg.encoding}")
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width * ch]
    a = a.reshape(msg.height, msg.width, ch)
    if msg.encoding.startswith("bgr"):
        a = a[..., [2, 1, 0] + ([3] if ch == 4 else [])]
    return np.ascontiguousarray(a[..., :3])


IMAGE_TYPES = {"sensor_msgs/msg/Image": Image, "sensor_msgs/msg/CompressedImage": CompressedImage}
YUV422_ENCODINGS = {"yuv422_yuy2": "yuyv", "yuyv": "yuyv", "yuv422": "uyvy", "uyvy": "uyvy"}
RAW_ENCODINGS = ("rgb8", "bgr8", "rgba8", "bgra8", "mono8") + tuple(YUV422_ENCODINGS)
FOLLOWER_CODES = {"DISABLED": 1, "HOLD": 2, "CATCHUP": 3, "ENABLED": 4, "REFUSED": 5}
FOLLOWER_OTHER = 6
FOLLOWER_FRESH_S = 0.5
KEY_RE = re.compile(r"^[A-Za-z0-9_]+$")
MOCAP_ROBOT_TYPE = "igus_rebel_dual_arm_real_mocap"
MOVEIT_ROBOT_TYPE = "igus_rebel_dual_arm_real_moveit"
SOURCE_DEFAULTS = {  # option -> default per --source (argparse default None, filled by apply_source_defaults)
    "mocap-real": {"action_names": "arm_{side}_joint{i}"},
    "moveit-real": {"action_names": "joint{i}"},
}
ACTION_TOPIC_DEFAULT = "/dual_arm/isaac_joint_commands"
FOLLOWER_TOPIC_DEFAULT = "/mocap_follower/status"
CONTROLLER_STATE_DEFAULT = "/rebel_arm_trajectory_controller/controller_state"
RECOMMENDED_KEYS = ["head", "wrist_left_gripper", "wrist_left_knife", "wrist_right_gripper", "wrist_right_knife", "knife"]
from cut_cameras import CAMERA_PRESETS, CELL_CAMERAS, MAIN_DOMAIN_CAMERAS, NOT_IN_CELL_PRESET  # noqa: E402


def _yuv_to_rgb(y, u, v):
    y = (y.astype(np.float32) - 16.0) * 1.164
    u = u.astype(np.float32) - 128.0
    v = v.astype(np.float32) - 128.0
    r = y + 1.596 * v
    g = y - 0.392 * u - 0.813 * v
    b = y + 2.017 * u
    return np.clip(np.stack([r, g, b], axis=-1), 0, 255).astype(np.uint8)


def image_msg_to_rgb(msg, topic: str = "?", min_size=None) -> np.ndarray:
    """sensor_msgs Image / CompressedImage -> HxWx3 uint8 RGB. Raises ValueError (naming topic and encoding).

    min_size=(w, h): a JPEG may be decoded at 1/2, 1/4 or 1/8 scale, never below min_size (faster, then resized).
    """
    if isinstance(msg, CompressedImage):
        from PIL import Image as PILImage

        fmt = msg.format.lower()
        if not any(f in fmt for f in ("jpeg", "jpg", "png")):
            raise ValueError(f"{topic}: unsupported compressed format {msg.format!r} (jpeg/png only)")
        try:
            img = PILImage.open(io.BytesIO(bytes(msg.data)))
            if min_size is not None:
                img.draft("RGB", tuple(min_size))  # no-op for PNG
            return np.asarray(img.convert("RGB"))
        except Exception as e:  # noqa: BLE001
            raise ValueError(f"{topic}: cannot decode compressed image ({msg.format}): {e}") from e
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8", "rgba8", "bgra8"):
        return image_to_np(msg)
    if enc == "mono8":
        a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width]
        return np.ascontiguousarray(np.repeat(a[..., None], 3, axis=2))
    if enc in YUV422_ENCODINGS:
        a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 2]
        a = a.reshape(msg.height, msg.width // 2, 4)  # 2 pixels per 4 bytes
        if YUV422_ENCODINGS[enc] == "yuyv":
            y0, u, y1, v = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
        else:  # uyvy
            u, y0, v, y1 = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
        y = np.stack([y0, y1], axis=2).reshape(msg.height, msg.width)
        return _yuv_to_rgb(y, np.repeat(u, 2, axis=1), np.repeat(v, 2, axis=1))
    raise ValueError(f"{topic}: unsupported image encoding {msg.encoding!r} (supported: {', '.join(RAW_ENCODINGS)})")


def resize_rgb(a: np.ndarray, width: int, height: int) -> np.ndarray:
    if a.shape[1] == width and a.shape[0] == height:
        return a
    from PIL import Image as PILImage

    return np.asarray(PILImage.fromarray(a).resize((width, height), PILImage.BILINEAR))


class Recorder(Node):
    """cameras: real source only, {key: object with latest() -> (rgb, time.monotonic()) and alive()}."""

    def __init__(self, args, cameras=None):
        super().__init__("dataset_recorder")
        self.args = args
        self.real = args.source == "real"
        self.cameras = cameras or {}
        self.state = self.action = None
        self.state_stamp = 0.0
        self.action_wall = 0.0
        self.images, self.image_stamp = {}, {}
        self.sim_time = None
        self.real_q, self.real_t = {}, {}      # side -> joints from its driver, arrival time.monotonic()
        self.fallback_q = {}                   # side -> joints from /quest_teleop/joint_states
        self.events: queue.Queue[str] = queue.Queue()
        self.create_subscription(JointState, "/quest_teleop/action", self.on_action, 10)
        self.create_subscription(String, "/quest_teleop/events", lambda m: self.events.put(m.data), 10)
        if self.real:
            for s in SIDES:
                self.create_subscription(JointState, REAL_STATE_TOPIC.format(side=s),
                                         lambda m, s=s: self.on_real_state(s, m), 10)
            self.create_subscription(JointState, REAL_FALLBACK_TOPIC, self.on_fallback_state, 10)
            return
        self.create_subscription(JointState, "/dual_arm/isaac_joint_states", self.on_state, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        for key, topic in CAMERAS.items():
            if key in args.cameras:
                self.create_subscription(Image, topic, lambda m, k=key: self.on_image(k, m), qos_profile_sensor_data)

    def now(self):
        """Sampling clock: sim time (/clock, None until the first message) or wall time on the real robot."""
        return time.monotonic() if self.real else self.sim_time

    def on_real_state(self, side, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            self.real_q[side], self.real_t[side] = [float(d[j]) for j in ARM_JOINTS], time.monotonic()

    def on_fallback_state(self, msg):
        d = dict(zip(msg.name, msg.position))
        for s in SIDES:
            names = [f"arm_{s}_{j}" for j in ARM_JOINTS]
            if all(n in d for n in names):
                self.fallback_q[s] = [float(d[n]) for n in names]

    def real_snapshot(self, starting=False):
        """Real source: (frame, stale) or (None, reason). starting=True (a new episode): every camera must be delivering
        right now; inside an open episode a camera that stalled for less than --camera-gap-s only counts as stale
        (longer stalls end the episode before this is called, see lost_camera)."""
        now = time.monotonic()
        max_age = 2.0 / self.args.fps
        if self.action is None:
            return None, "no /quest_teleop/action (is the real teleop running?)"
        state, stale = [], []
        for s in SIDES:
            if s in self.real_q:  # driver of this arm is running: always use it, report when it lags
                q = self.real_q[s]
                if now - self.real_t[s] > max_age:
                    stale.append(f"{s} joint_states {now - self.real_t[s]:.2f}s")
            elif s in self.fallback_q:
                q = self.fallback_q[s]
            else:
                return None, (f"no {REAL_STATE_TOPIC.format(side=s)} and no {REAL_FALLBACK_TOPIC} "
                              "(drivers / teleop not running?)")
            state += q + [0.0]  # no physical gripper: closedness slot stays 0
        frame = {"observation.state": np.asarray(state, np.float32)}
        for k in self.args.cameras:
            cam = self.cameras[k]
            rgb, t = cam.latest()
            if rgb is None or not cam.alive():
                return None, f"camera {k} {'stopped' if rgb is not None else 'has no frame'}" + (
                    f" ({cam.error})" if getattr(cam, "error", None) else "")
            if starting and getattr(cam, "state", "ok") != "ok":
                return None, (f"camera {k} is lost (USB disconnect?) - reconnecting, try again in a moment"
                              + (f" ({cam.error.split(' (')[0]})" if getattr(cam, "error", None) else ""))
            if starting and now - t > self.args.camera_gap_s:
                return None, f"camera {k} has no fresh frame for {now - t:.1f} s"
            if now - t > max_age:
                stale.append(f"{k} {now - t:.2f}s")
            frame[f"observation.images.{k}"] = rgb
        if now - self.action_wall > 0.2:
            stale.append(f"action {now - self.action_wall:.2f}s")
        frame["action"] = self.action.copy()
        return frame, stale

    def on_state(self, msg):
        d = dict(zip(msg.name, msg.position))
        out = []
        for s in SIDES:
            out += [d[f"arm_{s}_{j}"] for j in ARM_JOINTS]
            c = d.get(f"arm_{s}_xeg32_left_carriage_joint", GRIPPER_OPEN)
            out.append(np.clip((c - GRIPPER_OPEN) / (GRIPPER_CLOSED - GRIPPER_OPEN), 0.0, 1.0))
        self.state, self.state_stamp = np.asarray(out, np.float32), stamp_s(msg)

    def on_action(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(n in d for n in NAMES):
            self.action, self.action_wall = np.asarray([d[n] for n in NAMES], np.float32), time.monotonic()

    def on_image(self, key, msg):
        self.images[key], self.image_stamp[key] = msg, stamp_s(msg)

    def on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def snapshot(self, starting=False):
        """Latest frame if every stream is present, else (None, reason). Also returns staleness warnings."""
        if self.real:
            return self.real_snapshot(starting)
        if self.state is None:
            return None, "no /dual_arm/isaac_joint_states"
        if self.action is None:
            return None, "no /quest_teleop/action (is run_quest_dual_arm.sh running?)"
        missing = [k for k in self.args.cameras if k not in self.images]
        if missing:
            return None, f"no image yet from {missing} (start the sim with --cameras)"
        stale = []
        max_age = 2.0 / self.args.fps
        if self.sim_time is not None:
            for k in self.args.cameras:
                if self.sim_time - self.image_stamp[k] > max_age:
                    stale.append(f"{k} {self.sim_time - self.image_stamp[k]:.2f}s")
            if self.sim_time - self.state_stamp > max_age:
                stale.append(f"state {self.sim_time - self.state_stamp:.2f}s")
        if time.monotonic() - self.action_wall > 0.2:
            stale.append(f"action {time.monotonic() - self.action_wall:.2f}s(wall)")
        frame = {"observation.state": self.state.copy(), "action": self.action.copy()}
        for k in self.args.cameras:
            frame[f"observation.images.{k}"] = image_to_np(self.images[k])
        return frame, stale


def lost_camera(cameras: dict, keys, gap_s: float, now: float):
    """(key, time.monotonic() of its last frame) of a camera without a fresh frame for more than gap_s, else None.
    A camera that has never delivered counts as lost since `now`."""
    for k in keys:
        t = cameras[k].latest()[1]
        if t is None or now - t > gap_s:
            return k, (now if t is None else t)
    return None


class FrameHold:
    """Real source: frames wait hold_s before they enter the dataset. A camera drop is only recognised gap_s after its
    last frame, and the frames sampled in between carry that camera's frozen last image; holding frames back lets the
    recorder cut them off, so the episode ends exactly at the drop. hold_s = 0: add immediately (sim)."""

    def __init__(self, ds, hold_s=0.0):
        self.ds, self.hold_s = ds, hold_s
        self.pending = collections.deque()   # (monotonic time sampled, frame)
        self.committed = 0

    def reset(self):
        self.pending.clear()
        self.committed = 0

    def total(self):
        return self.committed + len(self.pending)

    def push(self, frame, wall):
        self.pending.append((wall, frame))
        self.commit_due(wall)

    def commit_due(self, wall):
        while self.pending and wall - self.pending[0][0] >= self.hold_s:
            self.ds.add_frame(self.pending.popleft()[1])
            self.committed += 1

    def flush(self):
        while self.pending:
            self.ds.add_frame(self.pending.popleft()[1])
            self.committed += 1

    def drop_after(self, wall) -> int:
        """Forget the pending frames sampled after `wall`; commit the rest. Returns how many were dropped."""
        keep = [(w, f) for w, f in self.pending if w <= wall]
        dropped = len(self.pending) - len(keep)
        self.pending = collections.deque(keep)
        self.flush()
        return dropped


def _accepts(fn, name: str) -> bool:
    try:
        return name in inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return False


def dataset_root(args) -> Path:
    return Path(args.root).expanduser().resolve() / args.name


def check_resumable(args):
    """Fail early (before any ROS or prompt) if nothing can be recorded to <root>/<name>: a folder that is not a
    dataset, an unfinished dataset, or an existing dataset that this LeRobot cannot resume (0.4.x has no resume)."""
    st = dataset_check.inspect(dataset_root(args))
    ok, why = dataset_check.describe(st)
    if not ok:
        sys.exit(f"[rec] ERROR: {st['path']}: {why}")
    if st["state"] == "dataset":
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        if not hasattr(LeRobotDataset, "resume"):
            import lerobot

            print(f"[rec] ERROR: dataset {st['path']} already exists and this LeRobot "
                  f"({getattr(lerobot, '__version__', '?')}) has no LeRobotDataset.resume - use a new --name "
                  "(or a LeRobot 0.5.x python to append).", file=sys.stderr, flush=True)
            sys.exit(2)
    return st["path"]


def make_dataset(args, camera_keys=None, extra_features=None, robot_type=None):
    """Create or resume <root>/<name>. camera_keys / extra_features / robot_type: the ROS sources (mocap-real,
    moveit-real); the Isaac / USB-camera sources take them from args.source."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = dataset_root(args)
    robot_type = robot_type or ROBOT_TYPE[args.source]
    features = {
        "observation.state": {"dtype": "float32", "shape": (len(NAMES),), "names": NAMES},
        "action": {"dtype": "float32", "shape": (len(NAMES),), "names": NAMES},
    }
    features.update(extra_features or {})
    for k in (args.cameras if camera_keys is None else camera_keys):
        h, w = args.image_shapes[k] if args.source == "real" else (args.height, args.width)
        features[f"observation.images.{k}"] = {"dtype": "video", "shape": (h, w, 3),
                                               "names": ["height", "width", "channels"]}
    st = dataset_check.inspect(root)
    if st["state"] == "dataset":
        check_resumable(args)  # a LeRobot without LeRobotDataset.resume cannot append
        check_resume(root / "meta" / "info.json", features, robot_type, args.fps)
        print(f"[rec] resuming existing dataset {root} ({st['episodes']} episodes)", flush=True)
        # image_writer_threads: resume() defaults to 0 = PNGs written inside add_frame; with 3 cameras at 640x480 that
        # capped recording at ~8 frames/s (every tick late, 2026-10-21). create() below gets the same threads.
        kw = {"vcodec": args.vcodec} if _accepts(LeRobotDataset.resume, "vcodec") else {}
        kw.update(streaming_kwargs(args, LeRobotDataset.resume))
        return configure_video(LeRobotDataset.resume(repo_id=args.repo_id, root=root,
                                                     image_writer_threads=args.image_writer_threads, **kw), args), root
    ok, why = dataset_check.describe(st)
    if not ok:
        raise SystemExit(f"[rec] ERROR: {root}: {why}")
    if st["state"] != "new":
        print(f"[rec] {why}", flush=True)
        dataset_check.prepare_new(st)
    print(f"[rec] creating dataset {root}", flush=True)
    kw = {"vcodec": args.vcodec} if _accepts(LeRobotDataset.create, "vcodec") else {}
    kw.update(streaming_kwargs(args, LeRobotDataset.create))
    ds = LeRobotDataset.create(repo_id=args.repo_id, fps=args.fps, features=features, root=root,
                               robot_type=robot_type, use_videos=True,
                               image_writer_threads=args.image_writer_threads, **kw)
    return configure_video(ds, args), root


def streaming_kwargs(args, fn) -> dict:
    """Encode each camera to MP4 while recording (LeRobot's streaming_encoding) instead of writing every frame as a
    PNG and encoding them all at save time: saving takes seconds, and no GBs of temporary PNGs (100 s x 3 cameras
    = 2.7 GB, 2026-11-03). Only when this LeRobot has it and --no-streaming-encode is not given."""
    if getattr(args, "streaming_encode", True) and _accepts(fn, "streaming_encoding"):
        kw = {"streaming_encoding": True}
        if _accepts(fn, "encoder_queue_maxsize"):
            kw["encoder_queue_maxsize"] = args.encoder_queue
        return kw
    return {}


def configure_video(ds, args):
    """Keyframe interval (--video-gop) for this dataset's videos. LeRobot hard-codes g=2 (a keyframe every 2 frames:
    fast random access in training, ~9x larger files than g=30 on the real cameras, 2026-11-03); it has no option, so
    set it on the streaming encoder, and on the module-level encode function used by the PNG path."""
    gop = getattr(args, "video_gop", None)
    if not gop:
        return ds
    enc = getattr(ds, "_streaming_encoder", None)
    if enc is not None and hasattr(enc, "g"):
        enc.g = gop
        print(f"[rec] video: encoded while recording ({enc.vcodec}, keyframe every {gop} frames)", flush=True)
        return ds
    import lerobot.datasets.lerobot_dataset as lds

    orig = getattr(lds, "encode_video_frames", None)
    if orig is not None and _accepts(orig, "g") and not getattr(orig, "_rr_gop", None):
        def encode_video_frames(*a, **kw):
            kw.setdefault("g", gop)
            return orig(*a, **kw)
        encode_video_frames._rr_gop = gop
        lds.encode_video_frames = encode_video_frames
        print(f"[rec] video: encoded at save time (PNG frames), keyframe every {gop} frames", flush=True)
    else:
        print("[rec] video: encoded at save time with LeRobot's default keyframe interval", flush=True)
    return ds


def save_episode(ds) -> bool:
    """ds.save_episode, but discard the episode if the streaming encoder dropped video frames (its queue was full):
    the video would then be shorter than state/action and out of sync. True = saved."""
    enc = getattr(ds, "_streaming_encoder", None)
    dropped = {k: n for k, n in (getattr(enc, "_dropped_frames", None) or {}).items() if n}
    if dropped:
        ds.clear_episode_buffer()
        print("[rec] ERROR: the video encoder could not keep up and dropped frames "
              + ", ".join(f"{k.split('.')[-1]}: {n}" for k, n in dropped.items())
              + " - episode DISCARDED (video would be out of sync). Fewer cameras or --no-streaming-encode.",
              flush=True)
        return False
    ds.save_episode(parallel_encoding=False)  # process-pool encoding deadlocks next to rclpy threads (fork)
    return True


def check_resume(info_path, features, robot_type, fps):
    """Refuse to append to a dataset recorded with other cameras / sizes / fps or the other source (sim vs real)."""
    info = json.loads(Path(info_path).read_text())
    have = {k: tuple(v["shape"]) for k, v in info["features"].items() if k.startswith(("observation.", "action"))}
    want = {k: tuple(v["shape"]) for k, v in features.items()}
    problems = []
    if have != want:
        problems.append(f"features {sorted(have.items())} != requested {sorted(want.items())}")
    if info.get("robot_type") != robot_type:
        problems.append(f"robot_type {info.get('robot_type')} != {robot_type}")
    if info.get("fps") != fps:
        problems.append(f"fps {info.get('fps')} != {fps}")
    if problems:
        raise SystemExit(f"[rec] ERROR: {info_path.parent.parent} already exists with a different setup: "
                         + "; ".join(problems) + ". Choose another dataset name.")


# ---------------------------------------------------------------------------------------------------------
# --source mocap-real / moveit-real: read-only recording of the real arm (OptiTrack mocap pipeline / MoveIt program)
# ---------------------------------------------------------------------------------------------------------
BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
# state edges must not be lost: reliable, volatile (matches the coordinator's reliable and the executor's reliable +
# transient_local publishers; volatile = no replay of an old run when the recorder starts)
TRIGGER_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)


def controller_topics(args):
    return sorted({args.controller_state_topic.format(side=s) for s in args.arms})


class MocapRecorder(Node):
    """Subscribes only. Best-effort QoS matches both reliable and best-effort publishers.

    args.source "mocap-real" (default): action = raw mocap target (JointState) + follower status feature.
    args.source "moveit-real": action = the trajectory controller's setpoint (JointTrajectoryControllerState)."""

    def __init__(self, args, cameras: dict, cam_context=None):
        self.moveit = getattr(args, "source", "mocap-real") == "moveit-real"
        super().__init__("dataset_recorder_moveit" if self.moveit else "dataset_recorder_mocap",
                         start_parameter_services=False)
        self.args = args
        self.cameras = dict(cameras)  # key -> (topic, ros type name)
        self.lock = threading.Lock()
        self.logged_cams: set = set()
        # One decode thread per camera keeps only the newest message, so JPEG decode + resize (~5-8 ms per frame)
        # runs off the sampling loop and in parallel (PIL releases the GIL). key -> (resized rgb | ValueError, rx)
        self.pending: dict = {}
        self.decoded: dict = {}
        self.wake = {k: threading.Event() for k in cameras}
        self.closing = False
        # cutting runs (trigger modes): a camera must never cost an episode -> missing / undecodable frames are replaced
        # by the last good frame (black if none) and flagged in cam_flags
        self.robust = getattr(args, "episode_trigger", "manual") != "manual"
        self.last_good: dict = {}
        self.cam_flags: dict = {}      # key -> "stale" | "missing" | "error" of the latest snapshot()
        for key in cameras:
            threading.Thread(target=self._decode_loop, args=(key,), daemon=True, name=f"decode_{key}").start()
        self.state, self.action, self.follower, self.images = {}, {}, {}, {}  # -> (value, rx monotonic)
        self.other_rad = np.deg2rad(np.asarray(args.other_arm_deg, np.float32))
        self.events: queue.Queue[str] = queue.Queue()
        self.stale_max = max(2.0 / args.fps, 0.1)
        self.cam_stale_max = max(3.0 / args.fps, 0.2)  # 15 fps cameras are normally 0.07-0.11 s old at a tick
        self.create_subscription(JointState, args.state_topic, self.on_state, BEST_EFFORT)
        if self.moveit:
            for topic in controller_topics(args):  # joint names in the message select the arm
                self.create_subscription(JointTrajectoryControllerState, topic, self.on_controller_state, BEST_EFFORT)
        else:
            self.create_subscription(JointState, args.action_topic, self.on_action, BEST_EFFORT)
            for topic in sorted({args.follower_status_topic.format(side=s) for s in args.arms}):
                self.create_subscription(String, topic, lambda m, t=topic: self.on_follower(t, m), BEST_EFFORT)
        self.create_subscription(String, "/quest_teleop/events", lambda m: self.events.put(m.data), 10)
        self.trigger = None
        mode = getattr(args, "episode_trigger", "manual")
        if mode != "manual":  # cutting runs: the episode follows the coordinator / executor status (subscriptions only)
            self.trigger = cut_trigger.make_trigger(mode, self.events.put, lambda t: print(t, flush=True),
                                                    post_roll_s=getattr(args, "post_roll_s", 5.0))
            if mode == "dual-cut":
                from dual_cut_msgs.msg import CoordinatorStatus  # lazily: only this mode needs the igus_rebel_ros2 install

                self.create_subscription(CoordinatorStatus, args.trigger_topic, self.trigger.on_status, TRIGGER_QOS)
                self.create_subscription(String, args.exec_report_topic, lambda m: self.trigger.on_report(m.data),
                                         TRIGGER_QOS)
            elif mode == "fjt-status":
                from action_msgs.msg import GoalStatusArray

                self.create_subscription(GoalStatusArray, args.trigger_topic, self.trigger.on_status,
                                         qos_profile_action_status_default)
            else:
                self.create_subscription(String, args.trigger_topic, lambda m: self.trigger.on_status(m.data), TRIGGER_QOS)
        # camera topics may live in another ROS domain (second context + executor thread); the vision overlays never do
        self.cam_node = self.cam_executor = self.cam_thread = None
        if cam_context is not None and any(k not in MAIN_DOMAIN_CAMERAS for k in cameras):
            self.cam_node = Node("dataset_recorder_cameras", context=cam_context, start_parameter_services=False)
            self.cam_executor = SingleThreadedExecutor(context=cam_context)
            self.cam_executor.add_node(self.cam_node)
            self.cam_thread = threading.Thread(target=self.cam_executor.spin, daemon=True, name="cam_executor")
            self.cam_thread.start()
        for key, (topic, tname) in self.cameras.items():
            msg_t = IMAGE_TYPES[tname]
            owner = self.cam_node if (self.cam_node is not None and key not in MAIN_DOMAIN_CAMERAS) else self
            owner.create_subscription(msg_t, topic, lambda m, k=key: self.on_image(k, m), qos_profile_sensor_data)

    def _joints(self, msg, fmt, side):
        return self._pick(msg.name, msg.position, fmt, side)

    @staticmethod
    def _pick(names_in, positions, fmt, side):
        d = dict(zip(names_in, positions))
        names = [fmt.format(side=side, i=i) for i in range(1, 7)]
        if all(n in d for n in names):
            return np.asarray([d[n] for n in names], np.float32)
        return None

    def on_state(self, msg):
        now = time.monotonic()
        for s in self.args.arms:
            v = self._joints(msg, self.args.state_names, s)
            if v is not None:
                with self.lock:
                    self.state[s] = (v, now)

    def on_action(self, msg):
        now = time.monotonic()
        for s in self.args.arms:
            v = self._joints(msg, self.args.action_names, s)
            if v is not None:
                with self.lock:
                    self.action[s] = (v, now)

    def on_controller_state(self, msg):
        """JointTrajectoryController setpoint: `reference` (humble+), else `desired` (older JTC), else ignore."""
        n = len(msg.joint_names)
        if n and len(msg.reference.positions) == n:
            pos = msg.reference.positions
        elif n and len(msg.desired.positions) == n:
            pos = msg.desired.positions
        else:
            return
        now = time.monotonic()
        for s in self.args.arms:
            v = self._pick(msg.joint_names, pos, self.args.action_names, s)
            if v is not None:
                with self.lock:
                    self.action[s] = (v, now)

    def on_follower(self, topic, msg):
        try:
            state = str(json.loads(msg.data).get("state"))
        except (ValueError, AttributeError):
            state = "?"
        code = FOLLOWER_CODES.get(state, FOLLOWER_OTHER)
        with self.lock:
            for s in self.args.arms:
                if self.args.follower_status_topic.format(side=s) == topic:
                    self.follower[s] = (code, time.monotonic())

    def on_image(self, key, msg):
        with self.lock:
            self.pending[key] = (msg, time.monotonic())
        self.wake[key].set()

    def _decode_loop(self, key):
        topic = self.cameras[key][0]
        a = self.args
        while not self.closing:
            if not self.wake[key].wait(0.2):
                continue
            self.wake[key].clear()
            with self.lock:
                msg, rx = self.pending.pop(key, (None, None))
            if msg is None:
                continue
            try:
                rgb = image_msg_to_rgb(msg, topic, min_size=(a.width, a.height))
                if key not in self.logged_cams:
                    self.logged_cams.add(key)
                    full = (getattr(msg, "width", 0), getattr(msg, "height", 0))
                    if isinstance(msg, CompressedImage):
                        from PIL import Image as PILImage
                        full = PILImage.open(io.BytesIO(bytes(msg.data))).size
                    enc = getattr(msg, "encoding", None) or getattr(msg, "format", "?")
                    note = "" if full == (a.width, a.height) else f" -> resized to {a.width}x{a.height}"
                    if full != (a.width, a.height) and abs(full[0] / full[1] - a.width / a.height) > 0.01:
                        note += " (aspect ratio changes)"
                    print(f"[rec] camera {key} ({topic}): {full[0]}x{full[1]} {enc}{note}", flush=True)
                out = resize_rgb(rgb, a.width, a.height)
                self.last_good[key] = out
            except ValueError as e:
                out = e
            with self.lock:
                self.images[key] = (out, rx)

    def close(self):
        self.closing = True
        for e in self.wake.values():
            e.set()
        if self.cam_executor is not None:
            self.cam_executor.shutdown(timeout_sec=1.0)
            if self.cam_thread is not None:
                self.cam_thread.join(timeout=2.0)
            self.cam_node.destroy_node()
            self.cam_node = None

    def snapshot(self):
        """(frame, stale list) when every stream has data, else (None, reason)."""
        a = self.args
        now = time.monotonic()
        with self.lock:
            state, action = dict(self.state), dict(self.action)
            follower, images = dict(self.follower), dict(self.images)
        for s in a.arms:
            if s not in state:
                return None, f"no {a.state_topic} with {a.state_names.format(side=s, i=1)}..{a.state_names.format(side=s, i=6)}"
            if s not in action:
                j = f"{a.action_names.format(side=s, i=1)}..{a.action_names.format(side=s, i=6)}"
                if self.moveit:
                    return None, (f"no setpoint (reference/desired positions) with {j} on "
                                  f"{' / '.join(controller_topics(a))}")
                return None, f"no {a.action_topic} with {j}"
        missing = [k for k in self.cameras if k not in images]
        if missing and not self.robust:
            return None, f"no image yet from {missing}"
        stale = []
        self.cam_flags = {}
        for s in a.arms:
            if now - state[s][1] > self.stale_max:
                stale.append(f"state[{s}] {now - state[s][1]:.2f}s")
            if now - action[s][1] > 0.2:
                stale.append(f"action[{s}] {now - action[s][1]:.2f}s")
        for k in self.cameras:
            if k in images and now - images[k][1] > self.cam_stale_max:
                stale.append(f"{k} {now - images[k][1]:.2f}s")
                self.cam_flags[k] = "stale"
        st, ac, fo = [], [], []
        for s in SIDES:
            if s in a.arms:
                st += list(state[s][0]) + [0.0]
                ac += list(action[s][0]) + [0.0]
                f = follower.get(s)
                fo.append(float(f[0]) if f is not None and now - f[1] <= FOLLOWER_FRESH_S else 0.0)
            else:
                st += list(self.other_rad) + [0.0]
                ac += list(self.other_rad) + [0.0]
                fo.append(0.0)
        frame = {"observation.state": np.asarray(st, np.float32), "action": np.asarray(ac, np.float32)}
        if not self.moveit:
            frame["observation.follower_state"] = np.asarray(fo, np.float32)
        if self.trigger is not None:
            frame["observation.cut_progress"] = np.asarray(self.trigger.progress(), np.float32)
        for k in self.cameras:
            img = images[k][0] if k in images else None
            if img is None or isinstance(img, ValueError):
                if not self.robust and img is not None:
                    raise img
                last = self.last_good.get(k)
                if last is None:  # nothing ever decoded: black frame of the dataset's size
                    last = np.zeros((a.height, a.width, 3), np.uint8)
                self.cam_flags[k] = "missing" if img is None else "error"
                stale.append(f"{k} {self.cam_flags[k]}")
                img = last
            frame[f"observation.images.{k}"] = img
        return frame, stale


def discover_cameras(duration_s: float, context=None):
    """Listen for duration_s and describe every Image / CompressedImage topic on the graph (of `context` or the default)."""
    node = Node("dataset_recorder_cameras", context=context, start_parameter_services=False)
    ex = SingleThreadedExecutor(context=context)
    ex.add_node(node)
    info: dict = {}

    def sub(topic, tname):
        d = info[topic] = {"topic": topic, "type": tname, "n": 0, "first": None, "last": None, "size": None,
                           "enc": None, "error": None}

        def cb(m, d=d):
            now = time.monotonic()
            d["n"] += 1
            d["first"] = d["first"] or now
            d["last"] = now
            if d["size"] is None:
                try:
                    if isinstance(m, CompressedImage):
                        from PIL import Image as PILImage

                        d["size"] = PILImage.open(io.BytesIO(bytes(m.data))).size
                        d["enc"] = m.format
                    else:
                        d["size"], d["enc"] = (m.width, m.height), m.encoding
                        if m.encoding.lower() not in RAW_ENCODINGS:
                            d["error"] = f"unsupported encoding {m.encoding!r}"
                except Exception as e:  # noqa: BLE001
                    d["error"] = f"cannot read image: {e}"
        node.create_subscription(IMAGE_TYPES[tname], topic, cb, qos_profile_sensor_data)

    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        for topic, types in node.get_topic_names_and_types():
            for t in types:
                if t in IMAGE_TYPES and topic not in info:
                    sub(topic, t)
        ex.spin_once(timeout_sec=0.1)
    node.destroy_node()
    out = []
    for d in sorted(info.values(), key=lambda x: x["topic"]):
        d["hz"] = (d["n"] - 1) / (d["last"] - d["first"]) if d["n"] >= 2 and d["last"] > d["first"] else None
        out.append(d)
    return out


def print_cameras(found):
    if not found:
        print("[rec] no sensor_msgs/msg/Image or CompressedImage topics found", flush=True)
        return
    print(f"[rec] {len(found)} image topic(s):", flush=True)
    for d in found:
        if d["size"] is None:
            desc = "no frames received" if d["n"] == 0 else d["error"] or "?"
        else:
            hz = f"{d['hz']:.1f} Hz" if d["hz"] else "1 frame"
            desc = f"{d['size'][0]}x{d['size'][1]}  {d['enc']}  {hz}"
            if d["error"]:
                desc += f"  UNSUPPORTED: {d['error']}"
        print(f"  {d['topic']:55s} {d['type'].split('/')[-1]:16s} {desc}", flush=True)


def pick_cameras(found, input_fn=input):
    """Ask for a key per topic (empty = skip). Returns {key: (topic, type name)}."""
    picked = {}
    print("[rec] name each camera to record (empty = skip). Recommended keys: " + ", ".join(RECOMMENDED_KEYS), flush=True)
    for d in found:
        if d["error"]:
            print(f"  skipping {d['topic']}: {d['error']}", flush=True)
            continue
        while True:
            try:
                key = input_fn(f"  key for {d['topic']} ({d['type'].split('/')[-1]}): ").strip()
            except EOFError:
                key = ""
            if not key:
                break
            if not KEY_RE.match(key):
                print("  key must match [A-Za-z0-9_]+", flush=True)
            elif key in picked:
                print(f"  key {key!r} already used", flush=True)
            else:
                picked[key] = (d["topic"], d["type"])
                break
    return picked


def topic_types(topics, discover_s, context=None):
    """Message type of each camera topic (from the graph); defaults to Image if the topic is not visible yet."""
    node = Node("dataset_recorder_cameras", context=context, start_parameter_services=False)
    ex = SingleThreadedExecutor(context=context)
    ex.add_node(node)
    t0 = time.monotonic()
    types = {}
    while time.monotonic() - t0 < discover_s and len(types) < len(topics):
        known = dict(node.get_topic_names_and_types())
        for t in topics:
            for tn in known.get(t, []):
                if tn in IMAGE_TYPES:
                    types[t] = tn
        ex.spin_once(timeout_sec=0.1)
    node.destroy_node()
    return types


def probe_cameras(specs, cam_context, timeout_s):
    """Cut mode start-up: wait up to timeout_s for every camera to deliver a frame. specs = {key: topic}.
    Returns ({key: (topic, type)} of the cameras that delivered one, [keys that did not]). The dataset's feature set is
    fixed from this, so a camera that is off at start-up is not recorded at all (the launch starts the recorder together
    with the stack; the cameras have their own launch)."""
    nodes = {"main": Node("dataset_recorder_probe", start_parameter_services=False)}
    execs = {"main": SingleThreadedExecutor()}
    execs["main"].add_node(nodes["main"])
    if cam_context is not None:
        nodes["camera"] = Node("dataset_recorder_probe", context=cam_context, start_parameter_services=False)
        execs["camera"] = SingleThreadedExecutor(context=cam_context)
        execs["camera"].add_node(nodes["camera"])
    got, subs = {}, {}
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s and len(got) < len(specs):
        for key, topic in specs.items():
            if key in subs:
                continue
            where = "camera" if "camera" in nodes and key not in MAIN_DOMAIN_CAMERAS else "main"
            for tn in dict(nodes[where].get_topic_names_and_types()).get(topic, []):
                if tn in IMAGE_TYPES:
                    subs[key] = (nodes[where].create_subscription(
                        IMAGE_TYPES[tn], topic, lambda m, k=key: got.__setitem__(k, True), qos_profile_sensor_data), tn)
                    break
        for ex in execs.values():
            ex.spin_once(timeout_sec=0.05)
    for w, n in nodes.items():
        execs[w].shutdown()
        n.destroy_node()
    kept = {k: (specs[k], subs[k][1]) for k in specs if k in got}
    return kept, [k for k in specs if k not in got]


def retask_episode(ds, task):
    """Replace the task text of the frames buffered so far (the suffix of a kept stopped episode is known only at the
    end). Returns False if this LeRobot keeps the buffer in another shape (the plain task is then saved)."""
    buf = getattr(ds, "episode_buffer", None)
    if isinstance(buf, dict) and isinstance(buf.get("task"), list):
        buf["task"] = [task] * len(buf["task"])
        return True
    return False


STOP_EVENT = threading.Event()   # --headless: set by SIGINT / SIGTERM


def run_mocap(args, cameras, ds, root, cam_context=None):
    node = MocapRecorder(args, cameras, cam_context)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)  # callbacks in their own thread (as the other sources)
    spin_thread.start()
    trig = node.trigger
    headless = getattr(args, "headless", False)
    print(f"[rec] {args.source}: arms driven = {args.arms}; undriven arm held at {args.other_arm_deg} deg; grippers 0.0", flush=True)
    if node.moveit:
        print(f"[rec] state {args.state_topic} ({args.state_names}) | action = controller setpoint "
              f"{controller_topics(args)} ({args.action_names})", flush=True)
    else:
        print(f"[rec] state {args.state_topic} ({args.state_names}) | action {args.action_topic} ({args.action_names}) | "
              f"follower status {sorted({args.follower_status_topic.format(side=s) for s in args.arms})}", flush=True)
    print(f"[rec] cameras: {({k: v[0] for k, v in cameras.items()} or 'none')}", flush=True)
    if cam_context is not None:
        print(f"[rec] camera topics are read in ROS domain {args.camera_domain} (second context); status, joint states and "
              f"controller state in domain {rclpy.get_default_context().get_domain_id()}"
              + ("; vision overlays stay in the robot domain" if any(k in MAIN_DOMAIN_CAMERAS for k in cameras) else ""),
              flush=True)
    print(f"[rec] {len(NAMES)}-D state/action: {NAMES}", flush=True)
    if trig is not None:
        print(f"[rec] episode trigger {args.episode_trigger}: status {args.trigger_topic}"
              + (f", phase reports {args.exec_report_topic}" if args.episode_trigger == "dual-cut" else "")
              + (f", post-roll {args.post_roll_s:g}s" if args.episode_trigger == "fjt-status" else "")
              + f" | an execution that ends normally is saved; a stopped / failed one is "
              + (f"saved with the task suffixed [stopped: <reason>]" if args.keep_stopped else "discarded")
              + f" | task: {args.task!r} | observation.cut_progress = {cut_trigger.FEATURE_NAMES[args.episode_trigger]}",
              flush=True)
        if args.clip_dir:
            print(f"[rec] sidecar clip folders (clip.json for side_cut_vision) under {args.clip_dir}", flush=True)
    if headless:
        print("[rec] READ-ONLY (subscribes only), --headless: no keyboard; SIGINT / SIGTERM discard an open episode, "
              "finalize the dataset and exit 0", flush=True)
    else:
        print("[rec] READ-ONLY (subscribes only). Controls: keyboard Enter = start/save, d = discard, q = quit "
              "(Quest events on /quest_teleop/events also work)"
              + (" - Enter/d/q override the cut trigger" if trig is not None else ""), flush=True)
        threading.Thread(target=keyboard_thread, args=(node.events,), daemon=True).start()

    period = 1.0 / args.fps
    recording = False
    owner = None             # "manual" (keyboard / Quest) or "auto" (cut trigger) - an auto end only closes an auto episode
    ep_exec = None           # execution id of an auto episode
    pending_start = None     # execution id whose episode could not start yet (a stream is missing): retried each loop
    next_t = ep_start = 0.0
    n_frames = n_stale = n_late = n_action_stale = 0
    cam_stats: dict = {}     # camera key -> {"stale": n, "missing": n, "error": n} frames of the open episode
    sidecar = None
    ep_index = 0
    last_status = last_skip = last_try = 0.0

    def try_start(who, exec_id=None, late_note=""):
        """Open an episode if every stream is present. Returns True if it started."""
        nonlocal recording, owner, ep_exec, next_t, ep_start, n_frames, n_stale, n_late, n_action_stale, cam_stats
        nonlocal sidecar, ep_index
        try:
            frame, why = node.snapshot()
        except ValueError as e:  # unsupported image encoding / undecodable frame
            frame, why = None, str(e)
        if frame is None:
            print(f"[rec] cannot start: {why}", flush=True)
            return False
        recording, owner, ep_exec = True, who, exec_id
        n_frames = n_stale = n_late = n_action_stale = 0
        cam_stats = {k: {"stale": 0, "missing": 0, "error": 0} for k in cameras}
        next_t = ep_start = time.monotonic()
        ep_index = ds.meta.total_episodes
        print(f"[rec] ● RECORDING episode {ep_index}  (task: {args.task})" + late_note, flush=True)
        if trig is not None and args.clip_dir:
            sidecar = cut_sidecar.Sidecar(
                args.clip_dir,
                {"backend": "lerobot", "trigger": args.episode_trigger, "owner": who, "exec_id": exec_id,
                 "dataset": str(root), "episode_index": ep_index, "episode_index_note": "pending: set when saved",
                 "task": args.task, "fps": args.fps, "cameras": {k: v[0] for k, v in cameras.items()},
                 "robot_type": ds.meta.robot_type, "keep_stopped": args.keep_stopped},
                None if args.no_snapshot else args.snapshot, log=lambda t: print(t, flush=True))
            if sidecar.dir:
                print(f"[rec] sidecar clip folder {sidecar.dir}", flush=True)
        return True

    def close_episode(save, label, task=None, outcome="manual", reason="", exec_info=None):
        nonlocal recording, sidecar
        recording = False
        sc, sidecar = sidecar, None
        final = {"outcome": outcome, "reason": reason, "frames": n_frames,
                 "wall_s": round(time.monotonic() - ep_start, 2), "frames_with_stale_stream": n_stale,
                 "late_ticks": n_late, "frames_with_stale_action": n_action_stale,
                 "camera_frame_flags": cam_stats, "task_saved": task or args.task,
                 "stop_reason": (exec_info or {}).get("stop_reason", ""), **{k: v for k, v in (exec_info or {}).items()
                                                                              if k != "stop_reason"}}
        will_save = save and n_frames >= 2
        if sc is not None:   # recording:false BEFORE the (slow) save so the probe stops writing into this clip
            sc.finish(saved="saving" if will_save else False, **final)
        if will_save:
            if task is not None and task != args.task and not retask_episode(ds, task):
                print("[rec] WARNING: cannot change the task text of the buffered frames with this LeRobot; "
                      "the plain task is saved", flush=True)
            print(f"[rec] saving episode {ds.meta.total_episodes} ({n_frames} frames, "
                  f"{time.monotonic() - ep_start:.1f}s wall, {n_stale} frames had a stale stream, "
                  f"{n_late} late ticks" + (f", {n_action_stale} with a stale action" if trig is not None else "")
                  + f"){label} ...", flush=True)
            if save_episode(ds):
                print(f"[rec] ■ saved. total episodes: {ds.meta.total_episodes}", flush=True)
                if sc is not None:
                    sc.finish(saved=True, episode_index=ds.meta.total_episodes - 1, **final)
            elif sc is not None:
                sc.finish(saved=False, **{**final, "outcome": "error", "reason": "video encoder dropped frames"})
        else:
            ds.clear_episode_buffer()
            print("[rec] episode too short - discarded" if save else f"[rec] ✕ discarded ({n_frames} frames){label}",
                  flush=True)

    try:
        while rclpy.ok() and not STOP_EVENT.is_set():
            now = time.monotonic()
            timeout = 0.05 if not recording else max(0.0, min(0.05, next_t - now))
            try:
                ev = node.events.get(timeout=timeout)
            except queue.Empty:
                ev = None
            if trig is not None:
                trig.poll()
            if sidecar is not None:
                sidecar.refresh(frames_so_far=n_frames)
            if isinstance(ev, tuple):  # cut trigger event
                if ev[0] == "start":
                    if recording:
                        print(f"[rec] execution {ev[1]} started but an episode is already open ({owner}) - not "
                              "starting another", flush=True)
                    else:
                        pending_start = ev[1]
                elif ev[0] == "end":
                    _, eid, outcome, reason = ev
                    info = trig.ended_info.pop(eid, {})
                    if pending_start == eid:
                        pending_start = None
                        print(f"[rec] execution {eid} ended ({outcome}) before an episode could start", flush=True)
                    if recording and owner == "auto" and ep_exec == eid:
                        if outcome == "finished":
                            close_episode(True, f" - execution finished ({reason})", outcome=outcome, reason=reason,
                                          exec_info=info)
                        elif args.keep_stopped:
                            close_episode(True, f" - execution {outcome}: {reason}",
                                          cut_trigger.task_with_suffix(args.task, outcome, reason), outcome, reason, info)
                        else:
                            close_episode(False, f" - execution {outcome}: {reason}", outcome=outcome, reason=reason,
                                          exec_info=info)
                    elif recording and ev[1] != ep_exec:
                        print(f"[rec] execution {eid} ended ({outcome}); the open episode is not tied to it "
                              f"({owner}) - left open", flush=True)
                continue
            if ev == "toggle" and not recording:
                pending_start = None
                try_start("manual")
            elif ev == "toggle" and recording:
                close_episode(True, " (manual)" if owner == "auto" else "", outcome="manual")
            elif ev == "reset":
                if recording:
                    close_episode(False, " - scene reset", outcome="reset")
            elif ev == "discard" and recording:
                close_episode(False, "", outcome="discarded by hand")
            elif ev == "quit":
                raise KeyboardInterrupt
            if pending_start is not None and not recording and now - last_try > 1.0:
                last_try = now
                if try_start("auto", pending_start, " [cut trigger]"):
                    pending_start = None
            if not recording or time.monotonic() < next_t:
                continue
            now = time.monotonic()
            next_t += period
            if now > next_t:  # stalled (encoding, GC...): do not try to catch up
                next_t = now + period
                n_late += 1
            try:
                frame, stale = node.snapshot()
            except ValueError as e:  # unsupported image encoding / undecodable frame
                print(f"[rec] ERROR: {e} - episode discarded", flush=True)
                close_episode(False, " - image error", outcome="error", reason=str(e))
                continue
            if frame is None:
                if now - last_skip > 1.0:
                    last_skip = now
                    print(f"[rec] frame skipped: {stale}", flush=True)
                continue
            frame["task"] = args.task
            ds.add_frame(frame)
            n_frames += 1
            for k, flag in node.cam_flags.items():
                cam_stats[k][flag] += 1
            if stale:
                n_stale += 1
                if any(x.startswith("action[") for x in stale):
                    n_action_stale += 1   # e.g. a CRI MoveJoint transit: the JTC is inactive, controller_state is silent
            if now - last_status > 1.0:
                last_status = now
                print(f"[rec] ● {n_frames} frames ({now - ep_start:.1f}s wall)"
                      + (f"  stale: {', '.join(stale)}" if stale else ""), flush=True)
            if now - ep_start > args.max_episode_s:
                if owner == "auto":
                    print(f"[rec] --max-episode-s {args.max_episode_s:g} reached: saving; the rest of execution "
                          f"{ep_exec} is NOT recorded", flush=True)
                node.events.put("toggle")
    except KeyboardInterrupt:
        pass
    finally:
        if recording and headless:
            # shut down fast: a finished episode was saved at its end; this one is incomplete
            print(f"[rec] shutdown: open episode discarded ({n_frames} frames)", flush=True)
            sc, sidecar = sidecar, None
            ds.clear_episode_buffer()
            if sc is not None:
                sc.finish(saved=False, outcome="shutdown", reason="recorder stopped", frames=n_frames)
        elif recording and n_frames >= 2:
            print(f"[rec] saving the open episode ({n_frames} frames) before exit ...", flush=True)
            saved = save_episode(ds)
            if sidecar is not None:
                sidecar.finish(saved=saved, outcome="manual" if saved else "error",
                               reason="recorder stopped" if saved else "video encoder dropped frames", frames=n_frames)
        ds.finalize()
        print(f"[rec] finalized {root}  ({ds.meta.total_episodes} episodes)", flush=True)
        node.close()
        ex.shutdown(timeout_sec=1.0)  # stop the callback thread before the node goes away
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


def apply_source_defaults(args, p):
    """Fill the options whose default depends on --source; reject options of the other real source."""
    moveit = args.source == "moveit-real"
    trigger = args.episode_trigger
    single_arm = trigger in cut_trigger.SINGLE_ARM_TRIGGERS
    if trigger != "manual" and not moveit:
        p.error("--episode-trigger needs --source moveit-real")
    if trigger == "manual" and (args.keep_stopped or args.trigger_topic or args.exec_report_topic):
        p.error("--keep-stopped / --trigger-topic / --exec-report-topic need --episode-trigger dual-cut, h-cut or fjt-status")
    if trigger != "dual-cut" and args.exec_report_topic:
        p.error("--exec-report-topic is only for --episode-trigger dual-cut")
    if trigger == "manual" and (args.headless or args.clip_dir or args.no_sidecar):
        p.error("--headless / --clip-dir / --no-sidecar need --episode-trigger dual-cut, h-cut or fjt-status")
    if trigger == "dual-cut":   # the dual cell: one /joint_states with arm_{side}_joint{i}, one JTC per side
        for opt, val in (("arms", ["left", "right"]), ("state_names", "arm_{side}_joint{i}"),
                         ("action_names", "arm_{side}_joint{i}"),
                         ("controller_state_topic", "/arm_{side}_trajectory_controller/controller_state")):
            if getattr(args, opt) is None:
                setattr(args, opt, val)
        if args.exec_report_topic is None:
            args.exec_report_topic = cut_trigger.DEFAULT_REPORT_TOPIC
    for opt, val in SOURCE_DEFAULTS.get(args.source, {}).items():
        if getattr(args, opt) is None:
            setattr(args, opt, val)
    if trigger != "manual":
        if args.trigger_topic is None:
            args.trigger_topic = cut_trigger.DEFAULT_TOPIC[trigger]
        if args.task is None:
            args.task = cut_trigger.DEFAULT_TASK
    if args.arms is None:
        args.arms = ["right"]
    if args.state_names is None:
        args.state_names = "joint{i}"
    if single_arm and len(set(args.arms)) != 1:
        p.error(f"--episode-trigger {trigger} records the single-arm stack: give exactly one side in --arms")
    if trigger != "manual" and args.post_roll_s < 0:
        p.error("--post-roll-s must be >= 0")
    if args.camera_domain is not None and not 0 <= args.camera_domain <= 232:
        p.error("--camera-domain must be 0..232 (Fast DDS)")
    if moveit:
        for opt, flag in (("action_topic", "--action-topic"), ("follower_status_topic", "--follower-status-topic")):
            if getattr(args, opt) is not None:
                p.error(f"{flag} is only for --source mocap-real; with --source moveit-real the action comes from "
                        "--controller-state-topic")
        if args.controller_state_topic is None:
            args.controller_state_topic = CONTROLLER_STATE_DEFAULT
    else:
        if args.controller_state_topic is not None:
            p.error("--controller-state-topic is only for --source moveit-real")
        if args.action_topic is None:
            args.action_topic = ACTION_TOPIC_DEFAULT
        if args.follower_status_topic is None:
            args.follower_status_topic = FOLLOWER_TOPIC_DEFAULT


def run_mocap_main(args, p):
    apply_source_defaults(args, p)
    cut = args.episode_trigger != "manual"
    if args.cameras is not None:
        p.error(f"--cameras is the Isaac option; use --camera KEY=TOPIC with --source {args.source}")
    if len(set(args.arms)) != len(args.arms):
        p.error("--arms lists a side twice")
    if args.source == "mocap-real" and args.state_topic == args.action_topic:
        p.error("--state-topic and --action-topic must differ")
    for opt in ("state_names", "action_names"):
        try:
            getattr(args, opt).format(side="left", i=1)
        except (KeyError, IndexError, ValueError) as e:
            p.error(f"--{opt.replace('_', '-')} must only use {{side}} and {{i}}: {e}")
    if args.source == "mocap-real" and len(args.arms) > 1 and "{side}" not in args.follower_status_topic:
        p.error("--follower-status-topic needs {side} when more than one arm is driven")
    if args.fps <= 0:
        p.error("--fps must be > 0")
    if not all(math.isfinite(x) for x in args.other_arm_deg):
        p.error("--other-arm-deg must be finite")
    specs = {k: CELL_CAMERAS[k] for k in CAMERA_PRESETS[args.camera_preset]} if args.camera_preset else {}
    for k in [x for x in (args.camera_keys or "").split(",") if x.strip()]:
        k = k.strip()
        if k not in CELL_CAMERAS:
            p.error(f"--camera-keys: unknown key {k!r}; known: {', '.join(CELL_CAMERAS)}")
        specs[k] = CELL_CAMERAS[k]
    for k, t in parse_camera_specs(args.camera, p).items():
        if k in specs and specs[k] != t:
            print(f"[rec] --camera {k}={t} replaces the preset topic {specs[k]}", flush=True)
        specs[k] = t
    if cut and not args.name:
        args.name = f"{'dual_cut' if args.episode_trigger == 'dual-cut' else 'h_cut'}_{time.strftime('%Y%m%d_%H%M%S')}"
        print(f"[rec] dataset name: {args.name}", flush=True)
    args.repo_id = args.repo_id if args.repo_id and not args.repo_id.endswith("/None") else f"local/{args.name}"
    if not args.list_cameras and (not args.name or not args.task):
        p.error("--name and --task are required")
    if cut and not args.no_sidecar and not args.clip_dir:
        args.clip_dir = cut_sidecar.default_clip_dir()
    if args.no_sidecar:
        args.clip_dir = None
    if not args.list_cameras:
        check_resumable(args)  # fail early, before any ROS or prompt
    if args.headless:
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        except ImportError:  # pragma: no cover
            rclpy.init(args=[])
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: STOP_EVENT.set())
    else:
        rclpy.init()
    cam_ctx = None
    try:
        main_domain = rclpy.get_default_context().get_domain_id()
        if args.camera_domain is not None and args.camera_domain != main_domain:
            from rclpy.context import Context

            cam_ctx = Context()
            try:
                from rclpy.signals import SignalHandlerOptions
                rclpy.init(args=[], context=cam_ctx, domain_id=args.camera_domain,
                           signal_handler_options=SignalHandlerOptions.NO)
            except ImportError:  # pragma: no cover
                rclpy.init(args=[], context=cam_ctx, domain_id=args.camera_domain)
        elif args.camera_domain is not None:
            print(f"[rec] --camera-domain {args.camera_domain} is the robot domain: one context", flush=True)
        if args.list_cameras:
            print_cameras(discover_cameras(args.discover_s, cam_ctx))
            return
        cameras = {}
        if args.pick_cameras:
            found = discover_cameras(args.discover_s, cam_ctx)
            print_cameras(found)
            cameras.update(pick_cameras(found))
        if specs and cut:
            kept, dropped = probe_cameras(specs, cam_ctx, args.camera_discover_s)
            for k in dropped:
                print(f"[rec] camera {k} ({specs[k]}) delivered no frame within {args.camera_discover_s:g}s: NOT recorded "
                      "(the feature set of this dataset is fixed now)", flush=True)
            cameras.update(kept)
        elif specs:
            for where, ctx in (("main", None), ("camera", cam_ctx)):
                keys = [k for k in specs if (k in MAIN_DOMAIN_CAMERAS or cam_ctx is None) == (where == "main")]
                if not keys:
                    continue
                types = topic_types([specs[k] for k in keys], args.discover_s, ctx)
                for k in keys:
                    t = specs[k]
                    cameras[k] = (t, types.get(t, "sensor_msgs/msg/Image"))
                    if t not in types:
                        print(f"[rec] WARNING: camera topic {t} not visible yet; assuming sensor_msgs/msg/Image", flush=True)
        if args.source == "moveit-real":
            extra, robot_type = {}, MOVEIT_ROBOT_TYPE
            if cut:
                extra = {"observation.cut_progress": {"dtype": "float32", "shape": (2,),
                                                      "names": cut_trigger.FEATURE_NAMES[args.episode_trigger]}}
                robot_type = cut_trigger.ROBOT_TYPE[args.episode_trigger]
        else:
            extra = {"observation.follower_state": {"dtype": "float32", "shape": (2,), "names": list(SIDES)}}
            robot_type = MOCAP_ROBOT_TYPE
        if args.image_writer_threads is None:
            args.image_writer_threads = max(4, 2 * len(cameras))
        ds, root = make_dataset(args, camera_keys=list(cameras), extra_features=extra, robot_type=robot_type)
        run_mocap(args, cameras, ds, root, cam_ctx)
    finally:
        if cam_ctx is not None:
            cam_ctx.try_shutdown()
        rclpy.try_shutdown()


def parse_camera_specs(specs, parser):
    out = {}
    for spec in specs or []:
        key, sep, topic = spec.partition("=")
        if not sep or not topic:
            parser.error(f"--camera expects KEY=TOPIC, got {spec!r}")
        if not KEY_RE.match(key):
            parser.error(f"--camera key {key!r} must match [A-Za-z0-9_]+")
        if key in out:
            parser.error(f"--camera key {key!r} given twice")
        out[key] = topic
    return out


def keyboard_thread(events: queue.Queue):
    for line in sys.stdin:
        c = line.strip().lower()
        events.put({"": "toggle", "d": "discard", "q": "quit"}.get(c, "toggle" if c == "s" else "unknown"))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", help="dataset folder name under --root (required except with --list-cameras)")
    p.add_argument("--task", help="language instruction stored with every frame (required except with --list-cameras)")
    p.add_argument("--root", default=None,
                   help="folder that holds the datasets (default: $ROBOT_REMOTE_DATASETS, else robot_remote/dataset; with "
                        "--episode-trigger the external drive's igus_nvidia/cut_datasets if mounted)")
    p.add_argument("--repo-id", default=None, help="default: local/<name>")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--source", choices=("sim", "isaac", "real", "mocap-real", "moveit-real"), default="sim",
                   help="sim (alias isaac) = Isaac topics + /clock; real = igus drivers + USB cameras, wall clock; "
                        "mocap-real / moveit-real = real arm over ROS topics only, read-only (see the options below)")
    p.add_argument("--cameras", nargs="*", default=None,
                   help="[sim / real] subset to record (empty = state/action only). sim: head wrist_left wrist_right, "
                        "must match the cameras the sim publishes; real: keys of --camera-config (default: all of them)")
    p.add_argument("--camera-config", default=None, help="[real] camera devices (default config/real_cameras.json)")
    p.add_argument("--image-writer-threads", type=int, default=None,
                   help="default: 4, or 2 per camera on real / mocap-real / moveit-real")
    p.add_argument("--vcodec", default="libsvtav1")
    p.add_argument("--video-gop", type=int, default=30,
                   help="keyframe every N frames (LeRobot default 2: ~9x larger files, faster random access in training;"
                        " 0 = LeRobot default)")
    p.add_argument("--no-streaming-encode", dest="streaming_encode", action="store_false",
                   help="write PNG frames and encode at save time (LeRobot default) instead of encoding while recording")
    p.add_argument("--encoder-queue", type=int, default=90,
                   help="frames buffered per camera for the streaming encoder; if it fills, frames are dropped and "
                        "the episode is discarded")
    p.add_argument("--camera-gap-s", type=float, default=0.5,
                   help="[real] a USB camera without a fresh frame for longer than this ends the open episode (saved up "
                        "to the drop if >= 1 s, else discarded; the camera is re-opened automatically and A works again "
                        "once it is back). Shorter stalls only count as stale frames")
    p.add_argument("--max-episode-s", type=float, default=None,
                   help="auto-save after this much sim / wall time (default 120; 600 with --episode-trigger)")
    g = p.add_argument_group("--source mocap-real / moveit-real (real arm, read-only; options shared by both unless noted)")
    g.add_argument("--arms", nargs="+", choices=SIDES, default=None,
                   help="driven sides (default: right; dual-cut: left right)")
    g.add_argument("--state-topic", default="/joint_states")
    g.add_argument("--state-names", default=None,
                   help="joint name format with {side} and {i}=1..6 (default joint{i}; dual stack and dual-cut: "
                        "arm_{side}_joint{i})")
    g.add_argument("--action-topic", default=None,
                   help=f"[mocap-real only] RAW mocap target (JointState, rad); default {ACTION_TOPIC_DEFAULT}")
    g.add_argument("--action-names", default=None,
                   help="joint name format of the action with {side} and {i}=1..6; default arm_{side}_joint{i} "
                        "(mocap-real, moveit-real dual-cut) or joint{i} (moveit-real otherwise)")
    g.add_argument("--follower-status-topic", default=None,
                   help=f"[mocap-real only] JSON status topic, {{side}} is replaced (default {FOLLOWER_TOPIC_DEFAULT}; "
                        "dual launch: /mocap_follower_{side}/status)")
    g.add_argument("--controller-state-topic", default=None,
                   help="[moveit-real only] control_msgs/JointTrajectoryControllerState whose setpoint is the action; "
                        f"{{side}} is replaced (default {CONTROLLER_STATE_DEFAULT}; dual stack e.g. "
                        "/arm_{side}_trajectory_controller/controller_state)")
    c = p.add_argument_group("--episode-trigger (cutting runs; needs --source moveit-real; run_record_cut.sh)")
    c.add_argument("--episode-trigger", choices=("manual", "dual-cut", "h-cut", "fjt-status"), default="manual",
                   help="manual (default): Enter / Quest start and save. dual-cut: an episode per ~/execute of "
                        "dual_cut_coordinator (all phases); h-cut: per execute_t_cut sequence of box_h_cut_executor; "
                        "fjt-status: per burst of FollowJointTrajectory goals of the single-arm controller (every "
                        "executed trajectory, like tools/cut_recorder). Enter / d / q still override")
    c.add_argument("--post-roll-s", type=float, default=5.0,
                   help="[fjt-status] the episode closes this long after the last active goal (default 5)")
    c.add_argument("--camera-domain", type=int, default=None, metavar="N",
                   help="read the camera topics in a second rclpy context in ROS domain N (real cell: cameras in 11, robot "
                        "stack in another domain); status, exec_report, joint_states, controller_state and the "
                        "left_vision / right_vision overlays stay in the robot domain. Default: one domain")
    c.add_argument("--headless", action="store_true",
                   help="no keyboard (launch): SIGINT / SIGTERM discard an open episode, finalize the dataset and exit 0")
    c.add_argument("--camera-discover-s", type=float, default=10.0,
                   help="start-up: wait up to this long for every chosen camera to deliver a frame; the others are "
                        "dropped for the whole session (default 10)")
    c.add_argument("--camera-keys", default=None, metavar="K1,K2",
                   help=f"cameras by key from: {', '.join(CELL_CAMERAS)} (added to --camera-preset / --camera)")
    c.add_argument("--clip-dir", default=None,
                   help="sidecar folders <YYYYmmdd_HHMMSS>/clip.json that side_cut_vision's probe finds (default: "
                        "tools/cut_recorder's out dir: the external drive's igus_nvidia/cut_videos if mounted, else "
                        "/home/farhan/Igus_rebel/outputs/cut_videos)")
    c.add_argument("--no-sidecar", action="store_true", help="write no sidecar folders")
    c.add_argument("--snapshot", default=cut_sidecar.DEFAULT_SNAPSHOT, help="seam snapshot copied into each sidecar folder")
    c.add_argument("--no-snapshot", action="store_true", help="do not copy a snapshot")
    c.add_argument("--keep-stopped", action="store_true",
                   help="save an execution that was STOPPED / failed / aborted (task suffixed ' [stopped: <reason>]') "
                        "instead of discarding it")
    c.add_argument("--trigger-topic", default=None,
                   help=f"status topic of the trigger (dual-cut: {cut_trigger.DEFAULT_TOPIC['dual-cut']}, "
                        f"h-cut: {cut_trigger.DEFAULT_TOPIC['h-cut']})")
    c.add_argument("--exec-report-topic", default=None,
                   help=f"[dual-cut] per-phase JSON report (default {cut_trigger.DEFAULT_REPORT_TOPIC})")
    g.add_argument("--other-arm-deg", nargs=6, type=float, default=[0.0] * 6, metavar="DEG",
                   help="constant pose of the undriven arm (state and action), default all 0")
    g.add_argument("--camera", action="append", metavar="KEY=TOPIC",
                   help="repeatable, any number of cameras; KEY becomes observation.images.KEY. Recommended keys: "
                        + ", ".join(RECOMMENDED_KEYS))
    g.add_argument("--camera-preset", choices=list(CAMERA_PRESETS), default=None,
                   help="cell = the 5 cell cameras (rebel_cameras cameras.launch.py), cell-left/cell-right = head + "
                        "that arm's gripper and 305 cameras, cut = head + both gripper cameras (the cameras that run by "
                        "default; for cutting). --camera KEY=TOPIC adds or overrides one, e.g. "
                        f"--camera knife={CELL_CAMERAS['knife']}")
    g.add_argument("--list-cameras", action="store_true", help="list image topics (resolution, encoding, rate), then exit")
    g.add_argument("--pick-cameras", action="store_true", help="list image topics, then ask a key for each to record")
    g.add_argument("--discover-s", type=float, default=3.0, help="listening time for --list/--pick-cameras")
    args = p.parse_args()
    if args.source == "isaac":
        args.source = "sim"
    if args.root is None:
        args.root = os.environ.get("ROBOT_REMOTE_DATASETS") or (
            cut_sidecar.default_dataset_root(dataset_check.DEFAULT_ROOT) if args.episode_trigger != "manual"
            else str(dataset_check.DEFAULT_ROOT))
    if args.camera_gap_s <= 0:
        p.error("--camera-gap-s must be > 0")
    if args.max_episode_s is None:
        args.max_episode_s = 600.0 if args.episode_trigger != "manual" else 120.0
    args.repo_id = args.repo_id or f"local/{args.name}"
    if args.source in ("mocap-real", "moveit-real"):
        run_mocap_main(args, p)
        return
    if args.camera or args.camera_preset or args.list_cameras or args.pick_cameras:
        p.error("--camera / --camera-preset / --list-cameras / --pick-cameras need --source mocap-real or moveit-real")
    if (args.episode_trigger != "manual" or args.keep_stopped or args.trigger_topic or args.exec_report_topic
            or args.headless or args.camera_domain is not None or args.clip_dir or args.camera_keys):
        p.error("--episode-trigger / --keep-stopped / --trigger-topic / --exec-report-topic need --source moveit-real")
    if args.action_topic or args.follower_status_topic or args.controller_state_topic:
        p.error("--action-topic / --follower-status-topic / --controller-state-topic need --source mocap-real or moveit-real")
    if not args.name or not args.task:
        p.error("--name and --task are required")

    cameras = {}
    if args.source == "real":
        import real_cameras

        cam_cfg = real_cameras.load_config(args.camera_config or real_cameras.CONFIG)
        args.cameras = list(cam_cfg) if args.cameras is None else args.cameras
        unknown = [k for k in args.cameras if k not in cam_cfg]
        if unknown:
            p.error(f"cameras {unknown} are not in the camera config (configured: {list(cam_cfg)})")
        print(f"[rec] opening cameras {args.cameras} ...", flush=True)
        cameras = real_cameras.open_cameras(args.cameras, cam_cfg)
        args.image_shapes = {k: cam.latest()[0].shape[:2] for k, cam in cameras.items()}
        for k, cam in cameras.items():
            print(f"[rec] camera {k}: {cam.device}  {args.image_shapes[k][1]}x{args.image_shapes[k][0]}", flush=True)
    else:
        args.cameras = list(CAMERAS) if args.cameras is None else args.cameras
        unknown = [k for k in args.cameras if k not in CAMERAS]
        if unknown:
            p.error(f"sim cameras are {list(CAMERAS)}, not {unknown}")
    if args.image_writer_threads is None:
        args.image_writer_threads = max(4, 2 * len(args.cameras)) if args.source == "real" else 4

    rclpy.init()
    node = Recorder(args, cameras)
    try:
        ds, root = make_dataset(args)
    except BaseException:
        for cam in cameras.values():
            cam.close()
        raise
    print(f"[rec] {len(NAMES)}-D state/action: {NAMES}", flush=True)
    if args.source == "real":
        print("[rec] REAL robot: wall-clock sampling, Y = discard the open episode (the teleop moves the arms home); "
              f"a camera without frames for {args.camera_gap_s:g} s ends the open episode (saved up to the drop)",
              flush=True)
    print("[rec] controls: Quest A = start / save episode, B = discard, Y (hold 1 s on the real robot) = reset "
          "(X = rotation frame) | keyboard: Enter = start/save, d = discard, q = quit", flush=True)
    threading.Thread(target=keyboard_thread, args=(node.events,), daemon=True).start()

    executor = rclpy.executors.SingleThreadedExecutor()
    spin_thread = None
    recording = False
    next_t = None
    n_frames = n_stale = n_late = 0
    # real cameras: frames wait a little so that the ones sampled after a camera dropped can still be cut off
    hold = FrameHold(ds, args.camera_gap_s + 0.1 if (node.real and cameras) else 0.0)
    SALVAGE_MIN_S = 1.0     # an episode cut short by a camera loss is kept only if it is at least this long

    def end_on_camera_loss(key, t_last):
        """A camera has had no frame for > --camera-gap-s inside an open episode: end the episode AT the drop. No frame
        after the drop is kept; what was recorded before it is saved if long enough, else discarded."""
        nonlocal recording, n_frames
        recording = False
        hold.drop_after(t_last + 1.0 / args.fps)  # a frame this close still shows a normally old image
        n_frames = hold.committed
        into, ep = max(0.0, t_last - ep_start), ds.meta.total_episodes
        cam = cameras.get(key)
        why = f" ({cam.error.split(' (')[0]})" if getattr(cam, "error", None) else ""
        keep = n_frames >= 2 and n_frames / args.fps >= SALVAGE_MIN_S
        print(f"[rec] ERROR: camera {key} lost {into:.1f} s into episode {ep}{why} - episode ended and "
              + (f"saved up to the drop ({n_frames} frames)" if keep else
                 f"DISCARDED (only {n_frames / args.fps:.1f} s recorded before the drop, minimum {SALVAGE_MIN_S:g} s)")
              + "; press A to start a new one once it is back", flush=True)
        if not keep:
            ds.clear_episode_buffer()
            hold.reset()
            print(f"[rec] ✕ discarded ({n_frames} frames) - camera {key} lost", flush=True)  # the GUI's idle trigger
            return
        print(f"[rec] saving episode {ep} ({n_frames} frames, {n_frames / args.fps:.1f}s {clock_name}, "
              f"{n_stale} frames had a stale stream" + (f", {n_late} late ticks" if n_late else "") + ") ...",
              flush=True)
        if save_episode(ds):
            print(f"[rec] ■ saved. total episodes: {ds.meta.total_episodes}", flush=True)
        hold.reset()

    clock_name = "real" if node.real else "sim"
    last_status = last_dbg = 0.0
    ep_start = None
    try:
        # ROS callbacks run in their own thread: one spin_once per loop handled ONE message, and with two igus drivers
        # at ~150 Hz plus the action stream the queue fell behind (joint_states 0.39 s old, ~8 frames/s, 2026-10-21)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        while rclpy.ok():
            time.sleep(0.001)
            # ---- a camera that dropped off USB ends the open episode (before anything else, so no frame follows)
            if recording and hold.hold_s > 0:
                lost = lost_camera(cameras, args.cameras, args.camera_gap_s, time.monotonic())
                if lost:
                    end_on_camera_loss(*lost)
                else:
                    hold.commit_due(time.monotonic())
            # ---- events
            while not node.events.empty():
                ev = node.events.get()
                if DEBUG:
                    print(f"[rec dbg] event {ev!r} at wall {time.monotonic():.2f} recording={recording}", flush=True)
                if ev == "toggle" and not recording:
                    frame, why = node.snapshot(starting=True)
                    if frame is None:
                        print(f"[rec] cannot start: {why}", flush=True)
                        continue
                    hold.reset()
                    recording, n_frames, n_stale, n_late = True, 0, 0, 0
                    next_t = ep_start = node.now()
                    print(f"[rec] ● RECORDING episode {ds.meta.total_episodes}  (task: {args.task})", flush=True)
                elif ev == "toggle" and recording:
                    recording = False
                    hold.flush()
                    n_frames = hold.committed
                    if n_frames < 2:
                        ds.clear_episode_buffer()
                        hold.reset()
                        print("[rec] episode too short - discarded", flush=True)
                        continue
                    print(f"[rec] saving episode {ds.meta.total_episodes} ({n_frames} frames, "
                          f"{n_frames / args.fps:.1f}s {clock_name}, {n_stale} frames had a stale stream"
                          + (f", {n_late} late ticks" if n_late else "") + ") ...", flush=True)
                    if save_episode(ds):
                        print(f"[rec] ■ saved. total episodes: {ds.meta.total_episodes}", flush=True)
                    hold.reset()
                elif ev == "reset":
                    if recording:
                        recording = False
                        ds.clear_episode_buffer()
                        hold.reset()
                        print(f"[rec] ✕ discarded ({n_frames} frames) - scene reset", flush=True)
                    else:
                        print("[rec] scene reset (no open episode)", flush=True)
                elif ev == "discard" and recording:
                    recording = False
                    ds.clear_episode_buffer()
                    hold.reset()
                    print(f"[rec] ✕ discarded ({n_frames} frames)", flush=True)
                elif ev == "quit":
                    raise KeyboardInterrupt
            # ---- sample on sim time (real: wall time)
            now = node.now()
            if DEBUG and recording and time.monotonic() - last_dbg > 1.0:
                last_dbg = time.monotonic()
                print(f"[rec dbg] clock={now} next_t={next_t} frames={n_frames}", flush=True)
            if not recording or now is None:
                continue
            if now + 1e-6 < next_t:
                continue
            if abs(now - next_t) > 1.0:
                # sim: /clock jumped (sim restarted, or a foreign publisher on the same ROS domain); real: this loop
                # was blocked. Never let that count as episode time - resync and warn.
                print(f"[rec] WARNING: " + (f"recorder blocked {now - next_t:.2f} s" if node.real else
                      f"/clock jumped {next_t:.2f} -> {now:.2f} s") + "; resyncing"
                      + ("" if node.real else " (another Isaac on this ROS domain? see ros_env.sh)"), flush=True)
                ep_start += now - next_t
                next_t = now
            next_t += 1.0 / args.fps
            if now > next_t:  # clock ran ahead of the sampling (sim stall / slow add_frame) - don't try to catch up
                next_t = now + 1.0 / args.fps
                n_late += 1
            frame, stale = node.snapshot()
            if frame is None:
                print(f"[rec] frame skipped: {stale}", flush=True)
                continue
            frame["task"] = args.task
            t_add = time.monotonic()
            hold.push(frame, time.monotonic())
            n_frames = hold.total()
            if DEBUG:
                print(f"[rec dbg] frame {n_frames} clock {now:.3f} add_frame {1000 * (time.monotonic() - t_add):.0f} ms", flush=True)
            if stale:
                n_stale += 1
            if time.monotonic() - last_status > 1.0:
                last_status = time.monotonic()
                print(f"[rec] ● {n_frames} frames ({n_frames / args.fps:.1f}s {clock_name})"
                      + (f"  stale: {', '.join(stale)}" if stale else "")
                      + (f"  late ticks: {n_late}" if n_late else ""), flush=True)
            if now - ep_start > args.max_episode_s:
                node.events.put("toggle")
    except KeyboardInterrupt:
        pass
    finally:
        if recording and hold.hold_s > 0:
            lost = lost_camera(cameras, args.cameras, args.camera_gap_s, time.monotonic())
            if lost:
                end_on_camera_loss(*lost)
        if recording:
            hold.flush()
            n_frames = hold.committed
        if recording and n_frames >= 2:
            print(f"[rec] saving the open episode ({n_frames} frames) before exit ...", flush=True)
            save_episode(ds)
        ds.finalize()
        print(f"[rec] finalized {root}  ({ds.meta.total_episodes} episodes)", flush=True)
        for cam in cameras.values():
            cam.close()
        executor.shutdown(timeout_sec=1.0)  # stop the callback thread before the node goes away
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
