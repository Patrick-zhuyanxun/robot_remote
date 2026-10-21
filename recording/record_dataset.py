#!/usr/bin/env python3
"""Record the dual-arm Quest teleop (Isaac sim or the real ReBeLs) into a LeRobotDataset (v3.0, HFRVLA lerobot).

Inputs (ROS 2, read through Isaac Sim's bundled Python-3.12 rclpy - see run_record.sh):
  /dual_arm/isaac_joint_states     -> observation.state (14): arm_{left,right}_joint1..6 + gripper closedness
  /quest_teleop/action             -> action            (14): same names, commanded by the teleop
  /dual_arm/{head,wrist_left,wrist_right}/rgb -> observation.images.{head,wrist_left,wrist_right}
  /quest_teleop/events             <- Quest buttons: "toggle" (A/X: start / save), "discard" (B), "reset" (Y: scene reset,
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
"""

import argparse
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import rclpy.executors

import dataset_check
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
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

    def real_snapshot(self):
        """Real source: (frame, stale) or (None, reason)."""
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

    def snapshot(self):
        """Latest frame if every stream is present, else (None, reason). Also returns staleness warnings."""
        if self.real:
            return self.real_snapshot()
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


def make_dataset(args):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(args.root).expanduser().resolve() / args.name
    features = {
        "observation.state": {"dtype": "float32", "shape": (len(NAMES),), "names": NAMES},
        "action": {"dtype": "float32", "shape": (len(NAMES),), "names": NAMES},
    }
    for k in args.cameras:
        h, w = args.image_shapes[k] if args.source == "real" else (args.height, args.width)
        features[f"observation.images.{k}"] = {"dtype": "video", "shape": (h, w, 3),
                                               "names": ["height", "width", "channels"]}
    st = dataset_check.inspect(root)
    if st["state"] == "dataset":
        check_resume(root / "meta" / "info.json", features, ROBOT_TYPE[args.source], args.fps)
        print(f"[rec] resuming existing dataset {root} ({st['episodes']} episodes)", flush=True)
        # image_writer_threads: resume() defaults to 0 = PNGs written inside add_frame; with 3 cameras at 640x480 that
        # capped recording at ~8 frames/s (every tick late, 2026-10-21). create() below gets the same threads.
        return LeRobotDataset.resume(repo_id=args.repo_id, root=root, vcodec=args.vcodec,
                                     image_writer_threads=args.image_writer_threads), root
    ok, why = dataset_check.describe(st)
    if not ok:
        raise SystemExit(f"[rec] ERROR: {root}: {why}")
    if st["state"] != "new":
        print(f"[rec] {why}", flush=True)
        dataset_check.prepare_new(st)
    print(f"[rec] creating dataset {root}", flush=True)
    ds = LeRobotDataset.create(repo_id=args.repo_id, fps=args.fps, features=features, root=root,
                               robot_type=ROBOT_TYPE[args.source], use_videos=True, vcodec=args.vcodec,
                               image_writer_threads=args.image_writer_threads)
    return ds, root


def check_resume(info_path, features, robot_type, fps):
    """Refuse to append to a dataset recorded with other cameras / sizes / fps or the other source (sim vs real)."""
    import json

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


def keyboard_thread(events: queue.Queue):
    for line in sys.stdin:
        c = line.strip().lower()
        events.put({"": "toggle", "d": "discard", "q": "quit"}.get(c, "toggle" if c == "s" else "unknown"))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", required=True, help="dataset folder name under --root")
    p.add_argument("--task", required=True, help="language instruction stored with every frame")
    p.add_argument("--root", default=os.environ.get("ROBOT_REMOTE_DATASETS", str(dataset_check.DEFAULT_ROOT)),
                   help="folder that holds the datasets (default: robot_remote/dataset)")
    p.add_argument("--repo-id", default=None, help="default: local/<name>")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--source", choices=["sim", "real"], default="sim",
                   help="sim = Isaac topics + /clock; real = igus drivers + USB cameras, wall clock")
    p.add_argument("--cameras", nargs="*", default=None,
                   help="subset to record (empty = state/action only). sim: head wrist_left wrist_right, must match "
                        "the cameras the sim publishes; real: keys of --camera-config (default: all of them)")
    p.add_argument("--camera-config", default=None, help="real: camera devices (default config/real_cameras.json)")
    p.add_argument("--image-writer-threads", type=int, default=None, help="default: 4, or 2 per camera on real")
    p.add_argument("--vcodec", default="libsvtav1")
    p.add_argument("--max-episode-s", type=float, default=120.0, help="auto-save after this much sim / wall time")
    args = p.parse_args()
    args.repo_id = args.repo_id or f"local/{args.name}"

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
        print("[rec] REAL robot: wall-clock sampling, Y = discard the open episode (the teleop moves the arms home)",
              flush=True)
    print("[rec] controls: Quest A/X = start / save episode, B = discard, Y = reset scene | keyboard: Enter = start/save, "
          "d = discard, q = quit", flush=True)
    threading.Thread(target=keyboard_thread, args=(node.events,), daemon=True).start()

    executor = rclpy.executors.SingleThreadedExecutor()
    spin_thread = None
    recording = False
    next_t = None
    n_frames = n_stale = n_late = 0
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
            # ---- events
            while not node.events.empty():
                ev = node.events.get()
                if DEBUG:
                    print(f"[rec dbg] event {ev!r} at wall {time.monotonic():.2f} recording={recording}", flush=True)
                if ev == "toggle" and not recording:
                    frame, why = node.snapshot()
                    if frame is None:
                        print(f"[rec] cannot start: {why}", flush=True)
                        continue
                    recording, n_frames, n_stale, n_late = True, 0, 0, 0
                    next_t = ep_start = node.now()
                    print(f"[rec] ● RECORDING episode {ds.meta.total_episodes}  (task: {args.task})", flush=True)
                elif ev == "toggle" and recording:
                    recording = False
                    if n_frames < 2:
                        ds.clear_episode_buffer()
                        print("[rec] episode too short - discarded", flush=True)
                        continue
                    print(f"[rec] saving episode {ds.meta.total_episodes} ({n_frames} frames, "
                          f"{n_frames / args.fps:.1f}s {clock_name}, {n_stale} frames had a stale stream"
                          + (f", {n_late} late ticks" if n_late else "") + ") ...", flush=True)
                    ds.save_episode(parallel_encoding=False)  # process-pool encoding deadlocks next to rclpy threads (fork)
                    print(f"[rec] ■ saved. total episodes: {ds.meta.total_episodes}", flush=True)
                elif ev == "reset":
                    if recording:
                        recording = False
                        ds.clear_episode_buffer()
                        print(f"[rec] ✕ discarded ({n_frames} frames) - scene reset", flush=True)
                    else:
                        print("[rec] scene reset (no open episode)", flush=True)
                elif ev == "discard" and recording:
                    recording = False
                    ds.clear_episode_buffer()
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
            ds.add_frame(frame)
            n_frames += 1
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
        if recording and n_frames >= 2:
            print(f"[rec] saving the open episode ({n_frames} frames) before exit ...", flush=True)
            ds.save_episode(parallel_encoding=False)
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
