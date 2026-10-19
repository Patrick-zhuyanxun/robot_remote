#!/usr/bin/env python3
"""Record the Isaac dual-arm Quest teleop into a LeRobotDataset (v3.0, written with the HFRVLA lerobot).

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
    def __init__(self, args):
        super().__init__("dataset_recorder")
        self.args = args
        self.state = self.action = None
        self.state_stamp = 0.0
        self.action_wall = 0.0
        self.images, self.image_stamp = {}, {}
        self.sim_time = None
        self.events: queue.Queue[str] = queue.Queue()
        self.create_subscription(JointState, "/dual_arm/isaac_joint_states", self.on_state, 10)
        self.create_subscription(JointState, "/quest_teleop/action", self.on_action, 10)
        self.create_subscription(String, "/quest_teleop/events", lambda m: self.events.put(m.data), 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        for key, topic in CAMERAS.items():
            if key in args.cameras:
                self.create_subscription(Image, topic, lambda m, k=key: self.on_image(k, m), qos_profile_sensor_data)

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
    if (root / "meta" / "info.json").exists():
        print(f"[rec] resuming existing dataset {root}", flush=True)
        return LeRobotDataset.resume(repo_id=args.repo_id, root=root, vcodec=args.vcodec), root
    features = {
        "observation.state": {"dtype": "float32", "shape": (len(NAMES),), "names": NAMES},
        "action": {"dtype": "float32", "shape": (len(NAMES),), "names": NAMES},
    }
    for k in args.cameras:
        features[f"observation.images.{k}"] = {"dtype": "video", "shape": (args.height, args.width, 3),
                                               "names": ["height", "width", "channels"]}
    print(f"[rec] creating dataset {root}", flush=True)
    ds = LeRobotDataset.create(repo_id=args.repo_id, fps=args.fps, features=features, root=root,
                               robot_type="igus_rebel_dual_arm_isaac", use_videos=True, vcodec=args.vcodec,
                               image_writer_threads=4)
    return ds, root


def keyboard_thread(events: queue.Queue):
    for line in sys.stdin:
        c = line.strip().lower()
        events.put({"": "toggle", "d": "discard", "q": "quit"}.get(c, "toggle" if c == "s" else "unknown"))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", required=True, help="dataset folder name under --root")
    p.add_argument("--task", required=True, help="language instruction stored with every frame")
    p.add_argument("--root", default=os.environ.get("ROBOT_REMOTE_DATASETS", str(Path(__file__).resolve().parents[1] / "datasets")))
    p.add_argument("--repo-id", default=None, help="default: local/<name>")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--cameras", nargs="*", choices=list(CAMERAS), default=list(CAMERAS),
                   help="subset to record; must match the cameras the sim publishes (empty = state/action only)")
    p.add_argument("--vcodec", default="libsvtav1")
    p.add_argument("--max-episode-s", type=float, default=120.0, help="auto-save after this much sim time")
    args = p.parse_args()
    args.repo_id = args.repo_id or f"local/{args.name}"

    rclpy.init()
    node = Recorder(args)
    ds, root = make_dataset(args)
    print(f"[rec] {len(NAMES)}-D state/action: {NAMES}", flush=True)
    print("[rec] controls: Quest A/X = start / save episode, B = discard, Y = reset scene | keyboard: Enter = start/save, "
          "d = discard, q = quit", flush=True)
    threading.Thread(target=keyboard_thread, args=(node.events,), daemon=True).start()

    recording = False
    next_t = None
    n_frames = n_stale = 0
    last_status = last_dbg = 0.0
    ep_start = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.002)
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
                    recording, n_frames, n_stale, next_t, ep_start = True, 0, 0, node.sim_time, node.sim_time
                    print(f"[rec] ● RECORDING episode {ds.meta.total_episodes}  (task: {args.task})", flush=True)
                elif ev == "toggle" and recording:
                    recording = False
                    if n_frames < 2:
                        ds.clear_episode_buffer()
                        print("[rec] episode too short - discarded", flush=True)
                        continue
                    print(f"[rec] saving episode {ds.meta.total_episodes} ({n_frames} frames, "
                          f"{n_frames / args.fps:.1f}s sim, {n_stale} frames had a stale stream) ...", flush=True)
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
            # ---- sample on sim time
            if DEBUG and recording and time.monotonic() - last_dbg > 1.0:
                last_dbg = time.monotonic()
                print(f"[rec dbg] sim_time={node.sim_time} next_t={next_t} frames={n_frames}", flush=True)
            if not recording or node.sim_time is None:
                continue
            if node.sim_time + 1e-6 < next_t:
                continue
            if abs(node.sim_time - next_t) > 1.0:
                # /clock jumped (sim restarted, or a foreign publisher on the same ROS domain): never let
                # that count as episode time - resync and warn.
                print(f"[rec] WARNING: /clock jumped {next_t:.2f} -> {node.sim_time:.2f} s; resyncing "
                      "(another Isaac on this ROS domain? see ros_env.sh)", flush=True)
                ep_start += node.sim_time - next_t
                next_t = node.sim_time
            next_t += 1.0 / args.fps
            if node.sim_time > next_t:  # sim stalled briefly - don't try to catch up
                next_t = node.sim_time + 1.0 / args.fps
            frame, stale = node.snapshot()
            if frame is None:
                print(f"[rec] frame skipped: {stale}", flush=True)
                continue
            frame["task"] = args.task
            t_add = time.monotonic()
            ds.add_frame(frame)
            n_frames += 1
            if DEBUG:
                print(f"[rec dbg] frame {n_frames} sim {node.sim_time:.3f} add_frame {1000 * (time.monotonic() - t_add):.0f} ms", flush=True)
            if stale:
                n_stale += 1
            if time.monotonic() - last_status > 1.0:
                last_status = time.monotonic()
                print(f"[rec] ● {n_frames} frames ({n_frames / args.fps:.1f}s sim)"
                      + (f"  stale: {', '.join(stale)}" if stale else ""), flush=True)
            if node.sim_time - ep_start > args.max_episode_s:
                node.events.put("toggle")
    except KeyboardInterrupt:
        pass
    finally:
        if recording and n_frames >= 2:
            print(f"[rec] saving the open episode ({n_frames} frames) before exit ...", flush=True)
            ds.save_episode(parallel_encoding=False)
        ds.finalize()
        print(f"[rec] finalized {root}  ({ds.meta.total_episodes} episodes)", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
