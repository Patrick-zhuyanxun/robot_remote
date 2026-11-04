#!/usr/bin/env python3
"""Raw recorder of CUTTING runs: one rosbag2 (sqlite3) per execution, FULL resolution, no conversion.

Started by the cutting launches of ~/igus_rebel_ros2 (cut_recorder_backend:=raw, the default); no LeRobot, no numpy, no
image decoding: plain system python3 + /opt/ros/humble (rosbag2_py). READ-ONLY: subscriptions only (raw / serialized), no
publishers, services or clients. The messages are stored byte for byte with their original type; the bag timestamp is the
receive time (wall = ROS system clock), the header stamps inside the messages are untouched.

Episode (same triggers as record_dataset.py, recording/cut_trigger.py)
  dual-cut    /dual_cut_coordinator/status + exec_report: one episode per ~/execute (all phases)
  h-cut       /box_h_cut_executor/t_cut_status: one per execute_t_cut sequence
  fjt-status  any FollowJointTrajectory goal on the single-arm controller, closes --fjt-hold-s after the last one
  --now       one episode from start-up to SIGINT / SIGTERM (testing)
  plus --pre-roll-s (ring buffer of the messages before the trigger) and --post-roll-s (tail after the end).
  Outcome finished -> kept; stopped / aborted -> KEPT TOO, marked in clip.json ("outcome", "stop_reason");
  --discard-stopped deletes the bag of such an episode instead.

Output: <clip-dir>/<YYYYmmdd_HHMMSS>/{clip.json, <snapshot copy>, bag/}  (the folder side_cut_vision's probe looks for:
clip.json has "recording": true while the episode is open, rewritten every 20 s) and one line per episode in
<clip-dir>/index.jsonl. Default clip-dir = tools/cut_recorder's: the external drive's igus_nvidia/cut_videos if mounted.

Topics (each only if it is published; cameras are CompressedImage as published, in --camera-domain when given)
  cameras: head knife wrist_{left,right}_knife wrist_{left,right}_gripper left_vision right_vision (--camera-keys)
  /joint_states /tf /tf_static (all static ones are written at bag open) the controller_state of both JTCs and of the
  single-arm controller, /dual_cut_coordinator/{status,exec_report,plan_report} (plan_report: the latest one is written at
  bag open too), /box_h_cut_executor/t_cut_status, the FJT action status topics, everything under /side_cut_vision/
  (measurement topics), --extra-topic TOPIC=TYPE.

Writer: subscriptions never wait for the disk (queue + writer thread). If the queue exceeds --queue-mb the OLDEST CAMERA
messages are dropped first (counted in clip.json); joint / status / tf messages are never dropped.
"""
import argparse
import collections
import json
import os
import shutil
import signal
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import cut_cameras  # noqa: E402
import cut_sidecar  # noqa: E402
import cut_trigger  # noqa: E402

REPLAY_GUARD_S = 1.5       # transient_local replays right after subscribing are not "live" messages
SIDECAR_TOPIC_PREFIXES = ("/side_cut_vision/",)
COMPRESSED = "sensor_msgs/msg/CompressedImage"


class Topic:
    def __init__(self, name, type_name, kind="live", qos="reliable", camera=False, domain="main", role=None):
        self.name, self.type, self.kind, self.qos, self.camera, self.domain, self.role = (
            name, type_name, kind, qos, camera, domain, role)


def fixed_topics(single_only=False):
    T = Topic
    topics = [
        T("/joint_states", "sensor_msgs/msg/JointState", qos="sensor"),
        T("/tf", "tf2_msgs/msg/TFMessage", qos="sensor"),
        T("/tf_static", "tf2_msgs/msg/TFMessage", kind="latched", qos="latched"),
        T("/dual_cut_coordinator/status", "dual_cut_msgs/msg/CoordinatorStatus", role="dual-status"),
        T("/dual_cut_coordinator/exec_report", "std_msgs/msg/String", role="dual-report"),
        T("/dual_cut_coordinator/plan_report", "std_msgs/msg/String", kind="latched", qos="latched"),
        T("/arm_left_trajectory_controller/controller_state", "control_msgs/msg/JointTrajectoryControllerState", qos="sensor"),
        T("/arm_right_trajectory_controller/controller_state", "control_msgs/msg/JointTrajectoryControllerState", qos="sensor"),
        T("/rebel_arm_trajectory_controller/controller_state", "control_msgs/msg/JointTrajectoryControllerState", qos="sensor"),
        T("/box_h_cut_executor/t_cut_status", "std_msgs/msg/String", role="hcut-status"),
    ]
    for ns in ("/rebel_arm_trajectory_controller", "/arm_left_trajectory_controller", "/arm_right_trajectory_controller"):
        topics.append(T(f"{ns}/follow_joint_trajectory/_action/status", "action_msgs/msg/GoalStatusArray", kind="latched",
                        qos="latched", role="fjt-status" if ns.startswith("/rebel") else None))
    return topics


class Episode:
    """One open (or closing) bag. Items are queued by the subscription callbacks, written by the writer thread."""

    def __init__(self, exec_id, trigger, sidecar, bag_dir, queue_limit, started_mono):
        self.exec_id, self.trigger, self.sidecar, self.bag_dir = exec_id, trigger, sidecar, bag_dir
        self.ctl, self.cam = collections.deque(), collections.deque()
        self.bytes, self.queue_limit = 0, queue_limit
        self.started_mono = started_mono
        self.closed = False            # no more input
        self.close_at = None           # monotonic time the post-roll ends
        self.outcome = self.reason = None
        self.exec_info = {}
        self.dropped = collections.Counter()   # topic -> camera messages dropped on a backed-up queue
        self.stats = {}                # topic -> {count, bytes, first, last, max_gap, type}
        self.writer = None
        self.error = None
        self.start_wall_ns = time.time_ns()
        self.written = 0

    def put(self, item):
        """item = (t_ns, Topic, bytes). Called under the recorder lock."""
        n = len(item[2])
        (self.cam if item[1].camera else self.ctl).append(item)
        self.bytes += n
        while self.bytes > self.queue_limit and self.cam:
            old = self.cam.popleft()
            self.bytes -= len(old[2])
            self.dropped[old[1].name] += 1


class RawCutRecorder:
    def __init__(self, args, cam_context=None):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        self.rclpy, self.args = rclpy, args
        self.lock = threading.Condition()
        self.ring = collections.deque()               # (t_ns, Topic, bytes), last pre-roll seconds
        self.latest = {}                              # latched topic -> (bytes, t_ns) newest replay / live message
        self.static = []                              # all /tf_static messages
        self.episodes = []                            # open or closing, oldest first
        self.events = collections.deque()
        self.t_sub = time.monotonic()
        self.msg_total = collections.Counter()        # since start
        self.seen_at_start = {}                       # camera key -> got a frame by --camera-discover-s
        self.cam_key = {}                             # camera topic -> key
        self.subscribed = {}                          # topic name -> Topic
        self.cam_context = cam_context
        self.node = Node("raw_cut_recorder", start_parameter_services=False)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.cam_node = self.cam_ex = None
        if cam_context is not None:
            self.cam_node = Node("raw_cut_recorder_cameras", context=cam_context, start_parameter_services=False)
            self.cam_ex = SingleThreadedExecutor(context=cam_context)
            self.cam_ex.add_node(self.cam_node)
        self.threads = [threading.Thread(target=self.ex.spin, daemon=True, name="ros_main")]
        if self.cam_ex is not None:
            self.threads.append(threading.Thread(target=self.cam_ex.spin, daemon=True, name="ros_cameras"))
        self.trigger = None
        self.trigger_seen = False
        self.trig_type = None
        self._qos = self._make_qos()
        self.writer_quit = False
        self.writer_thread = threading.Thread(target=self.writer_loop, daemon=True, name="bag_writer")

    # ---------------------------------------------------------------- QoS / subscriptions
    @staticmethod
    def _make_qos():
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
        return {
            "sensor": QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=100),
            "reliable": QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50),
            "latched": QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                  history=HistoryPolicy.KEEP_LAST, depth=20),
            "camera": qos_profile_sensor_data,
        }

    def subscribe(self, t):
        if t.name in self.subscribed:
            return
        from rosidl_runtime_py.utilities import get_message
        try:
            cls = get_message(t.type)
        except (ImportError, AttributeError, ValueError, LookupError) as e:
            if not hasattr(self, "_warned"):
                self._warned = set()
            if t.type not in self._warned:
                self._warned.add(t.type)
                print(f"[raw] message type {t.type} cannot be imported ({e}): {t.name} is not recorded", flush=True)
            return
        node = self.cam_node if (t.domain == "camera" and self.cam_node is not None) else self.node
        node.create_subscription(cls, t.name, lambda data, t=t: self.ingest(t, data), self._qos[t.qos], raw=True)
        self.subscribed[t.name] = t

    def setup_trigger(self):
        a = self.args
        if a.now or a.trigger is None:
            return
        kind = a.trigger
        self.trigger = cut_trigger.make_trigger(kind, lambda ev: self.events.append(ev), lambda s: print(s, flush=True),
                                                post_roll_s=a.fjt_hold_s)
        if kind == "dual-cut":
            from dual_cut_msgs.msg import CoordinatorStatus
            self.trig_type = CoordinatorStatus
        elif kind == "fjt-status":
            from action_msgs.msg import GoalStatusArray
            self.trig_type = GoalStatusArray
        # trigger inputs are fed from ingest() by topic name
        self.trig_topics = {"dual-cut": {a.trigger_topic: "status", a.exec_report_topic: "report"},
                            "h-cut": {a.trigger_topic: "status"}, "fjt-status": {a.trigger_topic: "status"}}[kind]

    def feed_trigger(self, t, data):
        role = self.trig_topics.get(t.name)
        if role is None:
            return
        if not self.trigger_seen:
            self.trigger_seen = True
            print(f"[raw] trigger input is flowing ({t.name})", flush=True)
        from rclpy.serialization import deserialize_message
        try:
            if self.args.trigger == "dual-cut":
                if role == "status":
                    self.trigger.on_status(deserialize_message(data, self.trig_type))
                else:
                    from std_msgs.msg import String
                    self.trigger.on_report(deserialize_message(data, String).data)
            elif self.args.trigger == "h-cut":
                from std_msgs.msg import String
                self.trigger.on_status(deserialize_message(data, String).data)
            else:
                self.trigger.on_status(deserialize_message(data, self.trig_type))
        except Exception as e:  # noqa: BLE001
            print(f"[raw] trigger input on {t.name} not understood: {type(e).__name__}: {e}", flush=True)

    # ---------------------------------------------------------------- ingest (callbacks of both executors)
    def ingest(self, t, data):
        now = time.time_ns()
        data = bytes(data)
        guard = t.kind == "latched" and time.monotonic() - self.t_sub < REPLAY_GUARD_S
        with self.lock:
            self.msg_total[t.name] += 1
            if t.camera:
                self.seen_at_start[self.cam_key.get(t.name, t.name)] = True
            if t.kind == "latched":
                if t.name == "/tf_static":
                    if data not in self.static:
                        self.static.append(data)
                else:
                    self.latest[t.name] = (data, now)
            if not guard:
                item = (now, t, data)
                self.ring.append(item)
                cutoff = now - int(self.args.pre_roll_s * 1e9)
                while self.ring and self.ring[0][0] < cutoff:
                    self.ring.popleft()
                for ep in self.episodes:
                    if not ep.closed:
                        ep.put(item)
                self.lock.notify()
        if self.trigger is not None:
            self.feed_trigger(t, data)

    # ---------------------------------------------------------------- episodes
    def open_episode(self, exec_id, why):
        a = self.args
        free_gb = shutil.disk_usage(a.clip_dir if os.path.isdir(a.clip_dir) else os.path.dirname(a.clip_dir) or "/").free / 1e9
        if free_gb < a.min_free_gb:
            print(f"[raw] NOT recording execution {exec_id}: only {free_gb:.1f} GB free at {a.clip_dir} "
                  f"(--min-free-gb {a.min_free_gb:g})", flush=True)
            return None
        info = {"backend": "raw_rosbag2", "trigger": a.trigger or "now", "exec_id": exec_id, "why": why,
                "pre_roll_s": a.pre_roll_s, "post_roll_s": a.post_roll_s, "bag": "bag",
                "cameras": {k: {"topic": t, "present_at_start": bool(self.seen_at_start.get(k)),
                                "domain": "robot" if k in cut_cameras.MAIN_DOMAIN_CAMERAS or self.cam_context is None
                                else f"camera ({a.camera_domain})"} for k, t in self.camera_topics.items()}}
        sidecar = cut_sidecar.Sidecar(a.clip_dir, info, None if a.no_snapshot else a.snapshot, log=lambda s: print(s, flush=True))
        if sidecar.dir is None:
            print("[raw] cannot create the episode folder: episode NOT recorded", flush=True)
            return None
        ep = Episode(exec_id, a.trigger or "now", sidecar, os.path.join(sidecar.dir, "bag"),
                     int(a.queue_mb * 1e6), time.monotonic())
        with self.lock:
            now = time.time_ns()
            for item in list(self.ring):                       # pre-roll
                ep.put(item)
            for raw in self.static:                            # latched context at bag open
                ep.put((now, self.subscribed["/tf_static"], raw))
            in_ring = {it[1].name for it in self.ring}
            for name, (raw, _) in self.latest.items():
                if name not in in_ring and name != "/tf_static":
                    ep.put((now, self.subscribed[name], raw))
            self.episodes.append(ep)
            self.lock.notify()
        print(f"[raw] ● RECORDING execution {exec_id} -> {ep.bag_dir}  (pre-roll {a.pre_roll_s:g}s, {len(ep.ctl) + len(ep.cam)} "
              "messages queued)", flush=True)
        return ep

    def end_episode(self, ep, outcome, reason, exec_info, immediate=False):
        ep.outcome, ep.reason, ep.exec_info = outcome, reason, exec_info or {}
        ep.close_at = time.monotonic() + (0.0 if immediate else self.args.post_roll_s)

    def close_due(self, force=False):
        now = time.monotonic()
        for ep in list(self.episodes):
            if ep.closed:
                continue
            if ep.close_at is not None and (force or now >= ep.close_at):
                with self.lock:
                    ep.closed = True
                    self.lock.notify()
                print(f"[raw] ■ execution {ep.exec_id} over ({ep.outcome}): closing the bag", flush=True)

    # ---------------------------------------------------------------- writer thread
    def writer_loop(self):
        while True:
            work = None
            with self.lock:
                while work is None:
                    for ep in self.episodes:
                        if ep.ctl or ep.cam:
                            batch = []
                            for _ in range(1 if self.args.debug_writer_delay_ms else 64):
                                q = ep.ctl or ep.cam
                                if not q:
                                    break
                                it = q.popleft()
                                ep.bytes -= len(it[2])
                                batch.append(it)
                            work = (ep, batch, False)
                            break
                        if ep.closed:
                            work = (ep, [], True)
                            break
                    if work is None:
                        if self.writer_quit and not self.episodes:
                            return
                        self.lock.wait(0.1)
            ep, batch, finish = work
            if batch and self.args.debug_writer_delay_ms:
                time.sleep(self.args.debug_writer_delay_ms / 1000.0)
            try:
                self.write_batch(ep, batch)
            except Exception as e:  # noqa: BLE001
                ep.error = f"{type(e).__name__}: {e}"
                print(f"[raw] ERROR writing {ep.bag_dir}: {ep.error}", flush=True)
                with self.lock:
                    ep.ctl.clear()
                    ep.cam.clear()
                    ep.bytes = 0
            if finish:
                self.finalize(ep)
                with self.lock:
                    self.episodes.remove(ep)

    def write_batch(self, ep, batch):
        import rosbag2_py
        if not batch:
            return
        if ep.writer is None:
            if ep.error:
                return
            ep.writer = rosbag2_py.SequentialWriter()
            ep.writer.open(rosbag2_py.StorageOptions(uri=ep.bag_dir, storage_id="sqlite3"),
                           rosbag2_py.ConverterOptions("cdr", "cdr"))
            ep.created = set()
        for t_ns, t, data in batch:
            if t.name not in ep.created:
                ep.writer.create_topic(rosbag2_py.TopicMetadata(name=t.name, type=t.type, serialization_format="cdr"))
                ep.created.add(t.name)
            ep.writer.write(t.name, data, t_ns)
            st = ep.stats.setdefault(t.name, {"count": 0, "bytes": 0, "first": t_ns, "last": t_ns, "max_gap": 0, "type": t.type})
            if st["count"]:
                st["max_gap"] = max(st["max_gap"], t_ns - st["last"])
            st["count"] += 1
            st["bytes"] += len(data)
            st["first"], st["last"] = min(st["first"], t_ns), max(st["last"], t_ns)
            ep.written += 1

    def finalize(self, ep):
        a = self.args
        ep.writer = None            # closes the bag, writes metadata.yaml
        topics = {}
        for name, st in sorted(ep.stats.items()):
            span = (st["last"] - st["first"]) / 1e9
            topics[name] = {"type": st["type"], "messages": st["count"], "bytes": st["bytes"],
                            "rate_hz": round((st["count"] - 1) / span, 2) if st["count"] > 1 and span > 0 else None,
                            "max_gap_s": round(st["max_gap"] / 1e9, 3)}
        cams = {}
        for k, topic in self.camera_topics.items():
            n = ep.stats.get(topic, {}).get("count", 0)
            cams[k] = {"topic": topic, "messages": n, "recorded": n > 0, "dropped_by_full_queue": ep.dropped.get(topic, 0)}
        keep = not (a.discard_stopped and ep.outcome in ("stopped", "aborted"))
        if not keep:
            shutil.rmtree(ep.bag_dir, ignore_errors=True)
        info = dict(ep.exec_info)
        final = {"outcome": ep.outcome, "reason": ep.reason, "kept": keep, "stop_reason": info.pop("stop_reason", ""),
                 "messages_written": ep.written, "topics": topics, "camera_status": cams,
                 "dropped_camera_messages": dict(ep.dropped), "writer_error": ep.error,
                 "wall_s": round(time.monotonic() - ep.started_mono, 2), **info}
        ep.sidecar.finish(**final)
        err = cut_sidecar.append_index(a.clip_dir, {
            "folder": os.path.basename(ep.sidecar.dir or ""), "path": ep.sidecar.dir, "bag": ep.bag_dir if keep else None,
            "start_local": cut_sidecar.local_iso(ep.start_wall_ns / 1e9), "end_local": cut_sidecar.local_iso(),
            "outcome": ep.outcome, "reason": ep.reason, "kept": keep, "trigger": ep.trigger, "exec_id": ep.exec_id,
            "messages": ep.written, "topics": {k: v["messages"] for k, v in topics.items()}})
        if err:
            print(f"[raw] WARNING: index.jsonl not written: {err}", flush=True)
        drops = sum(ep.dropped.values())
        print(f"[raw] saved {ep.sidecar.dir}: {ep.written} messages, {len(topics)} topics, outcome {ep.outcome}"
              + (f", {drops} camera messages DROPPED (slow disk)" if drops else "") + ("" if keep else " (bag deleted: --discard-stopped)"),
              flush=True)

    # ---------------------------------------------------------------- run
    def run(self, stop):
        a = self.args
        self.camera_topics = dict(self.cameras)
        fixed = fixed_topics()
        for t in fixed:
            self.subscribe(t)
        if self.trigger is not None:     # a --trigger-topic / --exec-report-topic that is not one of the usual ones
            types = {"dual-cut": "dual_cut_msgs/msg/CoordinatorStatus", "h-cut": "std_msgs/msg/String",
                     "fjt-status": "action_msgs/msg/GoalStatusArray"}
            for name, role in self.trig_topics.items():
                self.subscribe(Topic(name, "std_msgs/msg/String" if role == "report" else types[a.trigger],
                                     kind="latched" if a.trigger == "fjt-status" else "live",
                                     qos="latched" if a.trigger == "fjt-status" else "reliable"))
        for k, topic in self.camera_topics.items():
            dom = "main" if k in cut_cameras.MAIN_DOMAIN_CAMERAS else "camera"
            self.cam_key[topic] = k
            self.subscribe(Topic(topic, COMPRESSED, qos="camera", camera=True, domain=dom))
        for spec in a.extra_topic:
            name, _, typ = spec.partition("=")
            self.subscribe(Topic(name, typ))
        for spec in a.extra_camera:
            name, _, typ = spec.partition("=")
            self.cam_key[name] = name
            self.camera_topics[name] = name
            self.subscribe(Topic(name, typ or COMPRESSED, qos="camera", camera=True, domain="camera"))
        for th in self.threads:
            th.start()
        self.writer_thread.start()
        t_discover, next_graph, active, started_now = time.monotonic() + a.camera_discover_s, 0.0, None, False
        reported = False
        try:
            while not stop.is_set() and self.rclpy.ok():
                time.sleep(0.02)
                now = time.monotonic()
                if a.now and not started_now:
                    started_now = True
                    active = self.open_episode(1, "--now")
                    if active:
                        active.outcome, active.reason = "recording", ""
                if self.trigger is not None:
                    self.trigger.poll()
                while self.events:
                    ev = self.events.popleft()
                    if ev[0] == "start":
                        if active is not None and not active.closed:
                            if active.close_at is None:
                                print(f"[raw] execution {ev[1]} started but execution {active.exec_id} is still open - "
                                      "not starting another", flush=True)
                                continue
                            self.close_due(force=True)           # back-to-back: cut the previous bag now
                        active = self.open_episode(ev[1], "trigger")
                    elif ev[0] == "end":
                        _, eid, outcome, reason = ev
                        info = self.trigger.ended_info.pop(eid, {})
                        if active is not None and active.exec_id == eid and active.close_at is None:
                            self.end_episode(active, outcome, reason, info)
                        else:
                            print(f"[raw] execution {eid} ended ({outcome}) with no open episode", flush=True)
                self.close_due()
                for ep in self.episodes:
                    if not ep.closed:
                        ep.sidecar.refresh(messages_queued_or_written=ep.written,
                                           queue_mb=round(ep.bytes / 1e6, 1))
                        if now - ep.started_mono > a.max_episode_s and ep.close_at is None:
                            print(f"[raw] --max-episode-s {a.max_episode_s:g} reached: closing the bag", flush=True)
                            self.end_episode(ep, "max_episode", "maximum episode length", {}, immediate=True)
                if now >= t_discover and not reported:
                    reported = True
                    got = sorted(k for k in self.camera_topics if self.seen_at_start.get(k))
                    miss = sorted(k for k in self.camera_topics if not self.seen_at_start.get(k))
                    print(f"[raw] cameras delivering after {a.camera_discover_s:g}s: {got or 'none'}; "
                          f"NOT delivering (absent from the bags until they appear): {miss or 'none'}", flush=True)
                if now >= next_graph:
                    next_graph = now + 2.0
                    self.discover_prefix_topics()
        finally:
            for ep in self.episodes:
                if ep.close_at is None:
                    self.end_episode(ep, "shutdown", "recorder stopped", {}, immediate=True)
            self.close_due(force=True)
            with self.lock:
                self.writer_quit = True
                self.lock.notify_all()
            self.writer_thread.join(timeout=a.shutdown_timeout_s)
            if self.writer_thread.is_alive():
                print("[raw] WARNING: writer still busy at shutdown timeout", flush=True)

    def discover_prefix_topics(self):
        for name, types in self.node.get_topic_names_and_types():
            if name in self.subscribed or not any(name.startswith(p) for p in self.args.topic_prefix):
                continue
            if name in self.camera_topics.values():
                continue
            if types:
                self.subscribe(Topic(name, types[0]))
                print(f"[raw] also recording {name} ({types[0]})", flush=True)

    def close(self):
        for ex in (self.ex, self.cam_ex):
            if ex is not None:
                ex.shutdown(timeout_sec=1.0)
        for th in self.threads:
            th.join(timeout=2.0)
        self.node.destroy_node()
        if self.cam_node is not None:
            self.cam_node.destroy_node()


def build_parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--trigger", choices=("dual-cut", "h-cut", "fjt-status"), default=None)
    p.add_argument("--now", action="store_true", help="one episode from start-up until SIGINT / SIGTERM")
    p.add_argument("--clip-dir", default=None, help="episode folders + index.jsonl (default: the external drive's "
                   "igus_nvidia/cut_videos if mounted, else /home/farhan/Igus_rebel/outputs/cut_videos)")
    p.add_argument("--camera-keys", default=",".join(cut_cameras.CELL_CAMERAS), help="default: all keys")
    p.add_argument("--camera", action="append", default=[], metavar="KEY=TOPIC", help="define / override a camera")
    p.add_argument("--camera-domain", type=int, default=None, metavar="N",
                   help="camera topics in a second rclpy context in ROS domain N (the vision overlays stay in the robot domain)")
    p.add_argument("--camera-discover-s", type=float, default=10.0, help="log which cameras deliver after this long")
    p.add_argument("--pre-roll-s", type=float, default=2.0)
    p.add_argument("--post-roll-s", type=float, default=2.0)
    p.add_argument("--fjt-hold-s", type=float, default=5.0, help="[fjt-status] episode ends this long after the last active goal")
    p.add_argument("--queue-mb", type=float, default=512.0, help="writer queue limit per episode; above it the oldest camera messages are dropped")
    p.add_argument("--min-free-gb", type=float, default=5.0, help="do not start an episode with less free space")
    p.add_argument("--max-episode-s", type=float, default=1800.0)
    p.add_argument("--discard-stopped", action="store_true", help="delete the bag of a stopped / aborted execution (default: keep)")
    p.add_argument("--trigger-topic", default=None)
    p.add_argument("--exec-report-topic", default=cut_trigger.DEFAULT_REPORT_TOPIC)
    p.add_argument("--extra-topic", action="append", default=[], metavar="TOPIC=TYPE", help="record another robot-domain topic")
    p.add_argument("--extra-camera", action="append", default=[], metavar="TOPIC=TYPE", help="record another camera-domain topic")
    p.add_argument("--topic-prefix", action="append", default=list(SIDECAR_TOPIC_PREFIXES),
                   help="record every topic below this prefix once it appears (default /side_cut_vision/)")
    p.add_argument("--snapshot", default=cut_sidecar.DEFAULT_SNAPSHOT)
    p.add_argument("--no-snapshot", action="store_true")
    p.add_argument("--shutdown-timeout-s", type=float, default=60.0)
    p.add_argument("--debug-writer-delay-ms", type=float, default=0.0, help=argparse.SUPPRESS)   # tests: a slow disk
    return p


def main(argv=None):
    p = build_parser()
    a = p.parse_args(argv)
    if not a.now and a.trigger is None:
        p.error("--trigger dual-cut | h-cut | fjt-status (or --now)")
    a.trigger_topic = a.trigger_topic or (cut_trigger.DEFAULT_TOPIC[a.trigger] if a.trigger else None)
    if a.camera_domain is not None and not 0 <= a.camera_domain <= 232:
        p.error("--camera-domain must be 0..232 (Fast DDS)")
    cameras = {}
    for k in [x.strip() for x in a.camera_keys.split(",") if x.strip()]:
        if k not in cut_cameras.CELL_CAMERAS:
            p.error(f"unknown camera key {k!r}; known: {', '.join(cut_cameras.CELL_CAMERAS)}")
        cameras[k] = cut_cameras.CELL_CAMERAS[k]
    for spec in a.camera:
        k, sep, topic = spec.partition("=")
        if not sep or not topic or not k.replace("_", "").isalnum():
            p.error(f"--camera expects KEY=TOPIC, got {spec!r}")
        cameras[k] = topic
    for spec in a.extra_topic + a.extra_camera:
        if "=" not in spec:
            p.error(f"expects TOPIC=TYPE, got {spec!r}")
    explicit_dir = a.clip_dir is not None
    a.clip_dir = a.clip_dir or cut_sidecar.default_clip_dir()
    if not cut_sidecar.drive_writable():
        print("[raw] " + "!" * 70 + f"\n[raw] WARNING: the external drive {cut_sidecar.EXTERNAL_DRIVE} is NOT mounted / writable.\n"
              f"[raw] The bags (~15-20 MB/s with all cameras) go to {a.clip_dir}, and / is nearly full.\n[raw] " + "!" * 70, flush=True)
    try:
        os.makedirs(a.clip_dir, exist_ok=True)
        free = shutil.disk_usage(a.clip_dir).free / 1e9
        print(f"[raw] clip dir {a.clip_dir} ({free:.0f} GB free)" + ("" if explicit_dir else " [default]"), flush=True)
    except OSError as e:
        print(f"[raw] ERROR: clip dir not writable: {e}", file=sys.stderr)
        return 2
    try:
        import rosbag2_py  # noqa: F401
    except ImportError as e:
        print(f"[raw] ERROR: rosbag2_py not importable (source /opt/ros/humble): {e}", file=sys.stderr)
        return 2

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    cam_ctx = None
    rec = None
    try:
        main_domain = rclpy.get_default_context().get_domain_id()
        if a.camera_domain is not None and a.camera_domain != main_domain:
            from rclpy.context import Context
            cam_ctx = Context()
            rclpy.init(args=[], context=cam_ctx, domain_id=a.camera_domain, signal_handler_options=SignalHandlerOptions.NO)
            print(f"[raw] camera topics are read in ROS domain {a.camera_domain} (second context); status, joint states, tf "
                  f"and controller state in domain {main_domain}", flush=True)
        rec = RawCutRecorder(a, cam_ctx)
        rec.cameras = cameras
        try:
            rec.setup_trigger()
        except ImportError as e:
            print(f"[raw] ERROR: {e} (dual-cut needs dual_cut_msgs: source ~/igus_rebel_ros2/install/setup.bash)",
                  file=sys.stderr)
            return 2
        print("[raw] READ-ONLY (subscribes only), raw rosbag2 (sqlite3), full resolution. "
              + (f"trigger {a.trigger} on {a.trigger_topic}" if a.trigger else "--now"), flush=True)
        print(f"[raw] cameras: {cameras}", flush=True)
        rec.run(stop)
    finally:
        if rec is not None:
            rec.close()
        if cam_ctx is not None:
            cam_ctx.try_shutdown()
        rclpy.try_shutdown()
    print("[raw] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
