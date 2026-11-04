"""Raw cut recorder (recording/record_raw_cut.py): one rosbag2 per execution, full resolution, no LeRobot.

Fake publishers on the private test domain 97 (robot) and 98 (cameras), tests/ros_isolation.py; nothing touches a robot.
Dual-cut tests need dual_cut_msgs (source ~/igus_rebel_ros2/install/setup.bash).
"""
import json
import os
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402

REC = Path(__file__).resolve().parents[1] / "recording"
RAW = REC / "record_raw_cut.py"
sys.path.insert(0, str(REC))

try:
    import rclpy
    import rosbag2_py
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import CompressedImage, JointState
    from test_record_cut import (CAM_TOPICS, FakeCutGraph, FJT_STATUS, HAVE_DUAL_MSGS, HAVE_ROS, EXECUTING, PLANNED, STOPPED,
                                 VISION_TOPIC, WAITING, jpeg)
    from test_record_cut_auto import CAMERA_DOMAIN, CamerasInAnotherDomain
except ImportError:
    HAVE_ROS = HAVE_DUAL_MSGS = False

EXPECTED_JPEG = None


def expected_jpeg():
    return bytes(jpeg((50, 100, 200)).data)


def read_bag(path):
    """-> ({topic: type}, {topic: [(t_ns, bytes)]})"""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    msgs = {}
    while r.has_next():
        topic, data, t = r.read_next()
        msgs.setdefault(topic, []).append((t, data))
    return types, msgs


class RawProc:
    def __init__(self, *argv):
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        self.p = subprocess.Popen([sys.executable, str(RAW), *argv], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  text=True, env=env)
        self.lines = []
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.p.stdout:
            self.lines.append(line)

    @property
    def text(self):
        return "".join(self.lines)

    def wait_for(self, text, timeout=40):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if text in self.text:
                return True
            if self.p.poll() is not None:
                return text in self.text
            time.sleep(0.05)
        return False

    def stop(self, sig=signal.SIGTERM, timeout=30):
        if self.p.poll() is None:
            self.p.send_signal(sig)
        return self.p.wait(timeout)


class ExtraPublishers:
    """/tf, /tf_static (latched), /dual_cut_coordinator/plan_report (latched), a side_cut_vision measurement topic."""

    def start(self):
        latched = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node = n = Node("fake_extra_pubs")
        self.tf = n.create_publisher(TFMessage, "/tf", 10)
        tfs = n.create_publisher(TFMessage, "/tf_static", latched)
        pr = n.create_publisher(String, "/dual_cut_coordinator/plan_report", latched)
        self.meas = n.create_publisher(String, "/side_cut_vision/arm_left/measurement", 5)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "world", "camera_link"
        tfs.publish(TFMessage(transforms=[t]))
        pr.publish(String(data='{"plan": "retained"}'))
        n.create_timer(0.05, lambda: self.tf.publish(TFMessage(transforms=[TransformStamped()])))
        n.create_timer(0.5, lambda: self.meas.publish(String(data='{"m": 1}')))
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(n)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def stop(self):
        self.ex.shutdown()
        self.node.destroy_node()


def run_dual(g, hold=0.8, stop_in_a=None):
    g.set(WAITING, "A")
    time.sleep(0.2)
    g.set(EXECUTING, "A")
    time.sleep(hold)
    if stop_in_a:
        g.set(STOPPED, "A", True, stop_in_a)
        return
    g.phase_report("A")
    g.set(WAITING, "B")
    time.sleep(0.2)
    g.set(EXECUTING, "B")
    time.sleep(hold)
    g.phase_report("B")
    g.set(PLANNED)


class RawBase(unittest.TestCase):
    DUAL = True
    GRAPH_KW = dict(cams=False, vision=True)
    CAMERAS_IN_98 = True

    @classmethod
    def setUpClass(cls):
        cls.graph = FakeCutGraph(cls.DUAL, **cls.GRAPH_KW)
        cls.graph.start()
        cls.extra = ExtraPublishers()
        cls.extra.start()
        cls.cams = CamerasInAnotherDomain(CAM_TOPICS) if cls.CAMERAS_IN_98 else None

    @classmethod
    def tearDownClass(cls):
        if cls.cams:
            cls.cams.stop()
        cls.extra.stop()
        cls.graph.stop()

    def setUp(self):
        if self.cams:
            for t in CAM_TOPICS:
                self.cams.on[t] = True

    def tearDown(self):
        if self.DUAL:
            self.graph.set(0)

    def start(self, root, *extra):
        argv = ["--clip-dir", str(Path(root) / "clips"), "--camera-domain", CAMERA_DOMAIN, "--camera-discover-s", "2",
                "--pre-roll-s", "1", "--post-roll-s", "1", *extra]
        if "--snapshot" not in extra:
            argv.append("--no-snapshot")
        r = RawProc(*argv)
        self.addCleanup(lambda: (r.p.kill() if r.p.poll() is None else None, r.p.wait()))
        self.assertTrue(r.wait_for("READ-ONLY"), r.text)
        self.assertTrue(r.wait_for("trigger input is flowing", timeout=30), r.text)   # the idle status was seen
        self.assertTrue(r.wait_for("NOT delivering", timeout=15), r.text)           # camera discovery logged
        time.sleep(0.5)
        return r

    def episodes(self, root):
        d = Path(root) / "clips"
        return sorted(p for p in d.iterdir() if p.is_dir())


@unittest.skipUnless(HAVE_ROS and HAVE_DUAL_MSGS, "needs rclpy, rosbag2_py and dual_cut_msgs (source the igus install)")
class DualRawTests(RawBase):
    def test_finished_execution_bag_has_everything_at_full_resolution(self):
        with tempfile.TemporaryDirectory() as root:
            snap = Path(root) / "seam.json"
            snap.write_text('{"s": 1}')
            r = self.start(root, "--trigger", "dual-cut", "--snapshot", str(snap))
            r.lines.clear()
            t_start = time.time()
            run_dual(self.graph)
            t_end = time.time()
            self.assertTrue(r.wait_for("saved ", timeout=30), r.text)
            self.assertEqual(r.stop(), 0, r.text)
            (ep,) = self.episodes(root)
            self.assertRegex(ep.name, r"^\d{8}_\d{6}$")
            doc = json.loads((ep / "clip.json").read_text())
            self.assertFalse(doc["recording"])
            self.assertEqual((doc["outcome"], doc["kept"], doc["trigger"], doc["bag"]), ("finished", True, "dual-cut", "bag"))
            self.assertEqual(doc["snapshot"], "seam.json")
            self.assertTrue((ep / "seam.json").is_file())
            self.assertEqual({d["phase"] for d in doc["exec_reports"]}, {"A", "B"})
            types, msgs = read_bag(ep / "bag")
            want = {"/joint_states": "sensor_msgs/msg/JointState",
                    "/arm_left_trajectory_controller/controller_state": "control_msgs/msg/JointTrajectoryControllerState",
                    "/arm_right_trajectory_controller/controller_state": "control_msgs/msg/JointTrajectoryControllerState",
                    "/dual_cut_coordinator/status": "dual_cut_msgs/msg/CoordinatorStatus",
                    "/dual_cut_coordinator/exec_report": "std_msgs/msg/String",
                    "/dual_cut_coordinator/plan_report": "std_msgs/msg/String",
                    "/tf": "tf2_msgs/msg/TFMessage", "/tf_static": "tf2_msgs/msg/TFMessage",
                    "/camera/color/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
                    "/arm_left/gripper_camera/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
                    "/arm_right/gripper_camera/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
                    VISION_TOPIC: "sensor_msgs/msg/CompressedImage",
                    "/side_cut_vision/arm_left/measurement": "std_msgs/msg/String"}
            for topic, typ in want.items():
                self.assertEqual(types.get(topic), typ, f"{topic}: {sorted(types)}")
            # camera bytes are exactly what was published (no decode / resize / re-encode), two domains in one bag
            exp = expected_jpeg()
            for topic in CAM_TOPICS:
                self.assertGreater(len(msgs[topic]), 20, topic)
                for _, raw in msgs[topic][:: max(1, len(msgs[topic]) // 10)]:
                    self.assertEqual(bytes(deserialize_message(raw, CompressedImage).data), exp)
            # counts: clip.json == bag; rates plausible (joint_states/controller_state published at 50 Hz)
            for topic, info in doc["topics"].items():
                self.assertEqual(info["messages"], len(msgs[topic]), topic)
            span = (msgs["/joint_states"][-1][0] - msgs["/joint_states"][0][0]) / 1e9
            self.assertAlmostEqual(len(msgs["/joint_states"]) / span, 50.0, delta=8.0)
            self.assertEqual(len(msgs["/tf_static"]), 1)                 # latched context at bag open
            self.assertEqual(len(msgs["/dual_cut_coordinator/plan_report"]), 1)
            # pre-roll / post-roll: first camera frame before the first WAITING status, last joint state after PLANNED
            st = [(t, deserialize_message(d, __import__("dual_cut_msgs.msg", fromlist=["x"]).CoordinatorStatus))
                  for t, d in msgs["/dual_cut_coordinator/status"]]
            t_wait = next(t for t, m in st if m.state == WAITING)
            t_planned = min(t for t, m in st if m.state == PLANNED)
            first_cam = min(msgs[c][0][0] for c in CAM_TOPICS)
            self.assertGreater((t_wait - first_cam) / 1e9, 0.6, "pre-roll")
            self.assertGreater((msgs["/joint_states"][-1][0] - t_planned) / 1e9, 0.6, "post-roll")
            # nothing from before the pre-roll (the replay of a latched topic is stamped at bag open, not earlier)
            self.assertGreater(first_cam, int((t_start - 0.2) * 1e9) - 2_000_000_000)
            # index for the converter
            lines = (Path(root) / "clips" / "index.jsonl").read_text().splitlines()
            self.assertEqual(len(lines), 1)
            rec = json.loads(lines[0])
            self.assertEqual((rec["folder"], rec["outcome"], rec["kept"], rec["trigger"]), (ep.name, "finished", True, "dual-cut"))
            self.assertEqual(rec["topics"]["/joint_states"], len(msgs["/joint_states"]))
            self.assertTrue(rec["bag"].endswith("/bag"))

    def test_sidecar_lifecycle_while_open_and_read_only_graph(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.start(root, "--trigger", "dual-cut")
            threading.Thread(target=run_dual, args=(self.graph,), daemon=True).start()
            self.assertTrue(r.wait_for("RECORDING", timeout=20), r.text)
            time.sleep(0.6)
            (ep,) = self.episodes(root)
            doc = json.loads((ep / "clip.json").read_text())
            self.assertTrue(doc["recording"])
            self.assertEqual((doc["exec_id"], doc["backend"]), (1, "raw_rosbag2"))
            self.assertEqual(set(doc["cameras"]), {"head", "knife", "wrist_left_knife", "wrist_right_knife",
                                                   "wrist_left_gripper", "wrist_right_gripper", "left_vision", "right_vision"})
            self.assertTrue(doc["cameras"]["head"]["present_at_start"])
            self.assertFalse(doc["cameras"]["knife"]["present_at_start"])
            node = self.graph.node                                  # robot-domain graph view of the recorder
            pubs = {t for t, _ in node.get_publisher_names_and_types_by_node("raw_cut_recorder", "/")}
            self.assertLessEqual(pubs, {"/rosout", "/parameter_events"}, pubs)
            self.assertEqual(node.get_service_names_and_types_by_node("raw_cut_recorder", "/"), [])
            self.assertEqual(node.get_client_names_and_types_by_node("raw_cut_recorder", "/"), [])
            self.assertTrue(r.wait_for("saved ", timeout=30), r.text)
            self.assertEqual(r.stop(), 0)
            self.assertFalse(json.loads((ep / "clip.json").read_text())["recording"])

    def test_stopped_run_is_kept_with_outcome_and_stop_reason(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.start(root, "--trigger", "dual-cut")
            run_dual(self.graph, stop_in_a="operator ~/stop")
            self.assertTrue(r.wait_for("saved ", timeout=30), r.text)
            self.assertEqual(r.stop(), 0)
            (ep,) = self.episodes(root)
            doc = json.loads((ep / "clip.json").read_text())
            self.assertEqual((doc["outcome"], doc["kept"], doc["stop_reason"]), ("stopped", True, "operator ~/stop"))
            types, msgs = read_bag(ep / "bag")                     # readable
            self.assertGreater(len(msgs["/joint_states"]), 20)
            rec = json.loads((Path(root) / "clips" / "index.jsonl").read_text().splitlines()[0])
            self.assertEqual((rec["outcome"], rec["kept"]), ("stopped", True))

    def test_discard_stopped_opt_out(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.start(root, "--trigger", "dual-cut", "--discard-stopped")
            run_dual(self.graph, stop_in_a="x")
            self.assertTrue(r.wait_for("saved ", timeout=30), r.text)
            self.assertEqual(r.stop(), 0)
            (ep,) = self.episodes(root)
            self.assertFalse((ep / "bag").exists())
            self.assertFalse(json.loads((ep / "clip.json").read_text())["kept"])

    def test_headless_sigint_closes_the_open_bag_cleanly(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.start(root, "--trigger", "dual-cut")
            self.graph.set(WAITING, "A")
            time.sleep(0.2)
            self.graph.set(EXECUTING, "A")
            self.assertTrue(r.wait_for("RECORDING", timeout=20), r.text)
            time.sleep(1.0)
            t0 = time.monotonic()
            self.assertEqual(r.stop(signal.SIGINT), 0, r.text)
            self.assertLess(time.monotonic() - t0, 10.0)
            self.assertNotIn("Traceback", r.text)
            (ep,) = self.episodes(root)
            doc = json.loads((ep / "clip.json").read_text())
            self.assertFalse(doc["recording"])
            self.assertEqual((doc["outcome"], doc["kept"]), ("shutdown", True))
            types, msgs = read_bag(ep / "bag")
            self.assertGreater(len(msgs["/joint_states"]), 30)
            self.assertTrue((ep / "bag" / "metadata.yaml").is_file())

    def test_missing_camera_is_absent_and_flagged_and_a_slow_disk_drops_cameras_only(self):
        with tempfile.TemporaryDirectory() as root:
            self.cams.on[CAM_TOPICS[1]] = False
            r = self.start(root, "--trigger", "dual-cut", "--queue-mb", "0.2", "--debug-writer-delay-ms", "10")
            self.assertIn("NOT delivering", r.text)
            run_dual(self.graph, hold=1.5)
            self.assertTrue(r.wait_for("saved ", timeout=60), r.text)
            self.assertEqual(r.stop(), 0)
            (ep,) = self.episodes(root)
            doc = json.loads((ep / "clip.json").read_text())
            cs = doc["camera_status"]
            self.assertFalse(cs["wrist_left_gripper"]["recorded"])
            self.assertEqual(cs["wrist_left_gripper"]["messages"], 0)
            self.assertFalse(doc["cameras"]["wrist_left_gripper"]["present_at_start"])
            types, msgs = read_bag(ep / "bag")
            self.assertNotIn(CAM_TOPICS[1], types)
            drops = doc["dropped_camera_messages"]
            self.assertTrue(drops, "a backed-up queue drops camera messages")
            self.assertTrue(all(k in CAM_TOPICS or k == VISION_TOPIC for k in drops), drops)
            # control topics complete: every controller_state/status message that the recorder received was written
            self.assertGreater(len(msgs["/dual_cut_coordinator/status"]), 4)
            self.assertGreater(len(msgs["/arm_left_trajectory_controller/controller_state"]), 60)


@unittest.skipUnless(HAVE_ROS, "needs rclpy and rosbag2_py")
class FjtRawTests(RawBase):
    DUAL = False
    GRAPH_KW = dict(cams=False, vision=False)

    def setUp(self):
        super().setUp()
        self.graph.fjt()      # an idle status (transient_local: the recorder gets it as a replay and knows the topic flows)

    def test_fjt_status_episode_single_arm(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.start(root, "--trigger", "fjt-status", "--fjt-hold-s", "1.0")
            g = self.graph
            g.fjt((1, 1))
            time.sleep(0.4)
            g.fjt((1, 2))
            time.sleep(1.0)
            g.fjt((1, 4))
            self.assertTrue(r.wait_for("saved ", timeout=30), r.text)
            self.assertEqual(r.stop(), 0)
            (ep,) = self.episodes(root)
            doc = json.loads((ep / "clip.json").read_text())
            self.assertEqual((doc["outcome"], doc["trigger"]), ("finished", "fjt-status"))
            types, msgs = read_bag(ep / "bag")
            self.assertEqual(types[FJT_STATUS], "action_msgs/msg/GoalStatusArray")
            self.assertEqual(types["/rebel_arm_trajectory_controller/controller_state"],
                             "control_msgs/msg/JointTrajectoryControllerState")
            self.assertGreaterEqual(len(msgs[FJT_STATUS]), 3)
            self.assertNotIn("/dual_cut_coordinator/status", types)   # not published here

    def test_aborted_goal_is_kept_as_stopped(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.start(root, "--trigger", "fjt-status", "--fjt-hold-s", "1.0")
            g = self.graph
            g.fjt((9, 2))
            time.sleep(0.8)
            g.fjt((9, 6))
            self.assertTrue(r.wait_for("saved ", timeout=30), r.text)
            self.assertEqual(r.stop(), 0)
            (ep,) = self.episodes(root)
            doc = json.loads((ep / "clip.json").read_text())
            self.assertEqual((doc["outcome"], doc["kept"]), ("stopped", True))
            self.assertIn("ABORTED", doc["reason"])


@unittest.skipUnless(HAVE_ROS, "needs rclpy and rosbag2_py")
class RawOptionTests(unittest.TestCase):
    def run_raw(self, *argv):
        return subprocess.run([sys.executable, str(RAW), *argv], capture_output=True, text=True, env=dict(os.environ), timeout=60)

    def test_trigger_is_required_and_camera_keys_are_checked(self):
        self.assertNotEqual(self.run_raw().returncode, 0)
        out = self.run_raw("--now", "--camera-keys", "nope")
        self.assertNotEqual(out.returncode, 0)
        self.assertIn("unknown camera key", out.stderr)

    def test_no_lerobot_and_no_image_libraries_are_imported(self):
        code = ("import sys; sys.path.insert(0, %r); import record_raw_cut as r; "
                "r.main(['--help']) if False else None; "
                "bad=[m for m in ('lerobot','PIL','cv2') if m in sys.modules]; assert not bad, bad" % str(REC))
        out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=dict(os.environ), timeout=60)
        self.assertEqual(out.returncode, 0, out.stderr)

    def test_unmounted_drive_warns_loudly(self):
        with tempfile.TemporaryDirectory() as root:
            p = RawProc("--now", "--clip-dir", str(Path(root) / "c"), "--no-snapshot", "--camera-keys", "head")
            try:
                self.assertTrue(p.wait_for("READ-ONLY"), p.text)
            finally:
                p.stop()
            if not os.path.ismount("/media/farhan/My Passport"):
                self.assertIn("is NOT mounted", p.text)


if __name__ == "__main__":
    unittest.main()
