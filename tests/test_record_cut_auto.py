"""Automatic cut recording (record_dataset.py --headless / --camera-domain / sidecar clip folders / fjt-status).

Part 1 (no ROS): FjtStatusTrigger, the sidecar clip.json lifecycle and the default locations.
Part 2: the real CLI against fake publishers (tests/ros_isolation.py: robot domain 97, camera domain 98, localhost only);
nothing touches a robot. Dual-cut tests need dual_cut_msgs (source ~/igus_rebel_ros2/install/setup.bash).
"""
import json
import os
import signal
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "recording"))
import cut_sidecar as sc  # noqa: E402
import cut_trigger as ct  # noqa: E402

try:
    import numpy as np
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    import record_dataset as rec
    from test_record_cut import (CAM_TOPICS, CutRecordingBase, DUAL_REPORT, DUAL_STATUS, EXECUTING, FakeCutGraph, FJT_STATUS,
                                 HAVE_DUAL_MSGS, HAVE_LEROBOT, HAVE_ROS, IDLE, PLANNED, STATE_LEFT, STATE_RIGHT, VISION_TOPIC,
                                 WAITING, cut_argv, goal_status, jpeg, report)
except ImportError:
    HAVE_ROS = HAVE_LEROBOT = HAVE_DUAL_MSGS = False
    CutRecordingBase = object
if HAVE_LEROBOT:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

CAMERA_DOMAIN = "98"


def gs(*goals):
    """duck-typed GoalStatusArray: goals = (id byte, status)"""
    return SimpleNamespace(status_list=[SimpleNamespace(goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=bytes([g] * 16))),
                                                        status=st) for g, st in goals])


class FjtTriggerTests(unittest.TestCase):
    def setUp(self):
        self.events, self.t = [], [0.0]
        self.trig = ct.make_trigger("fjt-status", self.events.append, lambda x: None, lambda: self.t[0], post_roll_s=5.0)

    def at(self, t, *goals):
        self.t[0] = t
        self.trig.on_status(gs(*goals))
        self.trig.poll()

    def test_burst_of_goals_is_one_episode_closed_after_the_post_roll(self):
        self.at(0.0)                                   # idle, nothing happens
        self.at(1.0, (1, 1))                           # ACCEPTED -> start
        self.assertEqual(self.events, [("start", 1)])
        self.at(2.0, (1, 2))
        self.at(10.0, (1, 4), (2, 2))                  # a second trajectory right after the first
        self.at(12.0, (1, 4), (2, 4))
        self.at(14.0, (1, 4), (2, 4))                  # 4 s since the last active goal (t=10): still open
        self.assertEqual(len(self.events), 1)
        self.at(15.5, (1, 4), (2, 4))                  # > 5 s
        self.assertEqual(self.events[-1], ("end", 1, "finished", "2 goal(s)"))

    def test_aborted_or_canceled_goal_of_the_episode_is_stopped(self):
        self.at(1.0, (7, 2))
        self.at(3.0, (7, 6))
        self.at(9.0, (7, 6))
        self.assertEqual(self.events[-1][:3], ("end", 1, "stopped"))
        self.assertIn("ABORTED", self.events[-1][3])
        self.at(30.0, (7, 6), (8, 1))                  # a new goal: second episode; the old ABORTED goal is not its
        self.at(31.0, (7, 6), (8, 4))
        self.at(40.0, (7, 6), (8, 4))
        self.assertEqual(self.events[-1][:3], ("end", 2, "finished"))

    def test_old_terminal_goals_at_startup_do_not_start_anything(self):
        self.at(0.0, (3, 4), (4, 6))
        self.at(20.0, (3, 4), (4, 6))
        self.assertEqual(self.events, [])

    def test_progress_and_defaults(self):
        self.at(1.0, (1, 2))
        self.assertEqual(self.trig.progress(), [1.0, 2.0])
        self.at(2.0, (1, 4))
        self.assertEqual(self.trig.progress(), [0.0, 4.0])
        self.assertEqual(ct.DEFAULT_TOPIC["fjt-status"], FJT_STATUS_T)
        self.assertEqual(ct.ROBOT_TYPE["fjt-status"], "igus_rebel_single_arm_real_cut")

    def test_no_silence_logging(self):
        logs = []
        trig = ct.make_trigger("fjt-status", lambda e: None, logs.append, lambda: self.t[0])
        self.t[0] = 0
        trig.on_status(gs((1, 2)))
        self.t[0] = 100
        trig.poll()
        self.assertFalse([l for l in logs if "no status message" in l])


FJT_STATUS_T = "/rebel_arm_trajectory_controller/follow_joint_trajectory/_action/status"


class SidecarTests(unittest.TestCase):
    def test_lifecycle_refresh_and_snapshot(self):
        with tempfile.TemporaryDirectory() as d:
            snap = Path(d) / "seam.json"
            snap.write_text('{"x": 1}')
            t = [0.0]
            s = sc.Sidecar(os.path.join(d, "clips"), {"exec_id": 3, "dataset": "/ds", "episode_index": 5},
                           str(snap), clock=lambda: t[0], log=lambda x: None)
            doc = json.loads((Path(s.dir) / "clip.json").read_text())
            self.assertTrue(doc["recording"])
            self.assertEqual((doc["exec_id"], doc["dataset"], doc["episode_index"]), (3, "/ds", 5))
            self.assertEqual(doc["snapshot"], "seam.json")
            self.assertTrue((Path(s.dir) / "seam.json").is_file())
            self.assertRegex(Path(s.dir).name, r"^\d{8}_\d{6}$")          # the name side_cut_vision's probe expects
            m0 = (Path(s.dir) / "clip.json").stat().st_mtime_ns
            t[0] = 5.0
            s.refresh()
            self.assertEqual((Path(s.dir) / "clip.json").stat().st_mtime_ns, m0)   # not yet
            t[0] = sc.REFRESH_S + 1
            time.sleep(0.02)
            s.refresh(frames_so_far=7)
            self.assertGreater((Path(s.dir) / "clip.json").stat().st_mtime_ns, m0)
            self.assertLess(sc.REFRESH_S, 60.0)
            s.finish(saved=True, outcome="finished", episode_index=5, frames=100)
            doc = json.loads((Path(s.dir) / "clip.json").read_text())
            self.assertFalse(doc["recording"])
            self.assertEqual((doc["saved"], doc["outcome"], doc["frames"]), (True, "finished", 100))
            self.assertIn("stop_local", doc)

    def test_probe_finds_the_open_clip_unchanged(self):
        """side_cut_vision.trigger.open_clip_dir semantics: newest clip.json with recording true, modified < 30 min."""
        with tempfile.TemporaryDirectory() as d:
            s = sc.Sidecar(d, {}, None, log=lambda x: None)
            self.assertEqual(self.open_clip_dir(d), s.dir)
            s.finish(saved=False, outcome="stopped")
            self.assertIsNone(self.open_clip_dir(d))

    @staticmethod
    def open_clip_dir(out_dir, max_age_s=1800.0):   # copy of side_cut_vision/trigger.py open_clip_dir (now_s = time.time())
        best = None
        for name in os.listdir(out_dir):
            cj = os.path.join(out_dir, name, "clip.json")
            if not os.path.isfile(cj):
                continue
            with open(cj) as f:
                rec_ = json.load(f).get("recording") is True
            mt = os.path.getmtime(cj)
            if not rec_ or time.time() - mt > max_age_s:
                continue
            if best is None or mt > best[0]:
                best = (mt, os.path.join(out_dir, name))
        return best[1] if best else None

    def test_unwritable_clip_dir_never_raises(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "afile"
            f.write_text("x")
            logs = []
            s = sc.Sidecar(str(f / "sub"), {}, None, log=logs.append)       # a path below a file
            self.assertIsNone(s.dir)
            self.assertTrue(s.error)
            s.refresh()
            s.finish(saved=True)                                           # no-ops
            self.assertTrue(logs)

    def test_default_locations(self):
        old = sc.drive_writable
        try:
            sc.drive_writable = lambda: True
            self.assertEqual(sc.default_clip_dir(), "/media/farhan/My Passport/igus_nvidia/cut_videos")
            self.assertEqual(sc.default_dataset_root("/x"), "/media/farhan/My Passport/igus_nvidia/cut_datasets")
            sc.drive_writable = lambda: False
            self.assertEqual(sc.default_clip_dir(), "/home/farhan/Igus_rebel/outputs/cut_videos")
            self.assertEqual(sc.default_dataset_root("/x"), "/x")
        finally:
            sc.drive_writable = old


@unittest.skipUnless(HAVE_ROS, "needs rclpy (source /opt/ros/humble)")
class AutoOptionTests(unittest.TestCase):
    def parse(self, *argv):
        import contextlib
        import io
        old, captured = sys.argv, {}
        sys.argv = ["x", *argv]
        old_run = rec.run_mocap_main
        rec.run_mocap_main = lambda a, p: (rec.apply_source_defaults(a, p), captured.update(args=a))
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                rec.main()
        finally:
            rec.run_mocap_main, sys.argv = old_run, old
        return captured["args"]

    def test_fjt_status_defaults_single_arm(self):
        a = self.parse("--source", "moveit-real", "--episode-trigger", "fjt-status", "--arms", "left")
        self.assertEqual((a.arms, a.state_names, a.trigger_topic, a.post_roll_s), (["left"], "joint{i}", FJT_STATUS_T, 5.0))
        self.assertEqual(a.controller_state_topic, "/rebel_arm_trajectory_controller/controller_state")
        with self.assertRaises(SystemExit):
            self.parse("--source", "moveit-real", "--episode-trigger", "fjt-status", "--arms", "left", "right")

    def test_cut_root_default_follows_the_drive(self):
        old = sc.drive_writable
        os.environ.pop("ROBOT_REMOTE_DATASETS", None)
        try:
            sc.drive_writable = lambda: True
            a = self.parse("--source", "moveit-real", "--episode-trigger", "dual-cut")
            self.assertEqual(a.root, "/media/farhan/My Passport/igus_nvidia/cut_datasets")
            a = self.parse("--source", "moveit-real", "--task", "t", "--name", "n")
            self.assertTrue(a.root.endswith("robot_remote/dataset"), a.root)    # manual mode keeps the project default
            sc.drive_writable = lambda: False
            a = self.parse("--source", "moveit-real", "--episode-trigger", "h-cut")
            self.assertTrue(a.root.endswith("robot_remote/dataset"), a.root)
        finally:
            sc.drive_writable = old

    def test_camera_keys_and_presets(self):
        self.assertEqual(set(rec.CAMERA_PRESETS["cut-all"]), {"head", "knife", "wrist_left_knife", "wrist_right_knife",
                                                            "wrist_left_gripper", "wrist_right_gripper", "left_vision",
                                                            "right_vision"})
        self.assertEqual(rec.CELL_CAMERAS["left_vision"], "/side_cut_vision/arm_left/overlay/compressed")
        self.assertEqual(rec.MAIN_DOMAIN_CAMERAS, ("left_vision", "right_vision"))
        with self.assertRaises(SystemExit):
            rec.run_mocap_main(self.parse("--source", "moveit-real", "--episode-trigger", "dual-cut", "--camera-keys", "nope",
                                          "--name", "n"), SimpleParser())

    def test_auto_options_are_errors_without_a_trigger(self):
        for argv in (["--headless"], ["--clip-dir", "/tmp/x"], ["--no-sidecar"]):
            with self.assertRaises(SystemExit):
                self.parse("--source", "moveit-real", "--name", "n", "--task", "t", *argv)


class SimpleParser:
    def error(self, msg):
        raise SystemExit(msg)


class CamerasInAnotherDomain:
    """Fake camera drivers in their own rclpy Context on the camera domain."""

    def __init__(self, topics):
        self.ctx = Context()
        rclpy.init(args=[], context=self.ctx, domain_id=int(CAMERA_DOMAIN))
        self.node = Node("fake_cams_other_domain", context=self.ctx)
        pubs = {t: self.node.create_publisher(CompressedImage, t, qos_profile_sensor_data) for t in topics}
        self.on = {t: True for t in topics}
        msg = jpeg((50, 100, 200))
        self.node.create_timer(0.05, lambda: [p.publish(msg) for t, p in pubs.items() if self.on[t]])
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def stop(self):
        self.ex.shutdown()
        self.node.destroy_node()
        self.ctx.try_shutdown()


def run_dual(g, hold=0.8):
    g.set(WAITING, "A")
    time.sleep(0.2)
    g.set(EXECUTING, "A")
    time.sleep(hold)
    g.phase_report("A")
    g.set(WAITING, "B")
    time.sleep(0.2)
    g.set(EXECUTING, "B")
    time.sleep(hold)
    g.phase_report("B")
    g.set(PLANNED)


def clip_docs(root):
    return [json.loads(p.read_text()) for p in sorted((Path(root) / "clips").glob("*/clip.json"))]


@unittest.skipUnless(HAVE_ROS and HAVE_LEROBOT and HAVE_DUAL_MSGS,
                     "needs rclpy, lerobot and dual_cut_msgs (source ~/igus_rebel_ros2/install/setup.bash)")
class DualAutoTests(CutRecordingBase):
    GRAPH_KW = dict(cams=False, vision=True)      # cameras are in the other domain; the overlay stays in the robot domain

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.cams = CamerasInAnotherDomain(CAM_TOPICS)

    @classmethod
    def tearDownClass(cls):
        cls.cams.stop()
        super().tearDownClass()

    def setUp(self):
        super().setUp()
        for t in CAM_TOPICS:
            self.cams.on[t] = True

    def test_camera_domain_split_sidecar_lifecycle_and_vision_overlay(self):
        with tempfile.TemporaryDirectory() as root:
            snap = Path(root) / "seam.json"
            snap.write_text('{"seam": true}')
            r = self.launch(root, "dual-cut", "--camera-domain", CAMERA_DOMAIN, "--camera-keys", "left_vision",
                            "--snapshot", str(snap), "--headless")
            g = self.graph
            self.assertIn(f"camera topics are read in ROS domain {CAMERA_DOMAIN}", r.text)
            self.assertNotIn("NOT recorded", r.text)         # the cameras exist ONLY in the camera domain
            threading.Thread(target=run_dual, args=(g,), daemon=True).start()
            self.assertTrue(r.wait_for("RECORDING", timeout=20), r.text)
            time.sleep(0.7)
            docs = clip_docs(root)                           # while the episode is open
            self.assertEqual(len(docs), 1)
            self.assertTrue(docs[0]["recording"])
            self.assertEqual((docs[0]["exec_id"], docs[0]["trigger"], docs[0]["backend"]), (1, "dual-cut", "lerobot"))
            self.assertEqual(docs[0]["episode_index"], 0)
            self.assertTrue(docs[0]["dataset"].endswith("/ds"))
            self.assertEqual(docs[0]["snapshot"], "seam.json")
            clip_dir = next((Path(root) / "clips").iterdir())
            self.assertTrue((clip_dir / "seam.json").is_file())
            self.assertTrue(r.wait_for("saved. total episodes: 1", timeout=60), r.text)
            time.sleep(0.5)
            (doc,) = clip_docs(root)
            self.assertFalse(doc["recording"])
            self.assertEqual((doc["outcome"], doc["saved"], doc["episode_index"]), ("finished", True, 0))
            self.assertGreater(doc["frames"], 30)
            self.assertEqual({d["phase"] for d in doc["exec_reports"]}, {"A", "B"})
            self.assertEqual(doc["stop_reason"], "")
            self.assertEqual(set(doc["camera_frame_flags"]), {"head", "wrist_left_gripper", "wrist_right_gripper", "left_vision"})
            # the robot-domain recorder node does not see the camera topics, only the overlay
            node = self.graph.node
            subs = dict(node.get_subscriber_names_and_types_by_node("dataset_recorder_moveit", "/"))
            self.assertIn(VISION_TOPIC, subs)
            self.assertNotIn(CAM_TOPICS[0], subs)
            r.p.send_signal(signal.SIGTERM)
            self.assertEqual(r.finish(timeout=30), 0, r.text)
            ds = LeRobotDataset("local/ds", root=Path(root) / "ds")
            self.assertEqual(ds.meta.total_episodes, 1)
            for key, rgb in (("head", (50, 100, 200)), ("left_vision", (220, 40, 40))):
                img = np.asarray(ds[len(ds) // 2][f"observation.images.{key}"])
                np.testing.assert_allclose(img.reshape(3, -1).mean(axis=1), np.array(rgb) / 255, atol=0.08, err_msg=key)

    def test_headless_sigint_discards_the_open_episode_and_keeps_the_dataset_loadable(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.launch(root, "dual-cut", "--camera-domain", CAMERA_DOMAIN, "--headless")
            g = self.graph
            run_dual(g, hold=0.5)                                     # episode 0 finishes and is saved at its end
            self.assertTrue(r.wait_for("saved. total episodes: 1", timeout=60), r.text)
            time.sleep(1.0)
            g.set(WAITING, "A")
            time.sleep(0.2)
            g.set(EXECUTING, "A")
            self.assertTrue(r.wait_for("RECORDING episode 1", timeout=20), r.text)
            time.sleep(1.0)
            t0 = time.monotonic()
            r.p.send_signal(signal.SIGINT)                            # the launch's Ctrl-C
            self.assertEqual(r.finish(timeout=30), 0, r.text)
            self.assertLess(time.monotonic() - t0, 10.0, "shutdown must be fast")
            self.assertIn("open episode discarded", r.text)
            self.assertNotIn("Traceback", r.text)
            ds = LeRobotDataset("local/ds", root=Path(root) / "ds")
            self.assertEqual(ds.meta.total_episodes, 1)
            self.assertGreater(len(ds), 10)
            self.assertEqual(ds[0]["observation.images.head"].shape[0], 3)
            docs = clip_docs(root)
            self.assertEqual([d["recording"] for d in docs], [False, False])
            self.assertEqual((docs[1]["outcome"], docs[1]["saved"]), ("shutdown", False))

    def test_camera_dropped_at_startup_and_missing_mid_episode(self):
        with tempfile.TemporaryDirectory() as root:
            self.cams.on[CAM_TOPICS[1]] = False                       # left gripper camera is not running at start-up
            r = Proc_(root, "--camera-domain", CAMERA_DOMAIN, "--camera-discover-s", "4", "--headless")
            self.addCleanup(lambda: (r.p.kill() if r.p.poll() is None else None, r.p.wait()))
            self.assertTrue(r.wait_for("READ-ONLY"), r.text)
            self.assertIn("camera wrist_left_gripper", r.text)
            self.assertIn("NOT recorded", r.text)
            time.sleep(1.5)
            g = self.graph
            g.set(WAITING, "A")
            time.sleep(0.2)
            g.set(EXECUTING, "A")
            self.assertTrue(r.wait_for("RECORDING", timeout=20), r.text)
            time.sleep(0.8)
            self.cams.on[CAM_TOPICS[2]] = False                       # right gripper camera dies mid-episode
            time.sleep(1.5)
            g.phase_report("A")
            g.set(PLANNED)
            self.assertTrue(r.wait_for("saved. total episodes: 1", timeout=60), r.text)   # the episode is NOT lost
            time.sleep(0.5)
            r.p.send_signal(signal.SIGTERM)
            self.assertEqual(r.finish(timeout=30), 0, r.text)
            ds = LeRobotDataset("local/ds", root=Path(root) / "ds")
            self.assertNotIn("observation.images.wrist_left_gripper", ds.meta.features)
            self.assertIn("observation.images.wrist_right_gripper", ds.meta.features)
            (doc,) = clip_docs(root)
            self.assertNotIn("wrist_left_gripper", doc["camera_frame_flags"])
            flags = doc["camera_frame_flags"]["wrist_right_gripper"]
            self.assertGreater(flags["stale"], 10, flags)             # last frame repeated, flagged
            self.assertEqual(doc["camera_frame_flags"]["head"]["stale"], 0)
            n = len(ds)
            img = np.asarray(ds[n - 1]["observation.images.wrist_right_gripper"])   # still the last real frame, not black
            np.testing.assert_allclose(img.reshape(3, -1).mean(axis=1), [50 / 255, 100 / 255, 200 / 255], atol=0.08)


def Proc_(root, *extra):
    from test_record_mocap import Proc
    return Proc(cut_argv(root, "dual-cut", *extra))


@unittest.skipUnless(HAVE_ROS and HAVE_LEROBOT, "needs rclpy (source /opt/ros/humble) and lerobot")
class FjtAutoTests(CutRecordingBase):
    DUAL = False

    def test_fjt_status_saves_a_burst_and_discards_an_aborted_one_headless(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.launch(root, "fjt-status", "--arms", "right", "--post-roll-s", "1.0", "--headless")
            self.assertIn("episode trigger fjt-status", r.text)
            g = self.graph
            g.fjt((1, 1))
            time.sleep(0.5)
            g.fjt((1, 2))
            time.sleep(1.0)
            g.fjt((1, 4), (2, 2))                                     # next trajectory of the same cut: still one episode
            time.sleep(1.0)
            g.fjt((1, 4), (2, 4))
            self.assertTrue(r.wait_for("saved. total episodes: 1", timeout=60), r.text)
            time.sleep(1.5)
            g.fjt((1, 4), (2, 4), (3, 2))                             # second cut, aborted
            time.sleep(1.0)
            g.fjt((1, 4), (2, 4), (3, 6))
            self.assertTrue(r.wait_for("✕ discarded", timeout=30), r.text)
            self.assertIn("ABORTED", r.text)
            r.p.send_signal(signal.SIGTERM)
            self.assertEqual(r.finish(timeout=30), 0, r.text)
            ds = LeRobotDataset("local/ds", root=Path(root) / "ds")
            self.assertEqual(ds.meta.total_episodes, 1)
            self.assertEqual(ds.meta.robot_type, "igus_rebel_single_arm_real_cut")
            self.assertEqual(list(ds.meta.features["observation.cut_progress"]["names"]), ct.FEATURE_NAMES["fjt-status"])
            hf = ds.hf_dataset.with_format(None)
            np.testing.assert_allclose(np.asarray(hf["observation.state"], np.float32)[:, 7:13],
                                       np.tile(STATE_RIGHT, (len(ds), 1)), atol=1e-6)
            prog = np.asarray(hf["observation.cut_progress"], np.float32)
            self.assertIn(1.0, prog[:, 0])                              # a goal was active
            docs = clip_docs(root)
            self.assertEqual([(d["outcome"], d["saved"]) for d in docs], [("finished", True), ("stopped", False)])
            self.assertIn("goal_last_status", docs[0])


if __name__ == "__main__":
    unittest.main()
