"""Real-arm mocap recording (record_dataset.py --source mocap-real).

Everything runs on a private ROS graph (tests/ros_isolation.py: domain 97, localhost only, set explicitly) with fake
publishers; nothing touches a robot. Run with system python3 + /opt/ros/humble sourced:
    source /opt/ros/humble/setup.bash && python3 -m unittest discover -s tests -p 'test_record_mocap.py' -v
"""
import io
import json
import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402  (first: forces the private test domain + localhost only before rclpy is imported)

TEST_DOMAIN = ros_isolation.TEST_DOMAIN

REC_DIR = Path(__file__).resolve().parents[1] / "recording"
RECORDER = REC_DIR / "record_dataset.py"
sys.path.insert(0, str(REC_DIR))
try:
    import numpy as np
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image, JointState
    from std_msgs.msg import String
    import record_dataset as rec
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False
try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    HAVE_LEROBOT = True
except ImportError:
    HAVE_LEROBOT = False

STATE_RIGHT = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60]
TARGET_RIGHT = [0.15, -0.25, 0.35, -0.45, 0.55, -0.65]
TARGET_LEFT = [0.9, 0.8, 0.7, 0.6, 0.5, 0.4]  # published by the mocap PC but the left arm is not driven
OTHER_DEG = [0.0, 10.0, 20.0, 0.0, -30.0, 0.0]
CAM_BGR = (200, 100, 50)  # -> RGB (50, 100, 200)


def sub_env():
    env = dict(os.environ)
    env.update(ROS_DOMAIN_ID=TEST_DOMAIN, ROS_LOCALHOST_ONLY="1", HF_HUB_OFFLINE="1", PYTHONUNBUFFERED="1")
    return env


class FakeRobotGraph:
    """Fake mocap PC + real-arm stack: joint states, raw mocap targets, follower status, 4 cameras."""

    def start(self):
        rclpy.init()
        self.node = Node("fake_mocap_robot")
        n = self.node
        self.js = n.create_publisher(JointState, "/joint_states", 10)
        self.cmd = n.create_publisher(JointState, "/dual_arm/isaac_joint_commands", 10)
        self.status = n.create_publisher(String, "/mocap_follower/status", 10)
        self.cam_big = n.create_publisher(Image, "/camera/color/image_raw", qos_profile_sensor_data)
        self.cam_jpg = n.create_publisher(CompressedImage, "/arm_right/gripper_camera/image_raw/compressed",
                                          qos_profile_sensor_data)
        self.cam_knife = n.create_publisher(CompressedImage, "/arm_right/wrist_camera/color/image_raw/compressed",
                                            qos_profile_sensor_data)
        self.cam_bad = n.create_publisher(Image, "/depth_cam/image_raw", qos_profile_sensor_data)
        big = Image(height=720, width=1280, encoding="bgr8", step=1280 * 3)
        frame = np.zeros((720, 1280, 3), np.uint8)
        frame[:] = CAM_BGR
        big.data = frame.tobytes()
        self.big = big
        from PIL import Image as PILImage
        buf = io.BytesIO()
        PILImage.new("RGB", (320, 240), (50, 100, 200)).save(buf, "JPEG", quality=95)
        self.jpg = CompressedImage(format="rgb8; jpeg compressed bgr8", data=buf.getvalue())
        buf = io.BytesIO()
        PILImage.new("RGB", (1920, 1080), (50, 100, 200)).save(buf, "JPEG", quality=95)  # decoded at 1/2 scale
        self.knife_jpg = CompressedImage(format="rgb8; jpeg compressed bgr8", data=buf.getvalue())
        self.bad = Image(height=4, width=4, encoding="16UC1", step=8, data=bytes(32))
        n.create_timer(0.02, self.pub_fast)
        n.create_timer(0.05, self.pub_cams)
        n.create_timer(1 / 15, lambda: self.cam_knife.publish(self.knife_jpg))  # Orbbec color runs at 15 fps
        n.create_timer(0.1, lambda: self.status.publish(String(data=json.dumps({"state": "ENABLED", "reason": "following"}))))
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(n)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()

    def pub_fast(self):
        now = self.node.get_clock().now().to_msg()
        m = JointState(name=[f"joint{i}" for i in range(1, 7)], position=STATE_RIGHT)
        m.header.stamp = now
        self.js.publish(m)
        c = JointState(name=[f"arm_left_joint{i}" for i in range(1, 7)] + [f"arm_right_joint{i}" for i in range(1, 7)]
                       + ["arm_right_gripper"], position=TARGET_LEFT + TARGET_RIGHT + [0.04])
        c.header.stamp = now
        self.cmd.publish(c)

    def pub_cams(self):
        self.cam_big.publish(self.big)
        self.cam_jpg.publish(self.jpg)
        self.cam_bad.publish(self.bad)

    def stop(self):
        self.ex.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown()


class Proc:
    def __init__(self, argv):
        self.p = subprocess.Popen([sys.executable, str(RECORDER), *argv], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True, env=sub_env())
        self.lines = []
        self.q = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.p.stdout:
            self.lines.append(line)
            self.q.put(line)

    def send(self, text):
        self.p.stdin.write(text)
        self.p.stdin.flush()

    def wait_for(self, text, timeout=40):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if any(text in l for l in self.lines):
                return True
            if self.p.poll() is not None and self.q.empty():
                return any(text in l for l in self.lines)
            time.sleep(0.05)
        return False

    @property
    def text(self):
        return "".join(self.lines)

    def finish(self, timeout=60):
        try:
            return self.p.wait(timeout)
        except subprocess.TimeoutExpired:
            self.p.kill()
            raise
        finally:
            self.p.stdin.close()


@unittest.skipUnless(HAVE_ROS and HAVE_LEROBOT, "needs rclpy (source /opt/ros/humble) and lerobot")
class MocapRecordingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.graph = FakeRobotGraph()
        cls.graph.start()

    @classmethod
    def tearDownClass(cls):
        cls.graph.stop()

    def test_list_cameras(self):
        out = subprocess.run([sys.executable, str(RECORDER), "--source", "mocap-real", "--list-cameras",
                              "--discover-s", "3"], capture_output=True, text=True, env=sub_env(), timeout=60)
        self.assertEqual(out.returncode, 0, out.stdout + out.stderr)
        lines = {l.split()[0]: l for l in out.stdout.splitlines() if l.strip().startswith("/")}
        big = lines["/camera/color/image_raw"]
        self.assertIn("1280x720", big)
        self.assertIn("bgr8", big)
        self.assertRegex(big, r"\d+\.\d Hz")
        self.assertIn("320x240", lines["/arm_right/gripper_camera/image_raw/compressed"])
        self.assertIn("CompressedImage", lines["/arm_right/gripper_camera/image_raw/compressed"])
        self.assertIn("UNSUPPORTED", lines["/depth_cam/image_raw"])
        self.assertIn("16UC1", lines["/depth_cam/image_raw"])

    def test_record_episode_and_read_only(self):
        with tempfile.TemporaryDirectory() as root:
            argv = ["--name", "ds", "--task", "fake pick", "--root", root, "--fps", "30",
                    "--camera-preset", "cell-right",  # head + /arm_right gripper (jpeg) + knife (jpeg, 15 fps)
                    "--camera", "head=/camera/color/image_raw",  # overrides the preset's compressed head (raw bgr8 path)
                    "--other-arm-deg", *map(str, OTHER_DEG), "--discover-s", "3"]
            r = Proc(["--source", "mocap-real", *argv])
            try:
                self.assertTrue(r.wait_for("READ-ONLY"), r.text)
                started = False
                for _ in range(40):  # streams need a moment to connect
                    r.send("\n")
                    if r.wait_for("RECORDING", timeout=0.5):
                        started = True
                        break
                self.assertTrue(started, r.text)
                time.sleep(0.5)
                self.assert_read_only_graph()
                time.sleep(1.5)
                r.send("\n")
                self.assertTrue(r.wait_for("saved. total episodes: 1", timeout=60), r.text)
                r.send("q\n")
                self.assertEqual(r.finish(), 0, r.text)
            finally:
                if r.p.poll() is None:
                    r.p.kill()
                r.p.wait()
            self.assertIn("camera head (/camera/color/image_raw): 1280x720 bgr8 -> resized to 640x480", r.text)
            self.assertIn("320x240", r.text)
            self.assertIn("camera wrist_right_knife (/arm_right/wrist_camera/color/image_raw/compressed): 1920x1080", r.text)
            self.assertIn("replaces the preset topic /camera/color/image_raw/compressed", r.text)
            self.check_dataset(Path(root) / "ds")

    def assert_read_only_graph(self):
        node = self.graph.node
        name = "dataset_recorder_mocap"
        pubs = {t for t, _ in node.get_publisher_names_and_types_by_node(name, "/")}
        self.assertLessEqual(pubs, {"/rosout", "/parameter_events"}, pubs)
        self.assertEqual(node.get_service_names_and_types_by_node(name, "/"), [])
        self.assertEqual(node.get_client_names_and_types_by_node(name, "/"), [])
        subs = {t for t, _ in node.get_subscriber_names_and_types_by_node(name, "/")}
        self.assertIn("/joint_states", subs)
        self.assertIn("/dual_arm/isaac_joint_commands", subs)

    def check_dataset(self, root):
        ds = LeRobotDataset("local/ds", root=root)
        self.assertEqual(ds.meta.robot_type, "igus_rebel_dual_arm_real_mocap")
        self.assertEqual(ds.meta.total_episodes, 1)
        feats = ds.meta.features
        self.assertEqual(list(feats["observation.state"]["names"]), rec.NAMES)
        self.assertEqual(list(feats["action"]["names"]), rec.NAMES)
        self.assertEqual(tuple(feats["observation.state"]["shape"]), (14,))
        self.assertEqual(tuple(feats["action"]["shape"]), (14,))
        self.assertEqual(tuple(feats["observation.follower_state"]["shape"]), (2,))
        self.assertEqual(list(feats["observation.follower_state"]["names"]), ["left", "right"])
        self.assertIn("observation.images.head", feats)
        self.assertIn("observation.images.wrist_right_gripper", feats)
        self.assertIn("observation.images.wrist_right_knife", feats)
        self.assertNotIn("observation.images.wrist_left_gripper", feats)
        self.assertEqual(ds.fps, 30)
        n = len(ds)
        self.assertTrue(40 <= n <= 90, f"{n} frames for ~2 s at 30 fps")
        hf = ds.hf_dataset.with_format(None)
        state = np.asarray(hf["observation.state"], np.float32)
        action = np.asarray(hf["action"], np.float32)
        fol = np.asarray(hf["observation.follower_state"], np.float32)
        other = np.deg2rad(np.asarray(OTHER_DEG, np.float32))
        np.testing.assert_allclose(state[:, 7:13], np.tile(STATE_RIGHT, (n, 1)), atol=1e-6)
        np.testing.assert_allclose(state[:, 0:6], np.tile(other, (n, 1)), atol=1e-6)
        np.testing.assert_allclose(action[:, 7:13], np.tile(TARGET_RIGHT, (n, 1)), atol=1e-6)  # mocap target, not state
        np.testing.assert_allclose(action[:, 0:6], np.tile(other, (n, 1)), atol=1e-6)         # not the published left target
        for col in (6, 13):
            self.assertTrue(np.all(state[:, col] == 0.0))
            self.assertTrue(np.all(action[:, col] == 0.0))  # extra published gripper name (0.04) is ignored
        self.assertTrue(np.all(fol[:, 0] == 0.0))
        self.assertTrue(np.all(fol[:, 1] == rec.FOLLOWER_CODES["ENABLED"]))
        for key in ("head", "wrist_right_gripper", "wrist_right_knife"):
            for idx in (0, n // 2, n - 1):
                img = np.asarray(ds[idx][f"observation.images.{key}"])
                self.assertEqual(img.shape, (3, 480, 640))
                mean = img.reshape(3, -1).mean(axis=1)
                np.testing.assert_allclose(mean, [50 / 255, 100 / 255, 200 / 255], atol=0.08, err_msg=f"{key} colour")


@unittest.skipUnless(HAVE_ROS and HAVE_LEROBOT, "needs rclpy and lerobot")
class DatasetGuardTests(unittest.TestCase):
    def test_existing_dataset_without_resume_is_refused(self):
        if hasattr(LeRobotDataset, "resume"):
            self.skipTest("this LeRobot can resume")
        with tempfile.TemporaryDirectory() as root:
            ds = LeRobotDataset.create(repo_id="local/old", fps=30, root=Path(root) / "old", use_videos=False,
                                       features={"observation.state": {"dtype": "float32", "shape": (2,), "names": ["a", "b"]}})
            for _ in range(2):
                ds.add_frame({"observation.state": np.zeros(2, np.float32), "task": "t"})
            ds.save_episode()
            ds.finalize()
            out = subprocess.run([sys.executable, str(RECORDER), "--source", "mocap-real", "--name", "old", "--task", "t",
                                  "--root", root], capture_output=True, text=True, env=sub_env(), timeout=60)
            self.assertEqual(out.returncode, 2)
            self.assertIn("no LeRobotDataset.resume", out.stderr)
            self.assertIn("--name", out.stderr)

    def test_unfinished_or_foreign_folders_are_refused_before_ros(self):
        """dataset_check classification is shared with the other sources: never write into a folder that is not a dataset."""
        with tempfile.TemporaryDirectory() as root:
            (Path(root) / "foreign").mkdir()
            (Path(root) / "foreign" / "notes.txt").write_text("x")
            out = subprocess.run([sys.executable, str(RECORDER), "--source", "moveit-real", "--name", "foreign", "--task",
                                  "t", "--root", root], capture_output=True, text=True, env=sub_env(), timeout=60)
            self.assertNotEqual(out.returncode, 0)
            self.assertIn("not a LeRobot dataset", out.stderr)

    def test_recorder_node_creates_no_publishers_services_or_clients(self):
        import argparse
        args = argparse.Namespace(fps=30, state_topic="/joint_states", action_topic="/dual_arm/isaac_joint_commands",
                                  follower_status_topic="/mocap_follower/status", arms=["right"], other_arm_deg=[0] * 6,
                                  state_names="joint{i}", action_names="arm_{side}_joint{i}")
        rclpy.init()
        try:
            node = rec.MocapRecorder(args, {"head": ("/camera/color/image_raw", "sensor_msgs/msg/Image")})
            self.assertEqual([p.topic_name for p in node.publishers], ["/parameter_events"])
            self.assertEqual(list(node.services), [])
            self.assertEqual(list(node.clients), [])
            self.assertEqual(len(list(node.subscriptions)), 5)  # state, action, status, events, 1 camera
            node.destroy_node()
        finally:
            rclpy.try_shutdown()

    def test_camera_frames_are_decoded_off_the_sampling_thread(self):
        import argparse
        from unittest import mock
        args = argparse.Namespace(fps=30, state_topic="/joint_states", action_topic="/dual_arm/isaac_joint_commands",
                                  follower_status_topic="/mocap_follower/status", arms=["right"], other_arm_deg=[0] * 6,
                                  state_names="joint{i}", action_names="arm_{side}_joint{i}", width=4, height=2)
        rclpy.init()
        try:
            node = rec.MocapRecorder(args, {"head": ("/camera/color/image_raw", "sensor_msgs/msg/Image")})
            node.on_state(JointState(name=[f"joint{i}" for i in range(1, 7)], position=STATE_RIGHT))
            node.on_action(JointState(name=[f"arm_right_joint{i}" for i in range(1, 7)], position=TARGET_RIGHT))

            def wait_decoded(value):
                end = time.monotonic() + 5
                while time.monotonic() < end:
                    got = node.images.get("head")
                    if got is not None and not isinstance(got[0], ValueError) and np.all(got[0] == value):
                        return
                    time.sleep(0.01)
                self.fail(f"frame {value} never decoded")

            with mock.patch.object(rec, "image_msg_to_rgb", wraps=rec.image_msg_to_rgb) as dec:
                node.on_image("head", Image(height=2, width=4, encoding="rgb8", step=12, data=bytes([10] * 24)))
                wait_decoded(10)
                a, _ = node.snapshot()
                b, _ = node.snapshot()  # no new message: no decode in snapshot
                self.assertEqual(dec.call_count, 1)
                node.on_image("head", Image(height=2, width=4, encoding="rgb8", step=12, data=bytes([20] * 24)))
                wait_decoded(20)
                c, _ = node.snapshot()
                self.assertEqual(dec.call_count, 2)
            self.assertTrue(np.all(a["observation.images.head"] == 10))
            self.assertTrue(np.all(b["observation.images.head"] == 10))
            self.assertTrue(np.all(c["observation.images.head"] == 20))
            node.on_image("head", Image(height=1, width=1, encoding="16UC1", step=2, data=bytes(2)))
            end = time.monotonic() + 5
            while not isinstance(node.images["head"][0], ValueError) and time.monotonic() < end:
                time.sleep(0.01)
            with self.assertRaises(ValueError):
                node.snapshot()  # undecodable frame surfaces in the sampling loop (episode discarded there)
            node.close()
            node.destroy_node()
        finally:
            rclpy.try_shutdown()


@unittest.skipUnless(HAVE_ROS, "needs rclpy")
class ImageDecodeTests(unittest.TestCase):
    def test_yuyv_uyvy_mono_and_unknown(self):
        y, u, v = 120, 128, 128  # grey: u = v = 128
        yuyv = Image(height=2, width=4, encoding="yuv422_yuy2", step=8, data=bytes([y, u, y, v] * 4))
        out = rec.image_msg_to_rgb(yuyv, "/cam")
        self.assertEqual(out.shape, (2, 4, 3))
        self.assertTrue(np.all(np.abs(out.astype(int) - 121) <= 1))
        uyvy = Image(height=2, width=4, encoding="uyvy", step=8, data=bytes([u, y, v, y] * 4))
        np.testing.assert_array_equal(rec.image_msg_to_rgb(uyvy, "/cam"), out)
        # red-ish pixel pair: v high -> r > g
        red = rec.image_msg_to_rgb(Image(height=1, width=2, encoding="yuyv", step=4, data=bytes([81, 90, 81, 240])), "/c")
        self.assertGreater(red[0, 0, 0], 200)
        self.assertLess(red[0, 0, 2], 60)
        mono = Image(height=2, width=3, encoding="mono8", step=4, data=bytes([1, 2, 3, 0, 4, 5, 6, 0]))
        self.assertEqual(rec.image_msg_to_rgb(mono, "/m").tolist()[1][2], [6, 6, 6])
        with self.assertRaises(ValueError) as cm:
            rec.image_msg_to_rgb(Image(height=1, width=1, encoding="16UC1", step=2, data=bytes(2)), "/depth")
        self.assertIn("/depth", str(cm.exception))
        self.assertIn("16UC1", str(cm.exception))

    def test_pick_cameras_prompts_per_topic(self):
        found = [{"topic": "/a", "type": "sensor_msgs/msg/Image", "error": None},
                 {"topic": "/b", "type": "sensor_msgs/msg/Image", "error": "unsupported encoding '16UC1'"},
                 {"topic": "/c", "type": "sensor_msgs/msg/CompressedImage", "error": None},
                 {"topic": "/d", "type": "sensor_msgs/msg/Image", "error": None}]
        answers = iter(["bad key!", "head", "", "head", "wrist_right_knife"])  # /a: invalid then head; /c skip; /d: dup, ok
        picked = rec.pick_cameras(found, input_fn=lambda _p: next(answers))
        self.assertEqual(picked, {"head": ("/a", "sensor_msgs/msg/Image"),
                                  "wrist_right_knife": ("/d", "sensor_msgs/msg/Image")})


if __name__ == "__main__":
    unittest.main()
