"""Real-arm MoveIt / scripted recording (record_dataset.py --source moveit-real).

Private ROS graph (tests/ros_isolation.py: domain 97, localhost only, set explicitly) with fake publishers; nothing
touches a robot.
    source /opt/ros/humble/setup.bash && python3 -m unittest discover -s tests -p 'test_record_m*.py' -v
"""
import argparse
import io
import os
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
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "recording"))
try:
    import numpy as np
    import rclpy
    from control_msgs.msg import JointTrajectoryControllerState
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image, JointState
    import record_dataset as rec
    from test_record_mocap import (CAM_BGR, HAVE_LEROBOT, HAVE_ROS, OTHER_DEG, RECORDER, STATE_RIGHT, Proc,
                                   sub_env)
except ImportError:
    HAVE_ROS = HAVE_LEROBOT = False
if HAVE_LEROBOT:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

REF_RIGHT = [0.15, -0.25, 0.35, -0.45, 0.55, -0.65]   # JTC reference (the setpoint we must record)
DESIRED_RIGHT = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]        # deliberately different (older-JTC field)
JOINTS = [f"joint{i}" for i in range(1, 7)]
TOPIC = "/rebel_arm_trajectory_controller/controller_state"


NO_TRIGGER = dict(episode_trigger="manual", keep_stopped=False, trigger_topic=None, exec_report_topic=None, task="t",
                  arms=None, state_names=None, post_roll_s=5.0, camera_domain=None, headless=False, clip_dir=None,
                  no_sidecar=False)   # the other options of apply_source_defaults


def jtc_msg(names, reference=None, desired=None, feedback=None):
    m = JointTrajectoryControllerState(joint_names=list(names))
    if reference is not None:
        m.reference.positions = list(reference)
    if desired is not None:
        m.desired.positions = list(desired)
    if feedback is not None:
        m.feedback.positions = list(feedback)
    return m


class FakeMoveitGraph:
    """Fake single-arm ros2_control stack + MoveIt PC: /joint_states, JTC controller_state at 50 Hz, 3 cameras."""

    def start(self):
        rclpy.init()
        self.node = n = Node("fake_moveit_robot")
        self.js = n.create_publisher(JointState, "/joint_states", 10)
        self.ctrl = n.create_publisher(JointTrajectoryControllerState, TOPIC, 10)  # reliable, like the real JTC
        self.cam_big = n.create_publisher(Image, "/camera/color/image_raw", qos_profile_sensor_data)
        self.cam_jpg = n.create_publisher(CompressedImage, "/arm_right/gripper_camera/image_raw/compressed",
                                          qos_profile_sensor_data)
        self.cam_knife = n.create_publisher(CompressedImage, "/arm_right/wrist_camera/color/image_raw/compressed",
                                            qos_profile_sensor_data)
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
        PILImage.new("RGB", (1280, 800), (50, 100, 200)).save(buf, "JPEG", quality=95)
        self.knife_jpg = CompressedImage(format="rgb8; jpeg compressed bgr8", data=buf.getvalue())
        n.create_timer(0.02, self.pub_fast)
        n.create_timer(0.05, lambda: (self.cam_big.publish(self.big), self.cam_jpg.publish(self.jpg)))
        n.create_timer(1 / 15, lambda: self.cam_knife.publish(self.knife_jpg))
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(n)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def pub_fast(self):
        now = self.node.get_clock().now().to_msg()
        m = JointState(name=JOINTS, position=STATE_RIGHT)
        m.header.stamp = now
        self.js.publish(m)
        c = jtc_msg(JOINTS, reference=REF_RIGHT, desired=DESIRED_RIGHT, feedback=STATE_RIGHT)
        c.header.stamp = now
        self.ctrl.publish(c)

    def stop(self):
        self.ex.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown()


def recorder_args(**kw):
    d = dict(source="moveit-real", fps=30, state_topic="/joint_states", state_names="joint{i}", action_names="joint{i}",
             controller_state_topic=TOPIC, arms=["right"], other_arm_deg=[0] * 6, width=4, height=2)
    d.update(kw)
    return argparse.Namespace(**d)


@unittest.skipUnless(HAVE_ROS and HAVE_LEROBOT, "needs rclpy (source /opt/ros/humble) and lerobot")
class MoveitRecordingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.graph = FakeMoveitGraph()
        cls.graph.start()

    @classmethod
    def tearDownClass(cls):
        cls.graph.stop()

    def test_record_episode_uses_controller_reference_and_is_read_only(self):
        with tempfile.TemporaryDirectory() as root:
            argv = ["--source", "moveit-real", "--name", "ds", "--task", "open the box", "--root", root, "--fps", "30",
                    "--camera-preset", "cell-right", "--camera", "head=/camera/color/image_raw",
                    "--other-arm-deg", *map(str, OTHER_DEG), "--discover-s", "3"]
            r = Proc(argv)
            try:
                self.assertTrue(r.wait_for("READ-ONLY"), r.text)
                started = False
                for _ in range(40):
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
            self.assertIn("moveit-real: arms driven", r.text)
            self.assertIn(TOPIC, r.text)
            self.check_dataset(Path(root) / "ds")

    def assert_read_only_graph(self):
        node = self.graph.node
        name = "dataset_recorder_moveit"
        pubs = {t for t, _ in node.get_publisher_names_and_types_by_node(name, "/")}
        self.assertLessEqual(pubs, {"/rosout", "/parameter_events"}, pubs)
        self.assertEqual(node.get_service_names_and_types_by_node(name, "/"), [])
        self.assertEqual(node.get_client_names_and_types_by_node(name, "/"), [])
        subs = dict(node.get_subscriber_names_and_types_by_node(name, "/"))
        self.assertIn("/joint_states", subs)
        self.assertIn(TOPIC, subs)
        self.assertIn("control_msgs/msg/JointTrajectoryControllerState", subs[TOPIC])
        self.assertNotIn("/dual_arm/isaac_joint_commands", subs)
        self.assertNotIn("/mocap_follower/status", subs)

    def check_dataset(self, root):
        ds = LeRobotDataset("local/ds", root=root)
        self.assertEqual(ds.meta.robot_type, "igus_rebel_dual_arm_real_moveit")
        self.assertEqual(ds.meta.total_episodes, 1)
        feats = ds.meta.features
        self.assertEqual(list(feats["observation.state"]["names"]), rec.NAMES)
        self.assertEqual(list(feats["action"]["names"]), rec.NAMES)
        self.assertEqual(tuple(feats["observation.state"]["shape"]), (14,))
        self.assertEqual(tuple(feats["action"]["shape"]), (14,))
        self.assertNotIn("observation.follower_state", feats)
        for key in ("head", "wrist_right_gripper", "wrist_right_knife"):
            self.assertIn(f"observation.images.{key}", feats)
        self.assertEqual(ds.fps, 30)
        n = len(ds)
        self.assertTrue(40 <= n <= 90, f"{n} frames for ~2 s at 30 fps")
        hf = ds.hf_dataset.with_format(None)
        state = np.asarray(hf["observation.state"], np.float32)
        action = np.asarray(hf["action"], np.float32)
        other = np.deg2rad(np.asarray(OTHER_DEG, np.float32))
        np.testing.assert_allclose(state[:, 7:13], np.tile(STATE_RIGHT, (n, 1)), atol=1e-6)
        np.testing.assert_allclose(state[:, 0:6], np.tile(other, (n, 1)), atol=1e-6)
        np.testing.assert_allclose(action[:, 7:13], np.tile(REF_RIGHT, (n, 1)), atol=1e-6)  # reference, not desired/feedback
        np.testing.assert_allclose(action[:, 0:6], np.tile(other, (n, 1)), atol=1e-6)
        for col in (6, 13):
            self.assertTrue(np.all(state[:, col] == 0.0))
            self.assertTrue(np.all(action[:, col] == 0.0))
        for key in ("head", "wrist_right_gripper", "wrist_right_knife"):
            for idx in (0, n // 2, n - 1):
                img = np.asarray(ds[idx][f"observation.images.{key}"])
                self.assertEqual(img.shape, (3, 480, 640))
                mean = img.reshape(3, -1).mean(axis=1)
                np.testing.assert_allclose(mean, [50 / 255, 100 / 255, 200 / 255], atol=0.08, err_msg=f"{key} colour")


@unittest.skipUnless(HAVE_ROS, "needs rclpy (source /opt/ros/humble)")
class ControllerStateUnitTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.try_shutdown()

    def make(self, **kw):
        node = rec.MocapRecorder(recorder_args(**kw), {})
        self.addCleanup(lambda: (node.close(), node.destroy_node()))
        return node

    def test_reference_preferred_then_desired_then_ignored(self):
        node = self.make()
        node.on_state(JointState(name=JOINTS, position=STATE_RIGHT))
        frame, why = node.snapshot()
        self.assertIsNone(frame)
        self.assertIn(TOPIC, why)
        self.assertIn("no setpoint", why)
        # neither reference nor desired: ignored
        node.on_controller_state(jtc_msg(JOINTS, feedback=STATE_RIGHT))
        self.assertEqual(node.action, {})
        # reference of the wrong length is not used; desired is (older JTC)
        node.on_controller_state(jtc_msg(JOINTS, reference=REF_RIGHT[:3], desired=DESIRED_RIGHT))
        np.testing.assert_allclose(node.action["right"][0], DESIRED_RIGHT, atol=1e-6)
        node.action.clear()
        # reference empty: desired
        node.on_controller_state(jtc_msg(JOINTS, desired=DESIRED_RIGHT))
        np.testing.assert_allclose(node.action["right"][0], DESIRED_RIGHT, atol=1e-6)
        # both present: reference wins
        node.on_controller_state(jtc_msg(JOINTS, reference=REF_RIGHT, desired=DESIRED_RIGHT))
        np.testing.assert_allclose(node.action["right"][0], REF_RIGHT, atol=1e-6)
        # no joint names / unrelated joints: ignored, last value kept
        node.on_controller_state(jtc_msg([], reference=[]))
        node.on_controller_state(jtc_msg(["x1", "x2", "x3", "x4", "x5", "x6"], reference=DESIRED_RIGHT))
        np.testing.assert_allclose(node.action["right"][0], REF_RIGHT, atol=1e-6)
        frame, stale = node.snapshot()
        np.testing.assert_allclose(frame["action"][7:13], REF_RIGHT, atol=1e-6)
        self.assertEqual(sorted(frame), ["action", "observation.state"])  # no follower_state

    def test_dual_arm_topics_and_names_select_the_arm(self):
        topic = "/arm_{side}_trajectory_controller/controller_state"
        node = self.make(arms=["left", "right"], state_names="arm_{side}_joint{i}", action_names="arm_{side}_joint{i}",
                         controller_state_topic=topic)
        self.assertEqual(rec.controller_topics(node.args), ["/arm_left_trajectory_controller/controller_state",
                                                            "/arm_right_trajectory_controller/controller_state"])
        names = {t for t in (s.topic_name for s in node.subscriptions)}
        self.assertIn("/arm_left_trajectory_controller/controller_state", names)
        self.assertIn("/arm_right_trajectory_controller/controller_state", names)
        left = [f"arm_left_joint{i}" for i in range(1, 7)]
        node.on_controller_state(jtc_msg(left, reference=REF_RIGHT))
        self.assertEqual(list(node.action), ["left"])
        # a single topic without {side} serves both arms through the joint names
        node2 = self.make(arms=["left", "right"], state_names="arm_{side}_joint{i}", action_names="arm_{side}_joint{i}")
        right = [f"arm_right_joint{i}" for i in range(1, 7)]
        node2.on_controller_state(jtc_msg(left + right, reference=REF_RIGHT + DESIRED_RIGHT))
        np.testing.assert_allclose(node2.action["left"][0], REF_RIGHT, atol=1e-6)
        np.testing.assert_allclose(node2.action["right"][0], DESIRED_RIGHT, atol=1e-6)

    def test_node_creates_no_publishers_services_or_clients(self):
        node = self.make()
        self.assertEqual([p.topic_name for p in node.publishers], ["/parameter_events"])
        self.assertEqual(list(node.services), [])
        self.assertEqual(list(node.clients), [])
        self.assertEqual(len(list(node.subscriptions)), 3)  # state, controller state, events


class SourceOptionTests(unittest.TestCase):
    """Option handling runs before any ROS use, so these need no graph."""

    def run_cli(self, *argv):
        return subprocess.run([sys.executable, str(RECORDER), *argv], capture_output=True, text=True, env=sub_env(),
                              timeout=60)

    @unittest.skipUnless(HAVE_ROS, "needs rclpy")
    def test_mocap_only_options_are_errors_with_moveit_real(self):
        for flag, val in (("--action-topic", "/x"), ("--follower-status-topic", "/y")):
            out = self.run_cli("--source", "moveit-real", flag, val, "--name", "n", "--task", "t")
            self.assertEqual(out.returncode, 2, out.stderr)
            self.assertIn(flag, out.stderr)
            self.assertIn("--source mocap-real", out.stderr)

    @unittest.skipUnless(HAVE_ROS, "needs rclpy")
    def test_controller_state_topic_is_an_error_with_other_sources(self):
        for src in ("mocap-real", "isaac"):
            out = self.run_cli("--source", src, "--controller-state-topic", "/z", "--name", "n", "--task", "t")
            self.assertEqual(out.returncode, 2, out.stderr)
            self.assertIn("--controller-state-topic", out.stderr)

    @unittest.skipUnless(HAVE_ROS, "needs rclpy")
    def test_source_dependent_defaults(self):
        for src, names in (("mocap-real", "arm_{side}_joint{i}"), ("moveit-real", "joint{i}")):
            a = argparse.Namespace(source=src, action_names=None, action_topic=None, follower_status_topic=None,
                                   controller_state_topic=None, **NO_TRIGGER)
            rec.apply_source_defaults(a, argparse.ArgumentParser())
            self.assertEqual(a.action_names, names)
        self.assertEqual(a.controller_state_topic, "/rebel_arm_trajectory_controller/controller_state")
        self.assertIsNone(a.action_topic)
        a = argparse.Namespace(source="mocap-real", action_names="custom", action_topic=None, follower_status_topic=None,
                               controller_state_topic=None, **NO_TRIGGER)
        rec.apply_source_defaults(a, argparse.ArgumentParser())
        self.assertEqual(a.action_names, "custom")
        self.assertEqual(a.action_topic, "/dual_arm/isaac_joint_commands")
        self.assertEqual(a.follower_status_topic, "/mocap_follower/status")


if __name__ == "__main__":
    unittest.main()
