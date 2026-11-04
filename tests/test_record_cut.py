"""Recording of cutting runs (record_dataset.py --source moveit-real --episode-trigger dual-cut | h-cut).

Part 1 (no ROS): the start / end rules of recording/cut_trigger.py with synthetic status sequences.
Part 2: the real CLI against fake publishers on the private test domain (tests/ros_isolation.py: domain 97, localhost
only); nothing touches a robot. The dual-cut tests need dual_cut_msgs (source ~/igus_rebel_ros2/install/setup.bash).
    source /opt/ros/humble/setup.bash && source ~/igus_rebel_ros2/install/setup.bash
    python3 -m unittest discover -s tests -p 'test_record_cut.py' -v
"""
import io
import json
import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402  (first: forces the private test domain + localhost only before rclpy is imported)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "recording"))
import cut_trigger as ct  # noqa: E402  (standard library only)

try:
    import numpy as np
    import rclpy
    from control_msgs.msg import JointTrajectoryControllerState
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
    from unique_identifier_msgs.msg import UUID
    from rclpy.qos import qos_profile_action_status_default
    from sensor_msgs.msg import CompressedImage, JointState
    from std_msgs.msg import String
    import record_dataset as rec
    from test_record_mocap import CAM_BGR, HAVE_LEROBOT, HAVE_ROS, Proc, STATE_RIGHT, TARGET_LEFT, TARGET_RIGHT
except ImportError:
    HAVE_ROS = HAVE_LEROBOT = False
try:
    from dual_cut_msgs.msg import CoordinatorStatus
    HAVE_DUAL_MSGS = True
except ImportError:
    HAVE_DUAL_MSGS = False
if HAVE_LEROBOT:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

JOINTS = [f"joint{i}" for i in range(1, 7)]
REF_LEFT = TARGET_LEFT                      # JTC setpoints (deliberately different from the measured state)
REF_RIGHT = TARGET_RIGHT
STATE_LEFT = [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6]
DUAL_STATUS = "/dual_cut_coordinator/status"
DUAL_REPORT = "/dual_cut_coordinator/exec_report"
HCUT_STATUS = "/box_h_cut_executor/t_cut_status"
FJT_STATUS = "/rebel_arm_trajectory_controller/follow_joint_trajectory/_action/status"
CAM_TOPICS = ["/camera/color/image_raw/compressed", "/arm_left/gripper_camera/image_raw/compressed",
              "/arm_right/gripper_camera/image_raw/compressed"]
VISION_TOPIC = "/side_cut_vision/arm_left/overlay/compressed"
IDLE, PLANNING, PLANNED, WAITING, EXECUTING, STOPPED = range(6)


class Msg:  # a CoordinatorStatus look-alike for the unit tests
    def __init__(self, state, phase="", stop_latched=False, stop_reason=""):
        self.state, self.phase, self.stop_latched, self.stop_reason = state, phase, stop_latched, stop_reason


def report(phase, completed=True, **kw):
    return json.dumps(dict(plan_variant="x", phase=phase, completed=completed, wall_s=1.0, stop_latched=False,
                           stop_reason="", **kw))


def hcut(index, step, status, message=""):
    return json.dumps({"step_index": index, "step": step, "status": status, "message": message, "stamp_ns": 1})


class DualTriggerTests(unittest.TestCase):
    def setUp(self):
        self.events, self.logs, self.t = [], [], [0.0]
        self.trig = ct.DualCutTrigger(self.events.append, self.logs.append, lambda: self.t[0])

    def feed(self, *states):
        for s in states:
            self.trig.on_status(s)

    def test_two_phase_execution_is_one_episode(self):
        self.feed(Msg(IDLE), Msg(PLANNED))
        self.assertEqual(self.events, [])
        self.feed(Msg(WAITING, "A"), Msg(EXECUTING, "A"))
        self.assertEqual(self.events, [("start", 1)])           # one start, at the first executing state
        self.trig.on_report(report("A"))
        self.feed(Msg(WAITING, "B"), Msg(EXECUTING, "B"))
        self.trig.on_report(report("B"))
        self.assertEqual(len(self.events), 1)                   # the barrier between the phases does not end it
        self.feed(Msg(PLANNED))
        self.assertEqual(self.events[1], ("end", 1, "finished", "phases A+B"))

    def test_progress_codes(self):
        self.feed(Msg(IDLE), Msg(WAITING, "A"))
        self.assertEqual(self.trig.progress(), [3.0, 1.0])
        self.feed(Msg(EXECUTING, "B"))
        self.assertEqual(self.trig.progress(), [4.0, 2.0])
        self.feed(Msg(PLANNED, ""))
        self.assertEqual(self.trig.progress(), [2.0, 0.0])

    def test_stopped_ends_with_the_stop_reason(self):
        self.feed(Msg(IDLE), Msg(EXECUTING, "A"), Msg(STOPPED, "A", True, "monitor: min distance"))
        self.assertEqual(self.events[-1], ("end", 1, "stopped", "monitor: min distance"))
        self.feed(Msg(STOPPED, "A", True, "monitor: min distance"), Msg(IDLE))
        self.assertEqual(len(self.events), 2)                   # nothing more until the next execution

    def test_stop_latched_flag_alone_ends_it(self):
        self.feed(Msg(IDLE), Msg(EXECUTING, "A"), Msg(EXECUTING, "A", True, "operator ~/stop"))
        self.assertEqual(self.events[-1], ("end", 1, "stopped", "operator ~/stop"))

    def test_joining_a_running_execution_is_refused(self):
        self.feed(Msg(EXECUTING, "A"), Msg(EXECUTING, "A"), Msg(PLANNED))
        self.assertEqual(self.events, [])
        self.feed(Msg(WAITING, "A"))
        self.assertEqual(self.events, [("start", 1)])

    def test_planned_without_report_waits_for_it_then_aborts(self):
        self.feed(Msg(IDLE), Msg(WAITING, "A"), Msg(PLANNED))
        self.assertEqual(self.events, [("start", 1)])           # grace: the phase report may be in flight
        self.t[0] = ct.END_GRACE_S + 0.1
        self.trig.poll()
        self.assertEqual(self.events[-1][:3], ("end", 1, "aborted"))

    def test_phase_b_failing_after_a_completed_is_an_abort(self):
        self.feed(Msg(IDLE), Msg(WAITING, "A"), Msg(EXECUTING, "A"))
        self.trig.on_report(report("A"))
        self.feed(Msg(WAITING, "B"), Msg(PLANNED))              # phase B barrier failed: no B report
        self.assertEqual(len(self.events), 1)
        self.t[0] = ct.END_GRACE_S + 0.1
        self.trig.poll()
        self.assertEqual(self.events[-1][:3], ("end", 1, "aborted"))

    def test_report_arriving_after_planned_still_finishes(self):
        self.feed(Msg(IDLE), Msg(EXECUTING, "A"), Msg(PLANNED))
        self.trig.on_report(report("A"))
        self.assertEqual(self.events[-1][:3], ("end", 1, "finished"))

    def test_reports_outside_an_execution_are_ignored(self):
        self.trig.on_report(report("A"))                        # transient_local replay at start-up
        self.feed(Msg(IDLE), Msg(EXECUTING, "A"), Msg(PLANNED))
        self.t[0] = 1.0
        self.trig.poll()
        self.assertEqual(self.events[-1][:3], ("end", 1, "aborted"))

    def test_back_to_idle_is_an_abort(self):
        self.feed(Msg(IDLE), Msg(EXECUTING, "A"), Msg(IDLE))
        self.assertEqual(self.events[-1][:3], ("end", 1, "aborted"))

    def test_silence_is_logged_but_never_ends_the_execution(self):
        self.feed(Msg(IDLE), Msg(EXECUTING, "A"))
        self.t[0] = 10.0
        self.trig.poll()
        self.trig.poll()
        self.assertEqual(self.events, [("start", 1)])
        self.assertEqual(sum("no status message" in l for l in self.logs), 1)
        self.feed(Msg(EXECUTING, "A"), Msg(PLANNED))
        self.trig.on_report(report("A"))
        self.assertEqual(self.events[-1][:3], ("end", 1, "finished"))

    def test_second_execution_gets_a_new_id(self):
        self.feed(Msg(IDLE), Msg(EXECUTING, "A"), Msg(PLANNED))
        self.trig.on_report(report("A"))
        self.feed(Msg(PLANNED), Msg(WAITING, "A"))
        self.assertEqual([e for e in self.events if e[0] == "start"], [("start", 1), ("start", 2)])


class HCutTriggerTests(unittest.TestCase):
    def setUp(self):
        self.events, self.t = [], [0.0]
        self.trig = ct.HCutTrigger(self.events.append, lambda t: None, lambda: self.t[0])

    def test_full_sequence(self):
        self.trig.on_status(hcut(0, "sequence", "running"))
        self.assertEqual(self.events, [("start", 1)])
        self.assertEqual(self.trig.progress(), [0.0, 1.0])
        for i, name in enumerate(["approach_standoff", "side_cut_penetrate", "continue_side_cut", "continue_top_cut"], 1):
            self.trig.on_status(hcut(i, name, "running"))
            self.assertEqual(self.trig.progress(), [float(i), 1.0])
            self.trig.on_status(hcut(i, name, "done"))
            self.assertEqual(self.trig.progress(), [float(i), 2.0])
        self.assertEqual(len(self.events), 1)
        self.trig.on_status(hcut(5, "sequence", "done", "all 4 steps completed"))
        self.assertEqual(self.events[-1][:3], ("end", 1, "finished"))

    def test_failed_step(self):
        self.trig.on_status(hcut(0, "sequence", "running"))
        self.trig.on_status(hcut(2, "side_cut_penetrate", "running"))
        self.trig.on_status(hcut(2, "side_cut_penetrate", "failed", "execution did not complete"))
        self.assertEqual(self.events[-1][:3], ("end", 1, "stopped"))
        self.assertIn("side_cut_penetrate", self.events[-1][3])
        self.assertEqual(self.trig.progress(), [2.0, 3.0])

    def test_steps_without_a_start_do_not_open_an_execution(self):
        self.trig.on_status(hcut(2, "side_cut_penetrate", "running"))
        self.trig.on_status(hcut(5, "sequence", "done"))
        self.assertEqual(self.events, [])

    def test_garbage_is_ignored(self):
        self.trig.on_status("not json")
        self.trig.on_status(json.dumps({"foo": 1}))
        self.assertEqual(self.events, [])

    def test_task_suffix(self):
        self.assertEqual(ct.task_with_suffix("cut the carton tape", "stopped", "operator ~/stop"),
                         "cut the carton tape [stopped: operator ~/stop]")
        self.assertEqual(ct.task_with_suffix("t", "aborted", ""), "t [aborted: unknown]")


@unittest.skipUnless(HAVE_ROS, "needs rclpy (source /opt/ros/humble)")
class CutOptionTests(unittest.TestCase):
    """Argument handling of record_dataset.py (no ROS graph needed)."""

    def parse(self, *argv):
        import contextlib
        a = None
        old = sys.argv
        sys.argv = ["x", *argv]
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                captured = {}  # main() parses, then hands the namespace to run_mocap_main: stop there

                def fake_run(args, parser):
                    rec.apply_source_defaults(args, parser)
                    captured["args"] = args

                old_run = rec.run_mocap_main
                rec.run_mocap_main = fake_run
                try:
                    rec.main()
                finally:
                    rec.run_mocap_main = old_run
                a = captured["args"]
        finally:
            sys.argv = old
        return a

    def test_dual_cut_defaults(self):
        a = self.parse("--source", "moveit-real", "--episode-trigger", "dual-cut", "--name", "n")
        self.assertEqual(a.arms, ["left", "right"])
        self.assertEqual(a.state_names, "arm_{side}_joint{i}")
        self.assertEqual(a.action_names, "arm_{side}_joint{i}")
        self.assertEqual(a.controller_state_topic, "/arm_{side}_trajectory_controller/controller_state")
        self.assertEqual(a.trigger_topic, DUAL_STATUS)
        self.assertEqual(a.exec_report_topic, DUAL_REPORT)
        self.assertEqual(a.task, "cut the carton tape")
        self.assertEqual(a.max_episode_s, 600.0)

    def test_h_cut_defaults_and_single_arm_mapping(self):
        a = self.parse("--source", "moveit-real", "--episode-trigger", "h-cut", "--name", "n", "--arms", "left")
        self.assertEqual(a.arms, ["left"])
        self.assertEqual(a.state_names, "joint{i}")
        self.assertEqual(a.controller_state_topic, "/rebel_arm_trajectory_controller/controller_state")
        self.assertEqual(a.trigger_topic, HCUT_STATUS)
        self.assertIsNone(a.exec_report_topic)
        self.assertEqual(self.parse("--source", "moveit-real", "--episode-trigger", "h-cut", "--name", "n").arms, ["right"])

    def test_manual_is_unchanged(self):
        a = self.parse("--source", "moveit-real", "--name", "n", "--task", "t")
        self.assertEqual((a.episode_trigger, a.arms, a.state_names, a.max_episode_s), ("manual", ["right"], "joint{i}", 120.0))

    def test_invalid_combinations(self):
        for argv in (["--source", "mocap-real", "--episode-trigger", "dual-cut"],
                     ["--source", "moveit-real", "--keep-stopped"],
                     ["--source", "moveit-real", "--episode-trigger", "h-cut", "--arms", "left", "right"],
                     ["--source", "moveit-real", "--episode-trigger", "h-cut", "--exec-report-topic", "/x"]):
            with self.assertRaises(SystemExit):
                self.parse(*argv, "--name", "n", "--task", "t")

    def test_cut_preset_is_head_and_both_gripper_cameras(self):
        self.assertEqual(rec.CAMERA_PRESETS["cut"], ["head", "wrist_left_gripper", "wrist_right_gripper"])
        self.assertEqual(rec.CELL_CAMERAS["knife"], "/knife_camera/image_raw/compressed")
        self.assertNotIn("knife", rec.CAMERA_PRESETS["cell"])


RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
TRANSIENT = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def jpeg(rgb, size=(320, 240)):
    from PIL import Image as PILImage
    buf = io.BytesIO()
    PILImage.new("RGB", size, rgb).save(buf, "JPEG", quality=95)
    return CompressedImage(format="rgb8; jpeg compressed bgr8", data=buf.getvalue())


def goal_status(*goals):
    """goals = (uuid byte 0..255, GoalStatus code)"""
    m = GoalStatusArray()
    for g, st in goals:
        m.status_list.append(GoalStatus(goal_info=GoalInfo(goal_id=UUID(uuid=[g] * 16)), status=st))
    return m


class FakeCutGraph:
    """Fake cutting cell: dual /joint_states (arm_*_joint*) AND single-arm /joint_states cannot share a topic, so
    `dual` chooses which one is published. JTC controller_state at 50 Hz, 3 cameras, coordinator status at 5 Hz."""

    def __init__(self, dual, cams=True, vision=False):
        self.dual = dual
        self.cams, self.vision = cams, vision
        self.cam_on = {t: True for t in CAM_TOPICS}
        self.state = IDLE
        self.phase = ""
        self.stop_latched, self.stop_reason = False, ""
        self.jtc_silent = threading.Event()   # CRI transit simulation: controller_state stops

    def start(self):
        rclpy.init()
        self.node = n = Node("fake_cut_cell")
        self.js = n.create_publisher(JointState, "/joint_states", 10)
        if self.dual:
            self.jtc = {s: n.create_publisher(JointTrajectoryControllerState,
                                              f"/arm_{s}_trajectory_controller/controller_state", 10)
                        for s in ("left", "right")}
            self.status_pub = n.create_publisher(CoordinatorStatus, DUAL_STATUS, RELIABLE)
            self.report_pub = n.create_publisher(String, DUAL_REPORT, TRANSIENT)
        else:
            self.jtc = {"single": n.create_publisher(JointTrajectoryControllerState,
                                                     "/rebel_arm_trajectory_controller/controller_state", 10)}
            self.hcut_pub = n.create_publisher(String, HCUT_STATUS, TRANSIENT)
        if not self.dual:
            self.fjt_pub = n.create_publisher(GoalStatusArray, FJT_STATUS, qos_profile_action_status_default)
        if self.cams:
            self.cam_pubs = {t: n.create_publisher(CompressedImage, t, qos_profile_sensor_data) for t in CAM_TOPICS}
            n.create_timer(0.05, lambda: [p.publish(jpeg((50, 100, 200))) for t, p in self.cam_pubs.items()
                                          if self.cam_on[t]])
        if self.vision:   # side_cut_vision overlay: robot domain, ~3 Hz
            vp = n.create_publisher(CompressedImage, VISION_TOPIC, qos_profile_sensor_data)
            n.create_timer(0.3, lambda: vp.publish(jpeg((220, 40, 40))))
        n.create_timer(0.02, self.pub_fast)
        if self.dual:
            n.create_timer(0.2, self.publish_status)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(n)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def pub_fast(self):
        now = self.node.get_clock().now().to_msg()
        if self.dual:
            names = [f"arm_{s}_{j}" for s in ("left", "right") for j in JOINTS]
            js = JointState(name=names, position=STATE_LEFT + STATE_RIGHT)
        else:
            js = JointState(name=JOINTS, position=STATE_RIGHT)
        js.header.stamp = now
        self.js.publish(js)
        if self.jtc_silent.is_set():
            return
        for side, pub in self.jtc.items():
            names = [f"arm_{side}_{j}" for j in JOINTS] if self.dual else JOINTS
            ref = REF_LEFT if side == "left" else REF_RIGHT
            m = JointTrajectoryControllerState(joint_names=names)
            m.reference.positions = list(ref)
            m.desired.positions = [9.0] * 6
            m.header.stamp = now
            pub.publish(m)

    # ---- coordinator
    def publish_status(self):
        m = CoordinatorStatus(state=self.state, phase=self.phase, stop_latched=self.stop_latched,
                              stop_reason=self.stop_reason)
        m.header.stamp = self.node.get_clock().now().to_msg()
        self.status_pub.publish(m)

    def set(self, state, phase="", stop_latched=False, stop_reason=""):
        self.state, self.phase, self.stop_latched, self.stop_reason = state, phase, stop_latched, stop_reason
        self.publish_status()

    def phase_report(self, phase, completed=True):
        self.report_pub.publish(String(data=report(phase, completed)))

    def fjt(self, *goals):
        self.fjt_pub.publish(goal_status(*goals))

    def hcut(self, *a):
        self.hcut_pub.publish(String(data=hcut(*a)))

    def stop(self):
        self.ex.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown()


def cut_argv(root, trigger, *extra):
    return ["--source", "moveit-real", "--episode-trigger", trigger, "--name", "ds", "--root", root, "--fps", "30",
            "--camera-preset", "cut", "--discover-s", "3", "--clip-dir", str(Path(root) / "clips"), *extra]


class CutRecordingBase(unittest.TestCase):
    DUAL = True
    GRAPH_KW = {}

    @classmethod
    def setUpClass(cls):
        cls.graph = FakeCutGraph(cls.DUAL, **cls.GRAPH_KW)
        cls.graph.start()

    @classmethod
    def tearDownClass(cls):
        cls.graph.stop()

    def setUp(self):
        self.later_errors = []

    def later(self, delay, fn):
        """Run fn in a thread after `delay` s (while the recorder records); its AssertionError fails the test."""
        def run():
            try:
                fn()
            except BaseException as e:  # noqa: BLE001
                self.later_errors.append(e)
        t = threading.Timer(delay, run)
        t.start()
        self.addCleanup(lambda: (t.join(), self.assertEqual(self.later_errors, [])))

    def tearDown(self):
        self.graph.jtc_silent.clear()
        if self.DUAL:
            self.graph.set(IDLE)

    def launch(self, root, trigger, *extra):
        r = Proc(cut_argv(root, trigger, *extra))
        self.addCleanup(lambda: (r.p.kill() if r.p.poll() is None else None, r.p.wait()))
        self.assertTrue(r.wait_for("READ-ONLY"), r.text)
        time.sleep(2.0)   # discovery + one idle status, so the recorder knows the coordinator is not executing
        return r

    def quit(self, r):
        r.send("q\n")
        self.assertEqual(r.finish(), 0, r.text)

    def read_only_graph(self, node_name, want_subs):
        node = self.graph.node
        pubs = {t for t, _ in node.get_publisher_names_and_types_by_node(node_name, "/")}
        self.assertLessEqual(pubs, {"/rosout", "/parameter_events"}, pubs)
        self.assertEqual(node.get_service_names_and_types_by_node(node_name, "/"), [])
        self.assertEqual(node.get_client_names_and_types_by_node(node_name, "/"), [])
        subs = dict(node.get_subscriber_names_and_types_by_node(node_name, "/"))
        for t in want_subs:
            self.assertIn(t, subs)


@unittest.skipUnless(HAVE_ROS and HAVE_LEROBOT and HAVE_DUAL_MSGS,
                     "needs rclpy, lerobot and dual_cut_msgs (source ~/igus_rebel_ros2/install/setup.bash)")
class DualCutRecordingTests(CutRecordingBase):
    def run_execution(self, g, two_phases=True, hold=0.8):
        g.set(WAITING, "A")
        time.sleep(0.2)
        g.set(EXECUTING, "A")
        time.sleep(hold)
        g.phase_report("A")
        if two_phases:
            g.set(WAITING, "B")
            time.sleep(0.3)
            g.set(EXECUTING, "B")
            time.sleep(hold)
            g.phase_report("B")
        g.set(PLANNED)

    def test_two_phase_execution_is_saved_as_one_episode_and_is_read_only(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.launch(root, "dual-cut")
            g = self.graph
            self.later(0.8, lambda: self.read_only_graph(
                "dataset_recorder_moveit",
                [DUAL_STATUS, DUAL_REPORT, "/joint_states", "/arm_left_trajectory_controller/controller_state",
                 "/arm_right_trajectory_controller/controller_state"]))
            t0 = time.monotonic()
            self.run_execution(g)
            wall = time.monotonic() - t0
            self.assertTrue(r.wait_for("saved. total episodes: 1", timeout=60), r.text)
            time.sleep(1.0)
            g.set(WAITING, "A")                       # a status after the end must not open a second episode
            g.set(PLANNED)
            self.quit(r)
            self.assertNotIn("total episodes: 2", r.text)
            self.assertIn("execution finished", r.text)
            ds = LeRobotDataset("local/ds", root=Path(root) / "ds")
            self.assertEqual(ds.meta.total_episodes, 1)
            self.assertEqual(ds.meta.robot_type, "igus_rebel_dual_arm_real_cut")
            self.assertEqual(list(ds.meta.features["observation.cut_progress"]["names"]), ct.FEATURE_NAMES["dual-cut"])
            self.assertEqual(tuple(ds.meta.features["observation.cut_progress"]["shape"]), (2,))
            for key in ("head", "wrist_left_gripper", "wrist_right_gripper"):
                self.assertIn(f"observation.images.{key}", ds.meta.features)
            n = len(ds)
            self.assertTrue(0.7 * wall * 30 <= n <= 1.15 * wall * 30 + 5, f"{n} frames for {wall:.1f} s at 30 fps")
            hf = ds.hf_dataset.with_format(None)
            state = np.asarray(hf["observation.state"], np.float32)
            action = np.asarray(hf["action"], np.float32)
            np.testing.assert_allclose(state[:, 0:6], np.tile(STATE_LEFT, (n, 1)), atol=1e-6)
            np.testing.assert_allclose(state[:, 7:13], np.tile(STATE_RIGHT, (n, 1)), atol=1e-6)
            np.testing.assert_allclose(action[:, 0:6], np.tile(REF_LEFT, (n, 1)), atol=1e-6)    # JTC reference
            np.testing.assert_allclose(action[:, 7:13], np.tile(REF_RIGHT, (n, 1)), atol=1e-6)  # (not desired = 9.0)
            prog = np.asarray(hf["observation.cut_progress"], np.float32)
            pairs = [tuple(p) for p in prog]
            seen = list(dict.fromkeys(pairs))
            self.assertEqual(seen[0], (float(WAITING), 1.0), seen)
            self.assertIn((float(EXECUTING), 1.0), seen)
            self.assertIn((float(WAITING), 2.0), seen)
            self.assertEqual(seen[-1], (float(EXECUTING), 2.0), seen)
            self.assertEqual(seen.index((float(EXECUTING), 1.0)) < seen.index((float(WAITING), 2.0)), True)
            self.assertEqual({p[1] for p in pairs}, {1.0, 2.0})
            self.assertEqual(set(ds.meta.tasks.index), {"cut the carton tape"})
            img = np.asarray(ds[n // 2]["observation.images.head"])
            np.testing.assert_allclose(img.reshape(3, -1).mean(axis=1), [50 / 255, 100 / 255, 200 / 255], atol=0.08)

    def test_stopped_execution_is_discarded_and_the_next_one_is_saved(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.launch(root, "dual-cut")
            g = self.graph
            g.set(WAITING, "A")
            time.sleep(0.2)
            g.set(EXECUTING, "A")
            time.sleep(1.0)
            g.set(STOPPED, "A", True, "monitor: cross distance 0.02 m")
            self.assertTrue(r.wait_for("✕ discarded", timeout=20), r.text)
            self.assertIn("stopped: monitor: cross distance 0.02 m", r.text)
            time.sleep(0.5)
            g.set(IDLE)                               # ~/rearm
            g.set(PLANNED)
            time.sleep(0.6)
            self.run_execution(g, two_phases=False)
            self.assertTrue(r.wait_for("saved. total episodes: 1", timeout=60), r.text)
            self.quit(r)
            ds = LeRobotDataset("local/ds", root=Path(root) / "ds")
            self.assertEqual(ds.meta.total_episodes, 1)
            self.assertEqual(set(ds.meta.tasks.index), {"cut the carton tape"})

    def test_keep_stopped_saves_with_a_task_suffix(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.launch(root, "dual-cut", "--keep-stopped", "--task", "cut it")
            g = self.graph
            g.set(WAITING, "A")
            time.sleep(0.2)
            g.set(EXECUTING, "A")
            time.sleep(1.0)
            g.set(STOPPED, "A", True, "operator ~/stop")
            self.assertTrue(r.wait_for("saved. total episodes: 1", timeout=60), r.text)
            self.quit(r)
            ds = LeRobotDataset("local/ds", root=Path(root) / "ds")
            self.assertEqual(ds.meta.total_episodes, 1)
            self.assertEqual(set(ds.meta.tasks.index), {"cut it [stopped: operator ~/stop]"})
            self.assertEqual(ds[0]["task"], "cut it [stopped: operator ~/stop]")

    def test_enter_overrides_the_trigger_and_a_silent_jtc_is_counted_not_invented(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.launch(root, "dual-cut")
            g = self.graph
            g.set(WAITING, "A")
            time.sleep(0.2)
            g.set(EXECUTING, "A")
            self.assertTrue(r.wait_for("RECORDING", timeout=20), r.text)
            time.sleep(0.8)
            g.jtc_silent.set()                        # CRI MoveJoint transit: controller_state stops
            time.sleep(1.0)
            r.send("\n")                              # Enter = save early, while the coordinator is still EXECUTING
            self.assertTrue(r.wait_for("saved. total episodes: 1", timeout=60), r.text)
            self.assertRegex(r.text, r"\d+ with a stale action")
            self.assertNotIn(" 0 with a stale action", r.text)
            g.set(PLANNED)                            # the late end of that execution does not touch anything
            time.sleep(0.5)
            self.quit(r)
            self.assertNotIn("total episodes: 2", r.text)


@unittest.skipUnless(HAVE_ROS and HAVE_LEROBOT, "needs rclpy (source /opt/ros/humble) and lerobot")
class HCutRecordingTests(CutRecordingBase):
    DUAL = False

    def sequence(self, g, fail_at=None):
        g.hcut(0, "sequence", "running", "T cut sequence dispatched (4 steps)")
        for i, name in enumerate(["approach_standoff", "side_cut_penetrate", "continue_side_cut", "continue_top_cut"], 1):
            g.hcut(i, name, "running")
            time.sleep(0.5)
            if fail_at == i:
                g.hcut(i, name, "failed", "execution did not complete")
                return
            g.hcut(i, name, "done")
            time.sleep(0.15)      # the real sequencer settles between steps (t_cut_settle_timeout_s)
        g.hcut(5, "sequence", "done", "all 4 steps completed")

    def test_sequence_saved_failure_discarded_single_arm_names(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.launch(root, "h-cut", "--arms", "right")
            g = self.graph
            self.sequence(g, fail_at=2)
            self.assertTrue(r.wait_for("✕ discarded", timeout=20), r.text)
            self.assertIn("side_cut_penetrate", r.text)
            time.sleep(0.3)
            self.later(0.6, lambda: self.read_only_graph(
                "dataset_recorder_moveit", [HCUT_STATUS, "/joint_states", "/rebel_arm_trajectory_controller/controller_state"]))
            t0 = time.monotonic()
            self.sequence(g)
            wall = time.monotonic() - t0
            self.assertTrue(r.wait_for("saved. total episodes: 1", timeout=60), r.text)
            self.quit(r)
            ds = LeRobotDataset("local/ds", root=Path(root) / "ds")
            self.assertEqual(ds.meta.total_episodes, 1)
            self.assertEqual(ds.meta.robot_type, "igus_rebel_single_arm_real_cut")
            self.assertEqual(list(ds.meta.features["observation.cut_progress"]["names"]), ct.FEATURE_NAMES["h-cut"])
            n = len(ds)
            self.assertTrue(0.7 * wall * 30 <= n <= 1.15 * wall * 30 + 5, f"{n} frames for {wall:.1f} s")
            hf = ds.hf_dataset.with_format(None)
            state = np.asarray(hf["observation.state"], np.float32)
            action = np.asarray(hf["action"], np.float32)
            np.testing.assert_allclose(state[:, 7:13], np.tile(STATE_RIGHT, (n, 1)), atol=1e-6)   # right = --arms side
            np.testing.assert_allclose(action[:, 7:13], np.tile(REF_RIGHT, (n, 1)), atol=1e-6)
            np.testing.assert_allclose(state[:, 0:6], 0.0, atol=1e-6)                              # undriven left
            seen = list(dict.fromkeys(tuple(p) for p in np.asarray(hf["observation.cut_progress"], np.float32)))
            self.assertIn(seen[0], [(0.0, 1.0), (1.0, 1.0)], seen)   # step 1 follows the dispatch within microseconds
            self.assertIn((1.0, 1.0), seen)
            self.assertIn((3.0, 2.0), seen)
            self.assertEqual(set(ds.meta.tasks.index), {"cut the carton tape"})

    def test_left_side_mapping(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.launch(root, "h-cut", "--arms", "left")
            self.sequence(self.graph)
            self.assertTrue(r.wait_for("saved. total episodes: 1", timeout=60), r.text)
            self.quit(r)
            ds = LeRobotDataset("local/ds", root=Path(root) / "ds")
            state = np.asarray(ds.hf_dataset.with_format(None)["observation.state"], np.float32)
            np.testing.assert_allclose(state[:, 0:6], np.tile(STATE_RIGHT, (len(ds), 1)), atol=1e-6)  # joint1..6 -> left


if __name__ == "__main__":
    unittest.main()
