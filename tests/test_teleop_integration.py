"""Run with the sourced Quest / ROS environment for ROS-node integration coverage."""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402,F401  - FIRST: never share the robot's ROS graph
import io
from pathlib import Path
import sys
import time
import unittest
import os
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "quest_teleop"))
try:
    import numpy as np
    import rclpy
    import rclpy.executors
    import quest_dual_arm_teleop as teleop
except ImportError:
    teleop = None

from test_ethernet_rebel import FakeRobot


@unittest.skipIf(teleop is None, "source ROS and use .venv_quest/bin/python for node integration")
class NodeIntegrationTests(unittest.TestCase):
    def test_default_dual_ethernet_and_no_home(self):
        args, _ = teleop.parse_args(["--backend", "real", "--control", "position", "--dry-run"])
        self.assertEqual(args.arms, ["left", "right"])
        self.assertTrue(args.no_home)
        self.assertEqual(args.max_joint_vel, .35)

    def test_invalid_unsafe_arguments_are_rejected(self):
        cases = [["--backend", "real", "--control", "position", "--real-home"],
                 ["--backend", "real", "--control", "position", "--rate", "0"],
                 ["--backend", "real", "--control", "position", "--max-joint-vel", "nan"],
                 ["--backend", "real", "--control", "position", "--command-timeout", "2"],
                 ["--arms", "left", "left"]]
        for argv in cases:
            with self.subTest(argv=argv), patch("sys.stderr"), self.assertRaises(SystemExit):
                teleop.parse_args(argv)

    def test_sim_and_legacy_modes_remain_available(self):
        sim, _ = teleop.parse_args([])
        legacy, _ = teleop.parse_args(["--backend", "real-ros", "--arms", "right"])
        self.assertEqual(sim.backend, "isaac")
        self.assertEqual(sim.max_joint_vel, .785)
        self.assertTrue(legacy.no_home)

    def test_ros_node_uses_independent_ethernet_feedback_and_no_trajectory_publisher(self):
        left, right = FakeRobot(), FakeRobot()
        right.joints_deg = [1, 2, 3, 4, 5, 6]
        node = None
        rclpy.init()
        try:
            args, _ = teleop.parse_args(["--backend", "real", "--control", "position", "--dry-run"])
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            with patch.object(teleop, "load_endpoints", return_value={"left": left.endpoint, "right": right.endpoint}):
                node = teleop.DualArmTeleop(args, chain)
            node.wait_ready()
            self.assertFalse(hasattr(node, "traj_pub"))
            self.assertFalse(hasattr(node, "cmd_pub"))
            self.assertIsNot(node.arms["left"].q_meas, node.arms["right"].q_meas)
            np.testing.assert_allclose(node.arms["left"].q_meas, np.zeros(6))
            np.testing.assert_allclose(node.arms["right"].q_meas, np.deg2rad([1, 2, 3, 4, 5, 6]))
            node.send({"left": np.ones(6) * .001, "right": np.ones(6) * -.002})
            self.assertFalse(left.targets or right.targets)
            self.assertTrue(all(a.q_meas.shape == (6,) for a in node.arms.values()))
        finally:
            if node:
                node.destroy_node()
            rclpy.try_shutdown()
            left.close()
            right.close()

    def test_real_urdf_fk_and_rate_limit(self):
        chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                   tcp_xyz=teleop.TCP_PRESETS["flange"])
        # Read-only controller observation: right arm with no configured tool, 2026-10-15.
        q = np.deg2rad([-4.4, 13.11, -4.82, 0, -8.25, 4.41])
        observed = np.array([.0978, -.0075, .9133])
        self.assertLess(np.linalg.norm(chain.fk(q)[:3, 3] - observed), .003)
        delta = teleop.step_toward(q, q + .1, .35 / 30) - q
        self.assertLessEqual(float(np.max(np.abs(delta))), .35 / 30 + 1e-10)

    def test_quest_ik_session_dual_grip_release_tracking_loss_and_regrip(self):
        left, right = FakeRobot(), FakeRobot()
        left.joints_deg = list(teleop.HOME_DEG)
        right.joints_deg = list(teleop.HOME_DEG)
        node = None
        rclpy.init()
        try:
            args, _ = teleop.parse_args(["--backend", "real", "--control", "position", "--quiet", "--no-park", "--no-start-home"])
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            with patch.object(teleop, "load_endpoints", return_value={"left": left.endpoint, "right": right.endpoint}):
                node = teleop.DualArmTeleop(args, chain)

            class FakeQuest:
                buttons = {}
                closed = False
                tick = 0
                snapshots = []

                def __init__(self, *unused):
                    pass

                def read_all(self):
                    self.snapshots.append(tuple(a.moving for a in node.ethernet.arms.values()))
                    i = self.tick
                    self.tick += 1
                    if i == 8:
                        raise KeyboardInterrupt
                    # Engage both, lose left tracking, keep holding (must stay disabled),
                    # release to re-arm, engage again, then release both.
                    tracked_left = i != 3
                    squeeze = 0.0 if i in (0, 5, 7) else 1.0
                    return {"left": (tracked_left, np.array([.001 if i == 2 else 0, 0, 0]),
                                     np.array([0, 0, 0, 1]), squeeze, 0),
                            "right": (True, np.array([0, .002 if i == 2 else 0, 0]),
                                      np.array([0, 0, 0, 1]), squeeze, 0)}

                def close(self):
                    self.closed = True

            quest = FakeQuest()
            with patch.object(teleop, "QuestController", return_value=quest):
                teleop.run_session(node, chain, args)
            self.assertTrue(quest.closed)
            self.assertEqual(quest.snapshots[3], (True, True))
            self.assertEqual(quest.snapshots[4], (False, True))
            self.assertEqual(quest.snapshots[5], (False, True))
            self.assertEqual(quest.snapshots[6], (False, False))
            self.assertEqual(quest.snapshots[7], (True, True))
            self.assertEqual(quest.snapshots[8], (False, False))
            self.assertTrue(left.targets and right.targets)
            self.assertTrue(all(a.position is None for a in node.ethernet.arms.values()))
        finally:
            if node:
                node.destroy_node()
            rclpy.try_shutdown()
            left.close()
            right.close()

    def test_clean_exit_returns_both_arms_to_zero(self):
        left, right = FakeRobot(), FakeRobot()
        left.joints_deg = [0.0] * 6
        right.joints_deg = [10, 20, 80, 0, 40, -10]
        for robot in (left, right):
            robot.follow = True
        node = None
        rclpy.init()
        try:
            args, _ = teleop.parse_args(["--backend", "real", "--control", "position", "--quiet", "--park-vel-deg", "30"])
            self.assertEqual(args.park_deg, [0.0] * 6)  # start: home (0, 0, 65, 0, 50, 0); exit: all 0
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            with patch.object(teleop, "load_endpoints", return_value={"left": left.endpoint, "right": right.endpoint}):
                node = teleop.DualArmTeleop(args, chain)

            at_start = []

            class IdleQuest:  # operator never grips, then quits (Ctrl+C / Stop all)
                buttons = {}
                ticks = 0

                def __init__(self, *unused):
                    pass

                def read_all(self):
                    self.ticks += 1
                    if self.ticks == 1:  # the Quest starts only after the move to the home pose
                        at_start.extend([list(left.joints_deg), list(right.joints_deg)])
                    if self.ticks > 3:
                        raise KeyboardInterrupt
                    idle = (True, np.zeros(3), np.array([0, 0, 0, 1]), 0.0, 0.0)
                    return {"left": idle, "right": idle}

                def close(self):
                    pass

            with patch.object(teleop, "QuestController", return_value=IdleQuest()):
                teleop.run_session(node, chain, args)
            self.assertEqual(len(at_start), 2)
            for pose in at_start:
                np.testing.assert_allclose(pose, teleop.HOME_DEG, atol=1e-6)
            for robot in (left, right):
                np.testing.assert_allclose(robot.joints_deg, np.zeros(6), atol=1e-6)
                self.assertFalse(robot.in_use)
            self.assertTrue(all(a.position is None for a in node.ethernet.arms.values()))
        finally:
            if node:
                node.destroy_node()
            rclpy.try_shutdown()
            left.close()
            right.close()

    def test_sigint_during_session_ends_cleanly_and_parks(self):
        import os
        import signal
        left, right = FakeRobot(), FakeRobot()
        left.joints_deg = [0, 10, 60, 0, 30, 0]
        right.joints_deg = [5, 10, 60, 0, 30, -5]
        for robot in (left, right):
            robot.follow = True
        node = None
        rclpy.init()
        try:
            args, _ = teleop.parse_args(["--backend", "real", "--control", "position", "--quiet", "--park-vel-deg", "30"])
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            with patch.object(teleop, "load_endpoints", return_value={"left": left.endpoint, "right": right.endpoint}):
                node = teleop.DualArmTeleop(args, chain)

            class SignalQuest:  # Stop all = SIGTERM / Ctrl+C = SIGINT arriving between Quest reads
                buttons = {}
                ticks = 0
                closed = False

                def __init__(self, *unused):
                    pass

                def read_all(self):
                    self.ticks += 1
                    if self.ticks == 3:
                        os.kill(os.getpid(), signal.SIGTERM)
                    idle = (True, np.zeros(3), np.array([0, 0, 0, 1]), 0.0, 0.0)
                    return {"left": idle, "right": idle}

                def close(self):
                    self.closed = True

            quest = SignalQuest()
            with patch.object(teleop, "QuestController", return_value=quest):
                teleop.run_session(node, chain, args)
            self.assertTrue(quest.closed)
            for robot in (left, right):
                np.testing.assert_allclose(robot.joints_deg, np.zeros(6), atol=1e-6)
        finally:
            signal.signal(signal.SIGINT, signal.default_int_handler)
            signal.signal(signal.SIGTERM, signal.SIG_DFL)
            if node:
                node.destroy_node()
            rclpy.try_shutdown()
            left.close()
            right.close()

    def test_park_errors_never_escape(self):
        import threading

        class Link:
            fault = None
            _closed = threading.Event()

            def park(self, *a, **k):
                raise BrokenPipeError("robot closed")
        chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS)
        stub = type("Node", (), {"ethernet": Link(), "arms": {"left": type("A", (), {"chain": chain})()}})()
        args, _ = teleop.parse_args(["--backend", "real", "--control", "position", "--arms", "left"])
        logged = []
        log = type("L", (), {"error": lambda self, m: logged.append(m), "warning": lambda self, m: None})()
        teleop.park_on_exit(stub, args, log)  # must not raise
        self.assertIn("failed", logged[-1])
        args.park_deg = [0, 0, 0, 0, 0, 400]
        teleop.park_on_exit(stub, args, log)
        self.assertIn("outside the joint limits", logged[-1])
        import signal
        signal.signal(signal.SIGINT, signal.default_int_handler)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

    def _drive_left_toward_right(self, extra_args, right_deg, ticks=110, direction=(0.0, -1.0)):
        """Left arm driven by a fake Quest hand moving 40 cm toward the right arm, which is not driven and stands
        at right_deg. Returns (min clearance over the run, first clearance, right-arm CRI commands)."""
        from arm_collision import capsules, clearance, load_world_T_base
        left, right = FakeRobot(), FakeRobot()
        left.joints_deg = [0, 30, 60, 0, 40, 0]
        right.joints_deg = list(right_deg)
        left.follow = True
        node = None
        rclpy.init()
        try:
            args, _ = teleop.parse_args(["--backend", "real", "--control", "position", "--arms", "left", "--quiet", "--no-park", "--no-start-home",
                                         "--max-lin-vel", "0.3", "--max-joint-vel", "0.6", *extra_args])
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            with patch.object(teleop, "load_endpoints", return_value={"left": left.endpoint, "right": right.endpoint}):
                node = teleop.DualArmTeleop(args, chain)
                W = load_world_T_base()
                rcaps = capsules(chain, np.deg2rad(right.joints_deg), W["right"], .06, .04)
                seen = []

                class TowardRight:
                    buttons = {}
                    tick = 0

                    def __init__(self, *unused):
                        pass

                    def read_all(self):
                        self.tick += 1
                        if self.tick > ticks:
                            raise KeyboardInterrupt
                        seen.append(clearance(capsules(chain, np.deg2rad(left.joints_deg), W["left"], .06, .04), rcaps))
                        k = min(0.6, max(0, self.tick - 3) * 0.006)  # operator -y = right = toward the right arm
                        squeeze = 0.0 if self.tick < 3 else 1.0
                        hand = (True, np.array([direction[0] * k, direction[1] * k, 0]), np.array([0, 0, 0, 1]),
                                squeeze, 0)
                        return {"left": hand, "right": hand}

                    def close(self):
                        pass
                with patch.object(teleop, "QuestController", return_value=TowardRight()):
                    teleop.run_session(node, chain, args)
            return min(seen), seen[0], list(right.commands)
        finally:
            import signal
            signal.signal(signal.SIGINT, signal.default_int_handler)
            signal.signal(signal.SIGTERM, signal.SIG_DFL)
            if node:
                node.destroy_node()
            rclpy.try_shutdown()
            left.close()
            right.close()

    def test_unopened_other_arm_is_assumed_at_zero_and_never_contacted(self):
        guarded, start, right_cmds = self._drive_left_toward_right([], [0.0] * 6, ticks=160, direction=(-1.0, -1.0))
        self.assertGreater(start, 0.15)
        self.assertGreaterEqual(guarded, 0.05 - 0.01)  # held at the 5 cm margin to the (assumed) straight-up arm
        self.assertLess(guarded, 0.08)
        self.assertFalse(right_cmds)  # the other arm's controller is not even connected

    def test_watched_other_arm_stops_the_driven_arm_before_contact(self):
        guarded, start, right_cmds = self._drive_left_toward_right(["--watch-other"], teleop.HOME_DEG)
        self.assertGreater(start, 0.15)
        self.assertGreaterEqual(guarded, 0.05 - 0.01)  # held at the 5 cm surface clearance (one step of margin)
        self.assertLess(guarded, 0.08)                 # ...after really approaching it
        # the watched arm was only read: no Position Interface, no motion, and it handed control back
        self.assertIn(("CMD", "SetActive", "false"), right_cmds)
        self.assertFalse(any(c[:2] in (("CMD", "UsePositionInterface"), ("CONFIG", "SetPositionInterface"))
                             or c[1:2] in (("Enable",), ("Reset",)) for c in right_cmds))

    def test_calibrated_bases_and_capsule_clearance(self):
        from arm_collision import capsules, clearance, load_world_T_base
        W = load_world_T_base()
        rel = np.linalg.inv(W["left"]) @ W["right"]
        self.assertAlmostEqual(np.linalg.norm(rel[:3, 3]), 0.448, delta=0.002)  # measured arm spacing
        chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS)
        cap = lambda side, qd: capsules(chain, np.deg2rad(qd), W[side], .06, .04)
        self.assertGreater(clearance(cap("left", teleop.HOME_DEG), cap("right", teleop.HOME_DEG)), 0.2)
        self.assertLess(clearance(cap("left", [-90, 60, 60, 0, 0, 0]), cap("right", teleop.HOME_DEG)), 0.0)

    def test_jog_session_home_at_start_zero_at_exit(self):
        left, right = FakeRobot(), FakeRobot()
        left.joints_deg = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]
        right.joints_deg = [5.0, 0.0, 0.0, 0.0, 10.0, 0.0]
        node = None
        rclpy.init()
        try:
            args, _ = teleop.parse_args(["--backend", "real", "--quiet", "--park-vel-deg", "18"])
            self.assertEqual(args.control, "jog")
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            with patch.object(teleop, "load_endpoints", return_value={"left": left.endpoint, "right": right.endpoint}):
                node = teleop.DualArmTeleop(args, chain)
            at_start = []

            class IdleQuest:
                buttons = {}
                ticks = 0

                def __init__(self, *unused):
                    pass

                def read_all(self):
                    self.ticks += 1
                    if self.ticks == 1:
                        at_start.extend([list(left.joints_deg), list(right.joints_deg)])
                    if self.ticks > 3:
                        raise KeyboardInterrupt
                    idle = (True, np.zeros(3), np.array([0, 0, 0, 1]), 0.0, 0.0)
                    return {"left": idle, "right": idle}

                def close(self):
                    pass
            with patch.object(teleop, "QuestController", return_value=IdleQuest()):
                teleop.run_session(node, chain, args)
            for pose in at_start:
                self.assertLess(np.max(np.abs(np.array(pose) - teleop.HOME_DEG)), 1.0, pose)
            for robot in (left, right):
                self.assertLess(np.max(np.abs(robot.joints_deg)), 0.5, robot.joints_deg)
                self.assertFalse(any(robot.last_jog))
                self.assertIn(("CMD", "MotionTypeJoint"), robot.commands)
        finally:
            import signal
            signal.signal(signal.SIGINT, signal.default_int_handler)
            signal.signal(signal.SIGTERM, signal.SIG_DFL)
            if node:
                node.destroy_node()
            rclpy.try_shutdown()
            left.close()
            right.close()

    def test_rotation_frame_switch_applies_at_the_next_grip(self):
        left = FakeRobot()
        left.joints_deg = list(teleop.HOME_DEG)
        node = None
        rclpy.init()
        try:
            args, _ = teleop.parse_args(["--backend", "real", "--control", "position", "--arms", "left", "--quiet",
                                         "--no-park", "--no-start-home"])
            self.assertEqual(args.rot_frame, "tool")
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            with patch.object(teleop, "load_endpoints", return_value={"left": left.endpoint}):
                node = teleop.DualArmTeleop(args, chain)
            frames = []

            class SwitchQuest:  # grip, switch to base while gripping, release, grip again, switch back
                buttons = {}
                tick = 0

                def __init__(self, *unused):
                    pass

                def read_all(self):
                    i = self.tick
                    self.tick += 1
                    arm = node.arms["left"]
                    frames.append((arm.engaged, arm.rot_frame, node.rot_frame))
                    if i == 3:
                        node.read_commands(io.StringIO("rot_frame base\nbogus\nrot_frame sideways\n"))
                    if i == 7:
                        node.set_rot_frame("tool")
                    if i == 9:
                        raise KeyboardInterrupt
                    squeeze = 0.0 if i in (0, 5) else 1.0
                    return {"left": (True, np.zeros(3), np.array([0, 0, 0, 1]), squeeze, 0)}

                def close(self):
                    pass

            with patch.object(teleop, "QuestController", return_value=SwitchQuest()):
                teleop.run_session(node, chain, args)
            self.assertEqual(frames[2], (True, "tool", "tool"))   # engaged in the start frame
            self.assertEqual(frames[4], (True, "tool", "base"))   # switched while gripping: arm keeps tool
            self.assertEqual(frames[7], (True, "base", "base"))   # re-gripped: base frame
            self.assertEqual(frames[8], (True, "base", "tool"))   # bad commands ignored; switch back pending
        finally:
            if node:
                node.destroy_node()
            rclpy.try_shutdown()
            left.close()

    def test_sim_joint_state_is_the_newest_with_one_spin_per_tick(self):
        """Isaac publishes ~60 Hz, the loop takes one callback per 30 Hz tick: q_meas must not trail a queue."""
        from sensor_msgs.msg import JointState
        rclpy.init()
        node = pub_node = None
        try:
            args, _ = teleop.parse_args(["--quiet"])
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            node = teleop.DualArmTeleop(args, chain)
            pub_node = rclpy.create_node("fake_isaac_states")
            pub = pub_node.create_publisher(JointState, "/dual_arm/isaac_joint_states", 10)
            names = [f"arm_{s}_{j}" for s in teleop.SIDES for j in teleop.ARM_JOINTS]
            k = 0
            for _ in range(40):
                for _ in range(2):  # two states per teleop tick
                    k += 1
                    pub.publish(JointState(name=names, position=[k * 1e-3] * 12))
                time.sleep(0.02)
                rclpy.spin_once(node, timeout_sec=0.0)
            self.assertGreaterEqual(node.arms["left"].q_meas[0], (k - 2) * 1e-3 - 1e-9)  # was ~10 states behind
            self.assertGreater(node.stats.js_n, 0)
        finally:
            for n in (node, pub_node):
                if n:
                    n.destroy_node()
            rclpy.try_shutdown()

    def test_isaac_command_subscriber_keeps_only_the_newest(self):
        src = (Path(__file__).resolve().parents[1] / "dual_arm_sim/run_dual_arm_sim.py").read_text()
        self.assertIn('("sub.inputs:queueSize", 1)', src)

    def test_loop_stats_flags_a_slow_loop_and_slow_sim(self):
        from builtin_interfaces.msg import Time
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header
        st = teleop.LoopStats(30.0, 1.0)
        t0 = st.t_start
        st.on_state(JointState(header=Header(stamp=Time(sec=10))))
        st.reset(t0)
        for _ in range(15):  # 15 Hz loop over 1 s
            st.tick({"tick": .05, "xr": .01, "ik": .03, "pub": .001, "solve": .02, "guard": .005}, .02)
        st.on_state(JointState(header=Header(stamp=Time(sec=10, nanosec=500_000_000))))
        self.assertIsNone(st.report(t0 + .5))
        line, bad = st.report(t0 + 1.0)
        self.assertTrue(bad)
        self.assertIn("15 overruns", line)
        self.assertIn("sim  50 % real time", line)

    def test_timing_line_can_switch_between_warn_and_info(self):
        rclpy.init()
        node = rclpy.create_node("timing_log_test")
        try:
            for bad in (True, False, True, False):  # crashed with ValueError on the first switch
                teleop.log_timing(node.get_logger(), "[timing] test", bad)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()

    def test_y_button_resets_the_sim_scene(self):
        """Left Y while an arm is gripped: 'reset' event, arm released and back at HOME_DEG, then /dual_arm/reset_scene."""
        import threading
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Empty, String
        rclpy.init()
        node = fake = None
        try:
            args, _ = teleop.parse_args(["--quiet", "--no-home", "--arms", "left"])
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            node = teleop.DualArmTeleop(args, chain)
            fake = rclpy.create_node("fake_isaac")
            names = [f"arm_{s}_{j}" for s in teleop.SIDES for j in teleop.ARM_JOINTS]
            q = {n: v for n, v in zip(names, np.deg2rad([0, 20, 70, 0, 40, 0] * 2))}  # not at home
            events, resets = [], []
            fake.create_subscription(JointState, "/dual_arm/isaac_joint_commands",
                                     lambda m: q.update({n: v for n, v in zip(m.name, m.position) if n in q}), 50)
            fake.create_subscription(String, "/quest_teleop/events", lambda m: events.append(m.data), 10)
            fake.create_subscription(Empty, "/dual_arm/reset_scene", lambda m: resets.append(1), 10)
            pub = fake.create_publisher(JointState, "/dual_arm/isaac_joint_states", 10)
            stop = []

            ex = rclpy.executors.SingleThreadedExecutor()  # own executor: the teleop spins the global one
            ex.add_node(fake)

            def isaac():
                while not stop:
                    ex.spin_once(timeout_sec=0.0)
                    pub.publish(JointState(name=names, position=[q[n] for n in names]))
                    time.sleep(0.005)
            threading.Thread(target=isaac, daemon=True).start()
            seen = {}

            class YQuest:
                tick = 0
                buttons = {"left": (False, False)}

                def __init__(self, *unused):
                    pass

                def read_all(self):
                    self.tick += 1
                    i = self.tick
                    if i == 3:
                        seen["engaged_before"] = node.arms["left"].engaged
                    self.buttons = {"left": (False, i == 4)}  # Y pressed at tick 4
                    if i == 8:
                        seen["engaged_after"] = node.arms["left"].engaged
                        seen["q"] = node.arms["left"].q_meas.copy()
                        raise KeyboardInterrupt
                    return {"left": (True, np.zeros(3), np.array([0, 0, 0, 1.0]), 1.0 if i < 4 else 0.0, 0.0)}

                def close(self):
                    pass

            with patch.object(teleop, "QuestController", return_value=YQuest()):
                teleop.run_session(node, chain, args)
            time.sleep(0.2)
            stop.append(1)
            self.assertTrue(seen["engaged_before"])
            self.assertFalse(seen["engaged_after"])
            self.assertIn("reset", events)
            self.assertEqual(len(resets), 1)
            np.testing.assert_allclose(np.rad2deg(seen["q"]), teleop.HOME_DEG, atol=1.0)
        finally:
            for n in (node, fake):
                if n:
                    n.destroy_node()
            rclpy.try_shutdown()

    def test_real_ros_one_arm_action_includes_assumed_other_arm_and_y_dry_run_only_discards(self):
        """record_dataset.py --source real needs the 14-D action also in a one-arm session (the undriven arm at its
        assumed pose), and Y on a real dry run discards the open episode without moving anything."""
        node = None
        rclpy.init()
        try:
            args, _ = teleop.parse_args(["--backend", "real-ros", "--arms", "left", "--dry-run",
                                         "--other-arm-deg", "0", "10", "20", "0", "0", "0"])
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            node = teleop.DualArmTeleop(args, chain)
            sent = {"action": [], "events": [], "traj": []}
            node.action_pub.publish = sent["action"].append
            node.event_pub.publish = sent["events"].append
            node.traj_pub["left"].publish = sent["traj"].append
            from sensor_msgs.msg import JointState
            node._on_real_js("left", JointState(name=teleop.ARM_JOINTS, position=[.1] * 6))  # /left/joint_states
            node.publish_action()
            self.assertEqual(len(sent["action"]), 1)
            d = dict(zip(sent["action"][0].name, sent["action"][0].position))
            self.assertEqual(len(d), 14)
            np.testing.assert_allclose([d[f"arm_right_joint{i}"] for i in range(1, 7)],
                                       np.deg2rad([0, 10, 20, 0, 0, 0]))
            np.testing.assert_allclose([d[f"arm_left_joint{i}"] for i in range(1, 7)], np.full(6, .1))
            teleop.reset_scene(node, args, node.get_logger())
            self.assertEqual([m.data for m in sent["events"]], ["reset"])
            self.assertEqual(sent["traj"], [])  # no arm command from a button
            self.assertIsNone(node.scene_reset_pub)
        finally:
            if node:
                node.destroy_node()
            rclpy.try_shutdown()

    def test_park_skipped_without_ethernet_link(self):
        args, _ = teleop.parse_args(["--backend", "real", "--control", "position", "--dry-run"])
        self.assertTrue(args.dry_run)  # dry run never parks (park_on_exit returns before any motion)

        class Stub:
            ethernet = None
        teleop.park_on_exit(Stub(), args, None)  # no Ethernet link: nothing to do, no error


if __name__ == "__main__":
    unittest.main()
