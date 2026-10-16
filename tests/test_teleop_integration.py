"""Run with the sourced Quest / ROS environment for ROS-node integration coverage."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "quest_teleop"))
try:
    import numpy as np
    import rclpy
    import quest_dual_arm_teleop as teleop
except ImportError:
    teleop = None

from test_ethernet_rebel import FakeRobot


@unittest.skipIf(teleop is None, "source ROS and use .venv_quest/bin/python for node integration")
class NodeIntegrationTests(unittest.TestCase):
    def test_default_dual_ethernet_and_no_home(self):
        args, _ = teleop.parse_args(["--backend", "real", "--dry-run"])
        self.assertEqual(args.arms, ["left", "right"])
        self.assertTrue(args.no_home)
        self.assertEqual(args.max_joint_vel, .35)

    def test_invalid_unsafe_arguments_are_rejected(self):
        cases = [["--backend", "real", "--real-home"],
                 ["--backend", "real", "--rate", "0"],
                 ["--backend", "real", "--max-joint-vel", "nan"],
                 ["--backend", "real", "--command-timeout", "2"],
                 ["--arms", "left", "left"], ["--backend", "real-ros"]]
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
            args, _ = teleop.parse_args(["--backend", "real", "--dry-run"])
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
            args, _ = teleop.parse_args(["--backend", "real", "--quiet", "--no-park"])
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
            args, _ = teleop.parse_args(["--backend", "real", "--quiet", "--park-vel-deg", "30"])
            self.assertEqual(args.park_deg, list(teleop.HOME_DEG))  # default park pose = home (0, 0, 65, 0, 50, 0)
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            with patch.object(teleop, "load_endpoints", return_value={"left": left.endpoint, "right": right.endpoint}):
                node = teleop.DualArmTeleop(args, chain)

            class IdleQuest:  # operator never grips, then quits (Ctrl+C / Stop all)
                buttons = {}
                ticks = 0

                def __init__(self, *unused):
                    pass

                def read_all(self):
                    self.ticks += 1
                    if self.ticks > 3:
                        raise KeyboardInterrupt
                    idle = (True, np.zeros(3), np.array([0, 0, 0, 1]), 0.0, 0.0)
                    return {"left": idle, "right": idle}

                def close(self):
                    pass

            with patch.object(teleop, "QuestController", return_value=IdleQuest()):
                teleop.run_session(node, chain, args)
            for robot in (left, right):
                np.testing.assert_allclose(robot.joints_deg, teleop.HOME_DEG, atol=1e-6)
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
            args, _ = teleop.parse_args(["--backend", "real", "--quiet", "--park-vel-deg", "30"])
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
                np.testing.assert_allclose(robot.joints_deg, teleop.HOME_DEG, atol=1e-6)
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
        args, _ = teleop.parse_args(["--backend", "real", "--arms", "left"])
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

    def test_park_skipped_without_ethernet_link(self):
        args, _ = teleop.parse_args(["--backend", "real", "--dry-run"])
        self.assertTrue(args.dry_run)  # dry run never parks (park_on_exit returns before any motion)

        class Stub:
            ethernet = None
        teleop.park_on_exit(Stub(), args, None)  # no Ethernet link: nothing to do, no error


if __name__ == "__main__":
    unittest.main()
