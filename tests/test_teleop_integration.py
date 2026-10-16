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
            args, _ = teleop.parse_args(["--backend", "real", "--arms", "left", "--quiet", "--no-park",
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

    def test_park_skipped_without_ethernet_link(self):
        args, _ = teleop.parse_args(["--backend", "real", "--dry-run"])
        self.assertTrue(args.dry_run)  # dry run never parks (park_on_exit returns before any motion)

        class Stub:
            ethernet = None
        teleop.park_on_exit(Stub(), args, None)  # no Ethernet link: nothing to do, no error


if __name__ == "__main__":
    unittest.main()
