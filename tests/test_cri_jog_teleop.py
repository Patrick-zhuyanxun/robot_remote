"""Quest -> real ReBeL through the controller's own Cartesian jog (`--backend cri-jog`), against a FAKE igus driver.

The fake driver implements the interface of the real one as documented in igus_rebel_ros2 (README, "Teleoperation
through the controller's Cartesian jog"): the cart_jog_enable SetBool service (can refuse), cart_jog_cmd (stamp
checked like the driver: must be ~now, base frame only, per-axis clamp, 0.15 s watchdog, ignored unless the mode is
on), robot_cart_pose (mm) and hardware_state, with a first-order-lag plant that integrates the twist.
NOTHING here touches a real robot: the test process runs on its own ROS domain (ros_isolation).

Needs the igus_rebel_ros2 install that has igus_rebel_msgs (RobotCartPose / HardwareState) sourced.
"""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402  - FIRST: never share the robot's ROS graph
import contextlib
import csv
import io
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "quest_teleop"))
try:
    import numpy as np
    import rclpy
    import rclpy.executors
    from geometry_msgs.msg import TwistStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from igus_rebel_msgs.msg import HardwareState, RobotCartPose
    import quest_dual_arm_teleop as teleop
    import cri_jog
except ImportError:  # pragma: no cover
    teleop = None

F = False
MAX_VEL = 0.03


class FakeCriDriver:
    """The igus driver's Cartesian jog interface + a plant. Positions in metres (published in mm).
    ns: topic namespace ("igus_rebel" single instance, "igus_rebel_left" / "igus_rebel_right" in the dual stack).
    model: None (joints published as zeros) or {"chain": SerialChain, "rot": R_home, "q0": q}: the joints are the
    IK solution of the position (orientation held), so the arm guard sees real arm geometry. Only the fake uses IK."""

    def __init__(self, start_m=(0.30, -0.20, 0.50), refuse=None, gain=1.0, active=False, tau=0.15, delay_s=0.1,
                 ns="igus_rebel", model=None):
        self.node = rclpy.create_node(f"fake_cri_driver_{ns}")
        self.ns, self.model = ns, model
        self.pos = np.array(start_m, float)
        self.q = None if model is None else np.array(model["q0"], float)
        self.v, self.v_target = np.zeros(3), np.zeros(3)
        self.gain, self.tau, self.delay_s, self.refuse = gain, tau, delay_s, refuse
        self.active, self.pose_frozen = active, False
        self.kinstate, self.latched = 0, False
        self.ubuf = [(0.0, np.zeros(3))]
        self.last_valid = 0.0
        self.events = []      # ("cmd", t, v) every message received, ("enable", t, bool) every service call
        self.valid, self.rejected, self.ignored = 0, 0, 0
        self.hist = []        # (t, position)
        self.qhist = []       # (t, joints) when a model is used
        self.chat = []        # /quest_teleop/events
        self.pose_pub = self.node.create_publisher(RobotCartPose, f"/{ns}/robot_cart_pose",
                                                   QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability))
        self.hw_pub = self.node.create_publisher(HardwareState, f"/{ns}/hardware_state",
                                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_service(SetBool, f"/{ns}/cart_jog_enable", self._on_enable)
        self.node.create_subscription(TwistStamped, f"/{ns}/cart_jog_cmd", self._on_cmd,
                                      QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(String, "/quest_teleop/events", lambda m: self.chat.append(m.data), 10)
        self.ex = rclpy.executors.SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.stop = threading.Event()
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    # -- interface ------------------------------------------------------------------------------------------
    def _on_enable(self, req, res):
        self.events.append(("enable", time.monotonic(), bool(req.data)))
        if self.refuse:
            res.success, res.message = False, self.refuse
            return res
        self.active = bool(req.data)
        if not self.active:
            self.v_target[:] = 0.0
        res.success, res.message = True, "Cartesian jog mode " + ("ON" if self.active else "OFF")
        self._publish_hw()
        return res

    def _on_cmd(self, msg):
        now_ros = self.node.get_clock().now().nanoseconds * 1e-9
        v = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.events.append(("cmd", time.monotonic(), v))
        st = msg.header.stamp
        stamp = st.sec + st.nanosec * 1e-9
        age = now_ros - stamp
        if (stamp == 0.0 or age > 0.15 or age < -0.5 or msg.header.frame_id not in ("", "base_link")
                or np.any(np.asarray([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]) != 0.0)):
            self.rejected += 1
            return
        if not self.active:
            self.ignored += 1
            return
        self.valid += 1
        self.v_target = np.clip(v, -0.05, 0.05)  # cart_jog_max_percent 10 % at 200 %/(m/s)
        self.last_valid = time.monotonic()

    def leave_mode(self):
        self.active = False
        self.v_target[:] = 0.0
        self._publish_hw()

    # -- plant ----------------------------------------------------------------------------------------------
    def _publish_hw(self):
        m = HardwareState()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.component, m.connected, m.latched = "fake", True, self.latched
        m.cart_jog_active, m.cart_jog_state = self.active, "active" if self.active else "off"
        self.hw_pub.publish(m)

    def _publish_pose(self):
        m = RobotCartPose()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pos_cart_robot_raw = [float(x) * 1000.0 for x in self.pos] + [0.0, 0.0, 0.0]
        m.mode = "cartbase" if self.active else "joint"
        m.kinstate = self.kinstate
        if self.model is not None:
            t = np.eye(4)
            t[:3, :3], t[:3, 3] = self.model["rot"], self.pos
            self.q = self.model["chain"].ik(self.q, t, rot_weight=1.0)
            m.joint_position_rad = [float(x) for x in self.q]
            self.qhist.append((time.monotonic(), self.q.copy()))
        self.pose_pub.publish(m)

    def _run(self):
        last = last_pose = last_hw = time.monotonic()
        while not self.stop.is_set():
            self.ex.spin_once(timeout_sec=0.0)
            now = time.monotonic()
            dt, last = min(now - last, 0.05), now
            if self.active and now - self.last_valid > 0.15:
                self.v_target[:] = 0.0   # the driver's watchdog
            self.ubuf.append((now, self.v_target.copy() if self.active else np.zeros(3)))
            while len(self.ubuf) > 1 and self.ubuf[1][0] <= now - self.delay_s:
                self.ubuf.pop(0)
            self.v += (self.gain * self.ubuf[0][1] - self.v) * dt / self.tau
            self.pos = self.pos + self.v * dt
            self.hist.append((now, self.pos.copy()))
            if now - last_pose >= 0.01 and not self.pose_frozen:
                last_pose = now
                self._publish_pose()
            if now - last_hw >= 0.1:
                last_hw = now
                self._publish_hw()
            time.sleep(0.004)

    def close(self):
        self.stop.set()
        self.th.join()
        self.node.destroy_node()

    # -- analysis -------------------------------------------------------------------------------------------
    def cmds(self):
        return [(t, v) for k, t, v in self.events if k == "cmd"]

    def enables(self):
        return [v for k, _, v in self.events if k == "enable"]

    def nonzero_after(self, t, until=1e18):
        return [(tt, v) for tt, v in self.cmds() if t <= tt < until and np.linalg.norm(v) > 1e-9]

    def pos_at(self, t):
        return next((p for tt, p in self.hist if tt >= t), self.hist[-1][1])


class ScriptQuest:
    """Quest stand-in. script(t) -> (tracked, pos, squeeze, primary, secondary) for one hand, or a dict {hand: script}."""

    def __init__(self, script, duration, hand="right", probe=None):
        self.scripts = script if isinstance(script, dict) else {hand: script}
        self.duration, self.probe = duration, probe
        self.hand = next(iter(self.scripts))
        self.t0 = None
        self.buttons = {h: (False, False) for h in self.scripts}
        self.closed = False
        self.reads = []   # (monotonic, tracked, squeeze) of the first hand

    def read_all(self):
        if self.t0 is None:
            self.t0 = time.monotonic()
        t = time.monotonic() - self.t0
        if t > self.duration:
            raise KeyboardInterrupt
        if self.probe:
            self.probe(t, self)
        out = {}
        for h, script in self.scripts.items():
            tracked, pos, squeeze, prim, sec = script(t)
            self.buttons[h] = (prim, sec)
            out[h] = (tracked, None if pos is None else np.asarray(pos, float), np.array([0, 0, 0, 1.0]), squeeze, 0.0)
            if h == self.hand:
                self.reads.append((time.monotonic(), tracked, squeeze))
        return out

    def close(self):
        self.closed = True

    def first_read(self, pred):
        return next(t for t, tr, sq in self.reads if pred(tr, sq))


def grip_then_move(t, vec=(0.06, 0.03, 0.0), t_grip=0.4, t_ramp=1.0, squeeze=1.0):
    """Open hand first (the arm needs a release before the first grip), then grip while the hand moves by vec."""
    if t < t_grip:
        return True, [0, 0, 0], 0.0, F, F
    k = min((t - t_grip) / t_ramp, 1.0)
    return True, [vec[0] * k, vec[1] * k, vec[2] * k], squeeze, F, F


def drift(t):  # open hand first, then grip while the hand drifts forward at 2 cm/s: an engaged arm always has work
    return True, [0.02 * t, 0, 0], (0.0 if t < 0.4 else 1.0), F, F


@unittest.skipIf(teleop is None, "source ROS and the igus_rebel_ros2 install (igus_rebel_msgs) to run the cri-jog tests")
class CriJogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = cri_jog.make_urdf()   # once: the arm guard builds its chain from it (patched into every session)
        cls.chain = teleop.SerialChain(cls.urdf, "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
        home = cls.chain.fk(np.deg2rad(teleop.HOME_DEG))
        cls.home_pos, cls.home_rot = home[:3, 3].copy(), home[:3, :3].copy()

    def model(self):
        return {"chain": self.chain, "rot": self.home_rot, "q0": np.deg2rad(teleop.HOME_DEG)}

    def session(self, script, duration, extra=(), arm="right", probe=None, hand=None, driver=None, before=None,
                trace=None, quest_patch=True):
        """Runs cri_jog.run_session against the fake driver. Returns (driver, quest, error)."""
        rclpy.init()
        drv = node = None
        err = None
        quest = ScriptQuest(script, duration, hand=hand or arm, probe=probe)
        old_handlers = (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM))   # incl. rclpy's
        try:
            drv = FakeCriDriver(**(driver or {}))
            if before:
                before(drv)
            args, _ = teleop.parse_args(["--backend", "cri-jog", "--arms", arm, "--quiet", "--trace", trace or "",
                                         "--cri-wait-s", "8", *extra])
            node = cri_jog.CriJogNode(args)
            time.sleep(0.5)  # discovery
            factory = MagicMock(return_value=quest)
            self.make_quest = factory
            with patch.object(cri_jog, "make_quest", factory), patch.object(cri_jog, "make_urdf", lambda: self.urdf):
                try:
                    cri_jog.run_session(node, args, Path("unused.env"))
                except Exception as exc:  # noqa: BLE001 - the tests look at it
                    err = exc
            time.sleep(0.2)  # let the last messages reach the fake
            self.assertEqual((signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM)), old_handlers)
        finally:
            if node:
                node.close()
            if drv:
                drv.close()
            rclpy.try_shutdown()
        return drv, quest, err

    # ------------------------------------------------------------------------------------------------ happy path
    def test_hand_motion_converges_to_the_mapped_target_below_the_speed_limit(self):
        """Mapping = the IK backends' hand mapping (yaw, scale) plus --cri-signs; speed <= --cri-max-lin-vel."""
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "cri_trace.csv"
            drv, quest, err = self.session(lambda t: grip_then_move(t, (0.06, 0.03, 0.0)), 7.5,
                                           ["--yaw-deg", "90", "--cri-signs", "1,-1,1", "--scale", "1.0"],
                                           trace=str(path))
            self.assertIsNone(err)
            rows = list(csv.DictReader(ln for ln in path.read_text().splitlines() if not ln.startswith("#")))
        start = np.array([0.30, -0.20, 0.50])
        # Rz(90) * (0.06, 0.03, 0) = (-0.03, 0.06, 0); signs (1, -1, 1) -> (-0.03, -0.06, 0)
        expect = start + np.array([-0.03, -0.06, 0.0])
        np.testing.assert_allclose(drv.hist[-1][1], expect, atol=0.004)
        speeds = [np.linalg.norm(v) for _, v in drv.cmds()]
        self.assertLessEqual(max(speeds), MAX_VEL * (1 + 1e-6))
        self.assertGreater(max(speeds), 0.9 * MAX_VEL)  # the far target really saturated the limit
        h = drv.hist[::20]  # 0.1 s
        arm_speed = max(np.linalg.norm(h[i + 1][1] - h[i][1]) / (h[i + 1][0] - h[i][0]) for i in range(len(h) - 1))
        self.assertLessEqual(arm_speed, MAX_VEL * 1.05)
        self.assertEqual(drv.enables(), [True, False])
        self.assertEqual((drv.rejected, drv.ignored), (0, 0))  # every message was stamped now() and accepted
        # stamps: every message carried a fresh stamp (checked by the fake); frequency >= 20 Hz while engaged
        t = [tt for tt, _ in drv.cmds()]
        self.assertGreater(len(t) / (t[-1] - t[0]), 25.0)
        self.assertLess(max(np.diff(t)), 0.15)
        # trace: cri-jog columns
        self.assertGreater(len(rows), 150)
        eng = [r for r in rows if r["engaged"] == "1"]
        self.assertTrue(eng)
        for k in ("robot_x", "target_y", "vx", "speed_mps", "err_mm", "pose_age_ms"):
            float(eng[-1][k])
        self.assertEqual(eng[-1]["jog_state"], "active")
        self.assertEqual(eng[-1]["pose_mode"], "cartbase")
        self.assertTrue(any("vmax" in r["limited"] for r in eng))
        self.assertIn("[cri] engaged", " ".join(r["note"] for r in rows))

    def test_release_sends_zero_within_one_tick_and_the_robot_stops(self):
        def script(t):  # hand runs away 25 cm, released at 2.4 s while still far from its target
            if t < 0.4:
                return True, [0, 0, 0], 0.0, F, F
            return True, [min(0.25, 0.1 * (t - 0.4)), 0, 0], (1.0 if t < 2.4 else 0.0), F, F
        drv, quest, err = self.session(script, 5.0)
        self.assertIsNone(err)
        t_rel = next(t for t, _, _ in quest.reads if t >= quest.t0 + 2.4)   # the first read with the grip open
        self.assertGreater(len(drv.nonzero_after(0)), 20)
        zeros = [tt for tt, v in drv.cmds() if tt >= t_rel and np.linalg.norm(v) == 0.0]
        self.assertTrue(zeros)
        self.assertLess(zeros[0] - t_rel, 0.1)              # within one tick (33 ms) + delivery
        self.assertEqual(drv.nonzero_after(t_rel + 0.1), [])  # nothing but zeros afterwards
        p1, p2 = drv.pos_at(t_rel + 1.0), drv.hist[-1][1]
        self.assertLess(np.linalg.norm(p2 - p1), 0.001)       # the robot has stopped
        self.assertGreater(np.linalg.norm(p1 - np.array([0.30, -0.20, 0.50])), 0.02)  # and had moved before

    def test_tracking_loss_sends_zero_and_needs_a_new_grip(self):
        def script(t):
            if t < 0.4:
                return True, [0, 0, 0], 0.0, F, F
            pos = [min(0.25, 0.1 * (t - 0.4)), 0, 0]
            if 2.4 <= t < 3.4:
                return False, None, 1.0, F, F    # lost; the squeeze value alone must not keep the arm driven
            return True, pos, 1.0, F, F          # tracking returns with the grip still squeezed
        drv, quest, err = self.session(script, 5.5)
        self.assertIsNone(err)
        t_lost = quest.first_read(lambda tr, sq: not tr)
        self.assertGreater(len(drv.nonzero_after(0, t_lost)), 20)
        zeros = [tt for tt, v in drv.cmds() if tt >= t_lost and np.linalg.norm(v) == 0.0]
        self.assertLess(zeros[0] - t_lost, 0.1)
        self.assertEqual(drv.nonzero_after(t_lost + 0.1), [])   # grip still held when tracking came back: no motion

    # ------------------------------------------------------------------------------------------------ start / exit
    def test_enable_refused_exits_with_the_drivers_message_and_publishes_nothing(self):
        msg = "refused: a controller claims the arm's command interfaces; deactivate the trajectory controller first"
        drv, quest, err = self.session(drift, 5.0, driver={"refuse": msg})
        self.assertIsInstance(err, cri_jog.CriJogError)
        self.assertIn("deactivate the trajectory controller first", str(err))
        self.assertEqual(drv.cmds(), [])
        self.assertEqual(drv.enables(), [True])   # tried once, never left a mode it did not enter
        self.assertTrue(quest.closed)

    def test_enable_refusal_ends_main_with_exit_code_2(self):
        rclpy.init()
        drv = None
        quest = ScriptQuest(drift, 5.0)
        old = (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM))
        try:
            drv = FakeCriDriver(refuse="deactivate the trajectory controller first")
            argv = ["quest_dual_arm_teleop.py", "--backend", "cri-jog", "--arms", "right", "--trace", ""]
            with patch.object(sys, "argv", argv), patch.object(cri_jog, "make_quest", return_value=quest), \
                    patch.object(teleop.rclpy, "init"), patch.object(teleop.rclpy, "try_shutdown"), \
                    patch.object(cri_jog, "make_urdf", lambda: self.urdf):
                with self.assertRaises(SystemExit) as cm:
                    teleop.main()
            self.assertEqual(cm.exception.code, 2)
            self.assertEqual(drv.cmds(), [])
        finally:
            signal.signal(signal.SIGINT, old[0])
            signal.signal(signal.SIGTERM, old[1])
            if drv:
                drv.close()
            rclpy.try_shutdown()

    def test_ctrl_c_and_sigterm_send_zero_then_leave_the_mode(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(sig=sig.name):
                sent = []

                def probe(t, quest):
                    if t > 2.5 and not sent:
                        sent.append(time.monotonic())
                        os.kill(os.getpid(), sig)
                t0 = time.monotonic()
                drv, quest, err = self.session(drift, 30.0, probe=probe)
                self.assertIsNone(err)
                self.assertLess(time.monotonic() - t0, 8.0)       # stopped by the signal, not by the script end
                self.assertEqual(drv.enables(), [True, False])
                ev = drv.events
                i_off = max(i for i, e in enumerate(ev) if e[0] == "enable")
                last_cmd = [e for e in ev[:i_off] if e[0] == "cmd"][-1]
                self.assertEqual(float(np.linalg.norm(last_cmd[2])), 0.0)   # zero BEFORE leaving the mode
                self.assertLess(ev[i_off][1] - last_cmd[1], 0.3)
                self.assertEqual(drv.nonzero_after(sent[0] + 0.15), [])
                self.assertFalse(drv.active)

    def test_an_exception_in_the_loop_still_sends_zero_and_leaves_the_mode(self):
        def probe(t, quest):
            if t > 2.5:
                raise RuntimeError("boom")
        drv, quest, err = self.session(drift, 30.0, probe=probe)
        self.assertIsInstance(err, RuntimeError)
        self.assertEqual(drv.enables(), [True, False])
        ev = drv.events
        i_off = max(i for i, e in enumerate(ev) if e[0] == "enable")
        last_cmd = [e for e in ev[:i_off] if e[0] == "cmd"][-1]
        self.assertEqual(float(np.linalg.norm(last_cmd[2])), 0.0)
        self.assertTrue(quest.closed)

    def test_no_auto_enable_does_not_enter_the_mode_but_still_leaves_it_at_exit(self):
        drv, quest, err = self.session(lambda t: grip_then_move(t, (0.03, 0, 0)), 4.0, ["--cri-no-auto-enable"],
                                       driver={"active": True})
        self.assertIsNone(err)
        self.assertEqual(drv.enables(), [False])
        self.assertGreater(np.linalg.norm(drv.hist[-1][1] - np.array([0.30, -0.20, 0.50])), 0.01)  # it did move

    # ------------------------------------------------------------------------------------------------ faults
    def test_stale_robot_pose_stops_the_jog(self):
        frozen = []

        def probe(t, quest):
            if t > 2.5 and not frozen:
                frozen.append(time.monotonic())
                drv_box[0].pose_frozen = True
        drv_box = [None]
        drv, quest, err = self.session(drift, 5.0, probe=probe, before=lambda d: drv_box.__setitem__(0, d))
        self.assertIsNone(err)
        self.assertGreater(len(drv.nonzero_after(0, frozen[0])), 20)
        self.assertEqual(drv.nonzero_after(frozen[0] + 0.35), [])   # --cri-pose-timeout 0.2 s + a tick

    def test_driver_leaving_the_mode_stops_the_jog_until_a_new_grip(self):
        left = []
        drv_box = [None]

        def probe(t, quest):
            if t > 2.5 and not left:
                left.append(time.monotonic())
                drv_box[0].leave_mode()

        def script(t):
            sq = 1.0
            if t < 0.4 or 3.5 <= t < 4.2 or 5.2 <= t < 5.8:   # open the hand twice: the first grip after it re-enters the mode,
                sq = 0.0                              # the second one drives
            return True, [0.02 * t, 0, 0], sq, F, F
        drv, quest, err = self.session(script, 8.0, probe=probe, before=lambda d: drv_box.__setitem__(0, d))
        self.assertIsNone(err)
        self.assertEqual(drv.enables(), [True, True, False])
        t_re = [t for k, t, v in drv.events if k == "enable"][1]
        t_second_grip = quest.t0 + 5.8
        self.assertGreater(len(drv.nonzero_after(0, left[0])), 20)
        self.assertEqual(drv.nonzero_after(left[0] + 0.15, t_second_grip - 0.05), [])  # also after the re-enable
        self.assertTrue(t_re > left[0])
        self.assertGreater(len(drv.nonzero_after(t_second_grip + 0.3)), 5)             # and it drives again

    def test_foreign_publisher_at_start_refuses_to_start(self):
        holder = []
        drv, quest, err = self.session(
            drift, 5.0, before=lambda d: holder.append(d.node.create_publisher(TwistStamped, "/igus_rebel/cart_jog_cmd", 2)))
        self.assertIsInstance(err, cri_jog.CriJogError)
        self.assertIn("already publish", str(err))
        self.assertEqual(drv.enables(), [])
        self.assertEqual(drv.cmds(), [])
        self.make_quest.assert_not_called()

    def test_foreign_publisher_appearing_stops_the_jog(self):
        appeared = []
        drv_box = [None]

        def probe(t, quest):
            if t > 2.5 and not appeared:
                appeared.append(time.monotonic())
                drv_box[0].pub = drv_box[0].node.create_publisher(TwistStamped, "/igus_rebel/cart_jog_cmd", 2)
        drv, quest, err = self.session(drift, 5.0, probe=probe, before=lambda d: drv_box.__setitem__(0, d))
        self.assertIsNone(err)
        self.assertGreater(len(drv.nonzero_after(0, appeared[0])), 20)
        self.assertEqual(drv.nonzero_after(appeared[0] + 0.3), [])
        late = [t for t, _ in drv.cmds() if appeared[0] + 0.6 < t < drv.events[-1][1] - 0.15]
        self.assertEqual(late, [])  # no heartbeat either while the other publisher is there

    def test_arm_moving_against_the_command_is_stopped_and_stays_stopped(self):
        """A mirrored jog axis would turn the position feedback into a runaway: the follow monitor latches."""
        def script(t):
            sq = 0.0 if (3.0 <= t < 3.5) or t < 0.4 else 1.0
            return True, [min(0.2, 0.1 * t), 0, 0], sq, F, F
        drv, quest, err = self.session(script, 6.0, driver={"gain": -1.0})
        self.assertIsNone(err)
        start = np.array([0.30, -0.20, 0.50])
        wrong = max(np.linalg.norm(p - start) for _, p in drv.hist)
        self.assertLess(wrong, 0.04)                       # ~2 cm of motion before it is caught
        self.assertEqual(drv.nonzero_after(quest.t0 + 2.9), [])   # latched: the second grip does nothing
        self.assertGreater(len(drv.nonzero_after(0)), 10)

    def test_arm_that_does_not_follow_is_let_go(self):
        drv, quest, err = self.session(lambda t: grip_then_move(t, (0.2, 0, 0), t_ramp=2.0), 4.0, driver={"gain": 0.0})
        self.assertIsNone(err)
        self.assertGreater(len(drv.nonzero_after(0)), 10)
        self.assertEqual(drv.nonzero_after(quest.t0 + 3.0), [])   # no motion in the last second without a new grip

    # ------------------------------------------------------------------------------------------------ dry run / args
    def test_dry_run_computes_but_publishes_nothing_and_never_calls_the_service(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "t.csv"
            drv, quest, err = self.session(lambda t: grip_then_move(t, (0.06, 0.0, 0.0)), 3.5, ["--dry-run"],
                                           trace=str(path))
            self.assertIsNone(err)
            rows = list(csv.DictReader(ln for ln in path.read_text().splitlines() if not ln.startswith("#")))
        self.assertEqual(drv.cmds(), [])
        self.assertEqual(drv.enables(), [])
        np.testing.assert_allclose(drv.hist[-1][1], [0.30, -0.20, 0.50], atol=1e-9)
        eng = [r for r in rows if r["engaged"] == "1"]
        self.assertTrue(eng)
        self.assertTrue(any(float(r["speed_mps"]) > 0.02 for r in eng))   # the twist was computed and traced ...
        self.assertTrue(all(r["sent"] == "0" for r in rows))              # ... but never sent

    def test_bad_options_are_refused_and_topic_defaults_follow_the_arm_count(self):
        for extra in (["--rate", "10"], ["--cri-signs", "1,0,1"], ["--cri-max-lin-vel", "0.5"], ["--cri-kp", "0"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                teleop.parse_args(["--backend", "cri-jog", "--arms", "left", *extra])
        args, _ = teleop.parse_args(["--backend", "cri-jog", "--arms", "left"])
        self.assertEqual((args.cri_cmd_topic, args.cri_enable_service, args.cri_state_topic, args.cri_pose_topic,
                          args.cri_max_lin_vel, args.cri_signs),
                         ("/igus_rebel/cart_jog_cmd", "/igus_rebel/cart_jog_enable", "/igus_rebel/hardware_state",
                          "/igus_rebel/robot_cart_pose", 0.03, (1.0, 1.0, 1.0)))
        for argv in (["--backend", "cri-jog"], ["--backend", "cri-jog", "--arms", "right", "left"]):   # both arms
            args, _ = teleop.parse_args(argv)
            self.assertEqual(args.arms, ["left", "right"])
            self.assertEqual((args.cri_cmd_topic, args.cri_enable_service, args.cri_state_topic, args.cri_pose_topic),
                             ("/igus_rebel_{side}/cart_jog_cmd", "/igus_rebel_{side}/cart_jog_enable",
                              "/igus_rebel_{side}/hardware_state", "/igus_rebel_{side}/robot_cart_pose"))
        args, _ = teleop.parse_args(["--backend", "cri-jog", "--arms", "left", "right", "--cri-cmd-topic",
                                     "/arm_{side}/jog"])   # explicit {side} override
        self.assertEqual(args.cri_cmd_topic, "/arm_{side}/jog")
        args, _ = teleop.parse_args(["--backend", "cri-jog", "--arms", "left", "--cri-cmd-topic", "/x/jog"])
        self.assertEqual(args.cri_cmd_topic, "/x/jog")

    def test_two_arms_with_identical_resolved_topics_are_refused(self):
        for opt in ("--cri-cmd-topic", "--cri-enable-service", "--cri-state-topic", "--cri-pose-topic"):
            err = io.StringIO()
            with contextlib.redirect_stderr(err), self.assertRaises(SystemExit) as cm:
                teleop.parse_args(["--backend", "cri-jog", "--arms", "left", "right", opt, "/same/name"])
            self.assertEqual(cm.exception.code, 2)
            self.assertIn("same name for both arms", err.getvalue())

    # ------------------------------------------------------------------------------------------------ buttons
    def test_buttons_events_and_no_motion_on_reset(self):
        """Right hand: A = toggle, B = discard events. Left hand: Y = reset event only (releases, NO motion), X = nothing."""
        def script_right(t):
            sq = 1.0 if t > 0.4 else 0.0
            return True, [0.02 * t, 0, 0], sq, 1.0 < t < 1.3, 1.6 < t < 1.9
        drv, quest, err = self.session(script_right, 3.0)
        self.assertIsNone(err)
        self.assertEqual(drv.chat, ["toggle", "discard"])

        def script_left(t):
            sq = 1.0 if t > 0.4 else 0.0
            return True, [0.02 * t, 0, 0], sq, 1.0 < t < 1.3, 2.0 < t < 3.5   # X at 1.0 s, Y held from 2.0 s (hold 1.0 s)
        drv, quest, err = self.session(script_left, 5.0, arm="left")
        self.assertIsNone(err)
        self.assertEqual(drv.chat, ["reset"])           # X (rotation frame) produced no event
        t_y = quest.t0 + 3.05                           # the hold completes at ~3.0 s
        self.assertEqual(drv.nonzero_after(t_y + 0.1), [])   # released on Y, and the hand is still gripped: no motion
        self.assertGreater(len(drv.nonzero_after(0, t_y)), 20)

    def test_short_y_press_does_not_discard_or_release(self):
        """Y must be held --y-hold-s (1.0 s): a 0.4 s press produces no 'reset' event and the gripped arm keeps moving."""
        def script(t):
            sq = 1.0 if t > 0.4 else 0.0
            return True, [0.02 * t, 0, 0], sq, False, 2.0 < t < 2.4
        drv, quest, err = self.session(script, 4.0, arm="left")
        self.assertIsNone(err)
        self.assertEqual(drv.chat, [])
        self.assertGreater(len(drv.nonzero_after(quest.t0 + 2.6)), 10)   # still driving the arm after the brush

    # ================================================================================================ dual arm
    def dual(self, scripts, duration, extra=(), probe=None, kw=None, before=None, trace=None, model=True):
        """cri_jog.run_session with BOTH arms against two fake drivers (/igus_rebel_left|right/...). Returns
        ({side: driver}, quest, error). The fakes derive joints from the position by IK (their own business), so the
        arm guard sees real arm geometry."""
        rclpy.init()
        drvs, node, err = {}, None, None
        self.drvs = drvs
        quest = ScriptQuest(scripts, duration, probe=probe)
        try:
            for side in ("left", "right"):
                kws = dict(ns=f"igus_rebel_{side}", start_m=self.home_pos, model=self.model() if model else None)
                kws.update((kw or {}).get(side, {}))
                drvs[side] = FakeCriDriver(**kws)
            if before:
                before(drvs)
            args, _ = teleop.parse_args(["--backend", "cri-jog", "--arms", "left", "right", "--quiet", "--trace",
                                         trace or "", "--cri-wait-s", "8", *extra])
            node = cri_jog.CriJogNode(args)
            time.sleep(0.6)  # discovery
            factory = MagicMock(return_value=quest)
            self.make_quest = factory
            with patch.object(cri_jog, "make_quest", factory), patch.object(cri_jog, "make_urdf", lambda: self.urdf):
                try:
                    cri_jog.run_session(node, args, Path("unused.env"))
                except Exception as exc:  # noqa: BLE001
                    err = exc
            time.sleep(0.2)
        finally:
            if node:
                node.close()
            for d in drvs.values():
                d.close()
            rclpy.try_shutdown()
        return drvs, quest, err

    def true_clearance(self, left_q_hist, right_q_hist, left_assumed=None, stride=3):
        """Surface clearance between the arms computed from the FAKES' own joints (independent of the teleop's trace)."""
        from arm_collision import capsules, clearance, load_world_T_base
        T = load_world_T_base()
        a, r = 0.06, 0.04   # --arm-radius / --tool-radius defaults
        out = []
        other = np.array([t for t, _ in right_q_hist])
        for k in range(0, len(left_q_hist), stride):
            t, ql = left_q_hist[k]
            qr = right_q_hist[int(np.argmin(np.abs(other - t)))][1]
            out.append((t, clearance(capsules(self.chain, ql, T["left"], a, r), capsules(self.chain, qr, T["right"], a, r))))
        return out

    def test_dual_both_arms_follow_their_own_hand(self):
        drvs, quest, err = self.dual({"left": lambda t: grip_then_move(t, (0.03, 0.0, 0.02)),
                                      "right": lambda t: grip_then_move(t, (0.04, 0.0, -0.02))}, 6.0)
        self.assertIsNone(err)
        np.testing.assert_allclose(drvs["left"].hist[-1][1], self.home_pos + [0.03, 0.0, 0.02], atol=0.004)
        np.testing.assert_allclose(drvs["right"].hist[-1][1], self.home_pos + [0.04, 0.0, -0.02], atol=0.004)
        for d in drvs.values():
            self.assertEqual(d.enables(), [True, False])
            self.assertEqual((d.rejected, d.ignored), (0, 0))
            self.assertLessEqual(max(np.linalg.norm(v) for _, v in d.cmds()), MAX_VEL * (1 + 1e-6))

    def test_dual_swap_hands(self):
        drvs, quest, err = self.dual({"right": lambda t: grip_then_move(t, (0.03, 0.0, 0.0)),
                                      "left": lambda t: grip_then_move(t, (0.0, 0.0, 0.03))}, 5.0,
                                     ["--swap-hands"])
        self.assertIsNone(err)
        np.testing.assert_allclose(drvs["left"].hist[-1][1], self.home_pos + [0.03, 0.0, 0.0], atol=0.004)  # right hand
        np.testing.assert_allclose(drvs["right"].hist[-1][1], self.home_pos + [0.0, 0.0, 0.03], atol=0.004)

    def test_dual_releasing_one_hand_stops_only_that_arm(self):
        def left(t):
            return True, [0.02 * t, 0, 0], (0.0 if (t < 0.4 or t >= 2.4) else 1.0), F, F
        drvs, quest, err = self.dual({"left": left, "right": drift}, 4.5)
        self.assertIsNone(err)
        t_rel = quest.t0 + 2.4
        self.assertGreater(len(drvs["left"].nonzero_after(0, t_rel)), 20)
        self.assertEqual(drvs["left"].nonzero_after(t_rel + 0.1), [])
        self.assertGreater(len(drvs["right"].nonzero_after(t_rel + 0.3)), 20)   # the right arm keeps following its hand

    def test_dual_a_fault_of_one_arm_stops_both(self):
        def stale(d): d["left"].pose_frozen = True
        def kinstate(d): d["left"].kinstate = 14
        def latched(d): d["left"].latched = True
        def mode_left(d): d["left"].leave_mode()
        def foreign(d): d["left"].foreign = d["left"].node.create_publisher(TwistStamped, "/igus_rebel_left/cart_jog_cmd", 2)
        for name, inject in (("stale pose", stale), ("KINSTATE", kinstate), ("latched driver", latched),
                             ("driver left the mode", mode_left), ("foreign publisher", foreign)):
            with self.subTest(fault=name):
                at = []

                def probe(t, quest):
                    if t > 2.5 and not at:
                        at.append(time.monotonic())
                        inject(self.drvs)
                drvs, quest, err = self.dual({"left": drift, "right": drift}, 4.5, probe=probe)
                self.assertIsNone(err)
                for side in ("left", "right"):
                    self.assertGreater(len(drvs[side].nonzero_after(0, at[0])), 20, side)
                    self.assertEqual(drvs[side].nonzero_after(at[0] + 0.4), [], f"{side} still driven after the {name}")

    def test_dual_enable_refused_on_one_arm_leaves_the_other_and_exits_2(self):
        msg = "refused: a controller claims the arm's command interfaces; deactivate the trajectory controller first"
        drvs, quest, err = self.dual({"left": drift, "right": drift}, 4.0, kw={"right": {"refuse": msg}})
        self.assertIsInstance(err, cri_jog.CriJogError)
        self.assertIn("right arm", str(err))
        self.assertIn(msg, str(err))
        self.assertIn("arm_right_trajectory_controller", str(err))
        self.assertEqual(drvs["left"].enables(), [True, False])    # entered, then left again
        self.assertEqual(drvs["right"].enables(), [True])
        self.assertEqual(drvs["left"].cmds() + drvs["right"].cmds(), [])
        # and the process exit code
        rclpy.init()
        drvs, old = {}, (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM))
        try:
            for side in ("left", "right"):
                drvs[side] = FakeCriDriver(ns=f"igus_rebel_{side}", start_m=self.home_pos, model=self.model(),
                                           refuse=msg if side == "left" else None)
            argv = ["quest_dual_arm_teleop.py", "--backend", "cri-jog", "--arms", "left", "right", "--trace", "",
                    "--cri-wait-s", "8"]
            with patch.object(sys, "argv", argv), patch.object(cri_jog, "make_quest", return_value=ScriptQuest(drift, 5.0)), \
                    patch.object(teleop.rclpy, "init"), patch.object(teleop.rclpy, "try_shutdown"), \
                    patch.object(cri_jog, "make_urdf", lambda: self.urdf):
                with self.assertRaises(SystemExit) as cm:
                    teleop.main()
            self.assertEqual(cm.exception.code, 2)
            time.sleep(0.2)
            self.assertEqual(drvs["right"].enables(), [True, False])
        finally:
            signal.signal(signal.SIGINT, old[0])
            signal.signal(signal.SIGTERM, old[1])
            for d in drvs.values():
                d.close()
            rclpy.try_shutdown()

    def test_dual_ctrl_c_sends_zero_to_both_then_leaves_both_modes(self):
        sent = []

        def probe(t, quest):
            if t > 2.5 and not sent:
                sent.append(time.monotonic())
                os.kill(os.getpid(), signal.SIGINT)
        drvs, quest, err = self.dual({"left": drift, "right": drift}, 30.0, probe=probe)
        self.assertIsNone(err)
        for side, d in drvs.items():
            self.assertEqual(d.enables(), [True, False], side)
            ev = d.events
            i_off = max(i for i, e in enumerate(ev) if e[0] == "enable")
            last_cmd = [e for e in ev[:i_off] if e[0] == "cmd"][-1]
            self.assertEqual(float(np.linalg.norm(last_cmd[2])), 0.0, side)
            self.assertEqual(d.nonzero_after(sent[0] + 0.15), [], side)

    def test_dual_foreign_publisher_at_start_refuses(self):
        holder = []
        drvs, quest, err = self.dual(
            {"left": drift, "right": drift}, 4.0,
            before=lambda d: holder.append(d["right"].node.create_publisher(TwistStamped, "/igus_rebel_right/cart_jog_cmd", 2)))
        self.assertIsInstance(err, cri_jog.CriJogError)
        self.assertIn("/igus_rebel_right/cart_jog_cmd", str(err))
        self.assertEqual((drvs["left"].enables(), drvs["right"].enables()), ([], []))
        self.make_quest.assert_not_called()

    # -- the arm-to-arm guard ------------------------------------------------------------------------------
    @staticmethod
    def toward(sign):
        """Hand slides 25 cm along base y (sign -1 = toward the right arm for the left hand), is opened 5.0-5.5 s and then
        moves back (away from the other arm)."""
        def f(t):
            if t < 0.4 or 5.0 <= t < 5.5:
                return True, [0, 0, 0], 0.0, F, F
            if t < 5.0:
                return True, [0, sign * min(0.25, 0.1 * (t - 0.4)), 0], 1.0, F, F
            return True, [0, sign * (0.25 - 0.04 * (t - 5.5)), 0], 1.0, F, F
        return f

    def test_dual_guard_stops_the_arms_before_min_arm_dist_and_moving_away_works(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "t.csv"
            drvs, quest, err = self.dual({"left": self.toward(-1), "right": self.toward(+1)}, 8.5,
                                         ["--cri-max-lin-vel", "0.05"], trace=str(path))
            self.assertIsNone(err)
            text = path.read_text()
        rows = list(csv.DictReader(ln for ln in text.splitlines() if not ln.startswith("#")))
        self.assertEqual(len(rows) % 2, 0)   # one row per arm per tick
        self.assertEqual({r["side"] for r in rows}, {"left", "right"})
        clr = self.true_clearance(drvs["left"].qhist, drvs["right"].qhist)
        t_vals = np.array([c for _, c in clr])
        self.assertGreater(float(t_vals.max()), 0.2)                        # started well apart ...
        self.assertLess(float(t_vals.min()), 0.05 + 0.04)                    # ... and really came into the guard's zone
        print(f"\n[dual guard test] clearance start {t_vals.max() * 100:.1f} cm, minimum {t_vals.min() * 100:.1f} cm, "
              f"end {clr[-1][1] * 100:.1f} cm (min_arm_dist 5.0 cm)")
        self.assertGreaterEqual(float(t_vals.min()), 0.05 - 0.006)          # but never below min_arm_dist (minus 6 mm)
        tr = np.array([float(r["clearance_mm"]) for r in rows if r["clearance_mm"]]) / 1000.0
        self.assertGreaterEqual(float(tr.min()), 0.05 - 0.006)               # the trace reports the same
        self.assertTrue(any(r["guard"] == "1" for r in rows if r["side"] == "left"))
        self.assertTrue(any(r["guard"] == "1" for r in rows if r["side"] == "right"))
        self.assertIn("arm guard", " ".join(r["note"] for r in rows))
        # blocked arms stay at zero until the re-grip at 5.5 s; moving away then works and the clearance grows again
        t_block = min(float(r["t"]) for r in rows if r["guard"] == "1")
        later = [r for r in rows if r["side"] == "left" and r["engaged"] == "1" and float(r["t"]) > 5.0]
        self.assertTrue(later)
        end_clr = clr[-1][1]
        self.assertGreater(end_clr, float(t_vals.min()) + 0.008)
        self.assertGreater(len(drvs["left"].nonzero_after(quest.t0 + 5.6)), 10)
        self.assertGreater(len(drvs["right"].nonzero_after(quest.t0 + 5.6)), 10)
        self.assertGreater(t_block, 1.0)

    def test_dual_without_the_guard_the_same_motion_goes_below_min_arm_dist(self):
        """Shows the scenario is not vacuous (and what the guard mutation looks like)."""
        drvs, quest, err = self.dual({"left": self.toward(-1), "right": self.toward(+1)}, 6.0,
                                     ["--cri-max-lin-vel", "0.05", "--no-arm-guard"])
        self.assertIsNone(err)
        clr = self.true_clearance(drvs["left"].qhist, drvs["right"].qhist)
        print(f"\n[no guard] minimum clearance {min(c for _, c in clr) * 100:.1f} cm")
        self.assertLess(min(c for _, c in clr), 0.05 - 0.02)

    def test_single_arm_guard_against_the_assumed_other_arm(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "t.csv"
            drv, quest, err = self.session(self.toward(+1), 6.0,
                                           ["--cri-max-lin-vel", "0.05", "--other-arm-deg", *map(str, teleop.HOME_DEG)],
                                           driver={"model": self.model(), "start_m": self.home_pos}, trace=str(path))
            self.assertIsNone(err)
            rows = list(csv.DictReader(ln for ln in path.read_text().splitlines() if not ln.startswith("#")))
        from arm_collision import capsules, clearance, load_world_T_base
        T = load_world_T_base()
        q_left = np.deg2rad(teleop.HOME_DEG)
        clr = [clearance(capsules(self.chain, q, T["right"], 0.06, 0.04), capsules(self.chain, q_left, T["left"], 0.06, 0.04))
               for _, q in drv.qhist[::3]]
        self.assertGreaterEqual(min(clr), 0.05 - 0.006)
        self.assertLess(min(clr), 0.09)
        self.assertTrue(any(r["guard"] == "1" for r in rows))

    def test_single_arm_watch_other_with_a_stale_pose_stops_the_driven_arm(self):
        holder, frozen = [], []

        def before(d):
            holder.append(FakeCriDriver(ns="igus_rebel_left", start_m=self.home_pos, model=self.model()))

        def probe(t, quest):
            if t > 2.5 and not frozen:
                frozen.append(time.monotonic())
                holder[0].pose_frozen = True
        try:
            drv, quest, err = self.session(drift, 5.0, ["--watch-other"], probe=probe, before=before)
        finally:
            for h in holder:
                h.close()
        self.assertIsNone(err)
        self.assertGreater(len(drv.nonzero_after(0, frozen[0])), 20)
        self.assertEqual(drv.nonzero_after(frozen[0] + 0.4), [])   # the guard is blind: the driven arm stops

    def test_without_the_urdf_the_guard_refuses_to_start_unless_no_arm_guard(self):
        def no_urdf():
            raise subprocess.CalledProcessError(1, "ros2 pkg prefix igus_rebel_description")
        rclpy.init()
        drv = node = None
        try:
            drv = FakeCriDriver(ns="igus_rebel")
            args, _ = teleop.parse_args(["--backend", "cri-jog", "--arms", "right", "--quiet", "--trace", "",
                                         "--cri-wait-s", "8"])
            node = cri_jog.CriJogNode(args)
            time.sleep(0.4)
            quest = ScriptQuest(drift, 3.0)
            with patch.object(cri_jog, "make_quest", return_value=quest) as mq, patch.object(cri_jog, "make_urdf", no_urdf):
                with self.assertRaises(cri_jog.CriJogError) as cm:
                    cri_jog.run_session(node, args, Path("unused.env"))
                mq.assert_not_called()
            self.assertIn("--no-arm-guard", str(cm.exception))
            self.assertEqual(drv.enables(), [])
        finally:
            if node:
                node.close()
            if drv:
                drv.close()
            rclpy.try_shutdown()
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "t.csv"
            rclpy.init()
            drv = node = None
            try:
                drv = FakeCriDriver(ns="igus_rebel")
                args, _ = teleop.parse_args(["--backend", "cri-jog", "--arms", "right", "--quiet", "--trace", str(path),
                                             "--cri-wait-s", "8", "--no-arm-guard"])
                node = cri_jog.CriJogNode(args)
                time.sleep(0.4)
                quest = ScriptQuest(drift, 3.0)
                with patch.object(cri_jog, "make_quest", return_value=quest), patch.object(cri_jog, "make_urdf", no_urdf):
                    cri_jog.run_session(node, args, Path("unused.env"))
                time.sleep(0.2)
                self.assertEqual(drv.enables(), [True, False])
            finally:
                if node:
                    node.close()
                if drv:
                    drv.close()
                rclpy.try_shutdown()
            text = path.read_text()
        self.assertIn("ARM GUARD OFF", text)
        rows = list(csv.DictReader(ln for ln in text.splitlines() if not ln.startswith("#")))
        self.assertTrue(all(r["clearance_mm"] == "" for r in rows))

    def test_guard_unit_rule(self):
        """ArmGuard.limit on hand-made geometry: approach is stopped in the near zone, retreat and far motion are not."""
        from arm_collision import load_world_T_base
        args, _ = teleop.parse_args(["--backend", "cri-jog", "--arms", "left", "right"])
        g = cri_jog.ArmGuard(self.chain, args, load_world_T_base(), ["left", "right"])
        self.assertAlmostEqual(g.margin, 2 * 0.03 * (0.2 + 1 / 30 + 0.15), places=6)
        q = np.deg2rad(teleop.HOME_DEG)
        self.assertIsNone(g.measure(0.0, {"left": q, "right": q}))
        far = g.d
        self.assertGreater(far, 0.2)
        self.assertEqual(g.limit({"left": np.array([0, -0.03, 0.0]), "right": np.array([0, 0.03, 0.0])}), {})   # not near
        g.min_dist = far - g.margin / 2          # pretend the zone starts here
        self.assertTrue(g.near is False)
        g.measure(0.1, {"left": q, "right": q})
        self.assertTrue(g.near)
        blocked = g.limit({"left": np.array([0, -0.03, 0.0]), "right": np.array([0, 0.03, 0.0])})
        self.assertEqual(set(blocked), {"left", "right"})
        self.assertIn("moving toward the other arm", blocked["left"])
        self.assertEqual(g.limit({"left": np.array([0, 0.03, 0.0]), "right": np.array([0, -0.03, 0.0])}), {})   # apart
        perp = np.cross(g.n["left"], [0, 0, 1.0])
        perp = 0.03 * perp / np.linalg.norm(perp)
        self.assertEqual(g.limit({"left": g.T["left"][:3, :3].T @ perp, "right": np.zeros(3)}), {})             # sideways
        self.assertEqual(set(g.limit({"left": np.array([0, -0.03, 0.0]), "right": np.zeros(3)})), {"left"})
        # backstop: closest point on a proximal link (velocity unknown) -> only the measured clearance rate decides
        g.measure(0.3, {"left": q, "right": q})
        g.idx = {"left": 1, "right": 1}
        both = {"left": np.array([0, 0.03, 0.0]), "right": np.array([0, -0.03, 0.0])}   # "away" by the TCP rule
        g.rate = None
        self.assertEqual(g.limit(both), {})
        g.rate = 0.01
        self.assertEqual(g.limit(both), {})
        g.rate = -0.01
        blocked = g.limit(both)
        self.assertEqual(set(blocked), {"left", "right"})
        self.assertIn("clearance shrinking", blocked["left"])
        self.assertEqual(g.limit({"left": np.zeros(3), "right": np.zeros(3)}), {})        # nothing commanded, nothing to stop
        self.assertIn("no fresh joint state of the right arm", g.measure(0.4, {"left": q, "right": None}))
        from arm_collision import segment_distance   # segment_closest agrees with arm_collision's distance
        rng = np.random.default_rng(1)
        for _ in range(200):
            p1, q1, p2, q2 = rng.normal(size=(4, 3))
            dist, c1, c2 = cri_jog.segment_closest(p1, q1, p2, q2)
            self.assertAlmostEqual(dist, segment_distance(p1, q1, p2, q2), places=9)
            self.assertAlmostEqual(float(np.linalg.norm(c1 - c2)), dist, places=9)

    # ------------------------------------------------------------------------------------------------ units
    def test_control_law_units(self):
        v, clamped = cri_jog.jog_velocity([0.0005, 0, 0], 2.0, 0.03, 0.001)
        self.assertEqual((float(np.linalg.norm(v)), clamped), (0.0, False))      # deadband
        v, clamped = cri_jog.jog_velocity([0.0011, 0, 0], 2.0, 0.03, 0.001)
        self.assertLess(float(np.linalg.norm(v)), 1e-3)                          # continuous at the deadband edge
        v, clamped = cri_jog.jog_velocity([0.5, 0.5, 0], 2.0, 0.03, 0.001)
        self.assertTrue(clamped)
        self.assertAlmostEqual(float(np.linalg.norm(v)), 0.03)
        np.testing.assert_allclose(v[0], v[1])                                   # direction kept
        mon = cri_jog.FollowMonitor()
        out = None
        for i in range(60):   # 3 cm/s commanded for 2 s, the robot goes the other way
            out = mon.update(i / 30.0, [-0.001 * i, 0, 0], [0.03, 0, 0]) or out
        self.assertEqual(out[0], "opposite")
        mon = cri_jog.FollowMonitor()
        out = None
        for i in range(60):   # following well: no trip
            out = mon.update(i / 30.0, [0.001 * i, 0, 0], [0.03, 0, 0]) or out
        self.assertIsNone(out)
        self.assertEqual(cri_jog.parse_signs("1, -1,1"), (1.0, -1.0, 1.0))
        with self.assertRaises(ValueError):
            cri_jog.parse_signs("1,1")

    def test_other_backends_do_not_need_igus_rebel_msgs(self):
        code = ("import sys; sys.path.insert(0, %r); import quest_dual_arm_teleop as t; "
                "a, _ = t.parse_args(['--backend', 'real-ros', '--arms', 'left']); "
                "assert (a.max_joint_vel, a.max_cmd_lead_deg, a.max_joint_acc) == (0.52, 10.0, 0.52), a; "
                "b, _ = t.parse_args([]); assert b.backend == 'isaac' and b.max_joint_vel == 0.785; "
                "assert 'igus_rebel_msgs' not in sys.modules; print('ok')"
                % str(Path(__file__).resolve().parents[1] / "quest_teleop"))
        env = dict(os.environ, ROS_DOMAIN_ID=ros_isolation.TEST_DOMAIN, ROS_LOCALHOST_ONLY="1")
        out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env, timeout=60)
        self.assertEqual(out.stdout.strip(), "ok", out.stderr)

    def test_runs_on_the_isolated_test_domain(self):
        rclpy.init()
        try:
            dom = rclpy.get_default_context().get_domain_id()
            self.assertEqual(str(dom), ros_isolation.TEST_DOMAIN)
            ros_isolation.check(dom)
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    unittest.main()
