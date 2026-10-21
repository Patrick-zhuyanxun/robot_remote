"""Real arm through the igus ROS driver (--backend real-ros) against a fake driver that behaves like the ReBeL:
joint_trajectory_controller with a velocity command interface (p 1.0, ff_velocity_scale 0.4; each single-point message
restarts from the measured state, position-only points -> linear interpolation) driving the plant identified in
Igus_rebel's controller.yaml (gain 1.26, delay 0.078 s, velocity lag 0.16 s, acceleration limit 0.52 rad/s^2).

  - leash: a fast back-and-forth hand motion used to leave the arm >15 deg behind its command (-> "let go");
    with --max-cmd-lead-deg 6 the command stays within 6 deg of the measured joints and the arm stays gripped
  - stall: an arm that does not move at all (motors off) is still let go
  - Y on the real robot: open episode discarded, both arms back to HOME_DEG; a button during the move stops it
"""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402  - FIRST: never share the robot's ROS graph
from pathlib import Path
import sys
import threading
import time
import unittest
import os
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "quest_teleop"))
try:
    import numpy as np
    import rclpy
    import rclpy.executors
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory
    import quest_dual_arm_teleop as teleop
except ImportError:  # pragma: no cover
    teleop = None


class FakeRebelDriver:
    """/<side>/rebel_arm_trajectory_controller/joint_trajectory -> JTC + plant at 100 Hz -> /<side>/joint_states."""

    def __init__(self, sides, q0_deg, frozen=False):
        self.node = rclpy.create_node("fake_rebel_driver")
        self.dt, self.delay_n = 0.01, 8
        self.q = {s: np.deg2rad(np.array(q0_deg, float)) for s in sides}
        self.v = {s: np.zeros(6) for s in sides}
        self.ubuf = {s: [np.zeros(6)] * self.delay_n for s in sides}
        self.seg = {s: None for s in sides}
        self.pending = {s: None for s in sides}
        self.frozen = frozen
        self.lock = threading.Lock()
        self.pubs = {s: self.node.create_publisher(JointState, f"/{s}/joint_states", 10) for s in sides}
        for s in sides:
            self.node.create_subscription(JointTrajectory, f"/{s}/rebel_arm_trajectory_controller/joint_trajectory",
                                          lambda m, s=s: self._on_traj(s, m), 50)
        self.ex = rclpy.executors.SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.stop = threading.Event()
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _on_traj(self, side, msg):
        pt = msg.points[0]
        la = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
        with self.lock:
            self.pending[side] = (np.array(pt.positions), la)

    def _run(self):
        t = 0.0
        while not self.stop.is_set():
            self.ex.spin_once(timeout_sec=0.0)
            with self.lock:
                for s in self.q:
                    if self.pending[s] is not None:  # new message: restart from the measured state (closed loop)
                        b, la = self.pending[s]
                        self.seg[s] = (t, self.q[s].copy(), b, la)
                        self.pending[s] = None
                    u = np.zeros(6)
                    if self.seg[s] is not None:
                        t0, a, b, la = self.seg[s]
                        k = min((t - t0) / la, 1.0)
                        ref = a + (b - a) * k
                        vref = (b - a) / la if k < 1.0 else np.zeros(6)
                        u = 0.4 * vref + 1.0 * (ref - self.q[s])
                    self.ubuf[s].append(u)
                    ud = self.ubuf[s].pop(0)
                    if not self.frozen:
                        dv = np.clip((1.26 * ud - self.v[s]) / 0.16 * self.dt, -0.52 * self.dt, 0.52 * self.dt)
                        self.v[s] = self.v[s] + dv
                        self.q[s] = self.q[s] + self.v[s] * self.dt
                    msg = JointState(name=list(teleop.ARM_JOINTS), position=[float(x) for x in self.q[s]])
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    self.pubs[s].publish(msg)
            t += self.dt
            time.sleep(self.dt)

    def close(self):
        self.stop.set()
        self.th.join()
        self.node.destroy_node()


class TimedQuest:
    """Left controller from a script f(t) -> (pos, squeeze, primary, secondary)."""

    def __init__(self, script, duration):
        self.script, self.duration, self.t0 = script, duration, None
        self.buttons = {"left": (False, False)}

    def read_all(self):
        if self.t0 is None:
            self.t0 = time.monotonic()
        t = time.monotonic() - self.t0
        if t > self.duration:
            raise KeyboardInterrupt
        pos, squeeze, prim, sec = self.script(t)
        self.buttons = {"left": (prim, sec)}
        return {"left": (True, np.asarray(pos, float), np.array([0, 0, 0, 1.0]), squeeze, 0.0)}

    def close(self):
        pass


def wave(t):  # open hand first (the real arm needs a release before the first grip), then fast back-and-forth
    if t < 0.4:
        return [0, 0, 0], 0.0, False, False
    return [0, 0.12 * np.sin(2 * np.pi * 0.5 * (t - 0.4)), 0.06 * np.sin(2 * np.pi * 0.7 * (t - 0.4))], 1.0, False, False


@unittest.skipIf(teleop is None, "source ROS and use .venv_quest/bin/python for node integration")
class RealRosTrackingTests(unittest.TestCase):
    def session(self, extra, script, duration, q0=teleop.HOME_DEG if teleop else None, frozen=False, probe=None):
        rclpy.init()
        node = drv = None
        events = []
        try:
            args, _ = teleop.parse_args(["--backend", "real-ros", "--arms", "left", "--quiet", "--no-start-home",
                                         "--no-park", *extra])
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            drv = FakeRebelDriver(["left"], q0, frozen=frozen)
            drv.node.create_subscription(String, "/quest_teleop/events", lambda m: events.append(m.data), 10)
            node = teleop.DualArmTeleop(args, chain)
            arm = node.arms["left"]
            stats = {"gap": 0.0, "engaged_ticks": 0, "let_go": 0}
            orig_warning = node.get_logger().warning
            quest = TimedQuest(script, duration)
            read = quest.read_all

            def read_all():
                out = read()
                if arm.engaged and arm.q_cmd is not None and arm.q_meas is not None:
                    stats["gap"] = max(stats["gap"], float(np.rad2deg(np.max(np.abs(arm.q_cmd - arm.q_meas)))))
                    stats["engaged_ticks"] += 1
                if probe:
                    probe(time.monotonic() - quest.t0, node)
                return out
            quest.read_all = read_all
            with patch.object(teleop, "QuestController", return_value=quest), \
                    patch.object(type(node.get_logger()), "warning", autospec=True,
                                 side_effect=lambda self_, msg, **kw: (stats.__setitem__("let_go", stats["let_go"] + ("let go" in msg)),
                                                                       orig_warning(msg, **kw))):
                teleop.run_session(node, chain, args)
            stats["q_end"] = np.rad2deg(arm.q_meas)
            stats["engaged_end"] = arm.engaged
            return stats, events
        finally:
            if node:
                node.destroy_node()
            if drv:
                drv.close()
            rclpy.try_shutdown()

    def test_leash_keeps_a_fast_back_and_forth_gripped(self):
        stats, _ = self.session([], wave, 7.0)
        self.assertGreater(stats["engaged_ticks"], 150)
        self.assertEqual(stats["let_go"], 0)
        self.assertTrue(stats["engaged_end"])
        self.assertLess(stats["gap"], 10.5)  # --max-cmd-lead-deg 10 (real-ros default)

    def test_without_leash_the_same_motion_lets_go(self):  # the behaviour the operator saw (15 deg lag)
        stats, _ = self.session(["--max-cmd-lead-deg", "0", "--max-joint-acc", "0"], wave, 7.0)
        self.assertGreater(stats["let_go"], 0)
        self.assertGreater(stats["gap"], 14.0)

    def test_arm_that_does_not_move_is_let_go(self):
        stats, _ = self.session([], wave, 4.0, frozen=True)
        self.assertGreater(stats["let_go"], 0)

    def test_y_moves_the_real_arms_home_and_a_button_stops_it(self):
        start = [20, 10, 50, 0, 30, 0]

        def press_y(t):
            return [0, 0, 0], 0.0, False, 0.3 < t < 0.5
        stats, events = self.session([], press_y, 9.0, q0=start)
        self.assertIn("reset", events)
        np.testing.assert_allclose(stats["q_end"], teleop.HOME_DEG, atol=1.0)

        def press_y_then_a(t):  # A 1 s into the move: stop there, and A must not also start a recording
            return [0, 0, 0], 0.0, 1.5 < t < 1.7, 0.3 < t < 0.5
        stats, events = self.session([], press_y_then_a, 4.0, q0=start)
        self.assertEqual(events, ["reset"])
        self.assertGreater(np.max(np.abs(stats["q_end"] - np.array(teleop.HOME_DEG))), 5.0)

    def test_stop_all_during_the_y_move_stops_the_arms(self):
        """Ctrl+C / GUI Stop all (SIGINT / SIGTERM only set the main loop's stop flag) must also end the home move."""
        import os
        import signal
        start = [20, 10, 50, 0, 30, 0]
        sent = []

        def press_y(t):
            return [0, 0, 0], 0.0, False, 0.3 < t < 0.5

        def probe(t, node):  # called on every Quest read, also from the reset move's abort check
            if t > 1.5 and not sent:
                sent.append(1)
                os.kill(os.getpid(), signal.SIGINT)
        t0 = time.monotonic()
        stats, events = self.session([], press_y, 30.0, q0=start, probe=probe)
        self.assertEqual(events, ["reset"])
        self.assertLess(time.monotonic() - t0, 8.0)  # did not run the move + settle to the end
        self.assertGreater(np.max(np.abs(stats["q_end"] - np.array(teleop.HOME_DEG))), 5.0)

    def test_y_on_a_dry_run_does_not_move(self):
        start = [20, 10, 50, 0, 30, 0]

        def press_y(t):
            return [0, 0, 0], 0.0, False, 0.3 < t < 0.5
        stats, events = self.session(["--dry-run"], press_y, 2.0, q0=start)
        self.assertIn("reset", events)
        np.testing.assert_allclose(stats["q_end"], start, atol=0.1)

    def overshoot(self, extra):
        """Hand sweeps 0.3 m sideways in 1 s and stops: how far does the arm run past where it finally settles?"""
        hist = []

        def sweep(t):
            if t < 0.4:
                return [0, 0, 0], 0.0, False, False
            return [0, 0.3 * min((t - 0.4) / 1.0, 1.0), 0], 1.0, False, False

        times = []

        def probe(t, node):
            if node.arms["left"].q_meas is not None:
                hist.append(node.arms["left"].q_meas.copy())
                times.append(t)
        stats, _ = self.session(extra, sweep, 7.0, probe=probe)
        q = np.rad2deg(np.array(hist))
        start, end = q[0], q[-1]
        d = np.sign(end - start)
        over = np.max((q - end) * d, axis=0)  # beyond the final value, in the direction of the move
        j = int(np.argmax(np.abs(end - start)))
        done = np.abs(q[:, j] - start[j]) >= 0.9 * abs(end[j] - start[j])
        stats["t90"] = float(np.array(times)[done][0] - 0.4)  # hand starts at 0.4 s
        return float(np.max(over)), float(np.max(np.abs(end - start))), stats

    def test_acceleration_limit_stops_the_arm_without_overshoot(self):
        over_on, moved, stats = self.overshoot([])
        self.assertGreater(moved, 20.0)
        self.assertEqual(stats["let_go"], 0)
        over_off, _, _ = self.overshoot(["--max-joint-acc", "0"])  # same speed, no acceleration limit
        self.assertLess(over_on, 7.0)  # default 0.52 rad/s^2: 3.5-5.6 deg on the fake arm (timing noise), 11.7 without
        self.assertGreater(over_off, over_on + 5.0)

    def test_accel_step(self):
        dt, vmax, amax = 1 / 30, 0.52, 0.45
        q, v = np.zeros(6), np.zeros(6)
        target = np.deg2rad([40, -20, 0, 0, 10, 0])
        vs = []
        for _ in range(300):
            q_new, v_new = teleop.accel_step(q, v, target, vmax, amax, dt)
            self.assertLessEqual(float(np.max(np.abs(v_new - v))), amax * dt + 1e-12)
            self.assertLessEqual(float(np.max(np.abs(v_new))), vmax + 1e-12)
            q, v = q_new, v_new
            vs.append(v)
            d = target - q
            if np.any(np.abs(d) > 1e-6):  # direction kept: q moves along the straight joint-space line
                np.testing.assert_allclose(np.cross(d[[0, 1]], target[[0, 1]]), 0, atol=1e-9)
        np.testing.assert_allclose(q, target, atol=np.deg2rad(0.2))
        self.assertLessEqual(float(np.max(np.rad2deg(q - target) * np.sign(target), initial=0)), 0.2)

    def test_trace_csv(self):
        """latest_run/teleop_trace.csv: one row per tick with hand / IK / command / measured joints and the log."""
        import csv
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "latest_run" / "teleop_trace.csv"
            self.session(["--trace", str(path)], wave, 4.0)
            lines = path.read_text().splitlines()
            self.assertTrue(lines[0].startswith("# robot_remote teleop trace"))
            rows = list(csv.DictReader(ln for ln in lines if not ln.startswith("#")))
        self.assertEqual(list(rows[0]), teleop.Trace.COLS)
        tele = [r for r in rows if r["mode"] == "teleop"]
        self.assertGreater(len(tele), 90)  # ~30 Hz for 4 s
        engaged = [r for r in tele if r["engaged"] == "1"]
        self.assertTrue(engaged)
        r = engaged[-1]
        for k in ("hand_y", "target_x", "ik1", "cmd1", "meas1", "lag_deg", "cmd_speed_dps"):
            float(r[k])
        self.assertLessEqual(max(float(x["lag_deg"]) for x in engaged), 10.5)
        self.assertTrue(any("leash" in x["limited"] or "accel" in x["limited"] for x in engaged))
        notes = " ".join(x["note"] for x in rows)
        self.assertIn("[left] engaged", notes)

    def test_runs_on_the_isolated_test_domain(self):
        rclpy.init()
        try:
            dom = rclpy.get_default_context().get_domain_id()
            self.assertEqual(str(dom), ros_isolation.TEST_DOMAIN)
            ros_isolation.check(dom)
        finally:
            rclpy.try_shutdown()

    def test_refuses_to_start_next_to_another_command_publisher(self):
        rclpy.init()
        node = drv = None
        try:
            drv = FakeRebelDriver(["left"], teleop.HOME_DEG)
            drv.node.create_publisher(JointTrajectory, "/left/rebel_arm_trajectory_controller/joint_trajectory", 10)
            args, _ = teleop.parse_args(["--backend", "real-ros", "--arms", "left", "--quiet"])
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            node = teleop.DualArmTeleop(args, chain)
            time.sleep(0.5)  # discovery
            with self.assertRaises(teleop.RobotError):
                node.wait_ready()
        finally:
            if node:
                node.destroy_node()
            if drv:
                drv.close()
            rclpy.try_shutdown()

    def test_another_command_publisher_mid_session_stops_the_teleop_commands(self):
        state = {}

        def probe(t, node):
            if t > 2.0 and "pub" not in state:  # a second command source appears while the arm is driven
                state["node"] = rclpy.create_node("intruder")
                state["pub"] = state["node"].create_publisher(
                    JointTrajectory, "/left/rebel_arm_trajectory_controller/joint_trajectory", 10)
                state["t"] = t
            if t > 5.2 and "node" in state and "gone" not in state:  # must not outlive this test
                state["node"].destroy_node()
                state["gone"] = True
            if t > 3.5 and "engaged" not in state:
                state["engaged"] = node.arms["left"].engaged
                state["q_cmd"] = node.arms["left"].q_cmd.copy()
            if t > 5.0 and "q_cmd_late" not in state:
                state["q_cmd_late"] = node.arms["left"].q_cmd.copy()
        try:
            stats, _ = self.session([], wave, 5.5, probe=probe)
        finally:
            if "node" in state and "gone" not in state:
                state["node"].destroy_node()
        time.sleep(1.0)  # let the graph forget the intruder before the next test
        self.assertFalse(state["engaged"])
        np.testing.assert_allclose(state["q_cmd_late"], state["q_cmd"])  # no new commands while it exists

    def release_while_moving(self, extra):
        """Hand sweeps fast, the grip is let go mid-motion: (overshoot past the final rest, final rest - last command)."""
        hist, rel = [], {}

        def sweep(t):
            if t < 0.4:
                return [0, 0, 0], 0.0, False, False
            return [0, 0.3 * min((t - 0.4) / 1.0, 1.0), 0], (1.0 if t < 1.6 else 0.0), False, False

        def probe(t, node):
            arm = node.arms["left"]
            if arm.q_meas is not None:
                hist.append((t, arm.q_meas.copy()))
            if t > 1.6 and "cmd" not in rel and arm.q_cmd is not None:
                rel["t"], rel["cmd"] = t, arm.q_cmd.copy()
        stats, _ = self.session(extra, sweep, 6.0, probe=probe)
        t = np.array([h[0] for h in hist])
        q = np.rad2deg(np.array([h[1] for h in hist]))
        after = q[t > rel["t"]]
        rest = after[-1]
        j = int(np.argmax(np.abs(after[0] - after[-1]) + np.abs(np.diff(after, axis=0)).sum(0)))
        d = np.sign(rest[j] - after[0, j]) or 1.0
        come_back = float(np.max((after[:, j] - rest[j]) * d))
        return come_back, float(rest[j] - after[0, j]), stats

    def test_release_brake_stops_without_coming_back(self):
        back_on, travel_on, _ = self.release_while_moving([])
        back_off, travel_off, _ = self.release_while_moving(["--no-release-brake"])
        self.assertLess(back_on, 4.0)  # fake ReBeL: 2.7 deg with the brake, 9.5 without
        self.assertGreater(back_off, back_on + 4.0)

    def test_x_button_switches_the_rotation_frame(self):
        """Left X: rotation frame tool <-> base (no recording event); A on the right still starts / saves."""
        frames = []

        def press_x_twice(t):
            return [0, 0, 0], 0.0, (0.3 < t < 0.5) or (1.0 < t < 1.2), False

        def probe(t, node):
            if not frames or frames[-1] != node.rot_frame:
                frames.append(node.rot_frame)
        _, events = self.session(["--rot-frame", "tool"], press_x_twice, 1.6, probe=probe)
        self.assertEqual(frames, ["tool", "base", "tool"])
        self.assertNotIn("toggle", events)

    def test_leash_helpers(self):
        q_meas = np.zeros(6)
        q = teleop.leash(np.deg2rad([20, -10, 0, 0, 5, 0]), q_meas, np.deg2rad(6))
        np.testing.assert_allclose(np.rad2deg(q), [6, -3, 0, 0, 1.5, 0], atol=1e-9)  # direction kept
        np.testing.assert_allclose(teleop.leash(np.deg2rad([1, 2, 0, 0, 0, 0]), q_meas, np.deg2rad(6)),
                                   np.deg2rad([1, 2, 0, 0, 0, 0]))


if __name__ == "__main__":
    unittest.main()
