"""Thumbstick rotation of the tool while gripping (quest_teleop/stick_rotation.py + the IK teleop loop).

Unit: dead zone / curve / sign / clip, the axes, the rate cap.
Integration: the real teleop (--backend real-ros) with a fake Quest that has thumbsticks and the fake igus driver of
tests/test_real_ros_tracking.py (a JTC + plant that behaves like the ReBeL). Everything runs on the private test
domain (tests/ros_isolation.py) - never on the robot's ROS graph.
"""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402,F401  - FIRST: never share the robot's ROS graph
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "quest_teleop"))
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
import stick_rotation as sr  # noqa: E402

try:
    import rclpy
    import quest_dual_arm_teleop as teleop
    from test_real_ros_tracking import FakeRebelDriver
except ImportError:  # pragma: no cover
    teleop = None

RIGHT = np.array([0.0, -1.0, 0.0])  # operator right in the base frame (yaw 0)


class StickMathTests(unittest.TestCase):
    def test_clean_clips_and_rejects_bad_values(self):
        self.assertEqual(sr.clean((0.3, -0.4)), (0.3, -0.4))
        self.assertEqual(sr.clean((5.0, -7.0)), (1.0, -1.0))
        for bad in (None, (), (0.5,), (float("nan"), 0.2), (0.2, float("inf")), ("a", 1), 3):
            self.assertEqual(sr.clean(bad), (0.0, 0.0), bad)

    def test_deadzone_curve_and_sign(self):
        dz = 0.15
        self.assertEqual(sr.shape(0.0, dz), 0.0)
        self.assertEqual(sr.shape(0.15, dz), 0.0)
        self.assertEqual(sr.shape(-0.1, dz), 0.0)
        self.assertAlmostEqual(sr.shape(1.0, dz), 1.0)
        self.assertAlmostEqual(sr.shape(-1.0, dz), -1.0)
        mid = (0.575 - dz) / (1 - dz)  # half-way between the dead zone and full deflection
        self.assertAlmostEqual(sr.shape(0.575, dz), 0.5 ** 2)
        self.assertAlmostEqual(sr.shape(-0.575, dz), -(0.5 ** 2))
        self.assertAlmostEqual(mid, 0.5)
        s = np.linspace(0.16, 1.0, 20)
        self.assertTrue(np.all(np.diff([sr.shape(x, dz) for x in s]) > 0))  # monotonic
        self.assertEqual(sr.shape(0.9, 0.0), 0.9 ** 2)  # dead zone 0 is allowed

    def test_axes_and_signs(self):
        z = np.array([1.0, 0.0, 0.0])  # tool horizontal, pointing +x
        spin = sr.omega((1.0, 0.0), z, RIGHT, "spin")
        np.testing.assert_allclose(spin, np.deg2rad(15) * z, atol=1e-12)  # right = + about the tool's Z
        yaw = sr.omega((1.0, 0.0), z, RIGHT, "yaw")
        np.testing.assert_allclose(yaw, [0, 0, -np.deg2rad(15)], atol=1e-12)  # right = aim right = - about +Z
        tilt = sr.omega((0.0, 1.0), z, RIGHT)
        # stick up: positive rotation about the tilt axis moves the tool Z toward up
        self.assertGreater(Rotation.from_rotvec(tilt * 0.1).apply(z)[2], 0.0)
        down = sr.omega((0.0, -1.0), z, RIGHT)
        self.assertLess(Rotation.from_rotvec(down * 0.1).apply(z)[2], 0.0)
        flipped = sr.omega((1.0, 0.0), z, RIGHT, "spin", signs=(-1.0, 1.0))
        np.testing.assert_allclose(flipped, -spin, atol=1e-12)
        np.testing.assert_allclose(sr.omega((0.0, 1.0), z, RIGHT, signs=(1.0, -1.0)), down, atol=1e-12)
        # inside the dead zone / unreadable: nothing
        for idle in ((0.1, 0.1), (0.0, 0.0), None, (float("nan"), 1.0)):
            np.testing.assert_array_equal(sr.omega(idle, z, RIGHT), np.zeros(3))

    def test_tilt_axis_when_the_tool_is_vertical_uses_the_operator_right_axis(self):
        down = np.array([0.0, 0.0, -1.0])
        np.testing.assert_allclose(sr.tilt_axis(down, RIGHT), RIGHT)
        np.testing.assert_allclose(sr.tilt_axis(np.array([0.05, 0.0, -1.0]) / np.linalg.norm([0.05, 0, -1]), RIGHT), RIGHT)
        tilted = np.array([0.6, 0.0, -0.8])  # 37 deg from vertical: z x up
        ax = sr.tilt_axis(tilted, RIGHT)
        self.assertAlmostEqual(float(np.linalg.norm(ax)), 1.0)
        self.assertGreater(Rotation.from_rotvec(ax * 0.1).apply(tilted)[2], tilted[2])  # toward up

    def test_rate_is_capped_by_max_rot_vel(self):
        z = np.array([1.0, 0.0, 0.0])
        full = np.linalg.norm(sr.omega((1.0, 0.0), z, RIGHT, rate_dps=15, max_dps=45))
        self.assertAlmostEqual(np.rad2deg(full), 15.0)
        capped = np.linalg.norm(sr.omega((1.0, 0.0), z, RIGHT, rate_dps=60, max_dps=20))
        self.assertAlmostEqual(np.rad2deg(capped), 20.0)
        both = np.linalg.norm(sr.omega((1.0, 1.0), z, RIGHT, rate_dps=40, max_dps=45))  # diagonal: 40 * sqrt(2) -> 45
        self.assertAlmostEqual(np.rad2deg(both), 45.0)
        np.testing.assert_array_equal(sr.omega((1.0, 1.0), z, RIGHT, rate_dps=0), np.zeros(3))

    @unittest.skipIf(teleop is None, "source ROS and use .venv_quest/bin/python")
    def test_quest_controller_reads_the_thumbstick(self):
        from types import SimpleNamespace
        q = object.__new__(teleop.QuestController)
        q._idx = SimpleNamespace(THUMBSTICK_X=0, THUMBSTICK_Y=1)
        self.assertEqual(q._stick([0.25, -0.5]), (0.25, -0.5))
        self.assertEqual(q._stick([3.0, -3.0]), (1.0, -1.0))                       # clipped
        self.assertEqual(q._stick([float("nan"), 0.5]), (0.0, 0.0))                # not finite
        self.assertEqual(q._stick([0.5]), (0.0, 0.0))                              # field missing
        self.assertEqual(q._stick(SimpleNamespace(is_none=True)), (0.0, 0.0))      # untracked controller

    @unittest.skipIf(teleop is None, "source ROS and use .venv_quest/bin/python")
    def test_options(self):
        a, _ = teleop.parse_args([])
        self.assertEqual((a.rot_input, a.stick_rot_vel, a.stick_deadzone, a.stick_x, list(a.stick_signs)),
                         ("hand+stick", 15.0, 0.15, "spin", [1.0, 1.0]))
        a, _ = teleop.parse_args(["--rot-input", "stick", "--stick-x", "yaw", "--stick-signs", "-1", "1"])
        self.assertEqual((a.rot_input, a.stick_x, list(a.stick_signs)), ("stick", "yaw", [-1.0, 1.0]))
        for bad in (["--stick-rot-vel", "-1"], ["--stick-deadzone", "1"], ["--stick-rot-vel", "nan"],
                    ["--rot-input", "wrist"], ["--stick-x", "roll"]):
            with self.subTest(bad), patch("sys.stderr"), self.assertRaises(SystemExit):
                teleop.parse_args(bad)

    @unittest.skipIf(teleop is None, "source ROS and use .venv_quest/bin/python")
    def test_trace_has_the_stick_columns_after_trigger(self):
        cols = teleop.Trace.COLS
        i = cols.index("trigger")
        self.assertEqual(cols[i + 1:i + 3], ["stick_x", "stick_y"])
        self.assertEqual(len(cols), len(set(cols)))


class StickQuest:
    """Left controller from script(t) -> dict(pos, squeeze, stick, quat); has thumbsticks like QuestController."""

    def __init__(self, script, duration):
        self.script, self.duration, self.t0 = script, duration, None
        self.buttons = {"left": (False, False)}
        self.sticks = {"left": (0.0, 0.0)}

    def read_all(self):
        if self.t0 is None:
            self.t0 = time.monotonic()
        t = time.monotonic() - self.t0
        if t > self.duration:
            raise KeyboardInterrupt
        d = self.script(t)
        self.sticks = {"left": tuple(d.get("stick", (0.0, 0.0)))}
        return {"left": (True, np.asarray(d.get("pos", [0, 0, 0]), float), np.asarray(d.get("quat", [0, 0, 0, 1.0]), float),
                         float(d.get("squeeze", 1.0)), 0.0)}

    def close(self):
        pass


@unittest.skipIf(teleop is None, "source ROS and use .venv_quest/bin/python for node integration")
class StickRotationTeleopTests(unittest.TestCase):
    def run_session(self, script, duration, extra=(), q0=None):
        """Real teleop (real-ros, left arm) against the fake driver. Returns (samples, notes, node-state):
        samples = [(t, R_tool, p_tool, engaged)] of the MEASURED arm, taken on every Quest read."""
        q0 = teleop.HOME_DEG if q0 is None else q0
        rclpy.init()
        node = drv = None
        samples, notes = [], []
        try:
            args, _ = teleop.parse_args(["--backend", "real-ros", "--arms", "left", "--quiet", "--no-start-home",
                                         "--no-park", *extra])
            chain = teleop.SerialChain(teleop.make_urdf(), "base_link", "link6", teleop.ARM_JOINTS,
                                       tcp_xyz=teleop.TCP_PRESETS["flange"])
            drv = FakeRebelDriver(["left"], q0)
            node = teleop.DualArmTeleop(args, chain)
            arm = node.arms["left"]
            quest = StickQuest(script, duration)
            read = quest.read_all
            orig_note = node.trace.note

            def read_all():
                out = read()
                if arm.q_meas is not None:
                    t = chain.fk(arm.q_meas)
                    cmd_r = getattr(arm, "cmd_r", None)
                    lead = 0.0 if cmd_r is None or arm.q_cmd is None else np.rad2deg(
                        (cmd_r * Rotation.from_matrix(chain.fk(arm.q_cmd)[:3, :3]).inv()).magnitude())
                    samples.append((time.monotonic() - quest.t0, Rotation.from_matrix(t[:3, :3]), t[:3, 3].copy(),
                                    arm.engaged, lead))
                return out
            quest.read_all = read_all
            with patch.object(teleop, "QuestController", return_value=quest), \
                    patch.object(node.trace, "note", side_effect=lambda m: (notes.append(m), orig_note(m))):
                teleop.run_session(node, chain, args)
            return samples, notes, dict(lead=samples[-1][4])
        finally:
            if node:
                node.destroy_node()
            if drv:
                drv.close()
            rclpy.try_shutdown()

    @staticmethod
    def at(samples, t):
        return min(samples, key=lambda s: abs(s[0] - t))

    @staticmethod
    def rel(a, b):
        """Rotation of b relative to a, expressed in a's own (tool) frame."""
        return (a[1].inv() * b[1]).as_rotvec()

    def test_stick_x_spins_the_tool_about_its_own_z_and_stops_when_released(self):
        def script(t):
            return dict(squeeze=0.0 if t < 0.4 else 1.0, stick=(1.0, 0.0) if 1.0 <= t < 3.0 else (0.0, 0.0))
        s, notes, _ = self.run_session(script, 6.0)
        a, b, c, d = self.at(s, 1.0), self.at(s, 3.6), self.at(s, 6.0), None
        spin = self.rel(a, b)
        self.assertGreater(spin[2], np.deg2rad(18), spin)             # 15 deg/s x 2 s = 30 deg (arm lag, accel limit)
        self.assertLess(spin[2], np.deg2rad(34), spin)
        self.assertLess(np.linalg.norm(spin[:2]), np.deg2rad(4), spin)  # about the tool Z, not tilted
        self.assertLess(np.linalg.norm(b[2] - a[2]), 0.01)            # the flange stays put
        more = self.rel(b, c)
        self.assertLess(np.linalg.norm(more), np.deg2rad(2.0), more)  # stick back to 0 -> the rotation stops
        self.assertEqual(sum("stick rotation held" in n for n in notes), 0)

    def test_stick_y_tilts_the_tool_up_and_down(self):
        z_up = lambda smp: smp[1].apply([0, 0, 1])[2]  # noqa: E731

        def up(t):
            return dict(squeeze=0.0 if t < 0.4 else 1.0, stick=(0.0, 1.0) if 1.0 <= t < 2.6 else (0.0, 0.0))

        def down(t):
            return dict(squeeze=0.0 if t < 0.4 else 1.0, stick=(0.0, -1.0) if 1.0 <= t < 2.6 else (0.0, 0.0))
        s, _, _ = self.run_session(up, 4.5)
        z0, z1 = z_up(self.at(s, 1.0)), z_up(self.at(s, 4.4))
        self.assertGreater(z1 - z0, 0.12, (z0, z1))                   # ~24 deg of tilt at 15 deg/s x 1.6 s
        self.assertLess(np.linalg.norm(self.at(s, 4.4)[2] - self.at(s, 1.0)[2]), 0.02)
        s, _, _ = self.run_session(down, 4.5)
        z0, z1 = z_up(self.at(s, 1.0)), z_up(self.at(s, 4.4))
        self.assertLess(z1 - z0, -0.12, (z0, z1))

    def test_stick_while_not_gripping_does_nothing(self):
        s, _, _ = self.run_session(lambda t: dict(squeeze=0.0, stick=(1.0, 1.0)), 3.0)
        self.assertFalse(any(x[3] for x in s))
        self.assertLess(np.linalg.norm(self.rel(s[0], s[-1])), np.deg2rad(0.2))
        self.assertLess(np.linalg.norm(s[-1][2] - s[0][2]), 1e-3)

    def test_rot_input_stick_ignores_the_hand_rotation_but_not_the_translation(self):
        def script(t):
            twist = Rotation.from_rotvec(np.deg2rad(30) * np.clip(t - 1.0, 0, 1) * np.array([0, -0.707, -0.707]))
            return dict(squeeze=0.0 if t < 0.4 else 1.0, quat=twist.as_quat(),
                        pos=[0.05 * np.clip(t - 1.0, 0, 1), 0, 0])
        s, _, _ = self.run_session(script, 4.0, ["--rot-input", "stick"])
        self.assertLess(np.linalg.norm(self.rel(self.at(s, 1.0), self.at(s, 3.9))), np.deg2rad(1.5))
        self.assertGreater(np.linalg.norm(self.at(s, 3.9)[2] - self.at(s, 1.0)[2]), 0.03)  # the hand still moves the tool
        # the same hand motion does turn the tool with the default (non-vacuous check)
        s, _, _ = self.run_session(script, 4.0)
        self.assertGreater(np.linalg.norm(self.rel(self.at(s, 1.0), self.at(s, 3.9))), np.deg2rad(15))

    def test_rot_input_stick_in_the_base_frame_also_ignores_the_hand(self):
        def script(t):
            twist = Rotation.from_rotvec(np.deg2rad(30) * np.clip(t - 1.0, 0, 1) * np.array([1.0, 0, 0]))
            return dict(squeeze=0.0 if t < 0.4 else 1.0, quat=twist.as_quat())
        s, _, _ = self.run_session(script, 3.5, ["--rot-input", "stick", "--rot-frame", "base"])
        self.assertLess(np.linalg.norm(self.rel(self.at(s, 1.0), self.at(s, 3.4))), np.deg2rad(1.5))

    def test_rot_input_stick_turns_with_the_stick(self):
        def script(t):
            return dict(squeeze=0.0 if t < 0.4 else 1.0, stick=(1.0, 0.0) if 1.0 <= t < 3.0 else (0.0, 0.0))
        s, _, _ = self.run_session(script, 5.0, ["--rot-input", "stick"])
        self.assertGreater(self.rel(self.at(s, 1.0), self.at(s, 4.9))[2], np.deg2rad(18))

    def test_rot_input_hand_ignores_the_stick(self):
        def script(t):
            return dict(squeeze=0.0 if t < 0.4 else 1.0, stick=(1.0, 1.0) if t >= 1.0 else (0.0, 0.0))
        s, _, _ = self.run_session(script, 3.5, ["--rot-input", "hand"])
        self.assertLess(np.linalg.norm(self.rel(self.at(s, 1.0), self.at(s, 3.4))), np.deg2rad(1.0))
        s, _, _ = self.run_session(script, 3.5, ["--stick-rot-vel", "0"])  # 0 = off, same thing
        self.assertLess(np.linalg.norm(self.rel(self.at(s, 1.0), self.at(s, 3.4))), np.deg2rad(1.0))

    def test_holding_the_stick_against_a_limit_builds_no_hidden_target(self):
        """Tilt up for far longer than the wrist can follow; after letting go the tool must not keep turning."""
        def script(t):
            return dict(squeeze=0.0 if t < 0.4 else 1.0, stick=(0.0, 1.0) if 1.0 <= t < 13.0 else (0.0, 0.0))
        s, notes, state = self.run_session(script, 17.0, ["--stick-rot-vel", "30"], q0=[0, 0, 65, 0, 50, 0])
        held = [n for n in notes if "stick rotation held" in n]
        self.assertTrue(held, "the wrist never reached its limit - make the test drive it further")
        self.assertIn("wrist cannot follow", held[0])
        a, b = self.at(s, 14.5), self.at(s, 16.9)
        self.assertLess(np.linalg.norm(self.rel(a, b)), np.deg2rad(1.0), self.rel(a, b))  # 0.1 deg with the guard, 2.3 without
        self.assertLess(state["lead"], 8.0)                           # the command is not far ahead of the joints either


if __name__ == "__main__":
    unittest.main()
