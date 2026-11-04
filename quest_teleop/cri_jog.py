#!/usr/bin/env python3
"""Quest -> one or both real igus ReBeLs through the CONTROLLER'S OWN Cartesian jog (`--backend cri-jog`).

The controller does the inverse kinematics: this module never solves IK. It streams a Cartesian velocity
(geometry_msgs/TwistStamped, base frame, translation only) to each igus_rebel driver, which turns it into CRI ALIVEJOG
X/Y/Z while the driver is in `MotionTypeCartBase`:

  Quest grip pose --clutch--> displacement of the hand since the grip (operator frame -> arm base axes, --yaw-deg,
                              --scale, --cri-signs: the same hand mapping as the IK backends)
    target  = robot position at the grip + that displacement          (robot_cart_pose, POSCARTROBOT)
    v       = clamp(kp * (target - robot position), --cri-max-lin-vel) (deadband --cri-deadband)
    --> cart_jog_cmd of that arm (>= 20 Hz, stamped now(), angular = 0)

One arm (`--arms right`): topics /igus_rebel/...; two arms (`--arms left right`): /igus_rebel_{side}/... (the dual
stack has one driver instance per arm). Each arm has its own clutch, follow monitor and re-grip state; hands map to arms
like the IK backends (--swap-hands).

Driver side (igus_rebel_ros2, README "Teleoperation through the controller's Cartesian jog"): `cart_jog_enable`
(SetBool), `cart_jog_cmd`, `hardware_state`, `robot_cart_pose`. The driver refuses the mode while a trajectory controller
is active (deactivate it first), clamps to cart_jog_max_percent and zeroes the jog 0.15 s after the last command.

Safety behaviour (fake-tested only, see tests/test_cri_jog_teleop.py):
  - Release / tracking loss of ONE hand stops only that arm. A fault of ONE arm stops BOTH (like the IK path): stale
    pose, driver latched / disconnected / stale hardware_state, controller KINSTATE != 0, another publisher on a
    cart_jog_cmd topic, the driver leaving the Cartesian mode by itself, the arm moving against its command
    (permanent latch of that arm), the arm guard not knowing where an arm is. Both arms then get zero twist and the grips
    must be opened and closed again. An arm that merely does not follow its command is let go on its own.
  - Arm-to-arm guard (default on; `--no-arm-guard` turns it off with a loud warning): the capsule model of
    arm_collision.py (FK only) on the joints each driver reports (robot_cart_pose.joint_position_rad). REACTIVE: the
    next joints are unknown, so it works on the newest measured pose:
      near zone   clearance <= --min-arm-dist + margin, margin = n_driven * --cri-max-lin-vel * T, with
                  T = --cri-pose-timeout (0.2 s, the oldest pose still trusted) + one tick + 0.15 s (the driver's jog
                  watchdog / stopping allowance) ~ 0.4 s: both arms may approach at full speed during T.
      in the zone an arm is stopped (zero twist, "arm guard" log, re-grip needed; moving away is allowed) when
        (a) its commanded velocity points toward the other arm along the closest-point direction and the closest point
            is on the end-effector body (flange / gripper / knife / joint-6 origin: with angular = 0 these move exactly
            like the commanded TCP, so the commanded velocity predicts the clearance rate), or
        (b) the measured clearance is shrinking (> 2 mm/s over >= 0.25 s): a backstop for closest points on the
            forearm / elbow (their velocity is the controller's IK and unknown) or a mirrored axis.
      (a) is evaluated on the command itself, so after a re-grip an approach is refused at once instead of creeping
      closer in steps; both rules only remove the approach, they never push apart, so the guard cannot oscillate.
    The guard assumes the controller's base axes equal URDF base_link (the --cri-signs only mirror the hand mapping).
    Single arm: the other arm is assumed at --other-arm-deg, or read live from its robot_cart_pose with --watch-other
    (read-only; a stale pose stops the driven arm). Stand, table and cameras are not modelled.
  - At exit (Ctrl+C / SIGTERM / Stop all / exception): zero twist to every arm, then `cart_jog_enable false` on every arm
    this session entered. No automatic home / park motion: a robot moves only while its grip is held.
  - `--dry-run`: computes and traces the twists, publishes nothing, never calls the service.

Rotation is NOT implemented (the meaning of the controller's A/B/C jog axes is unmeasured): angular is always zero
and the X (rotation frame) button has no effect.
TODO(rotation): map the hand rotation to angular velocity once the A/B/C axes are measured, and set the driver's
cart_jog_allow_rotation.

/quest_teleop/action: nothing is published (no joint commands exist here and none are invented); the operator's command
is each arm's cart_jog_cmd and the trace. /quest_teleop/{side}/hand, /state, /events and /joint_states (measured joints
from robot_cart_pose, for RViz) are still published.

Imports: numpy, rclpy and the standard ROS message packages. igus_rebel_msgs (source the igus_rebel_ros2 install) is
imported when the node is created; the Quest reader (quest_arm_teleop, isaacteleop) when the session starts; the URDF
chain and arm_collision (scipy, igus_rebel_description) when the guard is built.
"""

import csv
import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

HERE = Path(__file__).resolve().parent
SIDES = ("left", "right")
ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
# one arm: the single-instance driver; two arms: one driver instance per arm (rebel_dual_table_description)
SINGLE_TOPICS = {"cri_cmd_topic": "/igus_rebel/cart_jog_cmd", "cri_enable_service": "/igus_rebel/cart_jog_enable",
                 "cri_state_topic": "/igus_rebel/hardware_state", "cri_pose_topic": "/igus_rebel/robot_cart_pose"}
DUAL_TOPICS = {k: v.replace("/igus_rebel/", "/igus_rebel_{side}/") for k, v in SINGLE_TOPICS.items()}
DEFAULT_WATCH_POSE_TOPIC = "/igus_rebel_{side}/robot_cart_pose"
MAX_CRI_LIN_VEL = 0.1      # m/s; the driver clamps to cart_jog_max_percent (10 %) anyway
MIN_RATE_HZ = 20.0         # the driver zeroes the jog 0.15 s after the last command
ZERO_TICKS = 5             # zero twists published after an arm stops being driven (the topic is lossy)
DRIVER_STOP_S = 0.15       # the driver's jog watchdog: allowance for the stopping time in the guard margin
EE_FIRST_CAPSULE = 5       # capsules 5.. (joint-6 origin->flange, flange->jaw, flange->knife) move like the TCP


class CriJogError(RuntimeError):
    """The session cannot start or must end (enable refused, no driver, another command publisher ...)."""


class YHold:
    """Left controller Y (the reset / discard button) that must be HELD `hold_s` seconds on the real robot, so a thumb
    brushing the button does not discard the episode or send the arms home (real logs 2026-11-03: 18 accidental
    presses in one day). hold_s <= 0 = immediate (the previous behaviour: act on the rising edge; the simulator).

    update() is called once per tick and returns True exactly once per hold, at the moment the hold time is reached
    (not on release). A hold starts on a rising edge of Y while no other button is pressed (a grip that is already
    squeezed does not matter: that is how the operator holds the controller). It is cancelled by Y being released
    early (one info log) or by a NEW press of another button or a NEW grip squeeze while it runs (the other button
    still acts normally in the caller). A Y that stays pressed after a completed or cancelled hold does nothing until it
    is released and pressed again. resync() after a reset move marks the current buttons as already seen."""

    def __init__(self, hold_s, log, long_what, clutch=0.5, what="reset", hand="left"):
        self.hold_s, self.log, self.clutch, self.hand = float(hold_s), log, clutch, hand
        self.what, self.long_what = what, long_what   # short / long description of what the hold does, for the logs
        self.start = None      # monotonic time the current hold began
        self.prev_y = False    # Y at the previous tick
        self.seen = frozenset()  # other buttons / grips pressed at the previous tick

    def _y(self, buttons):
        return bool(buttons.get(self.hand, (False, False))[1])

    def _others(self, buttons, readings):
        """(other buttons pressed, grips squeezed) right now, as sets of names."""
        btn = set()
        for h, (prim, sec) in buttons.items():
            if prim:
                btn.add(f"{h} {'X' if h == 'left' else 'A'}")
            if sec and h != self.hand:
                btn.add(f"{h} B")
        grip = {f"{h} grip" for h, r in (readings or {}).items() if r[0] and r[3] > self.clutch}
        return btn, grip

    def resync(self, buttons, readings=None):
        self.prev_y, self.start = self._y(buttons), None
        btn, grip = self._others(buttons, readings)
        self.seen = frozenset(btn | grip)

    def update(self, buttons, readings, now):
        y, prev = self._y(buttons), self.prev_y
        btn, grip = self._others(buttons, readings)
        new = (btn | grip) - self.seen        # pressed since the previous tick
        self.prev_y, self.seen = y, frozenset(btn | grip)
        if self.hold_s <= 0:
            return y and not prev
        if self.start is None:
            if y and not prev and not btn:
                self.start = now
                self.log.info(f"[{self.hand} hand] Y held - keep holding {self.hold_s:.1f} s to {self.what}")
            return False
        if not y:
            self.log.info(f"[{self.hand} hand] Y released after {now - self.start:.1f} s - hold {self.hold_s:.1f} s "
                          f"to {self.long_what}")
            self.start = None
            return False
        if new:
            self.log.info(f"[{self.hand} hand] Y hold cancelled ({', '.join(sorted(new))}) - release Y and hold it "
                          f"again to {self.what}")
            self.start = None
            return False
        if now - self.start >= self.hold_s:
            self.start = None
            return True
        return False


# ----------------------------------------------------------------------------- arguments
def parse_signs(text):
    try:
        vals = [float(x) for x in str(text).replace(" ", "").split(",")]
    except ValueError:
        raise ValueError(f"{text!r} is not three comma separated numbers") from None
    if len(vals) != 3 or any(v not in (1.0, -1.0) for v in vals):
        raise ValueError(f"{text!r}: need three values, each 1 or -1 (e.g. 1,1,-1)")
    return tuple(vals)


def add_arguments(p):
    g = p.add_argument_group("cri-jog backend (the controller's own Cartesian jog; one or two arms, translation only)")
    g.add_argument("--cri-cmd-topic", default=None,
                   help="TwistStamped jog command topic of the driver; {side} = left / right. Default "
                        f"{SINGLE_TOPICS['cri_cmd_topic']} (one arm) or {DUAL_TOPICS['cri_cmd_topic']} (two arms)")
    g.add_argument("--cri-enable-service", default=None,
                   help="std_srvs/SetBool that enters / leaves the driver's Cartesian jog mode; {side} allowed. "
                        f"Default {SINGLE_TOPICS['cri_enable_service']} / {DUAL_TOPICS['cri_enable_service']}")
    g.add_argument("--cri-state-topic", default=None,
                   help="igus_rebel_msgs/HardwareState topic of the driver; {side} allowed. "
                        f"Default {SINGLE_TOPICS['cri_state_topic']} / {DUAL_TOPICS['cri_state_topic']}")
    g.add_argument("--cri-pose-topic", default=None,
                   help="igus_rebel_msgs/RobotCartPose topic (the controller's POSCARTROBOT); {side} allowed. "
                        f"Default {SINGLE_TOPICS['cri_pose_topic']} / {DUAL_TOPICS['cri_pose_topic']}")
    g.add_argument("--cri-watch-pose-topic", default=DEFAULT_WATCH_POSE_TOPIC,
                   help="--watch-other with one arm: RobotCartPose topic of the OTHER arm ({side} = that arm)")
    g.add_argument("--cri-max-lin-vel", type=float, default=0.03,
                   help="largest Cartesian speed sent [m/s] (default 0.03; the driver clamps further). The driver's "
                        "%%-to-m/s mapping is an UNMEASURED placeholder: start low")
    g.add_argument("--cri-kp", type=float, default=2.0,
                   help="proportional gain [1/s]: speed = kp * (target - robot position)")
    g.add_argument("--cri-deadband", type=float, default=0.001,
                   help="no motion within this distance of the target [m]")
    g.add_argument("--cri-max-offset", type=float, default=0.30,
                   help="the target stays within this distance of the robot position at the grip [m]")
    g.add_argument("--cri-signs", default="1,1,1", metavar="SX,SY,SZ",
                   help="per-axis sign (1 or -1) applied to the mapped hand displacement, for a controller base frame "
                        "whose axes are mirrored against the URDF base_link (unknown until tools/igus_fk_check was run "
                        "on the real arm). Default 1,1,1. The arm guard assumes the two frames agree")
    g.add_argument("--cri-pose-units", choices=["mm", "m"], default="mm",
                   help="units of robot_cart_pose.pos_cart_robot_raw (believed mm)")
    g.add_argument("--cri-pose-timeout", type=float, default=0.2,
                   help="zero the jog when a robot pose is older than this [s]")
    g.add_argument("--cri-state-timeout", type=float, default=1.0,
                   help="zero the jog when hardware_state (10 Hz) is older than this [s]")
    g.add_argument("--cri-wait-s", type=float, default=30.0,
                   help="how long to wait for the driver's pose / state topics at start [s]")
    g.add_argument("--cri-no-auto-enable", action="store_true",
                   help="do not call cart_jog_enable true at start (the mode is then entered by hand and the teleop "
                        "moves only while hardware_state says it is active)")
    g.add_argument("--no-arm-guard", action="store_true",
                   help="run WITHOUT the arm-to-arm guard (no URDF needed; NOTHING keeps the two arms apart)")
    g.add_argument("--cri-guard-margin", type=float, default=None,
                   help="guard reaction allowance [m]; default n_driven * --cri-max-lin-vel * (--cri-pose-timeout + one "
                        "tick + 0.15 s)")


def validate(args, p):
    """Argument checks for --backend cri-jog (p.error exits with a message). Resolves the topic defaults."""
    if args.backend != "cri-jog":
        return
    args.arms = sorted(args.arms, key=SIDES.index)
    defaults = SINGLE_TOPICS if len(args.arms) == 1 else DUAL_TOPICS
    for key, default in defaults.items():
        if getattr(args, key) is None:
            setattr(args, key, default)
    if len(args.arms) == 2:
        for key in defaults:
            resolved = {getattr(args, key).format(side=s) for s in args.arms}
            if len(resolved) < 2:
                p.error(f"--arms left right: --{key.replace('_', '-')} {getattr(args, key)!r} resolves to the same name "
                        "for both arms; use a {side} placeholder, e.g. /igus_rebel_{side}/...")
    try:
        args.cri_signs = parse_signs(args.cri_signs)
    except ValueError as exc:
        p.error(f"--cri-signs: {exc}")
    if args.rate < MIN_RATE_HZ:
        p.error(f"--rate must be at least {MIN_RATE_HZ:.0f} Hz in cri-jog (the driver zeroes the jog after 0.15 s)")
    for key in ("cri_max_lin_vel", "cri_kp", "cri_max_offset", "cri_pose_timeout", "cri_state_timeout", "cri_wait_s"):
        if not np.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            p.error(f"--{key.replace('_', '-')} must be finite and positive")
    if not np.isfinite(args.cri_deadband) or args.cri_deadband < 0:
        p.error("--cri-deadband must be finite and >= 0")
    if args.cri_guard_margin is not None and (not np.isfinite(args.cri_guard_margin) or args.cri_guard_margin < 0):
        p.error("--cri-guard-margin must be finite and >= 0")
    if args.cri_max_lin_vel > MAX_CRI_LIN_VEL:
        p.error(f"--cri-max-lin-vel above {MAX_CRI_LIN_VEL} m/s is refused (the % mapping is unmeasured)")


# ----------------------------------------------------------------------------- control law
def jog_velocity(err, kp, vmax, deadband):
    """Proportional Cartesian speed toward the target: continuous at the deadband edge, norm limited to vmax.
    Returns (v[3], clamped)."""
    err = np.asarray(err, float)
    n = float(np.linalg.norm(err))
    if n <= deadband:
        return np.zeros(3), False
    v = kp * err * (1.0 - deadband / n)
    s = float(np.linalg.norm(v))
    if s > vmax:
        return v * (vmax / s), True
    return v, False


class FollowMonitor:
    """Does the robot follow the jog it is given? Over the last second compares the commanded displacement (integral
    of the sent velocity) with the measured one. Catches the two failures a velocity jog can have that the position
    feedback alone would turn into a runaway or a silent stall:
      opposite - the robot moved against the command (axis sign / driver cart_jog_signs wrong): positive feedback
      stalled  - it barely moves (motors off, e-stop, % mapping far too low)."""

    WINDOW_S = 1.0

    def __init__(self, min_cmd_m=0.02, reverse_m=0.01, follow_ratio=0.15):
        self.min_cmd, self.reverse, self.ratio = min_cmd_m, reverse_m, follow_ratio
        self.buf = deque()

    def reset(self):
        self.buf.clear()

    def update(self, t, pos, v):
        self.buf.append((t, np.array(pos, float), np.array(v, float)))
        while len(self.buf) > 2 and t - self.buf[1][0] >= self.WINDOW_S:
            self.buf.popleft()
        t0, p0, _ = self.buf[0]
        if t - t0 < 0.8 * self.WINDOW_S:
            return None
        c = sum((self.buf[i][2] * (self.buf[i + 1][0] - self.buf[i][0]) for i in range(len(self.buf) - 1)),
                np.zeros(3))
        nc = float(np.linalg.norm(c))
        if nc < self.min_cmd:
            return None
        proj = float(np.dot(np.asarray(pos, float) - p0, c) / nc)
        if proj < -self.reverse:
            return "opposite", f"moved {proj * 1e3:+.1f} mm along a {nc * 1e3:.1f} mm command"
        if proj < self.ratio * nc:
            return "stalled", f"moved {proj * 1e3:+.1f} mm along a {nc * 1e3:.1f} mm command"
        return None


class JogLogic:
    """Clutch + control law of one arm. Pure computation (no ROS): step() maps one tick of Quest data and the
    robot state to the Cartesian velocity to send."""

    def __init__(self, args, log, dry_run=False, side=None):
        self.args, self.log, self.dry = args, log, dry_run
        self.tag = "[cri]" if side is None else f"[cri {side}]"
        self.on_th, self.off_th = args.clutch + 0.1, args.clutch - 0.1   # grip hysteresis, like the real backends
        self.signs = np.asarray(args.cri_signs, float)
        yaw = np.deg2rad(args.yaw_deg)
        c, s = np.cos(yaw), np.sin(yaw)
        self.r_base_op = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # operator frame -> arm base axes
        self.engaged = False
        self.need_regrip = True            # the operator must show an open hand once before the first grip
        self.latch = None                  # permanent stop (arm moves against the command)
        self.origin_p = self.robot_snap = self.target = None
        self.limited = []
        self.monitor = FollowMonitor()
        self.last_fault = None

    def hand_in_base(self, pos):
        return self.r_base_op @ np.asarray(pos, float)

    def drop(self, why):
        if self.engaged:
            self.log.warning(f"{self.tag} released: {why} - nothing is sent; release and grip again")
        self.engaged = False
        self.need_regrip = True

    def step(self, now, tracked, hand_p, squeeze, cur, fault):
        """-> velocity [m/s] in the arm base axes (zeros unless the arm is being driven)."""
        a = self.args
        v = np.zeros(3)
        self.limited = []
        if fault != self.last_fault:
            if fault:
                self.log.warning(f"{self.tag} stopped: {fault}")
            elif self.last_fault:
                self.log.info(f"{self.tag} ok again - release and grip to drive the arm")
            self.last_fault = fault
        if self.latch:
            self.drop(self.latch)
            return v
        if fault:
            self.drop(fault)
            return v
        if not tracked or hand_p is None:
            self.drop("tracking lost")
            return v
        if self.need_regrip:
            if squeeze < self.off_th:
                self.need_regrip = False
            return v
        if not self.engaged:
            if squeeze < self.on_th:
                return v
            self.origin_p = self.hand_in_base(hand_p)
            self.robot_snap = np.array(cur, float)
            self.engaged = True
            self.monitor.reset()
            self.log.info(f"{self.tag} engaged: robot at " + np.array2string(self.robot_snap, precision=4) + " m")
        elif squeeze < self.off_th:
            self.engaged = False
            self.log.info(f"{self.tag} released")
            return v
        delta = self.signs * a.scale * (self.hand_in_base(hand_p) - self.origin_p)
        n = float(np.linalg.norm(delta))
        if n > a.cri_max_offset:
            delta *= a.cri_max_offset / n
            self.limited.append("offset")
        self.target = self.robot_snap + delta
        v, clamped = jog_velocity(self.target - cur, a.cri_kp, a.cri_max_lin_vel, a.cri_deadband)
        if clamped:
            self.limited.append("vmax")
        if not self.dry:
            res = self.monitor.update(now, cur, v)
            if res:
                kind, detail = res
                if kind == "opposite":
                    self.latch = (f"the robot moves AGAINST the jog command ({detail}); check the driver's "
                                  "cart_jog_signs / the axis directions. Refusing to move until the teleop is restarted")
                    self.log.error(f"{self.tag} {self.latch}")
                else:
                    self.log.warning(f"{self.tag} the robot does not follow the jog command ({detail}): motors off, "
                                     "e-stop, or the driver's %-to-m/s mapping far too low?")
                self.drop("robot does not follow the command")
                return np.zeros(3)
        return v


# ----------------------------------------------------------------------------- arm-to-arm guard
def segment_closest(p1, q1, p2, q2):
    """Closest points of segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection 5.1.9): (dist, c1, c2)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r)), p1, p2
    if a <= eps:
        s, t = 0.0, float(np.clip(f / e, 0.0, 1.0))
    else:
        c = d1 @ r
        if e <= eps:
            s, t = float(np.clip(-c / a, 0.0, 1.0)), 0.0
        else:
            b = d1 @ d2
            den = a * e - b * b
            s = float(np.clip((b * f - c * e) / den, 0.0, 1.0)) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                s, t = float(np.clip(-c / a, 0.0, 1.0)), 0.0
            elif t > 1.0:
                s, t = float(np.clip((b - c) / a, 0.0, 1.0)), 1.0
    c1, c2 = p1 + d1 * s, p2 + d2 * t
    return float(np.linalg.norm(c1 - c2)), c1, c2


def closest_pair(caps_a, caps_b):
    """(surface clearance, index in a, index in b, closest point on a, closest point on b) over all capsule pairs."""
    best = None
    for i, (p0, p1, ra) in enumerate(caps_a):
        for j, (s0, s1, rb) in enumerate(caps_b):
            dist, c1, c2 = segment_closest(p0, p1, s0, s1)
            if best is None or dist - ra - rb < best[0]:
                best = (dist - ra - rb, i, j, c1, c2)
    return best


class ArmGuard:
    """Reactive arm-to-arm guard (see the module docstring for the rule and the margin). measure() once per tick from the
    newest measured joints, then limit() on the velocities the arms are about to be sent."""

    RATE_WINDOW_S = 0.25
    RATE_TOL = 0.002      # m/s of clearance shrinkage that stops the commanding arms (backstop, measured)

    def __init__(self, chain, args, world_T_base, driven):
        from arm_collision import capsules
        self._capsules = capsules
        self.chain, self.args, self.T = chain, args, world_T_base
        self.driven = list(driven)
        self.watched = bool(getattr(args, "watch_other", False)) and len(self.driven) == 1
        self.assumed = {s: np.deg2rad(args.other_arm_deg) for s in SIDES if s not in self.driven and not self.watched}
        self.margin = (args.cri_guard_margin if args.cri_guard_margin is not None else
                       len(self.driven) * args.cri_max_lin_vel * (args.cri_pose_timeout + 1.0 / args.rate + DRIVER_STOP_S))
        self.min_dist = args.min_arm_dist
        self.tol_v = 0.05 * args.cri_max_lin_vel
        self.hist = deque()
        self.d = self.rate = None
        self.near = False
        self.idx, self.n, self.n_ok = {}, {}, False

    def measure(self, now, joints):
        """joints: {side: q[6] or None} of the driven (and watched) arms. Returns None, or why the guard is blind."""
        q = {}
        for s in SIDES:
            q[s] = self.assumed[s] if s in self.assumed else joints.get(s)
            if q[s] is None or not np.all(np.isfinite(q[s])):
                self.d = None
                return f"no fresh joint state of the {s} arm"
        caps = {s: self._capsules(self.chain, q[s], self.T[s], self.args.arm_radius, self.args.tool_radius)
                for s in SIDES}
        d, i, j, c_l, c_r = closest_pair(caps["left"], caps["right"])
        self.d, self.idx = d, {"left": i, "right": j}
        gap = c_l - c_r
        norm = float(np.linalg.norm(gap))
        self.n_ok = norm > 1e-6
        self.n = {"left": gap / norm, "right": -gap / norm} if self.n_ok else {}
        self.near = d <= self.min_dist + self.margin
        self.hist.append((now, d))
        while len(self.hist) > 1 and now - self.hist[1][0] >= self.RATE_WINDOW_S:
            self.hist.popleft()
        t_old, d_old = self.hist[0]
        self.rate = (d - d_old) / (now - t_old) if now - t_old >= self.RATE_WINDOW_S else None
        return None

    def limit(self, v):
        """v: {side: velocity in that arm's controller base axes}. -> {side: reason} of the arms that must stop."""
        blocked = {}
        if self.d is None or not self.near:
            return blocked
        violated = self.d <= self.min_dist
        commanding = [s for s, x in v.items() if float(np.linalg.norm(x)) > 1e-9]
        for s in commanding:
            if self.idx[s] < EE_FIRST_CAPSULE:
                continue   # closest point on a proximal link: its velocity is the controller's IK, see the rate backstop
            if not self.n_ok:
                blocked[s] = f"arms touching ({self.d * 100:.1f} cm clearance)"
                continue
            toward = -float(self.T[s][:3, :3] @ np.asarray(v[s]) @ self.n[s])
            if toward > (0.0 if violated else self.tol_v):
                blocked[s] = (f"moving toward the other arm at {toward * 1e3:.0f} mm/s, clearance "
                              f"{self.d * 100:.1f} cm (limit {self.min_dist * 100:.0f} cm + {self.margin * 100:.1f} cm)")
        if self.rate is not None and self.rate < (0.0 if violated else -self.RATE_TOL):
            for s in commanding:
                blocked.setdefault(s, f"clearance shrinking at {-self.rate * 1e3:.0f} mm/s, clearance "
                                      f"{self.d * 100:.1f} cm")
        return blocked


def make_urdf() -> str:
    """The single-arm URDF of the sourced igus_rebel_ros2 install (same as quest_dual_arm_teleop.make_urdf)."""
    pkg = subprocess.run(["ros2", "pkg", "prefix", "igus_rebel_description"], check=True,
                         capture_output=True, text=True).stdout.strip()
    xacro = Path(pkg) / "share/igus_rebel_description/urdf/igus_rebel_xeg32_wrapper.urdf.xacro"
    return subprocess.run(["xacro", str(xacro), "include_ros2_control:=false"], check=True,
                          capture_output=True, text=True).stdout


def build_guard(args, driven):
    """The guard, or None with --no-arm-guard. Without the URDF (or the calibration) the session is refused."""
    if args.no_arm_guard:
        return None
    try:
        from arm_collision import load_world_T_base
        from quest_arm_teleop import SerialChain, TCP_PRESETS
        tcp = tuple(args.tcp) if args.tcp else TCP_PRESETS[args.tcp_preset]
        chain = SerialChain(make_urdf(), "base_link", "link6", ARM_JOINTS, tcp_xyz=tcp)
        world_T_base = load_world_T_base(args.calibration)
        return ArmGuard(chain, args, world_T_base, driven)
    except Exception as exc:  # noqa: BLE001 - reported as the reason to refuse
        raise CriJogError(f"the arm-to-arm guard needs the arm URDF (igus_rebel_description from the sourced "
                          f"igus_rebel_ros2 install: ros2 pkg prefix igus_rebel_description, xacro) and "
                          f"{args.calibration}, but: {type(exc).__name__}: {exc}. Source the install, or pass "
                          "--no-arm-guard to run WITHOUT any protection between the two arms") from exc


# ----------------------------------------------------------------------------- trace / log
class CriTrace:
    """CSV, one row per arm per control tick (overwritten every start; same default path as the other backends)."""

    COLS = (["t", "mode", "side", "tracked", "engaged", "grip", "hand_x", "hand_y", "hand_z", "robot_x", "robot_y",
             "robot_z", "target_x", "target_y", "target_z", "err_mm", "vx", "vy", "vz", "speed_mps", "limited", "sent",
             "pose_age_ms", "pose_mode", "kinstate", "jog_state", "clearance_mm", "guard", "note"])

    def __init__(self, path, args):
        self.f, self.notes, self.last_note = None, [], {}
        self.t0 = time.monotonic()
        if not path:
            return
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(path, "w", newline="")
        self.f.write(f"# robot_remote teleop trace (backend cri-jog), started {time.strftime('%Y-%m-%d %H:%M:%S %z')}; "
                     "overwritten on every run\n# argv: " + " ".join(sys.argv[1:]) + "\n"
                     f"# limits: speed {args.cri_max_lin_vel} m/s, kp {args.cri_kp} 1/s, deadband {args.cri_deadband} m, "
                     f"offset {args.cri_max_offset} m, signs {list(args.cri_signs)}, rate {args.rate} Hz, arms "
                     f"{list(args.arms)}, arm guard {'OFF' if args.no_arm_guard else 'on'} (min {args.min_arm_dist} m)\n"
                     "# units: t [s since start]; hand_* [m, Quest controller in the operator frame]; robot_* / target_* "
                     "[m, controller base frame, robot_cart_pose]; v* = twist computed this tick [m/s, base axes]; "
                     "sent = 1 if a twist went to the driver (0 in a dry run); jog_state = driver hardware_state; "
                     "clearance_mm = smallest surface gap between the two arms (capsule model, measured joints; same "
                     "value in both arms' rows); guard = 1 if the arm guard stopped this arm this tick\n")
        self.w = csv.writer(self.f)
        self.w.writerow(self.COLS)
        self.last_flush = time.monotonic()

    def note(self, text):
        if self.f is None:
            return
        now = time.monotonic()
        if now - self.last_note.get(text, -1e9) < 1.0:
            return
        self.last_note[text] = now
        self.notes.append(text)

    def row(self, side, tracked, engaged, grip, hand, robot, target, v, limited, sent, pose_age, pose_mode, kinstate,
            jog_state, clearance=None, guard=False):
        if self.f is None:
            return
        f3 = lambda x: [""] * 3 if x is None else [f"{c:.5f}" for c in x]  # noqa: E731
        err = "" if target is None or robot is None else f"{1e3 * np.linalg.norm(np.asarray(target) - robot):.2f}"
        self.w.writerow([f"{time.monotonic() - self.t0:.4f}", "cri-jog", side, int(bool(tracked)), int(bool(engaged)),
                         f"{grip:.3f}", *f3(hand), *f3(robot), *f3(target), err, *[f"{c:.5f}" for c in v],
                         f"{np.linalg.norm(v):.5f}", "+".join(limited), int(bool(sent)),
                         "" if pose_age is None else f"{1000 * pose_age:.0f}", pose_mode, kinstate, jog_state,
                         "" if clearance is None else f"{1000 * clearance:.1f}", int(bool(guard)),
                         " | ".join(self.notes)])
        self.notes = []
        if time.monotonic() - self.last_flush > 1.0:
            self.f.flush()
            self.last_flush = time.monotonic()

    def close(self):
        if self.f is not None:
            self.f.close()
            self.f = None


class Log:
    """Node logger that also copies every message into the trace (one rclpy call site per severity)."""

    def __init__(self, node, trace):
        self.node, self.trace = node, trace

    def info(self, msg):
        self.trace.note(msg)
        self.node.get_logger().info(msg)

    def warning(self, msg):
        self.trace.note(msg)
        self.node.get_logger().warning(msg)

    def error(self, msg):
        self.trace.note(msg)
        self.node.get_logger().error(msg)


# ----------------------------------------------------------------------------- ROS node
class ArmLink:
    """Everything about one arm's driver: pose / hardware_state subscriptions, jog publisher, enable client.
    watch_only: just the (read-only) pose of an arm this session does not drive."""

    def __init__(self, node, side, args, watch_only=False):
        from igus_rebel_msgs.msg import HardwareState, RobotCartPose
        self.node, self.side, self.watch_only = node, side, watch_only
        sub = lambda s: s.format(side=side)  # noqa: E731
        self.pose_topic = sub(args.cri_watch_pose_topic if watch_only else args.cri_pose_topic)
        self.pose_scale = 1e-3 if args.cri_pose_units == "mm" else 1.0
        self.lock = threading.Lock()
        self.pose = self.hw = None
        self.pose_rx = self.hw_rx = 0.0
        self.mode_requested = False   # we asked for the mode and the driver did not refuse (a lost answer counts)
        self.sent_any = False         # we published at least one twist
        node.create_subscription(RobotCartPose, self.pose_topic, self._on_pose,
                                 QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability,
                                            durability=qos_profile_sensor_data.durability))
        if watch_only:
            return
        self.cmd_topic, self.enable_service, self.state_topic = (
            sub(args.cri_cmd_topic), sub(args.cri_enable_service), sub(args.cri_state_topic))
        # the driver is reliable on hardware_state and best-effort on cart_jog_cmd (depth 2, like the gamepad node)
        self.cmd_pub = node.create_publisher(TwistStamped, self.cmd_topic, 2)
        node.create_subscription(HardwareState, self.state_topic, self._on_state,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                            history=HistoryPolicy.KEEP_LAST))
        self.enable_client = node.create_client(SetBool, self.enable_service)

    def _on_pose(self, msg):
        with self.lock:
            self.pose, self.pose_rx = msg, time.monotonic()

    def _on_state(self, msg):
        with self.lock:
            self.hw, self.hw_rx = msg, time.monotonic()

    def snapshot(self):
        """Latest driver data: {cur, pose_age, pose_mode, kinstate, joints, jog_state, hw_active, hw_age, ...}."""
        with self.lock:
            pose, hw, pose_rx, hw_rx = self.pose, self.hw, self.pose_rx, self.hw_rx
        now, out = time.monotonic(), {"cur": None, "pose_age": None, "pose_mode": "", "kinstate": "", "joints": None,
                                      "jog_state": "", "hw_active": False, "hw_age": None, "latched": False,
                                      "connected": True}
        if pose is not None:
            raw = np.asarray(pose.pos_cart_robot_raw, float)
            out["cur"] = raw[:3] * self.pose_scale if np.all(np.isfinite(raw[:3])) else None
            age = now - pose_rx
            st = pose.header.stamp
            if st.sec or st.nanosec:  # acquisition time in the driver; a delayed sample must not look fresh
                age = max(age, (self.node.get_clock().now().nanoseconds - (st.sec * 10 ** 9 + st.nanosec)) * 1e-9)
            out.update(pose_age=age, pose_mode=pose.mode, kinstate=int(pose.kinstate),
                       joints=np.asarray(pose.joint_position_rad, float))
        if hw is not None:
            out.update(jog_state=hw.cart_jog_state, hw_age=now - hw_rx, latched=bool(hw.latched),
                       connected=bool(hw.connected),
                       hw_active=bool(hw.cart_jog_active) and hw.cart_jog_state == "active")
        return out

    def foreign_publishers(self):
        """Number of OTHER nodes publishing on the jog command topic (two sources would fight over the jog)."""
        return max(0, self.node.count_publishers(self.cmd_topic) - 1)

    def publish_twist(self, v):
        self.sent_any = True
        msg = TwistStamped()  # frame_id empty = the driver's base frame
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(x) for x in v)
        self.cmd_pub.publish(msg)

    def call_enable(self, on, timeout=8.0):
        """(ok, message). Blocks (the executor thread keeps spinning); the driver can take ~3 s."""
        if on:
            self.mode_requested = True
        try:
            if not self.enable_client.wait_for_service(timeout_sec=min(timeout, 5.0)):
                return False, f"service {self.enable_service} is not available (is the driver running, same ROS_DOMAIN_ID?)"
            fut = self.enable_client.call_async(SetBool.Request(data=bool(on)))
            end = time.monotonic() + timeout
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.01)
            if not fut.done():
                return False, f"no answer from {self.enable_service} within {timeout:.0f} s"
            r = fut.result()
            if on and not r.success:
                self.mode_requested = False   # explicitly refused: nothing to leave
            return bool(r.success), str(r.message)
        except Exception as exc:  # noqa: BLE001 - reported to the caller
            return False, f"{type(exc).__name__}: {exc}"


class CriJogNode(Node):
    def __init__(self, args):
        try:
            import igus_rebel_msgs.msg  # noqa: F401
        except ImportError as exc:
            raise CriJogError(f"cannot import igus_rebel_msgs ({exc}). Source the igus_rebel_ros2 install that has "
                              "RobotCartPose / HardwareState (quest_teleop/run_quest_cri_jog.sh does: IGUS_REBEL_INSTALL)"
                              ) from exc
        super().__init__("quest_cri_jog_teleop")
        self.args = args
        self.sides = list(args.arms)
        self.links = {s: ArmLink(self, s, args) for s in self.sides}
        other = next((s for s in SIDES if s not in self.sides), None)
        # --watch-other with one arm: read-only pose of the other arm for the guard
        self.watch = ArmLink(self, other, args, watch_only=True) if (
            other and getattr(args, "watch_other", False) and not args.no_arm_guard) else None
        self.event_pub = self.create_publisher(String, "/quest_teleop/events", 10)
        self.viz_js_pub = self.create_publisher(JointState, "/quest_teleop/joint_states", 10)
        self.hand_pub = {s: self.create_publisher(PoseStamped, f"/quest_teleop/{s}/hand", 10) for s in self.sides}
        self.state_pub = self.create_publisher(String, "/quest_teleop/state", 10)
        self.jog_executor = SingleThreadedExecutor()
        self.jog_executor.add_node(self)
        self.jog_thread = threading.Thread(target=self.jog_executor.spin, daemon=True)
        self.jog_thread.start()

    def close(self):
        try:
            self.jog_executor.shutdown()
            self.jog_thread.join(timeout=2.0)
        finally:
            self.destroy_node()


def make_quest(hands, env_file):
    from quest_arm_teleop import QuestController  # scipy + isaacteleop (the Quest venv); not needed before this point
    return QuestController(hands, env_file)


# ----------------------------------------------------------------------------- session
def health(link, args, snap, foreign):
    """-> (fault reason or None, only_mode_off). Anything unexpected stops the jog (of BOTH arms, see run_session)."""
    if foreign and not args.dry_run:
        return f"{foreign} other node(s) publish on {link.cmd_topic} (a second teleop / gamepad / test?)", False
    if snap["cur"] is None or snap["pose_age"] is None:
        return f"no robot pose on {link.pose_topic}", False
    if snap["pose_age"] > args.cri_pose_timeout:
        return f"robot pose is stale (older than {args.cri_pose_timeout} s)", False
    if snap["kinstate"] != 0:
        return f"controller KINSTATE {snap['kinstate']}", False
    if args.dry_run:
        return None, False
    if snap["hw_age"] is None or snap["hw_age"] > args.cri_state_timeout:
        return "hardware_state is stale (driver gone?)", False
    if snap["latched"] or not snap["connected"]:
        return "the driver is latched / disconnected", False
    if not snap["hw_active"]:
        return f"the driver is not in Cartesian jog mode (state '{snap['jog_state']}')", True
    if snap["pose_mode"] != "cartbase":
        return f"the controller reports mode '{snap['pose_mode']}', not cartbase", False
    return None, False


def wait_ready(node, args, log, stop):
    end = time.monotonic() + args.cri_wait_s
    links = list(node.links.values()) + ([node.watch] if node.watch else [])
    while rclpy.ok() and not stop["requested"]:
        missing = []
        for link in links:
            snap = link.snapshot()
            ok = snap["cur"] is not None and snap["pose_age"] <= args.cri_pose_timeout
            if not link.watch_only and not args.dry_run:
                ok = ok and snap["hw_age"] is not None
            if not ok:
                missing.append(link.pose_topic)
        if not missing:
            return
        if time.monotonic() > end:
            raise CriJogError(
                f"no data from the driver within {args.cri_wait_s:.0f} s (missing: {', '.join(missing)}). Is the "
                f"igus_rebel driver running (hardware_protocol:=rebel) on ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}?")
        log.info(f"waiting for {', '.join(missing)} and the hardware_state of the arms")
        time.sleep(0.5)
    raise KeyboardInterrupt


def run(args, env_file):
    """Entry point for `--backend cri-jog` (rclpy is already initialised)."""
    node = CriJogNode(args)
    try:
        run_session(node, args, env_file)
    finally:
        node.close()


def enter_modes(node, args, log):
    """cart_jog_enable true on every driven arm; collects every refusal (arms that did enter are left again by the
    caller's shutdown)."""
    if args.dry_run:
        log.info("dry run: cart_jog_enable is not called")
        return
    if args.cri_no_auto_enable:
        for s, link in node.links.items():
            log.info("--cri-no-auto-enable: enter the mode yourself (ros2 service call "
                     f"{link.enable_service} std_srvs/srv/SetBool \"{{data: true}}\"); the {s} arm moves only while "
                     "the driver reports it active")
        return
    refused = []
    for s, link in node.links.items():
        ok, msg = link.call_enable(True)
        if ok:
            log.warning(f"Cartesian jog mode ON ({s}): {msg}")
        else:
            ctl = f"arm_{s}_trajectory_controller" if "igus_rebel_" in link.enable_service else "rebel_arm_trajectory_controller"
            hint = (f"  -> deactivate the trajectory controller first (ros2 control switch_controllers --deactivate {ctl})"
                    if "controller" in msg.lower() else "")
            refused.append(f"{s} arm ({link.enable_service}): {msg}{hint}")
    if refused:
        raise CriJogError("cart_jog_enable true was refused or failed - " + " ;; ".join(refused))


def run_session(node, args, env_file):
    trace = CriTrace(getattr(args, "trace", None), args)
    log = Log(node, trace)
    stop = {"requested": False}
    old = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}

    def request_stop(signum, frame):
        stop["requested"] = True
    for sig in old:  # start-up (CloudXR, the enable call) is interruptible; the loop only flags a stop
        signal.signal(sig, signal.default_int_handler)
    xr = None
    try:
        try:
            names = " + ".join(node.sides)
            log.warning(f"CRI JOG ({names} arm{'s' if len(node.sides) > 1 else ''})"
                        f"{' DRY RUN - nothing is published' if args.dry_run else ''}: the controller does the IK; "
                        f"translation only, max {args.cri_max_lin_vel} m/s. Keep the e-stop in reach.")
            guard = build_guard(args, node.sides)
            if guard is None:
                log.error("ARM GUARD OFF (--no-arm-guard): NOTHING keeps the two arms apart. Watch the arms yourself.")
            else:
                log.info(f"arm guard on: stop an approaching arm at clearance <= {args.min_arm_dist * 100:.0f} cm + "
                         f"{guard.margin * 100:.1f} cm margin (reactive; the other arm "
                         + (f"is read from {node.watch.pose_topic}" if node.watch else
                            f"is assumed at {list(args.other_arm_deg)} deg" if len(node.sides) == 1 else "is driven too")
                         + "; stand, table and cameras are not modelled)")
            wait_ready(node, args, log, stop)
            foreign = {s: link.foreign_publishers() for s, link in node.links.items()}
            if any(foreign.values()):
                msg = (", ".join(f"{n} other node(s) already publish on {node.links[s].cmd_topic}"
                                 for s, n in foreign.items() if n)
                       + ": a gamepad / second teleop / test on this ROS domain. Refusing to start - stop it first")
                if not args.dry_run:
                    raise CriJogError(msg)
                log.warning(msg + " (dry run: nothing is published anyway)")
            hand_of = {s: ({"left": "right", "right": "left"}[s] if args.swap_hands else s) for s in node.sides}
            xr = make_quest(sorted(set(hand_of.values())), env_file)
            log.info("CloudXR up. Quest Browser -> https://nvidia.github.io/IsaacTeleop/client/v1.3.131/ , Server IP = "
                     "this PC, Connect. Hold SQUEEZE to move an arm: "
                     + ", ".join(f"{hand_of[s]} hand -> {s} arm" for s in node.sides)
                     + " (translation only; the controller's own jog).")
            log.info("rotation frame toggle (X) has no effect in cri-jog: rotation is not implemented")
            enter_modes(node, args, log)
            logics = {s: JogLogic(args, log, dry_run=args.dry_run, side=s if len(node.sides) > 1 else None)
                      for s in node.sides}
            for sig in old:  # from here Ctrl+C / SIGTERM end the loop at a tick boundary, never mid-publish
                signal.signal(sig, request_stop)
            main_loop(node, args, xr, logics, guard, trace, log, stop, hand_of)
        except KeyboardInterrupt:
            pass
        except CriJogError as exc:
            log.error(str(exc))
            raise
    finally:
        for sig in old:  # a second Ctrl+C / SIGTERM may now abort the (slow) leave-the-mode call
            signal.signal(sig, signal.default_int_handler)
        try:
            shutdown_jog(node, args, log)
        finally:
            for sig, handler in old.items():
                signal.signal(sig, handler)
            try:
                if xr is not None:
                    xr.close()
            finally:
                trace.close()


def shutdown_jog(node, args, log):
    """Zero twist to every arm first (a few: the topic is lossy), then leave the Cartesian jog mode on each arm."""
    if args.dry_run:
        return
    try:
        driven = [l for l in node.links.values() if l.sent_any]  # never put a twist on a topic of a session that did not drive
        for _ in range(3):
            for link in driven:
                link.publish_twist(np.zeros(3))
            time.sleep(0.02)
        # Leave the mode only where this session entered it (or, with --cri-no-auto-enable, drove in it): never switch
        # off a mode somebody else (a gamepad node) is using when we refused to start.
        for s, link in node.links.items():
            if link.mode_requested or (args.cri_no_auto_enable and link.sent_any):
                ok, msg = link.call_enable(False)
                if ok:
                    log.warning(f"Cartesian jog mode OFF ({s}): {msg}")
                else:
                    log.error(f"cart_jog_enable false failed on the {s} arm: {msg} (the driver leaves the mode by itself "
                              "after cart_jog_idle_exit_s without commands)")
    except KeyboardInterrupt:
        log.error("interrupted while leaving the Cartesian jog mode (the driver zeroes the jog after 0.15 s and leaves "
                  "the mode after cart_jog_idle_exit_s)")


def publish_viz(node, snaps, joints_all, readings, hand_of, logics):
    stamp = node.get_clock().now().to_msg()
    names, pos = [], []
    for s in SIDES:
        q = joints_all.get(s)
        if q is not None:
            names += [f"arm_{s}_{j}" for j in ARM_JOINTS]
            pos += [float(x) for x in q]
    if names:
        msg = JointState(name=names, position=pos)
        msg.header.stamp = stamp
        node.viz_js_pub.publish(msg)
    state = {}
    for s in node.sides:
        tracked, p, quat, _, _ = readings[hand_of[s]]
        state[s] = "engaged" if logics[s].engaged else ("tracked" if tracked else "lost")
        if tracked:
            ps = PoseStamped()
            ps.header.frame_id, ps.header.stamp = "quest", stamp
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
            (ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
             ps.pose.orientation.w) = map(float, quat)
            node.hand_pub[s].publish(ps)
    node.state_pub.publish(String(data=json.dumps(state)))


def main_loop(node, args, xr, logics, guard, trace, log, stop, hand_of):
    dt = 1.0 / args.rate
    sides = list(node.sides)
    zero_ticks = {s: 0 for s in sides}
    foreign = {s: 0 for s in sides}
    last_enable_try = {s: 0.0 for s in sides}
    prev_squeeze = {s: 1.0 for s in sides}
    latched_seen = {s: False for s in sides}
    last_foreign = last_log = 0.0
    last_buttons = {}
    # Y must be held --y-hold-s to discard the episode (an accidental thumb press must not); here it moves nothing
    y_hold = YHold(args.y_hold_s, log, "discard the open episode", clutch=args.clutch)
    while rclpy.ok() and not stop["requested"]:
        t0 = time.perf_counter()
        readings = xr.read_all()
        now = time.monotonic()
        if now - last_foreign > 0.1:
            last_foreign = now
            foreign = {s: node.links[s].foreign_publishers() for s in sides}
        snaps = {s: node.links[s].snapshot() for s in sides}
        # -- faults: ONE arm's fault stops BOTH arms (like the IK path); a driver leaving the mode counts too
        faults = {s: health(node.links[s], args, snaps[s], foreign[s]) for s in sides}
        fault_all = next((f"{s} arm: {f}" for s, (f, _) in faults.items() if f), None)
        # -- guard input: measured joints of every arm that is not assumed
        joints_all = {s: snaps[s]["joints"] for s in sides}
        if node.watch is not None:
            ws = node.watch.snapshot()
            fresh = ws["pose_age"] is not None and ws["pose_age"] <= args.cri_pose_timeout
            joints_all[node.watch.side] = ws["joints"] if fresh else None
        blind = guard.measure(now, joints_all) if guard else None
        if blind and not fault_all:
            fault_all = f"arm guard is blind: {blind}"
        if guard:
            for s, q in guard.assumed.items():
                joints_all[s] = q
        was_engaged = {s: logics[s].engaged for s in sides}
        v, hand_p = {}, {}
        for s in sides:
            tracked, pos, quat, squeeze, _ = readings[hand_of[s]]
            hand_p[s] = pos if tracked else None
            v[s] = logics[s].step(now, tracked, hand_p[s], squeeze, snaps[s]["cur"], fault_all)
        # an arm that moves against its command: that arm is latched, the other one lets go (re-grip allowed)
        newly = [s for s in sides if logics[s].latch and not latched_seen[s]]
        for s in newly:
            latched_seen[s] = True
            for o in sides:
                if o != s:
                    logics[o].drop(f"the {s} arm moves against its command")
                    v[o] = np.zeros(3)
        # -- arm-to-arm guard on the velocities about to be sent
        blocked = guard.limit(v) if (guard and not fault_all) else {}
        for s, why in blocked.items():
            log.warning(f"[cri {s}] arm guard: {why} - zero twist; release and grip, then move away")
            logics[s].drop(f"arm guard: {why}")
            v[s] = np.zeros(3)
        for s in sides:
            if was_engaged[s] and not logics[s].engaged:
                zero_ticks[s] = ZERO_TICKS
        # -- the driver left the mode on its own: a new grip enters it again, then another grip is needed to move
        for s in sides:
            squeeze = readings[hand_of[s]][3]
            tracked = readings[hand_of[s]][0]
            grip_edge = squeeze >= logics[s].on_th and prev_squeeze[s] < logics[s].on_th
            prev_squeeze[s] = squeeze
            if (faults[s][1] and grip_edge and not args.dry_run and not args.cri_no_auto_enable and tracked
                    and not foreign[s] and now - last_enable_try[s] > 3.0):
                last_enable_try[s] = now
                node.links[s].publish_twist(np.zeros(3))
                ok, msg = node.links[s].call_enable(True)
                if ok:
                    log.warning(f"[cri {s}] Cartesian jog mode re-enabled: {msg} - release and grip again")
                else:
                    log.error(f"[cri {s}] re-enabling the Cartesian jog mode FAILED: {msg}")
        sent = {s: False for s in sides}
        if not args.dry_run:
            for s in sides:
                link = node.links[s]
                # Heartbeat of zeros while the mode is on keeps it alive (the driver leaves it after 10 s without
                # commands); with another publisher on the topic only the final zeros go out.
                if logics[s].engaged or (snaps[s]["hw_active"] and not foreign[s]) or zero_ticks[s] > 0:
                    link.publish_twist(v[s] if logics[s].engaged else np.zeros(3))
                    sent[s] = True
                if zero_ticks[s] > 0 and not logics[s].engaged:
                    zero_ticks[s] -= 1
        clearance = guard.d if guard else None
        for s in sides:
            tr, pos, _, squeeze, _ = readings[hand_of[s]]
            trace.row(s, tr, logics[s].engaged, squeeze, hand_p[s], snaps[s]["cur"],
                      logics[s].target if logics[s].engaged else None, v[s], logics[s].limited, sent[s],
                      snaps[s]["pose_age"], snaps[s]["pose_mode"], snaps[s]["kinstate"], snaps[s]["jog_state"],
                      clearance, s in blocked)
        publish_viz(node, snaps, joints_all, readings, hand_of, logics)
        # A / B (right hand) = record start-stop / discard, Y (left, held --y-hold-s) = discard + reset event (no motion
        # here), X = nothing
        for h, (prim, sec) in xr.buttons.items():
            was_p, was_s = last_buttons.get(h, (False, False))
            for pressed, was, ev in ((prim, was_p, "frame" if h == "left" else "toggle"),
                                     (sec, was_s, "reset" if h == "left" else "discard")):
                if pressed and not was and ev != "reset":
                    log.info(f"[{h} hand] button -> {ev}")
                    if ev == "frame":
                        log.info("rotation frame: no effect in cri-jog (rotation is not implemented)")
                    else:
                        node.event_pub.publish(String(data=ev))
            last_buttons[h] = (prim, sec)
        if y_hold.update(xr.buttons, readings, time.monotonic()):
            log.info("[left hand] button -> reset")
            node.event_pub.publish(String(data="reset"))
            for s in sides:   # no motion: the scene reset of the other backends moves the arms
                if logics[s].engaged and not args.dry_run:
                    node.links[s].publish_twist(np.zeros(3))
                    zero_ticks[s] = ZERO_TICKS
                logics[s].drop("reset button")
        if not args.quiet and t0 - last_log > 0.5:
            for s in sides:
                if logics[s].engaged and snaps[s]["cur"] is not None:
                    last_log = t0
                    log.info("[cri %s] hand[cm] %s | robot dEE[cm] %s | err %.1f mm | v[mm/s] %s%s" % (
                        s, np.array2string(100 * (logics[s].hand_in_base(hand_p[s]) - logics[s].origin_p), precision=1,
                                           floatmode="fixed"),
                        np.array2string(100 * (snaps[s]["cur"] - logics[s].robot_snap), precision=1, floatmode="fixed"),
                        1e3 * np.linalg.norm(logics[s].target - snaps[s]["cur"]),
                        np.array2string(1000 * v[s], precision=1, floatmode="fixed"),
                        "" if clearance is None else " | arm clearance %.1f cm" % (100 * clearance)))
        time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
