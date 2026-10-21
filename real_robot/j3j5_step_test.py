#!/usr/bin/env python3
"""Real-arm tracking test on the home <-> zero line ONLY (operator-approved range, 2026-10-21): joint3 and joint5 move
together along the straight joint-space line between home (0, 0, 65, 0, 50, 0) and zero (all 0); joint1/2/4/6 are
held at 0. Commands stay within J3 = [10, 55] deg so that an overshoot still ends inside home <-> zero.

Each move is streamed exactly like the Quest teleop streams it (one single-point JointTrajectory per 30 Hz tick to the
igus driver's joint_trajectory_controller, lookahead 0.15 s, acceleration-limited command + leash), so the measured
lag / overshoot / settle are what the teleop gets with those limits. Profiles (one arm at a time):
  normal   v 30 deg/s, a 0.52 rad/s^2, leash 10, J3 10 <-> 55   (current defaults)
  fast     v 45 deg/s, a 0.8,  leash 14, J3 20 <-> 45
  faster   v 45 deg/s, a 1.2,  leash 14, J3 25 <-> 40
  stop     normal profile 55 -> 10, the command stops at J3 32 (like letting go): with the release brake, then without
Safety: refuses to start if the arm is not on the line (J1/2/4/6 within 1.5 deg of 0, J5 within 3 deg of 50/65*J3) or
if another node publishes on the arm's command topic. Aborts (holds the measured pose) if any joint leaves
J3 [3, 62] during a test move ([-2, 67] during the 10 deg/s approach / return), J5 the same on the line, or
J1/2/4/6 drift > 2 deg, or joint states go stale > 0.3 s. Ctrl+C holds the arm.

Needs only the igus driver of that arm (real_robot/dual_arm_driver.launch.py arms:=<side>), no teleop / MoveIt.
  .venv_quest/bin/python real_robot/j3j5_step_test.py --arm left [--profiles normal fast faster stop]
Writes latest_run/j3j5_<arm>.csv (t, profile, phase, cmd, meas per tick).
"""
import argparse
import csv
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "quest_teleop"))
from quest_dual_arm_teleop import BRAKE_DELAY_S, accel_step, leash  # noqa: E402

JOINTS = [f"joint{i}" for i in range(1, 7)]
HOME = np.deg2rad([0, 0, 65, 0, 50, 0])
RATIO = 50.0 / 65.0
LO, HI = 10.0, 55.0                       # commanded J3 range [deg]
# abort well inside home <-> zero: after an abort the arm still coasts (~10 deg at 30 deg/s on a 0.52 rad/s^2 arm)
J3_ABORT = (3.0, 62.0)          # fast test moves
J3_ABORT_SLOW = (-2.0, 67.0)     # the 10 deg/s approach / return to home, which start or end at 0 / 65
# (v deg/s, a rad/s^2, leash deg, J3 range): faster profiles move over a shorter range because they overshoot more
# (dry run on the model arm: fast over 10-55 ran 13 deg past 55)
PROFILES = {"normal": (30, 0.52, 10, (10.0, 55.0)), "fast": (45, 0.8, 14, (20.0, 45.0)),
            "faster": (45, 1.2, 14, (25.0, 40.0)),
            "tune": (30, 0.52, 10, (20.0, 45.0))}  # gain / lookahead tuning: teleop defaults, short range (margin)
RATE, LOOKAHEAD = 30.0, 0.15
SEND_VEL = False
DEC = None  # braking deceleration of the command [rad/s^2] (default = the acceleration)
GLIDE = False
GLIDE_DEC = None
GLIDE_ONLY = True
RELEASE_MODES = None  # the measured-velocity brake was dropped (velocity spikes sent it to the range limit twice)
DEC_RELEASE = [None]


def line(j3_deg, lo=LO, hi=HI):
    """Joint vector on the home <-> zero line for a J3 angle [deg], clamped to [lo, hi] (test moves: [LO, HI];
    the slow approach / return: the whole line [0, 65])."""
    j3 = float(np.clip(j3_deg, lo, hi))
    return np.deg2rad([0, 0, j3, 0, j3 * RATIO, 0])


class Abort(Exception):
    pass


class Tester(Node):
    def __init__(self, arm, out):
        super().__init__("j3j5_step_test")
        self.arm = arm
        self.topic = f"/{arm}/rebel_arm_trajectory_controller/joint_trajectory"
        self.pub = self.create_publisher(JointTrajectory, self.topic, 10)
        self.q = None
        self.t_q = 0.0
        self.v = np.zeros(6)
        self.create_subscription(JointState, f"/{arm}/joint_states", self.on_js, QoSProfile(depth=1))
        self.f = open(out, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "profile", "phase", *[f"cmd{i}" for i in range(1, 7)], *[f"meas{i}" for i in range(1, 7)]])
        self.t0 = time.monotonic()

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            q, now = np.array([d[j] for j in JOINTS]), time.monotonic()
            if self.q is not None and now - self.t_q > 0.004:
                vi = (q - self.q) / (now - self.t_q)
                self.v = 0.7 * self.v + 0.3 * vi if now - self.t_q < 0.1 else np.zeros(6)
            self.q, self.t_q = q, now

    def spin(self, dt):
        end = time.monotonic() + dt
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.005, end - time.monotonic())))

    def check(self, j3_range=J3_ABORT):
        if time.monotonic() - self.t_q > 0.3:
            raise Abort("joint states stale > 0.3 s")
        d = np.rad2deg(self.q)
        j5_range = (j3_range[0] * RATIO - 1.0, j3_range[1] * RATIO + 1.0)
        if not (j3_range[0] <= d[2] <= j3_range[1] and j5_range[0] <= d[4] <= j5_range[1]):
            raise Abort(f"J3/J5 left the home-zero range: J3 {d[2]:.1f}, J5 {d[4]:.1f}")
        if np.max(np.abs(d[[0, 1, 3, 5]])) > 2.0:
            raise Abort(f"J1/J2/J4/J6 drifted: {np.round(d[[0, 1, 3, 5]], 1).tolist()}")
        if self.count_publishers(self.topic) > 1:
            raise Abort(f"another node publishes on {self.topic}")

    def send(self, q, la=None, vel=None):
        la = LOOKAHEAD if la is None else la
        pt = JointTrajectoryPoint(positions=[float(x) for x in q],
                                  time_from_start=Duration(sec=int(la), nanosec=int((la % 1) * 1e9)))
        if vel is not None:  # velocity at the point -> JTC splines + uses it as feedforward
            pt.velocities = [float(x) for x in vel]
        self.pub.publish(JointTrajectory(joint_names=JOINTS, points=[pt]))

    def log(self, profile, phase, cmd):
        self.w.writerow([f"{time.monotonic() - self.t0:.4f}", profile, phase, *np.round(np.rad2deg(cmd), 3),
                         *np.round(np.rad2deg(self.q), 3)])

    def hold(self, profile="hold", s=1.0):
        q = self.q.copy()
        self.send(q, 0.3)
        end = time.monotonic() + s
        while time.monotonic() < end:
            self.spin(1 / RATE)
            self.log(profile, "hold", q)

    def move(self, profile, j3_to, vmax_deg, acc, lead_deg, stop_at=None, brake=True, settle=2.5, lo=LO, hi=HI):
        """Stream an accel-limited, leashed command from the current pose to line(j3_to); stop_at = J3 where the
        command stops (like letting go), then optionally the release brake. Logs every tick, then settles."""
        target = line(j3_to, lo, hi)
        slow = vmax_deg <= 10.0
        limits = J3_ABORT_SLOW if slow else J3_ABORT
        v_cmd, dt = np.zeros(6), 1 / RATE
        # start exactly on the line at the measured J3 (tiny correction of J5 / the others; checked < 3 deg before)
        q_cmd = line(np.rad2deg(self.q[2]), 0.0, 65.0)
        phase = "move"
        while True:
            self.check(limits)
            a_lim = max(acc, DEC_RELEASE[0]) if phase == "released+glide" else acc
            q_new, v_cmd = accel_step(q_cmd, v_cmd, target, np.deg2rad(vmax_deg), a_lim, dt,
                                      adec=(DEC_RELEASE[0] if phase == "released+glide" else DEC)
                                      if (DEC and not vmax_deg <= 10.0) else None)
            q_new = leash(q_new, self.q, np.deg2rad(lead_deg))
            q_new = line(np.rad2deg(q_new[2]), min(lo, np.rad2deg(q_cmd[2])), max(hi, np.rad2deg(q_cmd[2])))
            # ^ never off the line; never beyond [lo, hi] except where an approach starts outside it
            v_cmd = (q_new - q_cmd) / dt
            q_cmd = q_new
            self.send(q_cmd, vel=v_cmd if SEND_VEL else None)
            self.spin(dt)
            self.log(profile, phase, q_cmd)
            if stop_at is not None and np.sign(target[2] - q_cmd[2]) * (np.deg2rad(stop_at) - q_cmd[2]) <= 0:
                phase = "released"
                if brake == "hold":  # stop at once: hold the measured pose (the controller brakes as hard as it can)
                    q_cmd = line(np.rad2deg(self.q[2]), 0.0, 65.0)
                    self.send(q_cmd)
                    phase = "released+hold"
                    break
                if brake == "glide":  # keep streaming: the command brakes at acc to a stop, like a move's end
                    dec = GLIDE_DEC or acc  # release braking (default firm: the gentler DEC glides ~26 deg from 30 deg/s)
                    DEC_RELEASE[0] = dec
                    target = line(np.rad2deg(q_cmd[2] + v_cmd[2] * abs(v_cmd[2]) / (2 * dec)), lo, hi)  # reachable
                    stop_at, phase = None, "released+glide"
                    continue
                if brake:  # same formula as the teleop's release brake, kept on the line
                    a = acc
                    v = self.v.copy()
                    stop = self.q + v * np.abs(v) / (2 * a) + v * BRAKE_DELAY_S
                    q_cmd = line(np.rad2deg(stop[2]), 0.0, 65.0)
                    t_stop = float(np.clip(np.max(np.abs(v)) / a + BRAKE_DELAY_S, 0.15, 2.5))
                    self.send(q_cmd, t_stop)
                    phase = "released+brake"
                break
            if np.max(np.abs(q_cmd - target)) < 1e-4:
                break
        end = time.monotonic() + settle
        while time.monotonic() < end:
            self.check(limits)
            self.spin(1 / RATE)
            self.log(profile, phase + " settle", q_cmd)


def main():
    global LOOKAHEAD, SEND_VEL, DEC, GLIDE, GLIDE_DEC, RELEASE_MODES
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arm", choices=["left", "right"], required=True)
    p.add_argument("--profiles", nargs="+", default=["normal", "fast", "faster", "stop"],
                   choices=list(PROFILES) + ["stop"])
    p.add_argument("--vel", type=float, help="tune profile: command speed [deg/s] (default 30)")
    p.add_argument("--acc", type=float, help="tune profile: command acceleration [rad/s^2] (default 0.52)")
    p.add_argument("--lead", type=float, help="tune profile: leash [deg] (default 10)")
    p.add_argument("--send-vel", action="store_true", help="put the command velocity into each streamed point")
    p.add_argument("--release", nargs="+", choices=["hold", "glide"], help="stop profile: release modes to test")
    p.add_argument("--glide-dec", type=float, help="braking of the release glide [rad/s^2] (default = --acc)")
    p.add_argument("--glide", action="store_true", help="stop profile: also a release that glides to a stop at --dec")
    p.add_argument("--dec", type=float, help="braking deceleration of the command [rad/s^2] (default = --acc)")
    p.add_argument("--lookahead", type=float, default=LOOKAHEAD,
                   help="time_from_start of each streamed point [s] (teleop --real-lookahead, default 0.15)")
    p.add_argument("--out", type=Path, help="CSV path (default latest_run/j3j5_<arm>.csv)")
    p.add_argument("--no-home", action="store_true", help="stay in the test range at the end (next run follows)")
    args = p.parse_args()
    LOOKAHEAD = float(np.clip(args.lookahead, 0.03, 0.3))
    SEND_VEL = args.send_vel
    DEC = float(np.clip(args.dec, 0.05, 2.0)) if args.dec else None
    GLIDE = args.glide
    GLIDE_DEC = args.glide_dec
    RELEASE_MODES = tuple(args.release) if args.release else None
    v0, a0, l0, r0 = PROFILES["tune"]
    PROFILES["tune"] = (args.vel or v0, float(np.clip(args.acc or a0, 0.1, 2.0)), args.lead or l0, r0)
    if args.vel:  # the stop profile runs at the "normal" profile's speed
        vn, an, ln, rn = PROFILES["normal"]
        PROFILES["normal"] = (args.vel, an, ln, rn)
    out = args.out or HERE.parent / "latest_run" / f"j3j5_{args.arm}.csv"
    out.parent.mkdir(exist_ok=True)
    rclpy.init()
    node = Tester(args.arm, out)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        end = time.monotonic() + 10
        while node.q is None and time.monotonic() < end:
            node.spin(0.1)
        if node.q is None:
            raise Abort(f"no /{args.arm}/joint_states (driver running? ROS_DOMAIN_ID)")
        node.spin(0.5)
        d = np.rad2deg(node.q)
        print(f"[test] {args.arm} start joints [deg] {np.round(d, 1).tolist()}", flush=True)
        if np.max(np.abs(d[[0, 1, 3, 5]])) > 1.5 or abs(d[4] - RATIO * d[2]) > 3.0 or not (-1 <= d[2] <= 66):
            raise Abort("arm is not on the home-zero line - refusing (bring it to home or zero first)")
        if node.count_publishers(node.topic) > 1:
            raise Abort(f"another node publishes on {node.topic} - refusing")
        if not (LO <= d[2] <= HI):  # slow approach into the test range along the line
            print(f"[test] approach: J3 {d[2]:.1f} -> {np.clip(d[2], LO, HI):.0f} at 10 deg/s", flush=True)
            node.move("approach", float(np.clip(d[2], LO, HI)), 10, 0.3, 6, settle=1.0, lo=0.0, hi=65.0)
        for prof in args.profiles:
            v, a, lead, (lo, hi) = PROFILES["normal" if prof == "stop" else prof]
            if prof == "stop":
                for brake in (RELEASE_MODES or ((("glide",) if GLIDE else ()) + ((True, False) if not GLIDE_ONLY else ()))):
                    name = f"stop_{brake if isinstance(brake, str) else 'brake' if brake else 'nobrake'}"
                    print(f"[test] {name}: 55 -> 10 stopping at J3 32", flush=True)
                    node.move(name, HI, v, a, lead)
                    node.move(name, LO, v, a, lead, stop_at=32.0, brake=brake)
                continue
            print(f"[test] {prof}: v {v} deg/s, a {a} rad/s^2, leash {lead}: J3 {lo}->{hi}->{lo}", flush=True)
            node.move(prof + " approach", lo, 10, 0.3, 6, settle=1.0, lo=LO, hi=HI)
            node.move(prof, hi, v, a, lead, lo=lo, hi=hi)
            node.move(prof, lo, v, a, lead, lo=lo, hi=hi)
        if not args.no_home:
            print("[test] back to (just short of) home at 10 deg/s", flush=True)
            node.move("home", 64.0, 10, 0.3, 6, settle=2.0, lo=0.0, hi=65.0)  # 64: a small overshoot stays inside home
        print(f"[test] done, joints [deg] {np.round(np.rad2deg(node.q), 1).tolist()}  log: {out}", flush=True)
    except (Abort, KeyboardInterrupt) as exc:
        print(f"[test] ABORT: {exc} - holding the measured pose", flush=True)
        if node.q is not None:
            node.send(node.q.copy(), 0.3)
            node.spin(0.5)
        return 1
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
