#!/usr/bin/env python3
"""Real-arm tracking test of ONE joint inside an operator-approved range, every other joint held where it is.

Operator-approved ranges (2026-10-21, from the all-zero pose): left arm J2 0..20 deg, J4 0..+20 deg; right arm
J2 0..20 deg, J4 -21..0 deg (J3/J5 on the home<->zero
line: real_robot/j3j5_step_test.py). The joint is commanded only inside the range shrunk by --margin (default 4 deg) so
an overshoot stays inside it (default 6: the model arm overshot 3.5 deg with 4); the test aborts (holds the measured pose) if that joint leaves the range shrunk by 0.5 deg
(1 deg past the range while slowly leaving its end at 10 deg/s), if any other joint drifts > 2 deg from where it
started, if joint states go stale or another node publishes commands. Streams like the Quest teleop: one single-point
JointTrajectory per 30 Hz tick, lookahead 0.15 s, acceleration-limited command + leash, release brake.

  .venv_quest/bin/python real_robot/joint_step_test.py --arm left --joint 2 --range 0 20 [--tag standard]
Profiles: move  (30 deg/s, a 0.52, leash 10: lo -> hi -> lo of the shrunk range)
          stop  (same, hi -> lo, the command stops half way like a release: with the brake, then without)
Writes latest_run/joint<N>_<tag>.csv
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
RATE, LOOKAHEAD = 30.0, 0.15
V, A, LEAD = 30.0, 0.52, 10.0
SEND_VEL = False
DEC = None


class Abort(Exception):
    pass


class Tester(Node):
    def __init__(self, arm, j, rng, margin, out):
        super().__init__("joint_step_test")
        self.j = j - 1
        self.rng = (min(rng), max(rng))
        self.cmd_rng = (self.rng[0] + margin, self.rng[1] - margin)
        self.fast_abort = (self.rng[0] + 0.5, self.rng[1] - 0.5)
        self.slow_abort = (self.rng[0] - 1.0, self.rng[1] + 1.0)
        self.topic = f"/{arm}/rebel_arm_trajectory_controller/joint_trajectory"
        self.pub = self.create_publisher(JointTrajectory, self.topic, 10)
        self.q, self.t_q, self.v, self.hold = None, 0.0, np.zeros(6), None
        self.create_subscription(JointState, f"/{arm}/joint_states", self.on_js, QoSProfile(depth=1))
        self.f = open(out, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "profile", "phase", *[f"cmd{i}" for i in range(1, 7)], *[f"meas{i}" for i in range(1, 7)]])
        self.t0 = time.monotonic()

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(n in d for n in JOINTS):
            q, now = np.array([d[n] for n in JOINTS]), time.monotonic()
            if self.q is not None and now - self.t_q > 0.004:
                vi = (q - self.q) / (now - self.t_q)
                self.v = 0.7 * self.v + 0.3 * vi if now - self.t_q < 0.1 else np.zeros(6)
            self.q, self.t_q = q, now

    def spin(self, dt):
        end = time.monotonic() + dt
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.005, end - time.monotonic())))

    def pose(self, val_deg, clamp):
        q = self.hold.copy()
        q[self.j] = np.deg2rad(float(np.clip(val_deg, *clamp)))
        return q

    def check(self, slow):
        if time.monotonic() - self.t_q > 0.3:
            raise Abort("joint states stale > 0.3 s")
        d = np.rad2deg(self.q)
        lo, hi = self.slow_abort if slow else self.fast_abort
        if not (lo <= d[self.j] <= hi):
            raise Abort(f"J{self.j + 1} {d[self.j]:.1f} left [{lo}, {hi}]")
        others = [i for i in range(6) if i != self.j]
        drift = np.abs(d[others] - np.rad2deg(self.hold[others]))
        if np.max(drift) > 2.0:
            raise Abort(f"other joints drifted {np.round(drift, 1).tolist()}")
        if self.count_publishers(self.topic) > 1:
            raise Abort(f"another node publishes on {self.topic}")

    def send(self, q, la=LOOKAHEAD, vel=None):
        pt = JointTrajectoryPoint(positions=[float(x) for x in q],
                                  time_from_start=Duration(sec=int(la), nanosec=int((la % 1) * 1e9)))
        if vel is not None:
            pt.velocities = [float(x) for x in vel]
        self.pub.publish(JointTrajectory(joint_names=JOINTS, points=[pt]))

    def log(self, profile, phase, cmd):
        self.w.writerow([f"{time.monotonic() - self.t0:.4f}", profile, phase, *np.round(np.rad2deg(cmd), 3),
                         *np.round(np.rad2deg(self.q), 3)])

    def move(self, profile, to_deg, vmax=None, acc=None, lead=None, stop_at=None, brake=True, settle=2.5, slow=False):
        vmax, acc, lead = vmax or V, acc or A, lead or LEAD
        clamp = self.slow_abort if slow else self.cmd_rng
        clamp = (min(clamp[0], np.rad2deg(self.q[self.j])), max(clamp[1], np.rad2deg(self.q[self.j])))
        if slow:  # leaving / reaching an end of the range: never command beyond the approved range itself
            clamp = (max(clamp[0], self.rng[0]), min(clamp[1], self.rng[1]))
        target = self.pose(to_deg, clamp)
        q_cmd, v_cmd, dt, phase = self.pose(np.rad2deg(self.q[self.j]), clamp), np.zeros(6), 1 / RATE, "move"
        while True:
            self.check(slow)
            q_new, v_cmd = accel_step(q_cmd, v_cmd, target, np.deg2rad(vmax), acc, dt, adec=None if slow else DEC)
            q_new = self.pose(np.rad2deg(leash(q_new, self.q, np.deg2rad(lead))[self.j]), clamp)
            v_cmd = (q_new - q_cmd) / dt
            q_cmd = q_new
            self.send(q_cmd, vel=v_cmd if SEND_VEL and not slow else None)
            self.spin(dt)
            self.log(profile, phase, q_cmd)
            if stop_at is not None and np.sign(target[self.j] - q_cmd[self.j]) * (np.deg2rad(stop_at) - q_cmd[self.j]) <= 0:
                phase = "released"
                if brake:
                    v = self.v[self.j]
                    stop = self.q[self.j] + v * abs(v) / (2 * acc) + v * BRAKE_DELAY_S
                    q_cmd = self.pose(np.rad2deg(stop), self.rng)  # never beyond the approved range
                    t_stop = float(np.clip(abs(v) / acc + BRAKE_DELAY_S, 0.15, 2.5))
                    self.send(q_cmd, t_stop)
                    phase = "released+brake"
                break
            if abs(q_cmd[self.j] - target[self.j]) < 1e-4:
                break
        end = time.monotonic() + settle
        while time.monotonic() < end:
            self.check(slow)
            self.spin(1 / RATE)
            self.log(profile, phase + " settle", q_cmd)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arm", choices=["left", "right"], required=True)
    p.add_argument("--joint", type=int, choices=[2, 4], required=True, help="only the operator-approved joints")
    p.add_argument("--range", type=float, nargs=2, required=True, metavar=("A", "B"), help="approved range [deg]")
    p.add_argument("--margin", type=float, default=6.0, help="commanded range = approved range shrunk by this [deg]")
    p.add_argument("--tag", default="test")
    p.add_argument("--vel", type=float, help="command speed [deg/s] (default 30)")
    p.add_argument("--acc", type=float, help="command acceleration [rad/s^2] (default 0.52)")
    p.add_argument("--lead", type=float, help="leash [deg] (default 10)")
    p.add_argument("--send-vel", action="store_true", help="put the command velocity into each streamed point")
    p.add_argument("--dec", type=float, help="braking deceleration of the command [rad/s^2] (default = --acc)")
    p.add_argument("--no-stop", action="store_true", help="only the up/down moves, no half-way stop")
    args = p.parse_args()
    global V, A, LEAD, SEND_VEL, DEC
    DEC = args.dec
    V, A, LEAD, SEND_VEL = args.vel or V, float(np.clip(args.acc or A, 0.1, 2.0)), args.lead or LEAD, args.send_vel
    # operator-approved ranges (2026-10-21, from the all-zero pose): left J2 0..20, J4 0..+20 (positive direction);
    # right (tested earlier that day) J2 0..20, J4 -21..0
    approved = {"left": {2: (0.0, 20.0), 4: (0.0, 20.0)}, "right": {2: (0.0, 20.0), 4: (-21.0, 0.0)}}[args.arm][args.joint]
    if min(args.range) < approved[0] or max(args.range) > approved[1]:
        print(f"[test] refusing: J{args.joint} range {args.range} is outside the approved {approved}", flush=True)
        return 2
    out = HERE.parent / "latest_run" / f"joint{args.joint}_{args.tag}.csv"
    out.parent.mkdir(exist_ok=True)
    rclpy.init()
    node = Tester(args.arm, args.joint, args.range, args.margin, out)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        end = time.monotonic() + 10
        while node.q is None and time.monotonic() < end:
            node.spin(0.1)
        if node.q is None:
            raise Abort(f"no /{args.arm}/joint_states")
        node.spin(0.5)
        d = np.rad2deg(node.q)
        print(f"[test] {args.arm} start joints [deg] {np.round(d, 1).tolist()}", flush=True)
        if np.max(np.abs(d)) > 1.5:
            raise Abort("start pose is not all zero (bring the arm to zero first) - refusing")
        if node.count_publishers(node.topic) > 1:
            raise Abort(f"another node publishes on {node.topic} - refusing")
        node.hold = node.q.copy()
        lo, hi = node.cmd_rng
        near, far = (lo, hi) if abs(lo) < abs(hi) else (hi, lo)  # the end of the range next to zero first
        print(f"[test] J{args.joint}: slow to {near:.0f}, then {near:.0f}<->{far:.0f} at {V:.0f} deg/s, stop test", flush=True)
        node.move("approach", near, vmax=10, acc=0.3, lead=6, settle=1.0, slow=True)
        node.move("move", far)
        node.move("move", near)
        mid = (lo + hi) / 2
        # the measured-velocity release brake was dropped: joint_states velocity spikes put its stopping point at the
        # range limit (twice on 2026-10-21); the teleop now glides the command instead
        for brake in (() if args.no_stop else (False,)):
            name = f"stop_{'brake' if brake else 'nobrake'}"
            node.move(name, far)
            node.move(name, near, stop_at=mid, brake=brake)
        print("[test] back to zero: 5 deg/s to 1 deg inside the range, then the last degree in 2 s", flush=True)
        inside = 1.0 if near > 0 else -1.0
        node.move("zero", inside, vmax=5, acc=0.2, lead=4, settle=1.0, slow=True)
        node.send(node.hold.copy(), 2.0)
        end = time.monotonic() + 2.5
        while time.monotonic() < end:
            node.check(True)
            node.spin(1 / RATE)
            node.log("zero", "final", node.hold)
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
