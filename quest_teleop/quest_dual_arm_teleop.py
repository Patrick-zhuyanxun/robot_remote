#!/usr/bin/env python3
"""Meta Quest (Isaac Teleop / CloudXR) -> BOTH igus ReBeL arms of the Isaac dual-arm rig.

  left  controller --clutch--> left  gripper TCP --IK--> arm_left_joint1..6   \\
  right controller --clutch--> right gripper TCP --IK--> arm_right_joint1..6  -> /dual_arm/isaac_joint_commands
  triggers -> arm_{left,right}_xeg32_*_carriage_joint                         /

Commands go straight to Isaac (no ros2_control / MoveIt): the rig's joints follow position targets
directly, which is much snappier than streaming through the joint_trajectory_controller.
Do NOT run dual_arm_sim/start_right_arm_moveit.sh at the same time - its ros2_control would also
write the right arm and yank it back.

Controls (per hand): hold SQUEEZE to move that arm, release to freeze it; TRIGGER closes its gripper.

  robot_remote/quest_teleop/run_quest_dual_arm.sh

--backend real connects both real ReBeLs directly over Ethernet (CRI + Position Interface).
--backend real-ros preserves the older, single-arm ROS trajectory backend.
Real backends use slower limits, start at the measured pose, and do not command grippers.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from quest_arm_teleop import ARM_JOINTS, HOME_DEG, TCP_PRESETS, QuestController, SerialChain, step_toward
from ethernet_rebel import DEFAULT_CONFIG, EthernetArms, RobotError, load_endpoints, watch_arm
from arm_collision import DEFAULT_CALIBRATION, capsules, clearance, load_world_T_base

HERE = Path(__file__).resolve().parent
# Per-tick trace of the newest run, overwritten every start (robot_remote/latest_run/, see Trace). The launcher also
# copies each process's console log there. ROBOT_REMOTE_TRACE="" turns it off, another path moves it.
DEFAULT_TRACE = HERE.parent / "latest_run" / "teleop_trace.csv"
GLIDE_MAX_DEG = 20.0  # longest release glide (largest joint); the real arm needs ~20 deg to stop from ~30 deg/s
# Release brake: how long the ReBeL keeps going at its current speed before braking takes hold. 0.25 s from the
# controller.yaml fit (delay 0.078 + velocity lag 0.16); the real left arm (J3/J5 test 2026-10-21, 32 deg/s) stopped
# 6.7 deg past that prediction -> +0.22 s.
BRAKE_DELAY_S = float(os.environ.get("ROBOT_REMOTE_BRAKE_DELAY", "0.45"))
SIDES = ("left", "right")

# Joint-state subscriptions keep only the newest message. The loop handles ONE ready callback per 30 Hz tick
# (rclpy.spin_once) while Isaac publishes every render frame (~60 Hz): with a depth-10 queue each take returned
# the oldest of ten queued states, so q_meas (engage snapshot, guard, lag check) ran ~0.15-0.3 s behind.
LATEST = QoSProfile(depth=1)
LATEST_SENSOR = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability,
                           durability=qos_profile_sensor_data.durability)

# Arm base poses in the rig/world frame, from dual_arm_rig_v2.usda (sliders and lift at 0).
ARM_BASE_XYZ = {"left": (0.228209, -0.179297, 1.1093), "right": (-0.231737, -0.179297, 1.1093)}
ARM_BASE_YAW_DEG = -90.0

# Real-arm defaults (--backend real / real-ros): slower than the sim, the operator stands next to the robot.
# 2026-10-21: 0.35 rad/s / 0.15 m/s / 30 deg/s raised to 0.52 / 0.25 / 45 together with the acceleration-limited
# command (max_joint_acc, below the ReBeL's ~0.52 rad/s^2) and a 10 deg leash; without the acceleration limit those
# speeds overshoot 9-20 deg (offline model of Igus_rebel controller.yaml's plant). The Ethernet backend keeps 0.35.
REAL_DEFAULTS = {"max_joint_vel": 0.52, "max_lin_vel": 0.25, "max_rot_vel": 45.0, "max_cmd_lead_deg": 10.0,
                 "max_joint_acc": 0.52, "max_joint_dec": 0.3, "glide_dec": 0.4}  # 0.52 = the ReBeL's identified limit; the launcher offers 0.45-0.8 to try
ETHERNET_DEFAULTS = {"max_joint_vel": 0.35, "max_lin_vel": 0.15, "max_rot_vel": 30.0, "max_cmd_lead_deg": 0.0,
                     "max_joint_acc": 0.0, "max_joint_dec": 0.0, "glide_dec": 0.0}
SIM_DEFAULTS = {"max_joint_vel": 0.785, "max_lin_vel": 0.50, "max_rot_vel": 90.0, "max_cmd_lead_deg": 0.0,
                "max_joint_acc": 0.0, "max_joint_dec": 0.0, "glide_dec": 0.0}

# OpenXR anchor frame is set by quest_arm_teleop.OPERATOR_T_ANCHOR: operator X fwd, Y left, Z up.


class LoopStats:
    """Timing report every --diag-period s: loop rate and overruns, time spent in the Quest read / IK + guard /
    publishing, age of the newest joint state, how often joint states arrive and, in the sim, how fast sim time
    runs against the wall clock. Tells a slow loop (this process) apart from a slow simulator (GPU / CPU)."""

    def __init__(self, rate, period):
        self.rate, self.period = rate, period
        self.reset(time.perf_counter())
        self.js_n, self.js_stamp = 0, None   # joint-state messages counted / sim stamp, written by the callbacks

    def reset(self, now):
        self.t_start = now
        self.ticks, self.overruns = 0, 0
        self.parts = {"tick": [], "xr": [], "ik": [], "pub": [], "solve": [], "guard": []}
        self.age = []
        self.js_n0, self.js_stamp0 = getattr(self, "js_n", 0), getattr(self, "js_stamp", None)

    def on_state(self, msg):
        self.js_n += 1
        st = msg.header.stamp
        if st.sec or st.nanosec:
            self.js_stamp = st.sec + st.nanosec * 1e-9

    def tick(self, parts, age):
        self.ticks += 1
        self.overruns += parts["tick"] > 1.0 / self.rate
        for k, v in parts.items():
            self.parts[k].append(v * 1e3)
        if age is not None:
            self.age.append(age * 1e3)

    def report(self, now):
        """One summary line (and whether it looks unhealthy) once per period, else None."""
        wall = now - self.t_start
        if wall < self.period or not self.ticks:
            return None
        def pct(v, q):
            return float(np.percentile(v, q)) if v else float("nan")
        js_hz = (self.js_n - self.js_n0) / wall
        rtf = None
        if self.js_stamp is not None and self.js_stamp0 is not None:
            rtf = (self.js_stamp - self.js_stamp0) / wall
        try:
            cpu = Path("/proc/thread-self/stat").read_text().rsplit(")", 1)[1].split()[36]
        except (OSError, IndexError):
            cpu = "?"
        line = (f"[timing] loop {self.ticks / wall:4.1f} Hz (target {self.rate:.0f}), {self.overruns} overruns | "
                f"tick p50 {pct(self.parts['tick'], 50):4.1f} max {max(self.parts['tick']):5.1f} ms | "
                f"quest {pct(self.parts['xr'], 95):4.1f} ik {pct(self.parts['ik'], 95):4.1f} "
                f"(solve {pct(self.parts['solve'], 95):4.1f} max {max(self.parts['solve'], default=0.0):5.1f}, guard {pct(self.parts['guard'], 95):4.1f}) "
                f"pub {pct(self.parts['pub'], 95):4.1f} ms p95 | CPU {cpu} | joint states {js_hz:4.1f} Hz, "
                f"age p95 {pct(self.age, 95):5.0f} ms" + (f" | sim {rtf * 100:3.0f} % real time" if rtf is not None else ""))
        bad = (self.ticks / wall < 0.9 * self.rate or (rtf is not None and rtf < 0.9)
               or pct(self.age, 95) > 3e3 / self.rate)
        self.reset(now)
        return line, bad


class Trace:
    """CSV, one row per driven arm per control tick (and per step of the automatic home / reset / park moves), for
    debugging motion: what the hand did, what IK asked for, what was actually sent, where the arm really was.
    Every log message of the teleop goes into the `note` column of the next row. Overwritten on every start."""

    J = range(1, 7)
    COLS = (["t", "mode", "side", "tracked", "engaged", "grip", "trigger", "hand_x", "hand_y", "hand_z",
             "target_x", "target_y", "target_z"] + [f"ik{i}" for i in J] + [f"cmd{i}" for i in J]
            + [f"meas{i}" for i in J] + ["meas_age_ms", "lag_deg", "cmd_speed_dps", "limited", "sent", "note"])

    def __init__(self, path, args=None):
        self.f, self.notes, self.last_note = None, [], {}
        self.t0 = time.monotonic()
        if not path:
            return
        import csv
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(path, "w", newline="")
        self.f.write(f"# robot_remote teleop trace, started {time.strftime('%Y-%m-%d %H:%M:%S %z')}; overwritten on "
                     "every run\n# argv: " + " ".join(sys.argv[1:]) + "\n"
                     + (f"# limits: joint {getattr(args, 'max_joint_vel', '')} rad/s, accel {getattr(args, 'max_joint_acc', '')} "
                        f"rad/s^2, lead {getattr(args, 'max_cmd_lead_deg', '')} deg, TCP {getattr(args, 'max_lin_vel', '')} m/s, "
                        f"{getattr(args, 'max_rot_vel', '')} deg/s, rate {getattr(args, 'rate', '')} Hz\n" if args else "")
                     + "# units: t [s since start], joints [deg], hand_* [m, Quest controller in the operator frame: x fwd, "
                     "y left, z up], target_* [m, commanded flange in the arm base frame]; ik = IK solution, cmd = joint "
                     "command sent after the speed / acceleration / leash / collision limits (limited = which ones "
                     "changed it), meas = measured joints, lag_deg = max |cmd - meas|, sent = 1 if a command went out "
                     "this tick\n")
        self.w = csv.writer(self.f)
        self.w.writerow(self.COLS)
        self.last_flush = time.monotonic()

    def note(self, text):
        if self.f is None:
            return
        now = time.monotonic()
        if now - self.last_note.get(text, -1e9) < 1.0:  # throttled repeats (e.g. "holding") once a second
            return
        self.last_note[text] = now
        self.notes.append(text)

    def row(self, mode, side, tracked="", engaged="", grip="", trigger="", hand=None, target=None, ik=None,
            cmd=None, meas=None, meas_age=None, cmd_speed=None, limited="", sent=""):
        if self.f is None:
            return
        f3 = lambda v: ["" if v is None else f"{x:.4f}" for x in (v if v is not None else [None] * 3)]  # noqa: E731
        fd = lambda v: ["" if v is None else f"{x:.3f}" for x in (np.rad2deg(v) if v is not None else [None] * 6)]  # noqa: E731
        lag = "" if cmd is None or meas is None else f"{np.rad2deg(np.max(np.abs(np.asarray(cmd) - meas))):.3f}"
        b = lambda v: "" if v == "" else int(bool(v))  # noqa: E731
        self.w.writerow([f"{time.monotonic() - self.t0:.4f}", mode, side, b(tracked), b(engaged),
                         "" if grip == "" else f"{grip:.3f}", "" if trigger == "" else f"{trigger:.3f}",
                         *f3(hand), *f3(target), *fd(ik), *fd(cmd), *fd(meas),
                         "" if meas_age is None else f"{1000 * meas_age:.0f}", lag,
                         "" if cmd_speed is None else f"{cmd_speed:.2f}", limited, b(sent), " | ".join(self.notes)])
        self.notes = []
        if time.monotonic() - self.last_flush > 1.0:
            self.f.flush()
            self.last_flush = time.monotonic()

    def close(self):
        if self.f is not None:
            self.f.close()
            self.f = None


class TracingLog:
    """The node's logger, also copying every message into the trace (each method keeps one rclpy call site)."""

    def __init__(self, log, trace):
        self._log, self._trace = log, trace

    def __getattr__(self, name):
        return getattr(self._log, name)

    def debug(self, msg, **kw):
        return self._log.debug(msg, **kw)

    def info(self, msg, **kw):
        self._trace.note(msg)
        return self._log.info(msg, **kw)

    def warning(self, msg, **kw):
        self._trace.note(msg)
        return self._log.warning(msg, **kw)

    def error(self, msg, **kw):
        self._trace.note(msg)
        return self._log.error(msg, **kw)


def log_timing(log, line, bad):
    """WARN when unhealthy, else INFO. Two call sites on purpose: rclpy raises ValueError when ONE call site logs
    at a different severity than before (a single `(log.warning if bad else log.info)(line)` crashed the teleop)."""
    if bad:
        log.warning(line)
    else:
        log.info(line)


def tool_from_controller(r_tool: Rotation, twist_axis_ctrl, right_ctrl, right_base) -> Rotation:
    """Fixed-at-engage rotation M mapping controller-local axes onto tool-local axes:
       controller twist axis (Quest handle long axis)      -> tool +Z (flange axis): twist hand = spin tool
       'operator right' seen in the controller frame (at engage, orthogonalised)
                                                            -> tool axis pointing to base 'right'
                                                                                    : nod hand  = tilt tool
    Using the operator-right direction measured at engage (instead of a fixed controller +X) keeps the
    nod sense right for both hands, whatever the left/right OpenXR grip-frame conventions are."""
    def basis(a3, a1):
        e3 = np.asarray(a3, float) / np.linalg.norm(a3)
        e1 = np.asarray(a1, float) - np.dot(a1, e3) * e3
        if np.linalg.norm(e1) < 1e-3:  # degenerate: hand pointing sideways - fall back to any perpendicular
            e1 = np.cross(e3, [0.0, 0.0, 1.0]) if abs(e3[2]) < 0.9 else np.cross(e3, [1.0, 0.0, 0.0])
        e1 /= np.linalg.norm(e1)
        return np.column_stack([e1, np.cross(e3, e1), e3])

    e_basis = basis(twist_axis_ctrl, right_ctrl)
    f_basis = basis([0.0, 0.0, 1.0], r_tool.inv().apply(right_base))
    return Rotation.from_matrix(f_basis @ e_basis.T)


def make_urdf() -> str:
    pkg = subprocess.run(["ros2", "pkg", "prefix", "igus_rebel_description"], check=True,
                         capture_output=True, text=True).stdout.strip()
    xacro = Path(pkg) / "share/igus_rebel_description/urdf/igus_rebel_xeg32_wrapper.urdf.xacro"
    return subprocess.run(["xacro", str(xacro), "include_ros2_control:=false"], check=True,
                          capture_output=True, text=True).stdout


class Arm:
    """Per-arm clutch + IK state."""

    def __init__(self, side, chain, args):
        self.side, self.chain, self.args = side, chain, args
        self.names = list(ARM_JOINTS) if args.backend != "isaac" else [f"arm_{side}_{j}" for j in ARM_JOINTS]
        self.grip_names = [f"arm_{side}_xeg32_left_carriage_joint", f"arm_{side}_xeg32_right_carriage_joint"]
        self.shoulder = chain.joint_origin(1)
        self.world_T_base = np.eye(4)
        self.world_T_base[:3, :3] = Rotation.from_euler("z", ARM_BASE_YAW_DEG, degrees=True).as_matrix()
        self.world_T_base[:3, 3] = ARM_BASE_XYZ[side]
        self.q_meas = None
        self.t_meas = 0.0  # monotonic time of the last joint state
        self.q_cmd = None
        self.q_ik_tick, self.limited = None, []  # this tick's IK solution and the limits that changed it (trace)
        self.v_cmd = np.zeros(6)  # velocity of the last command (acceleration-limited real-ros command)
        self.v_meas = np.zeros(6)  # measured joint velocity, smoothed (real-ros)
        self.engaged = False
        self.tracked = False
        self.home_p = self.home_r = self.origin_p = self.origin_r = None
        self.p_des = None
        self.reach = args.max_reach
        self.last_trig = None
        self.last_log = 0.0
        self.r_delta = Rotation.identity()
        self.d_local = Rotation.identity()
        self.origin_q = Rotation.identity()
        self.m_tool_ctrl = Rotation.identity()
        self.rot_frame = args.rot_frame  # latched at engage from DualArmTeleop.rot_frame
        self.raw_quat = np.array([0.0, 0.0, 0.0, 1.0])

    def capsules(self, q):
        """Collision capsules of this arm (links + flange + XEG-32 + knife, whatever --tcp-preset is) in the
        shared world frame, for the inter-arm guard."""
        return capsules(self.chain, np.asarray(q), self.world_T_base, self.args.arm_radius, self.args.tool_radius)


class DualArmTeleop(Node):
    def __init__(self, args, chain):
        super().__init__("quest_dual_arm_teleop")
        self.args = args
        self.arms = {s: Arm(s, chain, args) for s in args.arms}
        self.stats = LoopStats(args.rate, args.diag_period)
        self.trace = Trace(getattr(args, "trace", None), args)
        self.tlog = TracingLog(self.get_logger(), self.trace)
        # Real arms: place the bases from the hand-eye calibrations (world = camera_link); sim: USD poses.
        self.base_poses = {}
        if args.backend != "isaac":
            self.base_poses = load_world_T_base(args.calibration)
            for side, arm in self.arms.items():
                arm.world_T_base = self.base_poses[side]
        self.watch = None          # optional read-only link to the arm we do NOT drive (--watch-other)
        # One real arm selected: the other arm is assumed to stand still at --other-arm-deg (default all 0,
        # straight up) for the collision guard, unless --watch-other reads its live pose.
        self.watch_side = next((s for s in SIDES if s not in self.arms), None) if args.backend != "isaac" else None
        self.real = args.backend != "isaac"
        self.ethernet = None
        if args.backend == "real":
            self.ethernet = EthernetArms(load_endpoints(args.config, args.left_ip, args.right_ip), args.arms,
                                         dry_run=args.dry_run, enable_motors=args.enable_motors, control=args.control,
                                         state_timeout=args.state_timeout,
                                         command_timeout=args.command_timeout, max_joint_vel=args.max_joint_vel,
                                         max_track_error=np.deg2rad(args.max_track_err_deg), rate=args.rate)
        elif self.real:
            # igus ROS driver per arm (real_robot/dual_arm_driver.launch.py): /<side>/joint_states in,
            # /<side>/rebel_arm_trajectory_controller/joint_trajectory out
            self.traj_pub = {s: self.create_publisher(JointTrajectory, args.real_cmd_topic.format(side=s), 10)
                             for s in self.arms}
            for s in self.arms:
                self.create_subscription(JointState, args.real_state_topic.format(side=s),
                                         lambda m, s=s: self._on_real_js(s, m), LATEST_SENSOR)
        else:
            self.cmd_pub = self.create_publisher(JointState, "/dual_arm/isaac_joint_commands", 10)
        # for teleop_viz / RViz: where each arm's TCP is being asked to go (frame <side>/base_link)
        self.target_pub = {s: self.create_publisher(PoseStamped, f"/quest_teleop/{s}/target", 10) for s in args.arms}
        # for record_dataset.py: full 14-D action every tick + episode events from the controller buttons
        self.action_pub = self.create_publisher(JointState, "/quest_teleop/action", 10)
        self.event_pub = self.create_publisher(String, "/quest_teleop/events", 10)
        # for teleop_viz.py --source real (RViz of the physical arms): measured joints of both arms (the
        # undriven one at its assumed / watched pose), each driving controller's pose, and the grip state
        self.viz_js_pub = self.create_publisher(JointState, "/quest_teleop/joint_states", 10)
        self.hand_pub = {s: self.create_publisher(PoseStamped, f"/quest_teleop/{s}/hand", 10) for s in args.arms}
        self.state_pub = self.create_publisher(String, "/quest_teleop/state", 10)
        # rotation frame (tool / base), switchable while running: GUI button (stdin line "rot_frame tool|base")
        # or `ros2 topic pub --once /quest_teleop/set_rot_frame std_msgs/String "data: base"`
        self.rot_frame = args.rot_frame
        self.create_subscription(String, "/quest_teleop/set_rot_frame", lambda m: self.set_rot_frame(m.data), 10)
        # scene reset (left controller Y, GUI button / stdin "reset_scene"): sim only, see reset_scene()
        self.reset_requested = False
        self.scene_reset_pub = None if self.real else self.create_publisher(Empty, "/dual_arm/reset_scene", 1)
        self.all_q = {s: None for s in SIDES}       # measured arm joints of BOTH arms (driven or not)
        self.all_grip = {s: 0.0 for s in SIDES}     # measured gripper closedness [0, 1]
        if not self.real:
            self.create_subscription(JointState, "/dual_arm/isaac_joint_states", self._on_js, LATEST)

    def set_rot_frame(self, mode):
        """Switch the rotation frame. An arm that is engaged keeps its frame until it is released and gripped
        again: the two mappings give different targets for the same hand pose, so switching mid-grip would
        swing the tool."""
        mode = str(mode).strip().lower()
        if mode not in ("tool", "base"):
            self.get_logger().warning(f"rotation frame: unknown mode {mode!r} (tool or base)")
            return
        self.rot_frame = mode
        held = [s for s, a in self.arms.items() if a.engaged and a.rot_frame != mode]
        self.get_logger().info(f"rotation frame: {mode}" + (
            f" (the {' + '.join(held)} arm keeps {self.arms[held[0]].rot_frame} until you release and grip again)"
            if held else ""))

    def read_commands(self, stream):
        """Command lines from the launcher (stdin pipe): `rot_frame tool|base`, `reset_scene`. Ends at EOF."""
        for line in iter(stream.readline, ""):
            words = line.split()
            if len(words) == 2 and words[0] == "rot_frame":
                self.set_rot_frame(words[1])
            elif words == ["reset_scene"]:
                self.reset_requested = True
            elif words:
                self.get_logger().warning(f"unknown command {line.strip()!r} (expected: rot_frame tool|base, reset_scene)")

    def _on_real_js(self, side, msg):
        self.stats.on_state(msg)
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            q = np.array([d[j] for j in ARM_JOINTS])
            arm, now = self.arms[side], time.monotonic()
            if arm.q_meas is not None and now - arm.t_meas > 0.004:  # measured joint velocity (for the release brake)
                v = (q - arm.q_meas) / (now - arm.t_meas)
                arm.v_meas = 0.7 * arm.v_meas + 0.3 * v if now - arm.t_meas < 0.1 else np.zeros(6)
            arm.q_meas, arm.t_meas = q, now
            self.all_q[side] = q

    def _on_js(self, msg):
        self.stats.on_state(msg)
        d = dict(zip(msg.name, msg.position))
        for s in SIDES:
            names = [f"arm_{s}_{j}" for j in ARM_JOINTS]
            if all(n in d for n in names):
                self.all_q[s] = np.array([d[n] for n in names])
            g = f"arm_{s}_xeg32_left_carriage_joint"
            if g in d:
                a = self.args
                self.all_grip[s] = float(np.clip((d[g] - a.gripper_open) / (a.gripper_closed - a.gripper_open), 0.0, 1.0))
        for arm in self.arms.values():
            if all(n in d for n in arm.names):
                arm.q_meas = np.array([d[n] for n in arm.names])
                arm.t_meas = time.monotonic()

    def spin_for(self, seconds):
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            rclpy.spin_once(self, timeout_sec=0.002)
            self.update_ethernet()

    def update_ethernet(self):
        if self.ethernet:
            if self.ethernet.fault:
                raise RobotError(self.ethernet.fault)
            for side, link in self.ethernet.arms.items():
                if link.fault:
                    raise RobotError(f"{side}: {link.fault}")
                q, stamp = link.joints()
                self.arms[side].q_meas, self.arms[side].t_meas = np.array(q), stamp
                self.all_q[side] = np.array(q)

    def publish_viz(self, readings, hand_of):
        stamp = self.get_clock().now().to_msg()
        names, pos = [], []
        for s in SIDES:
            q = self.arms[s].q_meas if s in self.arms else self.undriven_q(s)
            if q is not None:
                names += [f"arm_{s}_{j}" for j in ARM_JOINTS]
                pos += [float(v) for v in q]
        if names:
            msg = JointState(name=names, position=pos)
            msg.header.stamp = stamp
            self.viz_js_pub.publish(msg)
        state = {}
        for s, arm in self.arms.items():
            tracked, p, quat, squeeze, _ = readings[hand_of[s]]
            state[s] = "engaged" if arm.engaged else ("tracked" if tracked else "lost")
            if tracked:
                ps = PoseStamped()
                ps.header.frame_id, ps.header.stamp = "quest", stamp  # operator frame: x fwd, y left, z up
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
                (ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
                 ps.pose.orientation.w) = map(float, quat)
                self.hand_pub[s].publish(ps)
        self.state_pub.publish(String(data=json.dumps(state)))

    def undriven_q(self, side):
        """Real backends: joints of the arm this session does not drive - watched live, else the assumed pose."""
        if side != self.watch_side:
            return None
        if self.watch is not None and self.watch.status is not None:
            return np.array(self.watch.status.joints)
        return np.deg2rad(self.args.other_arm_deg)

    def other_arm(self):
        """(side, capsules) of the watched arm from its latest STATUS, or (side, None) if that is stale."""
        if self.watch_side is None:
            return None, None
        chain = self.arms[next(iter(self.arms))].chain
        if self.watch is None:  # not watched: assumed parked at --other-arm-deg
            return self.watch_side, capsules(chain, np.deg2rad(self.args.other_arm_deg), self.base_poses[self.watch_side],
                                             self.args.arm_radius, self.args.tool_radius)
        with self.watch._lock:
            st, fault = self.watch.status, self.watch.fault
        if fault or st is None or time.monotonic() - st.received > self.args.state_timeout:
            return self.watch_side, None
        return self.watch_side, capsules(chain, np.array(st.joints), self.base_poses[self.watch_side],
                                         self.args.arm_radius, self.args.tool_radius)

    def destroy_node(self):
        self.trace.close()
        if self.watch:
            self.watch.close()
        if self.ethernet:
            self.ethernet.close()
        return super().destroy_node()

    def wait_ready(self):
        if self.ethernet and len(self.arms) == 1 and self.args.watch_other:
            other = self.watch_side
            ep = load_endpoints(self.args.config, self.args.left_ip, self.args.right_ip)[other]
            try:
                self.watch = watch_arm(other, ep["host"], ep["cri_port"], self.args.state_timeout)
                self.watch_side = other
                self.get_logger().info(f"watching the {other} arm read-only at {ep['host']} for the collision guard "
                                       f"(joints [deg] {np.round(np.rad2deg(self.watch.status.joints), 1).tolist()})")
            except (OSError, RobotError) as exc:
                raise RobotError(f"cannot read the {other} arm for the collision guard ({exc}); "
                                 "fix the link or run without --watch-other (assumes it at --other-arm-deg)") from exc
        if self.watch_side and self.watch is None:
            self.get_logger().info(f"collision guard: the {self.watch_side} arm is not driven and assumed to stay at "
                                   f"{self.args.other_arm_deg} deg (use --watch-other if someone moves it)")
        if self.ethernet:
            self.ethernet.connect()
            for side in self.ethernet.enabled:
                self.get_logger().warning(f"[{side}] motors enabled (Reset + Enable)")
            self.update_ethernet()
            for side, link in self.ethernet.arms.items():
                self.get_logger().info(f"real arm connected [{side}] {link.host}:{link.cri_port}, "
                                       f"joints [deg] {np.round(np.rad2deg(link.status.joints), 2).tolist()}; "
                                       f"ERROR={link.status.error}, KINSTATE={link.status.kinstate}")
            return
        what = (f"{self.args.real_state_topic.format(side='<side>')} from the real arms (ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}; "
                "is the robot driver running?)" if self.real else
                "/dual_arm/isaac_joint_states (is run_dual_arm_sim.sh running?)")
        while rclpy.ok() and any(a.q_meas is None for a in self.arms.values()):
            self.get_logger().info(f"waiting for {what}", throttle_duration_sec=3.0)
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.real:
            for side, arm in self.arms.items():
                self.get_logger().info(f"real arm connected [{side}] via the igus ROS driver, joints [deg] "
                                       f"{np.round(np.rad2deg(arm.q_meas), 1).tolist()}")
            foreign = self.foreign_publishers()
            if foreign:
                what = ", ".join(f"{n} on {self.args.real_cmd_topic.format(side=s)}" for s, n in foreign.items())
                if not self.args.dry_run:
                    raise RobotError(f"another node publishes arm commands ({what}): MoveIt / Servo / a second teleop "
                                     "or a test on this ROS domain. Refusing to drive - stop it first")
                self.get_logger().warning(f"other command publisher(s): {what} (dry run: not driving anyway)")

    def foreign_publishers(self):
        """real-ros: {side: number of OTHER nodes publishing on that arm's trajectory command topic}. Two command
        sources on one arm fight; a test teleop on the robot's domain moved a live arm this way (2026-10-21)."""
        if not self.real or self.ethernet is not None:
            return {}
        out = {}
        for side in self.arms:
            n = self.count_publishers(self.args.real_cmd_topic.format(side=side)) - 1  # minus our own publisher
            if n > 0:
                out[side] = n
        return out

    def send(self, cmds: dict, grips: dict | None = None):
        if self.ethernet:
            return self.ethernet.send(cmds)
        if self.real:
            return self.send_real(cmds)
        names, positions = [], []
        for side, q in cmds.items():
            names += self.arms[side].names
            positions += [float(v) for v in q]
        for side, pos in (grips or {}).items():
            names += self.arms[side].grip_names
            positions += [float(pos), float(pos)]
        if names:
            self.cmd_pub.publish(JointState(name=names, position=positions))

    def release_brake(self, side, log):
        """Real-ros, when an arm is released (or loses tracking) while moving: the ReBeL keeps going past the last
        streamed command and the controller then pulls it back. The command instead GLIDES to a stop: the stopping
        distance comes from the command's own velocity (clean - an earlier version used the measured velocity, whose
        spikes once put the stopping point 30 deg away, 2026-10-21); the command then restarts from rest and is streamed
        toward that point (--glide-dec), so the lagging arm runs into it instead of past it. At most GLIDE_MAX_DEG; if
        the stopping point is too close to the other arm the arm holds where it is instead."""
        arm = self.arms[side]
        arm.gliding = False
        if not self.real or self.ethernet is not None or self.args.dry_run or not self.args.release_brake:
            return
        a = self.args.glide_dec or self.args.max_joint_acc or 0.52
        v = arm.v_cmd.copy()
        if arm.q_cmd is None or float(np.max(np.abs(v))) < np.deg2rad(1.0):
            return
        d = v * np.abs(v) / (2 * a)
        m = float(np.max(np.abs(d)))
        if m > np.deg2rad(GLIDE_MAX_DEG):
            d *= np.deg2rad(GLIDE_MAX_DEG) / m
        target = arm.q_cmd + d
        if make_guard(self, self.args)({side: target, **{s: self.arms[s].q_meas for s in self.arms if s != side}}):
            self.send_real({side: arm.q_meas.copy()})  # too close to the other arm: hold
            arm.q_cmd = arm.q_meas.copy()
            return
        arm.glide_target, arm.gliding = target, True
        log.info(f"[{side}] release: gliding to a stop {np.round(np.rad2deg(d), 1).tolist()} deg further")

    def glide_step(self, side, dt):
        """One tick of a release glide (None when the arm is not gliding or the glide has ended)."""
        arm = self.arms[side]
        if not getattr(arm, "gliding", False) or arm.engaged:
            arm.gliding = False
            return None
        a = self.args.glide_dec or self.args.max_joint_acc or 0.52
        q, _ = accel_step(arm.q_cmd, arm.v_cmd, arm.glide_target, self.args.max_joint_vel,
                          max(a, self.args.max_joint_acc or a), dt, adec=a)
        if self.args.max_cmd_lead_deg > 0:
            q = leash(q, arm.q_meas, np.deg2rad(self.args.max_cmd_lead_deg))
        if np.max(np.abs(q - arm.glide_target)) < 1e-4:
            arm.gliding = False
        return q

    def send_real(self, cmds: dict):
        """One short trajectory per tick to the real arm's joint_trajectory_controller: reach the commanded
        joints in --real-lookahead s. Each new message replaces the previous one; when messages stop (grip
        released, stale state, Wi-Fi gap) the controller finishes the last short segment and holds."""
        for side, q in cmds.items():
            if self.args.dry_run:
                continue
            la = self.args.real_lookahead
            pt = JointTrajectoryPoint(positions=[float(v) for v in q],
                                      time_from_start=Duration(sec=int(la), nanosec=int((la % 1) * 1e9)))
            self.traj_pub[side].publish(JointTrajectory(joint_names=self.arms[side].names, points=[pt]))

    def publish_action(self):
        """Commanded joints of both arms (held while released; measured for undriven arms) + gripper closedness."""
        names, pos = [], []
        for s in SIDES:
            arm = self.arms.get(s)
            q = arm.q_cmd if arm is not None and arm.q_cmd is not None else self.all_q[s]
            if q is None and arm is None:
                q = self.undriven_q(s)  # one-arm real session: the other arm's assumed / watched pose
            g = arm.last_trig if arm is not None and arm.last_trig is not None else self.all_grip[s]
            if q is None:
                return
            names += [f"arm_{s}_{j}" for j in ARM_JOINTS] + [f"arm_{s}_gripper"]
            pos += [float(v) for v in q] + [float(g)]
        msg = JointState(name=names, position=pos)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.action_pub.publish(msg)

    def go_home(self, vel=0.4, rate=30.0):
        self.move_to(HOME_DEG, vel, rate)

    def move_to(self, q_deg, vel, rate=30.0, guard=None, label="home", settle_s=1.0, tol_deg=1.0, interrupt=None):
        """Stream every driven arm along a straight joint-space line to q_deg at <= vel [rad/s] (sim and
        real-ros). guard(cmd) may return a reason to stop. Real arms: wait (<= 4 s) until within tol_deg.
        interrupt() -> reason or None is also checked while settling at the goal. Returns True when the pose was
        reached."""
        q_goal = np.deg2rad(q_deg)
        self.spin_for(0.1)
        start = {s: a.q_meas.copy() for s, a in self.arms.items()}
        span = max(np.max(np.abs(q_goal - q)) for q in start.values())
        steps = max(1, int(np.ceil(span / vel * rate)))
        log = self.tlog
        log.warning(f"[{label}] moving {', '.join(self.arms)} to {list(q_deg)} deg ({steps / rate:.1f} s at "
                    f"{np.rad2deg(vel):.0f} deg/s)")
        for k in range(1, steps + 1):
            cmd = {s: q + (q_goal - q) * k / steps for s, q in start.items()}
            reason = guard(cmd) if guard else None
            if reason:
                log.error(f"[{label}] stopped: {reason}")
                return False
            self.send(cmd)
            self.spin_for(1.0 / rate)
            for s, a in self.arms.items():
                self.trace.row(label, s, cmd=cmd[s], meas=a.q_meas, meas_age=time.monotonic() - a.t_meas, sent=1)
        end = time.monotonic() + (4.0 if self.real else settle_s)
        while True:
            reason = interrupt() if interrupt else None
            if reason:
                log.error(f"[{label}] stopped: {reason}")
                return False
            self.send({s: q_goal for s in self.arms})
            self.spin_for(1.0 / rate)
            for s, a in self.arms.items():
                self.trace.row(label + " settle", s, cmd=q_goal, meas=a.q_meas, meas_age=time.monotonic() - a.t_meas,
                               sent=1)
            err = max(np.max(np.abs(np.rad2deg(a.q_meas - q_goal))) for a in self.arms.values())
            if (self.real and err < tol_deg) or time.monotonic() > end:
                break
        log.warning(f"[{label}] done: " + ", ".join(f"{s} {np.round(np.rad2deg(a.q_meas), 1).tolist()} deg"
                                                    for s, a in self.arms.items()))
        return err < tol_deg or not self.real


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--backend", choices=["isaac", "real", "real-ros"], default="isaac",
                   help="real = dual-arm Ethernet; real-ros = legacy single-arm ROS topics")
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Ethernet endpoints JSON")
    p.add_argument("--left-ip", help="override left Ethernet IP")
    p.add_argument("--right-ip", help="override right Ethernet IP")
    p.add_argument("--command-timeout", type=float, default=0.25, help="Ethernet command watchdog [s]")
    p.add_argument("--control", choices=["jog", "position"], default="jog",
                   help="Ethernet: jog = CRI ALIVEJOG velocity jog like the igus ROS driver (default, works on our arms); "
                        "position = Position Interface streaming (stopped by KINSTATE 51 on RobotControl V15)")
    p.add_argument("--real-state-topic", default="/{side}/joint_states",
                   help="real-ros: joint states (joint1..6) of each arm; {side} = left / right")
    p.add_argument("--real-cmd-topic", default="/{side}/rebel_arm_trajectory_controller/joint_trajectory",
                   help="real-ros: joint_trajectory_controller command topic of each arm")
    p.add_argument("--real-lookahead", type=float, default=0.15,
                   help="real arm: each streamed point is reached this many seconds after it is sent")
    p.add_argument("--state-timeout", type=float, default=0.5,
                   help="real arm: let go (stop sending) if no joint state arrived for this long [s]")
    p.add_argument("--max-track-err-deg", type=float, default=15.0,
                   help="real arm: let go if any joint lags its command by more than this [deg]")
    p.add_argument("--max-cmd-lead-deg", type=float, default=None,
                   help="real-ros: keep each tick's joint command within this of the MEASURED joints (leash; 0 = off; "
                        "not used by the Ethernet backend, which has its own per-tick limit). Default "
                        "6 on the real arm, off in the sim. The ReBeL accelerates at only ~0.5 rad/s^2 behind ~0.25 s "
                        "of delay + lag (Igus_rebel controller.yaml), so a free-running command outran it by 15 deg "
                        "whenever the hand reversed. Under the leash the command waits for the arm; an arm that stays "
                        "at the leash without moving for --stall-s is let go instead")
    p.add_argument("--max-joint-acc", type=float, default=None,
                   help="real-ros: acceleration limit of the joint command [rad/s^2] (0 = off). Default 0.52 on the real "
                        "arm, the ~0.52 rad/s^2 it was identified to follow: the command then brakes the way the arm can, like a "
                        "MoveIt plan, instead of turning around instantly while the arm keeps going")
    p.add_argument("--max-joint-dec", type=float, default=None,
                   help="real-ros: braking deceleration of the joint command when it comes to rest [rad/s^2] (0 = same as "
                        "--max-joint-acc). Default 0.3: the arm then catches up before the command stops - real right arm "
                        "2026-10-21: overshoot 1.6 -> 0.3 deg and no pull-back, ~0.2 s longer per move")
    p.add_argument("--glide-dec", type=float, default=None,
                   help="real-ros: braking of the release glide [rad/s^2]. Default 0.4: real right arm 2026-10-21 (ff 0.8, "
                        "--max-joint-dec 0.3) slid ~20 deg from ~30 deg/s and came back 0.4 deg (0.52: 3-5 deg back)")
    p.add_argument("--no-release-brake", dest="release_brake", action="store_false",
                   help="real-ros: on release keep the last command (the arm overshoots it and is pulled back) instead "
                        "of letting the command glide to a stop")
    p.add_argument("--stall-s", type=float, default=1.0,
                   help="real arm with the leash: let go when the arm has not moved 0.5 deg for this long while its "
                        "command is at the leash (motors off, e-stop, blocked) [s]")
    p.add_argument("--dry-run", action="store_true", help="real: observe + Quest IK, no configuration or motion")
    p.add_argument("--real-home", action="store_true",
                   help="real arm: first move slowly (8.6 deg/s) to the teleop start pose HOME_DEG; off = start where it is")
    p.add_argument("--park-deg", type=float, nargs=6, default=[0.0] * 6, metavar="J",
                   help="Ethernet: joint pose [deg] every arm returns to before the program exits (default all 0)")
    p.add_argument("--no-start-home", action="store_true",
                   help=f"Ethernet: do not move the arms to the home pose {HOME_DEG} deg after connecting "
                        "(teleop then starts wherever they are; all 0 = straight up is singular)")
    p.add_argument("--park-vel-deg", type=float, default=10.0, help="Ethernet: speed of that return [deg/s]")
    p.add_argument("--no-park", action="store_true", help="Ethernet: do not return to --park-deg on exit")
    p.add_argument("--enable-motors", action="store_true",
                   help="Ethernet: if an arm only reports 'motor not enabled', send Reset + Enable at start "
                        "(like the iRC buttons; the arm then holds its pose). Refused on e-stop or other errors.")
    p.add_argument("--arms", nargs="+", choices=SIDES, default=list(SIDES), help="which rig arms to drive")
    p.add_argument("--swap-hands", action="store_true", help="left controller drives the right arm and vice versa")
    p.add_argument("--yaw-deg", type=float, default=0.0,
                   help="rotation about Z from operator frame (X fwd, Y left, Z up) to the arm base frames "
                        "(both arms share one orientation). 0 = you face the way the arms reach.")
    p.add_argument("--tcp-preset", choices=sorted(TCP_PRESETS), default="flange",
                   help="control point: flange = igus_tool0 on the flange (default), eef = repo knife tip xeg32_tool_tip, gripper = XEG-32 jaw centre")
    p.add_argument("--tcp", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                   help="custom control point in link6 frame [m]; overrides --tcp-preset")
    p.add_argument("--scale", type=float, default=1.0, help="hand->EE translation scale")
    p.add_argument("--rate", type=float, default=30.0)
    p.add_argument("--max-joint-vel", type=float, default=None,
                   help="rad/s per joint (default 0.785 = 45 deg/s in the sim, 0.35 on the real arm)")
    p.add_argument("--max-lin-vel", type=float, default=None, help="TCP speed limit [m/s] (default 0.5 sim, 0.15 real)")
    p.add_argument("--max-rot-vel", type=float, default=None,
                   help="TCP rotation speed limit [deg/s] (default 90 sim, 30 real)")
    p.add_argument("--rot-frame", choices=["tool", "base"], default="tool",  # also switchable while running
                   help="tool: twisting the controller spins the tool about its own Z, nodding tilts it "
                        "(controller-local rotation applied in the tool frame); base: hand rotation applied "
                        "about the fixed base axes")
    p.add_argument("--ctrl-twist-axis", type=float, nargs=3, default=(0.0, -0.707, -0.707), metavar=("X", "Y", "Z"),
                   help="Quest handle long axis in the controller (OpenXR grip) frame; measured from your "
                        "twists as about (-0.16, -0.73, -0.66)")
    p.add_argument("--max-lead-deg", type=float, default=15.0,
                   help="anti-windup: command may lead the commanded-joint pose by at most this rotation [deg]")
    p.add_argument("--max-lead-m", type=float, default=0.05,
                   help="anti-windup: command may lead the commanded-joint pose by at most this distance [m]")
    p.add_argument("--rot-weight", type=float, default=1.0, help="orientation gain of the secondary IK task")
    p.add_argument("--clutch", type=float, default=0.5, help="squeeze threshold")
    p.add_argument("--max-reach", type=float, default=0.55, help="EE target clamp around the shoulder [m]")
    p.add_argument("--min-arm-dist", type=float, default=0.05,
                   help="inter-arm guard: smallest allowed surface clearance between the two arms' capsules [m]")
    p.add_argument("--arm-radius", type=float, default=0.06, help="capsule radius of the arm links [m]")
    p.add_argument("--tool-radius", type=float, default=0.04, help="capsule radius of flange/gripper/knife [m]")
    p.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION,
                   help="real arms: hand-eye calibrations that place both arm bases (camera_link in each base)")
    p.add_argument("--other-arm-deg", type=float, nargs=6, default=[0.0] * 6, metavar="J",
                   help="real, one arm: pose [deg] the other (not driven) arm is assumed to keep, for the collision "
                        "guard (default all 0 = straight up)")
    p.add_argument("--watch-other", action="store_true",
                   help="Ethernet, one arm: read the other arm's live pose read-only over CRI instead of assuming "
                        "--other-arm-deg (when someone else is moving it)")
    p.add_argument("--gripper-open", type=float, default=0.00483688, help="carriage pos at trigger 0 [m]")
    p.add_argument("--gripper-closed", type=float, default=-0.0116631, help="carriage pos at trigger 1 [m]")
    p.add_argument("--no-home", action="store_true")
    p.add_argument("--quiet", action="store_true", help="no per-0.5 s axis-check log while engaged")
    p.add_argument("--trace", default=os.environ.get("ROBOT_REMOTE_TRACE", str(DEFAULT_TRACE)),
                   help="per-tick CSV of hand / IK / command / measured joints, overwritten every run ('' = off)")
    p.add_argument("--diag-period", type=float, default=5.0,
                   help="seconds between [timing] lines (loop rate, Quest / IK time, joint-state age, sim speed)")
    args, ros_args = p.parse_known_args(argv)
    for k, v in {"isaac": SIM_DEFAULTS, "real": ETHERNET_DEFAULTS, "real-ros": REAL_DEFAULTS}[args.backend].items():
        if getattr(args, k) is None:
            setattr(args, k, v)
    if len(set(args.arms)) != len(args.arms):
        p.error("--arms must not contain duplicates")
    for key in ("max_joint_acc", "max_joint_dec", "glide_dec", "max_cmd_lead_deg"):
        if not np.isfinite(getattr(args, key)) or getattr(args, key) < 0:
            p.error(f"--{key.replace('_', '-')} must be finite and >= 0")
    for key in ("rate", "state_timeout", "command_timeout", "max_joint_vel", "max_lin_vel", "max_rot_vel",
                "max_track_err_deg", "max_lead_deg", "max_lead_m", "max_reach", "scale", "min_arm_dist", "park_vel_deg",
                "arm_radius", "tool_radius", "diag_period"):
        if not np.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            p.error(f"--{key.replace('_', '-')} must be finite and positive")
    if args.backend != "isaac":

        if args.backend == "real" and args.real_home:
            p.error("Ethernet starts at the measured pose; prepare a bent pose with the robot's own controls")
        if args.backend == "real" and (args.rate < 10 or args.command_timeout < 2 / args.rate
                                       or args.command_timeout > .5):
            p.error("Ethernet requires rate >= 10 and 2/rate <= command-timeout <= 0.5 s")
        args.no_home = not args.real_home  # the real arm moves on its own only when asked to
    return args, ros_args


def main():
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    # rclpy only flags SIGTERM (rclpy.ok() -> False) and keeps running; during start-up make it abort instead.
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    tcp = tuple(args.tcp) if args.tcp else TCP_PRESETS[args.tcp_preset]
    chain = SerialChain(make_urdf(), "base_link", "link6", ARM_JOINTS, tcp_xyz=tcp)
    node = DualArmTeleop(args, chain)
    log = node.tlog
    if sys.stdin is not None and not sys.stdin.isatty():  # launched by the GUI: it writes commands to our stdin
        threading.Thread(target=node.read_commands, args=(sys.stdin,), daemon=True).start()
    log.info(f"control point ({'custom' if args.tcp else args.tcp_preset}) in link6 = {np.round(tcp, 4).tolist()} m")
    if node.real:
        log.warning(f"REAL ARMS ({', '.join(args.arms)}){' DRY RUN - observation only' if args.dry_run else ''}: "
                    f"joint {np.rad2deg(args.max_joint_vel):.0f} deg/s, TCP {args.max_lin_vel} m/s, "
                    f"{args.max_rot_vel:.0f} deg/s. Keep the e-stop in reach.")
    # Startup failures must also release every TCP socket and any configured position source.
    try:
        run_session(node, chain, args)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def accel_step(q_prev, v_prev, q_target, vmax, amax, dt, adec=None):
    """One tick of an acceleration-limited joint command toward q_target: speed <= vmax, |dv| <= amax*dt, and slow
    enough to stop at the target (|v| <= sqrt(2 adec |remaining|), adec = braking deceleration, default amax: a gentler
    braking lets the lagging arm catch up before the command stops). All joints are scaled uniformly (like
    step_toward), so the joint-space direction is kept. Returns (q, v)."""
    adec = amax if adec is None else adec
    rem = q_target - q_prev
    v_des = rem / dt
    m = float(np.max(np.abs(v_des)))
    if m > vmax:
        v_des = v_des * (vmax / m)
    moving = np.abs(v_des) > 1e-9
    if np.any(moving):
        ratio = float(np.min(np.sqrt(2 * adec * np.abs(rem[moving])) / np.abs(v_des[moving])))
        if ratio < 1.0:
            v_des = v_des * ratio
    dv = v_des - v_prev
    m = float(np.max(np.abs(dv))) / (amax * dt)
    if m > 1.0:
        dv = dv / m
    v = v_prev + dv
    return q_prev + v * dt, v


def leash(q_cmd, q_meas, lead):
    """Keep the joint command within `lead` [rad] of the measured joints (largest joint; all joints scaled
    uniformly, like step_toward, so the IK direction is kept)."""
    return step_toward(q_meas, q_cmd, lead)


def stall_check(arm, lag_deg, args, now):
    """True when the command is >= 3 deg (or 0.3 x the leash) away from the arm right now and the arm has not moved
    0.5 deg for --stall-s: it is not following at all (motors off, e-stop, blocked). A following arm that far behind
    is driven at ~10 deg/s, so it always moves."""
    if args.max_cmd_lead_deg <= 0 or args.backend != "real-ros":
        arm.stall_ref = None
        return False
    ref = getattr(arm, "stall_ref", None)
    if ref is None or np.rad2deg(np.max(np.abs(arm.q_meas - ref[1]))) > 0.5:
        arm.stall_ref = (now, arm.q_meas.copy())
        return False
    return lag_deg >= max(3.0, 0.3 * args.max_cmd_lead_deg) and now - ref[0] > args.stall_s


def make_guard(node, args):
    """Collision guard for the slow automatic moves (start home / park): stop if the arms' surfaces would come
    closer than --min-arm-dist (the other, undriven arm at its assumed / watched pose)."""
    def guard(cmd):
        if not args.dry_run and node.foreign_publishers():
            return f"another node publishes arm commands {node.foreign_publishers()}"
        caps = [node.arms[s].capsules(cmd[s]) for s in cmd]
        if len(caps) == 1:
            side, other = node.other_arm()
            if side is not None and other is None:
                return f"lost the {side} arm's state"
            if other is not None:
                caps.append(other)
        if len(caps) == 2:
            d = clearance(*caps)
            if d < args.min_arm_dist:
                return f"arms {d * 100:.1f} cm apart (surface clearance)"
        return None
    return guard


def operator_abort(xr, args):
    """For automatic moves started from the Quest: returns a check that is True once the operator presses any
    button or squeezes a grip. A button or grip already held when the move starts counts only after it is released."""
    if xr is None:
        return lambda: False
    held = {h: b for h, b in xr.buttons.items()}
    grip_armed = {}

    def abort():
        readings = xr.read_all()
        for h, (prim, sec) in xr.buttons.items():
            was_p, was_s = held.get(h, (False, False))
            if (prim and not was_p) or (sec and not was_s):
                return True
            held[h] = (prim and was_p, sec and was_s)  # a released button arms its next press
        for h, r in readings.items():
            tracked, squeeze = r[0], r[3]
            if not tracked:
                continue
            if squeeze < args.clutch - 0.1:
                grip_armed[h] = True
            elif squeeze > args.clutch + 0.1 and grip_armed.get(h):
                return True
        return False
    return abort


def reset_scene(node, args, log, xr=None, stop=None):
    """Left controller Y (or the GUI button): start the next recording without stopping anything. Sim only: both
    arms are released and driven back to HOME_DEG, the grippers open, then the simulator closes the box flaps
    (/dual_arm/reset_scene). The recorder discards an episode that is still open ("reset" event). Real arms are
    Real arms: the open episode is discarded and both arms move slowly (--park-vel-deg) to HOME_DEG with the collision
    guard; any button or a grip during that move stops the arms where they are. Dry run: no motion."""
    if node.real:
        node.event_pub.publish(String(data="reset"))
        for arm in node.arms.values():
            arm.engaged, arm.need_regrip = False, True
            arm.last_trig = None
        if args.dry_run or node.ethernet is not None:
            log.warning("[reset] real robot: open episode discarded" + (" (dry run: the arms are not moved)"
                        if args.dry_run else " (Ethernet backend: the arms are not moved)"))
            return
        log.warning(f"[reset] real robot: open episode discarded; arms -> home at {args.park_vel_deg:.0f} deg/s "
                    "(any button or grip stops them)")
        guard, abort = make_guard(node, args), operator_abort(xr, args)

        def interrupt():  # Stop all / Ctrl+C (the main loop's stop flag) or any Quest button / grip
            if stop is not None and stop.get("requested"):
                return "Stop all / Ctrl+C"
            return "stopped by the operator" if abort() else None

        def check(cmd):
            return interrupt() or guard(cmd)
        node.move_to(HOME_DEG, np.deg2rad(args.park_vel_deg), guard=check, label="reset", interrupt=interrupt)
        node.spin_for(0.2)
        for arm in node.arms.values():
            arm.q_cmd = arm.q_meas.copy()
        log.info("[reset] done - release and grip to drive the arms again")
        return
    node.event_pub.publish(String(data="reset"))  # record_dataset.py: discard the open episode, if any
    for arm in node.arms.values():
        arm.engaged = False
        arm.last_trig = None
    log.warning("[reset] scene reset: arms -> home, grippers open, box flaps closed")
    node.send({}, {s: args.gripper_open for s in node.arms})
    node.go_home(vel=0.4)
    node.scene_reset_pub.publish(Empty())  # after the arms are clear of the box
    node.spin_for(0.3)
    for arm in node.arms.values():
        arm.q_cmd = arm.q_meas.copy()
    log.info("[reset] done - grip to drive the arms again")


def start_home(node, args, log):
    """After connecting, bring the real arms slowly (--park-vel-deg) to HOME_DEG, the teleop start pose; all 0
    (straight up) is singular and hard to control. Any failure ends the session before the Quest drives anything."""
    if node.ethernet is None or args.dry_run or args.no_start_home:
        return
    log.warning(f"[home] moving {', '.join(node.arms)} to the home pose {HOME_DEG} deg - Ctrl+C / Stop all stops them")
    if not node.ethernet.park(np.deg2rad(HOME_DEG), np.deg2rad(args.park_vel_deg),
                              log=lambda m: log.warning(m.replace("[park]", "[home]")), guard=make_guard(node, args)):
        raise RobotError("the move to the home pose was stopped by the collision guard - move the arms apart by hand")


def park_on_exit(node, args, log):
    """Before a clean exit, bring the real arms slowly back to --park-deg (default: all joints 0).
    Skipped on a dry run, with --no-park, or after a fault (stale feedback, e-stop, lost link): the arms
    then stay where they stopped. A second Ctrl+C / Stop all during the return stops the arms at once."""
    if not (getattr(node, "real", False) or node.ethernet is not None) or args.dry_run or args.no_park:
        return
    if node.ethernet is None:  # igus ROS driver
        signal.signal(signal.SIGINT, signal.default_int_handler)
        signal.signal(signal.SIGTERM, signal.default_int_handler)
        try:
            node.move_to(args.park_deg, np.deg2rad(args.park_vel_deg), guard=make_guard(node, args), label="park")
        except KeyboardInterrupt:
            log.error("[park] interrupted - arms stopped where they are")
        except Exception as exc:
            log.error(f"[park] failed, arms stopped: {exc}")
        return
    if node.ethernet.fault or node.ethernet._closed.is_set():
        log.error(f"[park] not returning to {args.park_deg} deg: {node.ethernet.fault or 'link already closed'}")
        return
    chain = next(iter(node.arms.values())).chain
    q_park = np.deg2rad(args.park_deg)
    if np.any(q_park < chain.lower) or np.any(q_park > chain.upper):
        log.error(f"[park] --park-deg {args.park_deg} is outside the joint limits - not moving")
        return
    signal.signal(signal.SIGINT, signal.default_int_handler)  # rclpy's handler would swallow a second Ctrl+C
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    guard = make_guard(node, args)
    try:
        node.ethernet.park(q_park, np.deg2rad(args.park_vel_deg),
                           log=lambda m: log.warning(m + (" - finish the return by hand" if "stopped" in m else "")),
                           guard=guard)
    except KeyboardInterrupt:
        log.error("[park] interrupted - arms stopped where they are")
    except Exception as exc:  # RobotError, socket errors, anything else: the arms are stopped by close()
        log.error(f"[park] failed, arms stopped: {exc}")


def run_session(node, chain, args):
    log = node.tlog
    node.wait_ready()
    if node.ethernet:
        start_home(node, args, log)
        node.update_ethernet()
    elif node.real:  # igus ROS driver: same start-home move, streamed through its trajectory controllers
        if not args.dry_run and not args.no_start_home:
            if not node.move_to(HOME_DEG, np.deg2rad(args.park_vel_deg), guard=make_guard(node, args), label="home"):
                raise RobotError("the arms did not reach the home pose (collision guard, or motors not enabled?)")
    elif not args.no_home:
        if node.real and args.dry_run:
            log.info("dry run: skipping the move to the start pose")
        else:
            node.go_home(vel=0.15 if node.real else 0.4)
    node.spin_for(0.2)
    for arm in node.arms.values():
        arm.q_cmd = arm.q_meas.copy()

    hand_of = {s: ({"left": "right", "right": "left"}[s] if args.swap_hands else s) for s in node.arms}
    xr = QuestController(sorted(set(hand_of.values())), HERE / "cloudxr_quest3.env")
    log.info("CloudXR up. Quest Browser -> https://nvidia.github.io/IsaacTeleop/client/v1.3.131/ , "
             "Server IP = this PC, Connect. Per hand: hold SQUEEZE to move that arm. "
             + ("Real grippers are not controlled." if node.real else "TRIGGER = its gripper."))

    r_base_op = Rotation.from_euler("z", args.yaw_deg, degrees=True)
    right_base = r_base_op.apply([0.0, -1.0, 0.0])  # operator 'right' in the arm base frame
    log.info(f"rotation frame: {node.rot_frame}" + (
        f" (controller twist axis {np.round(args.ctrl_twist_axis, 2).tolist()} -> tool Z)" if node.rot_frame == "tool" else ""))
    dt = 1.0 / args.rate
    # Ethernet: stay 10 % under the transport's per-tick limit, so encoder noise between the engage snapshot and
    # the first streamed target can never trip it (that check would stop both arms).
    max_dq = args.max_joint_vel * dt * (0.9 if node.ethernet else 1.0)
    # Real arms: hysteresis on the grip (engage above clutch+0.1, release below clutch-0.1), and the operator
    # must show an open hand once before the first engage.
    hyst = 0.1 if node.real else 0.0
    on_th, off_th = args.clutch + hyst, args.clutch - hyst
    for arm in node.arms.values():
        arm.need_regrip = node.real
    both = len(node.arms) == 2
    last_buttons = {}
    # Ctrl+C / SIGTERM (GUI Stop all) only request a stop; the loop ends at a tick boundary, never in the
    # middle of a robot command, so the exit path can still return the arms to the park pose.
    stop = {"requested": False}
    foreign_check = [0.0, {}]  # [last check time, {side: count}] of other publishers on the arm command topics

    def request_stop(signum, frame):
        stop["requested"] = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    try:
        while rclpy.ok() and not stop["requested"]:
            t0 = time.perf_counter()
            for _ in range(3):  # one spin_once handled a joint state only every other tick (15 Hz); 3 -> every tick
                rclpy.spin_once(node, timeout_sec=0.0)
            readings = xr.read_all()
            t_xr = time.perf_counter()
            node.update_ethernet()
            cand, grips = {}, {}
            solve_s = 0.0  # IK solver time (both arms) this tick
            for side, arm in node.arms.items():
                arm.q_ik_tick, arm.limited = None, []
            if node.real and time.monotonic() - foreign_check[0] > 0.5:
                foreign_check[:] = [time.monotonic(), node.foreign_publishers()]
                if foreign_check[1] and not args.dry_run:
                    log.error(f"another node publishes arm commands {foreign_check[1]} - both arms released, no commands "
                              "until it stops (MoveIt / Servo / a second teleop / a test on this ROS domain?)",
                              throttle_duration_sec=2.0)
                    for arm in node.arms.values():
                        arm.engaged, arm.need_regrip = False, True
            for side, arm in node.arms.items():
                q_glide = node.glide_step(side, dt) if node.real else None
                if q_glide is not None:
                    cand[side] = q_glide
                    arm.limited.append("glide")
                tracked, pos, quat, squeeze, trigger = readings[hand_of[side]]
                if tracked != arm.tracked:
                    log.info(f"[{side} arm <- {hand_of[side]} hand] " + ("TRACKED" if tracked else "tracking LOST"))
                    arm.tracked = tracked
                if not tracked:
                    if arm.engaged:
                        node.release_brake(side, log)
                    arm.engaged = False
                    if node.real:
                        arm.need_regrip = True
                    continue
                if node.real:
                    age = time.monotonic() - arm.t_meas
                    lag = np.rad2deg(np.max(np.abs(arm.q_cmd - arm.q_meas))) if arm.engaged else 0.0
                    stalled = arm.engaged and not args.dry_run and stall_check(arm, lag, args, time.monotonic())
                    if age > args.state_timeout or lag > args.max_track_err_deg or stalled:
                        if node.ethernet and not args.dry_run:
                            raise RobotError(f"{side}: stale feedback or joint following error; stopping both arms")
                        arm.engaged = False
                        arm.need_regrip = True
                        log.warning(f"[{side}] let go: " + (f"no joint state for {age:.2f} s (network?)" if age > args.state_timeout
                                    else f"arm does not follow its command (no motion for {args.stall_s:.1f} s: "
                                         "motors off, e-stop, blocked?)" if stalled
                                    else f"arm lags its command by {lag:.0f} deg") + " - release and grip again")
                        continue
                if getattr(arm, "need_regrip", False):
                    if squeeze < off_th:
                        arm.need_regrip = False
                    continue
                p_b = r_base_op.apply(pos)
                r_b = r_base_op * Rotation.from_quat(quat) * r_base_op.inv()
                if squeeze >= on_th and not arm.engaged:
                    t_now = chain.fk(arm.q_meas)
                    arm.home_p, arm.home_r = t_now[:3, 3].copy(), Rotation.from_matrix(t_now[:3, :3])
                    arm.origin_p, arm.origin_r = p_b.copy(), r_b
                    arm.origin_q = Rotation.from_quat(quat)  # controller orientation at engage (operator frame)
                    arm.rot_frame = node.rot_frame
                    arm.m_tool_ctrl = tool_from_controller(arm.home_r, args.ctrl_twist_axis,
                                                           arm.origin_q.inv().apply([0.0, -1.0, 0.0]), right_base)
                    arm.q_cmd = arm.q_meas.copy()
                    arm.cmd_p, arm.cmd_r = arm.home_p.copy(), arm.home_r  # rate-limited Cartesian command
                    # never clamp tighter than where the arm already is, or engaging would jump it inward
                    arm.reach = max(args.max_reach, float(np.linalg.norm(arm.home_p - arm.shoulder)))
                    arm.engaged = True
                    arm.gliding = False
                    arm.v_cmd = np.zeros(6)
                    arm.stall_ref = None
                    log.info(f"[{side}] engaged ({arm.rot_frame}-frame rotation)")
                elif squeeze < off_th and arm.engaged:
                    arm.engaged = False
                    log.info(f"[{side}] released")
                    node.release_brake(side, log)
                if arm.engaged:
                    p_des = arm.home_p + args.scale * (p_b - arm.origin_p)
                    d = p_des - arm.shoulder
                    if np.linalg.norm(d) > arm.reach:
                        p_des = arm.shoulder + d * (arm.reach / np.linalg.norm(d))
                    arm.r_delta = r_b * arm.origin_r.inv()  # hand rotation since engage, in arm base axes
                    arm.raw_quat = quat
                    if arm.rot_frame == "tool":
                        # rotation of the controller about its OWN axes since engage, re-expressed on the
                        # tool's own axes and applied on the tool side (right-multiplied)
                        arm.d_local = arm.origin_q.inv() * Rotation.from_quat(quat)
                        r_des = arm.home_r * (arm.m_tool_ctrl * arm.d_local * arm.m_tool_ctrl.inv())
                    else:
                        r_des = arm.r_delta * arm.home_r
                    # Cartesian rate limit: move the commanded pose toward the hand pose at <= v / w, so a
                    # fast or large hand rotation is followed along a smooth Cartesian path instead of the
                    # joint-space shortcut (which made the TCP swing 10-18 cm during 60-90 deg wrist turns).
                    dp = p_des - arm.cmd_p
                    n = np.linalg.norm(dp)
                    if n > args.max_lin_vel * dt:
                        dp *= args.max_lin_vel * dt / n
                    arm.cmd_p = arm.cmd_p + dp
                    dr = (r_des * arm.cmd_r.inv()).as_rotvec()
                    a = np.linalg.norm(dr)
                    w_max = np.deg2rad(args.max_rot_vel) * dt
                    if a > w_max:
                        dr *= w_max / a
                    arm.cmd_r = Rotation.from_rotvec(dr) * arm.cmd_r
                    # Anti-windup: keep the command within --max-lead-* of what the commanded joints reach.
                    # When the hand asks for more than the wrist can do (e.g. 130 deg roll, pitch past joint5),
                    # the IK otherwise chases a far-away configuration and the TCP swings on the way.
                    t_ach = chain.fk(arm.q_cmd)
                    r_ach = Rotation.from_matrix(t_ach[:3, :3])
                    e_r = (arm.cmd_r * r_ach.inv()).as_rotvec()
                    m = np.linalg.norm(e_r)
                    lead_r = np.deg2rad(args.max_lead_deg)
                    if m > lead_r:
                        arm.cmd_r = Rotation.from_rotvec(e_r * (lead_r / m)) * r_ach
                    e_p = arm.cmd_p - t_ach[:3, 3]
                    m = np.linalg.norm(e_p)
                    if m > args.max_lead_m:
                        arm.cmd_p = t_ach[:3, 3] + e_p * (args.max_lead_m / m)
                    t_des = np.eye(4)
                    t_des[:3, 3] = arm.cmd_p
                    t_des[:3, :3] = arm.cmd_r.as_matrix()
                    t_s = time.perf_counter()
                    q_ik = chain.ik(arm.q_cmd, t_des, rot_weight=args.rot_weight)
                    solve_s += time.perf_counter() - t_s
                    shaped = node.real and node.ethernet is None and not args.dry_run
                    if shaped and args.max_joint_acc > 0:
                        cand[side], _ = accel_step(arm.q_cmd, arm.v_cmd, q_ik, args.max_joint_vel, args.max_joint_acc, dt,
                                                   adec=args.max_joint_dec or None)
                    else:
                        cand[side] = step_toward(arm.q_cmd, q_ik, max_dq)
                    if not np.allclose(cand[side], q_ik, atol=1e-9):
                        arm.limited.append("accel" if shaped and args.max_joint_acc > 0 else "speed")
                    if shaped and args.max_cmd_lead_deg > 0:
                        leashed = leash(cand[side], arm.q_meas, np.deg2rad(args.max_cmd_lead_deg))
                        if not np.allclose(leashed, cand[side], atol=1e-9):
                            arm.limited.append("leash")
                        cand[side] = leashed
                    arm.q_ik_tick = q_ik
                    arm.p_des = p_des
                    ps = PoseStamped()
                    ps.header.frame_id, ps.header.stamp = f"{side}/base_link", node.get_clock().now().to_msg()
                    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, arm.cmd_p)
                    qx, qy, qz, qw = arm.cmd_r.as_quat()
                    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = (
                        float(qx), float(qy), float(qz), float(qw))
                    node.target_pub[side].publish(ps)
                if not node.real and (arm.last_trig is None or abs(trigger - arm.last_trig) > 0.01):
                    grips[side] = args.gripper_open + (args.gripper_closed - args.gripper_open) * trigger
                    arm.last_trig = trigger

            # Inter-arm guard (capsule model, bases from the hand-eye calibrations on the real arms): refuse any
            # step that brings the two arms' surfaces closer than --min-arm-dist while also reducing the
            # clearance (moving apart is always allowed). With one driven arm the other arm is the watched one,
            # read-only from its controller; if its state is stale the driven arm holds.
            t_g = time.perf_counter()
            if cand and (both or node.watch_side is not None):
                now = {s: (node.arms[s].q_meas if node.ethernet else node.arms[s].q_cmd) for s in node.arms}
                nxt = {s: cand.get(s, now[s]) for s in node.arms}
                if both:
                    d_now = clearance(node.arms["left"].capsules(now["left"]), node.arms["right"].capsules(now["right"]))
                    d_next = clearance(node.arms["left"].capsules(nxt["left"]), node.arms["right"].capsules(nxt["right"]))
                    other_side, stale = None, False
                else:
                    (side_d,) = node.arms
                    other_side, other = node.other_arm()
                    stale = other is None
                    if not stale:
                        d_now = clearance(node.arms[side_d].capsules(now[side_d]), other)
                        d_next = clearance(node.arms[side_d].capsules(nxt[side_d]), other)
                if stale:
                    log.warning(f"no fresh state from the {other_side} arm - holding (collision guard blind)",
                                throttle_duration_sec=1.0)
                    cand = {}
                elif d_next < args.min_arm_dist and d_next < d_now:
                    log.warning(f"arms {d_next * 100:.1f} cm apart (surface clearance) - holding (move them apart)",
                                throttle_duration_sec=1.0)
                    cand = {}
                if not cand:
                    for arm in node.arms.values():
                        if arm.q_ik_tick is not None:
                            arm.limited.append("guard")
                if not cand and node.ethernet:
                    for arm in node.arms.values():
                        arm.engaged, arm.need_regrip = False, True

            t_ik = time.perf_counter()
            guard_s = t_ik - t_g
            for side, arm in node.arms.items():  # velocity of the command actually sent (after guard / leash)
                # Not sent this tick -> velocity 0, also on the tick of a release: the glide then starts from rest
                # and eases toward the stopping point while the lagging arm catches up. Deliberate: continuing from
                # the release speed came back 2.8 deg on the model arm vs 0.8 this way, and this way measured
                # 0.5-1 cm travel / no pull-back on the real arms (2026-10-21 18:58 session).
                arm.v_cmd = (cand[side] - arm.q_cmd) / dt if side in cand and arm.q_cmd is not None else np.zeros(6)
            for side, q in cand.items():
                node.arms[side].q_cmd = q
            if node.ethernet:
                # Preparation / release ACKs can take a tick; do not let stale controller input
                # start a new stream after a blocked control iteration.
                if time.perf_counter() - t0 > args.command_timeout:
                    node.ethernet.send({})
                    for arm in node.arms.values():
                        arm.engaged, arm.need_regrip = False, True
                    continue
            if foreign_check[1] and not args.dry_run:
                cand = {}
            node.send(cand, grips)
            for side, arm in node.arms.items():
                tr, pos, _, squeeze, trigger = readings[hand_of[side]]
                node.trace.row("teleop", side, tr, arm.engaged, squeeze, trigger, hand=pos if tr else None,
                               target=arm.cmd_p if arm.engaged else None, ik=arm.q_ik_tick, cmd=arm.q_cmd,
                               meas=arm.q_meas, meas_age=time.monotonic() - arm.t_meas,
                               cmd_speed=float(np.rad2deg(np.max(np.abs(arm.v_cmd)))), limited="+".join(arm.limited),
                               sent=side in cand and not args.dry_run)
            node.publish_action()
            node.publish_viz(readings, hand_of)
            # controller buttons (rising edges): A/X = start/save (toggle), B = discard, Y (left) = reset the scene
            for h, (prim, sec) in xr.buttons.items():
                was_p, was_s = last_buttons.get(h, (False, False))
                for pressed, was, ev in ((prim, was_p, "toggle"), (sec, was_s, "reset" if h == "left" else "discard")):
                    if pressed and not was:
                        log.info(f"[{h} hand] button -> {ev}")
                        if ev == "reset":
                            node.reset_requested = True
                        else:
                            node.event_pub.publish(String(data=ev))
                last_buttons[h] = (prim, sec)
            if node.reset_requested:
                node.reset_requested = False
                reset_scene(node, args, log, xr, stop)
                last_buttons.update(xr.buttons)  # a button that stopped the reset move must not also act here
                continue
            t_pub = time.perf_counter()

            if not args.quiet:
                for side, arm in node.arms.items():
                    if side in cand and t0 - arm.last_log > 0.5:
                        arm.last_log = t0
                        hand_op = r_base_op.inv().apply(r_base_op.apply(readings[hand_of[side]][1]) - arm.origin_p) * 100
                        ee = chain.fk(arm.q_meas)[:3, 3]
                        log.info(
                            f"[{side}] hand[cm] fwd %+5.1f left %+5.1f up %+5.1f | cmd dEE x %+5.1f y %+5.1f z %+5.1f"
                            " | actual dEE x %+5.1f y %+5.1f z %+5.1f"
                            % (*hand_op, *((arm.cmd_p - arm.home_p) * 100), *((ee - arm.home_p) * 100))
                        )

                        def rv(r):  # "angle deg about [axis]"
                            v = r.as_rotvec()
                            a = np.rad2deg(np.linalg.norm(v))
                            return f"{a:5.1f}deg about {np.round(v / np.linalg.norm(v), 2) if a > 0.5 else '[  -  ]'}"
                        r_meas = Rotation.from_matrix(chain.fk(arm.q_meas)[:3, :3])
                        if arm.rot_frame == "tool":
                            hand_t = arm.m_tool_ctrl * arm.d_local * arm.m_tool_ctrl.inv()
                            log.info(f"[{side}] ROT(tool frame) hand {rv(hand_t)} | cmd {rv(arm.home_r.inv() * arm.cmd_r)}"
                                     f" | actual {rv(arm.home_r.inv() * r_meas)}   [axis z = spin about flange axis]")
                            continue
                        hand_rot_op = r_base_op.inv() * arm.r_delta * r_base_op   # operator frame: x fwd, y left, z up
                        meas_rot = r_meas * arm.home_r.inv()
                        log.info(f"[{side}] ROT hand(op x fwd,y left,z up) {rv(hand_rot_op)} | cmd(base, rate-limited) {rv(arm.cmd_r * arm.home_r.inv())}"
                                 f" | actual(base) {rv(meas_rot)} | raw quat xyzw {np.round(arm.raw_quat, 3)}")
            t_end = time.perf_counter()
            node.stats.tick({"tick": t_end - t0, "xr": t_xr - t0, "ik": t_ik - t_xr, "pub": t_pub - t_ik,
                             "solve": solve_s, "guard": guard_s},
                            time.monotonic() - min(a.t_meas for a in node.arms.values()))
            if rep := node.stats.report(t_end):
                log_timing(log, *rep)
            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        pass
    except BaseException:
        if node.ethernet:
            node.ethernet.close()  # a fault: stop at once, no return to the park pose
        raise
    finally:
        # Return to the park pose on a clean exit, then stop physical streams before the slow CloudXR shutdown.
        try:
            park_on_exit(node, args, log)
        finally:
            try:
                if node.ethernet:
                    node.ethernet.close()
            finally:
                xr.close()


if __name__ == "__main__":
    main()
