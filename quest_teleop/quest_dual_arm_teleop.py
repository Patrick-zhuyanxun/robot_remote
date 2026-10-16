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
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from quest_arm_teleop import ARM_JOINTS, HOME_DEG, TCP_PRESETS, QuestController, SerialChain, step_toward
from ethernet_rebel import DEFAULT_CONFIG, EthernetArms, RobotError, load_endpoints

HERE = Path(__file__).resolve().parent
SIDES = ("left", "right")

# Arm base poses in the rig/world frame, from dual_arm_rig_v2.usda (sliders and lift at 0).
ARM_BASE_XYZ = {"left": (0.228209, -0.179297, 1.1093), "right": (-0.231737, -0.179297, 1.1093)}
ARM_BASE_YAW_DEG = -90.0

# Real-arm defaults (--backend real): slower than the sim, the operator stands next to the robot.
REAL_DEFAULTS = {"max_joint_vel": 0.35, "max_lin_vel": 0.15, "max_rot_vel": 30.0}
SIM_DEFAULTS = {"max_joint_vel": 0.785, "max_lin_vel": 0.50, "max_rot_vel": 90.0}

# OpenXR anchor frame is set by quest_arm_teleop.OPERATOR_T_ANCHOR: operator X fwd, Y left, Z up.


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
        self.raw_quat = np.array([0.0, 0.0, 0.0, 1.0])

    def points_world(self, q):
        """Guard points in the rig frame: wrist (joint5 origin) plus flange, XEG-32 jaw centre and knife
        tip - always all of them, whatever --tcp-preset is, so the ~18 cm of tooling past the flange
        is covered by the inter-arm guard."""
        t = np.eye(4)
        k = 0
        wrist = None
        for t_o, axis, idx in self.chain.elems[:-1]:  # last element = the fixed TCP offset
            t = t @ t_o
            if axis is not None:
                if k == 4:
                    wrist = t[:3, 3].copy()
                r = np.eye(4)
                r[:3, :3] = Rotation.from_rotvec(axis * q[idx]).as_matrix()
                t = t @ r
                k += 1
        tool = [t[:3, :3] @ np.array(TCP_PRESETS[n]) + t[:3, 3] for n in ("flange", "gripper", "eef")]
        pts = np.stack([wrist, *tool])
        return (self.world_T_base[:3, :3] @ pts.T).T + self.world_T_base[:3, 3]


class DualArmTeleop(Node):
    def __init__(self, args, chain):
        super().__init__("quest_dual_arm_teleop")
        self.args = args
        self.arms = {s: Arm(s, chain, args) for s in args.arms}
        self.real = args.backend != "isaac"
        self.ethernet = None
        if args.backend == "real":
            self.ethernet = EthernetArms(load_endpoints(args.config, args.left_ip, args.right_ip), args.arms,
                                         dry_run=args.dry_run, enable_motors=args.enable_motors,
                                         state_timeout=args.state_timeout,
                                         command_timeout=args.command_timeout, max_joint_vel=args.max_joint_vel,
                                         max_track_error=np.deg2rad(args.max_track_err_deg), rate=args.rate)
        elif self.real:
            self.traj_pub = self.create_publisher(JointTrajectory, args.real_cmd_topic, 10)
            self.create_subscription(JointState, args.real_state_topic, self._on_real_js, qos_profile_sensor_data)
        else:
            self.cmd_pub = self.create_publisher(JointState, "/dual_arm/isaac_joint_commands", 10)
        # for teleop_viz / RViz: where each arm's TCP is being asked to go (frame <side>/base_link)
        self.target_pub = {s: self.create_publisher(PoseStamped, f"/quest_teleop/{s}/target", 10) for s in args.arms}
        # for record_dataset.py: full 14-D action every tick + episode events from the controller buttons
        self.action_pub = self.create_publisher(JointState, "/quest_teleop/action", 10)
        self.event_pub = self.create_publisher(String, "/quest_teleop/events", 10)
        self.all_q = {s: None for s in SIDES}       # measured arm joints of BOTH arms (driven or not)
        self.all_grip = {s: 0.0 for s in SIDES}     # measured gripper closedness [0, 1]
        if not self.real:
            self.create_subscription(JointState, "/dual_arm/isaac_joint_states", self._on_js, 10)

    def _on_real_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            q = np.array([d[j] for j in ARM_JOINTS])
            for side, arm in self.arms.items():
                arm.q_meas, arm.t_meas = q, time.monotonic()
                self.all_q[side] = q

    def _on_js(self, msg):
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

    def destroy_node(self):
        if self.ethernet:
            self.ethernet.close()
        return super().destroy_node()

    def wait_ready(self):
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
        what = (f"{self.args.real_state_topic} from the real arm (ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}; "
                "is the robot driver running?)" if self.real else
                "/dual_arm/isaac_joint_states (is run_dual_arm_sim.sh running?)")
        while rclpy.ok() and any(a.q_meas is None for a in self.arms.values()):
            self.get_logger().info(f"waiting for {what}", throttle_duration_sec=3.0)
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.real:
            q = next(iter(self.arms.values())).q_meas
            self.get_logger().info(f"real arm connected, joints [deg] {np.round(np.rad2deg(q), 1).tolist()}")
            n = self.count_publishers(self.args.real_cmd_topic)
            if n:
                self.get_logger().warning(f"{n} other publisher(s) on {self.args.real_cmd_topic} (MoveIt / Servo?). "
                                          "They will fight the teleop if they send commands - stop them first.")

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
            self.traj_pub.publish(JointTrajectory(joint_names=self.arms[side].names, points=[pt]))

    def publish_action(self):
        """Commanded joints of both arms (held while released; measured for undriven arms) + gripper closedness."""
        names, pos = [], []
        for s in SIDES:
            arm = self.arms.get(s)
            q = arm.q_cmd if arm is not None and arm.q_cmd is not None else self.all_q[s]
            g = arm.last_trig if arm is not None and arm.last_trig is not None else self.all_grip[s]
            if q is None:
                return
            names += [f"arm_{s}_{j}" for j in ARM_JOINTS] + [f"arm_{s}_gripper"]
            pos += [float(v) for v in q] + [float(g)]
        msg = JointState(name=names, position=pos)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.action_pub.publish(msg)

    def go_home(self, vel=0.4, rate=30.0):
        q_home = np.deg2rad(HOME_DEG)
        start = {s: a.q_meas.copy() for s, a in self.arms.items()}
        span = max(np.max(np.abs(q_home - q)) for q in start.values())
        steps = max(1, int(np.ceil(span / vel * rate)))
        self.get_logger().info(f"moving {list(self.arms)} to home pose {HOME_DEG} deg ({steps / rate:.1f} s)")
        for k in range(1, steps + 1):
            self.send({s: q + (q_home - q) * k / steps for s, q in start.items()})
            self.spin_for(1.0 / rate)
        for _ in range(int(rate)):
            self.send({s: q_home for s in self.arms})
            self.spin_for(1.0 / rate)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--backend", choices=["isaac", "real", "real-ros"], default="isaac",
                   help="real = dual-arm Ethernet; real-ros = legacy single-arm ROS topics")
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Ethernet endpoints JSON")
    p.add_argument("--left-ip", help="override left Ethernet IP")
    p.add_argument("--right-ip", help="override right Ethernet IP")
    p.add_argument("--command-timeout", type=float, default=0.25, help="Ethernet command watchdog [s]")
    p.add_argument("--real-state-topic", default="/joint_states", help="real arm: joint states (joint1..6)")
    p.add_argument("--real-cmd-topic", default="/rebel_arm_trajectory_controller/joint_trajectory",
                   help="real arm: joint_trajectory_controller command topic")
    p.add_argument("--real-lookahead", type=float, default=0.15,
                   help="real arm: each streamed point is reached this many seconds after it is sent")
    p.add_argument("--state-timeout", type=float, default=0.5,
                   help="real arm: let go (stop sending) if no joint state arrived for this long [s]")
    p.add_argument("--max-track-err-deg", type=float, default=15.0,
                   help="real arm: let go if any joint lags its command by more than this [deg]")
    p.add_argument("--dry-run", action="store_true", help="real: observe + Quest IK, no configuration or motion")
    p.add_argument("--real-home", action="store_true",
                   help="real arm: first move slowly (8.6 deg/s) to the teleop start pose HOME_DEG; off = start where it is")
    p.add_argument("--park-deg", type=float, nargs=6, default=list(HOME_DEG), metavar="J",
                   help=f"Ethernet: joint pose [deg] every arm returns to before the program exits (default {HOME_DEG})")
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
    p.add_argument("--rot-frame", choices=["tool", "base"], default="tool",
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
    p.add_argument("--min-arm-dist", type=float, default=0.12,
                   help="inter-arm guard: TCP/wrist points of the two arms may not get closer than this [m]")
    p.add_argument("--gripper-open", type=float, default=0.00483688, help="carriage pos at trigger 0 [m]")
    p.add_argument("--gripper-closed", type=float, default=-0.0116631, help="carriage pos at trigger 1 [m]")
    p.add_argument("--no-home", action="store_true")
    p.add_argument("--quiet", action="store_true", help="no per-0.5 s axis-check log while engaged")
    args, ros_args = p.parse_known_args(argv)
    for k, v in (REAL_DEFAULTS if args.backend != "isaac" else SIM_DEFAULTS).items():
        if getattr(args, k) is None:
            setattr(args, k, v)
    if len(set(args.arms)) != len(args.arms):
        p.error("--arms must not contain duplicates")
    for key in ("rate", "state_timeout", "command_timeout", "max_joint_vel", "max_lin_vel", "max_rot_vel",
                "max_track_err_deg", "max_lead_deg", "max_lead_m", "max_reach", "scale", "min_arm_dist", "park_vel_deg"):
        if not np.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            p.error(f"--{key.replace('_', '-')} must be finite and positive")
    if args.backend != "isaac":
        if args.backend == "real-ros" and len(args.arms) != 1:
            p.error("--backend real-ros requires --arms left or --arms right")
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
    log = node.get_logger()
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


def park_on_exit(node, args, log):
    """Before a clean exit, bring the real arms slowly back to --park-deg (default: HOME_DEG).
    Skipped on a dry run, with --no-park, or after a fault (stale feedback, e-stop, lost link): the arms
    then stay where they stopped. A second Ctrl+C / Stop all during the return stops the arms at once."""
    if node.ethernet is None or args.dry_run or args.no_park:
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

    def guard(cmd):
        if len(cmd) == 2 and args.min_arm_dist > 0:
            a, b = (node.arms[s].points_world(np.array(cmd[s])) for s in SIDES)
            d = np.min(np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2))
            if d < args.min_arm_dist:
                return f"arms {d * 100:.1f} cm apart - finish the return by hand"
        return None
    try:
        node.ethernet.park(q_park, np.deg2rad(args.park_vel_deg),
                           log=lambda m: log.warning(m), guard=guard)
    except KeyboardInterrupt:
        log.error("[park] interrupted - arms stopped where they are")
    except Exception as exc:  # RobotError, socket errors, anything else: the arms are stopped by close()
        log.error(f"[park] failed, arms stopped: {exc}")


def run_session(node, chain, args):
    log = node.get_logger()
    node.wait_ready()
    if not args.no_home:
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
    log.info(f"rotation frame: {args.rot_frame}" + (
        f" (controller twist axis {np.round(args.ctrl_twist_axis, 2).tolist()} -> tool Z)" if args.rot_frame == "tool" else ""))
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

    def request_stop(signum, frame):
        stop["requested"] = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    try:
        while rclpy.ok() and not stop["requested"]:
            t0 = time.perf_counter()
            rclpy.spin_once(node, timeout_sec=0.0)
            readings = xr.read_all()
            node.update_ethernet()
            cand, grips = {}, {}
            for side, arm in node.arms.items():
                tracked, pos, quat, squeeze, trigger = readings[hand_of[side]]
                if tracked != arm.tracked:
                    log.info(f"[{side} arm <- {hand_of[side]} hand] " + ("TRACKED" if tracked else "tracking LOST"))
                    arm.tracked = tracked
                if not tracked:
                    arm.engaged = False
                    if node.real:
                        arm.need_regrip = True
                    continue
                if node.real:
                    age = time.monotonic() - arm.t_meas
                    lag = np.rad2deg(np.max(np.abs(arm.q_cmd - arm.q_meas))) if arm.engaged else 0.0
                    if age > args.state_timeout or lag > args.max_track_err_deg:
                        if node.ethernet and not args.dry_run:
                            raise RobotError(f"{side}: stale feedback or joint following error; stopping both arms")
                        arm.engaged = False
                        arm.need_regrip = True
                        log.warning(f"[{side}] let go: " + (f"no joint state for {age:.2f} s (network?)" if age > args.state_timeout
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
                    arm.m_tool_ctrl = tool_from_controller(arm.home_r, args.ctrl_twist_axis,
                                                           arm.origin_q.inv().apply([0.0, -1.0, 0.0]), right_base)
                    arm.q_cmd = arm.q_meas.copy()
                    arm.cmd_p, arm.cmd_r = arm.home_p.copy(), arm.home_r  # rate-limited Cartesian command
                    # never clamp tighter than where the arm already is, or engaging would jump it inward
                    arm.reach = max(args.max_reach, float(np.linalg.norm(arm.home_p - arm.shoulder)))
                    arm.engaged = True
                    log.info(f"[{side}] engaged")
                elif squeeze < off_th and arm.engaged:
                    arm.engaged = False
                    log.info(f"[{side}] released")
                if arm.engaged:
                    p_des = arm.home_p + args.scale * (p_b - arm.origin_p)
                    d = p_des - arm.shoulder
                    if np.linalg.norm(d) > arm.reach:
                        p_des = arm.shoulder + d * (arm.reach / np.linalg.norm(d))
                    arm.r_delta = r_b * arm.origin_r.inv()  # hand rotation since engage, in arm base axes
                    arm.raw_quat = quat
                    if args.rot_frame == "tool":
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
                    q_ik = chain.ik(arm.q_cmd, t_des, rot_weight=args.rot_weight)
                    cand[side] = step_toward(arm.q_cmd, q_ik, max_dq)
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

            # Inter-arm guard: refuse any step that brings the arms' TCP/wrist points closer than
            # --min-arm-dist while also decreasing their distance (moving apart is always allowed).
            if both and cand:
                def min_dist(qs):
                    a = node.arms["left"].points_world(qs["left"])
                    b = node.arms["right"].points_world(qs["right"])
                    return np.min(np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2))
                now = {s: (node.arms[s].q_meas if node.ethernet else node.arms[s].q_cmd) for s in SIDES}
                nxt = {s: cand.get(s, now[s]) for s in SIDES}
                d_now, d_next = min_dist(now), min_dist(nxt)
                if d_next < args.min_arm_dist and d_next < d_now:
                    log.warning(f"arms {d_next * 100:.1f} cm apart - holding both (move them apart)",
                             throttle_duration_sec=1.0)
                    cand = {}
                    if node.ethernet:
                        for arm in node.arms.values():
                            arm.engaged, arm.need_regrip = False, True

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
            node.send(cand, grips)
            node.publish_action()
            # episode events: A/X = start/save (toggle), B/Y = discard; rising edges, either hand
            for h, (prim, sec) in xr.buttons.items():
                was_p, was_s = last_buttons.get(h, (False, False))
                for pressed, was, ev in ((prim, was_p, "toggle"), (sec, was_s, "discard")):
                    if pressed and not was:
                        node.event_pub.publish(String(data=ev))
                        log.info(f"[{h} hand] button -> {ev}")
                last_buttons[h] = (prim, sec)

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
                        if args.rot_frame == "tool":
                            hand_t = arm.m_tool_ctrl * arm.d_local * arm.m_tool_ctrl.inv()
                            log.info(f"[{side}] ROT(tool frame) hand {rv(hand_t)} | cmd {rv(arm.home_r.inv() * arm.cmd_r)}"
                                     f" | actual {rv(arm.home_r.inv() * r_meas)}   [axis z = spin about flange axis]")
                            continue
                        hand_rot_op = r_base_op.inv() * arm.r_delta * r_base_op   # operator frame: x fwd, y left, z up
                        meas_rot = r_meas * arm.home_r.inv()
                        log.info(f"[{side}] ROT hand(op x fwd,y left,z up) {rv(hand_rot_op)} | cmd(base, rate-limited) {rv(arm.cmd_r * arm.home_r.inv())}"
                                 f" | actual(base) {rv(meas_rot)} | raw quat xyzw {np.round(arm.raw_quat, 3)}")
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
