#!/usr/bin/env python3
"""RViz view of the Quest teleop (read-only, sends no commands): the Isaac rig (--source isaac, default) or the
physical arms driven over Ethernet (--source real).

--source real: joints from /quest_teleop/joint_states (published by the teleop: both arms, the undriven one at its
assumed / watched pose), arm bases from the hand-eye calibrations (world = the LEFT arm's base_link), and each
Quest controller as TF {side}/controller under the frame "quest" (operator frame: x forward, y left, z up,
placed by --quest-offset). The controller sphere is green while its grip drives the arm.

Publishes
  /left/joint_states, /right/joint_states   joint1..6 relayed from /dual_arm/isaac_joint_states
                                            (feeds two robot_state_publishers with frame_prefix left/ right/)
  TF world -> {left,right}/base_link        arm mounts from dual_arm_rig_v2.usda
  TF {side}/link6 -> {side}/control_tcp     the teleop control point (--tcp-preset, default flange)
  TF {side}/base_link -> {side}/hand_axes   at the TCP, rotated by --yaw-deg: these axes are the
                                            directions the TCP target moves when the HAND moves
                                            red = hand forward, green = hand left, blue = hand up
  /teleop_viz/markers                       text labels

  robot_remote/quest_teleop/run_teleop_viz.sh [--yaw-deg 0]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
from quest_arm_teleop import ARM_JOINTS, TCP_PRESETS, SerialChain  # noqa: E402
from quest_dual_arm_teleop import ARM_BASE_XYZ, ARM_BASE_YAW_DEG, make_urdf  # noqa: E402
from arm_collision import DEFAULT_CALIBRATION, load_world_T_base  # noqa: E402

SIDES = ("left", "right")


def tf_msg(parent, child, xyz, quat_xyzw, stamp):
    t = TransformStamped()
    t.header.frame_id, t.header.stamp, t.child_frame_id = parent, stamp, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
    (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = map(float, quat_xyzw)
    return t


class TeleopViz(Node):
    def __init__(self, yaw_deg, tcp_preset, source="isaac", quest_offset=(-0.6, -0.224, -1.0),
                 calibration=DEFAULT_CALIBRATION):
        super().__init__("teleop_viz")
        self.source = source
        self.hand_state = {}
        self.tcp = TCP_PRESETS[tcp_preset]
        self.tcp_preset = tcp_preset
        self.chain = SerialChain(make_urdf(), "base_link", "link6", ARM_JOINTS, tcp_xyz=self.tcp)
        self.r_axes = Rotation.from_euler("z", yaw_deg, degrees=True)
        self.yaw_deg = yaw_deg
        self.q = {s: None for s in SIDES}
        self.js_pub = {s: self.create_publisher(JointState, f"/{s}/joint_states", 10) for s in SIDES}
        self.mk_pub = self.create_publisher(MarkerArray, "/teleop_viz/markers", 1)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        if source == "real":
            self.create_subscription(JointState, "/quest_teleop/joint_states", self.on_js, 10)
            for s in SIDES:
                self.create_subscription(PoseStamped, f"/quest_teleop/{s}/hand",
                                         lambda m, s=s: self.on_hand(s, m), 10)
            self.create_subscription(String, "/quest_teleop/state", self.on_state, 10)
        else:
            self.create_subscription(JointState, "/dual_arm/isaac_joint_states", self.on_js, 10)
        self.create_timer(1 / 30, self.tick)
        self.create_timer(1.0, self.markers)

        now = self.get_clock().now().to_msg()
        static = []
        if source == "real":
            # world = left base_link; right base from the two hand-eye calibrations (camera_link in each base)
            w = load_world_T_base(calibration)
            for s in SIDES:
                t = np.linalg.inv(w["left"]) @ w[s]
                static.append(tf_msg("world", f"{s}/base_link", t[:3, 3], Rotation.from_matrix(t[:3, :3]).as_quat(), now))
            # operator frame "quest": axes = left base axes rotated by --yaw-deg, origin by --quest-offset
            static.append(tf_msg("world", "quest", quest_offset, self.r_axes.as_quat(), now))
        else:
            q_base = Rotation.from_euler("z", ARM_BASE_YAW_DEG, degrees=True).as_quat()
            for s in SIDES:
                static.append(tf_msg("world", f"{s}/base_link", ARM_BASE_XYZ[s], q_base, now))
        for s in SIDES:
            static.append(tf_msg(f"{s}/link6", f"{s}/control_tcp", self.tcp, [0, 0, 0, 1], now))
        self.static.sendTransform(static)
        self.get_logger().info(f"teleop frames: hand axes = arm base axes rotated {yaw_deg:+.0f} deg about Z")

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        for s in SIDES:
            names = [f"arm_{s}_{j}" for j in ARM_JOINTS]
            if all(n in d for n in names):
                self.q[s] = np.array([d[n] for n in names])
                self.js_pub[s].publish(JointState(header=msg.header, name=ARM_JOINTS, position=list(self.q[s])))

    def on_hand(self, side, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.tf.sendTransform([tf_msg("quest", f"{side}/controller", (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
                                      self.get_clock().now().to_msg())])

    def on_state(self, msg):
        try:
            self.hand_state = json.loads(msg.data)
        except ValueError:
            pass
        self.markers()

    def tick(self):
        now = self.get_clock().now().to_msg()
        out = []
        for s in SIDES:
            if self.q[s] is None:
                continue
            p = self.chain.fk(self.q[s])[:3, 3]
            out.append(tf_msg(f"{s}/base_link", f"{s}/hand_axes", p, self.r_axes.as_quat(), now))
        if out:
            self.tf.sendTransform(out)

    def markers(self):
        arr = MarkerArray()
        k = 0

        def text(frame, xyz, txt, rgb, size=0.03):
            nonlocal k
            m = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, id=k, ns="labels", text=txt)
            m.header.frame_id = frame
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
            m.pose.orientation.w = 1.0
            m.scale.z = size
            m.color.r, m.color.g, m.color.b, m.color.a = (*map(float, rgb), 1.0)
            arr.markers.append(m)
            k += 1

        for s in SIDES:
            f = f"{s}/hand_axes"
            text(f, (0.12, 0, 0), "hand FORWARD", (1.0, 0.3, 0.3))
            text(f, (0, 0.12, 0), "hand LEFT", (0.3, 1.0, 0.3))
            text(f, (0, 0, 0.12), "hand UP", (0.4, 0.6, 1.0))
            text(f, (0, 0, 0.17), "(translation directions only - never rotates)", (0.8, 0.8, 0.8), 0.02)
            text(f"{s}/base_link", (0, 0, -0.06), f"{s.upper()} arm base_link (IK frame)", (1, 1, 1), 0.035)
            text(f"{s}/control_tcp", (0, 0, -0.04), f"{s} control point ({self.tcp_preset}) - rotates with the wrist", (1, 1, 0.3), 0.025)
        if self.source == "real":
            for s in SIDES:
                st = self.hand_state.get(s)
                if st is None:
                    continue
                m = Marker(type=Marker.SPHERE, action=Marker.ADD, id=k, ns="controllers")
                m.header.frame_id = f"{s}/controller"
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.05
                rgb = {"engaged": (0.2, 1.0, 0.2), "tracked": (0.9, 0.9, 0.9), "lost": (1.0, 0.2, 0.2)}[st]
                m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 0.9)
                arr.markers.append(m)
                k += 1
                text(f"{s}/controller", (0, 0, 0.06), f"{s} controller: " + {"engaged": "GRIP - driving the arm",
                     "tracked": "released", "lost": "tracking lost"}[st], rgb, 0.03)
            text("world", (0, 0.25, 1.2), f"REAL arms  (Quest operator frame: --yaw-deg {self.yaw_deg:+.0f})", (1, 1, 1), 0.05)
        else:
            text("world", (0, 0.25, 1.75), f"Quest teleop frames  (--yaw-deg {self.yaw_deg:+.0f})", (1, 1, 1), 0.05)
        self.mk_pub.publish(arr)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--yaw-deg", type=float, default=0.0, help="same value you give the teleop script")
    p.add_argument("--tcp-preset", choices=sorted(TCP_PRESETS), default="flange", help="same value you give the teleop script")
    p.add_argument("--source", choices=["isaac", "real"], default="isaac",
                   help="isaac = the simulated rig; real = the physical arms (teleop --backend real)")
    p.add_argument("--quest-offset", type=float, nargs=3, default=(-0.6, -0.224, -1.0), metavar=("X", "Y", "Z"),
                   help="real: where the Quest operator-frame origin is drawn, in the left base frame [m]")
    args, ros_args = p.parse_known_args()
    rclpy.init(args=ros_args)
    node = TeleopViz(args.yaw_deg, args.tcp_preset, args.source, tuple(args.quest_offset))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
