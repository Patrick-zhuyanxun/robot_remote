#!/usr/bin/env python3
"""RViz view of the Quest teleop frames for the Isaac dual-arm rig (read-only, sends no commands).

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
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
from quest_arm_teleop import ARM_JOINTS, TCP_PRESETS, SerialChain  # noqa: E402
from quest_dual_arm_teleop import ARM_BASE_XYZ, ARM_BASE_YAW_DEG, make_urdf  # noqa: E402

SIDES = ("left", "right")


def tf_msg(parent, child, xyz, quat_xyzw, stamp):
    t = TransformStamped()
    t.header.frame_id, t.header.stamp, t.child_frame_id = parent, stamp, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
    (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = map(float, quat_xyzw)
    return t


class TeleopViz(Node):
    def __init__(self, yaw_deg, tcp_preset):
        super().__init__("teleop_viz")
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
        self.create_subscription(JointState, "/dual_arm/isaac_joint_states", self.on_js, 10)
        self.create_timer(1 / 30, self.tick)
        self.create_timer(1.0, self.markers)

        now = self.get_clock().now().to_msg()
        q_base = Rotation.from_euler("z", ARM_BASE_YAW_DEG, degrees=True).as_quat()
        static = []
        for s in SIDES:
            static.append(tf_msg("world", f"{s}/base_link", ARM_BASE_XYZ[s], q_base, now))
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
        text("world", (0, 0.25, 1.75), f"Quest teleop frames  (--yaw-deg {self.yaw_deg:+.0f})", (1, 1, 1), 0.05)
        self.mk_pub.publish(arr)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--yaw-deg", type=float, default=0.0, help="same value you give the teleop script")
    p.add_argument("--tcp-preset", choices=sorted(TCP_PRESETS), default="flange", help="same value you give the teleop script")
    args, ros_args = p.parse_known_args()
    rclpy.init(args=ros_args)
    node = TeleopViz(args.yaw_deg, args.tcp_preset)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
