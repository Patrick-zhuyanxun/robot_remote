#!/usr/bin/env python3
"""Relay one arm of the Isaac dual-arm rig to the single-arm igus_rebel ros2_control topics.

igus_rebel (hardware_protocol:=isaac_topic) talks topic_based_ros2_control on
  /isaac_joint_commands, /isaac_joint_states   with joints joint1..joint6
while the dual-arm Isaac rig (run_dual_arm_sim.sh) uses
  /dual_arm/isaac_joint_commands, /dual_arm/isaac_joint_states   with arm_<side>_joint1..6 (+ lift/sliders/...)

This node renames joints in both directions so MoveIt for a single ReBeL drives one arm of the rig.

  source /opt/ros/humble/setup.bash
  python3 robot_remote/dual_arm_sim/arm_relay.py --side right
"""

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]


class ArmRelay(Node):
    def __init__(self, side: str):
        super().__init__(f"isaac_{side}_arm_relay")
        self.to_rig = {j: f"arm_{side}_{j}" for j in ARM_JOINTS}
        self.from_rig = {v: k for k, v in self.to_rig.items()}
        self.cmd_pub = self.create_publisher(JointState, "/dual_arm/isaac_joint_commands", 10)
        self.state_pub = self.create_publisher(JointState, "/isaac_joint_states", 10)
        self.create_subscription(JointState, "/isaac_joint_commands", self.on_command, 10)
        self.create_subscription(JointState, "/dual_arm/isaac_joint_states", self.on_state, 10)
        self.get_logger().info(f"relaying joint1..6 <-> arm_{side}_joint1..6")

    def on_command(self, msg: JointState):
        out = JointState(header=msg.header)
        for i, name in enumerate(msg.name):
            if name not in self.to_rig:
                continue
            out.name.append(self.to_rig[name])
            if i < len(msg.position):
                out.position.append(msg.position[i])
        if out.name:
            self.cmd_pub.publish(out)

    def on_state(self, msg: JointState):
        out = JointState(header=msg.header)
        for i, name in enumerate(msg.name):
            if name not in self.from_rig:
                continue
            out.name.append(self.from_rig[name])
            for src, dst in ((msg.position, out.position), (msg.velocity, out.velocity), (msg.effort, out.effort)):
                if i < len(src):
                    dst.append(src[i])
        if out.name:
            self.state_pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=["left", "right"], default="right")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ArmRelay(args.side)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
