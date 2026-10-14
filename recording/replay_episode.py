#!/usr/bin/env python3
"""Replay a recorded episode's ACTIONS into the Isaac dual-arm rig and compare with the recorded STATES.

This is the round-trip check that action and observation.state are aligned: the arms are first moved to
the episode's first recorded state, then the actions are streamed at the dataset fps on SIM time, and the
measured joints are compared frame-by-frame with observation.state.

Do not run the Quest teleop at the same time (both would command the arms).
  robot_remote/recording/run_replay.sh --name dual_rebel_pick --episode 0
"""

import argparse
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from record_dataset import ARM_JOINTS, GRIPPER_CLOSED, GRIPPER_OPEN, NAMES, SIDES


from rclpy.node import Node


class Replayer(Node):
    def __init__(self):
        super().__init__("episode_replayer")
        self.pub = self.create_publisher(JointState, "/dual_arm/isaac_joint_commands", 10)
        self.state = None
        self.sim_time = None
        self.create_subscription(JointState, "/dual_arm/isaac_joint_states", self.on_state, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)

    def on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_state(self, msg):
        d = dict(zip(msg.name, msg.position))
        out = []
        for s in SIDES:
            out += [d[f"arm_{s}_{j}"] for j in ARM_JOINTS]
            c = d.get(f"arm_{s}_xeg32_left_carriage_joint", GRIPPER_OPEN)
            out.append(np.clip((c - GRIPPER_OPEN) / (GRIPPER_CLOSED - GRIPPER_OPEN), 0.0, 1.0))
        self.state = np.asarray(out)

    def send(self, v):
        names, pos = [], []
        for s in SIDES:
            i = NAMES.index(f"arm_{s}_joint1")
            names += [f"arm_{s}_{j}" for j in ARM_JOINTS]
            pos += [float(x) for x in v[i:i + 6]]
            g = float(v[NAMES.index(f"arm_{s}_gripper")])
            carriage = GRIPPER_OPEN + (GRIPPER_CLOSED - GRIPPER_OPEN) * g
            names += [f"arm_{s}_xeg32_left_carriage_joint", f"arm_{s}_xeg32_right_carriage_joint"]
            pos += [carriage, carriage]
        self.pub.publish(JointState(name=names, position=pos))

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", required=True)
    p.add_argument("--root", default=os.environ.get("ROBOT_REMOTE_DATASETS", "~/datasets"))
    p.add_argument("--episode", type=int, default=0)
    args = p.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(args.root).expanduser().resolve() / args.name
    ds = LeRobotDataset(f"local/{args.name}", root=root, episodes=[args.episode])
    hf = ds.hf_dataset
    idx = [i for i in range(len(hf)) if int(hf[i]["episode_index"]) == args.episode]
    actions = np.stack([hf[i]["action"].numpy() for i in idx])
    states = np.stack([hf[i]["observation.state"].numpy() for i in idx])
    print(f"[replay] episode {args.episode}: {len(idx)} frames @ {ds.fps} fps", flush=True)

    rclpy.init()
    node = Replayer()
    while rclpy.ok() and (node.state is None or node.sim_time is None):
        rclpy.spin_once(node, timeout_sec=0.1)

    # 1) go to the first recorded state (slowly), let it settle
    start = node.state.copy()
    for k in range(1, 91):
        node.send(start + (states[0] - start) * k / 90)
        node.spin_for(1 / 30)
    node.spin_for(1.5)
    print(f"[replay] start-state error {np.abs(node.state - states[0])[[*range(6), *range(7, 13)]].max():.4f} rad", flush=True)

    # 2) stream actions on sim time, sample the measured state at the same instants
    measured = []
    t_next = node.sim_time
    k = 0
    while k < len(actions):
        rclpy.spin_once(node, timeout_sec=0.002)
        if node.sim_time < t_next:
            continue
        measured.append(node.state.copy())   # state at frame k, before action k is applied (= how it was recorded)
        node.send(actions[k])
        k += 1
        t_next += 1.0 / ds.fps
    measured = np.stack(measured)

    arm = [*range(6), *range(7, 13)]
    err = np.abs(measured[:, arm] - states[:, arm])
    motion = np.abs(states[:, arm] - states[0, arm]).max()
    print(f"[replay] recorded arm motion (max joint excursion): {np.rad2deg(motion):.1f} deg", flush=True)
    print(f"[replay] |replayed state - recorded state|  mean {np.rad2deg(err.mean()):.2f} deg  "
          f"p95 {np.rad2deg(np.percentile(err, 95)):.2f} deg  max {np.rad2deg(err.max()):.2f} deg", flush=True)
    # alignment: does a time shift explain the residual better? (positive = replay lags the recording)
    best = min(range(-10, 11), key=lambda s: np.abs(np.roll(measured[:, arm], -s, 0)[10:-10] - states[10:-10, arm]).mean())
    print(f"[replay] best-fit frame shift between replay and recording: {best} frames", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
