#!/usr/bin/env python
"""Keyboard teleop for an SO101 follower, used to validate the pipeline before Quest teleop.

Two modes:
  joint : each key pair drives one joint directly (no URDF / IK needed).
  ee    : keys jog an absolute end-effector target pose expressed in the robot BASE frame
          (x/y/z translation + rotation about the base x/y/z axes), solved with placo IK.

Run from the lerobot venv:
  cd ~/Robotic_infra/lerobot && source .venv/bin/activate
  python robot_remote/keyboard_so101.py --mode joint
  python robot_remote/keyboard_so101.py --mode ee

Press ESC to quit. Hold the arm before quitting: torque is released on disconnect.

Note: SO101 has only 5 arm DOF, so a full 6-D pose is generally not reachable. Position is
tracked strongly; orientation is a soft target weighted by --ori-weight. Rotation about the
base z axis (rz) at a fixed position, for instance, can only partially be realised by the wrist.
"""

import argparse
import time
from pathlib import Path

import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.robot_utils import precise_sleep

DEFAULT_URDF = Path(__file__).resolve().parent / "SO101" / "so101_new_calib.urdf"
MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ARM_JOINTS = MOTORS[:-1]

# joint mode: key -> (motor, direction)
JOINT_KEYS = {
    "1": ("shoulder_pan", +1), "q": ("shoulder_pan", -1),
    "2": ("shoulder_lift", +1), "w": ("shoulder_lift", -1),
    "3": ("elbow_flex", +1), "e": ("elbow_flex", -1),
    "4": ("wrist_flex", +1), "r": ("wrist_flex", -1),
    "5": ("wrist_roll", +1), "t": ("wrist_roll", -1),
    "6": ("gripper", +1), "y": ("gripper", -1),
}  # fmt: skip

# ee mode: key -> (index into [x, y, z, rx, ry, rz], direction); all axes are BASE-frame axes
EE_KEYS = {
    "w": (0, +1), "s": (0, -1),   # x  forward / back
    "a": (1, +1), "d": (1, -1),   # y  left / right
    "r": (2, +1), "f": (2, -1),   # z  up / down
    "i": (3, +1), "k": (3, -1),   # rx rotate about base x
    "j": (4, +1), "l": (4, -1),   # ry rotate about base y
    "u": (5, +1), "o": (5, -1),   # rz rotate about base z
}  # fmt: skip
GRIPPER_OPEN_KEY, GRIPPER_CLOSE_KEY = "n", "m"
RESYNC_KEY = "h"

HELP = {
    "joint": "1/q pan  2/w lift  3/e elbow  4/r wrist_flex  5/t wrist_roll  6/y gripper  ESC quit",
    "ee": (
        "base frame:  w/s x  a/d y  r/f z  |  i/k rx  j/l ry  u/o rz  |  n/m gripper open/close  |"
        "  h resync target to current pose  |  ESC quit"
    ),
}


class KeyState:
    def __init__(self):
        self.pressed: set[str] = set()
        self.quit = False
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)

    def _on_press(self, key):
        if key == keyboard.Key.esc:
            self.quit = True
        elif getattr(key, "char", None):
            self.pressed.add(key.char.lower())

    def _on_release(self, key):
        if getattr(key, "char", None):
            self.pressed.discard(key.char.lower())


def run_joint_mode(robot, keys: KeyState, fps: int, joint_speed: float, gripper_speed: float):
    obs = robot.get_observation()
    target = {m: float(obs[f"{m}.pos"]) for m in MOTORS}
    while not keys.quit:
        t0 = time.perf_counter()
        obs = robot.get_observation()
        for k in list(keys.pressed):
            if k in JOINT_KEYS:
                motor, sign = JOINT_KEYS[k]
                speed = gripper_speed if motor == "gripper" else joint_speed
                target[motor] += sign * speed / fps
        for m in MOTORS:
            # Avoid wind-up when a joint is blocked at its limit: keep target near the measured pose.
            cur = float(obs[f"{m}.pos"])
            target[m] = float(np.clip(target[m], cur - 10.0, cur + 10.0))
        target["gripper"] = float(np.clip(target["gripper"], 0.0, 100.0))
        robot.send_action({f"{m}.pos": v for m, v in target.items()})
        precise_sleep(max(1.0 / fps - (time.perf_counter() - t0), 0.0))


class BaseFrameEETarget:
    """Absolute EE target in the robot base frame, jogged by base-frame deltas and solved with IK.

    The target is integrated from commands (not re-read from the measured pose every tick), so it
    does not drift while idle. Anti-windup keeps it within a small distance of what IK can reach.
    """

    def __init__(self, kinematics, q_arm: np.ndarray, args):
        self.kin = kinematics
        self.args = args
        self.q_cmd = q_arm.copy()
        self.resync(q_arm)

    def resync(self, q_arm: np.ndarray):
        t = self.kin.forward_kinematics(q_arm)
        self.pos = t[:3, 3].copy()
        self.rot = Rotation.from_matrix(t[:3, :3])
        self.q_cmd = q_arm.copy()

    def jog(self, v: np.ndarray, dt: float):
        """v = [vx, vy, vz, wx, wy, wz] in the base frame (m/s, rad/s)."""
        self.pos = self.pos + v[:3] * dt
        # Left-multiplication = rotation about the fixed base axes, around the EE point.
        self.rot = Rotation.from_rotvec(v[3:] * dt) * self.rot
        lo, hi = np.array(self.args.ee_min), np.array(self.args.ee_max)
        self.pos = np.clip(self.pos, lo, hi)

    def solve(self, rotating: bool) -> np.ndarray:
        t_des = np.eye(4)
        t_des[:3, :3] = self.rot.as_matrix()
        t_des[:3, 3] = self.pos
        q = self.q_cmd
        for _ in range(self.args.ik_iters):
            q = self.kin.inverse_kinematics(
                q, t_des, position_weight=1.0, orientation_weight=self.args.ori_weight
            )
        self.q_cmd = q

        # Anti-windup on orientation only: when the 5-DOF wrist cannot realise the commanded
        # rotation, keep the rotation target within max_rot_err of what was reached. Position is
        # not pulled toward the reached pose (that would let orientation drag the EE away); an
        # unreachable position is only bounded by the workspace box and max_pos_err from the
        # reached pose when no rotation is being commanded.
        t_ach = self.kin.forward_kinematics(q)
        err_p = self.pos - t_ach[:3, 3]
        n = np.linalg.norm(err_p)
        if not rotating and n > self.args.max_pos_err:
            self.pos = t_ach[:3, 3] + err_p * (self.args.max_pos_err / n)
        r_ach = Rotation.from_matrix(t_ach[:3, :3])
        err_r = (self.rot * r_ach.inv()).as_rotvec()
        a = np.linalg.norm(err_r)
        max_r = np.deg2rad(self.args.max_rot_err_deg)
        if a > max_r:
            self.rot = Rotation.from_rotvec(err_r * (max_r / a)) * r_ach
        return q


def run_ee_mode(robot, keys: KeyState, args):
    from lerobot.model.kinematics import RobotKinematics

    kin = RobotKinematics(
        urdf_path=str(args.urdf), target_frame_name="gripper_frame_link", joint_names=ARM_JOINTS
    )
    fps = args.fps
    obs = robot.get_observation()
    q_meas = np.array([obs[f"{m}.pos"] for m in ARM_JOINTS], dtype=float)
    target = BaseFrameEETarget(kin, q_meas, args)
    gripper = float(obs["gripper.pos"])

    last_print = 0.0
    resync_held = False
    while not keys.quit:
        t0 = time.perf_counter()
        obs = robot.get_observation()
        q_meas = np.array([obs[f"{m}.pos"] for m in ARM_JOINTS], dtype=float)

        if RESYNC_KEY in keys.pressed:
            if not resync_held:
                target.resync(q_meas)
            resync_held = True
        else:
            resync_held = False

        v = np.zeros(6)
        for k in list(keys.pressed):
            if k in EE_KEYS:
                idx, sign = EE_KEYS[k]
                v[idx] += sign
        v[:3] *= args.ee_speed
        v[3:] *= np.deg2rad(args.rot_speed)
        target.jog(v, 1.0 / fps)
        q_cmd = target.solve(rotating=bool(np.any(v[3:])))

        gripper += (
            float(GRIPPER_OPEN_KEY in keys.pressed) - float(GRIPPER_CLOSE_KEY in keys.pressed)
        ) * args.gripper_speed / fps
        gripper = float(np.clip(gripper, 0.0, 100.0))

        action = {f"{m}.pos": float(q) for m, q in zip(ARM_JOINTS, q_cmd, strict=True)}
        action["gripper.pos"] = gripper
        robot.send_action(action)

        if t0 - last_print > 0.5:
            t_meas = kin.forward_kinematics(q_meas)
            rpy_t = target.rot.as_euler("xyz", degrees=True)
            rpy_m = Rotation.from_matrix(t_meas[:3, :3]).as_euler("xyz", degrees=True)
            print(
                f"\rtarget xyz={np.round(target.pos, 3)} rpy={np.round(rpy_t, 1)} | "
                f"meas xyz={np.round(t_meas[:3, 3], 3)} rpy={np.round(rpy_m, 1)}      ",
                end="",
                flush=True,
            )
            last_print = t0
        precise_sleep(max(1.0 / fps - (time.perf_counter() - t0), 0.0))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["joint", "ee"], default="joint")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--id", default="hucenrotia_follower_arm")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--joint-speed", type=float, default=30.0, help="deg/s in joint mode")
    parser.add_argument("--gripper-speed", type=float, default=60.0, help="gripper %%/s")
    parser.add_argument("--max-relative-target", type=float, default=5.0, help="max deg per step")
    # ee mode
    parser.add_argument("--ee-speed", type=float, default=0.05, help="m/s")
    parser.add_argument("--rot-speed", type=float, default=30.0, help="deg/s")
    parser.add_argument("--ori-weight", type=float, default=0.01, help="IK orientation weight (pos=1)")
    parser.add_argument("--ik-iters", type=int, default=10)
    parser.add_argument("--max-pos-err", type=float, default=0.02, help="anti-windup, m")
    parser.add_argument("--max-rot-err-deg", type=float, default=15.0, help="anti-windup, deg")
    parser.add_argument("--ee-min", type=float, nargs=3, default=[-0.45, -0.45, 0.0], help="base frame, m")
    parser.add_argument("--ee-max", type=float, nargs=3, default=[0.45, 0.45, 0.45], help="base frame, m")
    args = parser.parse_args()

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port, id=args.id, use_degrees=True, max_relative_target=args.max_relative_target
        )
    )
    robot.connect()
    keys = KeyState()
    keys.listener.start()
    print(f"[{args.mode} mode] {HELP[args.mode]}")
    try:
        if args.mode == "joint":
            run_joint_mode(robot, keys, args.fps, args.joint_speed, args.gripper_speed)
        else:
            run_ee_mode(robot, keys, args)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nDisconnecting (torque off) ...")
        keys.listener.stop()
        robot.disconnect()


if __name__ == "__main__":
    main()
