#!/usr/bin/env python
"""Quest controller probe: connect via Isaac Teleop / CloudXR and print the right controller
state WITHOUT any robot attached. Use it to validate headset <-> workstation before driving SO101.

Run with the Isaac Teleop venv:
  cd ~/Robotic_infra/lerobot_isaac && source .venv/bin/activate
  python robot_remote/quest_probe.py

Prints grip position (already rebased into the robot base frame: +x forward, +y left, +z up),
orientation (roll/pitch/yaw, deg), squeeze and trigger. Ctrl-C to quit.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

LEROBOT_ISAAC = Path.home() / "Robotic_infra" / "lerobot_isaac"
sys.path.insert(0, str(LEROBOT_ISAAC))  # makes the `examples` package importable

from examples.isaac_teleop_to_so101.isaac_teleop import XRController, XRControllerConfig  # noqa: E402

DEFAULT_ENV = LEROBOT_ISAAC / "examples" / "isaac_teleop_to_so101" / "default.env"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hand", choices=["left", "right"], default="right")
    parser.add_argument("--hz", type=float, default=5.0, help="print rate")
    args = parser.parse_args()

    teleop = XRController(XRControllerConfig(hand_side=args.hand, cloudxr_env_file=str(DEFAULT_ENV)))
    print("Starting CloudXR runtime + session (~30 s) ...")
    teleop.connect()
    try:
        print("Connected. Open the CloudXR web client in the Quest browser and press Connect.")
        origin = None
        last = 0.0
        while True:
            a = teleop.get_action()
            now = time.perf_counter()
            if now - last < 1.0 / args.hz:
                time.sleep(0.005)
                continue
            last = now
            if not teleop.is_tracking:
                print("\rwaiting for controller tracking ...                                  ", end="", flush=True)
                origin = None
                continue
            pos = np.asarray(a["grip_pos"], dtype=float)
            if origin is None:
                origin = pos.copy()
            rpy = Rotation.from_quat(np.asarray(a["grip_quat"], dtype=float)).as_euler("xyz", degrees=True)
            d = (pos - origin) * 100
            print(
                f"\rpos[m]={np.round(pos, 3)}  d_from_start[cm]={np.round(d, 1)}  "
                f"rpy[deg]={np.round(rpy, 0)}  squeeze={a['squeeze']:.2f}  trigger={a['trigger']:.2f}   ",
                end="",
                flush=True,
            )
    except KeyboardInterrupt:
        pass
    finally:
        print("\nDisconnecting ...")
        teleop.disconnect()


if __name__ == "__main__":
    main()
