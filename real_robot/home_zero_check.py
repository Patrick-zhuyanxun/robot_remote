#!/usr/bin/env python3
"""Hardware check of the Ethernet path without a Quest: connect, (optionally) Reset + Enable, stream the arms slowly
to the home pose, hold, stream them back to all joints 0, release. Same transport, warm-up, speed limit, following
error checks and inter-arm capsule guard as the Quest teleop's start-home / exit-park moves.

  source /opt/ros/humble/setup.bash && source igus_ws/install/setup.bash
  .venv_quest/bin/python real_robot/home_zero_check.py --arms left right --enable-motors

Someone stands at the e-stop. Ctrl+C stops the arms at once (the Position Interface link is closed).
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "quest_teleop"))
from arm_collision import capsules, clearance, load_world_T_base  # noqa: E402
from ethernet_rebel import DEFAULT_CONFIG, EthernetArms, RobotError, load_endpoints  # noqa: E402
from quest_arm_teleop import ARM_JOINTS, HOME_DEG, SerialChain  # noqa: E402
from quest_dual_arm_teleop import make_urdf  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arms", nargs="+", choices=("left", "right"), default=["left", "right"])
    p.add_argument("--enable-motors", action="store_true", help="Reset + Enable arms that report motor not enabled")
    p.add_argument("--vel-deg", type=float, default=10.0)
    p.add_argument("--hold-s", type=float, default=2.0)
    p.add_argument("--min-arm-dist", type=float, default=0.05)
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="arm endpoints JSON")
    p.add_argument("--control", choices=["jog", "position"], default="jog")
    args = p.parse_args()

    chain = SerialChain(make_urdf(), "base_link", "link6", ARM_JOINTS)
    w = load_world_T_base()
    other = {"left": "right", "right": "left"}

    def guard(cmd):
        caps = {s: capsules(chain, np.array(q), w[s], .06, .04) for s, q in cmd.items()}
        if len(caps) == 1:
            (s,) = caps
            caps[other[s]] = capsules(chain, np.zeros(6), w[other[s]], .06, .04)  # undriven arm assumed at 0
        d = clearance(*caps.values())
        return f"arms {d * 100:.1f} cm apart" if d < args.min_arm_dist else None

    pair = EthernetArms(load_endpoints(args.config), args.arms, enable_motors=args.enable_motors, control=args.control)
    log_ctrl = f"control = {args.control}"
    log = lambda m: print(m, flush=True)  # noqa: E731
    t0 = time.monotonic()
    try:
        pair.connect()
        print(f"[check] {log_ctrl}", flush=True)
        for s in pair.enabled:
            log(f"[check] {s}: motors enabled (Reset + Enable)")
        for s, a in pair.arms.items():
            log(f"[check] {s} start joints [deg] {np.round(np.rad2deg(a.joints()[0]), 2).tolist()}")
        for name, target in (("home", np.deg2rad(HOME_DEG)), ("zero", np.zeros(6))):
            ok = pair.park(target, np.deg2rad(args.vel_deg), log=lambda m, n=name: log(m.replace("[park]", f"[{n}]")),
                           guard=guard)
            if not ok:
                raise RobotError(f"move to {name} stopped by the guard")
            for s, a in pair.arms.items():
                q = np.rad2deg(a.joints()[0])
                with a._lock:
                    st = a.status
                log(f"[check] {s} at {name}: joints [deg] {np.round(q, 2).tolist()}, max error "
                    f"{np.max(np.abs(q - np.rad2deg(target))):.2f} deg, {st.error} KINSTATE {st.kinstate}; "
                    f"unknown PI messages: {a.unknown_position_messages[:3]}")
            if name == "home":
                time.sleep(args.hold_s)
        log(f"[check] PASSED in {time.monotonic() - t0:.1f} s")
    except KeyboardInterrupt:
        log("[check] interrupted - arms stopped")
    except (OSError, RobotError) as exc:
        log(f"[check] FAILED: {exc}")
        for s, a in pair.arms.items():
            with a._lock:
                st = a.status
            if st:
                log(f"[check] {s} now: joints {np.round(np.rad2deg(st.joints), 2).tolist()} {st.error} "
                    f"KINSTATE {st.kinstate}; unknown PI messages: {a.unknown_position_messages[:5]}")
        sys.exit(1)
    finally:
        pair.close()


if __name__ == "__main__":
    main()
