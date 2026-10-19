"""Headless physics check of the box-task scene (setup_box_task_scene.py). No ROS, no real robot.

  ~/isaac-sim-6.1/python.sh robot_remote/dual_arm_sim/check_box_task_scene.py [--usd FILE] [--only camera]

1. rest:   5 s untouched -> box drift [mm], flap angles [deg]
2. flaps:  each flap set 60 deg open -> must fall back closed; 110 deg -> must fall fully open
3. push:   box given 0.3 m/s sideways -> slides, stops on the table, stays upright
4. penetration: box pushed 1 cm into the table -> ejection speed must stay at the 0.5 m/s cap
5. camera: arm_left_joint1 swung +-45 deg -> wrist camera tilt joint wobble [deg]
"""
import argparse
from pathlib import Path

HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser()
ap.add_argument("--usd", type=Path, default=HERE / "dual_arm_rig_v2.usda")
ap.add_argument("--only", choices=["camera"], default=None, help="camera wobble only (e.g. on the old USD)")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import UsdPhysics  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

omni.usd.get_context().open_stage(str(args.usd))
while omni.usd.get_context().get_stage_loading_status()[2] > 0:
    app.update()
stage = omni.usd.get_context().get_stage()
for g in ("/World/DualArmJointBridge", "/World/DualArmCameraBridge"):
    if stage.GetPrimAtPath(g).IsValid():
        stage.RemovePrim(g)
root = next(p.GetPath().pathString for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI))
sim = SimulationContext(physics_dt=1 / 120, rendering_dt=1 / 60, stage_units_in_meters=1.0)
sim.initialize_physics()
robot = SingleArticulation(root, name="dual_arm")
robot.initialize()
sim.play()
names = robot.dof_names
home = np.deg2rad([0.0, 0.0, 65.0, 0.0, 50.0, 0.0] * 2)
arm_idx = np.array([names.index(f"arm_{s}_joint{i + 1}") for s in ("left", "right") for i in range(6)])
robot.set_joint_positions(home, joint_indices=arm_idx)
robot.apply_action(ArticulationAction(joint_positions=home, joint_indices=arm_idx))


def step(seconds):
    for _ in range(int(seconds * 60)):
        sim.step(render=False)


def pose(pr):
    p, q = pr.get_world_pose()
    return np.array(p), R.from_quat([q[1], q[2], q[3], q[0]])


def set_pose(pr, p, r):
    q = r.as_quat()
    pr.set_world_pose(position=p, orientation=np.array([q[3], q[0], q[1], q[2]]))
    pr.set_linear_velocity(np.zeros(3))
    pr.set_angular_velocity(np.zeros(3))


ok = True


def report(cond, text):
    global ok
    ok &= bool(cond)
    print(f"[check] {'PASS' if cond else 'FAIL'} {text}", flush=True)


def camera_test():
    tilt = names.index("arm_left_xeg32_camera_tilt_joint")
    step(1.0)
    angles = []
    for k in range(240):  # 4 s: +-45 deg on joint1, 1 s per swing
        target = home.copy()
        target[0] += np.deg2rad(45) * (1 if (k // 60) % 2 == 0 else -1)
        robot.apply_action(ArticulationAction(joint_positions=target, joint_indices=arm_idx))
        sim.step(render=False)
        angles.append(robot.get_joint_positions()[tilt])
    a = np.rad2deg(np.array(angles))
    print(f"[check] wrist camera tilt while swinging joint1: mean {a.mean():+.2f} deg, std {a.std():.3f} deg, "
          f"peak-to-peak {np.ptp(a):.3f} deg", flush=True)
    robot.apply_action(ArticulationAction(joint_positions=home, joint_indices=arm_idx))
    return np.ptp(a)


if args.only == "camera":
    camera_test()
    app.close()
    raise SystemExit(0)

BOX = "/World/CardboardBox"
body = SingleRigidPrim(f"{BOX}/Body")
body.initialize()
# flap: (hinge axis in the Body frame, opening sign, hinge point in the Body frame)
FLAPS = {"LidBack": ([0, 1, 0], +1), "LidFront": ([0, 1, 0], -1), "LidLeft": ([1, 0, 0], -1), "LidRight": ([1, 0, 0], +1)}
flaps = {}
for n in FLAPS:
    pr = SingleRigidPrim(f"{BOX}/{n}")
    pr.initialize()
    j = stage.GetPrimAtPath(f"{BOX}/{n}Joint")
    flaps[n] = (pr, np.array(j.GetAttribute("physics:localPos0").Get()))

step(0.5)
p_b0, r_b0 = pose(body)
closed = {n: pose(pr) for n, (pr, _) in flaps.items()}
rel0 = {n: (r_b0.inv() * closed[n][1]) for n in flaps}


def angle(n):
    """Opening angle of a flap [deg], 0 = closed, positive = open."""
    _, rb = pose(body)
    axis, sign = FLAPS[n]
    rel = rel0[n].inv() * (rb.inv() * pose(flaps[n][0])[1])
    rv = rel.as_rotvec()
    return sign * np.rad2deg(np.dot(rv, axis))


# 1. rest
step(5.0)
p_b, r_b = pose(body)
drift = np.linalg.norm(p_b - p_b0) * 1e3
report(drift < 1.0, f"rest 5 s: box drift {drift:.2f} mm; flap angles " +
       ", ".join(f"{n} {angle(n):+.2f}" for n in flaps) + " deg")
report(all(abs(angle(n)) < 1.0 for n in flaps), "rest: all four flaps stay closed")

# 2. flaps (majors first: a closed major flap lies on the minor flaps; the majors stay open afterwards)
for n in ("LidBack", "LidFront", "LidLeft", "LidRight"):
    pr, hinge_b = flaps[n]
    axis, sign = FLAPS[n]
    for start, expect in ((60, "closed"), (110, "open")):
        p_b, r_b = pose(body)
        hinge_w = p_b + r_b.apply(hinge_b)
        rot = R.from_rotvec(r_b.apply(axis) * np.deg2rad(sign * start))
        p_c, r_c = p_b + r_b.apply(r_b0.inv().apply(closed[n][0] - p_b0)), r_b * rel0[n]
        set_pose(pr, hinge_w + rot.apply(p_c - hinge_w), rot * r_c)
        step(5.0)
        a = angle(n)
        good = abs(a) < 2.0 if expect == "closed" else a > 170
        report(good, f"{n}: released at {start} deg -> {a:6.1f} deg (expect {expect})")
p_b, _ = pose(body)
report(np.linalg.norm(p_b - p_b0) < 0.01, f"flap tests moved the box {np.linalg.norm(p_b - p_b0) * 1e3:.1f} mm")

# 3. push
p_b0, r_b0 = pose(body)
body.set_linear_velocity(np.array([0.3, 0.0, 0.0]))
step(2.0)
p_b, r_b = pose(body)
tilt = np.rad2deg(np.arccos(np.clip(r_b.apply([0, 0, 1])[2], -1, 1)))
slid = np.linalg.norm(p_b[:2] - p_b0[:2])
v = np.linalg.norm(body.get_linear_velocity())
report(0.005 < slid < 0.1 and tilt < 3 and v < 0.01 and abs(p_b[2] - p_b0[2]) < 0.005,
       f"push 0.3 m/s: slid {slid * 100:.1f} cm, tilt {tilt:.1f} deg, final speed {v * 1e3:.1f} mm/s, "
       f"height change {(p_b[2] - p_b0[2]) * 1e3:+.1f} mm")

# 4. penetration (what a stiff arm pressing into the box does): 1 cm into the table -> ejection speed
p_b, r_b = pose(body)
for pr in [body] + [f[0] for f in flaps.values()]:
    p, r = pose(pr)
    set_pose(pr, p - [0, 0, 0.01], r)
peak, last = 0.0, pose(body)[0]
for _ in range(60):  # TGS removes penetration as a position bias, so measure the speed from the motion
    sim.step(render=False)
    now = pose(body)[0]
    peak, last = max(peak, float(np.linalg.norm(now - last)) * 60), now
step(2.0)
p_e, _ = pose(body)
report(peak <= 0.6 and abs(p_e[2] - p_b[2]) < 0.003, f"1 cm penetration: peak ejection speed {peak:.2f} m/s "
       f"(cap 0.5), settles {(p_e[2] - p_b[2]) * 1e3:+.1f} mm from rest height")

# 5. wrist camera
report(camera_test() < 0.5, "wrist camera tilt stays put (< 0.5 deg peak-to-peak)")
print(f"[check] {'ALL PASS' if ok else 'SOME CHECKS FAILED'}", flush=True)
app.close()
