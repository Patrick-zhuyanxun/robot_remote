"""Load the ReBeL USD in Isaac Sim, drive each joint to a target and report tracking.

  ~/isaac-sim/python.sh robot_remote/igus_rebel_sim/check_rebel_sim.py           # headless
  ~/isaac-sim/python.sh robot_remote/igus_rebel_sim/check_rebel_sim.py --gui     # with viewport
"""

import functools
import sys

print = functools.partial(print, flush=True)  # noqa: A001  (Kit exits without flushing stdout)
from pathlib import Path

from isaacsim import SimulationApp

GUI = "--gui" in sys.argv
simulation_app = SimulationApp({"headless": not GUI})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

USD = Path(__file__).resolve().parent / "rebel_6dof.usd"

world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 60)
world.scene.add_default_ground_plane()
add_reference_to_stage(str(USD), "/World/rebel")
robot = world.scene.add(SingleArticulation("/World/rebel", name="rebel"))
world.reset()

print("[check] dof names:", robot.dof_names)
lower, upper = robot.dof_properties["lower"], robot.dof_properties["upper"]
print("[check] limits [deg]:", np.round(np.rad2deg(np.stack([lower, upper], 1)), 1).tolist())

targets_deg = [np.zeros(6), np.array([30, -30, 60, 20, 45, -30.0]), np.array([-45, -20, 60, -30, -60, 60.0]), np.array([0, 0, 0, 0, 80, 0.0]), np.array([0, 0, 0, 0, -80, 0.0])]
for tgt in targets_deg:
    robot.apply_action(__import__("isaacsim.core.utils.types", fromlist=["ArticulationAction"]).ArticulationAction(
        joint_positions=np.deg2rad(tgt)
    ))
    for _ in range(480):  # 4 s of sim time (URDF caps every joint at 45 deg/s, like the real ReBeL)
        world.step(render=GUI)
    q = np.rad2deg(robot.get_joint_positions())
    print(f"[check] target {tgt.round(1).tolist()} -> reached {q.round(1).tolist()}  max err {np.abs(q - tgt).max():.2f} deg")

if GUI:
    while simulation_app.is_running():
        world.step(render=True)
simulation_app.close()
