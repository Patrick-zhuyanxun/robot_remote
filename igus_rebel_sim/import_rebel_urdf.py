"""Convert the igus ReBeL 6DOF URDF (generated from iRC_ROS xacro) into a USD for Isaac Sim 5.1.

Run with Isaac Sim's bundled python:
  ~/isaac-sim/python.sh robot_remote/igus_rebel_sim/import_rebel_urdf.py
"""

from pathlib import Path

import functools as _ft

print = _ft.partial(print, flush=True)  # noqa: A001

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402
from isaacsim.asset.importer.urdf import _urdf  # noqa: E402

HERE = Path(__file__).resolve().parent
URDF = HERE / "rebel_6dof.urdf"
USD = HERE / "rebel_6dof.usd"

_, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = False  # keep flange / tcp / tool0 frames
cfg.fix_base = True
cfg.make_default_prim = True
cfg.create_physics_scene = True
cfg.self_collision = False
cfg.import_inertia_tensor = True
cfg.distance_scale = 1.0
cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
cfg.default_drive_strength = 1e5
cfg.default_position_drive_damping = 1e3

_, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=str(URDF),
    import_config=cfg,
    dest_path=str(USD),
    get_articulation_root=True,
)
print(f"[import] articulation root: {prim_path}")

# The 5.1 importer ignores default_drive_strength/damping for position drives and authors
# stiffness=625, damping=0, which never settles. Override the gains on the root layer.
# USD angular drive units are per degree: torque = stiffness * err_deg + damping * err_vel_deg.
from pxr import Usd, UsdPhysics  # noqa: E402

DRIVE_STIFFNESS = 400.0  # N·m/deg
DRIVE_DAMPING = 40.0  # N·m·s/deg
DRIVE_MAX_FORCE = 1000.0  # N·m

stage = Usd.Stage.Open(str(USD))
stage.SetEditTarget(stage.GetRootLayer())
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.RevoluteJoint):
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
        drive.CreateDampingAttr().Set(DRIVE_DAMPING)
        drive.CreateMaxForceAttr().Set(DRIVE_MAX_FORCE)
        print(f"[import] drive gains set on {prim.GetPath()}")
stage.GetRootLayer().Save()
print(f"[import] wrote {USD}")
simulation_app.close()
