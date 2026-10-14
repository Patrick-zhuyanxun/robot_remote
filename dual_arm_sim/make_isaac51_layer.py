"""Make the compal_robot dual-arm USD usable in Isaac Sim 5.1.

The upstream rig (authored for Isaac Sim 6.1) nests every arm rigid body under its parent body
(base_link/motor1/link1/...). Isaac Sim 5.1's PhysX only treats a nested rigid body as its own body
when it resets the xform stack, so it reports "CreateJoint - no bodies defined" for every arm joint
and the articulation ends up with only lift + sliders (3 DOF).

This writes <rig>_isaac51.usda: a thin layer that sublayers the untouched upstream file and, for
every rigid body nested under another rigid body, authors its world pose + !resetXformStack!.
Geometry, joints and drives are not touched.

  ~/isaac-sim/python.sh make_isaac51_layer.py [dual_arm_rig_v2.usda]
"""

import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})  # needed before pxr is importable

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

HERE = Path(__file__).resolve().parent
src = HERE / (sys.argv[1] if len(sys.argv) > 1 else "dual_arm_rig_v2.usda")
dst = src.with_name(src.stem + "_isaac51.usda")

stage = Usd.Stage.Open(str(src))
cache = UsdGeom.XformCache(Usd.TimeCode.Default())


def is_body(p: Usd.Prim) -> bool:
    return p.HasAPI(UsdPhysics.RigidBodyAPI)


nested = []
for prim in stage.Traverse():
    if not is_body(prim):
        continue
    parent = prim.GetParent()
    while parent and not parent.IsPseudoRoot():
        if is_body(parent):
            nested.append(prim)
            break
        parent = parent.GetParent()

layer = Sdf.Layer.CreateNew(str(dst))
layer.subLayerPaths.append(f"./{src.name}")
out = Usd.Stage.Open(layer)
out.SetEditTarget(layer)
out.SetDefaultPrim(out.GetPrimAtPath(stage.GetDefaultPrim().GetPath()))

for prim in nested:
    world = Gf.Transform(cache.GetLocalToWorldTransform(prim))
    over = out.OverridePrim(prim.GetPath())
    xf = UsdGeom.Xformable(over)
    xf.ClearXformOpOrder()
    xf.SetResetXformStack(True)
    xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(world.GetTranslation()))
    xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(world.GetRotation().GetQuat()))
    xf.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(world.GetScale()))

layer.Save()
print(f"{len(nested)} nested rigid bodies re-rooted -> {dst.name}", flush=True)
simulation_app.close()
