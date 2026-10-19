"""Open the dual igus ReBeL rig in Isaac Sim 5.1 and bridge its joints to ROS 2.

Topics (same as compal_robot's isaac_topic ros2_control setup, see README.md):
  /dual_arm/isaac_joint_commands  sensor_msgs/JointState  (subscribed -> position targets)
  /dual_arm/isaac_joint_states    sensor_msgs/JointState  (published  <- simulated joints)
  /clock

Cameras are off by default. --cameras publishes head (gemini336L_rgb) + both wrist cameras as RGB at
--cam-width x --cam-height on /dual_arm/{head,wrist_left,wrist_right}/rgb (for record_dataset.py).
--upstream-bridge uses compal_robot's dual_arm_rig_ros2_bridge.build_bridge() instead.

Launch via run_dual_arm_sim.sh (sets up Isaac Sim's bundled ROS 2 Humble libraries):
  robot_remote/dual_arm_sim/run_dual_arm_sim.sh [--headless] [--rig v1|v2] [--cameras]
"""

import argparse
import functools
import sys
from pathlib import Path

print = functools.partial(print, flush=True)  # noqa: A001  (Kit exits without flushing stdout)

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--rig", choices=["v1", "v2"], default="v2")
parser.add_argument("--cameras", nargs="*", choices=["head", "wrist_left", "wrist_right"], default=None,
                    help="publish RGB on /dual_arm/<cam>/rgb; '--cameras' alone = all three, or list a subset")
parser.add_argument("--cam-width", type=int, default=640)
parser.add_argument("--cam-height", type=int, default=480)
parser.add_argument("--upstream-bridge", action="store_true",
                    help="use compal_robot's dual_arm_rig_ros2_bridge.py instead (1280x800 head + depth + box TF)")
parser.add_argument("--self-test", action="store_true", help="drive a few joints, print tracking, exit")
parser.add_argument("--zero-pose", action="store_true", help="start the arms at q = 0 (straight up) instead of the home pose")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

HERE = Path(__file__).resolve().parent
ISAAC_VERSION = (Path(sys.executable).resolve().parents[3] / "VERSION").read_text().strip()
RIG = "dual_arm_rig_v2" if args.rig == "v2" else "dual_arm_rig"
# The rig was authored for Isaac Sim 6.1. On 5.x use the *_isaac51.usda layer, which re-roots the
# nested arm rigid bodies (see make_isaac51_layer.py); otherwise 5.x only sees lift + sliders.
USD = HERE / (f"{RIG}_isaac51.usda" if ISAAC_VERSION.startswith("5.") else f"{RIG}.usda")
CMD_TOPIC = "/dual_arm/isaac_joint_commands"
STATE_TOPIC = "/dual_arm/isaac_joint_states"

enable_extension("isaacsim.ros2.bridge")
if not args.headless:
    # python.sh apps don't load the graph editor; isaac-sim.sh does. Adds Window > Graph Editors > Action Graph.
    enable_extension("omni.graph.window.action")
simulation_app.update()

omni.usd.get_context().open_stage(str(USD))
while omni.usd.get_context().get_stage_loading_status()[2] > 0:
    simulation_app.update()
stage = omni.usd.get_context().get_stage()
root = next(p.GetPath().pathString for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI))
print(f"[dual_arm] Isaac Sim {ISAAC_VERSION}: opened {USD.name}, articulation root {root}")
# A stage saved from a running sim (File > Save) also stores the graphs built below; Controller.edit then fails
# ("A graph already exists") and the saved copy would keep its old settings. Rebuild them (in memory only).
for graph in ("/World/DualArmJointBridge", "/World/DualArmCameraBridge"):
    if stage.GetPrimAtPath(graph).IsValid():
        stage.RemovePrim(graph)
        print(f"[dual_arm] removed the {graph} graph saved in {USD.name}; rebuilding it")

sim = SimulationContext(physics_dt=1 / 120, rendering_dt=1 / 60, stage_units_in_meters=1.0)
sim.initialize_physics()
robot = SingleArticulation(root, name="dual_arm")
robot.initialize()
print(f"[dual_arm] {robot.num_dof} dofs: {robot.dof_names}")

if args.upstream_bridge:
    sys.path.insert(0, str(HERE))
    import dual_arm_rig_ros2_bridge  # upstream graph: joints + cameras + box TF

    dual_arm_rig_ros2_bridge.build_bridge()
else:
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/DualArmJointBridge", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("simtime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("sub", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ctrl", "isaacsim.core.nodes.IsaacArticulationController"),
                ("pub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.SET_VALUES: [
                ("sub.inputs:topicName", CMD_TOPIC),
                # The node takes one queued message per tick. With the default depth 10, a render rate under the
                # teleop's 30 Hz piles commands up and the arms trail the hand by up to 10 commands; keep the newest.
                ("sub.inputs:queueSize", 1),
                ("ctrl.inputs:robotPath", root),
                ("pub.inputs:topicName", STATE_TOPIC),
                ("pub.inputs:targetPrim", root),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "clock.inputs:execIn"),
                ("tick.outputs:tick", "sub.inputs:execIn"),
                ("tick.outputs:tick", "ctrl.inputs:execIn"),
                ("tick.outputs:tick", "pub.inputs:execIn"),
                ("context.outputs:context", "clock.inputs:context"),
                ("context.outputs:context", "sub.inputs:context"),
                ("context.outputs:context", "pub.inputs:context"),
                ("simtime.outputs:simulationTime", "clock.inputs:timeStamp"),
                ("simtime.outputs:simulationTime", "pub.inputs:timeStamp"),
                ("sub.outputs:jointNames", "ctrl.inputs:jointNames"),
                ("sub.outputs:positionCommand", "ctrl.inputs:positionCommand"),
            ],
        },
    )
print(f"[dual_arm] ROS 2: subscribe {CMD_TOPIC}, publish {STATE_TOPIC} + /clock")

CAMERA_TOPICS = {"head": "/dual_arm/head/rgb", "wrist_left": "/dual_arm/wrist_left/rgb",
                 "wrist_right": "/dual_arm/wrist_right/rgb"}
if args.cameras is not None and not args.upstream_bridge:
    selected = args.cameras or ["head", "wrist_left", "wrist_right"]
    # RGB only, one render product per camera at --cam-width x --cam-height (images stamped with sim time).
    def find_camera(name, under=None):
        for p in stage.Traverse():
            path = p.GetPath().pathString
            if p.GetTypeName() == "Camera" and p.GetName() == name and (under is None or f"/{under}/" in path):
                return path
        raise RuntimeError(f"camera prim {name!r} (under {under}) not found")

    finders = {"head": lambda: find_camera("gemini336L_rgb"),
               "wrist_left": lambda: find_camera("wrist_camera", "arm_left"),
               "wrist_right": lambda: find_camera("wrist_camera", "arm_right")}
    cams = {k: finders[k]() for k in selected}
    nodes = [("tick", "omni.graph.action.OnPlaybackTick"), ("context", "isaacsim.ros2.bridge.ROS2Context")]
    values, connects = [], []
    for key, prim in cams.items():
        nodes += [(f"{key}_rp", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                  (f"{key}_pub", "isaacsim.ros2.bridge.ROS2CameraHelper")]
        values += [(f"{key}_rp.inputs:cameraPrim", [prim]),
                   (f"{key}_rp.inputs:width", args.cam_width), (f"{key}_rp.inputs:height", args.cam_height),
                   (f"{key}_pub.inputs:type", "rgb"), (f"{key}_pub.inputs:topicName", CAMERA_TOPICS[key]),
                   (f"{key}_pub.inputs:frameId", f"{key}_camera")]
        connects += [("tick.outputs:tick", f"{key}_rp.inputs:execIn"),
                     (f"{key}_rp.outputs:execOut", f"{key}_pub.inputs:execIn"),
                     (f"{key}_rp.outputs:renderProductPath", f"{key}_pub.inputs:renderProductPath"),
                     ("context.outputs:context", f"{key}_pub.inputs:context")]
    og.Controller.edit({"graph_path": "/World/DualArmCameraBridge", "evaluator_name": "execution"},
                       {og.Controller.Keys.CREATE_NODES: nodes, og.Controller.Keys.SET_VALUES: values,
                        og.Controller.Keys.CONNECT: connects})
    for key, prim in cams.items():
        print(f"[dual_arm] camera {key:11s} {prim} -> {CAMERA_TOPICS[key]} ({args.cam_width}x{args.cam_height} rgb)")

sim.play()

if not args.zero_pose:
    # Start both arms in the natural working pose instead of straight up (q = 0, a singular pose).
    # Keep in sync with quest_teleop/quest_arm_teleop.py HOME_DEG.
    home_deg = [0.0, 0.0, 65.0, 0.0, 50.0, 0.0]
    names = robot.dof_names
    idx = np.array([names.index(f"arm_{s}_joint{i + 1}") for s in ("left", "right") for i in range(6)])
    q_home = np.deg2rad(home_deg * 2)
    robot.set_joint_positions(q_home, joint_indices=idx)
    robot.apply_action(ArticulationAction(joint_positions=q_home, joint_indices=idx))
    print(f"[dual_arm] arms set to home pose {home_deg} deg")

if args.self_test:
    names = robot.dof_names
    tests = {"lift_joint": 0.10, "arm_left_joint2": np.deg2rad(-30), "arm_right_joint3": np.deg2rad(45)}
    idx = [names.index(n) for n in tests]
    robot.apply_action(ArticulationAction(joint_positions=np.array(list(tests.values())), joint_indices=np.array(idx)))
    for _ in range(600):  # 5 s
        sim.step(render=True)  # render=True also runs app.update(), which ticks the ROS 2 OmniGraph
    q = robot.get_joint_positions()
    for n, i in zip(tests, idx, strict=True):
        print(f"[dual_arm] self-test {n}: target {tests[n]:+.3f} reached {q[i]:+.3f}")
    simulation_app.close()
    sys.exit(0)

print("[dual_arm] running - Ctrl+C to stop")
try:
    while simulation_app.is_running():
        sim.step(render=True)  # render=True also runs app.update(), which ticks the ROS 2 OmniGraph
except KeyboardInterrupt:
    pass
simulation_app.close()
