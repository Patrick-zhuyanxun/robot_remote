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
import os
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
parser.add_argument("--random-box", action="store_true",
                    help="place the (fixed) box at a random pose on the table at start; default: where the USD has it")
parser.add_argument("--box-seed", type=int, default=None, help="seed of the random box placement (default: random)")
parser.add_argument("--box-yaw-deg", type=float, default=30.0, help="random box yaw: +- this around the USD orientation")
parser.add_argument("--box-margin", type=float, default=0.02, help="box footprint stays this far inside the table edge [m]")
parser.add_argument("--box-min-offset", type=float, default=0.05,
                    help="box centre at least this far from the table centre (so there is something to move) [m]")
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



class BoxSpawner:
    """Random pose of /World/CardboardBox on the table (box task: see the box, move it to the table centre, open
    its four flaps). The box (Body + 4 flaps, see setup_box_task_scene.py) is placed closed, at rest, fully on
    the table top with its yaw within +-box_yaw_deg of the USD orientation (--random-box, before physics starts).
    The box body is kinematic (fixed); /dual_arm/reset_scene closes the flaps again."""

    ROOT, TABLE_TOP = "/World/CardboardBox", "/World/table/top"
    PARTS = ("Body", "LidBack", "LidFront", "LidLeft", "LidRight")

    def __init__(self, stage, args):
        from pxr import Gf, UsdGeom
        self.Gf, self.args = Gf, args
        self.rng = np.random.default_rng(args.box_seed)
        self.root = UsdGeom.Xformable(stage.GetPrimAtPath(self.ROOT))
        top = stage.GetPrimAtPath(self.TABLE_TOP)
        t, sc = np.array(top.GetAttribute("xformOp:translate").Get()), np.array(top.GetAttribute("xformOp:scale").Get())
        self.table_lo, self.table_hi = t[:2] - sc[:2] / 2, t[:2] + sc[:2] / 2
        self.table_c, self.z = t[:2], t[2] + sc[2] / 2 + 0.001        # 1 mm above the top: no start penetration
        q0 = stage.GetPrimAtPath(self.ROOT).GetAttribute("xformOp:orient").Get()
        self.yaw0 = 2 * np.arctan2(q0.GetImaginary()[2], q0.GetReal())  # USD yaw of the box frame
        body = stage.GetPrimAtPath(f"{self.ROOT}/Body")
        self.body_off = np.array(body.GetAttribute("xformOp:translate").Get())  # box centre in the box frame
        walls = [c for c in body.GetChildren() if c.GetName().endswith("Wall") or "Wall" in c.GetName()]
        self.half = np.max([np.abs(np.array(c.GetAttribute("xformOp:translate").Get())[:2])
                            + np.array(c.GetAttribute("xformOp:scale").Get())[:2] / 2 for c in walls], axis=0)
        self.parts = {}
        for name in self.PARTS:
            pr = stage.GetPrimAtPath(f"{self.ROOT}/{name}")
            if pr.IsValid():
                o = pr.GetAttribute("xformOp:orient").Get() if pr.HasAttribute("xformOp:orient") else None
                self.parts[name] = (np.array(pr.GetAttribute("xformOp:translate").Get()),
                                    np.array([o.GetReal(), *o.GetImaginary()]) if o else np.array([1.0, 0, 0, 0]))
        self.prims = None

    def sample(self):
        """(centre xy, yaw) of a box footprint inside the table, at least box_min_offset from its centre."""
        a = self.args
        for _ in range(10000):
            c = self.rng.uniform(self.table_lo, self.table_hi)
            yaw = self.yaw0 + np.deg2rad(self.rng.uniform(-a.box_yaw_deg, a.box_yaw_deg))
            r = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            corners = c + (r @ (np.array([[1, 1], [1, -1], [-1, 1], [-1, -1]]) * self.half).T).T
            if (np.all(corners >= self.table_lo + a.box_margin) and np.all(corners <= self.table_hi - a.box_margin)
                    and np.linalg.norm(c - self.table_c) >= a.box_min_offset):
                return c, yaw
        raise RuntimeError("no box pose fits on the table with these --box-* settings")

    def frame(self, c, yaw):
        """Box-frame origin for a box centre c: the Body sits at body_off in that frame."""
        r = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        return np.array([*(c - r @ self.body_off[:2]), self.z])

    def place_usd(self):
        c, yaw = self.sample()
        Gf = self.Gf
        self.root.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(*self.frame(c, yaw)))
        self.root.GetPrim().GetAttribute("xformOp:orient").Set(Gf.Quatf(float(np.cos(yaw / 2)), 0, 0, float(np.sin(yaw / 2))))
        return c, yaw

    def _rigid(self):
        from isaacsim.core.prims import SingleRigidPrim
        if self.prims is None:
            self.prims = {n: SingleRigidPrim(f"{self.ROOT}/{n}") for n in self.parts}
            for pr in self.prims.values():
                pr.initialize()
        return self.prims

    def body_pose(self):
        """World pose of the box Body: position, quaternion (w, x, y, z)."""
        return self._rigid()["Body"].get_world_pose()

    def current(self):
        """(centre xy, yaw) of the box as the USD places it now."""
        t = np.array(self.root.GetPrim().GetAttribute("xformOp:translate").Get())
        q = self.root.GetPrim().GetAttribute("xformOp:orient").Get()
        yaw = 2 * np.arctan2(q.GetImaginary()[2], q.GetReal())
        r = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        return t[:2] + r @ self.body_off[:2], yaw

    def close_flaps(self):
        """While simulating: put the four flaps back closed and at rest (the box body is kinematic and stays)."""
        from scipy.spatial.transform import Rotation
        self._rigid()
        c, yaw = self.current()
        r_box = Rotation.from_euler("z", yaw)
        o = self.frame(c, yaw)
        for name, (t, q_wxyz) in self.parts.items():
            if name == "Body":
                continue
            q = (r_box * Rotation.from_quat([*q_wxyz[1:], q_wxyz[0]])).as_quat()
            pr = self.prims[name]
            pr.set_world_pose(position=o + r_box.apply(t), orientation=np.array([q[3], q[0], q[1], q[2]]))
            pr.set_linear_velocity(np.zeros(3))
            pr.set_angular_velocity(np.zeros(3))


box = None
if stage.GetPrimAtPath(BoxSpawner.ROOT).IsValid():
    box = BoxSpawner(stage, args)
    c, yaw = box.place_usd() if args.random_box else box.current()
    print(f"[dual_arm] box fixed at ({c[0]:+.3f}, {c[1]:+.3f}) m, yaw {np.rad2deg(yaw):.0f} deg"
          + (" (random)" if args.random_box else "") + " - only its 4 flaps move; "
          "publish std_msgs/Empty on /dual_arm/reset_scene to close them again")

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

# /dual_arm/reset_scene (std_msgs/Empty): close the four flaps again, e.g. between recorded episodes;
# /dual_arm/box_pose (geometry_msgs/PoseStamped): where the box is now
reset_node, reset_req = None, []
if box is not None:
    try:
        import rclpy
        from std_msgs.msg import Empty

        rclpy.init()
        reset_node = rclpy.create_node("dual_arm_scene")
        reset_node.create_subscription(Empty, "/dual_arm/reset_scene", lambda _m: reset_req.append(1), 1)
        from geometry_msgs.msg import PoseStamped

        box_pub = reset_node.create_publisher(PoseStamped, "/dual_arm/box_pose", 10)  # Body centre, frame world
    except Exception as exc:  # noqa: BLE001  (sim still runs, only the reset topic is missing)
        print(f"[dual_arm] WARNING: no /dual_arm/reset_scene ({exc})")



class SimTiming:
    """[dual_arm] [timing] line every 5 s: frames/s, frame time, sim speed (sim time / wall time) and the CPU the
    main thread ran on. Isaac is limited by this one thread; sim speed < 100 % = the arms move in slow motion."""

    def __init__(self, period=5.0):
        import time as _t
        self.t, self.period = _t, period
        self.t0, self.sim0, self.frames, self.dts = _t.perf_counter(), sim.current_time, 0, []

    def frame(self, dt):
        self.frames += 1
        self.dts.append(dt)
        now = self.t.perf_counter()
        if now - self.t0 < self.period:
            return
        wall, simt = now - self.t0, sim.current_time - self.sim0
        try:
            cpu = Path("/proc/thread-self/stat").read_text().rsplit(")", 1)[1].split()[36]
        except (OSError, IndexError):
            cpu = "?"
        print(f"[dual_arm] [timing] {self.frames / wall:5.1f} frames/s | frame p50 {np.percentile(self.dts, 50) * 1e3:5.1f} "
              f"p95 {np.percentile(self.dts, 95) * 1e3:5.1f} max {max(self.dts) * 1e3:6.1f} ms | sim {simt / wall * 100:3.0f} % "
              f"real time | main thread on CPU {cpu} (allowed {len(os.sched_getaffinity(0))} CPUs)")
        self.t0, self.sim0, self.frames, self.dts = now, sim.current_time, 0, []


timing = SimTiming()
print("[dual_arm] running - Ctrl+C to stop")
try:
    while simulation_app.is_running():
        t_frame = timing.t.perf_counter()
        sim.step(render=True)  # render=True also runs app.update(), which ticks the ROS 2 OmniGraph
        timing.frame(timing.t.perf_counter() - t_frame)
        if reset_node is not None:
            rclpy.spin_once(reset_node, timeout_sec=0.0)
            if reset_req:
                reset_req.clear()
                box.close_flaps()
                print("[dual_arm] box flaps closed")
            pos, quat = box.body_pose()
            msg = PoseStamped()
            msg.header.frame_id = "world"
            msg.header.stamp = reset_node.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pos)
            msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = map(float, quat)
            box_pub.publish(msg)
except KeyboardInterrupt:
    pass
simulation_app.close()
