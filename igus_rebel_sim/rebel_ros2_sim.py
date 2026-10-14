"""Isaac Sim ReBeL that follows ROS 2 joint commands (bridge for MoveIt / teleop testing).

Graph (runs every physics tick):
  ROS2SubscribeJointState(<cmd topic>) -> IsaacArticulationController(rebel)   [position targets]
  ROS2PublishJointState(/isaac_joint_states)                                    [what the sim reached]

With `ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware`, the mock
hardware echoes every executed trajectory point onto /joint_states, so subscribing to that topic
makes the simulated arm replay what MoveIt executes.

Launch through run_rebel_ros2_sim.sh (sets up Isaac Sim's internal ROS 2 Humble libraries):
  robot_remote/igus_rebel_sim/run_rebel_ros2_sim.sh [--headless] [--topic /joint_states]
"""

import argparse
import functools
from pathlib import Path

print = functools.partial(print, flush=True)  # noqa: A001  (Kit exits without flushing stdout)

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--topic", default="/joint_states", help="sensor_msgs/JointState command topic")
parser.add_argument("--feedback-topic", default="/isaac_joint_states")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import omni.graph.core as og  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

USD = Path(__file__).resolve().parent / "rebel_6dof.usd"
ROBOT_PRIM = "/World/rebel"
ARTICULATION_ROOT = f"{ROBOT_PRIM}/root_joint/root_joint"  # prim carrying PhysicsArticulationRootAPI

world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 60)
world.scene.add_default_ground_plane()
add_reference_to_stage(str(USD), ROBOT_PRIM)
# Initialise physics before building the graph so the articulation exists when nodes first run.
world.reset()

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/ROS2JointBridge", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("context", "isaacsim.ros2.bridge.ROS2Context"),
            ("sub", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ctrl", "isaacsim.core.nodes.IsaacArticulationController"),
            ("pub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("simtime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ],
        keys.SET_VALUES: [
            ("sub.inputs:topicName", args.topic),
            ("ctrl.inputs:robotPath", ARTICULATION_ROOT),
            ("pub.inputs:topicName", args.feedback_topic),
            ("pub.inputs:targetPrim", ARTICULATION_ROOT),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "sub.inputs:execIn"),
            ("tick.outputs:tick", "ctrl.inputs:execIn"),
            ("tick.outputs:tick", "pub.inputs:execIn"),
            ("context.outputs:context", "sub.inputs:context"),
            ("context.outputs:context", "pub.inputs:context"),
            ("sub.outputs:jointNames", "ctrl.inputs:jointNames"),
            ("sub.outputs:positionCommand", "ctrl.inputs:positionCommand"),
            ("simtime.outputs:simulationTime", "pub.inputs:timeStamp"),
        ],
    },
)

world.play()
print(f"[rebel_ros2_sim] following {args.topic}, publishing sim state on {args.feedback_topic}")
try:
    while simulation_app.is_running():
        world.step(render=True)  # render=True also runs app.update(), which ticks the ROS 2 OmniGraph
except KeyboardInterrupt:
    pass
simulation_app.close()
