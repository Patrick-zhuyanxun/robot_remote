"""igus ROS driver (Igus_rebel, CRI) for one or both physical ReBeLs, each in its own namespace, for the Quest teleop.

  ros2 launch real_robot/dual_arm_driver.launch.py                  (from the robot_remote folder; both arms)
  ros2 launch real_robot/dual_arm_driver.launch.py arms:=left

Per arm <side> (IP from config/real_arms.json):
  /<side>/controller_manager, /<side>/igus_rebel (hardware), /<side>/joint_states,
  /<side>/rebel_arm_trajectory_controller/joint_trajectory   <- the teleop streams here (--backend real-ros)
  robot_state_publisher with frame_prefix <side>/ (TF for RViz)
A few seconds after start each arm's motors are enabled (/<side>/igus_rebel/reset_and_enable); the arm then holds.
Same nodes and parameters as Igus_rebel's rebel.launch.py (whose spawners use an absolute /controller_manager and
therefore cannot be namespaced). Runs on the private local ROS graph of ros_env.sh (ROS_DOMAIN_ID 42, localhost).
"""
import json
import os
import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

ROOT = Path(__file__).resolve().parents[1]


def setup(context):
    arms = [a.strip() for a in LaunchConfiguration("arms").perform(context).split(",") if a.strip()]
    proto = LaunchConfiguration("hardware_protocol").perform(context)
    enable = LaunchConfiguration("enable_motors").perform(context).lower() == "true"
    ips = json.loads((ROOT / "config" / "real_arms.json").read_text())
    desc_pkg = Path(get_package_share_directory("igus_rebel_description"))
    ctrl_src = Path(get_package_share_directory("igus_rebel")) / "config" / (
        "controller.yaml" if proto == "rebel" else "controller_mock.yaml")
    # The upstream file keys its sections by bare node names (controller_manager, rebel_arm_trajectory_controller),
    # which only match nodes in the root namespace: re-key them as /**/<name> so /left/... and /right/... match.
    ctrl = yaml.safe_load(ctrl_src.read_text())
    ctrl_cfg = Path(tempfile.mkstemp(prefix="rebel_controllers_", suffix=".yaml")[1])
    ctrl_cfg.write_text(yaml.safe_dump({f"/**/{k}": v for k, v in ctrl.items()}))
    actions = []
    for side in arms:
        ip = LaunchConfiguration(f"{side}_ip").perform(context) or ips[side]["host"]
        description = {"robot_description": ParameterValue(Command([
            FindExecutable(name="xacro"), " ", str(desc_pkg / "urdf" / "igus_rebel_xeg32_wrapper.urdf.xacro"),
            " hardware_protocol:=", proto, " robot_ip:=", ip]), value_type=str)}
        cm = f"/{side}/controller_manager"
        actions.append(GroupAction([
            PushRosNamespace(side),
            Node(package="controller_manager", executable="ros2_control_node", parameters=[description, str(ctrl_cfg)],
                 output="both"),
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[description, {"frame_prefix": f"{side}/"}], output="both"),
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster", "--controller-manager", cm], output="both"),
            Node(package="controller_manager", executable="spawner",
                 arguments=["rebel_arm_trajectory_controller", "--controller-manager", cm], output="both"),
        ]))
        actions.append(LogInfo(msg=f"[driver] {side} arm: " + (f"robot {ip}:3920" if proto == "rebel" else proto)))
        if proto == "rebel" and enable:
            # The driver sends Reset + Enable when it connects, but that first attempt has not stuck on our arms;
            # re-send it once the driver is up (`ros2 service call` waits for the service, prints success=...).
            actions.append(TimerAction(period=6.0, actions=[
                LogInfo(msg=f"[driver] enabling the {side} arm's motors (reset_and_enable) - the arm will hold"),
                ExecuteProcess(cmd=["ros2", "service", "call", f"/{side}/igus_rebel/reset_and_enable",
                                    "std_srvs/srv/Trigger"], output="screen")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        # same private graph as ros_env.sh (the Quest teleop and RViz use it): this PC only
        SetEnvironmentVariable("ROS_DOMAIN_ID", os.environ.get("ROBOT_REMOTE_DOMAIN_ID", "42")),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        DeclareLaunchArgument("arms", default_value="left,right", description="left, right or left,right"),
        DeclareLaunchArgument("enable_motors", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("left_ip", default_value="", description="default: config/real_arms.json"),
        DeclareLaunchArgument("right_ip", default_value="", description="default: config/real_arms.json"),
        DeclareLaunchArgument("hardware_protocol", default_value="rebel", choices=["rebel", "mock_hardware"]),
        OpaqueFunction(function=setup),
    ])
