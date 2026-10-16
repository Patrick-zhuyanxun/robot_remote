"""Pre-test of ONE physical igus ReBeL from RViz / MoveIt, before any Quest teleop. One command:

  ros2 launch real_robot/rviz_pretest.launch.py arm:=left            (from the robot_remote folder)

Starts the igus ROS driver for that arm (CRI, IP from config/real_arms.json), enables the motors
(/igus_rebel/reset_and_enable, a few seconds after start; the arm then holds its pose), and opens MoveIt + RViz.
Ctrl+C stops everything. The ROS graph stays on this PC (ROS_DOMAIN_ID 91, localhost only); the arm is
reached over CRI/TCP. Do not run it next to the Quest teleop or another driver of the same arm.

Arguments:
  arm:=left|right               which arm (default left)
  robot_ip:=<ip>                override the IP from config/real_arms.json
  enable_motors:=true|false     call reset_and_enable after start (default true)
  hardware_protocol:=mock_hardware   no robot: try the RViz workflow
  use_rviz:=true|false
"""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]


def setup(context):
    arm = LaunchConfiguration("arm").perform(context)
    proto = LaunchConfiguration("hardware_protocol").perform(context)
    ip = LaunchConfiguration("robot_ip").perform(context)
    if not ip:
        ip = json.loads((ROOT / "config" / "real_arms.json").read_text())[arm]["host"]
    real = proto == "rebel"
    enable = LaunchConfiguration("enable_motors").perform(context).lower() == "true"

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory("igus_rebel")) / "launch" / "rebel.launch.py")),
        launch_arguments={"hardware_protocol": proto, "robot_ip": ip}.items())
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory("igus_rebel_moveit_config"))
                                          / "launch" / "move_group.launch.py")),
        launch_arguments={"hardware_protocol": proto, "use_gui": LaunchConfiguration("use_rviz"),
                          "run_teleop": "false", "run_box_h_cut_executor": "false",
                          "run_camera_pole": "false"}.items())
    actions = [LogInfo(msg=f"[pretest] {arm} arm, " + (f"robot {ip}:3920" if real else f"{proto} (no robot)")
                       + ", ROS_DOMAIN_ID 91 localhost-only. Someone at the e-stop!"),
               driver, moveit]
    if real and enable:
        # The driver sends Reset + Enable when it connects, but on our arm that first attempt has not stuck;
        # re-send it once the driver is up. `ros2 service call` waits for the service, then prints success=...
        actions.append(TimerAction(period=6.0, actions=[
            LogInfo(msg="[pretest] enabling the motors (reset_and_enable) - the arm will hold its pose"),
            ExecuteProcess(cmd=["ros2", "service", "call", "/igus_rebel/reset_and_enable", "std_srvs/srv/Trigger"],
                           output="screen")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "91"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        DeclareLaunchArgument("arm", default_value="left", choices=["left", "right"]),
        DeclareLaunchArgument("robot_ip", default_value="", description="default: config/real_arms.json"),
        DeclareLaunchArgument("enable_motors", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("hardware_protocol", default_value="rebel", choices=["rebel", "mock_hardware"]),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=setup),
    ])
