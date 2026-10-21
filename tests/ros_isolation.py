"""Import FIRST in every test module that touches ROS (before rclpy / the teleop are imported or initialised).

Tests start real teleop nodes with real-arm backends against fake drivers. On the robot's ROS graph (ros_env.sh:
ROS_DOMAIN_ID 42, localhost) their trajectory commands reach a live igus driver: on 2026-10-21 a test run moved the
real left arm while the operator's session was up. So every test process is forced onto its own domain, and
subprocesses started through ros_env.sh (ROBOT_REMOTE_DOMAIN_ID) inherit it.
"""
import os

TEST_DOMAIN = "97"
ROBOT_DOMAINS = {"42", "91", "11", "0"}  # robot_remote, rviz_pretest, legacy real arm, ROS default

os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN
os.environ["ROBOT_REMOTE_DOMAIN_ID"] = TEST_DOMAIN
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["ROBOT_REMOTE_TRACE"] = ""  # never overwrite the operator's latest_run/ trace
os.environ.pop("ROBOT_REMOTE_TARGET", None)  # ros_env.sh: "real" would switch to the LAN robot domain


def check(domain_id):
    """Raise if a test context ended up on a robot domain."""
    if str(domain_id) in ROBOT_DOMAINS:
        raise RuntimeError(f"test ROS context on robot domain {domain_id} - refusing (see tests/ros_isolation.py)")
