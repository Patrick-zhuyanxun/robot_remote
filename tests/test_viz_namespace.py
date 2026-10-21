"""RViz helper (run_teleop_viz.sh --source real) must never publish on the igus driver's /left|right/joint_states:
the teleop reads the measured joints there, and the relayed copy fed it its own stale joints (2026-10-21: frozen 0.7 s,
then 7 deg overshoot after a release). It uses its own viz_left / viz_right namespace and frames."""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402,F401  - FIRST: never share the robot's ROS graph
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
DOMAIN = 89
try:
    import rclpy
    from rclpy.duration import Duration
    from sensor_msgs.msg import JointState
    import tf2_ros
except ImportError:  # pragma: no cover
    rclpy = None


@unittest.skipIf(rclpy is None, "source ROS (and igus_ws) for this test")
class VizNamespaceTest(unittest.TestCase):
    def test_real_view_keeps_off_the_driver_topics(self):
        env = {**os.environ, "ROBOT_REMOTE_DOMAIN_ID": str(DOMAIN), "NO_RVIZ": "1"}
        viz = subprocess.Popen([str(ROOT / "quest_teleop/run_teleop_viz.sh"), "--source", "real"], env=env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        ctx = rclpy.Context()
        rclpy.init(context=ctx, domain_id=DOMAIN)
        node = rclpy.create_node("viz_ns_probe", context=ctx)
        ex = rclpy.executors.SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        try:
            pub = node.create_publisher(JointState, "/quest_teleop/joint_states", 10)
            got = {"viz": 0, "driver_topic": 0}
            node.create_subscription(JointState, "/viz_left/joint_states", lambda m: got.__setitem__("viz", got["viz"] + 1), 10)
            node.create_subscription(JointState, "/left/joint_states",
                                     lambda m: got.__setitem__("driver_topic", got["driver_topic"] + 1), 10)
            buf = tf2_ros.Buffer()
            tf2_ros.TransformListener(buf, node, spin_thread=False)
            names = [f"arm_{s}_joint{i}" for s in ("left", "right") for i in range(1, 7)]
            end, ok = time.monotonic() + 40, False
            while time.monotonic() < end:
                msg = JointState(name=names, position=[0.0, 0.2, 1.1, 0.0, 0.9, 0.0] * 2)
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
                ex.spin_once(timeout_sec=0.05)
                if got["viz"] > 5 and buf.can_transform("world", "viz_left/link6", rclpy.time.Time()):
                    ok = True
                    break
            self.assertTrue(ok, f"viz arm / TF not up: {got}")
            for _ in range(20):
                ex.spin_once(timeout_sec=0.05)
            self.assertEqual(got["driver_topic"], 0)
            self.assertEqual(node.count_publishers("/left/joint_states"), 0)
            self.assertEqual(node.count_publishers("/right/joint_states"), 0)
            self.assertFalse(buf.can_transform("world", "left/link6", rclpy.time.Time(), timeout=Duration(seconds=0.2)))
        finally:
            os.killpg(viz.pid, signal.SIGINT)
            try:
                viz.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(viz.pid, signal.SIGKILL)
            node.destroy_node()
            rclpy.shutdown(context=ctx)


if __name__ == "__main__":
    unittest.main()
