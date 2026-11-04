"""TracingLog (the teleop's logger wrapper) with the REAL rclpy logger: a throttled warning after a plain one must not
raise. Before the fix every message went through one rclpy call site, so the first throttled warning ("arms X cm
apart ... holding") raised 'Requested logging filters cannot be changed between calls' and killed the real teleop
(5 sessions on 2026-11-03). No ROS node, no DDS traffic."""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402,F401  - FIRST: never share the robot's ROS graph
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "quest_teleop"))
import rclpy.logging  # noqa: E402
import quest_dual_arm_teleop as qt  # noqa: E402


class Notes:
    def __init__(self):
        self.notes = []

    def note(self, msg):
        self.notes.append(msg)


class TracingLogTests(unittest.TestCase):
    def test_throttled_after_plain_does_not_raise_and_throttles(self):
        trace = Notes()
        log = qt.TracingLog(rclpy.logging.get_logger("tracing_log_test"), trace)
        log.warning("plain warning")  # the 'REAL ARMS ...' warning at start-up
        log.info("plain info")
        emitted = [log.warning("arms 4.9 cm apart - holding", throttle_duration_sec=0.3) for _ in range(5)]
        self.assertEqual([e is not False for e in emitted], [True, False, False, False, False])
        time.sleep(0.35)
        self.assertIsNot(log.warning("arms 4.9 cm apart - holding", throttle_duration_sec=0.3), False)
        log.warning("plain warning again")
        log.error("error", throttle_duration_sec=1.0)
        log.info("info", throttle_duration_sec=1.0)
        self.assertEqual(trace.notes.count("arms 4.9 cm apart - holding"), 2)

    def test_each_call_site_is_throttled_separately(self):
        log = qt.TracingLog(rclpy.logging.get_logger("tracing_log_test2"), Notes())
        a = log.warning("a", throttle_duration_sec=10.0)
        b = log.warning("b", throttle_duration_sec=10.0)
        self.assertIsNot(a, False)
        self.assertIsNot(b, False)


if __name__ == "__main__":
    unittest.main()
