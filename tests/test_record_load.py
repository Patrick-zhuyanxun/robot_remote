"""Recorder keeps 30 frames/s with fresh state with three 640x480 cameras, fast igus joint_states, and when CONTINUING
an existing dataset (2026-10-21: LeRobotDataset.resume() without image writer threads wrote the PNGs inside add_frame
-> ~8 frames/s, every frame late and its joint_states 0.39 s stale)."""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402,F401  - FIRST: never share the robot's ROS graph
import json
import os
import re
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path

import test_real_recording as trr

ROOT = Path(__file__).resolve().parents[1]
DOMAIN = 86
NOISE_VIDEO = ("import cv2, numpy as np, sys\n"
               "w = cv2.VideoWriter(sys.argv[1], cv2.VideoWriter_fourcc(*'mp4v'), 30, (640, 480))\n"
               "r = np.random.default_rng(int(sys.argv[2]))\n"
               "for i in range(30):\n    w.write(r.integers(0, 255, (480, 640, 3), np.uint8))\n"
               "w.release()\n")


@unittest.skipIf(trr.rclpy is None or trr.LEROBOT_PY is None, "needs ROS 2 rclpy and the LeRobot recorder Python")
class RecordUnderLoadTest(unittest.TestCase):
    def record_once(self, tmp, ev):
        """One recorder session with a ~3 s episode; returns its output lines."""
        env = {**os.environ, "ROBOT_REMOTE_DOMAIN_ID": str(DOMAIN), "LEROBOT_PY": trr.LEROBOT_PY}
        rec = subprocess.Popen([str(ROOT / "recording/run_record.sh"), "--source", "real", "--name", "ds", "--task", "t",
                                "--root", str(tmp), "--fps", "30", "--camera-config", str(tmp / "cams.json")],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        lines = []
        reader = threading.Thread(target=lambda: [lines.append(ln.rstrip()) for ln in rec.stdout], daemon=True)
        reader.start()

        def wait(text, timeout=90):
            end = time.monotonic() + timeout
            while time.monotonic() < end and not any(text in ln for ln in lines):
                self.assertIsNone(rec.poll(), "\n".join(lines[-20:]))
                time.sleep(0.05)
            self.assertTrue(any(text in ln for ln in lines), "\n".join(lines[-20:]))
        try:
            wait("[rec] controls")
            time.sleep(1.0)
            ev.publish(trr.String(data="toggle"))
            wait("RECORDING")
            time.sleep(3.0)
            ev.publish(trr.String(data="toggle"))
            wait("saved. total episodes")
            rec.stdin.write("q\n")
            rec.stdin.flush()
            rec.wait(timeout=60)
        finally:
            if rec.poll() is None:
                rec.kill()
            reader.join(timeout=5)
            rec.stdin.close()
            rec.stdout.close()
        return lines

    def test_three_cameras_create_and_resume_keep_30_fps(self):
        rclpy, JointState, String = trr.rclpy, trr.JointState, trr.String
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            cams = {}
            for i, k in enumerate(("wrist_left", "wrist_right", "head")):  # noise: slow to compress, like real images
                subprocess.run([trr.LEROBOT_PY, "-c", NOISE_VIDEO, str(tmp / f"{k}.mp4"), str(i)], check=True, timeout=60)
                cams[k] = {"device": str(tmp / f"{k}.mp4"), "width": 640, "height": 480, "fps": 30, "fourcc": None}
            (tmp / "cams.json").write_text(json.dumps(cams))
            ctx = rclpy.Context()
            rclpy.init(context=ctx, domain_id=DOMAIN)
            node = rclpy.create_node("fast_rig", context=ctx)
            js = {s: node.create_publisher(JointState, f"/{s}/joint_states", 10) for s in ("left", "right")}
            act = node.create_publisher(JointState, "/quest_teleop/action", 10)
            ev = node.create_publisher(String, "/quest_teleop/events", 10)
            stop = threading.Event()

            def publish():  # 200 Hz per driver + the ~30 Hz action
                k = 0
                while not stop.is_set():
                    for p in js.values():
                        p.publish(JointState(name=trr.ARM, position=trr.LEFT_Q))
                    if k % 7 == 0:
                        act.publish(JointState(name=trr.NAMES, position=trr.ACTION))
                    k += 1
                    time.sleep(0.005)
            th = threading.Thread(target=publish, daemon=True)
            th.start()
            try:
                for session in ("create", "resume"):
                    lines = self.record_once(tmp, ev)
                    saving = next(ln for ln in lines if "saving episode" in ln)
                    self.assertEqual(any("resuming existing dataset" in ln for ln in lines), session == "resume")
                    m = re.search(r"\((\d+) frames, ([\d.]+)s real, (\d+) frames had a stale stream", saving)
                    frames, stale = int(m.group(1)), int(m.group(3))
                    self.assertGreater(frames, 80, f"{session}: {saving}")  # ~3 s at 30 fps
                    self.assertLess(stale, 0.1 * frames, f"{session}: {saving}")
                    self.assertNotIn("late ticks", saving, session)
            finally:
                stop.set()
                th.join()
                node.destroy_node()
                rclpy.shutdown(context=ctx)


if __name__ == "__main__":
    unittest.main()
