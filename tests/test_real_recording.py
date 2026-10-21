"""Real-robot recording without hardware: record_dataset.py --source real (its own LeRobot Python, as run_record.sh
starts it) against fake igus-driver / teleop topics, with looping video files standing in for the USB cameras.
Runs on a private ROS domain so it cannot see (or feed) a real session."""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402,F401  - FIRST: never share the robot's ROS graph
import json
import os
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LEROBOT_PY = os.environ.get("LEROBOT_PY") or next(
    (str(p) for p in (ROOT / ".venv_record/bin/python", Path.home() / "Robotic_infra/lerobot/.venv/bin/python")
     if p.exists()), None)
DOMAIN = 87
try:
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
except ImportError:  # pragma: no cover
    rclpy = None

ARM = [f"joint{i}" for i in range(1, 7)]
NAMES = [f"arm_{s}_{j}" for s in ("left", "right") for j in ARM + ["gripper"]]
LEFT_Q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
RIGHT_ASSUMED = [0.0, -0.1, -0.2, -0.3, -0.4, -0.5]
ACTION = [0.01 * i for i in range(14)]


def make_video(path, colour):
    code = (f"import cv2, numpy as np\n"
            f"w = cv2.VideoWriter({str(path)!r}, cv2.VideoWriter_fourcc(*'mp4v'), 30, (320, 240))\n"
            f"for i in range(30):\n"
            f"    f = np.full((240, 320, 3), {colour}, np.uint8); f[:, i * 10:i * 10 + 10] = 255; w.write(f)\n"
            f"w.release()\n")
    subprocess.run([LEROBOT_PY, "-c", code], check=True, timeout=60)


@unittest.skipIf(rclpy is None or LEROBOT_PY is None, "needs ROS 2 rclpy and the LeRobot recorder Python")
class RealRecordingTest(unittest.TestCase):
    def test_wall_clock_episode_with_cameras_fallback_arm_and_reset(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            make_video(tmp / "left.mp4", [0, 0, 200])
            make_video(tmp / "head.mp4", [0, 200, 0])
            cfg = {"_comment": "test", "wrist_left": {"device": str(tmp / "left.mp4"), "width": 320, "height": 240,
                                                      "fps": 30, "fourcc": None},
                   "head": {"device": str(tmp / "head.mp4"), "width": 320, "height": 240, "fps": 30, "fourcc": None}}
            (tmp / "cams.json").write_text(json.dumps(cfg))

            ctx = rclpy.Context()
            rclpy.init(context=ctx, domain_id=DOMAIN)
            node = rclpy.create_node("fake_real_rig", context=ctx)
            pubs = {"left": node.create_publisher(JointState, "/left/joint_states", 10),
                    "viz": node.create_publisher(JointState, "/quest_teleop/joint_states", 10),
                    "action": node.create_publisher(JointState, "/quest_teleop/action", 10),
                    "events": node.create_publisher(String, "/quest_teleop/events", 10)}
            stop = threading.Event()

            def publish():  # one-arm session: only the left driver runs; the teleop reports the right arm's pose
                while not stop.is_set():
                    pubs["left"].publish(JointState(name=ARM, position=LEFT_Q))
                    pubs["viz"].publish(JointState(name=[f"arm_{s}_{j}" for s in ("left", "right") for j in ARM],
                                                   position=[9.0] * 6 + RIGHT_ASSUMED))
                    pubs["action"].publish(JointState(name=NAMES, position=ACTION))
                    time.sleep(1 / 30)

            env = {**os.environ, "ROBOT_REMOTE_DOMAIN_ID": str(DOMAIN), "LEROBOT_PY": LEROBOT_PY}
            rec = subprocess.Popen([str(ROOT / "recording/run_record.sh"), "--source", "real", "--name", "ds",
                                    "--task", "cut the box", "--root", str(tmp), "--fps", "15",
                                    "--camera-config", str(tmp / "cams.json")],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, env=env)
            lines = []
            threading.Thread(target=lambda: [lines.append(ln.rstrip()) for ln in rec.stdout], daemon=True).start()
            pub_thread = threading.Thread(target=publish, daemon=True)
            pub_thread.start()

            def wait(text, timeout=90):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    if any(text in ln for ln in lines):
                        return
                    self.assertIsNone(rec.poll(), "recorder exited:\n" + "\n".join(lines[-30:]))
                    time.sleep(0.05)
                self.fail(f"no {text!r} in recorder output:\n" + "\n".join(lines[-30:]))

            def event(ev):
                pubs["events"].publish(String(data=ev))

            try:
                wait("[rec] controls")
                time.sleep(1.0)  # discovery
                event("toggle")
                wait("RECORDING episode 0")
                time.sleep(2.0)
                event("toggle")
                wait("saved. total episodes: 1")
                event("toggle")  # second episode, thrown away by Y (real: discard only)
                wait("RECORDING episode 1")
                time.sleep(0.5)
                event("reset")
                wait("discarded")
                rec.stdin.write("q\n")
                rec.stdin.flush()
                self.assertEqual(rec.wait(timeout=60), 0, "\n".join(lines[-30:]))
            finally:
                stop.set()
                pub_thread.join()
                if rec.poll() is None:
                    rec.kill()
                rec.stdin.close()
                rec.stdout.close()
                node.destroy_node()
                rclpy.shutdown(context=ctx)

            root = tmp / "ds"
            info = json.loads((root / "meta/info.json").read_text())
            self.assertEqual(info["robot_type"], "igus_rebel_dual_arm_real")
            self.assertEqual(info["fps"], 15)
            self.assertEqual(info["total_episodes"], 1)
            for k in ("wrist_left", "head"):
                self.assertEqual(info["features"][f"observation.images.{k}"]["shape"], [240, 320, 3])
            out = subprocess.run([LEROBOT_PY, "-c", (
                "import json, glob, pandas as pd\n"
                f"df = pd.concat(pd.read_parquet(f) for f in sorted(glob.glob({str(root)!r} + '/data/**/*.parquet', recursive=True)))\n"
                "print(json.dumps({'n': len(df), 'state': df['observation.state'].iloc[-1].tolist(),\n"
                "  'action': df['action'].iloc[-1].tolist(), 'dt': df['timestamp'].diff().dropna().round(4).unique().tolist()}))\n")],
                capture_output=True, text=True, check=True, timeout=60)
            d = json.loads(out.stdout.strip().splitlines()[-1])
            self.assertTrue(22 <= d["n"] <= 38, d["n"])  # ~2 s at 15 fps of wall time
            self.assertEqual(len(d["dt"]), 1, d["dt"])  # timestamps = frame_index / fps
            self.assertAlmostEqual(d["dt"][0], 1 / 15, places=4)
            exp_state = LEFT_Q + [0.0] + RIGHT_ASSUMED + [0.0]  # driver for left, teleop fallback for right, no grippers
            for got, want in zip(d["state"], exp_state):
                self.assertAlmostEqual(got, want, places=5)
            for got, want in zip(d["action"], ACTION):
                self.assertAlmostEqual(got, want, places=5)
            videos = sorted(p.relative_to(root).parts[1] for p in root.glob("videos/*/*/*.mp4"))
            self.assertEqual(videos, ["observation.images.head", "observation.images.wrist_left"])
            self.assertFalse(any("frame skipped" in ln for ln in lines), "\n".join(lines))
            self.assertFalse(any("Traceback" in ln for ln in lines), "\n".join(lines))  # clean shutdown

    def test_sim_source_still_samples_on_clock(self):
        """The shared sampling loop also serves --source sim: frames every 1/fps of /clock, state from Isaac."""
        from rosgraph_msgs.msg import Clock

        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            ctx = rclpy.Context()
            rclpy.init(context=ctx, domain_id=DOMAIN)
            node = rclpy.create_node("fake_isaac", context=ctx)
            p_clock = node.create_publisher(Clock, "/clock", 10)
            p_js = node.create_publisher(JointState, "/dual_arm/isaac_joint_states", 10)
            p_act = node.create_publisher(JointState, "/quest_teleop/action", 10)
            p_ev = node.create_publisher(String, "/quest_teleop/events", 10)
            stop = threading.Event()
            names = [f"arm_{s}_{j}" for s in ("left", "right") for j in ARM]

            def publish():  # sim at half real time
                t0 = time.monotonic()
                while not stop.is_set():
                    sim = 0.5 * (time.monotonic() - t0)
                    msg = Clock()
                    msg.clock.sec, msg.clock.nanosec = int(sim), int((sim % 1) * 1e9)
                    p_clock.publish(msg)
                    js = JointState(name=names, position=LEFT_Q + RIGHT_ASSUMED)
                    js.header.stamp = msg.clock
                    p_js.publish(js)
                    p_act.publish(JointState(name=NAMES, position=ACTION))
                    time.sleep(1 / 60)

            env = {**os.environ, "ROBOT_REMOTE_DOMAIN_ID": str(DOMAIN), "LEROBOT_PY": LEROBOT_PY}
            rec = subprocess.Popen([str(ROOT / "recording/run_record.sh"), "--name", "sim", "--task", "t",
                                    "--root", str(tmp), "--fps", "10", "--cameras"], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
            lines = []
            threading.Thread(target=lambda: [lines.append(ln.rstrip()) for ln in rec.stdout], daemon=True).start()
            th = threading.Thread(target=publish, daemon=True)
            th.start()
            try:
                end = time.monotonic() + 90
                while not any("[rec] controls" in ln for ln in lines) and time.monotonic() < end and rec.poll() is None:
                    time.sleep(0.05)
                time.sleep(1.0)
                p_ev.publish(String(data="toggle"))
                time.sleep(4.0)  # = 2 s of sim time
                p_ev.publish(String(data="toggle"))
                end = time.monotonic() + 60
                while not any("saved. total episodes: 1" in ln for ln in lines) and time.monotonic() < end:
                    time.sleep(0.05)
                rec.stdin.write("q\n")
                rec.stdin.flush()
                self.assertEqual(rec.wait(timeout=60), 0, "\n".join(lines[-30:]))
            finally:
                stop.set()
                th.join()
                if rec.poll() is None:
                    rec.kill()
                rec.stdin.close()
                rec.stdout.close()
                node.destroy_node()
                rclpy.shutdown(context=ctx)
            info = json.loads((tmp / "sim/meta/info.json").read_text())
            self.assertEqual(info["robot_type"], "igus_rebel_dual_arm_isaac")
            self.assertEqual(info["total_episodes"], 1, "\n".join(lines[-30:]))
            self.assertTrue(15 <= info["total_frames"] <= 25, info["total_frames"])  # 2 s sim at 10 fps, not 4 s wall


if __name__ == "__main__":
    unittest.main()
