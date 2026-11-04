"""USB camera drop-outs of the real recorder (no real cameras, no robot): a camera that disappears and comes back is
found again (RealCamera), and record_dataset.py --source real ends the open episode at the drop instead of skipping
frames for the rest of the take. Cameras are looping video files; "unplugging" one = moving its file away (the same
signal as a vanished /dev/videoN: the reader loses it and retries until it is back).
Runs on a private ROS domain, never the robot's."""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[1] / "recording"))
import ros_isolation  # noqa: E402,F401  - FIRST: never share the robot's ROS graph
import argparse
import contextlib
import io
import json
import os
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

import real_cameras
from test_real_recording import ACTION, ARM, LEFT_Q, LEROBOT_PY, RIGHT_ASSUMED, ROOT

try:
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
except ImportError:  # pragma: no cover
    rclpy = None

DOMAIN = 85


def write_video(path, colour, n=30, size=(320, 240)):
    w = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 30, size)
    for i in range(n):
        f = np.full((size[1], size[0], 3), colour, np.uint8)
        f[:, i * 10:i * 10 + 10] = 255
        w.write(f)
    w.release()


def wait_for(cond, timeout, step=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return False


class RealCameraRetryTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        write_video(self.tmp / "a.mp4", [0, 0, 200])
        self.cams = []

    def tearDown(self):
        for c in self.cams:
            c.close()
        self._tmp.cleanup()

    def camera(self, path, **kw):
        c = real_cameras.RealCamera("wrist_right", path, 320, 240, 30, None, **kw)
        c.RETRY_S = 0.1
        self.cams.append(c)
        return c

    def test_lost_camera_keeps_its_thread_and_resumes_when_it_is_back(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            cam = self.camera(str(self.tmp / "a.mp4")).open()
            self.assertEqual((cam.state, cam.error), ("ok", None))
            self.assertTrue(cam.healthy())
            (self.tmp / "a.mp4").rename(self.tmp / "gone.mp4")
            self.assertTrue(wait_for(lambda: cam.state == "lost", 2), "never reported lost")
            self.assertFalse(cam.healthy())
            self.assertTrue(cam.alive())            # the thread keeps retrying
            self.assertIsNotNone(cam.error)
            time.sleep(0.5)
            self.assertGreater(cam.age(), 0.4)       # no fresh frames while lost
            n = cam.frames
            (self.tmp / "gone.mp4").rename(self.tmp / "a.mp4")
            self.assertTrue(wait_for(lambda: cam.state == "ok", 3), "never reconnected")
            self.assertTrue(wait_for(lambda: cam.frames > n + 3, 2))
            self.assertLess(cam.age(), 0.2)
            self.assertIsNone(cam.error)
            self.assertEqual(cam.reconnects, 1)
        lines = out.getvalue().splitlines()
        self.assertEqual([ln for ln in lines if "lost (" in ln], ["[cam] wrist_right lost (device disappeared) - retrying"])
        self.assertEqual(len([ln for ln in lines if "reconnected" in ln]), 1, lines)  # once each, however long it took
        self.assertTrue(any(ln.startswith("[cam] wrist_right reconnected at ") for ln in lines), lines)

    def test_retry_resolves_the_same_camera_again_and_reopens_it_with_the_same_settings(self):
        """The ARC re-enumerates as another /dev/videoN: the resolver (same sysfs identity) finds it again."""
        write_video(self.tmp / "b.mp4", [0, 200, 0])
        where = {"dev": str(self.tmp / "a.mp4")}
        calls = []

        def resolver():
            calls.append(where["dev"])
            if where["dev"] is None:
                raise ConnectionError("no camera at USB port 1-2.2")
            return where["dev"]

        with contextlib.redirect_stdout(io.StringIO()):
            cam = self.camera(where["dev"], resolver=resolver).open()
            first = cam.latest()[0].copy()
            (self.tmp / "a.mp4").unlink()
            where["dev"] = None                                   # unplugged: resolver cannot find it
            self.assertTrue(wait_for(lambda: cam.state == "lost", 2))
            n = len(calls)
            self.assertTrue(wait_for(lambda: len(calls) >= n + 3, 2), "not retrying while it is gone")
            self.assertEqual(cam.state, "lost")
            where["dev"] = str(self.tmp / "b.mp4")                # replugged under a new node
            self.assertTrue(wait_for(lambda: cam.state == "ok", 3))
            self.assertEqual(cam.device, str(self.tmp / "b.mp4"))
            self.assertEqual(cam.latest()[0].shape, first.shape)  # same size / format settings
            self.assertGreater(cam.latest()[0][..., 1].mean(), first[..., 1].mean())  # frames of the new node
            self.assertEqual(cam.reconnects, 1)

    def test_a_device_that_opens_with_the_wrong_format_is_retried_not_fatal(self):
        write_video(self.tmp / "wrong.mp4", [1, 2, 3], size=(160, 120))
        where = {"dev": str(self.tmp / "a.mp4")}
        with contextlib.redirect_stdout(io.StringIO()):
            cam = self.camera(where["dev"], resolver=lambda: where["dev"]).open()
            (self.tmp / "a.mp4").unlink()
            self.assertTrue(wait_for(lambda: cam.state == "lost", 2))
            where["dev"] = str(self.tmp / "wrong.mp4")             # RuntimeError inside the retry
            time.sleep(0.5)
            self.assertEqual(cam.state, "lost")
            self.assertTrue(cam.alive())
            write_video(self.tmp / "right.mp4", [9, 9, 9])
            where["dev"] = str(self.tmp / "right.mp4")
            self.assertTrue(wait_for(lambda: cam.state == "ok", 3))

    def test_close_while_lost_returns_promptly(self):
        with contextlib.redirect_stdout(io.StringIO()):
            cam = self.camera(str(self.tmp / "a.mp4")).open()
            (self.tmp / "a.mp4").unlink()
            self.assertTrue(wait_for(lambda: cam.state == "lost", 2))
            t0 = time.monotonic()
            cam.close()
        self.assertLess(time.monotonic() - t0, 1.0)
        self.assertFalse(cam.alive())


class FakeCam:
    def __init__(self, t):
        self.t = t

    def latest(self):
        return (None, None) if self.t is None else (np.zeros((2, 2, 3), np.uint8), self.t)


class FakeDataset:
    def __init__(self):
        self.frames = []

    def add_frame(self, f):
        self.frames.append(f)


class EpisodeCutUnitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        global rd
        import record_dataset as rd

    def test_lost_camera_needs_a_gap_longer_than_the_threshold(self):
        cams = {"head": FakeCam(10.0), "wrist": FakeCam(9.2)}
        self.assertIsNone(rd.lost_camera(cams, ["head", "wrist"], 1.0, 10.0))   # wrist is 0.8 s old
        self.assertEqual(rd.lost_camera(cams, ["head", "wrist"], 0.5, 10.0), ("wrist", 9.2))
        self.assertIsNone(rd.lost_camera({"head": FakeCam(10.0)}, ["head"], 0.5, 10.4))   # short stall: only stale
        self.assertEqual(rd.lost_camera({"head": FakeCam(10.0)}, ["head"], 0.5, 10.6), ("head", 10.0))
        self.assertEqual(rd.lost_camera({"head": FakeCam(None)}, ["head"], 0.5, 10.6), ("head", 10.6))

    def test_frame_hold_cuts_frames_sampled_after_the_drop(self):
        ds = FakeDataset()
        h = rd.FrameHold(ds, 0.55)
        for i in range(21):                      # one frame per 0.1 s, t = 0 .. 2.0
            h.push({"i": i}, i * 0.1)
        self.assertEqual([f["i"] for f in ds.frames], list(range(0, 15)))  # held back by 0.55 s (committed: t <= 1.4)
        self.assertEqual(h.total(), 21)
        # the camera's last frame was at t = 1.55, detected 0.5 s later (t = 2.0): frames 16..20 carry its frozen image
        self.assertEqual(h.drop_after(1.55), 5)
        self.assertEqual([f["i"] for f in ds.frames], list(range(0, 16)))
        self.assertEqual((h.committed, h.total()), (16, 16))

    def test_hold_zero_adds_immediately(self):
        ds = FakeDataset()
        h = rd.FrameHold(ds, 0.0)
        h.push({"i": 0}, 5.0)
        self.assertEqual(len(ds.frames), 1)
        h.reset()
        self.assertEqual((h.committed, h.total()), (0, 0))

    def test_snapshot_refuses_to_start_while_a_camera_is_lost_but_tolerates_a_short_stall_in_an_episode(self):
        if True:
            args = argparse.Namespace(source="real", fps=15, cameras=["head"], camera_gap_s=0.5)
            cam = type("C", (), {})()
            cam.state, cam.error = "ok", None
            cam.alive = lambda: True
            cam.latest = lambda: (np.zeros((4, 4, 3), np.uint8), time.monotonic() - 0.3)
            node = rd.Recorder.__new__(rd.Recorder)
            node.args, node.cameras, node.real = args, {"head": cam}, True
            node.action, node.action_wall = np.zeros(14, np.float32), time.monotonic()
            node.real_q = {s: [0.0] * 6 for s in rd.SIDES}
            node.real_t = {s: time.monotonic() for s in rd.SIDES}
            node.fallback_q = {}
            frame, stale = node.snapshot(starting=True)
            self.assertIsNotNone(frame)                       # 0.3 s old < gap: fine
            self.assertTrue(any("head" in x for x in stale))  # but counted as stale
            cam.state = "lost"
            frame, why = node.snapshot(starting=True)
            self.assertIsNone(frame)
            self.assertIn("lost", why)
            frame, stale = node.snapshot()                    # inside an episode: below the gap = stale frame only
            self.assertIsNotNone(frame)
            cam.state = "ok"
            cam.latest = lambda: (np.zeros((4, 4, 3), np.uint8), time.monotonic() - 0.7)
            frame, why = node.snapshot(starting=True)
            self.assertIsNone(frame)
            self.assertIn("no fresh frame", why)


@unittest.skipIf(rclpy is None or LEROBOT_PY is None, "needs ROS 2 rclpy and the LeRobot recorder Python")
class RealRecordingCameraLossTests(unittest.TestCase):
    """The real recorder process against fake topics and video-file cameras (head = the one that gets unplugged)."""

    def run_rig(self, script, extra_args=(), fps=15):
        """script(rig) drives the session. Returns (rig, dataset root)."""
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        tmp = Path(tmp.name)
        write_video(tmp / "left.mp4", [0, 0, 200])
        write_video(tmp / "head.mp4", [0, 200, 0])
        cfg = {"wrist_left": {"device": str(tmp / "left.mp4"), "width": 320, "height": 240, "fps": 30, "fourcc": None},
               "head": {"device": str(tmp / "head.mp4"), "width": 320, "height": 240, "fps": 30, "fourcc": None}}
        (tmp / "cams.json").write_text(json.dumps(cfg))
        ctx = rclpy.Context()
        rclpy.init(context=ctx, domain_id=DOMAIN)
        node = rclpy.create_node("fake_real_rig_camloss", context=ctx)
        pubs = {"left": node.create_publisher(JointState, "/left/joint_states", 10),
                "viz": node.create_publisher(JointState, "/quest_teleop/joint_states", 10),
                "action": node.create_publisher(JointState, "/quest_teleop/action", 10),
                "events": node.create_publisher(String, "/quest_teleop/events", 10)}
        stop = threading.Event()
        names = [f"arm_{s}_{j}" for s in ("left", "right") for j in ARM + ["gripper"]]

        def publish():
            while not stop.is_set():
                pubs["left"].publish(JointState(name=ARM, position=LEFT_Q))
                pubs["viz"].publish(JointState(name=[f"arm_{s}_{j}" for s in ("left", "right") for j in ARM],
                                               position=[9.0] * 6 + RIGHT_ASSUMED))
                pubs["action"].publish(JointState(name=names, position=ACTION))
                time.sleep(1 / 30)

        env = {**os.environ, "ROBOT_REMOTE_DOMAIN_ID": str(DOMAIN), "ROS_DOMAIN_ID": str(DOMAIN),
               "ROS_LOCALHOST_ONLY": "1", "LEROBOT_PY": LEROBOT_PY}
        rec = subprocess.Popen([str(ROOT / "recording/run_record.sh"), "--source", "real", "--name", "ds",
                                "--task", "cut the box", "--root", str(tmp), "--fps", str(fps),
                                "--camera-config", str(tmp / "cams.json"), *extra_args],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        rig = type("Rig", (), {})()
        rig.lines, rig.rec, rig.tmp = [], rec, tmp
        threading.Thread(target=lambda: [rig.lines.append(ln.rstrip()) for ln in rec.stdout], daemon=True).start()
        th = threading.Thread(target=publish, daemon=True)
        th.start()

        def wait(text, timeout=90, after=0):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if any(text in ln for ln in rig.lines[after:]):
                    return
                self.assertIsNone(rec.poll(), "recorder exited:\n" + "\n".join(rig.lines[-30:]))
                time.sleep(0.02)
            self.fail(f"no {text!r} in recorder output:\n" + "\n".join(rig.lines[-30:]))

        rig.wait = wait
        rig.mark = lambda: len(rig.lines)
        rig.toggle = lambda: pubs["events"].publish(String(data="toggle"))
        rig.unplug = lambda: (tmp / "head.mp4").rename(tmp / "head.unplugged")
        rig.replug = lambda: (tmp / "head.unplugged").rename(tmp / "head.mp4")
        try:
            wait("[rec] controls")
            time.sleep(1.0)  # discovery
            script(rig)
            rec.stdin.write("q\n")
            rec.stdin.flush()
            self.assertEqual(rec.wait(timeout=60), 0, "\n".join(rig.lines[-30:]))
        finally:
            stop.set()
            th.join()
            if rec.poll() is None:
                rec.kill()
            rec.stdin.close()
            rec.stdout.close()
            node.destroy_node()
            rclpy.shutdown(context=ctx)
        if os.environ.get("REC_TEST_SHOW"):
            print("\n".join(rig.lines))
        self.assertFalse(any("Traceback" in ln for ln in rig.lines), "\n".join(rig.lines))
        return rig, tmp / "ds"

    def read_rows_and_video_frames(self, root):
        out = subprocess.run([LEROBOT_PY, "-c", (
            "import json, glob, pandas as pd, av\n"
            f"root = {str(root)!r}\n"
            "df = pd.concat(pd.read_parquet(f) for f in sorted(glob.glob(root + '/data/**/*.parquet', recursive=True)))\n"
            "vids = {}\n"
            "for cam in ('head', 'wrist_left'):\n"
            "    vids[cam] = sum(sum(1 for _ in av.open(f).decode(video=0)) for f in sorted(glob.glob(\n"
            "        root + '/videos/observation.images.' + cam + '/**/*.mp4', recursive=True)))\n"
            "print(json.dumps({'n': len(df), 'episodes': sorted(df['episode_index'].unique().tolist()),\n"
            "  'per_episode': df.groupby('episode_index').size().tolist(), 'video': vids}))\n")],
            capture_output=True, text=True, check=True, timeout=120)
        return json.loads(out.stdout.strip().splitlines()[-1])

    def test_camera_drop_ends_the_episode_at_the_drop_and_a_new_one_starts_after_reconnect(self):
        t = {}

        def script(rig):
            rig.toggle()
            t["start"] = time.monotonic()
            rig.wait("RECORDING episode 0")
            time.sleep(3.0)
            m = rig.mark()
            rig.unplug()
            t["drop"] = time.monotonic()
            rig.wait("[rec] ERROR: camera head lost", timeout=10, after=m)
            t["error"] = time.monotonic()
            rig.wait("saved. total episodes: 1", after=m)
            # the episode ended: nothing more is recorded and nothing is skipped in a loop
            m = rig.mark()
            time.sleep(1.2)
            self.assertFalse(any("frame skipped" in ln or "● " in ln for ln in rig.lines[m:]), rig.lines[m:])
            rig.toggle()                                   # still lost: refused
            rig.wait("cannot start: camera head", after=m)
            self.assertFalse(any("RECORDING" in ln for ln in rig.lines[m:]))
            rig.replug()
            rig.wait("[cam] head reconnected at", after=m)
            m = rig.mark()
            rig.toggle()                                   # no recorder restart needed
            rig.wait("RECORDING episode 1", after=m)
            time.sleep(2.0)
            rig.toggle()
            rig.wait("saved. total episodes: 2", after=m)

        rig, root = self.run_rig(script)
        text = "\n".join(rig.lines)
        err = [ln for ln in rig.lines if ln.startswith("[rec] ERROR:")]
        self.assertEqual(len(err), 1, text)
        self.assertRegex(err[0], r"camera head lost \d+\.\d s into episode 0")
        self.assertIn("episode ended and saved up to the drop (", err[0])
        self.assertIn("press A to start a new one once it is back", err[0])
        self.assertEqual(len([ln for ln in rig.lines if "[cam] head lost (" in ln]), 1, text)
        self.assertEqual(len([ln for ln in rig.lines if "[cam] head reconnected at" in ln]), 1, text)
        self.assertLess(t["error"] - t["drop"], 1.5)       # noticed within gap + a bit
        d = self.read_rows_and_video_frames(root)
        self.assertEqual(d["episodes"], [0, 1])
        n0, n1 = d["per_episode"]
        drop_after = t["drop"] - t["start"]                # recorded time that existed before the drop
        # no frames after the drop (a few ticks of start-up / jitter slack), and nothing much lost before it
        self.assertLessEqual(n0, drop_after * 15 + 3, (n0, drop_after))
        self.assertGreaterEqual(n0, (drop_after - 0.6) * 15, (n0, drop_after))
        self.assertTrue(25 <= n1 <= 38, n1)                # the second take: ~2 s
        self.assertEqual(d["video"]["head"], d["n"], d)    # frame count == rows == video frames, every camera
        self.assertEqual(d["video"]["wrist_left"], d["n"], d)

    def test_episode_cut_short_by_a_drop_under_one_second_is_discarded(self):
        def script(rig):
            m = rig.mark()
            rig.toggle()
            rig.wait("RECORDING episode 0", after=m)
            time.sleep(0.3)
            m = rig.mark()
            rig.unplug()
            rig.wait("[rec] ERROR: camera head lost", timeout=10, after=m)
            self.assertTrue(any("DISCARDED" in ln for ln in rig.lines[m:] if ln.startswith("[rec] ERROR:")))
            rig.wait("✕ discarded (", after=m)
            time.sleep(0.5)
            self.assertFalse(any("saved. total episodes" in ln for ln in rig.lines[m:]))
            rig.replug()
            rig.wait("[cam] head reconnected at", after=m)
            m = rig.mark()
            rig.toggle()
            rig.wait("RECORDING episode 0", after=m)       # still episode 0: nothing was saved
            time.sleep(1.5)
            rig.toggle()
            rig.wait("saved. total episodes: 1", after=m)

        rig, root = self.run_rig(script)
        err = [ln for ln in rig.lines if ln.startswith("[rec] ERROR:")]
        self.assertEqual(len(err), 1, "\n".join(rig.lines))
        self.assertIn("episode ended and DISCARDED", err[0])
        d = self.read_rows_and_video_frames(root)
        self.assertEqual(d["episodes"], [0])
        self.assertTrue(15 <= d["n"] <= 28, d)
        self.assertEqual(d["video"]["head"], d["n"])
        self.assertEqual(d["video"]["wrist_left"], d["n"])

    def test_a_stall_below_the_threshold_keeps_the_episode_going(self):
        """--camera-gap-s 4: a 1.5 s unplug is only a stale stream; the take continues and nothing is cut."""
        t = {}

        def script(rig):
            m = rig.mark()
            rig.toggle()
            t["start"] = time.monotonic()
            rig.wait("RECORDING episode 0", after=m)
            time.sleep(1.5)
            rig.unplug()
            time.sleep(1.5)
            rig.replug()
            rig.wait("[cam] head reconnected at", after=m)
            time.sleep(1.5)
            rig.toggle()
            t["stop"] = time.monotonic()
            rig.wait("saved. total episodes: 1", after=m)

        rig, root = self.run_rig(script, extra_args=("--camera-gap-s", "4"))
        self.assertFalse(any(ln.startswith("[rec] ERROR:") for ln in rig.lines), "\n".join(rig.lines))
        d = self.read_rows_and_video_frames(root)
        dur = t["stop"] - t["start"]
        self.assertEqual(d["episodes"], [0])
        self.assertGreaterEqual(d["n"], (dur - 0.5) * 15, (d, dur))   # the whole take, timeline not compressed
        self.assertTrue(any("had a stale stream" in ln and "saving episode 0" in ln for ln in rig.lines))
        self.assertEqual(d["video"]["head"], d["n"])


if __name__ == "__main__":
    unittest.main()
