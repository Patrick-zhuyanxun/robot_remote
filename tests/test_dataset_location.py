"""Where a recording goes: recording/dataset_check.py (what is at <root>/<name>), the recorder acting on it (create in a
new / empty folder, re-create a never-saved shell, append to a compatible dataset, refuse everything else without
touching it) and the launcher's save-location line / Start check."""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402,F401  - FIRST: never share the robot's ROS graph
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "recording"))
sys.path.insert(0, str(ROOT / "launcher"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import dataset_check as dc  # noqa: E402

SIM_INFO = {"robot_type": "igus_rebel_dual_arm_isaac", "fps": 30, "total_episodes": 2, "features": {
    "observation.state": {"shape": [14]}, "action": {"shape": [14]},
    "observation.images.head": {"shape": [480, 640, 3]}}}


def write_dataset(path, info, episodes=True):
    (path / "meta").mkdir(parents=True)
    (path / "meta/info.json").write_text(json.dumps(info))
    if episodes:
        (path / "meta/episodes/chunk-000").mkdir(parents=True)
        (path / "meta/episodes/chunk-000/file-000.parquet").write_bytes(b"x")
        (path / "data/chunk-000").mkdir(parents=True)
        (path / "data/chunk-000/file-000.parquet").write_bytes(b"x")


class DatasetCheckTests(unittest.TestCase):
    def test_states(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            self.assertEqual(dc.inspect(d / "nope")["state"], "new")
            (d / "empty/sub").mkdir(parents=True)
            self.assertEqual(dc.inspect(d / "empty")["state"], "empty_dir")
            write_dataset(d / "shell", {**SIM_INFO, "total_episodes": 0}, episodes=False)
            self.assertEqual(dc.inspect(d / "shell")["state"], "shell")
            write_dataset(d / "ds", SIM_INFO)
            st = dc.inspect(d / "ds")
            self.assertEqual((st["state"], st["episodes"]), ("dataset", 2))
            write_dataset(d / "killed", SIM_INFO, episodes=False)
            (d / "killed/data").mkdir()
            (d / "killed/data/x.parquet").write_bytes(b"x")
            self.assertEqual(dc.inspect(d / "killed")["state"], "incomplete")
            (d / "photos").mkdir()
            (d / "photos/a.jpg").write_bytes(b"x")
            self.assertEqual(dc.inspect(d / "photos")["state"], "not_dataset")
            (d / "f").write_text("x")
            self.assertEqual(dc.inspect(d / "f")["state"], "file")
            for name, ok in (("nope", True), ("empty", True), ("shell", True), ("ds", True), ("killed", False),
                             ("photos", False), ("f", False)):
                self.assertEqual(dc.describe(dc.inspect(d / name))[0], ok, name)

    def test_compatibility(self):
        self.assertEqual(dc.compatible(SIM_INFO, "sim", 30, {"head": (480, 640)}), [])
        self.assertEqual(len(dc.compatible(SIM_INFO, "real", 30, {"head": (480, 640)})), 1)
        self.assertEqual(len(dc.compatible(SIM_INFO, "sim", 15, {"head": (480, 640)})), 1)
        self.assertIn("cameras", dc.compatible(SIM_INFO, "sim", 30, {"head": (480, 640), "knife": (480, 640)})[0])
        self.assertIn("image size", dc.compatible(SIM_INFO, "sim", 30, {"head": (240, 320)})[0])
        ok, text = dc.describe(dc.inspect(Path("/nonexistent")), [])
        self.assertTrue(ok)

    def test_prepare_new_only_removes_empty_folders_and_a_shell(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / "empty/a/b").mkdir(parents=True)
            dc.prepare_new(dc.inspect(d / "empty"))
            self.assertFalse((d / "empty").exists())
            write_dataset(d / "shell", {**SIM_INFO, "total_episodes": 0}, episodes=False)
            dc.prepare_new(dc.inspect(d / "shell"))
            self.assertFalse((d / "shell").exists())
            write_dataset(d / "ds", SIM_INFO)
            dc.prepare_new(dc.inspect(d / "ds"))  # a real dataset: untouched
            self.assertTrue((d / "ds/meta/info.json").exists())


try:
    import test_real_recording as trr
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
except ImportError:  # pragma: no cover
    trr = None


@unittest.skipIf(trr is None or trr.rclpy is None or trr.LEROBOT_PY is None,
                 "needs ROS 2 rclpy and the LeRobot recorder Python")
class RecorderLocationTests(unittest.TestCase):
    """record_dataset.py --source real with video files as cameras (as in test_real_recording)."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.dir = Path(cls.tmp.name)
        trr.make_video(cls.dir / "head.mp4", [0, 200, 0])
        cls.cams = cls.dir / "cams.json"
        cls.cams.write_text(json.dumps({"head": {"device": str(cls.dir / "head.mp4"), "width": 320, "height": 240,
                                                 "fps": 30, "fourcc": None}}))
        cls.ctx = rclpy.Context()
        rclpy.init(context=cls.ctx, domain_id=trr.DOMAIN + 1)
        cls.node = rclpy.create_node("fake_rig_location", context=cls.ctx)
        pubs = [(cls.node.create_publisher(JointState, f"/{s}/joint_states", 10), JointState(name=trr.ARM, position=trr.LEFT_Q))
                for s in ("left", "right")]
        pubs.append((cls.node.create_publisher(JointState, "/quest_teleop/action", 10),
                     JointState(name=trr.NAMES, position=trr.ACTION)))
        cls.ev = cls.node.create_publisher(String, "/quest_teleop/events", 10)
        cls.stop = threading.Event()

        def publish():
            while not cls.stop.is_set():
                for pub, msg in pubs:
                    pub.publish(msg)
                time.sleep(1 / 30)
        cls.th = threading.Thread(target=publish, daemon=True)
        cls.th.start()

    @classmethod
    def tearDownClass(cls):
        cls.stop.set()
        cls.th.join()
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.ctx)
        cls.tmp.cleanup()

    def record(self, root, name, fps=15, episode=True):
        """One recorder session; returns (exit code, output lines)."""
        env = {**os.environ, "ROBOT_REMOTE_DOMAIN_ID": str(trr.DOMAIN + 1), "LEROBOT_PY": trr.LEROBOT_PY}
        rec = subprocess.Popen([str(ROOT / "recording/run_record.sh"), "--source", "real", "--name", name, "--task", "t",
                                "--root", str(root), "--fps", str(fps), "--camera-config", str(self.cams)],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        lines = []
        reader = threading.Thread(target=lambda: [lines.append(ln.rstrip()) for ln in rec.stdout], daemon=True)
        reader.start()

        def wait(text, timeout=90):
            end = time.monotonic() + timeout
            while time.monotonic() < end and rec.poll() is None:
                if any(text in ln for ln in lines):
                    return True
                time.sleep(0.05)
            return any(text in ln for ln in lines)
        try:
            if wait("[rec] controls") and episode:
                time.sleep(1.0)
                self.ev.publish(String(data="toggle"))
                self.assertTrue(wait("RECORDING"), "\n".join(lines))
                time.sleep(1.0)
                self.ev.publish(String(data="toggle"))
                self.assertTrue(wait("saved. total episodes"), "\n".join(lines))
            if rec.poll() is None:
                rec.stdin.write("q\n")
                rec.stdin.flush()
            code = rec.wait(timeout=60)
        finally:
            if rec.poll() is None:
                rec.kill()
            reader.join(timeout=5)
            rec.stdin.close()
            rec.stdout.close()
        return code, lines

    def episodes(self, path):
        return json.loads((path / "meta/info.json").read_text())["total_episodes"]

    def test_new_append_empty_shell_and_refusals(self):
        root = self.dir / "root"
        code, lines = self.record(root, "ds")  # new
        self.assertEqual(code, 0, "\n".join(lines))
        code, lines = self.record(root, "ds")  # continue the same dataset
        self.assertEqual(code, 0, "\n".join(lines))
        self.assertTrue(any("resuming existing dataset" in ln and "(1 episodes)" in ln for ln in lines), lines)
        self.assertEqual(self.episodes(root / "ds"), 2)

        (root / "empty").mkdir()  # an empty folder made with "New folder"
        code, lines = self.record(root, "empty")
        self.assertEqual((code, self.episodes(root / "empty")), (0, 1), "\n".join(lines))

        code, lines = self.record(root, "shell", episode=False)  # quit before saving: info.json only
        self.assertEqual(code, 0, "\n".join(lines))
        self.assertEqual(dc.inspect(root / "shell")["state"], "shell")
        code, lines = self.record(root, "shell")  # LeRobot cannot resume that: re-created
        self.assertEqual((code, self.episodes(root / "shell")), (0, 1), "\n".join(lines))

        (root / "photos").mkdir()
        (root / "photos/a.jpg").write_bytes(b"keep me")
        code, lines = self.record(root, "photos")
        self.assertNotEqual(code, 0)
        self.assertTrue(any("not a LeRobot dataset" in ln for ln in lines), lines)
        self.assertEqual([p.name for p in (root / "photos").iterdir()], ["a.jpg"])

        code, lines = self.record(root, "ds", fps=30)  # incompatible: refused, dataset untouched
        self.assertNotEqual(code, 0)
        self.assertTrue(any("CANNOT be continued" in ln or "different setup" in ln for ln in lines), lines)
        self.assertEqual(self.episodes(root / "ds"), 2)


try:
    import tkinter  # noqa: F401
    import robot_remote_gui as gui
except ImportError:  # pragma: no cover
    gui = None


@unittest.skipIf(gui is None or not os.environ.get("DISPLAY"), "needs tkinter and a display")
class LauncherLocationTests(unittest.TestCase):
    def test_default_hint_start_check_and_browse(self):
        self.assertEqual(Path(gui.DEFAULTS["root"]), ROOT / "dataset")
        with tempfile.TemporaryDirectory() as d, patch.object(gui, "CONFIG", Path(d) / "launcher.json"):
            d = Path(d)
            gui.CONFIG.write_text(json.dumps({"root": str(ROOT)}))  # old setting: the project folder itself
            app = gui.App()
            app.withdraw()
            try:
                self.assertEqual(Path(app.v_root.get()), ROOT / "dataset")
                app.v_target.set("isaac")
                app.v_record.set(True)
                app.v_task.set("t")
                app.v_res.set("640x480")
                app.v_fps.set("30")
                for k, v in app.v_cams.items():
                    v.set(k == "head")
                app.v_root.set(str(d))
                write_dataset(d / "ds", SIM_INFO)
                app.v_name.set("ds")
                app.update()
                self.assertIn("2 episodes", app.v_path_hint.cget("text"))
                self.assertIn("appended", app.v_path_hint.cget("text"))
                app._collect()  # compatible: no error
                app.v_fps.set("15")
                self.assertIn("CANNOT be continued", app.v_path_hint.cget("text"))
                with self.assertRaises(ValueError):
                    app._collect()
                app.v_fps.set("30")
                (d / "photos").mkdir()
                (d / "photos/a.jpg").write_bytes(b"x")
                app.v_name.set("photos")
                self.assertIn("not a LeRobot dataset", app.v_path_hint.cget("text"))
                with self.assertRaises(ValueError):
                    app._collect()
                app.v_name.set("brand_new")
                self.assertIn("new dataset", app.v_path_hint.cget("text"))
                app.v_root.set("/tmp")
                with patch.object(gui.filedialog, "askdirectory", return_value=str(d / "ds")):
                    app._browse()  # picked the dataset folder itself -> continue it
                self.assertEqual((Path(app.v_root.get()), app.v_name.get()), (d, "ds"))
            finally:
                app.destroy()


if __name__ == "__main__":
    unittest.main()
