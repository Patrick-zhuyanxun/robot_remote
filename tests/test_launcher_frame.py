"""Launcher run page: the rotation-frame button talks to the running teleop over its stdin (hidden Tk window)."""
import sys as _sys, pathlib as _pl
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parent))
import ros_isolation  # noqa: E402,F401  - FIRST: never share the robot's ROS graph
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "launcher"))
try:
    import tkinter
    import robot_remote_gui as gui
except ImportError:
    gui = None

# stands in for the teleop: confirms each "rot_frame X" line the way quest_dual_arm_teleop.py logs it
ECHO = ("import sys\nprint('[INFO] [t]: rotation frame: tool (controller twist axis ...)', flush=True)\n"
        "for line in sys.stdin:\n    print('[INFO] [t]: rotation frame: ' + line.split()[1], flush=True)\n")


@unittest.skipIf(gui is None or not os.environ.get("DISPLAY"), "needs tkinter and a display")
class FrameButtonTests(unittest.TestCase):
    def pump(self, app, until, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not until():
            app.update()
            time.sleep(0.02)

    def test_toggle_sends_command_and_follows_confirmation(self):
        with tempfile.TemporaryDirectory() as d, patch.object(gui, "CONFIG", Path(d) / "launcher.json"), \
                patch.object(gui, "LOG_DIR", Path(d)), \
                patch.object(gui, "LATEST_DIR", Path(d) / "latest"):
            app = gui.App()
            app.withdraw()
            try:
                app.cfg = {**gui.DEFAULTS, "width": 640, "height": 480}
                app.run_frame = app._build_run()
                self.assertEqual(str(app.b_frame["state"]), "disabled")
                app.procs["teleop"] = gui.Proc("teleop", [sys.executable, "-c", ECHO], app._on_line, stdin=True)
                self.pump(app, lambda: str(app.b_frame["state"]) == "normal")
                self.assertEqual(app.b_frame["text"], gui.FRAME_TEXT["tool"])
                app._toggle_frame()
                self.assertEqual(str(app.b_frame["state"]), "disabled")  # until the teleop confirms
                self.pump(app, lambda: app.cfg["rot_frame"] == "base" and str(app.b_frame["state"]) == "normal")
                self.assertEqual(app.b_frame["text"], gui.FRAME_TEXT["base"])
                self.assertEqual(json.loads(gui.CONFIG.read_text())["rot_frame"], "base")  # next start uses it
                app._toggle_frame()
                self.pump(app, lambda: app.cfg["rot_frame"] == "tool" and str(app.b_frame["state"]) == "normal")
                self.assertEqual(app.b_frame["text"], gui.FRAME_TEXT["tool"])
            finally:
                for p in app.procs.values():
                    p.kill()
                app.destroy()
            self.assertIn("rotation frame", (Path(d) / "latest" / "teleop.log").read_text())  # newest-session copy

    def test_real_target_records_with_usb_cameras(self):
        with tempfile.TemporaryDirectory() as d, patch.object(gui, "CONFIG", Path(d) / "launcher.json"), \
                patch.object(gui, "LOG_DIR", Path(d)):
            app = gui.App()
            app.withdraw()
            try:
                app.v_target.set("real")
                app.update()
                states = lambda ws: {str(w["state"]) for w in ws if "state" in w.keys()}  # noqa: E731
                self.assertEqual(states(app.real_only_widgets), {"normal"})
                self.assertEqual(states(app.sim_only_widgets), {"disabled"})
                app.v_record.set(True)
                app.v_root.set(d)  # a fresh location (the default name may already hold a sim dataset)
                for k, v in app.v_real_cams.items():
                    v.set(k != "knife")
                cfg = app._collect()
                self.assertTrue(cfg["record"])
                self.assertEqual(cfg["real_cameras"], [k for k, _ in gui.REAL_CAMERAS if k != "knife"])
                app.v_target.set("isaac")
                app.update()
                self.assertEqual(states(app.real_only_widgets), {"disabled"})
                self.assertEqual(states(app.sim_only_widgets) - {"readonly"}, {"normal"})
            finally:
                app.destroy()


if __name__ == "__main__":
    unittest.main()
