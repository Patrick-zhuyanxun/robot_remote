"""Launcher run page: the rotation-frame button talks to the running teleop over its stdin (hidden Tk window)."""
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
                patch.object(gui, "LOG_DIR", Path(d)):
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


if __name__ == "__main__":
    unittest.main()
