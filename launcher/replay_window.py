#!/usr/bin/env python3
"""Pick a recorded LeRobot dataset folder, choose an episode, and play it back.

  robot_remote/launcher/run_replay_gui.sh         (or the "Play back a dataset" button in the launcher)

Two ways to play an episode:
  * View: LeRobot's own viewer (lerobot-dataset-viz, Rerun) shows the camera videos and the state/action curves.
    Needs only the dataset, no simulator.
  * Replay in Isaac: starts the Isaac rig (once, kept running for further replays), streams the episode's
    recorded actions into it and reports how closely the arms reproduce the recorded states.
"""

import json
import os
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_remote_gui import C, ROOT, Proc, apply_style, load_cfg  # noqa: E402


def lerobot_python() -> str:
    """Same lookup as recording/run_record.sh: $LEROBOT_PY, the repo's .venv_record, ~/Robotic_infra/lerobot."""
    for p in (os.environ.get("LEROBOT_PY"), ROOT / ".venv_record" / "bin" / "python",
              Path.home() / "Robotic_infra" / "lerobot" / ".venv" / "bin" / "python"):
        if p and Path(p).is_file():
            return str(p)
    return "python3"


class ReplayWindow(tk.Toplevel):
    def __init__(self, master, start_dir: str | None = None, standalone: bool = False):
        super().__init__(master)
        self.standalone = standalone
        self.title("robot_remote — play back a dataset")
        self.configure(bg=C["bg"])
        self.geometry("980x700")
        self.sim: Proc | None = None
        self.replay: Proc | None = None
        self.info: dict | None = None
        self.v_path = tk.StringVar(value=start_dir or "")
        self.v_show_isaac = tk.BooleanVar(value=True)
        self._build()
        self.protocol("WM_DELETE_WINDOW", self.close)
        if self.v_path.get():
            self.load()

    # ------------------------------------------------------------------ layout
    def _build(self):
        g = ttk.LabelFrame(self, text="1  Dataset folder")
        g.pack(fill="x", padx=14, pady=(12, 4))
        row = ttk.Frame(g)
        row.pack(fill="x", padx=8, pady=6)
        ttk.Entry(row, textvariable=self.v_path).pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="Browse...", command=self.browse).pack(side="left", padx=6)
        ttk.Button(row, text="Load", command=self.load).pack(side="left")
        self.l_info = ttk.Label(g, text="Choose the folder of one dataset (the one that contains meta/, data/, videos/).",
                                style="Dim.TLabel")
        self.l_info.pack(anchor="w", padx=8, pady=(0, 6))

        g = ttk.LabelFrame(self, text="2  Episode")
        g.pack(fill="both", expand=True, padx=14, pady=4)
        self.eps = tk.Listbox(g, height=10, bg=C["panel"], fg=C["fg"], selectbackground=C["accent"],
                              font=("DejaVu Sans Mono", 10), relief="flat", activestyle="none", exportselection=False)
        self.eps.pack(fill="both", expand=True, padx=8, pady=6)

        g = ttk.LabelFrame(self, text="3  Play")
        g.pack(fill="x", padx=14, pady=4)
        row = ttk.Frame(g)
        row.pack(fill="x", padx=8, pady=6)
        self.b_view = ttk.Button(row, text="👁  View videos & curves (LeRobot viewer)", command=self.view)
        self.b_view.pack(side="left")
        self.b_isaac = ttk.Button(row, text="▶  Replay in Isaac", style="Accent.TButton", command=self.replay_isaac)
        self.b_isaac.pack(side="left", padx=10)
        ttk.Checkbutton(row, text="show the Isaac window", variable=self.v_show_isaac).pack(side="left")
        self.b_stop = ttk.Button(row, text="■ Stop Isaac", command=self.stop_sim)
        self.b_stop.pack(side="right")
        ttk.Label(g, text="View needs no simulator. Replay in Isaac starts the sim once (30-60 s) and keeps it running for the "
                          "next replays. Do not run the Quest teleop at the same time.", style="Dim.TLabel",
                  wraplength=900).pack(anchor="w", padx=8, pady=(0, 6))

        g = ttk.LabelFrame(self, text="Messages")
        g.pack(fill="both", expand=True, padx=14, pady=(4, 12))
        self.log = tk.Text(g, height=10, bg=C["panel"], fg=C["fg"], relief="flat", font=("DejaVu Sans Mono", 9))
        self.log.pack(fill="both", expand=True, padx=4, pady=4)
        self._buttons()

    def _msg(self, text):
        self.after(0, lambda: (self.log.insert("end", text + "\n"), self.log.see("end")))

    def _buttons(self):
        busy = self.replay is not None and self.replay.alive()
        ready = self.info is not None and not busy
        for b in (self.b_view, self.b_isaac):
            b.configure(state="normal" if ready else "disabled")
        self.b_stop.configure(state="normal" if self.sim is not None and self.sim.alive() else "disabled")

    # ------------------------------------------------------------------ dataset
    def browse(self):
        start = self.v_path.get() or load_cfg().get("root") or str(Path.home())
        d = filedialog.askdirectory(parent=self, initialdir=str(Path(start).expanduser()),
                                    title="Choose a dataset folder (contains meta/)")
        if d:
            self.v_path.set(d)
            self.load()

    def load(self):
        path = Path(self.v_path.get().strip()).expanduser()
        self.info = None
        self.eps.delete(0, "end")
        self._buttons()
        if not (path / "meta" / "info.json").is_file():
            self.l_info.configure(text=f"Not a LeRobot dataset folder (no meta/info.json): {path}")
            return
        self.l_info.configure(text="Reading the dataset...")

        def work():
            env = {**os.environ, "HF_HUB_OFFLINE": "1"}
            r = subprocess.run([lerobot_python(), str(ROOT / "recording" / "dataset_info.py"), str(path)],
                               capture_output=True, text=True, env=env)
            try:
                info = json.loads(r.stdout.strip().splitlines()[-1])
            except (IndexError, ValueError):
                info = {"error": (r.stderr.strip().splitlines() or ["could not read the dataset"])[-1]}
            self.after(0, lambda: self._show(path, info))

        threading.Thread(target=work, daemon=True).start()

    def _show(self, path, info):
        if "error" in info:
            self.l_info.configure(text=info["error"])
            return
        self.info = {**info, "path": str(path)}
        fps = info["fps"]
        self.l_info.configure(text=f"{info['name']}:  {info['total_episodes']} episodes, {info['total_frames']} frames, "
                                   f"{fps} fps, cameras: {', '.join(info['cameras']) or 'none'}")
        for e in info["episodes"]:
            self.eps.insert("end", f"episode {e['index']:3d}   {e['length']:5d} frames   {e['length'] / fps:6.1f} s   {e['task']}")
        if info["episodes"]:
            self.eps.selection_set(0)
        self._buttons()

    def _episode(self):
        sel = self.eps.curselection()
        return self.info["episodes"][sel[0] if sel else 0]["index"]

    # ------------------------------------------------------------------ play
    def view(self):
        ep = self._episode()
        cmd = [lerobot_python(), "-m", "lerobot.scripts.lerobot_dataset_viz", "--repo-id", f"local/{self.info['name']}",
               "--root", self.info["path"], "--episode-index", str(ep)]
        self._msg(f"[view] opening episode {ep} in the LeRobot viewer (Rerun)...")
        # the Rerun viewer binary lives next to that python; HF offline = never look the local dataset up on the Hub
        env = {"HF_HUB_OFFLINE": "1", "PATH": f"{Path(cmd[0]).parent}:{os.environ.get('PATH', '')}"}
        p = Proc("view", cmd, lambda _n, line: self._msg(f"[view] {line}") if "Error" in line or "Traceback" in line else None,
                 extra_env=env)
        self._msg(f"[view] log: {p.log_path}")

    def replay_isaac(self):
        ep = self._episode()
        self.replay = None
        threading.Thread(target=self._replay_seq, args=(ep,), daemon=True).start()

    def _replay_seq(self, ep):
        if self.sim is None or not self.sim.alive():
            cmd = [str(ROOT / "dual_arm_sim" / "run_dual_arm_sim.sh")]
            if not self.v_show_isaac.get() or os.environ.get("ROBOT_REMOTE_SIM_HEADLESS") == "1":
                cmd.append("--headless")
            self._msg("[isaac] starting the simulator (30-60 s)...")
            self.sim = Proc("replay_sim", cmd)
            self.after(0, self._buttons)
            while self.sim.alive() and not any("[dual_arm] running" in ln for ln in self.sim.lines):
                threading.Event().wait(0.5)
            if not self.sim.alive():
                self._msg(f"[isaac] failed to start - see {self.sim.log_path}")
                return
            self._msg("[isaac] running")
        self._msg(f"[replay] episode {ep} of {self.info['name']}: moving to its start pose, then streaming the actions")
        self.replay = Proc("replay", [str(ROOT / "recording" / "run_replay.sh"), "--path", self.info["path"],
                                      "--episode", str(ep)],
                           lambda _n, line: self._msg(line) if line.startswith("[replay]") or "Error" in line else None)
        self.after(0, self._buttons)
        self.replay.p.wait()
        self._msg(f"[replay] done (exit {self.replay.p.returncode})")
        self.after(0, self._buttons)

    def stop_sim(self):
        def work():
            for p in (self.replay, self.sim):
                if p is not None and p.alive():
                    p.interrupt()
                    try:
                        p.p.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        p.kill()
            self._msg("[isaac] stopped")
            self.after(0, self._buttons)
        threading.Thread(target=work, daemon=True).start()

    def close(self):
        self.stop_sim()
        self.after(1500, self.master.destroy if self.standalone else self.destroy)


if __name__ == "__main__":
    root = tk.Tk()
    root.withdraw()
    apply_style(root)
    ReplayWindow(root, sys.argv[1] if len(sys.argv) > 1 else None, standalone=True)
    root.mainloop()
