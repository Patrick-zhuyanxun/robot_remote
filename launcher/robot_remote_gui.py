#!/usr/bin/env python3
"""robot_remote launcher: one window to configure and start Quest teleop + data recording.

  robot_remote/launcher/run_gui.sh

Settings page -> Start:  Isaac sim (run_dual_arm_sim.sh --cameras ...) -> Quest teleop (run_quest_dual_arm.sh)
-> recorder (run_record.sh). Real robot: igus drivers -> teleop (run_quest_real_arm.sh) -> recorder
(run_record.sh --source real, USB cameras of config/real_cameras.json) -> RViz. The running page shows each process, the recording state, the Quest
connection URL, and the controller legend. Settings live in config/launcher.json.
"""

import json
import os
import queue
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
import webbrowser
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "recording"))
import dataset_check  # noqa: E402  (standard library only)
LOG_DIR = ROOT / "logs"
LATEST_DIR = ROOT / "latest_run"  # newest session only: <process>.log + teleop_trace.csv, emptied on every Start
CONFIG = ROOT / "config" / "launcher.json"
LEGACY_CONFIG = Path.home() / ".config" / "robot_remote_launcher.json"
QUEST_URL = "https://nvidia.github.io/IsaacTeleop/client/v1.3.131/"
CAMERAS = [("head", "Head RGB (top view)"), ("wrist_left", "Left wrist RGB"), ("wrist_right", "Right wrist RGB")]
ENDPOINTS = json.loads((ROOT / "config/real_arms.json").read_text())


# real-arm teleop speed presets (quest_dual_arm_teleop.py flags). normal = the real-ros defaults.
# "fast" (45 deg/s, a 0.8, leash 14) was removed after the real-arm J3/J5 test 2026-10-21: no faster arm (~32 deg/s peak
# either way) but 8-10 deg overshoot and > 2.5 s to settle, vs 1.5 deg / 0.9 s with "normal".
# Real-arm release stop (right arm 2026-10-21): the ReBeL itself brakes at only ~30 deg/s^2 after ~0.3 s, so a released
# arm keeps going ~24 deg from ~30 deg/s but only ~6 deg from ~15 deg/s (glide, no pull-back). "precise" = stops fast.
REAL_SPEED = {"precise": ["--max-joint-vel", "0.26", "--max-lin-vel", "0.12", "--max-rot-vel", "22"],
              "normal": []}


def real_camera_list():
    """[(key, label)] of config/real_cameras.json (USB cameras of the real rig)."""
    try:
        cfg = json.loads((ROOT / "config/real_cameras.json").read_text())
        return [(k, v.get("label", k)) for k, v in cfg.items() if not k.startswith("_")]
    except (OSError, ValueError):
        return []


REAL_CAMERAS = real_camera_list()
DEFAULTS = {
    "target": "isaac", "arms": ["left", "right"], "cameras": ["head", "wrist_left", "wrist_right"],
    "real_cameras": [k for k, _ in REAL_CAMERAS],
    "record": True, "name": "dual_rebel_task", "task": "pick up the box",
    "root": os.environ.get("ROBOT_REMOTE_DATASETS", str(dataset_check.DEFAULT_ROOT)), "resolution": "640x480", "fps": 30, "yaw_deg": 0,
    "left_ip": ENDPOINTS["left"]["host"], "right_ip": ENDPOINTS["right"]["host"], "real_dry_run": True,
    "real_park": True, "real_enable": True, "real_rviz": True, "real_start_home": True,
    "real_accel": "0.52",
    "real_speed": "precise",  # teleop speed preset of the real arms, see REAL_SPEED
    "real_gains": "standard",  # arm controller gains, config/real_controller_gains.json  # --max-joint-acc [rad/s^2] of the real-arm command (identified arm limit ~0.52)
    "sim_rviz": True,  # sim: RViz with Isaac arms vs teleop commands + Quest controllers + lag panel
    "rot_frame": "tool",  # Quest hand rotation applied about the tool's own axes, or about the fixed base axes
}
RESET_COLOR = "#ff7b72"  # Y button (scene reset) in the controller legend
FRAME_COLOR = "#3fb9a8"  # X button (rotation frame tool <-> base) in the controller legend
FRAME_TEXT = {"tool": "Rotation: TOOL frame  (click: base)", "base": "Rotation: BASE frame  (click: tool)"}
C = {"bg": "#1f2329", "panel": "#2a2f37", "fg": "#e8e8e8", "dim": "#9aa3ad", "accent": "#4ea1ff",
     "ok": "#3fb950", "warn": "#d29922", "err": "#f85149", "rec": "#ff3b30"}


def open_manual(page: str = "") -> None:
    """Open the operator manual (docs/operator_manual.html) in the default browser."""
    webbrowser.open((ROOT / "docs" / "operator_manual.html").as_uri() + (f"#{page}" if page else ""))


def apply_style(root):
    st = ttk.Style(root)
    st.theme_use("clam")
    for w in ("TFrame", "TLabelframe"):
        st.configure(w, background=C["bg"])
    st.configure("TLabelframe.Label", background=C["bg"], foreground=C["accent"], font=("DejaVu Sans", 11, "bold"))
    st.configure("TLabel", background=C["bg"], foreground=C["fg"], font=("DejaVu Sans", 10))
    st.configure("Dim.TLabel", foreground=C["dim"])
    st.configure("Big.TLabel", font=("DejaVu Sans", 16, "bold"))
    st.configure("TCheckbutton", background=C["bg"], foreground=C["fg"])
    st.configure("TRadiobutton", background=C["bg"], foreground=C["fg"])
    st.map("TCheckbutton", background=[("active", C["bg"])])
    st.map("TRadiobutton", background=[("active", C["bg"])], foreground=[("disabled", C["dim"])])
    st.configure("Accent.TButton", font=("DejaVu Sans", 12, "bold"), padding=8)


def clear_latest():
    """Empty latest_run/ (only the files the launcher and the teleop write there) before a new session."""
    LATEST_DIR.mkdir(exist_ok=True)
    for f in list(LATEST_DIR.glob("*.log")) + list(LATEST_DIR.glob("*.csv")):
        f.unlink()


def lan_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 9))  # UDP connect sends no packet; it just picks the default interface
            return s.getsockname()[0]
    except OSError:
        return "?"


def load_cfg():
    try:
        source = CONFIG if CONFIG.exists() else LEGACY_CONFIG
        cfg = {**DEFAULTS, **json.loads(source.read_text())}
        if source == LEGACY_CONFIG:
            # Keep old files in place; persist a discoverable project-local copy.
            cfg["real_dry_run"] = True
            CONFIG.parent.mkdir(parents=True, exist_ok=True)
            CONFIG.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n")
        cfg["record"] = True  # Record starts ticked every time (a session without it saved nothing on A, 2026-10-21)
        # former defaults (~/datasets, robot_remote/datasets) and the project folder itself -> robot_remote/dataset
        if cfg["root"].rstrip("/") in (str(Path.home() / "datasets"), str(ROOT / "datasets"), str(ROOT)):
            cfg["root"] = DEFAULTS["root"]
        return cfg
    except (OSError, ValueError):
        return dict(DEFAULTS)


def cpu_sets():
    """Split the CPUs so Isaac, the teleop and the helpers do not compete. Isaac is limited by ONE main thread, so
    it gets the fast cores; on a hybrid CPU (i9-14900K: P-cores 0-15 at 3.2 GHz without turbo, E-cores 16-31 at
    2.4 GHz) Linux otherwise also runs that thread on an E-core. sim = fast cores minus the last two physical cores,
    teleop = those two (IK runs every 33 ms), aux (RViz, robot_state_publishers, recorder, drivers) = slow cores.
    Returns {} (no pinning) when ROBOT_REMOTE_NO_PIN=1 or the machine is too small."""
    if os.environ.get("ROBOT_REMOTE_NO_PIN") == "1":
        return {}
    try:
        cpus = sorted(os.sched_getaffinity(0))
        fmax = {c: int(Path(f"/sys/devices/system/cpu/cpu{c}/cpufreq/cpuinfo_max_freq").read_text()) for c in cpus}
        core = {c: int(Path(f"/sys/devices/system/cpu/cpu{c}/topology/core_id").read_text()) for c in cpus}
    except (OSError, ValueError):
        return {}
    fast = [c for c in cpus if fmax[c] == max(fmax.values())]
    slow = [c for c in cpus if c not in fast]
    fast_cores = sorted({core[c] for c in fast})
    if len(fast_cores) < 4:
        return {}
    teleop_cores = set(fast_cores[-2:])
    teleop = [c for c in fast if core[c] in teleop_cores]
    sim = [c for c in fast if core[c] not in teleop_cores]
    aux = slow or teleop
    fmt = lambda cs: ",".join(map(str, cs))  # noqa: E731
    return {"sim": fmt(sim), "teleop": fmt(teleop), "aux": fmt(aux)}


CPU_SETS = cpu_sets()
CPU_ROLE = {"sim": "sim", "replay_sim": "sim", "teleop": "teleop"}  # everything else: aux


class Proc:
    """A launched script: log file + optional line callback + stdin for the recorder. Pinned to its CPU set
    (cpu_sets(); children inherit it)."""

    def __init__(self, name, cmd, on_line=None, stdin=False, extra_env=None):
        self.name, self.cmd, self.on_line = name, cmd, on_line
        self.cpus = CPU_SETS.get(CPU_ROLE.get(name, "aux"))
        if self.cpus and shutil.which("taskset"):
            cmd = ["taskset", "-c", self.cpus, *cmd]
        LOG_DIR.mkdir(exist_ok=True)
        self.log_path = LOG_DIR / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.log"
        env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH")}
        env.update(extra_env or {})
        self.p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  stdin=subprocess.PIPE if stdin else subprocess.DEVNULL,
                                  text=True, bufsize=1, env=env, start_new_session=True)
        self.lines: list[str] = []
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        LATEST_DIR.mkdir(exist_ok=True)
        with open(self.log_path, "w") as f, open(LATEST_DIR / f"{self.name}.log", "w") as latest:
            if self.cpus:
                f.write(f"[launcher] {self.name} pinned to CPUs {self.cpus}\n")
            latest.write(f"[launcher] {self.name} started {time.strftime('%Y-%m-%d %H:%M:%S')}: {' '.join(self.cmd)}\n"
                         f"[launcher] full copy: {self.log_path}\n")
            for line in self.p.stdout:
                f.write(line)
                f.flush()
                latest.write(line)
                latest.flush()
                self.lines.append(line)
                if self.on_line:
                    self.on_line(self.name, line.rstrip())

    def alive(self):
        return self.p.poll() is None

    def send(self, text):
        if self.p.stdin and self.alive():
            self.p.stdin.write(text + "\n")
            self.p.stdin.flush()

    def interrupt(self):
        if self.alive():
            os.killpg(self.p.pid, signal.SIGINT)

    def kill(self):
        """SIGTERM, then SIGKILL after 3 s (a hung process ignores the polite request)."""
        if self.alive():
            os.killpg(self.p.pid, signal.SIGTERM)
            try:
                self.p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.p.pid, signal.SIGKILL)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("robot_remote — Quest teleop & data recording")
        self.configure(bg=C["bg"])
        # tall enough for the real-arm option rows; never taller than the screen (the bottom row holds Start)
        self.geometry(f"1380x{min(900, self.winfo_screenheight() - 80)}")
        self.cfg = load_cfg()
        self.events: queue.Queue = queue.Queue()
        self.procs: dict[str, Proc] = {}
        self.stage = "idle"
        self._style()
        self.settings_frame = self._build_settings()
        self.run_frame = None
        self.settings_frame.pack(fill="both", expand=True, padx=14, pady=10)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._poll)

    # ------------------------------------------------------------------ look & feel
    def _style(self):
        apply_style(self)

    # ------------------------------------------------------------------ settings page
    def _build_settings(self):
        f = ttk.Frame(self)
        left, right = ttk.Frame(f), ttk.Frame(f)
        right.pack(side="right", fill="y", padx=(14, 0))
        left.pack(side="left", fill="both", expand=True)

        g = ttk.LabelFrame(left, text="1  Target")
        g.pack(fill="x", pady=4)
        self.v_target = tk.StringVar(value=self.cfg["target"])
        ttk.Radiobutton(g, text="Isaac Sim dual-arm simulation", value="isaac", variable=self.v_target).pack(anchor="w", padx=8)
        ttk.Radiobutton(g, text="Real igus ReBeL arms — igus ROS driver over wired Ethernet", value="real",
                        variable=self.v_target).pack(anchor="w", padx=8)
        self.real_box = ttk.Frame(g)
        self.real_box.pack(fill="x", padx=28, pady=(0, 4))
        row = ttk.Frame(self.real_box)
        row.pack(anchor="w")
        self.v_real_ips = {}
        for side in ("left", "right"):
            ttk.Label(row, text=f"{side.title()} IP").pack(side="left", padx=(0, 4))
            self.v_real_ips[side] = tk.StringVar(value=self.cfg[f"{side}_ip"])
            ttk.Entry(row, textvariable=self.v_real_ips[side], width=16).pack(side="left", padx=(0, 14))
        row = ttk.Frame(self.real_box)
        row.pack(anchor="w")
        self.v_real_dry = tk.BooleanVar(value=self.cfg["real_dry_run"])
        ttk.Checkbutton(row, text="Dry run (read status + Quest IK; no motion or configuration)",
                        variable=self.v_real_dry).pack(side="left")
        self.v_real_enable = tk.BooleanVar(value=self.cfg["real_enable"])
        ttk.Checkbutton(row, text="Enable motors at start (Reset + Enable)",
                        variable=self.v_real_enable).pack(side="left", padx=14)
        self.v_real_rviz = tk.BooleanVar(value=self.cfg["real_rviz"])
        row = ttk.Frame(self.real_box)  # second row: RViz + the automatic moves at start / exit
        row.pack(anchor="w")
        ttk.Checkbutton(row, text="RViz: live arms + Quest controllers", variable=self.v_real_rviz).pack(side="left")
        self.v_real_park = tk.BooleanVar(value=self.cfg["real_park"])
        self.v_real_start_home = tk.BooleanVar(value=self.cfg["real_start_home"])
        ttk.Checkbutton(row, text="Home (0, 0, 65, 0, 50, 0) deg after connecting",
                        variable=self.v_real_start_home).pack(side="left", padx=14)
        ttk.Checkbutton(row, text="All joints 0 deg before exit",
                        variable=self.v_real_park).pack(side="left", padx=14)
        row = ttk.Frame(self.real_box)
        row.pack(anchor="w")
        ttk.Label(row, text="Speed").pack(side="left")
        self.v_real_speed = tk.StringVar(value=self.cfg["real_speed"] if self.cfg["real_speed"] in REAL_SPEED else "precise")
        ttk.Combobox(row, textvariable=self.v_real_speed, values=list(REAL_SPEED), width=8,
                     state="readonly").pack(side="left", padx=(6, 4))
        ttk.Label(row, text="(precise 15°/s: stops ~1 cm after release; normal 30°/s: faster, slides further)",
                  style="Dim.TLabel").pack(side="left", padx=(0, 14))
        row = ttk.Frame(self.real_box)  # controller gains + command acceleration
        row.pack(anchor="w")
        gains = json.loads((ROOT / "config/real_controller_gains.json").read_text())
        self.real_gains = {k: v for k, v in gains.items() if not k.startswith("_")}
        ttk.Label(row, text="Arm gains").pack(side="left")
        self.v_real_gains = tk.StringVar(value=self.cfg["real_gains"] if self.cfg["real_gains"] in self.real_gains else "standard")
        ttk.Combobox(row, textvariable=self.v_real_gains, values=list(self.real_gains), width=10,
                     state="readonly").pack(side="left", padx=(6, 4))
        ttk.Label(row, text="(responsive: p2/ff0.8 on J2-J5, tighter tracking)",
                  style="Dim.TLabel").pack(side="left", padx=(0, 14))
        ttk.Label(row, text="Joint acceleration [rad/s²]").pack(side="left")
        self.v_real_accel = tk.StringVar(value=str(self.cfg["real_accel"]) if str(self.cfg["real_accel"]) in ("0.45", "0.52")
                                         else "0.52")
        ttk.Combobox(row, textvariable=self.v_real_accel, values=["0.45", "0.52"], width=5,
                     state="readonly").pack(side="left", padx=6)
        ttk.Label(self.real_box, text="Starts the igus ROS driver per arm, then the teleop on its topics.",
                  style="Dim.TLabel").pack(anchor="w")
        self.v_target.trace_add("write", lambda *_: self._target_changed())

        g = ttk.LabelFrame(left, text="2  Arms to control")
        g.pack(fill="x", pady=4)
        self.v_arms = {s: tk.BooleanVar(value=s in self.cfg["arms"]) for s in ("left", "right")}
        row = ttk.Frame(g)
        row.pack(anchor="w", padx=8, pady=4)
        ttk.Checkbutton(row, text="Left arm  <-  left controller", variable=self.v_arms["left"]).pack(side="left", padx=(0, 20))
        ttk.Checkbutton(row, text="Right arm  <-  right controller", variable=self.v_arms["right"]).pack(side="left")

        g = ttk.LabelFrame(left, text="3  Cameras (sim: rendered by Isaac; real: USB cameras) - recorded")
        g.pack(fill="x", pady=4)
        cam_box = g
        self.v_cams = {k: tk.BooleanVar(value=k in self.cfg["cameras"]) for k, _ in CAMERAS}
        self.sim_cam_box = row = ttk.Frame(g)
        row.pack(anchor="w", padx=8, pady=4)
        ttk.Label(row, text="Sim:", width=5).pack(side="left")
        for k, label in CAMERAS:
            ttk.Checkbutton(row, text=label, variable=self.v_cams[k]).pack(side="left", padx=(0, 18))
        self.v_real_cams = {k: tk.BooleanVar(value=k in self.cfg["real_cameras"]) for k, _ in REAL_CAMERAS}
        self.real_cam_box = row = ttk.Frame(g)
        row.pack(anchor="w", padx=8, pady=(0, 4))
        ttk.Label(row, text="Real:", width=5).pack(side="left")
        for k, label in REAL_CAMERAS:
            ttk.Checkbutton(row, text=label.split(" (")[0], variable=self.v_real_cams[k]).pack(side="left", padx=(0, 14))
        if not REAL_CAMERAS:
            ttk.Label(row, text="no config/real_cameras.json", style="Dim.TLabel").pack(side="left")
        self.sim_res_box = row = ttk.Frame(g)
        row.pack(anchor="w", padx=8, pady=(0, 4))
        ttk.Label(row, text="Resolution").pack(side="left")
        self.v_res = tk.StringVar(value=self.cfg["resolution"])
        ttk.Combobox(row, textvariable=self.v_res, values=["640x480", "320x240", "848x480"], width=9,
                     state="readonly").pack(side="left", padx=6)
        ttk.Label(row, text="FPS").pack(side="left", padx=(12, 0))
        self.v_fps = tk.StringVar(value=str(self.cfg["fps"]))
        ttk.Combobox(row, textvariable=self.v_fps, values=["30", "15", "10"], width=4, state="readonly").pack(side="left", padx=6)
        ttk.Label(row, text="Sim: more cameras = slower sim. Real: size from config/real_cameras.json",
                  style="Dim.TLabel").pack(side="left", padx=12)
        self.v_sim_rviz = tk.BooleanVar(value=self.cfg["sim_rviz"])
        self.sim_rviz_cb = ttk.Checkbutton(g, text="RViz (sim): Isaac arms vs teleop commands, Quest controllers, lag panel "
                                   "(Quest link or simulator?)", variable=self.v_sim_rviz)
        self.sim_rviz_cb.pack(anchor="w", padx=8)

        g = ttk.LabelFrame(left, text="4  Data recording")
        g.pack(fill="x", pady=4)
        self.v_record = tk.BooleanVar(value=self.cfg["record"])
        ttk.Checkbutton(g, text="Record a LeRobot dataset (unchecked = teleop only)", variable=self.v_record).pack(anchor="w", padx=8, pady=(4, 2))
        grid = ttk.Frame(g)
        grid.pack(fill="x", padx=8, pady=4)
        self.v_name, self.v_task, self.v_root = (tk.StringVar(value=self.cfg[k]) for k in ("name", "task", "root"))
        for r, (label, var, hint) in enumerate([
            ("Task (language instruction)", self.v_task, "e.g. pick up the box and place it on the table"),
            ("Dataset name", self.v_name, "an existing name appends to that dataset"),
        ]):
            ttk.Label(grid, text=label).grid(row=r, column=0, sticky="w", pady=3)
            ttk.Entry(grid, textvariable=var, width=48).grid(row=r, column=1, sticky="we", padx=6)
            ttk.Label(grid, text=hint, style="Dim.TLabel").grid(row=r, column=2, sticky="w")
        ttk.Label(grid, text="Save location").grid(row=2, column=0, sticky="w", pady=3)
        ttk.Entry(grid, textvariable=self.v_root, width=48).grid(row=2, column=1, sticky="we", padx=6)
        ttk.Button(grid, text="Browse...", command=self._browse).grid(row=2, column=2, sticky="w")
        self.v_path_hint = ttk.Label(grid, style="Dim.TLabel")
        self.v_path_hint.grid(row=3, column=1, columnspan=2, sticky="w")
        for v in (self.v_name, self.v_root, self.v_target, self.v_res, self.v_fps, self.v_record,
                  *self.v_cams.values(), *self.v_real_cams.values()):
            v.trace_add("write", lambda *_: self._update_path_hint())
        self._update_path_hint()
        self.real_only_widgets = self._descendants(self.real_cam_box)
        self.sim_only_widgets = self._descendants(self.sim_cam_box) + [self.sim_rviz_cb]
        self.sim_only_widgets += [w for w in self._descendants(self.sim_res_box) if isinstance(w, ttk.Combobox)
                                  and w.cget("values") and "640x480" in w.cget("values")]  # resolution (sim only)

        g = ttk.LabelFrame(left, text="5  Quest connection")
        g.pack(fill="x", pady=4)
        ip = lan_ip()
        ttk.Label(g, text=f"Open in the Quest Browser:  {QUEST_URL}").pack(anchor="w", padx=8, pady=(4, 0))
        ttk.Label(g, text=f"Server IP:  {ip}", style="Big.TLabel").pack(anchor="w", padx=8, pady=(2, 0))
        ttk.Label(g, text="First tap the page's \"accept cert\" link and accept the certificate, then come back and press Connect",
                  style="Dim.TLabel").pack(anchor="w", padx=8, pady=(0, 6))
        row = ttk.Frame(g)
        row.pack(anchor="w", padx=8, pady=(0, 6))
        ttk.Label(row, text="Stance").pack(side="left")
        self.v_yaw = tk.StringVar(value=str(self.cfg["yaw_deg"]))
        ttk.Radiobutton(row, text="Facing the same way as the arms (recommended)", value="0", variable=self.v_yaw).pack(side="left", padx=8)
        ttk.Radiobutton(row, text="Facing the robot", value="180", variable=self.v_yaw).pack(side="left")

        bar = ttk.Frame(left)
        bar.pack(fill="x", pady=10)
        ttk.Button(bar, text="▶  Start", style="Accent.TButton", command=self.start).pack(side="right")
        ttk.Button(bar, text="? Manual", command=open_manual).pack(side="right", padx=8)
        ttk.Button(bar, text="⏵ Play back a dataset...", command=self.open_replay).pack(side="left")

        ttk.Label(right, text="Quest controller", style="Big.TLabel").pack(anchor="w")
        self._legend(right).pack(pady=6)
        self.after(0, self._target_changed)
        return f

    @staticmethod
    def _descendants(w):
        out = []
        for c in w.winfo_children():
            out.append(c)
            out += App._descendants(c)
        return out

    def _target_changed(self):
        real = self.v_target.get() == "real"
        for w in self._descendants(self.real_box):
            try:
                w.configure(state="normal" if real else "disabled")
            except tk.TclError:
                pass
        for widgets, on in ((self.sim_only_widgets, not real), (self.real_only_widgets, real)):
            for w in widgets:
                try:
                    w.configure(state=("readonly" if isinstance(w, ttk.Combobox) else "normal") if on else "disabled")
                except tk.TclError:
                    pass

    def open_replay(self):
        from replay_window import ReplayWindow
        d = Path(self.v_root.get()).expanduser() / self.v_name.get()
        ReplayWindow(self, str(d) if (d / "meta" / "info.json").is_file() else None)

    def _browse(self):
        d = filedialog.askdirectory(initialdir=self.v_root.get() or str(dataset_check.DEFAULT_ROOT),
                                    title="Save location (the folder that holds the datasets, or a dataset to continue)")
        if not d:
            return
        if (Path(d) / "meta" / "info.json").is_file():  # picked a dataset itself: continue it
            self.v_root.set(str(Path(d).parent))
            self.v_name.set(Path(d).name)
        else:
            self.v_root.set(d)

    def _session_shape(self):
        """(source, fps, {camera: (height, width)}) the recorder would use with the current settings."""
        real = self.v_target.get() == "real"
        if real:
            try:
                cams = json.loads((ROOT / "config/real_cameras.json").read_text())
            except (OSError, ValueError):
                cams = {}
            shapes = {}
            for k, v in self.v_real_cams.items():
                if v.get() and k in cams:
                    w, h = int(cams[k].get("width", 640)), int(cams[k].get("height", 480))
                    shapes[k] = (w, h) if int(cams[k].get("rotate", 0)) in (90, 270) else (h, w)
        else:
            w, h = (int(x) for x in self.v_res.get().split("x"))
            shapes = {k: (h, w) for k, v in self.v_cams.items() if v.get()}
        return ("real" if real else "sim"), int(self.v_fps.get()), shapes

    def _dataset_verdict(self):
        """(ok, text) for the dataset at save location / name with the current settings."""
        st = dataset_check.inspect(Path(self.v_root.get()).expanduser() / self.v_name.get())
        problems = dataset_check.compatible(st["info"], *self._session_shape()) if st["state"] == "dataset" else []
        return dataset_check.describe(st, problems)

    def _update_path_hint(self):
        if not hasattr(self, "v_real_cams") or not hasattr(self, "v_path_hint"):
            return  # still building the page
        p = Path(self.v_root.get()).expanduser() / self.v_name.get()
        try:
            ok, text = self._dataset_verdict()
        except (ValueError, OSError) as exc:
            ok, text = False, str(exc)
        self.v_path_hint.configure(text=f"→ {p}\n   {text}", foreground=C["dim"] if ok else C["err"])

    # ------------------------------------------------------------------ controller legend
    def _legend(self, parent):
        cv = tk.Canvas(parent, width=400, height=560, bg=C["panel"], highlightthickness=0)

        def controller(cx, title, top, bottom, bottom_color="#d2a8ff", top_color="#58a6ff"):
            cv.create_text(cx, 22, text=title, fill=C["fg"], font=("DejaVu Sans", 12, "bold"))
            cv.create_oval(cx - 70, 40, cx + 70, 150, outline="#8b949e", width=3)        # tracking ring
            cv.create_rectangle(cx - 28, 110, cx + 28, 270, outline="#8b949e", width=3)   # handle
            cv.create_oval(cx - 45, 70, cx + 45, 130, fill="#30363d", outline="#8b949e", width=2)  # face
            cv.create_oval(cx - 30, 78, cx - 6, 102, fill=top_color, outline="")          # face button 1
            cv.create_text(cx - 18, 90, text=top[0], fill="white", font=("Arial", 11, "bold"))
            cv.create_oval(cx + 6, 92, cx + 30, 116, fill=bottom_color, outline="")       # face button 2
            cv.create_text(cx + 18, 104, text=bottom[0], fill="white", font=("Arial", 11, "bold"))
            cv.create_rectangle(cx - 33, 150, cx - 22, 210, fill=C["ok"], outline="")       # grip
            cv.create_polygon(cx + 28, 120, cx + 44, 132, cx + 30, 160, fill=C["warn"], outline="")  # trigger

        controller(105, "Left -> left arm", ("X",), ("Y",), RESET_COLOR, FRAME_COLOR)
        controller(295, "Right -> right arm", ("A",), ("B",))
        items = [
            (C["ok"], "Grip (middle finger) - hold", "the arm follows your hand; release = freeze.\nRelease, reposition your hand, grip again - the arm does not jump"),
            (C["warn"], "Trigger (index finger)", "closes the gripper (deeper = tighter)"),
            ("#58a6ff", "A (right)", "start recording -> press again to save the episode"),
            (FRAME_COLOR, "X (left) - rotation frame", "switch TOOL <-> BASE (applies at the next grip)"),
            ("#d2a8ff", "B (right)", "discard the current episode and re-record"),
            (RESET_COLOR, "Y (left) - reset scene", "discards an open episode; arms go home (sim: box flaps close;\n"
                                                   "real: 10 deg/s, any button / grip stops them)"),
            ("#8b949e", "Rotate the controller", "twist the handle = spin the tool; nod = tilt the tool"),
        ]
        y = 300
        for color, key, desc in items:
            cv.create_rectangle(20, y + 4, 36, y + 20, fill=color, outline="")
            k = cv.create_text(46, y, text=key, anchor="nw", fill=C["fg"], font=("DejaVu Sans", 11, "bold"))
            d = cv.create_text(46, cv.bbox(k)[3] + 2, text=desc, anchor="nw", fill=C["dim"],
                               font=("DejaVu Sans", 10), width=340)
            y = cv.bbox(d)[3] + 12
        cv.configure(height=y + 6)
        return cv

    # ------------------------------------------------------------------ start / run page
    def _collect(self):
        arms = [s for s, v in self.v_arms.items() if v.get()]
        cams = [k for k, v in self.v_cams.items() if v.get()]
        real_cams = [k for k, v in self.v_real_cams.items() if v.get()]
        w, h = self.v_res.get().split("x")
        real = self.v_target.get() == "real"
        cfg = {"target": self.v_target.get(), "arms": arms, "cameras": cams, "real_cameras": real_cams,
               "record": self.v_record.get(),
               "name": self.v_name.get().strip(), "task": self.v_task.get().strip(), "root": self.v_root.get().strip(),
               "resolution": self.v_res.get(), "fps": int(self.v_fps.get()), "yaw_deg": int(self.v_yaw.get()),
               "width": int(w), "height": int(h),
               "left_ip": self.v_real_ips["left"].get().strip(),
               "right_ip": self.v_real_ips["right"].get().strip(), "real_dry_run": self.v_real_dry.get(),
               "real_park": self.v_real_park.get(), "real_enable": self.v_real_enable.get(),
               "real_rviz": self.v_real_rviz.get(), "real_start_home": self.v_real_start_home.get(),
               "real_accel": self.v_real_accel.get(), "real_speed": self.v_real_speed.get(),
               "real_gains": self.v_real_gains.get(),
               "sim_rviz": self.v_sim_rviz.get(),
               "rot_frame": self.cfg.get("rot_frame", "tool")}  # set on the run page (button), kept for next time
        if real:
            try:
                for side in ("left", "right"):
                    socket.inet_pton(socket.AF_INET, cfg[f"{side}_ip"])
            except OSError as exc:
                raise ValueError("Enter valid left / right IPv4 addresses") from exc
            if cfg["left_ip"] == cfg["right_ip"]:
                raise ValueError("Left and right must have different IP addresses")
        if not arms:
            raise ValueError("Select at least one arm")
        if cfg["record"]:
            if not cfg["task"]:
                raise ValueError("Please enter a task description")
            if not re.fullmatch(r"[A-Za-z0-9_.\-]+", cfg["name"] or ""):
                raise ValueError("Dataset name: letters, digits, _ . - only")
            ok, text = self._dataset_verdict()
            if not ok:
                raise ValueError(f"Dataset {Path(cfg['root']).expanduser() / cfg['name']}:\n{text}")
        return cfg

    def start(self):
        try:
            cfg = self._collect()
        except ValueError as e:
            messagebox.showerror("Invalid settings", str(e))
            return
        if cfg["target"] == "real" and not cfg["real_dry_run"] and not messagebox.askyesno(
                "Real robot", f"You are about to drive REAL arms: {', '.join(cfg['arms'])}.\n"
                f"Left {cfg['left_ip']}; right {cfg['right_ip']}.\n\n"
                "- Someone stands at the e-stop.\n- The workspace is clear of people and objects.\n"
                "- Other robot drivers / control clients are closed.\n"
                + ("- Motors that report 'not enabled' are ENABLED now (Reset + Enable); the arms then hold.\n"
                   if cfg["real_enable"] else "- Motors are already enabled (iRC / robot controls).\n")
                + "- The path between straight up and home is clear. Real grippers are not controlled.\n"
                + "- Y (left controller) / Arms home: both arms move ON THEIR OWN back to home at 10 deg/s;\n"
                  "  any button or grip stops them.\n"
                + ("- After connecting the arms move ON THEIR OWN to home (0, 0, 65, 0, 50, 0) deg, 10 deg/s.\n"
                   if cfg["real_start_home"] else "- The arms start where they are (all 0 = straight up is hard to control).\n")
                + ("- On Stop all the arms move ON THEIR OWN back to all joints 0 deg, 10 deg/s.\n"
                   "  Press Stop all again to stop them where they are.\n" if cfg["real_park"] else "")
                + 
                "\nEach arm only moves while you hold its GRIP. Continue?", icon="warning"):
            return
        CONFIG.parent.mkdir(parents=True, exist_ok=True)
        CONFIG.write_text(json.dumps({k: v for k, v in cfg.items() if k in DEFAULTS}, ensure_ascii=False, indent=1))
        clear_latest()
        self.cfg = cfg
        self.settings_frame.pack_forget()
        self.run_frame = self._build_run()
        self.run_frame.pack(fill="both", expand=True, padx=14, pady=10)
        threading.Thread(target=self._launch_sequence, daemon=True).start()

    def _build_run(self):
        f = ttk.Frame(self)
        left, right = ttk.Frame(f), ttk.Frame(f)
        right.pack(side="right", fill="y", padx=(14, 0))
        left.pack(side="left", fill="both", expand=True)
        cfg = self.cfg
        self.rec_label = tk.Label(left, text="Starting...", bg=C["panel"], fg=C["fg"],
                                  font=("DejaVu Sans", 22, "bold"), pady=14)
        self.rec_label.pack(fill="x")
        info = (f"Task: {cfg['task']}\nDataset: {Path(cfg['root']).expanduser() / cfg['name']}" if cfg["record"]
                else "Teleop only (not recording)")
        real = cfg["target"] == "real"
        cams = cfg["real_cameras"] if real else cfg["cameras"]
        ttk.Label(left, text=info + f"\nArms: {', '.join(cfg['arms'])}    Cameras: {', '.join(cams) or 'none'}"
                  + (f"    @ {cfg['fps']} fps (USB cameras)" if real else f"    {cfg['resolution']} @ {cfg['fps']} fps"),
                  style="Dim.TLabel").pack(anchor="w", pady=6)
        g = ttk.LabelFrame(left, text="Processes")
        g.pack(fill="x", pady=4)
        self.status = {}
        sim_label = "Real robot link" if cfg["target"] == "real" else "Isaac simulation"
        for key, label in (("sim", sim_label), ("teleop", "Quest teleop (CloudXR)"), ("record", "Data recorder")):
            row = ttk.Frame(g)
            row.pack(fill="x", padx=8, pady=2)
            dot = tk.Label(row, text="●", fg=C["dim"], bg=C["bg"], font=("Arial", 14))
            dot.pack(side="left")
            ttk.Label(row, text=label, width=22).pack(side="left")
            txt = ttk.Label(row, text="waiting", style="Dim.TLabel")
            txt.pack(side="left")
            self.status[key] = (dot, txt)
        ttk.Label(left, text=f"Quest Browser: {QUEST_URL}    Server IP: {lan_ip()}", style="Dim.TLabel").pack(anchor="w", pady=4)
        btns = ttk.Frame(left)
        btns.pack(fill="x", pady=6)
        self.b_toggle = ttk.Button(btns, text="● Start / Save  (= A)", command=lambda: self._rec_cmd(""))
        self.b_discard = ttk.Button(btns, text="✕ Discard  (= B)", command=lambda: self._rec_cmd("d"))
        if cfg["record"]:
            self.b_toggle.pack(side="left", padx=(0, 8))
            self.b_discard.pack(side="left", padx=(0, 8))
        # live switch of the rotation frame; enabled once the teleop reports its frame (it applies at the next grip)
        self.b_frame = ttk.Button(btns, text=FRAME_TEXT[cfg["rot_frame"]], command=self._toggle_frame, state="disabled")
        self.b_frame.pack(side="left", padx=(0, 8))
        # sim: arms home + box flaps closed; real: arms slowly home (both: an open episode is discarded)
        ttk.Button(btns, text="⟲ Arms home  (= Y)" if cfg["target"] == "real" else "⟲ Reset scene  (= Y)",
                   command=self._reset_scene).pack(side="left", padx=(0, 8))
        ttk.Button(btns, text="■ Stop all", command=self.stop_all).pack(side="right")
        ttk.Button(btns, text="? Manual", command=open_manual).pack(side="right")
        ttk.Button(btns, text="Open log folder", command=lambda: subprocess.Popen(["xdg-open", str(LOG_DIR)])).pack(side="right", padx=8)
        g = ttk.LabelFrame(left, text="Messages")
        g.pack(fill="both", expand=True, pady=4)
        self.log = tk.Text(g, height=14, bg=C["panel"], fg=C["fg"], insertbackground=C["fg"], relief="flat",
                           font=("DejaVu Sans Mono", 9))
        self.log.pack(fill="both", expand=True, padx=4, pady=4)
        ttk.Label(right, text="Quest controller", style="Big.TLabel").pack(anchor="w")
        self._legend(right).pack(pady=6)
        return f

    # ------------------------------------------------------------------ process orchestration
    def _set(self, key, color, text):
        self.events.put(("status", key, color, text))

    def _msg(self, text):
        self.events.put(("log", text))

    def _wait_for(self, proc, pattern, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if any(re.search(pattern, ln) for ln in proc.lines):
                return True
            if not proc.alive():
                return False
            time.sleep(0.5)
        return False

    def _launch_real(self):
        cfg = self.cfg
        self._set("sim", C["dim"], "not used (real robot)")
        self._set("record", C["dim"], "starts after the arms connect" if cfg["record"] else "disabled")
        # 1) the igus ROS driver of each selected arm (real_robot/dual_arm_driver.launch.py), motors enabled
        mock = os.environ.get("ROBOT_REMOTE_DRIVER_MOCK") == "1"  # automated tests: no robot
        enable = cfg["real_enable"] and not cfg["real_dry_run"] and not mock
        launch = (f"source /opt/ros/humble/setup.bash && source '{ROOT}/igus_ws/install/setup.bash' && "
                  f"exec ros2 launch '{ROOT}/real_robot/dual_arm_driver.launch.py' arms:={','.join(cfg['arms'])} "
                  f"gains:={cfg.get('real_gains', 'standard')} "
                  f"enable_motors:={'true' if enable else 'false'} left_ip:={cfg['left_ip']} right_ip:={cfg['right_ip']}"
                  + (" hardware_protocol:=mock_hardware" if mock else ""))
        self._set("sim", C["warn"], "starting the igus ROS driver(s)...")
        self.procs["driver"] = Proc("driver", ["bash", "-c", launch], self._on_line)
        d, want = self.procs["driver"], len(cfg["arms"])
        pat = "success=True" if enable else "Configured and activated.*rebel_arm_trajectory_controller"
        end = time.monotonic() + 45
        while time.monotonic() < end and d.alive():
            if sum(bool(re.search(pat, ln)) for ln in d.lines) >= want or any("success=False" in ln for ln in d.lines):
                break
            time.sleep(0.5)
        if any("success=False" in ln for ln in d.lines) or sum(bool(re.search(pat, ln)) for ln in d.lines) < want:
            self._set("sim", C["err"], f"driver / motor enable failed - e-stop? arm on? log: {d.log_path.name}")
            return
        self._set("sim", C["ok"], "driver up" + (", motors enabled" if enable else ""))
        # 2) the Quest teleop, streaming to the drivers' trajectory controllers (--backend real-ros)
        cmd = [str(ROOT / "quest_teleop" / "run_quest_real_arm.sh"), "--arms", *cfg["arms"],
               "--yaw-deg", str(cfg["yaw_deg"]), "--rot-frame", cfg["rot_frame"], "--quiet",
               "--max-joint-acc", str(cfg.get("real_accel", "0.52")), *REAL_SPEED.get(cfg.get("real_speed"), [])]
        if cfg["yaw_deg"] == 180:
            cmd.append("--swap-hands")
        if cfg["real_dry_run"]:
            cmd.append("--dry-run")
        if not cfg["real_park"]:
            cmd.append("--no-park")
        if not cfg["real_start_home"]:
            cmd.append("--no-start-home")
        self._set("teleop", C["warn"], "connecting to the drivers, then moving to home...")
        brake = self.real_gains_table().get(cfg.get("real_gains"), {}).get("brake_delay_s")
        self.procs["teleop"] = Proc("teleop", cmd, self._on_line, stdin=True,
                                    extra_env={"ROBOT_REMOTE_BRAKE_DELAY": str(brake)} if brake else None)
        p = self.procs["teleop"]
        if not self._wait_for(p, r"real arm connected", 60):
            p.interrupt()
            self._set("teleop", C["err"], f"no joint states from the driver(s); log: {p.log_path.name}")
            return
        self._set("sim", C["ok"], "robot connected")
        if cfg["record"] and not self._start_recorder(["--source", "real", "--cameras", *cfg["real_cameras"]],
                                                      "opening the USB cameras..."):
            return
        if cfg["real_rviz"]:
            self._msg("[robot] opening RViz (live arms + Quest controllers)")
            self.procs["viz"] = Proc("viz", [str(ROOT / "quest_teleop" / "run_teleop_viz.sh"), "--source", "real",
                                             "--yaw-deg", str(cfg["yaw_deg"])])
        if not self._wait_for(p, r"CloudXR up", 180):
            p.interrupt()
            self._set("teleop", C["err"], f"failed to start - see log: {p.log_path}")
            return
        self._set("teleop", C["warn"], f"waiting for the Quest (Server IP {lan_ip()})")
        self.events.put(("rec", "idle", 0) if cfg["record"] else ("rec", "real", 0))

    @staticmethod
    def real_gains_table():
        try:
            return {k: v for k, v in json.loads((ROOT / "config/real_controller_gains.json").read_text()).items()
                    if not k.startswith("_")}
        except (OSError, ValueError):
            return {}

    def _start_recorder(self, extra, starting="starting..."):
        """run_record.sh with the dataset settings + extra args; True once it is ready for A."""
        cfg = self.cfg
        rec_cmd = [str(ROOT / "recording" / "run_record.sh"), "--name", cfg["name"], "--task", cfg["task"],
                   "--root", cfg["root"], "--fps", str(cfg["fps"]), *extra]
        self._set("record", C["warn"], starting)
        self.procs["record"] = Proc("record", rec_cmd, self._on_line, stdin=True)
        if not self._wait_for(self.procs["record"], r"\[rec\] controls", 120):
            why = next((ln for ln in reversed(self.procs["record"].lines) if "ERROR" in ln or "Error:" in ln), "")
            self._set("record", C["err"], f"failed to start - {why[-120:] or 'see log'}: {self.procs['record'].log_path.name}")
            return False
        self._set("record", C["ok"], "ready (press A to record)")
        return True

    def _launch_sequence(self):
        cfg = self.cfg
        if cfg["target"] == "real":
            return self._launch_real()
        sim_cmd = [str(ROOT / "dual_arm_sim" / "run_dual_arm_sim.sh"), "--cam-width", str(cfg["width"]),
                   "--cam-height", str(cfg["height"])]
        if cfg["cameras"]:
            sim_cmd += ["--cameras", *cfg["cameras"]]
        if os.environ.get("ROBOT_REMOTE_SIM_HEADLESS") == "1":  # for automated tests
            sim_cmd.append("--headless")
        self._set("sim", C["warn"], "starting (30-60 s, the Isaac window opens)...")
        self.procs["sim"] = Proc("sim", sim_cmd, self._on_line)
        if not self._wait_for(self.procs["sim"], r"\[dual_arm\] running", 300):
            self._set("sim", C["err"], f"failed to start - see log: {self.procs['sim'].log_path}")
            return
        self._set("sim", C["ok"], "running")

        tele_cmd = [str(ROOT / "quest_teleop" / "run_quest_dual_arm.sh"), "--arms", *cfg["arms"],
                    "--yaw-deg", str(cfg["yaw_deg"]), "--rot-frame", cfg["rot_frame"], "--quiet"]
        if cfg["yaw_deg"] == 180:
            tele_cmd.append("--swap-hands")
        self._set("teleop", C["warn"], "starting (arms go home, CloudXR ~10 s)...")
        self.procs["teleop"] = Proc("teleop", tele_cmd, self._on_line, stdin=True)
        if not self._wait_for(self.procs["teleop"], r"CloudXR up", 120):
            self._set("teleop", C["err"], f"failed to start - see log: {self.procs['teleop'].log_path}")
            return
        self._set("teleop", C["warn"], f"waiting for the Quest (Server IP {lan_ip()})")
        if cfg.get("sim_rviz"):
            self._msg("[teleop] opening RViz: solid = Isaac arms, ghost = teleop command, spheres = Quest controllers")
            self.procs["viz"] = Proc("viz", [str(ROOT / "quest_teleop" / "run_teleop_viz.sh"), "--source", "isaac",
                                             "--yaw-deg", str(cfg["yaw_deg"])])

        if cfg["record"]:
            if not self._start_recorder(["--width", str(cfg["width"]), "--height", str(cfg["height"]),
                                         "--cameras", *cfg["cameras"]]):
                return
            self.events.put(("rec", "idle", 0))
        else:
            self._set("record", C["dim"], "disabled")
            self.events.put(("rec", "teleop", 0))

    def _on_line(self, name, line):
        if name == "driver" and any(k in line for k in ("Connected to ReBeL", "success=", "did not accept",
                                                         "still report", "Hardware error")):
            self._msg(f"[driver] {line.split(']: ')[-1][:200]}")
        if name == "teleop" and any(k in line for k in ("REAL ARM", "real arm connected", "other publisher", "let go", "RobotError", "[park]", "[home]", "motors enabled", "watching the", "assumed to stay", "apart (surface", "guard blind",
                                                         "start pose", "dry run", "[reset]")):
            self._msg(f"[robot] {line.split(']: ')[-1]}")
        if name == "teleop" and "button -> toggle" in line and not self.cfg.get("record"):
            self._msg("[record] A pressed, but recording is OFF this session - nothing is saved. Stop all, tick "
                      "'Record a LeRobot dataset' (section 4) and Start again.")
            self.events.put(("rec", "not_recording", None))
        if name == "teleop" and (m := re.search(r"rotation frame: (tool|base)\b", line)):
            self.events.put(("frame", m.group(1)))
            self._msg(f"[teleop] {line.split(']: ')[-1]}")
        if name == "teleop":
            if "TRACKED" in line and "LOST" not in line:
                self._set("teleop", C["ok"], "Quest connected, controllers tracked")
            elif "tracking LOST" in line:
                self._set("teleop", C["warn"], "controller tracking lost (keep it in front of the headset)")
            if any(k in line for k in ("engaged", "released", "button", "TRACKED", "LOST", "apart", "Error", "Traceback")) \
                    or ("[timing]" in line and "WARN" in line):  # loop / sim too slow (see logs/teleop_*.log)
                self._msg(f"[teleop] {line.split(']: ')[-1]}")
        elif name == "record":
            if "RECORDING" in line:
                self.events.put(("rec", "recording", None))
            elif "saving episode" in line:
                self.events.put(("rec", "saving", None))
            elif "saved. total episodes" in line:
                n = int(re.search(r"total episodes: (\d+)", line).group(1))
                self.events.put(("rec", "idle", n))
            elif "discarded" in line:
                self.events.put(("rec", "idle", None))
            elif m := re.search(r"● (\d+) frames \(([\d.]+)s (?:sim|real)\)", line):
                self.events.put(("rec", "frames", (int(m.group(1)), float(m.group(2)), "stale" in line)))
            if line.startswith("[rec]") and "● " not in line or "WARNING" in line or "cannot start" in line:
                self._msg(line)
        elif name == "sim" and ("Traceback" in line or "Error" in line and "[Error]" not in line):
            self._msg(f"[sim] {line}")

    def _toggle_frame(self):
        """Ask the running teleop to switch the rotation frame; the label follows its confirmation line."""
        new = "base" if self.cfg["rot_frame"] == "tool" else "tool"
        p = self.procs.get("teleop")
        if p is None or not p.alive():
            return
        try:
            p.send(f"rot_frame {new}")
        except OSError:  # the teleop just exited
            return
        self.b_frame.configure(state="disabled")  # until the teleop confirms
        try:  # remember it for the next start
            saved = json.loads(CONFIG.read_text()) if CONFIG.exists() else {}
            CONFIG.write_text(json.dumps({**saved, "rot_frame": new}, ensure_ascii=False, indent=1))
        except (OSError, ValueError):
            pass

    def _reset_scene(self):
        p = self.procs.get("teleop")
        if p is not None and p.alive():
            try:
                p.send("reset_scene")
            except OSError:  # the teleop just exited
                pass

    def _rec_cmd(self, c):
        if "record" in self.procs:
            self.procs["record"].send(c)

    def _poll(self):
        while not self.events.empty():
            ev = self.events.get()
            if ev[0] == "status":
                _, key, color, text = ev
                dot, txt = self.status[key]
                dot.configure(fg=color)
                txt.configure(text=text)
            elif ev[0] == "log":
                self.log.insert("end", ev[1] + "\n")
                self.log.see("end")
            elif ev[0] == "rec":
                self._rec_state(ev[1], ev[2])
            elif ev[0] == "frame":  # the teleop confirmed a frame (GUI button or the Quest X button)
                self.cfg["rot_frame"] = ev[1]
                self.b_frame.configure(text=FRAME_TEXT[ev[1]], state="normal")
                try:  # remember it for the next start
                    saved = json.loads(CONFIG.read_text()) if CONFIG.exists() else {}
                    if saved.get("rot_frame") != ev[1]:
                        CONFIG.write_text(json.dumps({**saved, "rot_frame": ev[1]}, ensure_ascii=False, indent=1))
                except (OSError, ValueError):
                    pass
        for key, p in list(self.procs.items()):
            if key in self.status and not p.alive() and self.stage != "stopping" and getattr(p, "_reported", False) is False:
                p._reported = True
                self._set(key, C["err"], f"exited (code {p.p.returncode}), log: {p.log_path.name}")
        self.after(100, self._poll)

    def _rec_state(self, state, data):
        if state == "recording":
            self.rec_label.configure(text="● RECORDING   A = save,  B = discard,  Y = " +
                                     ("arms home (discards)" if self.cfg.get("target") == "real" else "reset scene (discards)"),
                                     bg=C["rec"])
            self._set("record", C["rec"], "recording")
        elif state == "frames":
            n, sec, stale = data
            self.rec_label.configure(text=f"● RECORDING  {sec:4.1f} s   ({n} frames)" + ("   ⚠ stale stream" if stale else ""))
        elif state == "saving":
            self.rec_label.configure(text="… saving (encoding videos, please wait)", bg=C["warn"])
        elif state == "idle":
            if data is not None:
                self.n_episodes = data
            n = getattr(self, "n_episodes", 0)
            self.rec_label.configure(text=("REAL ARMS   " if self.cfg.get("target") == "real" else "") +
                                     f"READY   {n} episodes saved   press A to record the next one", bg=C["panel"])
            self._set("record", C["ok"], f"ready ({n} episodes saved)")
        elif state == "teleop":
            self.rec_label.configure(text="TELEOP (not recording)", bg=C["panel"])
        elif state == "not_recording":
            self.rec_label.configure(text="RECORDING IS OFF - A saves nothing (tick Record before Start)", bg=C["warn"])
        elif state == "real":
            self.rec_label.configure(text=f"REAL {' + '.join(self.cfg['arms']).upper()} ARMS" +
                                     ("  (dry run)" if self.cfg["real_dry_run"] else "  - hold GRIP to move"), bg=C["warn"])

    def stop_all(self):
        if not self.procs:
            self.destroy()
            return
        parking = self.cfg.get("target") == "real" and self.cfg.get("real_park") and not self.cfg.get("real_dry_run")
        if self.stage == "stopping":
            if parking and "teleop" in self.procs:  # second press: stop the arms where they are
                self.procs["teleop"].interrupt()
                self.rec_label.configure(text="Return to 0 deg interrupted - arms stopped", bg=C["err"])
            return
        self.stage = "stopping"
        self.rec_label.configure(text="Returning the arms to 0 deg ...  press Stop all again to stop them now" if parking
                                 else "Stopping... (saving any open episode, closing processes)", bg=C["warn"])

        def _stop():
            if "record" in self.procs:
                self.procs["record"].send("q")
                for _ in range(120):
                    if not self.procs["record"].alive():
                        break
                    time.sleep(0.5)
            for key in ("teleop", "viz", "driver", "sim"):  # RViz and the drivers stay up while the arms return
                if key in self.procs:
                    self.procs[key].interrupt()
                    for _ in range(240 if key == "teleop" and parking else 40):  # the return to 0 deg takes up to ~20 s
                        if not self.procs[key].alive():
                            break
                        time.sleep(0.5)
                    self.procs[key].kill()
            self.events.put(("log", "All stopped."))
            self.after(500, self.destroy)

        threading.Thread(target=_stop, daemon=True).start()

    def on_close(self):
        if self.stage == "stopping":  # closing the window now would be a second Stop all
            if messagebox.askyesno("Stop the arms?", "The arms are still returning to 0 deg.\n"
                                   "Stop them where they are and close?", icon="warning"):
                self.stop_all()
            return
        if any(p.alive() for p in self.procs.values()):
            if messagebox.askyesno("Quit", "Stop all processes (an open episode is saved first) and close?"):
                self.stop_all()
        else:
            self.destroy()


if __name__ == "__main__":
    App().mainloop()
