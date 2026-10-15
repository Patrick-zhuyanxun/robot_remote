#!/usr/bin/env python3
"""robot_remote launcher: one window to configure and start Quest teleop + data recording.

  robot_remote/launcher/run_gui.sh

Settings page -> Start:  Isaac sim (run_dual_arm_sim.sh --cameras ...) -> Quest teleop (run_quest_dual_arm.sh)
-> recorder (run_record.sh). The running page shows each process, the recording state, the Quest
connection URL, and the controller legend. Settings live in config/launcher.json.
"""

import json
import os
import queue
import re
import signal
import socket
import subprocess
import threading
import time
import tkinter as tk
import webbrowser
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / "logs"
CONFIG = ROOT / "config" / "launcher.json"
LEGACY_CONFIG = Path.home() / ".config" / "robot_remote_launcher.json"
QUEST_URL = "https://nvidia.github.io/IsaacTeleop/client/v1.3.131/"
CAMERAS = [("head", "Head RGB (top view)"), ("wrist_left", "Left wrist RGB"), ("wrist_right", "Right wrist RGB")]
ENDPOINTS = json.loads((ROOT / "config/real_arms.json").read_text())
DEFAULTS = {
    "target": "isaac", "arms": ["left", "right"], "cameras": ["head", "wrist_left", "wrist_right"],
    "record": True, "name": "dual_rebel_task", "task": "pick up the box",
    "root": os.environ.get("ROBOT_REMOTE_DATASETS", str(ROOT / "datasets")), "resolution": "640x480", "fps": 30, "yaw_deg": 0,
    "left_ip": ENDPOINTS["left"]["host"], "right_ip": ENDPOINTS["right"]["host"], "real_dry_run": True,
    "real_park": True,
}
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
            if cfg["root"] == str(Path.home() / "datasets"):
                cfg["root"] = str(ROOT / "datasets")
            CONFIG.parent.mkdir(parents=True, exist_ok=True)
            CONFIG.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n")
        return cfg
    except (OSError, ValueError):
        return dict(DEFAULTS)


class Proc:
    """A launched script: log file + optional line callback + stdin for the recorder."""

    def __init__(self, name, cmd, on_line=None, stdin=False, extra_env=None):
        self.name, self.cmd, self.on_line = name, cmd, on_line
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
        with open(self.log_path, "w") as f:
            for line in self.p.stdout:
                f.write(line)
                f.flush()
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
        if self.alive():
            os.killpg(self.p.pid, signal.SIGTERM)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("robot_remote — Quest teleop & data recording")
        self.configure(bg=C["bg"])
        self.geometry("1380x820")
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
        ttk.Radiobutton(g, text="Real igus ReBeL dual arms — wired Ethernet (teleop only)", value="real",
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
        self.v_real_park = tk.BooleanVar(value=self.cfg["real_park"])
        ttk.Checkbutton(row, text="Return both arms to all-joints 0 deg before exit (10 deg/s)",
                        variable=self.v_real_park).pack(side="left", padx=14)
        ttk.Label(self.real_box, text="Limits: 20 deg/s per joint, 0.15 m/s, 30 deg/s. CRI TCP 3920. "
                                      "Enable motors using robot controls; starts at current pose.",
                  style="Dim.TLabel").pack(anchor="w")
        self.v_target.trace_add("write", lambda *_: self._target_changed())

        g = ttk.LabelFrame(left, text="2  Arms to control")
        g.pack(fill="x", pady=4)
        self.v_arms = {s: tk.BooleanVar(value=s in self.cfg["arms"]) for s in ("left", "right")}
        row = ttk.Frame(g)
        row.pack(anchor="w", padx=8, pady=4)
        ttk.Checkbutton(row, text="Left arm  <-  left controller", variable=self.v_arms["left"]).pack(side="left", padx=(0, 20))
        ttk.Checkbutton(row, text="Right arm  <-  right controller", variable=self.v_arms["right"]).pack(side="left")

        g = ttk.LabelFrame(left, text="3  Cameras (enabled in the sim and recorded)")
        g.pack(fill="x", pady=4)
        cam_box = g
        self.v_cams = {k: tk.BooleanVar(value=k in self.cfg["cameras"]) for k, _ in CAMERAS}
        row = ttk.Frame(g)
        row.pack(anchor="w", padx=8, pady=4)
        for k, label in CAMERAS:
            ttk.Checkbutton(row, text=label, variable=self.v_cams[k]).pack(side="left", padx=(0, 18))
        row = ttk.Frame(g)
        row.pack(anchor="w", padx=8, pady=(0, 4))
        ttk.Label(row, text="Resolution").pack(side="left")
        self.v_res = tk.StringVar(value=self.cfg["resolution"])
        ttk.Combobox(row, textvariable=self.v_res, values=["640x480", "320x240", "848x480"], width=9,
                     state="readonly").pack(side="left", padx=6)
        ttk.Label(row, text="FPS").pack(side="left", padx=(12, 0))
        self.v_fps = tk.StringVar(value=str(self.cfg["fps"]))
        ttk.Combobox(row, textvariable=self.v_fps, values=["30", "15", "10"], width=4, state="readonly").pack(side="left", padx=6)
        ttk.Label(row, text="More cameras = slower sim (3 cameras ~0.5x real time)", style="Dim.TLabel").pack(side="left", padx=12)

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
        for v in (self.v_name, self.v_root):
            v.trace_add("write", lambda *_: self._update_path_hint())
        self._update_path_hint()
        self.record_widgets = [w for w in self._descendants(g)] + [w for w in self._descendants(cam_box)]

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
        for w in self.record_widgets:
            try:
                w.configure(state="disabled" if real else ("readonly" if isinstance(w, ttk.Combobox) else "normal"))
            except tk.TclError:
                pass

    def open_replay(self):
        from replay_window import ReplayWindow
        d = Path(self.v_root.get()).expanduser() / self.v_name.get()
        ReplayWindow(self, str(d) if (d / "meta" / "info.json").is_file() else None)

    def _browse(self):
        d = filedialog.askdirectory(initialdir=self.v_root.get() or str(Path.home()))
        if d:
            self.v_root.set(d)

    def _update_path_hint(self):
        p = Path(self.v_root.get()).expanduser() / self.v_name.get()
        exists = (p / "meta" / "info.json").exists()
        self.v_path_hint.configure(text=f"→ {p}" + ("   (exists: new episodes will be appended)" if exists else "   (new dataset)"))

    # ------------------------------------------------------------------ controller legend
    def _legend(self, parent):
        cv = tk.Canvas(parent, width=400, height=560, bg=C["panel"], highlightthickness=0)

        def controller(cx, title, top, bottom):
            cv.create_text(cx, 22, text=title, fill=C["fg"], font=("DejaVu Sans", 12, "bold"))
            cv.create_oval(cx - 70, 40, cx + 70, 150, outline="#8b949e", width=3)        # tracking ring
            cv.create_rectangle(cx - 28, 110, cx + 28, 270, outline="#8b949e", width=3)   # handle
            cv.create_oval(cx - 45, 70, cx + 45, 130, fill="#30363d", outline="#8b949e", width=2)  # face
            cv.create_oval(cx - 30, 78, cx - 6, 102, fill="#58a6ff", outline="")          # face button 1
            cv.create_text(cx - 18, 90, text=top[0], fill="white", font=("Arial", 11, "bold"))
            cv.create_oval(cx + 6, 92, cx + 30, 116, fill="#d2a8ff", outline="")          # face button 2
            cv.create_text(cx + 18, 104, text=bottom[0], fill="white", font=("Arial", 11, "bold"))
            cv.create_rectangle(cx - 33, 150, cx - 22, 210, fill=C["ok"], outline="")       # grip
            cv.create_polygon(cx + 28, 120, cx + 44, 132, cx + 30, 160, fill=C["warn"], outline="")  # trigger

        controller(105, "Left -> left arm", ("X",), ("Y",))
        controller(295, "Right -> right arm", ("A",), ("B",))
        items = [
            (C["ok"], "Grip (middle finger) - hold", "the arm follows your hand; release = freeze.\nRelease, reposition your hand, grip again - the arm does not jump"),
            (C["warn"], "Trigger (index finger)", "closes the gripper (deeper = tighter)"),
            ("#58a6ff", "A / X", "start recording -> press again to save the episode"),
            ("#d2a8ff", "B / Y", "discard the current episode and re-record"),
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
        w, h = self.v_res.get().split("x")
        real = self.v_target.get() == "real"
        if real:
            cams = []
        cfg = {"target": self.v_target.get(), "arms": arms, "cameras": cams, "record": self.v_record.get() and not real,
               "name": self.v_name.get().strip(), "task": self.v_task.get().strip(), "root": self.v_root.get().strip(),
               "resolution": self.v_res.get(), "fps": int(self.v_fps.get()), "yaw_deg": int(self.v_yaw.get()),
               "width": int(w), "height": int(h),
               "left_ip": self.v_real_ips["left"].get().strip(),
               "right_ip": self.v_real_ips["right"].get().strip(), "real_dry_run": self.v_real_dry.get(),
               "real_park": self.v_real_park.get()}
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
                "- Motors are enabled and arms are in a bent working pose using robot controls.\n"
                "- Arms start at their current poses. Real grippers are not controlled.\n"
                + ("- On Stop all the arms move ON THEIR OWN back to all joints 0 deg (10 deg/s).\n"
                   "  Press Stop all again to stop them where they are.\n" if cfg["real_park"] else "")
                + 
                "\nEach arm only moves while you hold its GRIP. Continue?", icon="warning"):
            return
        CONFIG.parent.mkdir(parents=True, exist_ok=True)
        CONFIG.write_text(json.dumps({k: v for k, v in cfg.items() if k in DEFAULTS}, ensure_ascii=False, indent=1))
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
        ttk.Label(left, text=info + f"\nArms: {', '.join(cfg['arms'])}    Cameras: {', '.join(cfg['cameras']) or 'none'}"
                  f"    {cfg['resolution']} @ {cfg['fps']} fps", style="Dim.TLabel").pack(anchor="w", pady=6)
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
        self.b_toggle = ttk.Button(btns, text="● Start / Save  (= A/X)", command=lambda: self._rec_cmd(""))
        self.b_discard = ttk.Button(btns, text="✕ Discard  (= B/Y)", command=lambda: self._rec_cmd("d"))
        if cfg["record"]:
            self.b_toggle.pack(side="left", padx=(0, 8))
            self.b_discard.pack(side="left", padx=(0, 8))
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
        self._set("record", C["dim"], "not available on the real robot yet")
        cmd = [str(ROOT / "quest_teleop" / "run_quest_real_arm.sh"), "--arms", *cfg["arms"],
               "--left-ip", cfg["left_ip"], "--right-ip", cfg["right_ip"],
               "--yaw-deg", str(cfg["yaw_deg"]), "--quiet"]
        if cfg["yaw_deg"] == 180:
            cmd.append("--swap-hands")
        if cfg["real_dry_run"]:
            cmd.append("--dry-run")
        if not cfg["real_park"]:
            cmd.append("--no-park")
        self._set("teleop", C["warn"], "connecting to selected arms over Ethernet TCP...")
        self.procs["teleop"] = Proc("teleop", cmd, self._on_line)
        p = self.procs["teleop"]
        if not self._wait_for(p, r"real arm connected", 60):
            p.interrupt()
            self._set("teleop", C["err"], f"Ethernet preflight failed: check IP, motor state and CRI ownership; log: {p.log_path.name}")
            return
        self._set("sim", C["ok"], "robot connected")
        if not self._wait_for(p, r"CloudXR up", 180):
            p.interrupt()
            self._set("teleop", C["err"], f"failed to start - see log: {p.log_path}")
            return
        self._set("teleop", C["warn"], f"waiting for the Quest (Server IP {lan_ip()})")
        self.events.put(("rec", "real", 0))

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
                    "--yaw-deg", str(cfg["yaw_deg"]), "--quiet"]
        if cfg["yaw_deg"] == 180:
            tele_cmd.append("--swap-hands")
        self._set("teleop", C["warn"], "starting (arms go home, CloudXR ~10 s)...")
        self.procs["teleop"] = Proc("teleop", tele_cmd, self._on_line)
        if not self._wait_for(self.procs["teleop"], r"CloudXR up", 120):
            self._set("teleop", C["err"], f"failed to start - see log: {self.procs['teleop'].log_path}")
            return
        self._set("teleop", C["warn"], f"waiting for the Quest (Server IP {lan_ip()})")

        if cfg["record"]:
            rec_cmd = [str(ROOT / "recording" / "run_record.sh"), "--name", cfg["name"], "--task", cfg["task"],
                       "--root", cfg["root"], "--fps", str(cfg["fps"]), "--width", str(cfg["width"]),
                       "--height", str(cfg["height"]), "--cameras", *cfg["cameras"]]
            self._set("record", C["warn"], "starting...")
            self.procs["record"] = Proc("record", rec_cmd, self._on_line, stdin=True)
            if not self._wait_for(self.procs["record"], r"\[rec\] controls", 120):
                self._set("record", C["err"], f"failed to start - see log: {self.procs['record'].log_path}")
                return
            self._set("record", C["ok"], "ready (press A/X to record)")
            self.events.put(("rec", "idle", 0))
        else:
            self._set("record", C["dim"], "disabled")
            self.events.put(("rec", "teleop", 0))

    def _on_line(self, name, line):
        if name == "teleop" and any(k in line for k in ("REAL ARM", "real arm connected", "other publisher", "let go", "RobotError", "[park]",
                                                         "start pose", "dry run")):
            self._msg(f"[robot] {line.split(']: ')[-1]}")
        if name == "teleop":
            if "TRACKED" in line and "LOST" not in line:
                self._set("teleop", C["ok"], "Quest connected, controllers tracked")
            elif "tracking LOST" in line:
                self._set("teleop", C["warn"], "controller tracking lost (keep it in front of the headset)")
            if any(k in line for k in ("engaged", "released", "button", "TRACKED", "LOST", "apart", "Error", "Traceback")):
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
            elif m := re.search(r"● (\d+) frames \(([\d.]+)s sim\)", line):
                self.events.put(("rec", "frames", (int(m.group(1)), float(m.group(2)), "stale" in line)))
            if line.startswith("[rec]") and "● " not in line or "WARNING" in line or "cannot start" in line:
                self._msg(line)
        elif name == "sim" and ("Traceback" in line or "Error" in line and "[Error]" not in line):
            self._msg(f"[sim] {line}")

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
        for key, p in list(self.procs.items()):
            if not p.alive() and self.stage != "stopping" and getattr(p, "_reported", False) is False:
                p._reported = True
                self._set(key, C["err"], f"exited (code {p.p.returncode}), log: {p.log_path.name}")
        self.after(100, self._poll)

    def _rec_state(self, state, data):
        if state == "recording":
            self.rec_label.configure(text="● RECORDING   A/X = save,  B/Y = discard", bg=C["rec"])
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
            self.rec_label.configure(text=f"READY   {n} episodes saved   press A/X to record the next one", bg=C["panel"])
            self._set("record", C["ok"], f"ready ({n} episodes saved)")
        elif state == "teleop":
            self.rec_label.configure(text="TELEOP (not recording)", bg=C["panel"])
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
        self.rec_label.configure(text="Returning both arms to 0 deg ...  press Stop all again to stop them now" if parking
                                 else "Stopping... (saving any open episode, closing processes)", bg=C["warn"])

        def _stop():
            if "record" in self.procs:
                self.procs["record"].send("q")
                for _ in range(120):
                    if not self.procs["record"].alive():
                        break
                    time.sleep(0.5)
            for key in ("teleop", "sim"):
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
        if any(p.alive() for p in self.procs.values()):
            if messagebox.askyesno("Quit", "Stop all processes (an open episode is saved first) and close?"):
                self.stop_all()
        else:
            self.destroy()


if __name__ == "__main__":
    App().mainloop()
