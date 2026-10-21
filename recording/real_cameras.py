#!/usr/bin/env python3
"""Physical USB cameras of the real rig (V4L2 colour streams through OpenCV) for record_dataset.py --source real.

config/real_cameras.json maps a dataset key (observation.images.<key>) to a camera, found through sysfs when the
recorder starts (never /dev/videoN: it renumbers on every replug):
  {"serial": "CP96563000LY", "interface": 4}  Orbbec: USB serial + UVC interface of the colour stream (any port).
                                              The /dev/v4l/by-id links are NOT usable for Orbbecs: several
                                              interfaces share the name "video-index0" and the last one wins.
  {"usb_port": "1-3.1"}                       ARC "HD USB Camera" 05a3:9230 (wrist cameras): no serial, so only the
                                              USB port tells left from right. Keep them in the same ports, or run
                                              --list / --snapshot and fix the config after re-cabling.
  {"device": "/dev/..."}                      explicit device node / file (tests)
  wrist_left / wrist_right = the wrist cameras, head = third-person view in the middle (same key as the sim's head
  camera), knife = close-up of the knife. Only the colour (UVC) stream is read; depth would need pyorbbecsdk.

Each camera runs a reader thread that keeps only the newest decoded frame and its capture time (time.monotonic()).
A camera that stops producing frames is reported stale by latest(), never blocks the caller.

  recording/run_cameras.sh --list                    # every V4L2 colour node with its stable paths
  recording/run_cameras.sh --snapshot [--all]        # one JPEG per configured camera (--all: + unconfigured nodes)
  recording/run_cameras.sh --test 10                 # all configured cameras together for 10 s: fps / gaps
"""

import argparse
import glob
import json
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "real_cameras.json"


def video_nodes():
    """[{dev, port, interface, serial, product}] of every /dev/videoN that is a UVC function's FIRST node (index 0);
    port = USB device path like "1-3.2", interface = bInterfaceNumber."""
    out = []
    for v in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            if Path(v, "index").read_text().strip() != "0":
                continue  # index 1 = the UVC metadata node of the same function
            intf = Path(os.path.realpath(Path(v, "device")))
            usb = intf.parent
            rd = lambda f: (usb / f).read_text().strip() if (usb / f).exists() else ""  # noqa: E731
            out.append({"dev": f"/dev/{Path(v).name}", "port": usb.name,
                        "interface": int((intf / "bInterfaceNumber").read_text(), 16),
                        "serial": rd("serial"), "product": rd("product"), "id": f"{rd('idVendor')}:{rd('idProduct')}",
                        "speed": rd("speed")})
        except (OSError, ValueError):
            continue
    return out


def resolve_device(spec) -> str:
    """Device node for one config entry (see the module docstring)."""
    if spec.get("device"):
        return spec["device"]
    nodes = video_nodes()
    if spec.get("serial"):
        hit = [n for n in nodes if n["serial"] == spec["serial"] and n["interface"] == int(spec.get("interface", 0))]
        what = f"serial {spec['serial']} interface {spec.get('interface', 0)}"
    elif spec.get("usb_port"):
        hit = [n for n in nodes if n["port"] == spec["usb_port"] and n["interface"] == int(spec.get("interface", 0))]
        if spec.get("usb_id"):
            hit = [n for n in hit if n["id"] == spec["usb_id"]]
        what = f"USB port {spec['usb_port']}"
    else:
        raise ValueError(f"camera entry needs serial, usb_port or device: {spec}")
    if len(hit) != 1:
        raise ConnectionError(f"no camera at {what} ({'unplugged / moved to another port?' if not hit else 'ambiguous'}"
                              " - run recording/run_cameras.sh --list)")
    return hit[0]["dev"]


def load_config(path=CONFIG) -> dict:
    """{key: {device, fourcc, width, height, fps, ...}} without the "_comment" entries."""
    cfg = json.loads(Path(path).read_text())
    return {k: v for k, v in cfg.items() if not k.startswith("_")}


class RealCamera:
    """One V4L2 camera: open, configure, read continuously in a thread; latest() returns (rgb, capture_time)."""

    def __init__(self, key, device, width=640, height=480, fps=30, fourcc="MJPG", decode_fps=None, rotate=0,
                 open_timeout=5.0):
        self.key, self.device = key, str(device)
        self.width, self.height, self.fps, self.fourcc = int(width), int(height), float(fps), fourcc
        # decode at most this often (grab() still drains every frame so the buffer never ages); None = every frame
        self.decode_period = 0.0 if not decode_fps else 1.0 / float(decode_fps)
        self.rotate = {0: None, 90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                       270: cv2.ROTATE_90_COUNTERCLOCKWISE}[int(rotate)]
        self.open_timeout = open_timeout
        self._lock = threading.Lock()
        self._frame, self._t = None, None
        self.frames = 0
        self.error = None
        self._stop = threading.Event()
        self._cap = None
        self._thread = None

    def __repr__(self):
        return f"RealCamera({self.key}: {self.device})"

    def open(self):
        if not os.path.exists(self.device):
            raise ConnectionError(f"{self.key}: {self.device} does not exist (unplugged or moved to another USB port? "
                                  "run recording/run_cameras.sh --list)")
        # a video file stands in for a camera in tests: played in a loop at fps
        self.is_file = not self.device.startswith("/dev/")
        cap = cv2.VideoCapture(self.device, cv2.CAP_ANY if self.is_file else cv2.CAP_V4L2)
        if not cap.isOpened():
            raise ConnectionError(f"{self.key}: cannot open {self.device} (in use by another program?)")
        # FOURCC first: it decides which sizes / rates exist
        if self.fourcc and not self.is_file:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        code = int(cap.get(cv2.CAP_PROP_FOURCC))
        got = "".join(chr((code >> 8 * i) & 0xFF) for i in range(4))
        if (w, h) != (self.width, self.height) or (self.fourcc and not self.is_file and got != self.fourcc):
            cap.release()
            raise RuntimeError(f"{self.key}: {self.device} gave {got} {w}x{h}, wanted {self.fourcc} "
                               f"{self.width}x{self.height} (see v4l2-ctl -d {self.device} --list-formats-ext)")
        self.actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self._cap = cap
        self._thread = threading.Thread(target=self._loop, name=f"cam_{self.key}", daemon=True)
        self._thread.start()
        t0 = time.monotonic()
        while self.latest()[0] is None:
            if self.error or time.monotonic() - t0 > self.open_timeout:
                self.close()
                raise ConnectionError(f"{self.key}: {self.device} opened but sent no frame in {self.open_timeout:.0f} s"
                                      + (f" ({self.error})" if self.error else "")
                                      + " - not enough USB bandwidth? (journalctl -k: 'Not enough bandwidth for "
                                      "altsetting'). The ARC wrist cameras always reserve their maximum USB 2 bandwidth; "
                                      "Orbbecs must link at 5000 Mbit/s (run_cameras.sh --list), else they also "
                                      "use the 480 Mbit/s USB 2 side")
            time.sleep(0.02)
        return self

    def _loop(self):
        last_decode = 0.0
        fails = 0
        while not self._stop.is_set():
            if self.is_file:
                time.sleep(1.0 / self.fps)
            if not self._cap.grab():
                if self.is_file:
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                fails += 1
                if fails > 20:
                    self.error = "read failures (camera disconnected?)"
                    return
                time.sleep(0.01)
                continue
            fails = 0
            t = time.monotonic()
            if t - last_decode < self.decode_period:
                continue
            ok, bgr = self._cap.retrieve()
            if not ok or bgr is None or bgr.ndim != 3:
                continue
            last_decode = t
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            if self.rotate is not None:
                rgb = cv2.rotate(rgb, self.rotate)
            with self._lock:
                self._frame, self._t = rgb, t
                self.frames += 1

    def latest(self):
        """(rgb uint8 HxWx3, capture time.monotonic()) of the newest frame, or (None, None) before the first one."""
        with self._lock:
            return self._frame, self._t

    def alive(self):
        return self._thread is not None and self._thread.is_alive()

    def close(self):
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()
        self._cap = None


def open_cameras(keys, config=None) -> dict:
    """Open the configured cameras `keys` (all or nothing: on any failure the opened ones are closed again)."""
    config = load_config() if config is None else config
    missing = [k for k in keys if k not in config]
    if missing:
        raise KeyError(f"cameras {missing} are not in {CONFIG} (configured: {list(config)})")
    cams = {}
    try:
        for k in keys:
            c = config[k]
            cams[k] = RealCamera(k, resolve_device(c), c.get("width", 640), c.get("height", 480), c.get("fps", 30),
                                 c.get("fourcc", "MJPG"), c.get("decode_fps"), c.get("rotate", 0)).open()
    except Exception:
        for cam in cams.values():
            cam.close()
        raise
    return cams


def colour_nodes():
    """video_nodes() that offer a colour format (skips Orbbec depth Z16 / IR GREY / raw Bayer functions)."""
    import subprocess

    out = []
    for n in video_nodes():
        try:
            fmts = subprocess.run(["v4l2-ctl", "-d", n["dev"], "--list-formats"], capture_output=True, text=True,
                                  timeout=3).stdout
        except (OSError, subprocess.SubprocessError):
            continue
        if any(f"'{f}'" in fmts for f in ("MJPG", "YUYV")):
            out.append(n)
    return out


def node_spec(n) -> dict:
    """The config entry that finds node n again."""
    return {"serial": n["serial"], "interface": n["interface"]} if n["serial"] else \
        {"usb_port": n["port"], "usb_id": n["id"]}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--list", action="store_true", help="list colour nodes and their stable paths")
    p.add_argument("--snapshot", action="store_true", help="save one JPEG per configured camera to --out")
    p.add_argument("--all", action="store_true",
                   help="--snapshot: also probe colour nodes that are not in the config. Caution: probing the Gemini "
                        "305's second colour interface (6) left its colour stream (4) without frames until replugged")
    p.add_argument("--test", type=float, metavar="S", help="stream all configured cameras together for S seconds")
    p.add_argument("--cameras", nargs="*", help="subset of configured keys (default: all)")
    p.add_argument("--config", default=str(CONFIG))
    p.add_argument("--out", default=str(ROOT / "logs" / "camera_snapshots"))
    args = p.parse_args()
    if not (args.list or args.snapshot or args.test):
        args.list = True

    if args.list:
        for n in colour_nodes():
            print(f"{n['product']:20s} {n['dev']:12s} port {n['port']:9s} interface {n['interface']}  "
                  f"{n['speed']:>5s} Mbit/s  config: {json.dumps(node_spec(n))}")

    cfg = load_config(args.config) if Path(args.config).exists() else {}
    keys = args.cameras if args.cameras else list(cfg)

    if args.snapshot:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        done = set()
        for k in keys:
            try:
                cam = open_cameras([k], cfg)[k]
                time.sleep(0.5)  # let auto exposure settle
                rgb, _ = cam.latest()
                cam.close()
                cam.device = os.path.realpath(cam.device)
                f = out / f"{stamp}_{k}.jpg"
                cv2.imwrite(str(f), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                done.add(cam.device)
                print(f"[cam] {k}: {f}")
            except Exception as exc:
                print(f"[cam] {k}: FAILED {exc}")
        for n in colour_nodes() if args.all else []:  # unconfigured nodes too, to help fill in the config
            if n["dev"] in done:
                continue
            tag = f"{n['port']}_if{n['interface']}"
            for fourcc, fps in (("MJPG", 30), ("MJPG", 120), ("YUYV", 30)):
                try:
                    cam = RealCamera(tag, n["dev"], 640, 480, fps, fourcc).open()
                    break
                except Exception as exc:
                    err = exc
            else:
                print(f"[cam] {n['product']} {tag}: FAILED {err}")
                continue
            time.sleep(0.5)
            rgb, _ = cam.latest()
            cam.close()
            f = out / f"{stamp}_unconfigured_{tag}.jpg"
            cv2.imwrite(str(f), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            print(f"[cam] unconfigured {n['product']} {json.dumps(node_spec(n))}: {f}")

    if args.test:
        cams = open_cameras(keys, cfg)
        n0 = {k: c.frames for k, c in cams.items()}
        gap = {k: 0.0 for k in cams}
        last = {k: c.latest()[1] for k, c in cams.items()}
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.test:
            for k, c in cams.items():
                _, t = c.latest()
                if t != last[k]:
                    gap[k] = max(gap[k], t - last[k])
                    last[k] = t
            time.sleep(0.002)
        dt = time.monotonic() - t0
        for k, c in cams.items():
            rgb, t = c.latest()
            print(f"[cam] {k:12s} {(c.frames - n0[k]) / dt:6.1f} decoded fps  max gap {1000 * gap[k]:5.0f} ms  "
                  f"age {1000 * (time.monotonic() - t):4.0f} ms  {rgb.shape[1]}x{rgb.shape[0]}  "
                  f"(device {c.actual_fps:.0f} fps){'  DEAD: ' + c.error if c.error else ''}")
            c.close()


if __name__ == "__main__":
    main()
