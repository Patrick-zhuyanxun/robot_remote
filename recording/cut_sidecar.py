"""Per-episode sidecar folder of the cut recorder, compatible with src/side_cut_vision of igus_rebel_ros2.

Standard library only. For every episode (start .. end) the recorder keeps a folder

    <clip_dir>/<YYYYmmdd_HHMMSS>/clip.json   (+ a copy of the seam snapshot)

exactly where the passive tools/cut_recorder of igus_rebel_ros2 puts its clips, so that side_cut_vision's probe finds the
open clip unchanged: it looks for the newest <out_dir>/<name>/clip.json with "recording": true that was modified within
30 min and writes its measurements (side_cut_vision/ folder) next to it. clip.json is rewritten every REFRESH_S while the
episode is open (a crashed recorder leaves "recording": true forever; the probe then ignores it after 30 min).
"""
import datetime as _dt
import json
import os
import shutil
import time

EXTERNAL_DRIVE = "/media/farhan/My Passport"
# same defaults as tools/cut_recorder/cut_recorder.py and side_cut_vision/trigger.py (kept in sync by hand)
EXTERNAL_CLIP_DIR = EXTERNAL_DRIVE + "/igus_nvidia/cut_videos"
FALLBACK_CLIP_DIR = "/home/farhan/Igus_rebel/outputs/cut_videos"
EXTERNAL_DATASET_DIR = EXTERNAL_DRIVE + "/igus_nvidia/cut_datasets"
DEFAULT_SNAPSHOT = "/home/farhan/Igus_rebel/outputs/seam_3d_base_link.json"
REFRESH_S = 20.0     # well below the probe's 30 min freshness (and the lead's 60 s)


def drive_writable():
    return os.path.ismount(EXTERNAL_DRIVE) and os.access(EXTERNAL_DRIVE, os.W_OK | os.X_OK)


def default_clip_dir():
    return EXTERNAL_CLIP_DIR if drive_writable() else FALLBACK_CLIP_DIR


def default_dataset_root(fallback):
    """/ is 86 % full: cut datasets go to the external drive when it is mounted."""
    return EXTERNAL_DATASET_DIR if drive_writable() else str(fallback)


def local_iso(ts=None):
    return _dt.datetime.fromtimestamp(ts if ts is not None else time.time()).astimezone().isoformat(timespec="milliseconds")


def write_json_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=2, default=str)
    os.replace(tmp, path)


def make_clip_dir(clip_dir, when=None):
    name = _dt.datetime.fromtimestamp(when if when is not None else time.time()).strftime("%Y%m%d_%H%M%S")
    os.makedirs(clip_dir, exist_ok=True)
    path, n = os.path.join(clip_dir, name), 1
    while True:
        try:
            os.mkdir(path)
            return path
        except FileExistsError:
            n += 1
            path = os.path.join(clip_dir, f"{name}_{n}")


class Sidecar:
    """One episode's folder. Never raises: a full disk or a missing drive must not cost an episode (errors -> self.error)."""

    def __init__(self, clip_dir, info, snapshot=None, clock=time.monotonic, log=print):
        self.info, self._clock, self._log = dict(info), clock, log
        self.start_wall = time.time()
        self.error = None
        self.dir = None
        self.snapshot_copied = self.snapshot_error = None
        self._last_write = clock()
        try:
            self.dir = make_clip_dir(clip_dir, self.start_wall)
            if snapshot:
                self._copy_snapshot(snapshot)
            self._write(recording=True)
        except Exception as e:  # noqa: BLE001
            self.error = f"{type(e).__name__}: {e}"
            self._log(f"[rec] WARNING: sidecar folder not created in {clip_dir}: {self.error}")

    def _copy_snapshot(self, snapshot):
        try:
            if os.path.isfile(snapshot):
                dst = os.path.join(self.dir, os.path.basename(snapshot))
                shutil.copy2(snapshot, dst)
                self.snapshot_copied = os.path.basename(dst)
            else:
                self.snapshot_error = "snapshot file not found"
        except Exception as e:  # noqa: BLE001
            self.snapshot_error = f"{type(e).__name__}: {e}"

    def _write(self, recording, **extra):
        if self.dir is None:
            return
        doc = dict(self.info)
        doc.update(extra)
        doc.update(recording=recording, start_local=local_iso(self.start_wall), snapshot=self.snapshot_copied,
                   snapshot_error=self.snapshot_error, updated_local=local_iso())
        try:
            write_json_atomic(os.path.join(self.dir, "clip.json"), doc)
            self._last_write = self._clock()
        except Exception as e:  # noqa: BLE001
            self.error = f"{type(e).__name__}: {e}"
            self._log(f"[rec] WARNING: cannot write clip.json: {self.error}")

    def refresh(self, **extra):
        """Rewrite clip.json (still recording) if REFRESH_S passed; cheap to call every loop."""
        if self._clock() - self._last_write >= REFRESH_S:
            self._write(True, **extra)

    def finish(self, **final):
        self._write(False, stop_local=local_iso(), duration_s=round(time.time() - self.start_wall, 3), **final)
        return self.dir


def append_index(clip_dir, record):
    """One line per finished episode in <clip_dir>/index.jsonl (for the converter). Never raises."""
    try:
        os.makedirs(clip_dir, exist_ok=True)
        with open(os.path.join(clip_dir, "index.jsonl"), "a") as f:
            f.write(json.dumps(record, default=str) + "\n")
            f.flush()
            os.fsync(f.fileno())
        return None
    except Exception as e:  # noqa: BLE001
        return f"{type(e).__name__}: {e}"
