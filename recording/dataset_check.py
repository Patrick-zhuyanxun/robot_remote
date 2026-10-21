"""What is at a dataset path, and can this session record into it? Standard library only: used by the recorder
(record_dataset.py, LeRobot Python) and by the launcher (system Python, no LeRobot) before anything starts.

inspect(path)["state"]:
  new          nothing there                                   -> create
  empty_dir    an empty folder (e.g. made with "New folder")   -> create in it
  shell        meta/info.json only, 0 episodes: the recorder   -> re-create (holds no recording; LeRobot cannot
               started and quit before saving an episode          resume it - it would try the Hugging Face Hub)
  dataset      a LeRobot dataset with episodes                 -> append, if compatible() finds no problem
  incomplete   meta/info.json but the episode index is missing -> refuse (recorder killed before it finished, or
               (the recorder was killed before finalizing)        the folder was copied while recording)
  not_dataset  a non-empty folder that is not a dataset         -> refuse (never write into or delete it)
  file         a file, not a folder                             -> refuse
"""

import json
from pathlib import Path

DEFAULT_ROOT = Path(__file__).resolve().parents[1] / "dataset"
ROBOT_TYPE = {"sim": "igus_rebel_dual_arm_isaac", "real": "igus_rebel_dual_arm_real"}
SHELL_FILES = {Path("meta/info.json")}  # all that LeRobotDataset.create() + finalize() leave without an episode


def _files(path: Path):
    return [p.relative_to(path) for p in path.rglob("*") if p.is_file()]


def inspect(path) -> dict:
    path = Path(path).expanduser()
    if path.is_file():
        return {"state": "file", "path": path}
    if not path.exists():
        return {"state": "new", "path": path}
    files = _files(path)
    if not files:
        return {"state": "empty_dir", "path": path}
    info_path = path / "meta" / "info.json"
    if not info_path.is_file():
        return {"state": "not_dataset", "path": path, "files": len(files)}
    try:
        info = json.loads(info_path.read_text())
    except (OSError, ValueError):
        return {"state": "incomplete", "path": path, "reason": "meta/info.json cannot be read"}
    episodes = int(info.get("total_episodes", 0))
    if episodes == 0 and set(files) <= SHELL_FILES:
        return {"state": "shell", "path": path, "info": info, "episodes": 0}
    if not any((path / "meta" / "episodes").glob("*/*.parquet")):
        return {"state": "incomplete", "path": path, "info": info, "episodes": episodes,
                "reason": "meta/episodes/ is missing (the recorder did not finish - killed, or copied while recording)"}
    return {"state": "dataset", "path": path, "info": info, "episodes": episodes}


def summary(info: dict) -> str:
    cams = [k.split(".")[-1] for k in info.get("features", {}) if k.startswith("observation.images.")]
    src = {v: k for k, v in ROBOT_TYPE.items()}.get(info.get("robot_type"), info.get("robot_type"))
    return f"{src}, {info.get('fps')} fps, cameras: {', '.join(cams) or 'none'}"


def compatible(info: dict, source: str, fps: int, image_shapes: dict) -> list:
    """Problems that stop appending to an existing dataset ([] = OK). image_shapes = {camera: (height, width)}."""
    problems = []
    if info.get("robot_type") != ROBOT_TYPE[source]:
        problems.append(f"it was recorded on the {summary(info).split(',')[0]} robot_type, this session is {source}")
    if info.get("fps") != fps:
        problems.append(f"it is {info.get('fps')} fps, this session {fps} fps")
    have = {k.split(".")[-1]: tuple(v["shape"][:2]) for k, v in info.get("features", {}).items()
            if k.startswith("observation.images.")}
    want = {k: tuple(v) for k, v in image_shapes.items()}
    if set(have) != set(want):
        problems.append(f"its cameras are [{', '.join(sorted(have)) or 'none'}], this session "
                        f"[{', '.join(sorted(want)) or 'none'}]")
    else:
        sizes = [f"{k} {have[k][1]}x{have[k][0]} vs {want[k][1]}x{want[k][0]}" for k in want if have[k] != want[k]]
        if sizes:
            problems.append("image size differs: " + "; ".join(sizes))
    return problems


def describe(st: dict, problems=()) -> tuple:
    """(ok, one-line text) for the launcher."""
    s = st["state"]
    if s == "new":
        return True, "new dataset"
    if s == "empty_dir":
        return True, "empty folder: a new dataset is created in it"
    if s == "shell":
        return True, "dataset with no saved episode (an earlier session quit before saving): it is re-created"
    if s == "dataset":
        if problems:
            return False, f"existing dataset ({st['episodes']} episodes) CANNOT be continued: " + "; ".join(problems)
        return True, f"existing dataset, {st['episodes']} episodes ({summary(st['info'])}): new episodes are appended"
    if s == "incomplete":
        return False, f"incomplete dataset - {st['reason']}. Choose another name"
    if s == "not_dataset":
        return False, f"folder exists and is not a LeRobot dataset ({st['files']} files). Choose another name"
    return False, "this is a file, not a folder. Choose another name"


def prepare_new(st: dict) -> None:
    """Make room for LeRobotDataset.create() (which needs a path that does not exist): remove an empty folder, or
    the meta/info.json of a 'shell' (no episode was ever saved in it). Nothing else is ever deleted."""
    path = st["path"]
    if st["state"] == "shell":
        (path / "meta" / "info.json").unlink()
    if st["state"] in ("shell", "empty_dir"):
        for d in sorted((p for p in path.rglob("*") if p.is_dir()), key=lambda p: len(p.parts), reverse=True):
            d.rmdir()  # only empty directories remain; rmdir fails loudly otherwise
        path.rmdir()
