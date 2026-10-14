#!/usr/bin/env python3
"""Print a LeRobot dataset's summary and episode list as one JSON line (used by the replay window).

  <lerobot python> recording/dataset_info.py /path/to/dataset
"""

import json
import sys
from pathlib import Path


def main():
    root = Path(sys.argv[1]).expanduser().resolve()
    if not (root / "meta" / "info.json").is_file():
        print(json.dumps({"error": f"not a LeRobot dataset (no meta/info.json): {root}"}))
        return
    from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata

    try:
        meta = LeRobotDatasetMetadata(f"local/{root.name}", root=root)
    except Exception as e:  # incomplete folder (e.g. moved while the recorder was still finalizing)
        missing = [] if any((root / "meta" / "episodes").glob("*/*.parquet")) else ["meta/episodes/*.parquet"]
        print(json.dumps({"error": f"cannot read the dataset ({type(e).__name__})"
                          + (f"; missing {missing[0]} - was it moved before the recorder finished?" if missing else "")}))
        return
    eps = []
    for row in meta.episodes:
        tasks = row.get("tasks") or []
        eps.append({"index": int(row["episode_index"]), "length": int(row["length"]),
                    "task": tasks[0] if isinstance(tasks, list) and tasks else str(tasks)})
    print(json.dumps({"name": root.name, "fps": meta.fps, "robot_type": meta.robot_type,
                      "total_episodes": meta.total_episodes, "total_frames": meta.total_frames,
                      "cameras": [k.split(".")[-1] for k in meta.camera_keys], "episodes": eps}))


if __name__ == "__main__":
    main()
