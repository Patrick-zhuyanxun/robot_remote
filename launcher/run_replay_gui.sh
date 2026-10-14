#!/usr/bin/env bash
# Play back a recorded dataset: pick the folder, pick an episode, view it (LeRobot/Rerun) or replay it in Isaac.
#   launcher/run_replay_gui.sh [/path/to/dataset]
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$HERE/replay_window.py" "$@"
