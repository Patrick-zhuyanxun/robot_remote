#!/usr/bin/env bash
# Create robot_remote/.venv_record: Python 3.12 + LeRobot 0.5.1 for recording / replaying datasets.
# Python 3.12 matches the rclpy bundled with Isaac Sim 6.1, which the recorder imports (no /opt/ros needed).
# Skip this if you already have a LeRobot 0.5.x venv: export LEROBOT_PY=/path/to/that/venv/bin/python
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/../.venv_record"
[ -x "$VENV/bin/python" ] || uv venv --python 3.12 "$VENV"
VIRTUAL_ENV="$VENV" uv pip install "lerobot==0.5.1"
"$VENV/bin/python" -c "from lerobot.datasets.lerobot_dataset import LeRobotDataset; import av; print('[record] lerobot + av OK')"
echo "[record] done: $VENV"
