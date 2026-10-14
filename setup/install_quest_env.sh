#!/usr/bin/env bash
# Create robot_remote/.venv_quest: Python 3.10 (same as ROS 2 Humble's rclpy) + NVIDIA Isaac Teleop / CloudXR.
# --system-site-packages lets it import /opt/ros/humble's rclpy. numpy<2 because Humble's packages are built against 1.x.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/../.venv_quest"
[ -x "$VENV/bin/python" ] || uv venv --python /usr/bin/python3.10 --system-site-packages "$VENV"
VIRTUAL_ENV="$VENV" uv pip install "isaacteleop[cloudxr]==1.3.131" "numpy<2" scipy
# CloudXR runtime licence: accept once per machine (stored under ~/.cloudxr).
"$VENV/bin/python" -m isaacteleop.cloudxr --accept-eula
echo "[quest] done: $VENV"
