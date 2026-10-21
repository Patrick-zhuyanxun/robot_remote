#!/usr/bin/env bash
# Real rig USB cameras: list / snapshot / throughput test (config/real_cameras.json). See recording/real_cameras.py.
#   run_cameras.sh --list | --snapshot | --test 10 [--cameras wrist_left head]
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "${LEROBOT_PY:-}" ]; then
  LEROBOT_PY="$HERE/../.venv_record/bin/python"
  [ -x "$LEROBOT_PY" ] || LEROBOT_PY="$HOME/Robotic_infra/lerobot/.venv/bin/python"
fi
exec "$LEROBOT_PY" "$HERE/real_cameras.py" "$@"
