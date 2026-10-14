#!/usr/bin/env bash
# robot_remote launcher GUI (system Python 3.10 + tkinter; it starts the other launchers itself).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$HERE/robot_remote_gui.py" "$@"
