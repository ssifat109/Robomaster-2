#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/rm_venv/bin/activate"
export PYTHONPATH="$HOME/rm_venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
ROOT="/mnt/d/RB 2"
WS="$HOME/rb2_ws"
mkdir -p "$WS/src"
if [[ ! -e "$WS/src/rm_flatness_nav" ]]; then
    ln -s "$ROOT/rm_flatness_nav" "$WS/src/rm_flatness_nav"
fi
cd "$WS"
colcon build --symlink-install --packages-select rm_flatness_nav
source install/setup.bash
python -m pytest "$ROOT/rm_flatness_nav/test" -q
python -m compileall -q "$ROOT/rm_flatness_nav/rm_flatness_nav"
