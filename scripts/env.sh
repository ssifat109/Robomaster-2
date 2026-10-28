#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source "$HOME/rm_venv/bin/activate"
export PYTHONPATH="$HOME/rm_venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
source "$HOME/rb2_ws/install/setup.bash"
export ROS_DOMAIN_ID=43
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
