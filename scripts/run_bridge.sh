#!/bin/bash
export PYTHONPATH=/home/clrobur/.local/lib/python3.10/site-packages:/home/clrobur/airsim/Colosseum/PythonClient:$PYTHONPATH
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=/home/clrobur/airsim/settings/zenoh_sim.json5
source /home/clrobur/airsim/airsim_ros2_bridge/install/setup.bash
exec python3 /home/clrobur/airsim/airsim_ros2_bridge/airsim_ros2_bridge/bridge_node.py "$@"
