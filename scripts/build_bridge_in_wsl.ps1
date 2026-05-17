param(
    [string]$Distro = "Ubuntu-22.04",
    [string]$RosWs = "/home/branc/aerion_ros2_ws",
    [string]$Package = "airsim_ros2_bridge"
)

$ErrorActionPreference = "Stop"

$buildCommand = @"
set -euo pipefail
cd "$RosWs"
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --packages-select "$Package"
"@

wsl -d $Distro -- bash -lc $buildCommand
