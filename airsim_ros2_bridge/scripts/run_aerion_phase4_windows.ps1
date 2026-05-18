# AERION Phase 4 - Windows + WSL2 Ubuntu-22.04 auto run wrapper (v3, ASCII-only)
#
# Prerequisites:
#   1. WSL2 Ubuntu-22.04 with ROS2 Humble + airsim_ros2_bridge built
#   2. UE Editor + Colosseum + BlocksV2 built, settings sf_5drones_phase3.json deployed,
#      and Play pressed (so AirSim RPC :41451 is alive on Windows host)
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File scripts/run_aerion_phase4_windows.ps1
#   powershell -ExecutionPolicy Bypass -File scripts/run_aerion_phase4_windows.ps1 -DroneCount 3 -Sequence "LINE:10,DIAMOND:10"
#
# Changelog:
#   v1 2026-05-18: initial
#   v2 2026-05-18: removed here-string + bash backslash line continuation (PowerShell parser conflict)
#   v3 2026-05-18: removed all non-ASCII (Korean) text. PowerShell 5.x defaults to CP949 on Windows
#                  and corrupts UTF-8 Korean unless BOM is present. Now ASCII-only for safety.

param(
    [int]$DroneCount = 5,
    [string]$Sequence = "LINE:10,DIAMOND:10,ARROW:10,V:10,ECHELON:10",
    [double]$Altitude = 5.0,
    [bool]$SkipCamera = $true,
    [bool]$SkipRange = $true,
    [string]$WslDistro = "Ubuntu-22.04",
    [string]$WslWorkspace = "~/aerion_ros2_ws"
)

Write-Host "===========================================" -ForegroundColor Cyan
Write-Host "  AERION Phase 4 - Windows+WSL Wrapper v3"  -ForegroundColor Cyan
Write-Host "===========================================" -ForegroundColor Cyan

# 1) Windows host IP visible from WSL (default gateway)
$WslHostIp = wsl -d $WslDistro -- bash -c "ip route show | grep default | awk '{print `$3}'"
$WslHostIp = $WslHostIp.Trim()
Write-Host "[INFO] AirSim host IP (WSL->Windows): $WslHostIp" -ForegroundColor Yellow

if ([string]::IsNullOrEmpty($WslHostIp)) {
    Write-Error "Failed to detect WSL host IP. Is WSL running? Try: wsl -d $WslDistro"
    exit 1
}

# 2) AirSim ping (double quote escape via [char]34)
$Q = [char]34
$pingCmd = "cd /tmp && python3 -c 'import airsim; c=airsim.MultirotorClient(ip=${Q}$WslHostIp${Q}, timeout_value=5.0); print(${Q}ping=${Q}, c.ping(), ${Q} vehicles=${Q}, c.listVehicles())'"
Write-Host "[CHECK] AirSim RPC ping..." -ForegroundColor Cyan
$pingResult = wsl -d $WslDistro -- bash -c $pingCmd
Write-Host $pingResult
if ($pingResult -notmatch "ping= True") {
    Write-Warning "AirSim ping failed. Press Play in UE Editor and press Enter to retry:"
    Read-Host
    $pingResult = wsl -d $WslDistro -- bash -c $pingCmd
    Write-Host $pingResult
    if ($pingResult -notmatch "ping= True") {
        Write-Error "AirSim still not responding. Check UE Play state."
        exit 2
    }
}

# 3) Cleanup leftover ROS nodes
Write-Host "[CLEAN] killing leftover ROS nodes..." -ForegroundColor Cyan
wsl -d $WslDistro -- bash -c "pkill -KILL -f 'python3.*bridge_node|aerion_formation|aerion_leader|aerion_tf' 2>/dev/null; sleep 1"

# 4) Launch bridges + formation + leader + tf (new wsl window, single-line bash)
$enableCamera = if ($SkipCamera) { "false" } else { "true" }
$enableRange  = if ($SkipRange)  { "false" } else { "true" }

Write-Host "[LAUNCH] bridge x $DroneCount + formation + leader + tf (separate wsl window)" -ForegroundColor Cyan
$launchInner = "source /opt/ros/humble/setup.bash && source $WslWorkspace/install/setup.bash && ros2 launch airsim_ros2_bridge aerion_phase4_formation.launch.py drone_count:=$DroneCount airsim_ip:=$WslHostIp enable_camera:=$enableCamera enable_range:=$enableRange; exec bash"
Start-Process wsl -ArgumentList "-d", $WslDistro, "--", "bash", "-c", $launchInner

Write-Host "[WAIT] waiting 8s for bridge initialization..." -ForegroundColor Yellow
Start-Sleep -Seconds 8

# 5) takeoff
Write-Host "[TAKEOFF] arming $DroneCount drones to $Altitude m..." -ForegroundColor Cyan
$armCmd = "cd /tmp && python3 $WslWorkspace/src/airsim_ros2_bridge/scripts/airsim_arm_all.py --drones $DroneCount --altitude $Altitude --ip $WslHostIp"
wsl -d $WslDistro -- bash -c $armCmd

# 6) mission
Write-Host "[MISSION] sequence: $Sequence" -ForegroundColor Cyan
$missionCmd = "cd /tmp && python3 $WslWorkspace/src/airsim_ros2_bridge/scripts/aerion_mission.py --sequence '$Sequence'"
wsl -d $WslDistro -- bash -c $missionCmd

# 7) land
Write-Host "[LAND] landing all drones..." -ForegroundColor Cyan
$landCmd = "cd /tmp && python3 $WslWorkspace/src/airsim_ros2_bridge/scripts/airsim_land_all.py --drones $DroneCount --ip $WslHostIp"
wsl -d $WslDistro -- bash -c $landCmd

Write-Host "===========================================" -ForegroundColor Green
Write-Host "  AERION Phase 4 demo finished"            -ForegroundColor Green
Write-Host "  Stop bridge launch in the other wsl window with Ctrl+C" -ForegroundColor Yellow
Write-Host "===========================================" -ForegroundColor Green
