# AERION Phase 4 — Windows + WSL2 Ubuntu-22.04 자동 실행 wrapper
#
# 사전 조건:
#   1. WSL2 Ubuntu-22.04 + ROS2 Humble + airsim_ros2_bridge 빌드 완료
#   2. UE Editor + Colosseum + BlocksV2 빌드 완료
#   3. settings.json 5대 deploy 완료 (sf_5drones_phase3.json → ~/Documents/AirSim/settings.json)
#
# 사용:
#   powershell -ExecutionPolicy Bypass -File scripts/run_aerion_phase4_windows.ps1
#   powershell -ExecutionPolicy Bypass -File scripts/run_aerion_phase4_windows.ps1 -SkipCamera $false -Sequence "LINE:30,DIAMOND:30,ARROW:30"
#
# 흐름:
#   1. Windows host IP (WSL이 보는) 확인
#   2. 사용자에게 "UE Editor Play 후 Enter" 확인
#   3. WSL 진입 → ros2 launch (bridge × 5 + formation + leader + tf)
#   4. 새 WSL 셸에서 airsim_arm_all + mission + land_all
#
# 변경 이력:
#   2026-05-18 v1: 최초 작성 (Windows+WSL 자동 wrapper)

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
Write-Host "  AERION Phase 4 - Windows+WSL Wrapper" -ForegroundColor Cyan
Write-Host "===========================================" -ForegroundColor Cyan

# 1) Windows host IP (WSL이 보는)
$WslHostIp = wsl -d $WslDistro -- bash -c "ip route show | grep default | awk '{print \$3}'"
$WslHostIp = $WslHostIp.Trim()
Write-Host "[INFO] AirSim host IP (WSL→Windows): $WslHostIp" -ForegroundColor Yellow

if ([string]::IsNullOrEmpty($WslHostIp)) {
    Write-Error "WSL host IP 확인 실패. WSL이 떠있는지 확인."
    exit 1
}

# 2) AirSim RPC ping 검증
Write-Host "[CHECK] AirSim RPC ping..." -ForegroundColor Cyan
$pingResult = wsl -d $WslDistro -- bash -c "cd /tmp && python3 -c 'import airsim; c=airsim.MultirotorClient(ip=\`"$WslHostIp\`", timeout_value=5.0); print(\`"ping=\`",c.ping(),\`" vehicles=\`",c.listVehicles())'"
Write-Host $pingResult
if ($pingResult -notmatch "ping= True") {
    Write-Warning "AirSim ping 실패. UE Editor에서 Play를 누르고 Enter:"
    Read-Host
    # 재시도
    $pingResult = wsl -d $WslDistro -- bash -c "cd /tmp && python3 -c 'import airsim; c=airsim.MultirotorClient(ip=\`"$WslHostIp\`", timeout_value=5.0); print(\`"ping=\`",c.ping(),\`" vehicles=\`",c.listVehicles())'"
    Write-Host $pingResult
    if ($pingResult -notmatch "ping= True") {
        Write-Error "AirSim 여전히 응답 없음. UE Play 상태 확인."
        exit 2
    }
}

# 3) 좀비 정리
Write-Host "[CLEAN] 잔존 ROS 노드 정리..." -ForegroundColor Cyan
wsl -d $WslDistro -- bash -c "pkill -KILL -f 'python3.*bridge_node|aerion_formation|aerion_leader|aerion_tf' 2>/dev/null; sleep 1"

# 4) launch (백그라운드 새 wsl 윈도우)
$enableCamera = if ($SkipCamera) { "false" } else { "true" }
$enableRange = if ($SkipRange) { "false" } else { "true" }

Write-Host "[LAUNCH] bridge × $DroneCount + formation + leader + tf (별도 wsl 창)" -ForegroundColor Cyan
$launchCmd = @"
source /opt/ros/humble/setup.bash && \
source $WslWorkspace/install/setup.bash && \
ros2 launch airsim_ros2_bridge aerion_phase4_formation.launch.py \
  drone_count:=$DroneCount \
  airsim_ip:=$WslHostIp \
  enable_camera:=$enableCamera \
  enable_range:=$enableRange ; exec bash
"@
Start-Process wsl -ArgumentList "-d", $WslDistro, "--", "bash", "-c", $launchCmd

Write-Host "[WAIT] bridge 초기화 ~8초 대기..." -ForegroundColor Yellow
Start-Sleep -Seconds 8

# 5) takeoff
Write-Host "[TAKEOFF] $DroneCount 대 일괄 ($Altitude m)..." -ForegroundColor Cyan
wsl -d $WslDistro -- bash -c "cd /tmp && python3 $WslWorkspace/src/airsim_ros2_bridge/scripts/airsim_arm_all.py --drones $DroneCount --altitude $Altitude --ip $WslHostIp"

# 6) mission
Write-Host "[MISSION] $Sequence" -ForegroundColor Cyan
wsl -d $WslDistro -- bash -c "cd /tmp && python3 $WslWorkspace/src/airsim_ros2_bridge/scripts/aerion_mission.py --sequence `"$Sequence`""

# 7) land
Write-Host "[LAND] 일괄 착륙..." -ForegroundColor Cyan
wsl -d $WslDistro -- bash -c "cd /tmp && python3 $WslWorkspace/src/airsim_ros2_bridge/scripts/airsim_land_all.py --drones $DroneCount --ip $WslHostIp"

Write-Host "===========================================" -ForegroundColor Green
Write-Host "  AERION Phase 4 시연 완료" -ForegroundColor Green
Write-Host "  bridge launch 종료: 별도 wsl 창에서 Ctrl+C" -ForegroundColor Yellow
Write-Host "===========================================" -ForegroundColor Green
