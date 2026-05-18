# AERION Phase 4 — Windows + WSL2 Ubuntu-22.04 자동 실행 wrapper (v2)
#
# 사전 조건:
#   1. WSL2 Ubuntu-22.04 + ROS2 Humble + airsim_ros2_bridge 빌드 완료
#   2. UE Editor + Colosseum + BlocksV2 빌드 완료, settings 5대 deploy, Play 진입
#
# 사용:
#   powershell -ExecutionPolicy Bypass -File scripts/run_aerion_phase4_windows.ps1
#
# 변경 이력:
#   v1 2026-05-18: 최초 작성 (here-string + bash backslash line continuation 충돌 발견)
#   v2 2026-05-18: here-string 폐기, 한 줄 bash chain, [char]34로 quote escape 단순화

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
Write-Host "  AERION Phase 4 - Windows+WSL Wrapper v2"  -ForegroundColor Cyan
Write-Host "===========================================" -ForegroundColor Cyan

# 1) Windows host IP (WSL이 보는 default gateway)
$WslHostIp = wsl -d $WslDistro -- bash -c "ip route show | grep default | awk '{print `$3}'"
$WslHostIp = $WslHostIp.Trim()
Write-Host "[INFO] AirSim host IP (WSL->Windows): $WslHostIp" -ForegroundColor Yellow

if ([string]::IsNullOrEmpty($WslHostIp)) {
    Write-Error "WSL host IP 확인 실패. WSL이 떠있는지 확인."
    exit 1
}

# 2) AirSim ping
$Q = [char]34   # double quote
$pingCmd = "cd /tmp && python3 -c 'import airsim; c=airsim.MultirotorClient(ip=${Q}$WslHostIp${Q}, timeout_value=5.0); print(${Q}ping=${Q}, c.ping(), ${Q} vehicles=${Q}, c.listVehicles())'"
Write-Host "[CHECK] AirSim RPC ping..." -ForegroundColor Cyan
$pingResult = wsl -d $WslDistro -- bash -c $pingCmd
Write-Host $pingResult
if ($pingResult -notmatch "ping= True") {
    Write-Warning "AirSim ping 실패. UE Editor에서 Play를 누르고 Enter:"
    Read-Host
    $pingResult = wsl -d $WslDistro -- bash -c $pingCmd
    Write-Host $pingResult
    if ($pingResult -notmatch "ping= True") {
        Write-Error "AirSim 여전히 응답 없음. UE Play 상태 확인."
        exit 2
    }
}

# 3) 좀비 정리
Write-Host "[CLEAN] 잔존 ROS 노드 정리..." -ForegroundColor Cyan
wsl -d $WslDistro -- bash -c "pkill -KILL -f 'python3.*bridge_node|aerion_formation|aerion_leader|aerion_tf' 2>/dev/null; sleep 1"

# 4) launch (새 wsl 창, 한 줄 bash 명령)
$enableCamera = if ($SkipCamera) { "false" } else { "true" }
$enableRange  = if ($SkipRange)  { "false" } else { "true" }

Write-Host "[LAUNCH] bridge x $DroneCount + formation + leader + tf (별도 wsl 창)" -ForegroundColor Cyan
$launchInner = "source /opt/ros/humble/setup.bash && source $WslWorkspace/install/setup.bash && ros2 launch airsim_ros2_bridge aerion_phase4_formation.launch.py drone_count:=$DroneCount airsim_ip:=$WslHostIp enable_camera:=$enableCamera enable_range:=$enableRange; exec bash"
Start-Process wsl -ArgumentList "-d", $WslDistro, "--", "bash", "-c", $launchInner

Write-Host "[WAIT] bridge 초기화 ~8초 대기..." -ForegroundColor Yellow
Start-Sleep -Seconds 8

# 5) takeoff
Write-Host "[TAKEOFF] $DroneCount 대 일괄 ($Altitude m)..." -ForegroundColor Cyan
$armCmd = "cd /tmp && python3 $WslWorkspace/src/airsim_ros2_bridge/scripts/airsim_arm_all.py --drones $DroneCount --altitude $Altitude --ip $WslHostIp"
wsl -d $WslDistro -- bash -c $armCmd

# 6) mission
Write-Host "[MISSION] $Sequence" -ForegroundColor Cyan
$missionCmd = "cd /tmp && python3 $WslWorkspace/src/airsim_ros2_bridge/scripts/aerion_mission.py --sequence '$Sequence'"
wsl -d $WslDistro -- bash -c $missionCmd

# 7) land
Write-Host "[LAND] 일괄 착륙..." -ForegroundColor Cyan
$landCmd = "cd /tmp && python3 $WslWorkspace/src/airsim_ros2_bridge/scripts/airsim_land_all.py --drones $DroneCount --ip $WslHostIp"
wsl -d $WslDistro -- bash -c $landCmd

Write-Host "===========================================" -ForegroundColor Green
Write-Host "  AERION Phase 4 시연 완료" -ForegroundColor Green
Write-Host "  bridge launch 종료: 별도 wsl 창에서 Ctrl+C" -ForegroundColor Yellow
Write-Host "===========================================" -ForegroundColor Green
