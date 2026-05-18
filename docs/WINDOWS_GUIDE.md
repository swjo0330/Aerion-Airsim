# Windows 환경 Phase 4 포메이션 시연 가이드

Linux 노트북(`clrobur`)에서 검증된 코드를 **Windows 머신 + WSL2 Ubuntu-22.04 + ROS2 Humble** 환경에서 시연하는 절차.

## 시스템 구조

```
┌──────────────────────────── Windows Host ────────────────────────────┐
│                                                                       │
│  UE Editor (Windows native, BlocksV2 + Colosseum/AirSim plugin)       │
│   └── AirSim RPC server :41451 ←── WSL에서 RPC 호출                   │
│                                                                       │
│  ┌──────────── WSL2 (Ubuntu-22.04) ────────────┐                      │
│  │ ROS2 Humble + airsim_ros2_bridge + 5대 launch│                      │
│  │ formation_node + leader + tf_publisher       │                      │
│  └─────────────────────────────────────────────┘                      │
└───────────────────────────────────────────────────────────────────────┘
```

**핵심 차이 (Linux 단일 머신 vs Windows+WSL)**:
- AirSim RPC IP: Linux는 `127.0.0.1`, Windows+WSL은 **Windows host IP** (보통 `172.23.80.1` 또는 `wsl --status`로 확인)
- UE Editor: Windows native (Linux UE는 별도)
- 코드 위치: Windows repo `D:/01. PROJECTS/master/UE/AERION/Aerion-Airsim` ↔ WSL `~/aerion_ros2_ws/src/airsim_ros2_bridge` (인수인계 자산 `copy_bridge_to_wsl.ps1` 사용)

## 사전 준비 (한 번만)

### (Windows) Aerion-Airsim repo 최신화

```powershell
cd "D:/01. PROJECTS/master/UE/AERION/Aerion-Airsim"
git fetch origin
git checkout branc/airsim-ros2-bridge
git pull origin branc/airsim-ros2-bridge
```

### (WSL) 사전 의존성

```bash
# WSL Ubuntu-22.04 안에서
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-rmw-cyclonedds-cpp \
                    ros-humble-mavros ros-humble-mavros-extras \
                    python3-colcon-common-extensions python3-pip

# CycloneDDS 환경 (~/.bashrc에 추가)
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# airsim Python 패키지 (Colosseum PythonClient를 user install)
# (사용자 Windows repo 측에 Colosseum 별도 clone 필요 — 인수인계 README 참조)
python3 -m pip install --user --no-build-isolation -e ~/Colosseum/PythonClient
# 또는 단순:
python3 -m pip install --user airsim

# sysctl 영구화 (멀티드론 카메라 buffer)
sudo tee /etc/sysctl.d/30-aerion-dds.conf > /dev/null << 'EOF'
net.core.rmem_max=2147483647
net.core.rmem_default=134217728
net.core.wmem_max=2147483647
net.ipv4.ipfrag_high_thresh=134217728
net.core.netdev_max_backlog=30000
EOF
sudo sysctl --system
```

### (WSL) 코드 복사 + 빌드 (1회)

```powershell
# Windows PowerShell에서 인수인계 자산 사용
cd "D:/01. PROJECTS/master/UE/AERION/Aerion-Airsim"
powershell -ExecutionPolicy Bypass -File scripts/copy_bridge_to_wsl.ps1
powershell -ExecutionPolicy Bypass -File scripts/build_bridge_in_wsl.ps1
```

또는 WSL 내부에서 직접:
```bash
mkdir -p ~/aerion_ros2_ws/src
cp -r /mnt/d/01.\ PROJECTS/master/UE/AERION/Aerion-Airsim/airsim_ros2_bridge ~/aerion_ros2_ws/src/
cd ~/aerion_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select airsim_ros2_bridge
```

## 시연 (Phase 4 5대 포메이션)

### 0) Windows host IP 확인

```powershell
# Windows PowerShell에서 (WSL이 보는 host IP)
wsl hostname -I
# 또는:
ipconfig | findstr "vEthernet (WSL)"
# 출력 예: 172.23.80.1
```

WSL bash에서:
```bash
ip route show | grep default | awk '{print $3}'
# 출력 예: 172.23.80.1
```

이 IP를 `AIRSIM_IP` 환경변수로 export.

### 1) (Windows) UE Editor + Play

```powershell
# settings deploy
$src = "D:/01. PROJECTS/master/UE/AERION/Aerion-Airsim/settings/sf_5drones_phase3.json"
$dst = "$env:USERPROFILE/Documents/AirSim/settings.json"
New-Item -ItemType Directory -Path (Split-Path $dst) -Force | Out-Null
Copy-Item -Path $src -Destination $dst -Force

# UE Editor 실행 (BlocksV2 .uproject 경로는 사용자 환경 맞춤)
# 예: D:/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject
```

UE Editor에서 ▶ Play 클릭, 5대 drone1~5 spawn 확인.

### 2) (WSL) AirSim ping

```bash
cd /tmp && AIRSIM_IP=$(ip route show | grep default | awk '{print $3}') python3 -c "
import os, airsim
ip = os.environ.get('AIRSIM_IP', '172.23.80.1')
c = airsim.MultirotorClient(ip=ip, timeout_value=5.0)
print('AirSim @', ip, ' ping=', c.ping(), ' vehicles=', c.listVehicles())
"
# 기대: ping= True  vehicles= ['drone1'..'drone5']
```

### 3) (WSL) bridge launch (`airsim_ip` 인자에 Windows host IP)

```bash
source /opt/ros/humble/setup.bash
source ~/aerion_ros2_ws/install/setup.bash

# Windows host IP 자동 감지
AIRSIM_IP=$(ip route show | grep default | awk '{print $3}')
echo "AirSim @ $AIRSIM_IP"

# launch (카메라/range 부담 줄이려면 enable_camera:=false enable_range:=false)
ros2 launch airsim_ros2_bridge aerion_phase4_formation.launch.py \
  drone_count:=5 \
  enable_camera:=false \
  enable_range:=false
# launch 내부에서 bridge 5개 + formation + leader + tf publisher 동시 기동.
# bridge 파라미터 airsim_ip를 명시 override (launch 파일은 default 127.0.0.1):
# 또는 별도 환경변수 통해 launch 측 default 변경 (향후 보강).
```

> **주의**: 현재 `aerion_phase4_formation.launch.py`는 `airsim_ip` 기본값이 `127.0.0.1`로 하드코딩. Windows 환경에서는 `bridge_node` 파라미터에 직접 IP 주입 필요. 임시 해결: 직접 노드 실행:

```bash
# 5대 각각 별도 터미널 (또는 한 셸에서 백그라운드)
for n in 1 2 3 4 5; do
  ros2 run airsim_ros2_bridge bridge_node --ros-args \
    -r __ns:=/drone$n \
    -p vehicle_name:=drone$n \
    -p airsim_ip:=$AIRSIM_IP \
    -p airsim_port:=41451 \
    -p enable_camera:=false \
    -p enable_range:=false \
    -p enable_ardu_compat:=false \
    -p control_backend:=airsim_direct \
    -p kinematic_z_ned:=-5.0 &
done

# formation + leader + tf
ros2 run airsim_ros2_bridge aerion_formation --ros-args -p drone_count:=5 &
ros2 run airsim_ros2_bridge aerion_leader --ros-args -p init_z:=5.0 &
ros2 run airsim_ros2_bridge aerion_tf --ros-args -p drone_count:=5 &
```

(향후: launch 파일에 `airsim_ip` argument 추가하는 fix 권장 — 이슈 #TBD)

### 4) (WSL) takeoff + 포메이션 시퀀스

```bash
cd /tmp
AIRSIM_IP=$(ip route show | grep default | awk '{print $3}')

# arm_all (airsim_arm_all.py가 ip 인자 받음)
python3 ~/aerion_ros2_ws/src/airsim_ros2_bridge/scripts/airsim_arm_all.py \
  --drones 5 --altitude 5.0 --ip "$AIRSIM_IP"

# 10초 시퀀스 (시간 짧게)
python3 ~/aerion_ros2_ws/src/airsim_ros2_bridge/scripts/aerion_mission.py \
  --sequence "LINE:10,DIAMOND:10,ARROW:10,V:10,ECHELON:10"

# 종료
python3 ~/aerion_ros2_ws/src/airsim_ros2_bridge/scripts/airsim_land_all.py \
  --drones 5 --ip "$AIRSIM_IP"
```

### 5) (WSL) 검증 (옵션)

```bash
python3 ~/aerion_ros2_ws/src/airsim_ros2_bridge/scripts/verify_topics_phase3.py \
  --drones 5 --json-out /tmp/phase3.json

# RViz (옵션, WSLg + Windows display 필요)
rviz2 -d $(ros2 pkg prefix airsim_ros2_bridge)/share/airsim_ros2_bridge/config/aerion_5drones.rviz
```

## 알려진 함정 (Windows+WSL 특유)

| 함정 | 증상 | 해결 |
|---|---|---|
| AirSim IP가 `127.0.0.1`로 잘못 | `ConnectionRefusedError` 또는 `Stream is closed` | `ip route`로 Windows host IP 확보, `--ip` 또는 param으로 명시 |
| WSL clock과 Windows clock 불일치 | TF 시각 어긋남 | WSL 재시작 또는 `sudo hwclock -s` |
| Windows firewall이 41451 막음 | RPC timeout | Windows Defender Firewall에서 UE Editor 허용 |
| WSL ↔ Windows file system 느림 | colcon build 느림 | WSL native ext4 (`~/aerion_ros2_ws`) 사용, `/mnt/d/`에 직접 빌드 X |
| `python3 -c "import airsim"` namespace 가림 | `AttributeError: MultirotorClient` | `cd /tmp` prefix |
| UE Editor frame rate 부족 | RPC `Stream is closed` 또는 `Request timed out` | UE Editor `Editor Preferences → Use Less CPU when in Background` 비활성 |

## 향후 보강

- [ ] `aerion_phase4_formation.launch.py`에 `airsim_ip` argument 추가 (현재 하드코딩 `127.0.0.1`)
- [ ] `airsim_arm_all.py`/`airsim_land_all.py`에 `--ip` 인자 검증 (이미 있음, 확인 필요)
- [ ] PowerShell wrapper `scripts/run_aerion_phase4_windows.ps1` 작성 (WSL 진입 + 모든 단계 자동화)
- [ ] WSL 안에서 settings deploy 자동 (Windows mount path 통해)
