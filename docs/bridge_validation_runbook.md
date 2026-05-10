# AirSim ROS2 Bridge + MAVROS 검증 런북 (2026-05-10)

이 문서는 `branc/airsim-ros2-bridge` 기준, 현재 브릿지/카메라/MAVROS 호환성 검증 절차를 재현하기 위한 운영 노트다.

## 1) 검증 환경

- Windows host: Unreal + AirSim RPC 서버 실행
- WSL: `Ubuntu-22.04`
- ROS2: Humble
- 작업 경로:
  - Windows repo: `D:/01. PROJECTS/master/UE/AERION/Aerion-Airsim`
  - WSL ROS2 ws: `~/aerion_ros2_ws`
  - WSL bridge src: `~/aerion_ros2_ws/src/airsim_ros2_bridge`
- AirSim RPC:
  - Host: `172.23.80.1`
  - Port: `41451`

## 2) 사전 조건

1. Unreal/AirSim이 실행 중이고 RPC ping이 성공해야 한다.
2. WSL에서 `airsim` Python 패키지는 Colosseum PythonClient 기준으로 맞춰져 있어야 한다.
3. PX4 듀얼 + MAVROS 듀얼 실행 가능 상태여야 한다.

AirSim RPC ping 체크:

```bash
python3 - << 'PY'
import airsim
c=airsim.MultirotorClient(ip='172.23.80.1', port=41451, timeout_value=2.0)
print(c.ping())
PY
```

## 3) 실행 순서 (3 터미널)

### 터미널 A: PX4 듀얼

```bash
cd ~/aerion_ros2_ws/src/airsim_ros2_bridge
PX4_SIM_HOSTNAME=172.23.80.1 ./scripts/launch_px4_dual.sh
```

### 터미널 B: MAVROS 듀얼

```bash
cd ~/aerion_ros2_ws/src/airsim_ros2_bridge
./scripts/launch_mavros_px4_dual.sh
```

### 터미널 C: Bridge

```bash
cd ~/aerion_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run airsim_ros2_bridge bridge_node --ros-args \
  -p airsim_ip:=172.23.80.1 \
  -p airsim_port:=41451 \
  -p airsim_timeout_sec:=5.0 \
  -p enable_camera:=true \
  -p enable_ardu_compat:=true \
  -p ardu_compat_vehicle:=Drone0 \
  -p velocity_control_mode:=kinematic \
  -p control_backend:=px4_mavros \
  -p velocity_command_duration:=0.2 \
  -p kinematic_z_ned:=-1.0
```

## 4) 검증 명령

### MAVROS 상태

```bash
source /opt/ros/humble/setup.bash
source ~/aerion_ros2_ws/install/setup.bash
ros2 topic echo --once /mavros0/state
ros2 topic echo --once /mavros1/state
```

합격 기준:
- `connected: true`

### 카메라

```bash
source /opt/ros/humble/setup.bash
source ~/aerion_ros2_ws/install/setup.bash
ros2 topic echo --once /Drone0/camera/camera_info
ros2 topic hz /Drone0/camera/image
```

합격 기준:
- `camera_info`가 정상 출력 (예: 1280x720, frame_id 포함)
- `/Drone0/camera/image` 발행 확인

### 통합 스모크

```bash
cd ~/aerion_ros2_ws/src/airsim_ros2_bridge
./scripts/smoke_airsim_ros2_bridge.sh
```

합격 기준:
- `Smoke test passed.`

## 5) 이번 주기에서 반영된 핵심 변경

- 기본 control backend를 `px4_mavros`로 고정
- MAVROS alias 및 서비스 계층 확장
- 카메라 RPC 호환 처리:
  - `simGetImages`/`simGetCameraInfo` 호출 폴백
  - 카메라 이름 후보 탐색(`front_center`, `0`, `1`)
- 멀티스레드 executor 환경에서 AirSim RPC 재진입 충돌 방지:
  - AirSim client 호출 직렬화 래퍼 추가

## 6) 자주 발생한 오류와 대응

### A. `getpwnam(branc) failed`
- 증상: WSL이 사용자 해석 실패로 모든 `bash -lc` 실행 실패
- 대응: `-u root`로 진입 후 `/etc/passwd`, `/etc/wsl.conf` 복구 + `wsl --shutdown`

### B. `RTPS_TRANSPORT_SHM ... open_and_lock_file failed`
- 증상: FastDDS shared-memory 포트 락 경고
- 대응: `/dev/shm/fastrtps_*` 정리, daemon 재시작, 필요 시 `rmw_cyclonedds_cpp` 사용

### C. `IOLoop is already running`
- 원인: 멀티스레드 콜백에서 AirSim RPC 동시 호출
- 대응: AirSim client 호출을 락으로 직렬화

### D. `simGetImages ... bad cast`
- 원인: AirSim Python client/서버 스키마 불일치
- 대응: WSL `airsim`을 Colosseum PythonClient 기준으로 정렬

