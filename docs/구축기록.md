# Colosseum + PX4/APM SITL + ROS2 구축 기록

**작성일:** 2026-04-07
**환경:** Ubuntu 22.04.5 LTS, NVIDIA GPU (Driver 580, CUDA 13.0), RAM 62GB

---

## Phase 1: 기반 설치

### 1.1 clang-18 설치

Colosseum의 `build.sh`가 clang-18을 하드코딩하므로 LLVM apt repo에서 설치.

```bash
wget https://apt.llvm.org/llvm.sh && chmod +x llvm.sh
sudo ./llvm.sh 18
sudo apt-get install -y libc++-18-dev libc++abi-18-dev
```

**결과:** Ubuntu clang 18.1.8 설치 완료

**주의사항:**
- `setup.sh`가 clang-12를 설치하면서 libc++-18이 제거될 수 있음 → setup.sh 실행 후 libc++-18-dev 재설치 필요

### 1.2 Python 3.12 + uv 환경

uv가 이미 설치되어 있었음 (v0.10.12). Python 3.12 venv 생성:

```bash
cd /home/clrobur/airsim
uv venv --python 3.12 .venv
source .venv/bin/activate
```

**결과:** CPython 3.12.13, uv가 자동 다운로드

### 1.3 Colosseum 빌드

```bash
git clone https://github.com/CodexLabsLLC/Colosseum.git
cd Colosseum
```

**setup.sh 실행 시 문제 및 해결:**
- `vulkan-utils` 패키지가 Ubuntu 22.04에서 `vulkan-tools`로 변경됨
- `setup.sh` 내 `vulkan-utils` → `vulkan-tools`로 수정 후 실행
- sudo로 실행하면 rpclib/eigen 파일 소유자가 root가 됨 → `chown -R` 필요

```bash
# setup.sh 내 vulkan-utils → vulkan-tools 수정
sed -i 's/vulkan-utils/vulkan-tools/g' setup.sh  # 주의: sed -i가 빈 파일 만들 수 있음, Edit 도구 사용 권장

# sudo로 실행 (의존성 설치에 sudo 필요)
sudo bash setup.sh

# 소유권 복구 (setup.sh를 sudo로 실행한 경우)
sudo chown -R $USER:$USER /home/clrobur/airsim/Colosseum/
```

**build.sh 실행 시 문제 및 해결:**
- `build.sh`가 `-I/usr/lib/llvm-17/include/c++/v1~` 경로를 참조하지만 llvm-17이 없음
- llvm-18 경로로 수정: `-I/usr/lib/llvm-18/include/c++/v1`

```bash
# build.sh 내 llvm-17 → llvm-18 수정
# 라인 11: -I/usr/lib/llvm-17/include/c++/v1~ → -I/usr/lib/llvm-18/include/c++/v1

./build.sh
```

**빌드 결과:**
- `libAirLib.a` (22MB) — 핵심 라이브러리 ✓
- `libMavLinkCom.a` (6MB) — MAVLink 통신 ✓
- `librpc.a` (5MB) — RPC 통신 ✓
- MavLinkTest에서 빌드 에러 발생하지만 핵심 라이브러리에 영향 없음
- 플러그인이 `Unreal/Plugins/AirSim/Source/AirLib/`로 자동 복사됨

### 1.4 BlocksV2.uproject 수정

```json
"EngineAssociation": "5.4"  →  "EngineAssociation": "5.6"
```

### 1.5 airsim Python 패키지 설치

```bash
source .venv/bin/activate
cd Colosseum/PythonClient

# 의존성을 먼저 설치 (빌드 의존성 문제 해결)
uv pip install numpy setuptools msgpack-rpc-python backports.ssl_match_hostname

# editable 설치
uv pip install --no-build-isolation -e .
```

**결과:** airsim 1.8.1 설치 완료

### 1.6 PX4-Autopilot SITL 빌드

```bash
cd /home/clrobur/airsim
git clone https://github.com/PX4/PX4-Autopilot.git --recursive  # ~2-3GB, 수 분 소요

cd PX4-Autopilot

# Python 빌드 의존성 (시스템 Python 3.10에 설치)
pip3 install kconfiglib jsonschema jinja2 packaging pyros-genmsg

make px4_sitl_default none_iris  # 5-10분 소요
```

**결과:** `build/px4_sitl_default/bin/px4` 바이너리 생성, 기동 테스트 성공

**PX4 SITL 기동 테스트 출력 (정상):**
```
INFO  [px4] instance: 0
px4 starting.
INFO  [px4] startup script: /bin/sh etc/init.d-posix/rcS 0
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
```
센서 경고(Accel uncalibrated 등)는 시뮬레이터 미연결 상태이므로 정상.

---

## Phase 2: 설정 및 스크립트

### 2.1 settings.json (PX4 2대)

파일: `settings/px4_dual.json`

| 항목 | Drone0 | Drone1 |
|------|--------|--------|
| VehicleType | PX4Multirotor | PX4Multirotor |
| TcpPort (SITL↔Colosseum) | 4560 | 4561 |
| ControlPortRemote (→MAVROS) | 14555 | 14556 |
| MAV_SYS_ID | 1 | 2 |
| 카메라 | 1280x720, FOV 90° | 1280x720, FOV 90° |
| 초기 위치 | (0,0,0) | (5,0,0) |

`LocalHostIp`는 현재 `127.0.0.1` (로컬 테스트용). 원격 MAVROS 연결 시 해당 IP로 변경.

### 2.2 settings.json (APM 2대)

파일: `settings/apm_dual.json`

| 항목 | Drone0 | Drone1 |
|------|--------|--------|
| VehicleType | ArduCopter | ArduCopter |
| UdpPort (센서) | 9003 | 9005 |
| ControlPort (제어) | 9002 | 9004 |
| ClockType | "" (빈 문자열) | "" |

### 2.3 런치 스크립트

| 스크립트 | 용도 |
|---------|------|
| `scripts/launch_ue_blocksv2.sh [px4\|apm]` | UE 에디터 실행 + settings.json 복사 |
| `scripts/launch_px4_dual.sh` | PX4 SITL 2개 인스턴스 기동 |
| `scripts/launch_apm_dual.sh` | ArduPilot SITL 2개 인스턴스 기동 |

---

## Phase 3: Python ROS2 브릿지

### 3.1 패키지 구조

```
airsim_ros2_bridge/
├── package.xml                         # ROS2 패키지 매니페스트
├── setup.py / setup.cfg                # ament_python 빌드
├── resource/airsim_ros2_bridge         # 패키지 마커
└── airsim_ros2_bridge/
    ├── __init__.py
    ├── bridge_node.py                  # 메인 노드 (AirSim 연결, 드론별 publisher/controller 생성)
    ├── camera_publisher.py             # 카메라 이미지 + CameraInfo 발행
    ├── drone_controller.py             # cmd_vel, cmd_pos 토픽 → AirSim API
    └── utils.py                        # FOV→intrinsics 변환, RGBA→RGB 변환
```

### 3.2 발행 토픽 (드론별)

| 토픽 | 메시지 타입 | 설명 |
|------|-----------|------|
| `/{DroneN}/camera/image` | `sensor_msgs/Image` | RGB 이미지 (30fps) |
| `/{DroneN}/camera/camera_info` | `sensor_msgs/CameraInfo` | 카메라 intrinsics (K, P, D 행렬) |

### 3.3 구독 토픽 (드론 제어)

| 토픽 | 메시지 타입 | 설명 |
|------|-----------|------|
| `/{DroneN}/cmd_vel` | `geometry_msgs/Twist` | 속도 제어 (NED, m/s) |
| `/{DroneN}/cmd_pos` | `geometry_msgs/PoseStamped` | 위치 제어 (NED) |

### 3.4 카메라 파라미터 변환

AirSim은 핀홀 카메라 모델 (렌즈 왜곡 없음):
```
fx = width / (2 * tan(FOV_rad / 2))
fy = fx  (정사각 픽셀)
cx = width / 2
cy = height / 2
D = [0, 0, 0, 0, 0]  (왜곡 없음)
```

### 3.5 colcon 빌드

```bash
source /opt/ros/humble/setup.bash
cd /home/clrobur/airsim
colcon build --packages-select airsim_ros2_bridge
```

### 3.6 실행 방법

```bash
source /opt/ros/humble/setup.bash
source /home/clrobur/airsim/install/setup.bash
source /home/clrobur/airsim/.venv/bin/activate

ros2 run airsim_ros2_bridge bridge_node \
  --ros-args \
  -p vehicle_names:="['Drone0', 'Drone1']" \
  -p camera_fps:=30.0 \
  -p airsim_ip:="127.0.0.1"
```

---

## 기동 순서 (전체 파이프라인)

### PX4 모드

```
1. UE 에디터 실행: ./scripts/launch_ue_blocksv2.sh px4
2. UE 에디터에서 Play 클릭
3. PX4 SITL 기동: ./scripts/launch_px4_dual.sh
4. ROS2 브릿지 기동: ros2 run airsim_ros2_bridge bridge_node
5. (원격) MAVROS 에서 MAVLink 수신 확인
```

### APM 모드

```
1. UE 에디터 실행: ./scripts/launch_ue_blocksv2.sh apm
2. UE 에디터에서 Play 클릭
3. APM SITL 기동: ./scripts/launch_apm_dual.sh
4. ROS2 브릿지 기동: ros2 run airsim_ros2_bridge bridge_node
```

---

## 알려진 문제 및 해결

| 문제 | 원인 | 해결 |
|------|------|------|
| setup.sh에서 `vulkan-utils` 패키지 못 찾음 | Ubuntu 22.04에서 이름 변경 | `vulkan-tools`로 수정 |
| build.sh에서 llvm-17 경로 없음 | build.sh가 llvm-17 하드코딩 | `llvm-18`로 수정 |
| CMake에서 rpclib 파일 쓰기 실패 | setup.sh를 sudo로 실행해 소유자가 root | `chown -R` 실행 |
| PX4 빌드 시 kconfiglib 없음 | Python 빌드 의존성 미설치 | `pip3 install kconfiglib pyros-genmsg` |
| airsim pip 설치 시 의존성 체인 | 빌드 의존성이 미선언 | numpy, setuptools, msgpack-rpc-python 사전 설치 |
| MavLinkTest 빌드 실패 | 테스트 코드 컴파일 에러 | 무시 가능 (핵심 라이브러리는 정상 빌드) |
| UE 에디터에서 AirSim 플러그인 못 찾음 | BlocksV2 Plugins/ 디렉토리에 AirSim 없음 | `ln -s` 심볼릭 링크 생성 |
| UWorld::LineBatcher 제거 (UE 5.6) | PawnSimApi.cpp, WorldSimApi.cpp | `DrawDebugLine()` 함수로 대체 |
| TObjectPtr 암시적 변환 실패 | SimHUD.cpp `auto*` → TObjectPtr | `APawn*`로 명시적 타입 지정 |
| 링커 에러: undefined symbols | AirSim.Build.cs의 CompileMode 불일치 | `CppCompileWithRpc` → `HeaderOnlyWithRpc`로 변경 |
| ABI 불일치 (libAirLib.a) | 시스템 clang-18 vs UE 번들 clang-18 | build.sh에서 UE 번들 clang 사용하도록 수정 |

---

## UE 5.6 호환 패치 목록

Colosseum은 UE 5.2 기준으로 작성되어 UE 5.6에서 빌드하려면 아래 5개 파일을 수정해야 한다.

| # | 파일 | 변경 내용 |
|---|------|----------|
| 1 | `Colosseum/Unreal/Plugins/AirSim/Source/PawnSimApi.cpp` | LineBatcher → DrawDebugLine |
| 2 | `Colosseum/Unreal/Plugins/AirSim/Source/WorldSimApi.cpp` | PersistentLineBatcher → DrawDebugLine |
| 3 | `Colosseum/Unreal/Plugins/AirSim/Source/SimHUD/SimHUD.cpp` | auto* → APawn* (TObjectPtr 암시적 변환 해결) |
| 4 | `Colosseum/Unreal/Plugins/AirSim/Source/AirSim.Build.cs` | CppCompileWithRpc → HeaderOnlyWithRpc (링커 심볼 불일치 해결) |
| 5 | `Colosseum/build.sh` | 시스템 clang-18 → UE 번들 clang-18 경로 (ABI 불일치 해결) |

---

## 잠재적 런타임 이슈 (UE 5.6)

빌드는 성공했으나 런타임에서 문제가 될 수 있는 deprecated API 사용 목록:

| 우선순위 | 파일 | 이슈 | 대응 |
|---------|------|------|------|
| 높음 | AirBlueprintLib.h:61, WorldSimApi.cpp:69 | `IsPendingKillPending()` deprecated | `IsValid()` 또는 `IsActorBeingDestroyed()`로 교체 필요 시 |
| 중간 | UnrealImageCapture.cpp:150 | `FImageUtils::CompressImageArray()` deprecated (UE 5.1~) | 경고만 발생, 제거 시 `PNGCompressImageArray()`로 교체 |
| 중간 | RenderRequest.cpp | `FRenderCommand`, `ReadSurfaceData()` API 변경 가능 | 이미지 캡처 기능 테스트로 확인 |
| 낮음 | WorldSimApi.cpp, PIPCamera.h | 레거시 include 경로 (`Runtime/Engine/Classes/...`) | 빌드 성공했으므로 현재 문제 없음 |

**참고:** 빌드가 성공(Result: Succeeded)했으므로 위 이슈들은 UE 5.6에서 아직 제거되지 않은 상태. 실제 시뮬레이션 실행 시 카메라 이미지 캡처 등을 테스트하여 확인 필요.

---

## Phase 2 통합 검증 결과 (2026-04-08)

### 검증 환경
- UE 5.6.1 에디터 + BlocksV2 + AirSim 플러그인
- PX4 SITL 2대 (instance 0: TCP 4560, instance 1: TCP 4561)
- Python ROS2 브릿지 (시스템 Python 3.10 + rclpy)

### 검증 결과

| 항목 | 상태 | 비고 |
|------|------|------|
| UE 에디터 BlocksV2 로드 | **성공** | AirSim 플러그인 심볼릭 링크 필요 |
| AirSim C++ 빌드 (UE 5.6) | **성공** | 5개 패치 적용 후 Result: Succeeded |
| PX4 SITL TCP 연결 | **성공** | `Simulator connected on TCP port 4560/4561` |
| 드론 2대 스폰 | **성공** | Drone0 (0,0,0), Drone1 (5,0,0) |
| airsim Python API 연결 | **성공** | `confirmConnection()` 정상 |
| API 제어 (이륙/이동) | **성공** | `enableApiControl` → `takeoffAsync` → `moveToPositionAsync` |
| 카메라 이미지 캡처 | **성공** | 1280x720, RGB 3채널 (RGBA가 아닌 RGB로 반환됨) |
| 카메라 파라미터 | **성공** | FOV=89.9°, fx=641.08, plumb_bob 모델 |
| ROS2 `/Drone0/camera/image` 발행 | **성공** | sensor_msgs/Image |
| ROS2 `/Drone0/camera/camera_info` 발행 | **성공** | K=[641,0,640, 0,641,360, 0,0,1] |
| ROS2 `/Drone1/camera/*` 발행 | **성공** | Drone0과 동일 스펙 |
| ROS2 `/DroneN/cmd_vel`, `/cmd_pos` 수신 대기 | **성공** | geometry_msgs/Twist, PoseStamped |

### 발견 사항

1. **이미지 채널:** AirSim이 UE 5.6에서 RGB 3채널로 반환 (기존 문서에서는 RGBA 4채널로 기술됨). 브릿지 코드에서 자동 감지 처리 완료.

2. **PX4 SITL 실행 모드:** `PX4_SIM_MODEL=none_iris` 환경변수를 설정해야 AirSim TCP 연결 모드로 기동. 미설정 시 SIH(내부 시뮬레이터)로 기동되어 Colosseum과 연결 안 됨.

3. **ROS2 Python 버전:** ROS2 Humble의 rclpy는 Python 3.10 바인딩이므로, `.venv`의 Python 3.12에서는 사용 불가. ROS2 브릿지는 **시스템 Python 3.10**으로 실행해야 함. airsim 패키지도 시스템 Python에 설치 필요.

4. **마우스 캡처:** Play 시 AirSim이 마우스를 캡처함. **F1** 키로 마우스 해제.

### 실제 기동 순서 (검증됨)

```bash
# 1. settings.json 배포
cp /home/clrobur/airsim/settings/px4_dual.json ~/Documents/AirSim/settings.json

# 2. UE 에디터 실행 (GUI 필요, 별도 터미널에서)
/home/clrobur/airsim/unreal-engine/Engine/Binaries/Linux/UnrealEditor \
  /home/clrobur/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject

# 3. UE 에디터에서 Play 클릭

# 4. PX4 SITL 2대 기동
cd /home/clrobur/airsim/PX4-Autopilot/build/px4_sitl_default
PX4_SIM_MODEL=none_iris ./bin/px4 -i 0 -d etc &
sleep 3
PX4_SIM_MODEL=none_iris ./bin/px4 -i 1 -d etc &

# 5. ROS2 브릿지 기동 (시스템 Python 3.10)
export PYTHONPATH="/home/clrobur/airsim/Colosseum/PythonClient:/home/clrobur/airsim/airsim_ros2_bridge:$PYTHONPATH"
source /opt/ros/humble/setup.bash
python3 /home/clrobur/airsim/airsim_ros2_bridge/airsim_ros2_bridge/bridge_node.py

# 6. 토픽 확인
ros2 topic list | grep Drone
ros2 topic echo /Drone0/camera/camera_info --once
```

### 드론 제어 데모 (Python API)

```python
import airsim
client = airsim.MultirotorClient()
client.confirmConnection()

# 두 드론 API 제어 활성화 + 시동
for name in ["Drone0", "Drone1"]:
    client.enableApiControl(True, vehicle_name=name)
    client.armDisarm(True, vehicle_name=name)

# 동시 이륙
f0 = client.takeoffAsync(vehicle_name="Drone0")
f1 = client.takeoffAsync(vehicle_name="Drone1")
f0.join(); f1.join()

# 병렬 이동 (NED 좌표계)
f0 = client.moveToPositionAsync(-10, 0, -5, 5, vehicle_name="Drone0")
f1 = client.moveToPositionAsync(0, 10, -5, 5, vehicle_name="Drone1")
f0.join(); f1.join()
```

---

## Phase 4: ArduPilot(APM) SITL 구축

### 4.1 ArduPilot 설치

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### 4.2 ArduCopter SITL 빌드

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

### 4.3 settings.json (APM 2대)

파일: `settings/apm_dual.json` (이미 생성됨)

자동 생성 스크립트로도 생성 가능:
```bash
python3 scripts/generate_settings.py -f ardupilot -n 2 --deploy
```
상세 옵션은 아래 "설정 자동 생성기 (generate_settings.py)" 섹션 참조.

| 항목 | Drone0 | Drone1 |
|------|--------|--------|
| VehicleType | ArduCopter | ArduCopter |
| UdpPort (센서) | 9003 | 9013 |
| ControlPort (제어) | 9002 | 9012 |
| ClockType | "" | "" |

### 4.4 기동 순서

```
1. settings 배포: cp settings/apm_dual.json ~/Documents/AirSim/settings.json
2. UE 에디터 실행 + Play
3. ArduPilot SITL 기동: ./scripts/launch_apm_dual.sh
4. ROS2 브릿지 기동
```

### 4.5 검증 결과

(진행 중)

---

## 설정 자동 생성기 (generate_settings.py)

펌웨어 유형(PX4/ArduPilot)에 따라 포트 스킴이 다르므로, 자동 생성 스크립트를 사용한다.

### 사용법

```bash
# PX4 2대 (기본)
python3 scripts/generate_settings.py -f px4 -n 2 --deploy

# ArduPilot 2대
python3 scripts/generate_settings.py -f ardupilot -n 2 --deploy

# PX4 3대 + 원격 MAVROS
python3 scripts/generate_settings.py -f px4 -n 3 --mavros-ip 192.168.1.100 --deploy

# ArduPilot 4대
python3 scripts/generate_settings.py -f ardupilot -n 4 -o settings/apm_quad.json
```

### 옵션

| 옵션 | 설명 |
|------|------|
| `-f`, `--firmware` | 펌웨어 유형 (`px4` 또는 `ardupilot`) |
| `-n`, `--drones` | 드론 수 (기본: 2) |
| `--mavros-ip` | 원격 MAVROS IP (기본: 127.0.0.1) |
| `-o`, `--output` | 출력 파일 경로 |
| `-d`, `--deploy` | `~/Documents/AirSim/settings.json`에 자동 복사 |

### 펌웨어별 포트 오프셋 규칙

| 펌웨어 | 인스턴스 오프셋 | 예시 (Instance 0 → 1) |
|--------|---------------|----------------------|
| PX4 | +1 | TcpPort: 4560 → 4561 |
| ArduPilot | +10 | UdpPort: 9003 → 9013, ControlPortLocal: 9002 → 9012 |

### 주의사항

- ArduPilot의 `sim_vehicle.py --instance N`은 포트에 **N * 10** 오프셋 적용 (PX4의 +1과 다름)
- ArduCopter API는 `enableApiControl`/`armDisarm` **미구현** → MAVLink(MAVROS) 통한 제어 필수
- PX4는 `PX4_SIM_MODEL=none_iris` 환경변수 필수

---

## 파일 구조 (현재)

```
/home/clrobur/airsim/
├── .venv/                              # Python 3.12 가상환경 (uv)
├── CLAUDE.md                           # 프로젝트 지침 + 체크리스트
├── pyproject.toml                      # uv 프로젝트 의존성
├── Colosseum/                          # AirSim 포크 (빌드 완료)
│   ├── build.sh                        # 수정됨 (llvm-18 경로)
│   ├── setup.sh                        # 수정됨 (vulkan-tools)
│   ├── AirLib/lib/x64/Release/         # 빌드 결과물
│   ├── PythonClient/                   # airsim Python 패키지
│   └── Unreal/
│       ├── Plugins/AirSim/             # UE 플러그인
│       └── Environments/BlocksV2/      # 기본 환경 (5.6 수정됨)
├── PX4-Autopilot/                      # PX4 SITL (빌드 완료)
│   └── build/px4_sitl_default/bin/px4
├── settings/
│   ├── px4_dual.json                   # PX4 2대 설정
│   └── apm_dual.json                   # APM 2대 설정
├── scripts/
│   ├── launch_ue_blocksv2.sh           # UE 실행 스크립트
│   ├── launch_px4_dual.sh              # PX4 SITL 실행
│   └── launch_apm_dual.sh             # APM SITL 실행
├── airsim_ros2_bridge/                 # ROS2 브릿지 (colcon 빌드 완료)
│   └── airsim_ros2_bridge/
│       ├── bridge_node.py
│       ├── camera_publisher.py
│       ├── drone_controller.py
│       └── utils.py
└── docs/
    ├── 구축기록.md                      # 이 문서
    └── superpowers/
        ├── specs/                       # 설계 문서
        └── plans/                       # 구현 계획
```
