# Colosseum + PX4/APM SITL + ROS2 Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a complete drone simulation environment with Colosseum (UE 5.6), PX4/APM SITL for 2 drones, and a Python ROS2 bridge that publishes camera data and accepts control commands.

**Architecture:** Colosseum runs as an Unreal Engine 5.6 plugin simulating drone physics and rendering. PX4 SITL instances connect via TCP lockstep to Colosseum and send MAVLink telemetry over UDP to a remote MAVROS machine. A Python bridge (rclpy + airsim client) extracts camera images/parameters from Colosseum and publishes them as ROS2 topics. ArduPilot SITL is configured as a secondary flight controller option after PX4 is validated.

**Tech Stack:** Unreal Engine 5.6.1, Colosseum (main branch), PX4-Autopilot SITL, ArduPilot SITL, Python 3.12+ (uv + venv), rclpy, airsim Python client, clang-18

**Spec:** `docs/superpowers/specs/2026-04-07-colosseum-px4-sitl-ros2-design.md`

---

## File Structure

```
/home/clrobur/airsim/
├── Colosseum/                          # Cloned from GitHub (Task 2)
│   ├── setup.sh                        # Dependency installer
│   ├── build.sh                        # Build script (uses clang-18)
│   ├── PythonClient/                   # airsim Python package
│   └── Unreal/
│       ├── Plugins/AirSim/             # Built UE plugin
│       └── Environments/BlocksV2/      # Default environment
│           └── BlocksV2.uproject       # Modify: EngineAssociation → 5.6
├── pyproject.toml                      # uv project deps (numpy, opencv, pymavlink)
├── PX4-Autopilot/                      # Cloned from GitHub (Task 4)
├── settings/
│   ├── px4_dual.json                   # PX4 2-drone settings.json
│   └── apm_dual.json                   # APM 2-drone settings.json (Task 8)
├── airsim_ros2_bridge/                 # Python ROS2 bridge (Task 6-7)
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/airsim_ros2_bridge
│   └── airsim_ros2_bridge/
│       ├── __init__.py
│       ├── bridge_node.py              # Main entry point, spawns per-drone publishers
│       ├── camera_publisher.py         # Camera image + CameraInfo publishing
│       ├── drone_controller.py         # ROS2 topic → airsim control commands
│       └── utils.py                    # FOV→intrinsics, image format conversion
├── scripts/
│   ├── launch_px4_dual.sh              # Launches 2 PX4 SITL instances
│   ├── launch_apm_dual.sh             # Launches 2 APM SITL instances (Task 8)
│   └── launch_ue_blocksv2.sh          # Launches UE editor with BlocksV2
└── docs/
    └── superpowers/
        ├── specs/2026-04-07-colosseum-px4-sitl-ros2-design.md
        └── plans/2026-04-07-colosseum-px4-apm-ros2.md
```

---

## Task 1: Install System Dependencies (clang-18, Python 3.12, Vulkan)

**Files:**
- No files created/modified — system package installation only

- [ ] **Step 1: Add LLVM apt repository and install clang-18**

```bash
wget https://apt.llvm.org/llvm.sh
chmod +x llvm.sh
sudo ./llvm.sh 18
sudo apt-get install -y libc++-18-dev libc++abi-18-dev
rm llvm.sh
```

- [ ] **Step 2: Verify clang-18 installation**

Run: `clang-18 --version && clang++-18 --version`
Expected: `clang version 18.x.x` output for both

- [ ] **Step 3: Install uv (Python package manager)**

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env 2>/dev/null || export PATH="$HOME/.local/bin:$PATH"
```

- [ ] **Step 4: Verify uv installation**

Run: `uv --version`
Expected: `uv 0.x.x` or newer

- [ ] **Step 5: Create Python 3.12 venv with uv**

```bash
cd /home/clrobur/airsim
uv venv --python 3.12 .venv
source .venv/bin/activate
python --version
```

Expected: `Python 3.12.x` — uv will auto-download Python 3.12 if not installed.

- [ ] **Step 6: Create pyproject.toml for project dependency management**

Create `/home/clrobur/airsim/pyproject.toml`:

```toml
[project]
name = "airsim-simulation"
version = "0.1.0"
requires-python = ">=3.12"
dependencies = [
    "numpy",
    "opencv-python",
    "pymavlink",
]

[tool.uv]
dev-dependencies = []
```

Then lock and sync:

```bash
cd /home/clrobur/airsim
uv pip sync pyproject.toml 2>/dev/null || uv pip install -r pyproject.toml
```

- [ ] **Step 7: Install Vulkan tools (if not present)**

```bash
sudo apt-get install -y libvulkan1 vulkan-tools
vulkaninfo --summary 2>/dev/null | head -5
```

Expected: Vulkan info showing NVIDIA GPU

- [ ] **Step 8: Commit environment notes**

```bash
cd /home/clrobur/airsim
git init
git add CLAUDE.md docs/ pyproject.toml
git commit -m "docs: initial project setup with design spec and implementation plan"
```

---

## Task 2: Clone and Build Colosseum

**Files:**
- Create: `Colosseum/` (git clone)
- Modify: `Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject` (EngineAssociation)

- [ ] **Step 1: Clone Colosseum repository**

```bash
cd /home/clrobur/airsim
git clone https://github.com/CodexLabsLLC/Colosseum.git
```

Expected: Colosseum directory created with `setup.sh`, `build.sh`, etc.

- [ ] **Step 2: Run setup.sh to install build dependencies**

```bash
cd /home/clrobur/airsim/Colosseum
./setup.sh
```

Expected: Downloads rpclib v2.3.0, Eigen 3.4.0, installs apt packages. Note: This installs clang-12 but we already have clang-18 from Task 1.

- [ ] **Step 3: Verify clang-18 is available for build.sh**

build.sh hardcodes `CC=/usr/bin/clang-18` and `CXX=/usr/bin/clang++-18`. Verify:

```bash
ls -la /usr/bin/clang-18 /usr/bin/clang++-18
```

Expected: Both symlinks/binaries exist.

- [ ] **Step 4: Check build.sh for llvm-17 include path issue**

build.sh references `-I/usr/lib/llvm-17/include/c++/v1~`. This path may not exist with clang-18. Check and fix if needed:

```bash
grep -n "llvm-17" /home/clrobur/airsim/Colosseum/build.sh
```

If found, check if the path exists:
```bash
ls /usr/lib/llvm-17/include/c++/v1/ 2>/dev/null || echo "Path does not exist"
ls /usr/lib/llvm-18/include/c++/v1/ 2>/dev/null || echo "Path does not exist either"
```

If llvm-17 path doesn't exist but llvm-18 does, patch build.sh:
```bash
sed -i 's|llvm-17|llvm-18|g' /home/clrobur/airsim/Colosseum/build.sh
```

- [ ] **Step 5: Run build.sh**

```bash
cd /home/clrobur/airsim/Colosseum
./build.sh
```

Expected: Builds `libAirLib.a`, `libMavLinkCom.a`, `librpc.a`. Copies them to `Unreal/Plugins/AirSim/Source/AirLib/`. This may take 5-15 minutes.

- [ ] **Step 6: Verify build artifacts**

```bash
ls -la /home/clrobur/airsim/Colosseum/AirLib/lib/x64/Release/libAirLib.a
ls -la /home/clrobur/airsim/Colosseum/Unreal/Plugins/AirSim/Source/AirLib/lib/x64/Release/libAirLib.a
```

Expected: Both files exist (second is a copy of the first).

- [ ] **Step 7: Update BlocksV2.uproject EngineAssociation to 5.6**

Read the current file:
```bash
cat /home/clrobur/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject
```

Change `"EngineAssociation": "5.4"` to `"EngineAssociation": "5.6"`:

```json
{
    "FileVersion": 3,
    "EngineAssociation": "5.6",
    "Modules": [
        {
            "Name": "BlocksV2",
            "Type": "Runtime",
            "LoadingPhase": "Default",
            "AdditionalDependencies": ["AirSim"]
        }
    ],
    "Plugins": [
        { "Name": "AirSim", "Enabled": true },
        { "Name": "ModelingToolsEditorMode", "Enabled": true, "TargetAllowList": ["Editor"] }
    ]
}
```

- [ ] **Step 8: Commit Colosseum build state**

```bash
cd /home/clrobur/airsim
git add -A
git commit -m "build: clone and build Colosseum with UE 5.6 EngineAssociation"
```

---

## Task 3: Create Settings and Launch Scripts

**Files:**
- Create: `settings/px4_dual.json`
- Create: `scripts/launch_ue_blocksv2.sh`
- Create: `scripts/launch_px4_dual.sh`

- [ ] **Step 1: Create settings directory and PX4 dual-drone settings**

Create `settings/px4_dual.json`:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockType": "SteppableClock",
  "Vehicles": {
    "Drone0": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "LockStep": true,
      "UseTcp": true,
      "TcpPort": 4560,
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14555,
      "LocalHostIp": "127.0.0.1",
      "X": 0, "Y": 0, "Z": 0,
      "Parameters": {
        "NAV_RCL_ACT": 0,
        "NAV_DLL_ACT": 0,
        "COM_OBL_ACT": 1,
        "MAV_SYS_ID": 1
      },
      "Cameras": {
        "front_center": {
          "CaptureSettings": [{
            "ImageType": 0,
            "Width": 1280,
            "Height": 720,
            "FOV_Degrees": 90,
            "MotionBlurAmount": 0
          }],
          "X": 0.25, "Y": 0, "Z": -0.18,
          "Pitch": 0, "Roll": 0, "Yaw": 0
        }
      }
    },
    "Drone1": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "LockStep": true,
      "UseTcp": true,
      "TcpPort": 4561,
      "ControlPortLocal": 14541,
      "ControlPortRemote": 14556,
      "LocalHostIp": "127.0.0.1",
      "X": 5, "Y": 0, "Z": 0,
      "Parameters": {
        "NAV_RCL_ACT": 0,
        "NAV_DLL_ACT": 0,
        "COM_OBL_ACT": 1,
        "MAV_SYS_ID": 2
      },
      "Cameras": {
        "front_center": {
          "CaptureSettings": [{
            "ImageType": 0,
            "Width": 1280,
            "Height": 720,
            "FOV_Degrees": 90,
            "MotionBlurAmount": 0
          }],
          "X": 0.25, "Y": 0, "Z": -0.18,
          "Pitch": 0, "Roll": 0, "Yaw": 0
        }
      }
    }
  }
}
```

Note: `LocalHostIp` is set to `127.0.0.1` for local testing. Change to the remote MAVROS machine IP when connecting to the embodied-drone computer.

- [ ] **Step 2: Create UE launch script**

Create `scripts/launch_ue_blocksv2.sh`:

```bash
#!/bin/bash
# Launch Unreal Engine 5.6 with BlocksV2 + AirSim plugin
set -e

UE_EDITOR="/home/clrobur/airsim/unreal-engine/Engine/Binaries/Linux/UnrealEditor"
BLOCKS_PROJECT="/home/clrobur/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject"
SETTINGS_SRC="/home/clrobur/airsim/settings/px4_dual.json"
SETTINGS_DST="$HOME/Documents/AirSim/settings.json"

# Copy settings.json to AirSim config directory
mkdir -p "$HOME/Documents/AirSim"
cp "$SETTINGS_SRC" "$SETTINGS_DST"
echo "Copied settings to $SETTINGS_DST"

# Launch UE editor
echo "Launching Unreal Engine with BlocksV2..."
"$UE_EDITOR" "$BLOCKS_PROJECT"
```

```bash
chmod +x /home/clrobur/airsim/scripts/launch_ue_blocksv2.sh
```

- [ ] **Step 3: Create PX4 dual SITL launch script**

Create `scripts/launch_px4_dual.sh`:

```bash
#!/bin/bash
# Launch 2 PX4 SITL instances for Colosseum
set -e

PX4_DIR="/home/clrobur/airsim/PX4-Autopilot"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"

if [ ! -f "$PX4_BIN" ]; then
    echo "ERROR: PX4 SITL not built. Run 'cd $PX4_DIR && make px4_sitl_default none_iris' first."
    exit 1
fi

echo "Starting PX4 SITL instance 0 (SysID=1, TCP:4560)..."
cd "$PX4_BUILD_DIR"
$PX4_BIN -i 0 -d "$PX4_BUILD_DIR/etc" >"/tmp/px4_sitl_0.log" 2>&1 &
PID0=$!
echo "  PID: $PID0, log: /tmp/px4_sitl_0.log"

sleep 2

echo "Starting PX4 SITL instance 1 (SysID=2, TCP:4561)..."
$PX4_BIN -i 1 -d "$PX4_BUILD_DIR/etc" >"/tmp/px4_sitl_1.log" 2>&1 &
PID1=$!
echo "  PID: $PID1, log: /tmp/px4_sitl_1.log"

echo ""
echo "Both PX4 SITL instances running."
echo "  Instance 0: PID=$PID0, TCP:4560, MAVLink UDP:14555"
echo "  Instance 1: PID=$PID1, TCP:4561, MAVLink UDP:14556"
echo ""
echo "Press Ctrl+C to stop both instances."

trap "kill $PID0 $PID1 2>/dev/null; echo 'Stopped.'; exit 0" SIGINT SIGTERM
wait
```

```bash
chmod +x /home/clrobur/airsim/scripts/launch_px4_dual.sh
```

- [ ] **Step 4: Commit settings and scripts**

```bash
cd /home/clrobur/airsim
git add settings/ scripts/
git commit -m "config: add PX4 dual-drone settings and launch scripts"
```

---

## Task 4: Build PX4-Autopilot SITL

**Files:**
- Create: `PX4-Autopilot/` (git clone)

- [ ] **Step 1: Clone PX4-Autopilot**

```bash
cd /home/clrobur/airsim
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Expected: Large repo (~2-3GB with submodules). Takes several minutes.

- [ ] **Step 2: Install PX4 build dependencies**

```bash
cd /home/clrobur/airsim/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

Expected: Installs arm-none-eabi-gcc, cmake, ninja-build, and other PX4 build deps.

- [ ] **Step 3: Build PX4 SITL for none_iris target**

```bash
cd /home/clrobur/airsim/PX4-Autopilot
make px4_sitl_default none_iris
```

Expected: Builds PX4 firmware for SITL mode without a default simulator. The `none_iris` target means PX4 starts but waits for an external simulator (Colosseum) to connect. Build takes 5-10 minutes.

- [ ] **Step 4: Verify PX4 SITL binary**

```bash
ls -la /home/clrobur/airsim/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

Expected: Binary exists and is executable.

- [ ] **Step 5: Quick-test PX4 SITL starts (and immediately stop it)**

```bash
cd /home/clrobur/airsim/PX4-Autopilot/build/px4_sitl_default
timeout 5 ./bin/px4 -i 0 -d etc 2>&1 | head -20 || true
```

Expected: PX4 starts and prints initialization messages. It will timeout after 5 seconds since no simulator is connected — this is expected.

---

## Task 5: Validate UE + Colosseum + PX4 SITL Integration

**Files:**
- No new files — integration test of Tasks 1-4

This task validates the full simulation pipeline. It requires a display (X11/Wayland) for Unreal Engine.

- [ ] **Step 1: Deploy settings.json**

```bash
mkdir -p ~/Documents/AirSim
cp /home/clrobur/airsim/settings/px4_dual.json ~/Documents/AirSim/settings.json
```

- [ ] **Step 2: Launch Unreal Engine with BlocksV2**

In a terminal:
```bash
/home/clrobur/airsim/unreal-engine/Engine/Binaries/Linux/UnrealEditor /home/clrobur/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject
```

Expected: UE editor opens. First launch may trigger shader compilation (takes several minutes). The editor should show the BlocksV2 level with the AirSim plugin loaded.

If UE prompts to convert the project for 5.6, accept the conversion.

- [ ] **Step 3: Launch PX4 SITL instances**

In separate terminals:

Terminal A (instance 0):
```bash
cd /home/clrobur/airsim/PX4-Autopilot/build/px4_sitl_default
./bin/px4 -i 0 -d etc
```

Terminal B (instance 1):
```bash
cd /home/clrobur/airsim/PX4-Autopilot/build/px4_sitl_default
./bin/px4 -i 1 -d etc
```

- [ ] **Step 4: Play the simulation in UE editor**

Click the "Play" button in the UE editor toolbar. Expected:
- Two drones spawn at positions (0,0,0) and (5,0,0)
- PX4 SITL terminals show "Connected to simulator" or similar HIL messages
- Drones are visible in the viewport

- [ ] **Step 5: Verify MAVLink output with a quick Python test**

```bash
source /home/clrobur/airsim/.venv/bin/activate
python3 -c "
from pymavlink import mavutil
conn = mavutil.mavlink_connection('udp:127.0.0.1:14555')
msg = conn.recv_match(blocking=True, timeout=10)
if msg:
    print(f'Received MAVLink from Drone0: {msg.get_type()}')
else:
    print('No MAVLink received on 14555 within 10s')
"
```

Expected: Receives MAVLink messages from Drone0.

- [ ] **Step 6: Verify airsim Python client connection**

```bash
source /home/clrobur/airsim/.venv/bin/activate
cd /home/clrobur/airsim/Colosseum/PythonClient
uv pip install -e .
python3 -c "
import airsim
client = airsim.MultirotorClient()
client.confirmConnection()
print('Connected to AirSim!')
state = client.getMultirotorState(vehicle_name='Drone0')
print(f'Drone0 position: {state.kinematics_estimated.position}')
state = client.getMultirotorState(vehicle_name='Drone1')
print(f'Drone1 position: {state.kinematics_estimated.position}')
"
```

Expected: Prints connection confirmation and positions for both drones.

- [ ] **Step 7: Test camera image capture**

```bash
source /home/clrobur/airsim/.venv/bin/activate
python3 -c "
import airsim
client = airsim.MultirotorClient()
client.confirmConnection()

responses = client.simGetImages([
    airsim.ImageRequest('front_center', airsim.ImageType.Scene, False, False)
], vehicle_name='Drone0')

r = responses[0]
print(f'Image from Drone0: {r.width}x{r.height}, {len(r.image_data_uint8)} bytes')

info = client.simGetCameraInfo('front_center', vehicle_name='Drone0')
print(f'Camera FOV: {info.fov}')
print(f'Camera pose: {info.pose}')
"
```

Expected: Image dimensions 1280x720, non-zero byte count, FOV=90.

---

## Task 6: Build Python ROS2 Bridge — Camera Publisher

**Files:**
- Create: `airsim_ros2_bridge/package.xml`
- Create: `airsim_ros2_bridge/setup.py`
- Create: `airsim_ros2_bridge/setup.cfg`
- Create: `airsim_ros2_bridge/resource/airsim_ros2_bridge`
- Create: `airsim_ros2_bridge/airsim_ros2_bridge/__init__.py`
- Create: `airsim_ros2_bridge/airsim_ros2_bridge/utils.py`
- Create: `airsim_ros2_bridge/airsim_ros2_bridge/camera_publisher.py`

- [ ] **Step 1: Create ROS2 package scaffold**

Create `airsim_ros2_bridge/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>airsim_ros2_bridge</name>
  <version>0.1.0</version>
  <description>Python bridge between AirSim/Colosseum and ROS2</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>cv_bridge</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Create `airsim_ros2_bridge/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/airsim_ros2_bridge
[install]
install_scripts=$base/lib/airsim_ros2_bridge
```

Create `airsim_ros2_bridge/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'airsim_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'bridge_node = airsim_ros2_bridge.bridge_node:main',
        ],
    },
)
```

Create empty files:

```bash
mkdir -p /home/clrobur/airsim/airsim_ros2_bridge/airsim_ros2_bridge
mkdir -p /home/clrobur/airsim/airsim_ros2_bridge/resource
touch /home/clrobur/airsim/airsim_ros2_bridge/resource/airsim_ros2_bridge
touch /home/clrobur/airsim/airsim_ros2_bridge/airsim_ros2_bridge/__init__.py
```

- [ ] **Step 2: Write utils.py — camera intrinsics and image conversion**

Create `airsim_ros2_bridge/airsim_ros2_bridge/utils.py`:

```python
import math
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


def fov_to_intrinsics(fov_degrees: float, width: int, height: int) -> tuple[float, float, float, float]:
    """Convert horizontal FOV + resolution to camera intrinsic parameters.

    Returns (fx, fy, cx, cy).
    AirSim uses a pinhole camera model with no distortion.
    """
    fov_rad = math.radians(fov_degrees)
    fx = width / (2.0 * math.tan(fov_rad / 2.0))
    fy = fx  # square pixels
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


def build_camera_info(
    fov_degrees: float,
    width: int,
    height: int,
    frame_id: str,
    stamp: Time,
) -> CameraInfo:
    """Build a sensor_msgs/CameraInfo from AirSim camera parameters."""
    fx, fy, cx, cy = fov_to_intrinsics(fov_degrees, width, height)

    msg = CameraInfo()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def airsim_rgb_to_image_msg(
    image_data: bytes,
    width: int,
    height: int,
    frame_id: str,
    stamp: Time,
) -> Image:
    """Convert raw AirSim RGB image bytes to sensor_msgs/Image.

    AirSim returns uncompressed RGBA when pixels_as_float=False, compress=False.
    We convert RGBA to RGB for the standard /camera/image topic.
    """
    msg = Image()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.encoding = 'rgb8'
    msg.is_bigendian = False
    msg.step = width * 3

    rgba = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width, 4)
    rgb = rgba[:, :, :3]
    msg.data = rgb.tobytes()
    return msg
```

- [ ] **Step 3: Write camera_publisher.py**

Create `airsim_ros2_bridge/airsim_ros2_bridge/camera_publisher.py`:

```python
import airsim
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from airsim_ros2_bridge.utils import build_camera_info, airsim_rgb_to_image_msg


class CameraPublisher:
    """Publishes camera image and info for a single drone."""

    def __init__(
        self,
        node: Node,
        client: airsim.MultirotorClient,
        vehicle_name: str,
        camera_name: str = 'front_center',
        publish_rate: float = 30.0,
    ):
        self._node = node
        self._client = client
        self._vehicle_name = vehicle_name
        self._camera_name = camera_name

        topic_prefix = f'/{vehicle_name}/camera'
        self._image_pub = node.create_publisher(
            airsim_rgb_to_image_msg.__annotations__.get('return', None) or __import__('sensor_msgs.msg', fromlist=['Image']).Image,
            f'{topic_prefix}/image',
            10,
        )
        from sensor_msgs.msg import CameraInfo
        self._info_pub = node.create_publisher(
            CameraInfo,
            f'{topic_prefix}/camera_info',
            10,
        )

        self._frame_id = f'{vehicle_name}_{camera_name}_optical'

        # Get camera info once (FOV and resolution from settings)
        camera_info = client.simGetCameraInfo(camera_name, vehicle_name=vehicle_name)
        self._fov = camera_info.fov

        # Get image dimensions from a test capture
        test_response = client.simGetImages([
            airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)
        ], vehicle_name=vehicle_name)
        self._width = test_response[0].width
        self._height = test_response[0].height

        node.get_logger().info(
            f'[{vehicle_name}] Camera: {self._width}x{self._height}, FOV={self._fov:.1f}'
        )

        self._timer = node.create_timer(1.0 / publish_rate, self._publish_callback)

    def _publish_callback(self):
        try:
            responses = self._client.simGetImages([
                airsim.ImageRequest(self._camera_name, airsim.ImageType.Scene, False, False)
            ], vehicle_name=self._vehicle_name)

            if not responses or responses[0].width == 0:
                return

            r = responses[0]
            stamp = self._node.get_clock().now().to_msg()

            image_msg = airsim_rgb_to_image_msg(
                r.image_data_uint8, r.width, r.height, self._frame_id, stamp
            )
            self._image_pub.publish(image_msg)

            info_msg = build_camera_info(
                self._fov, r.width, r.height, self._frame_id, stamp
            )
            self._info_pub.publish(info_msg)

        except Exception as e:
            self._node.get_logger().warn(f'[{self._vehicle_name}] Camera error: {e}')
```

- [ ] **Step 4: Verify package structure**

```bash
find /home/clrobur/airsim/airsim_ros2_bridge -type f | sort
```

Expected:
```
airsim_ros2_bridge/airsim_ros2_bridge/__init__.py
airsim_ros2_bridge/airsim_ros2_bridge/camera_publisher.py
airsim_ros2_bridge/airsim_ros2_bridge/utils.py
airsim_ros2_bridge/package.xml
airsim_ros2_bridge/resource/airsim_ros2_bridge
airsim_ros2_bridge/setup.cfg
airsim_ros2_bridge/setup.py
```

- [ ] **Step 5: Commit camera publisher**

```bash
cd /home/clrobur/airsim
git add airsim_ros2_bridge/
git commit -m "feat: add ROS2 bridge package with camera publisher and utils"
```

---

## Task 7: Build Python ROS2 Bridge — Drone Controller and Main Node

**Files:**
- Create: `airsim_ros2_bridge/airsim_ros2_bridge/drone_controller.py`
- Create: `airsim_ros2_bridge/airsim_ros2_bridge/bridge_node.py`

- [ ] **Step 1: Write drone_controller.py**

Create `airsim_ros2_bridge/airsim_ros2_bridge/drone_controller.py`:

```python
import airsim
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


class DroneController:
    """Subscribes to ROS2 control topics and forwards to AirSim API."""

    def __init__(
        self,
        node: Node,
        client: airsim.MultirotorClient,
        vehicle_name: str,
    ):
        self._node = node
        self._client = client
        self._vehicle_name = vehicle_name

        topic_prefix = f'/{vehicle_name}'

        self._vel_sub = node.create_subscription(
            Twist,
            f'{topic_prefix}/cmd_vel',
            self._cmd_vel_callback,
            10,
        )

        self._pos_sub = node.create_subscription(
            PoseStamped,
            f'{topic_prefix}/cmd_pos',
            self._cmd_pos_callback,
            10,
        )

        node.get_logger().info(f'[{vehicle_name}] Controller listening on {topic_prefix}/cmd_vel and {topic_prefix}/cmd_pos')

    def _cmd_vel_callback(self, msg: Twist):
        """Forward velocity command to AirSim.

        Twist.linear.x/y/z → vx, vy, vz in NED frame (m/s)
        Duration is fixed at 0.1s for continuous streaming.
        """
        try:
            self._client.moveByVelocityAsync(
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                duration=0.1,
                vehicle_name=self._vehicle_name,
            )
        except Exception as e:
            self._node.get_logger().warn(f'[{self._vehicle_name}] cmd_vel error: {e}')

    def _cmd_pos_callback(self, msg: PoseStamped):
        """Forward position command to AirSim.

        PoseStamped.pose.position.x/y/z → target NED position
        Velocity is fixed at 5 m/s.
        """
        try:
            self._client.moveToPositionAsync(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                velocity=5.0,
                vehicle_name=self._vehicle_name,
            )
        except Exception as e:
            self._node.get_logger().warn(f'[{self._vehicle_name}] cmd_pos error: {e}')
```

- [ ] **Step 2: Write bridge_node.py — main entry point**

Create `airsim_ros2_bridge/airsim_ros2_bridge/bridge_node.py`:

```python
import rclpy
from rclpy.node import Node
import airsim

from airsim_ros2_bridge.camera_publisher import CameraPublisher
from airsim_ros2_bridge.drone_controller import DroneController


class AirSimBridgeNode(Node):
    def __init__(self):
        super().__init__('airsim_bridge')

        # Parameters
        self.declare_parameter('vehicle_names', ['Drone0', 'Drone1'])
        self.declare_parameter('camera_name', 'front_center')
        self.declare_parameter('camera_fps', 30.0)
        self.declare_parameter('airsim_ip', '127.0.0.1')
        self.declare_parameter('airsim_port', 41451)

        vehicle_names = self.get_parameter('vehicle_names').get_parameter_value().string_array_value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        camera_fps = self.get_parameter('camera_fps').get_parameter_value().double_value
        airsim_ip = self.get_parameter('airsim_ip').get_parameter_value().string_value
        airsim_port = self.get_parameter('airsim_port').get_parameter_value().integer_value

        # Connect to AirSim
        self.get_logger().info(f'Connecting to AirSim at {airsim_ip}:{airsim_port}...')
        self._client = airsim.MultirotorClient(ip=airsim_ip, port=airsim_port)
        self._client.confirmConnection()
        self.get_logger().info('Connected to AirSim!')

        # Create camera publishers and controllers for each vehicle
        self._camera_publishers = []
        self._drone_controllers = []

        for vehicle_name in vehicle_names:
            self.get_logger().info(f'Setting up {vehicle_name}...')

            cam_pub = CameraPublisher(
                node=self,
                client=self._client,
                vehicle_name=vehicle_name,
                camera_name=camera_name,
                publish_rate=camera_fps,
            )
            self._camera_publishers.append(cam_pub)

            controller = DroneController(
                node=self,
                client=self._client,
                vehicle_name=vehicle_name,
            )
            self._drone_controllers.append(controller)

        self.get_logger().info(f'Bridge running for {len(vehicle_names)} vehicles')


def main(args=None):
    rclpy.init(args=args)
    node = AirSimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Verify complete package structure**

```bash
find /home/clrobur/airsim/airsim_ros2_bridge -type f | sort
```

Expected:
```
airsim_ros2_bridge/airsim_ros2_bridge/__init__.py
airsim_ros2_bridge/airsim_ros2_bridge/bridge_node.py
airsim_ros2_bridge/airsim_ros2_bridge/camera_publisher.py
airsim_ros2_bridge/airsim_ros2_bridge/drone_controller.py
airsim_ros2_bridge/airsim_ros2_bridge/utils.py
airsim_ros2_bridge/package.xml
airsim_ros2_bridge/resource/airsim_ros2_bridge
airsim_ros2_bridge/setup.cfg
airsim_ros2_bridge/setup.py
```

- [ ] **Step 4: Build ROS2 package**

```bash
source /opt/ros/humble/setup.bash
cd /home/clrobur/airsim
colcon build --packages-select airsim_ros2_bridge --symlink-install
```

Expected: Build succeeds with no errors.

- [ ] **Step 5: Test bridge node startup (with simulation running)**

With UE + PX4 SITL running from Task 5:

```bash
source /opt/ros/humble/setup.bash
source /home/clrobur/airsim/install/setup.bash
source /home/clrobur/airsim/.venv/bin/activate
ros2 run airsim_ros2_bridge bridge_node --ros-args -p vehicle_names:="['Drone0', 'Drone1']"
```

Expected: Node starts, logs connection to AirSim and camera info for each drone.

- [ ] **Step 6: Verify published topics**

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/clrobur/airsim/install/setup.bash
ros2 topic list | grep -E "Drone|camera"
```

Expected output:
```
/Drone0/camera/camera_info
/Drone0/camera/image
/Drone0/cmd_pos
/Drone0/cmd_vel
/Drone1/camera/camera_info
/Drone1/camera/image
/Drone1/cmd_pos
/Drone1/cmd_vel
```

- [ ] **Step 7: Verify image data on topics**

```bash
ros2 topic hz /Drone0/camera/image
```

Expected: ~30 Hz message rate.

```bash
ros2 topic echo /Drone0/camera/camera_info --once
```

Expected: CameraInfo with width=1280, height=720, non-zero K matrix values.

- [ ] **Step 8: Commit bridge node and controller**

```bash
cd /home/clrobur/airsim
git add airsim_ros2_bridge/
git commit -m "feat: add drone controller and main bridge node"
```

---

## Task 8: Configure ArduPilot (APM) SITL Settings

**Files:**
- Create: `settings/apm_dual.json`
- Create: `scripts/launch_apm_dual.sh`

- [ ] **Step 1: Create APM dual-drone settings**

Create `settings/apm_dual.json`:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockType": "",
  "Vehicles": {
    "Drone0": {
      "VehicleType": "ArduCopter",
      "UseSerial": false,
      "UdpIp": "127.0.0.1",
      "UdpPort": 9003,
      "ControlPort": 9002,
      "LocalHostIp": "127.0.0.1",
      "X": 0, "Y": 0, "Z": 0,
      "Cameras": {
        "front_center": {
          "CaptureSettings": [{
            "ImageType": 0,
            "Width": 1280,
            "Height": 720,
            "FOV_Degrees": 90,
            "MotionBlurAmount": 0
          }],
          "X": 0.25, "Y": 0, "Z": -0.18,
          "Pitch": 0, "Roll": 0, "Yaw": 0
        }
      }
    },
    "Drone1": {
      "VehicleType": "ArduCopter",
      "UseSerial": false,
      "UdpIp": "127.0.0.1",
      "UdpPort": 9005,
      "ControlPort": 9004,
      "LocalHostIp": "127.0.0.1",
      "X": 5, "Y": 0, "Z": 0,
      "Cameras": {
        "front_center": {
          "CaptureSettings": [{
            "ImageType": 0,
            "Width": 1280,
            "Height": 720,
            "FOV_Degrees": 90,
            "MotionBlurAmount": 0
          }],
          "X": 0.25, "Y": 0, "Z": -0.18,
          "Pitch": 0, "Roll": 0, "Yaw": 0
        }
      }
    }
  }
}
```

Note: `ClockType` is empty string for ArduPilot (not SteppableClock). Change `LocalHostIp` to remote MAVROS IP when connecting externally.

- [ ] **Step 2: Create APM SITL launch script**

Create `scripts/launch_apm_dual.sh`:

```bash
#!/bin/bash
# Launch 2 ArduPilot SITL instances for Colosseum
# Requires ArduPilot to be installed and sim_vehicle.py available
set -e

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"

if [ ! -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]; then
    echo "ERROR: ArduPilot not found at $ARDUPILOT_DIR"
    echo "Set ARDUPILOT_DIR environment variable or clone ArduPilot to ~/ardupilot"
    exit 1
fi

# Copy APM settings
mkdir -p ~/Documents/AirSim
cp /home/clrobur/airsim/settings/apm_dual.json ~/Documents/AirSim/settings.json
echo "Copied APM settings to ~/Documents/AirSim/settings.json"

echo "Starting ArduPilot SITL instance 0 (UDP:9002/9003)..."
cd "$ARDUPILOT_DIR"
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -f airsim-copter --instance 0 &>/tmp/apm_sitl_0.log &
PID0=$!
echo "  PID: $PID0, log: /tmp/apm_sitl_0.log"

sleep 3

echo "Starting ArduPilot SITL instance 1 (UDP:9004/9005)..."
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -f airsim-copter --instance 1 &>/tmp/apm_sitl_1.log &
PID1=$!
echo "  PID: $PID1, log: /tmp/apm_sitl_1.log"

echo ""
echo "Both ArduPilot SITL instances running."
echo "  Instance 0: PID=$PID0, Sensor:9003, Control:9002"
echo "  Instance 1: PID=$PID1, Sensor:9005, Control:9004"
echo ""
echo "Press Ctrl+C to stop both instances."

trap "kill $PID0 $PID1 2>/dev/null; echo 'Stopped.'; exit 0" SIGINT SIGTERM
wait
```

```bash
chmod +x /home/clrobur/airsim/scripts/launch_apm_dual.sh
```

- [ ] **Step 3: Update launch_ue_blocksv2.sh to accept settings argument**

Replace `scripts/launch_ue_blocksv2.sh`:

```bash
#!/bin/bash
# Launch Unreal Engine 5.6 with BlocksV2 + AirSim plugin
# Usage: ./launch_ue_blocksv2.sh [px4|apm]  (default: px4)
set -e

MODE="${1:-px4}"
UE_EDITOR="/home/clrobur/airsim/unreal-engine/Engine/Binaries/Linux/UnrealEditor"
BLOCKS_PROJECT="/home/clrobur/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject"
SETTINGS_DST="$HOME/Documents/AirSim/settings.json"

case "$MODE" in
    px4)
        SETTINGS_SRC="/home/clrobur/airsim/settings/px4_dual.json"
        ;;
    apm)
        SETTINGS_SRC="/home/clrobur/airsim/settings/apm_dual.json"
        ;;
    *)
        echo "Usage: $0 [px4|apm]"
        exit 1
        ;;
esac

mkdir -p "$HOME/Documents/AirSim"
cp "$SETTINGS_SRC" "$SETTINGS_DST"
echo "Copied $MODE settings to $SETTINGS_DST"

echo "Launching Unreal Engine with BlocksV2..."
"$UE_EDITOR" "$BLOCKS_PROJECT"
```

- [ ] **Step 4: Commit APM configuration**

```bash
cd /home/clrobur/airsim
git add settings/apm_dual.json scripts/
git commit -m "config: add ArduPilot dual-drone settings and launch script"
```

---

## Task 9: Validate ArduPilot SITL Integration

**Files:**
- No new files — integration test of APM configuration

This task requires ArduPilot to be installed. If not already installed, clone and build first:

```bash
git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
git submodule update --init --recursive
./Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

- [ ] **Step 1: Deploy APM settings and launch UE**

```bash
/home/clrobur/airsim/scripts/launch_ue_blocksv2.sh apm
```

- [ ] **Step 2: Launch ArduPilot SITL instances**

```bash
/home/clrobur/airsim/scripts/launch_apm_dual.sh
```

Expected: Both instances start and show initialization output in logs.

- [ ] **Step 3: Play simulation and verify drones spawn**

Click Play in UE editor. Expected: Two drones spawn, ArduPilot SITL logs show connection to simulator.

- [ ] **Step 4: Verify with airsim Python client**

```bash
source /home/clrobur/airsim/.venv/bin/activate
python3 -c "
import airsim
client = airsim.MultirotorClient()
client.confirmConnection()
print('Connected!')
for name in ['Drone0', 'Drone1']:
    state = client.getMultirotorState(vehicle_name=name)
    print(f'{name} pos: {state.kinematics_estimated.position}')
"
```

Expected: Prints positions for both APM-controlled drones.

- [ ] **Step 5: Verify ROS2 bridge works with APM**

The bridge node is SITL-agnostic (connects via airsim RPC, not MAVLink). Verify:

```bash
source /opt/ros/humble/setup.bash
source /home/clrobur/airsim/install/setup.bash
source /home/clrobur/airsim/.venv/bin/activate
ros2 run airsim_ros2_bridge bridge_node --ros-args -p vehicle_names:="['Drone0', 'Drone1']"
```

In another terminal:
```bash
ros2 topic hz /Drone0/camera/image
```

Expected: ~30 Hz, same as with PX4.

---

## Execution Order Summary

```
Task 1: System deps (clang-18, Python 3.12, Vulkan)
  ↓
Task 2: Clone + build Colosseum
  ↓
Task 3: Settings + launch scripts (can parallel with Task 4)
  ↓
Task 4: Clone + build PX4-Autopilot
  ↓
Task 5: Integration test (UE + Colosseum + PX4 SITL)
  ↓
Task 6: ROS2 bridge — camera publisher
  ↓
Task 7: ROS2 bridge — controller + main node
  ↓
Task 8: APM settings + launch scripts
  ↓
Task 9: APM integration test
```

Tasks 3 and 4 can run in parallel. Tasks 6 and 8 can run in parallel (bridge code is independent of APM config). All other tasks are sequential.
