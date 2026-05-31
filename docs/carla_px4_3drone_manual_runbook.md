# CARLA + AirSim + PX4/MAVROS 3-Drone Manual Validation

This runbook freezes the first validated CARLA + AirSim + PX4/MAVROS manual
control setup for 3 drones.

## Goal

Validate a 3-drone CARLA/AirSim environment for manual control latency checks.

- `drone1` is the master drone.
- `drone2` and `drone3` are slave drones.
- Only `drone1` has a camera enabled.
- Every drone has exactly 3 range sensors: front, left, right.
- PX4 SITL and MAVROS are used for the standard control/topic path.
- The validation target is simultaneous manual control of all 3 drones in the
  current CARLA map.

## Topology

| Role | AirSim vehicle | ROS namespace | MAVROS namespace |
|---|---|---|---|
| Master | `drone1` | `/drone1` | `/drone1/mavros` |
| Slave 1 | `drone2` | `/drone2` | `/drone2/mavros` |
| Slave 2 | `drone3` | `/drone3` | `/drone3/mavros` |

Camera topics should exist only under `/drone1/camera/*`.

Range sensor topics:

- `/droneN/range/front`
- `/droneN/range/left`
- `/droneN/range/right`

## Run Order

Use a separate terminal for each long-running process.

### 1. Start CARLA + AirSim UE

```bash
cd ~/workspace/engines/CarlaUE5/Unreal/CarlaUnreal

~/workspace/engines/UE5.5_carla/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
  ~/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject \
  -log
```

Open the target CARLA map in UE, then press `Play`.

Do not execute the `.uproject` file directly. It must be passed to
`UnrealEditor`.

### 2. Start PX4 SITL

```bash
cd ~/workspace/projects/aerion-airsim

PX4_DIR=~/airsim/PX4-Autopilot \
DRONE_COUNT=3 \
bash scripts/launch_px4_instances.sh
```

### 3. Start MAVROS

```bash
cd ~/workspace/projects/aerion-airsim
source /opt/ros/humble/setup.bash
source install/setup.bash

DRONE_COUNT=3 \
bash scripts/launch_mavros_px4_instances.sh
```

### 4. Start the AirSim ROS2 bridge

```bash
cd ~/workspace/projects/aerion-airsim
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_WS=~/workspace/projects/aerion-airsim \
AIRSIM_IP=127.0.0.1 \
AIRSIM_PORT=41451 \
DRONE_COUNT=3 \
MASTER_VEHICLE=drone1 \
ENABLE_RANGE=true \
VELOCITY_CONTROL_MODE=kinematic \
FORWARD_AP_CMD_VEL_TO_MAVROS=false \
LOCAL_MOTION_FROM_MAVROS=false \
bash scripts/run_airsim_ros2_bridge_instances.sh
```

### 5. Prepare PX4 for manual control

PX4 may connect but refuse arming, or arm and immediately auto-disarm, unless
manual-validation parameters and neutral setpoints are applied first.

```bash
cd ~/workspace/projects/aerion-airsim
source /opt/ros/humble/setup.bash
source install/setup.bash

DRONE_COUNT=3 \
bash scripts/prepare_px4_manual_control.sh
```

Expected state for all 3 drones:

```text
connected: true
armed: true
mode: OFFBOARD
system_status: 4
```

### 6. Start 3-drone manual control

Recommended validation speed:

```bash
cd ~/workspace/projects/aerion-airsim
source /opt/ros/humble/setup.bash
source install/setup.bash

VEHICLES=drone1,drone2,drone3 \
ARM_FLAG=false \
FORCE_ARM_FLAG=false \
SPEED=3.0 \
VERTICAL_SPEED=1.5 \
YAW_RATE=1.2 \
HOLD_TIMEOUT=0.35 \
bash scripts/run_manual_3drone_control.sh
```

More aggressive latency check:

```bash
VEHICLES=drone1,drone2,drone3 \
ARM_FLAG=false \
FORCE_ARM_FLAG=false \
SPEED=5.0 \
VERTICAL_SPEED=2.0 \
YAW_RATE=1.8 \
HOLD_TIMEOUT=0.35 \
bash scripts/run_manual_3drone_control.sh
```

Keys:

```text
w/s: forward/back
a/d: left/right
r/f: up/down
q/e: yaw
space: stop
x: exit
```

## Verification

MAVROS state:

```bash
ros2 topic echo --once /drone1/mavros/state
ros2 topic echo --once /drone2/mavros/state
ros2 topic echo --once /drone3/mavros/state
```

Actual movement should be checked with MAVROS local pose:

```bash
ros2 topic echo --once /drone1/mavros/local_position/pose
ros2 topic echo --once /drone2/mavros/local_position/pose
ros2 topic echo --once /drone3/mavros/local_position/pose
```

Range topics:

```bash
ros2 topic echo --once /drone1/range/front
ros2 topic echo --once /drone1/range/left
ros2 topic echo --once /drone1/range/right
```

Topic overview:

```bash
ros2 topic list | grep -E '^/drone[123]/(camera|range|mavros)'
```

## Key Issues Found

1. Wrong UE project

   The CARLA/AirSim integration project is:

   ```text
   ~/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject
   ```

   Colosseum `CityEnv` is not the target project for this workflow.

2. `.uproject` is not executable

   Running the `.uproject` directly returns permission or command errors. Use
   `UnrealEditor <project>.uproject -log`.

3. AirSim module mismatch

   UE reported missing `CarlaUnreal` modules for `AirSim`. Rebuilding
   `CarlaUnrealEditor` against the active UE5.5_carla engine fixed the module
   mismatch.

4. MAVROS connected but control did not move drones

   `connected: true` alone is not enough. PX4 must be armed and in `OFFBOARD`.
   The failure state was:

   ```text
   connected: true
   armed: false
   mode: OFFBOARD
   ```

5. PX4 arming and auto-disarm

   PX4 reported health/preflight failures such as:

   ```text
   Preflight Fail: heading estimate invalid
   Arming denied: Resolve system health failures first
   Disarmed by auto preflight disarming
   ```

   `scripts/prepare_px4_manual_control.sh` applies the manual-validation PX4
   parameters, streams neutral setpoints, switches to `OFFBOARD`, and arms all
   3 drones.

6. AirSim RPC pose can be misleading for PX4Multirotor

   `airsim.getMultirotorState()` can report a zero pose for these PX4 vehicles.
   Use `/droneN/mavros/local_position/pose` to verify actual movement.

## First Validation Result

The first validation target is complete:

- CARLA + AirSim UE starts with the correct project.
- 3 PX4 SITL instances connect to 3 AirSim vehicles.
- 3 MAVROS namespaces publish standard topics.
- The bridge exposes master-only camera and 3 range sensors per drone.
- All 3 drones can be armed in `OFFBOARD`.
- All 3 drones can be controlled together with the manual keyboard controller.
