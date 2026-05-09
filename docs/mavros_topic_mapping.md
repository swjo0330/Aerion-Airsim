# AirSim ROS2 Bridge Topic Contract

This document describes the currently verified AirSim ROS2 bridge contract for the AERION workspace.

## Runtime model

The canonical bridge source lives in:

```text
Aerion-Airsim/airsim_ros2_bridge
```

The WSL runtime copy lives in:

```text
~/aerion_ros2_ws/src/airsim_ros2_bridge
```

The current verified setup is:

- Windows runs the Unreal `simulation/aerial` AirSim project.
- WSL runs ROS2 Humble and the bridge package.
- AirSim RPC is reached from WSL at `172.23.80.1:41451`.
- Camera publishing is disabled by default.
- `Drone0` exposes ArduPilot-compatible `/ap/*` aliases.
- `Drone0` and `Drone1` keep drone-scoped topics under `/DroneN/...`.

## Verified bridge launch

```bash
cd ~/aerion_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run airsim_ros2_bridge bridge_node --ros-args \
  -p airsim_ip:=172.23.80.1 \
  -p airsim_port:=41451 \
  -p enable_camera:=false \
  -p enable_ardu_compat:=true \
  -p ardu_compat_vehicle:=Drone0 \
  -p velocity_control_mode:=kinematic \
  -p velocity_command_duration:=0.2 \
  -p kinematic_z_ned:=-1.0 \
  -p airsim_timeout_sec:=2.0
```

## Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `vehicle_names` | `['Drone0', 'Drone1']` | AirSim vehicle names to bridge. |
| `airsim_ip` | `127.0.0.1` | AirSim RPC host. Use `172.23.80.1` from WSL in the current Windows setup. |
| `airsim_port` | `41451` | AirSim RPC port. |
| `airsim_timeout_sec` | `2.0` | RPC timeout used by the Python AirSim client. |
| `enable_camera` | `false` | Enables camera publishers only when explicitly requested. |
| `camera_name` | `front_center` | AirSim camera name for camera mode. |
| `camera_fps` | `30.0` | Camera publish rate when enabled. |
| `enable_ardu_compat` | `true` | Enables top-level `/ap/*` and `/mavros/*` aliases for one vehicle. |
| `ardu_compat_vehicle` | `Drone0` | Vehicle exposed through top-level ArduPilot-compatible aliases. |
| `velocity_control_mode` | `kinematic` | `kinematic` uses raw `simSetVehiclePose`; any other value uses AirSim `moveByVelocityAsync`. |
| `velocity_command_duration` | `0.2` | Duration or integration step for velocity commands. |
| `kinematic_z_ned` | `-1.0` | Locked NED altitude for horizontal kinematic velocity commands. |

## Drone-scoped topics

These topics are available per vehicle.

```text
/Drone0/cmd_pos
/Drone0/cmd_vel
/Drone0/mavros/local_position/pose
/Drone0/mavros/setpoint_position/local
/Drone0/mavros/setpoint_velocity/cmd_vel
/Drone0/mavros/setpoint_velocity/cmd_vel_unstamped

/Drone1/cmd_pos
/Drone1/cmd_vel
/Drone1/mavros/local_position/pose
/Drone1/mavros/setpoint_position/local
/Drone1/mavros/setpoint_velocity/cmd_vel
/Drone1/mavros/setpoint_velocity/cmd_vel_unstamped
```

### Control input

| Topic | Type | Frame | Handling |
| --- | --- | --- | --- |
| `/DroneN/cmd_vel` | `geometry_msgs/msg/Twist` | AirSim NED | Direct NED velocity command. |
| `/DroneN/cmd_pos` | `geometry_msgs/msg/PoseStamped` | AirSim NED | Direct NED position command. |
| `/DroneN/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/msg/TwistStamped` | ROS ENU | Converted to AirSim NED. |
| `/DroneN/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | ROS ENU | Converted to AirSim NED. |
| `/DroneN/mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` | ROS ENU | Converted to AirSim NED. |

### State output

| Topic | Type | Frame | Handling |
| --- | --- | --- | --- |
| `/DroneN/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | ROS ENU | AirSim NED pose converted to ENU. |

## ArduPilot-compatible aliases

When `enable_ardu_compat:=true`, aliases are exposed for the vehicle selected by `ardu_compat_vehicle`.

Current verified alias vehicle:

```text
Drone0
```

| Topic | Type | Direction | Notes |
| --- | --- | --- | --- |
| `/ap/cmd_vel` | `geometry_msgs/msg/Twist` | Subscribe | Primary compatibility velocity command. ROS ENU. |
| `/ap/pose/filtered` | `geometry_msgs/msg/PoseStamped` | Publish | Alias of selected vehicle pose in ROS ENU. |
| `/ap/twist/filtered` | `geometry_msgs/msg/TwistStamped` | Publish | Publishes latest command velocity in ROS ENU. |
| `/ap/status` | `std_msgs/msg/String` | Publish | Minimal heartbeat until the exact ArduPilot status type is added. |
| `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | Publish | Top-level MAVROS pose alias for the selected vehicle. |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | Subscribe | Top-level MAVROS velocity alias for the selected vehicle. |

## Why kinematic velocity mode exists

In the current Windows AirSim/Unreal setup, native AirSim velocity APIs connect successfully but did not reliably change horizontal position:

- `moveByVelocityAsync`
- `moveByVelocityZAsync`
- `moveToPositionAsync`

The raw RPC form of `simSetVehiclePose` does move the vehicle when encoded as AirSim's list-style msgpack payload. Therefore the verified control path uses:

```text
velocity_control_mode:=kinematic
```

This mode integrates incoming velocity commands over `velocity_command_duration` and applies the next pose with raw `simSetVehiclePose`. Horizontal commands keep altitude locked at `kinematic_z_ned` unless a non-zero z velocity is commanded.

## Smoke tests

Check AirSim RPC:

```bash
python3 - <<'PY'
import airsim
c = airsim.MultirotorClient(ip="172.23.80.1", port=41451, timeout_value=2)
c.confirmConnection()
print(c.client.call("getMultirotorState", "Drone0")[1][0])
PY
```

Check state topics:

```bash
ros2 topic echo --once /ap/status
ros2 topic echo --once /ap/pose/filtered
ros2 topic echo --once /ap/twist/filtered
```

Command `Drone0` through the compatibility topic:

```bash
ros2 topic pub --times 5 -r 5 /ap/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Expected result:

- `/ap/pose/filtered` changes position.
- `/ap/twist/filtered` reports the latest command velocity.
- `/ap/status` continues to publish.

## Deferred work

Do not block control-loop integration on these items:

- Camera publisher recovery.
- Full MAVROS topic parity.
- Full ArduPilot message parity for `/ap/status`.
- PX4 SITL settings validation.
- Zenoh or cross-machine DDS tuning.

The next practical step is to attach the existing Aerion control or mission node that publishes `/ap/cmd_vel` and consumes `/ap/pose/filtered`.
