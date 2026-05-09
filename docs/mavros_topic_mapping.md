# AirSim ROS2 Bridge Topic Contract

This document describes the AirSim ROS2 bridge contract for the AERION workspace.
The current project goal is to expand from the verified minimum `/ap/cmd_vel` control loop into a broader MAVROS/ArduPilot-compatible topic surface.

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

Use the helper script when running from the verified WSL workspace:

```bash
~/aerion_ros2_ws/src/airsim_ros2_bridge/scripts/run_airsim_ros2_bridge.sh
```

The equivalent explicit command is:

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
| `home_latitude` | `37.5665` | Approximate local-origin latitude used to synthesize GPS topics from AirSim local ENU pose. |
| `home_longitude` | `126.9780` | Approximate local-origin longitude used to synthesize GPS topics from AirSim local ENU pose. |
| `home_altitude` | `0.0` | Approximate local-origin altitude used to synthesize GPS topics from AirSim local ENU pose. |

## Drone-scoped topics

These topics are available per vehicle.

```text
/Drone0/cmd_pos
/Drone0/cmd_vel
/Drone0/mavros/local_position/pose
/Drone0/mavros/local_position/pose_cov
/Drone0/mavros/local_position/odom
/Drone0/mavros/local_position/velocity_local
/Drone0/mavros/local_position/velocity_local_cov
/Drone0/mavros/local_position/velocity_body
/Drone0/mavros/local_position/velocity_body_cov
/Drone0/mavros/global_position/global
/Drone0/mavros/global_position/local
/Drone0/mavros/global_position/raw/fix
/Drone0/mavros/global_position/rel_alt
/Drone0/mavros/global_position/compass_hdg
/Drone0/mavros/imu/data
/Drone0/mavros/imu/data_raw
/Drone0/mavros/battery
/Drone0/mavros/state
/Drone0/mavros/setpoint_position/local
/Drone0/mavros/setpoint_velocity/cmd_vel
/Drone0/mavros/setpoint_velocity/cmd_vel_unstamped

/Drone1/cmd_pos
/Drone1/cmd_vel
/Drone1/mavros/local_position/pose
/Drone1/mavros/local_position/pose_cov
/Drone1/mavros/local_position/odom
/Drone1/mavros/local_position/velocity_local
/Drone1/mavros/local_position/velocity_local_cov
/Drone1/mavros/local_position/velocity_body
/Drone1/mavros/local_position/velocity_body_cov
/Drone1/mavros/global_position/global
/Drone1/mavros/global_position/local
/Drone1/mavros/global_position/raw/fix
/Drone1/mavros/global_position/rel_alt
/Drone1/mavros/global_position/compass_hdg
/Drone1/mavros/imu/data
/Drone1/mavros/imu/data_raw
/Drone1/mavros/battery
/Drone1/mavros/state
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
| `/DroneN/mavros/local_position/pose_cov` | `geometry_msgs/msg/PoseWithCovarianceStamped` | ROS ENU | Same pose with approximate covariance. |
| `/DroneN/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | ROS ENU | Pose plus AirSim linear velocity converted to ENU. |
| `/DroneN/mavros/local_position/velocity_local` | `geometry_msgs/msg/TwistStamped` | ROS ENU | AirSim linear velocity converted to ENU. |
| `/DroneN/mavros/local_position/velocity_local_cov` | `geometry_msgs/msg/TwistWithCovarianceStamped` | ROS ENU | Local velocity with approximate covariance. |
| `/DroneN/mavros/local_position/velocity_body` | `geometry_msgs/msg/TwistStamped` | `base_link` | Currently mirrors ENU linear velocity with body frame id. |
| `/DroneN/mavros/local_position/velocity_body_cov` | `geometry_msgs/msg/TwistWithCovarianceStamped` | `base_link` | Body velocity placeholder with approximate covariance. |
| `/DroneN/mavros/global_position/global` | `sensor_msgs/msg/NavSatFix` | GPS | Synthesized from local ENU pose and `home_*` parameters. |
| `/DroneN/mavros/global_position/local` | `nav_msgs/msg/Odometry` | ROS ENU | Alias of local odometry. |
| `/DroneN/mavros/global_position/raw/fix` | `sensor_msgs/msg/NavSatFix` | GPS | Alias of synthesized GPS fix. |
| `/DroneN/mavros/global_position/rel_alt` | `std_msgs/msg/Float64` | meters | Local ENU z as relative altitude. |
| `/DroneN/mavros/global_position/compass_hdg` | `std_msgs/msg/Float64` | degrees | Yaw extracted from pose quaternion. |
| `/DroneN/mavros/imu/data` | `sensor_msgs/msg/Imu` | `base_link` | Orientation from AirSim pose; angular velocity and acceleration are unknown. |
| `/DroneN/mavros/imu/data_raw` | `sensor_msgs/msg/Imu` | `base_link` | Alias of synthetic IMU data. |
| `/DroneN/mavros/battery` | `sensor_msgs/msg/BatteryState` | n/a | Synthetic healthy full battery. |
| `/DroneN/mavros/state` | `mavros_msgs/msg/State` | n/a | Published only when `mavros_msgs` is installed in the ROS environment. |

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
| `/ap/navsat` | `sensor_msgs/msg/NavSatFix` | Publish | Synthesized GPS fix. |
| `/ap/imu/experimental/data` | `sensor_msgs/msg/Imu` | Publish | Synthetic IMU data from AirSim pose. |
| `/ap/battery` | `sensor_msgs/msg/BatteryState` | Publish | Synthetic healthy full battery. |
| `/ap/clock` | `rosgraph_msgs/msg/Clock` | Publish | ROS clock derived from node time. |
| `/imu` | `sensor_msgs/msg/Imu` | Publish | Gazebo-style root alias for selected vehicle. |
| `/navsat` | `sensor_msgs/msg/NavSatFix` | Publish | Gazebo-style root alias for selected vehicle. |
| `/odometry` | `nav_msgs/msg/Odometry` | Publish | Gazebo-style root alias for selected vehicle. |
| `/battery` | `sensor_msgs/msg/BatteryState` | Publish | Gazebo-style root alias for selected vehicle. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Publish | Gazebo-style root alias for selected vehicle. |
| `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | Publish | Top-level MAVROS pose alias for the selected vehicle. |
| `/mavros/local_position/pose_cov` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Publish | Top-level MAVROS pose covariance alias. |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | Publish | Top-level MAVROS odometry alias. |
| `/mavros/local_position/velocity_local` | `geometry_msgs/msg/TwistStamped` | Publish | Top-level local velocity alias. |
| `/mavros/local_position/velocity_local_cov` | `geometry_msgs/msg/TwistWithCovarianceStamped` | Publish | Top-level local velocity covariance alias. |
| `/mavros/local_position/velocity_body` | `geometry_msgs/msg/TwistStamped` | Publish | Top-level body velocity alias. |
| `/mavros/local_position/velocity_body_cov` | `geometry_msgs/msg/TwistWithCovarianceStamped` | Publish | Top-level body velocity covariance alias. |
| `/mavros/global_position/global` | `sensor_msgs/msg/NavSatFix` | Publish | Top-level synthesized GPS alias. |
| `/mavros/global_position/local` | `nav_msgs/msg/Odometry` | Publish | Top-level global/local odometry alias. |
| `/mavros/global_position/raw/fix` | `sensor_msgs/msg/NavSatFix` | Publish | Top-level raw GPS fix alias. |
| `/mavros/global_position/rel_alt` | `std_msgs/msg/Float64` | Publish | Top-level relative altitude alias. |
| `/mavros/global_position/compass_hdg` | `std_msgs/msg/Float64` | Publish | Top-level heading alias. |
| `/mavros/imu/data` | `sensor_msgs/msg/Imu` | Publish | Top-level IMU alias. |
| `/mavros/imu/data_raw` | `sensor_msgs/msg/Imu` | Publish | Top-level raw IMU alias. |
| `/mavros/battery` | `sensor_msgs/msg/BatteryState` | Publish | Top-level battery alias. |
| `/mavros/state` | `mavros_msgs/msg/State` | Publish | Published only when `mavros_msgs` is installed in the ROS environment. |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Subscribe | Top-level MAVROS stamped velocity alias for the selected vehicle. |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | Subscribe | Top-level MAVROS velocity alias for the selected vehicle. |
| `/mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` | Subscribe | Top-level MAVROS local position setpoint alias for the selected vehicle. |

## Compatibility gap list

The bridge is not yet a full MAVROS replacement. Remaining categories include:

- MAVROS services such as command, mode, arming, waypoint push/pull, parameter, and home-position services.
- MAVROS message families that require `mavros_msgs` beyond `State`, such as `ExtendedState`, `VfrHud`, `WaypointList`, RC messages, mission status, status text, and sys status.
- Accurate body-frame velocity rotation. The current body velocity topics are frame-id-compatible placeholders.
- Real IMU angular velocity, linear acceleration, magnetic field, airspeed, pressure, and RC telemetry.
- ArduPilot DDS-specific `/ap/status`, `/ap/airspeed`, `/ap/geopose/filtered`, `/ap/gps_global_origin/filtered`, `/ap/time`, `/ap/tf`, and `/ap/tf_static` type parity.
- Camera/perception topics while `enable_camera:=false`.

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

Use the helper script for the full RPC, topic, and motion smoke test:

```bash
~/aerion_ros2_ws/src/airsim_ros2_bridge/scripts/smoke_airsim_ros2_bridge.sh
```

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

## Control-loop probe

The package includes a minimal compatibility probe that consumes `/ap/pose/filtered` and publishes `/ap/cmd_vel`.
It is intended to validate the same contract that the Aerion control or mission node should use:

```bash
cd ~/aerion_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run airsim_ros2_bridge ap_pose_cmdvel_probe --ros-args \
  -p target_dx:=0.2 \
  -p target_dy:=0.0 \
  -p tolerance:=0.12 \
  -p max_duration_sec:=90.0 \
  -p max_speed:=0.25
```

The probe commands only after a fresh pose message arrives. This avoids repeatedly commanding from stale state when AirSim RPC or ROS discovery is slow.

## Deferred work

Do not block control-loop integration on these items:

- Replacing the probe with the production Aerion control or mission node.
- Camera publisher recovery.
- Full MAVROS topic parity.
- Full ArduPilot message parity for `/ap/status`.
- PX4 SITL settings validation.
- Zenoh or cross-machine DDS tuning.
