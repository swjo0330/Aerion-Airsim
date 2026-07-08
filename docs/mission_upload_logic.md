# Mission Upload Logic

This document defines the upload boundary for AERION mission files.

## Short Version

The mission editor now exports GPS mission coordinates by default. MAVROS/PX4
receives those GPS mission items directly. Legacy local mission JSON remains
supported and is converted at upload time only when needed.

```text
mission JSON gps
  latitude, longitude, altitude
    -> build MAVROS Waypoint[]
    -> /mavros/mission/clear
    -> /mavros/mission/push
    -> /mavros/mission/set_current
    -> optional arm
    -> optional AUTO.MISSION
```

The upload implementation is:

```text
airsim_ros2_bridge/airsim_ros2_bridge/gps_route_mission.py
```

The explicit script entrypoint is:

```bash
scripts/upload_mission_json_to_mavros.sh
```

The guided full-stack runner is:

```bash
scripts/run_gps_route_mission_experiment.sh
```

Use `RUNNER_MODE=editor` to open the mission editor, `RUNNER_MODE=dry-run` to
inspect the generated upload command, and the default `RUNNER_MODE=full` for
the guided UE/PX4/MAVROS run.

## Default GPS Mission JSON

Editor missions are exported as GPS waypoints:

```json
{
  "coordinate_frame": "gps",
  "waypoints": [
    {"latitude": 37.5682966, "longitude": 126.9780000, "altitude": 10.0}
  ]
}
```

The editor keeps the original local `east/north/up` points under
`metadata.local_source` for debugging and regeneration. GPS export uses the
fixed AirSim `OriginGeopoint` configured in settings, currently Seoul
`37.5665, 126.9780, 38`.

## Legacy Local Mission JSON

Older editor missions may still be saved as local map/mission coordinates:

```json
{
  "coordinate_frame": "local_enu",
  "waypoints": [
    {"east": 12.3, "north": 45.6, "up": 20.0}
  ]
}
```

This keeps the mission reusable when the AirSim `OriginGeopoint`, PX4 home, or
spawn point changes. GPS is a deployment detail. The current FCU home position
is the source of truth when the mission is uploaded.

## Conversion Rule

For local ENU missions:

```text
east  -> meters east from FCU home
north -> meters north from FCU home
up    -> meters above FCU home
```

At upload, `aerion_gps_route_mission` reads:

```text
/<namespace>/home_position/home
```

Then it converts each local point to a GPS mission point:

```text
latitude  = home.latitude  + north_m / earth_radius
longitude = home.longitude + east_m / (earth_radius * cos(home.latitude))
altitude  = home.altitude + up_m
```

For `ALTITUDE_MODE=relative_to_home`, the MAVLink mission frame is relative
altitude and `z_alt` becomes the waypoint height above home.

## Local MAVROS Upload

Use this when MAVROS is running on the AirSim/PX4 machine:

```bash
cd ~/workspace/projects/aerion-airsim
colcon build --packages-select airsim_ros2_bridge

MISSION_FILE=recordings/missions/clicked_mission_gps.json \
MAVROS_NAMESPACE=drone1/mavros \
START_MISSION=true \
bash scripts/upload_mission_json_to_mavros.sh
```

## Remote MAVROS Upload

Use this when MAVLink is forwarded by MAVProxy and MAVROS runs on an external
machine.

AirSim/PX4 machine:

```bash
# Stop local MAVROS first so MAVProxy can own the PX4 MAVLink input port.
MAC_TAILSCALE_IP=<remote_ip> \
bash scripts/run_mavproxy_to_embodied.sh
```

Remote machine:

```bash
# Run MAVROS connected to the forwarded MAVLink stream first.
# Then upload the GPS mission file:
MISSION_FILE=/path/to/clicked_mission_gps.json \
MAVROS_NAMESPACE=mavros \
START_MISSION=true \
bash scripts/upload_mission_json_to_mavros.sh
```

For remote control, only one writer should send arm, mode, mission, or setpoint
commands. Keep local MAVROS stopped, and keep Zenoh read-only for observation.

## Inputs

Prefer the editor GPS export:

```text
recordings/missions/clicked_mission_gps.json
```

Legacy local and prepared local files still work:

```text
recordings/missions/clicked_mission.json
recordings/missions/clicked_mission_current_home_fcu_axes.json
```

The clicked mission experiment runner auto-detects GPS vs local mission files.
GPS files are uploaded directly; local files are prepared against the current
AirSim/PX4 home before upload.

## Important Options

```text
MISSION_FILE             mission JSON to upload
MAVROS_NAMESPACE         drone1/mavros locally, often mavros remotely
ALTITUDE_MODE            auto: relative for GPS, relative_to_home for local
INCLUDE_TAKEOFF          auto: false for GPS, true for local
TAKEOFF_ALTITUDE_SOURCE  first_waypoint for editor missions
TAKEOFF_POSITION_SOURCE  current to take off from current home position
ACCEPTANCE_RADIUS        waypoint acceptance radius in meters
MISSION_ARM             true to arm during upload
START_MISSION            true to switch to AUTO.MISSION after upload
```

Defaults are conservative: the script uploads the mission but does not arm or
start unless requested.

Use `DRY_RUN=true` to verify the generated upload command without contacting
ROS2/MAVROS.

If ROS2 CLI fails with a CycloneDDS interface error such as `enp108s0: does not
match an available interface`, the saved DDS config is pointing at a down
interface. For local AirSim/PX4 checks, either bring that interface up or run
the command with the default FastDDS middleware:

```bash
env -u CYCLONEDDS_URI RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  MISSION_FILE=recordings/missions/clicked_mission_gps.json \
  START_MISSION=false \
  bash scripts/upload_mission_json_to_mavros.sh
```

## Validation Checklist

Before upload:

```bash
env -u CYCLONEDDS_URI RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
ros2 topic echo --once /drone1/mavros/state

env -u CYCLONEDDS_URI RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
ros2 topic echo --once /drone1/mavros/home_position/home
```

For remote MAVROS, replace `/drone1/mavros` with `/mavros` if that is the remote
namespace.

Expected:

```text
connected: true
home_position/home has finite latitude, longitude, altitude
```
