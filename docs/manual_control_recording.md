# Manual Control Recording

This note covers the AERION manual MAVROS controller additions for GPS route
recording/following and camera frame recording.

## Runtime

```bash
cd ~/workspace/projects/aerion-airsim
source /opt/ros/humble/setup.bash
source install/setup.bash

VEHICLES=drone1 \
ARM_FLAG=false \
FORCE_ARM_FLAG=false \
SPEED=3.0 \
VERTICAL_SPEED=1.5 \
YAW_RATE=1.2 \
RECORDING_DIR=~/workspace/projects/aerion-airsim/recordings \
CAMERA_TOPIC=/drone1/camera/image \
CAMERA_RECORD_FPS=10.0 \
bash scripts/run_manual_3drone_control.sh
```

## Keys

| Key | Action |
| --- | --- |
| `w` / `s` | Move forward / backward |
| `a` / `d` | Move left / right |
| `r` / `f` | Move up / down |
| `q` / `e` | Yaw left / right |
| `space` | Hover |
| `g` | Record the current GPS fix as a route waypoint |
| `o` | Export recorded GPS waypoints to JSON and CSV |
| `l` | Load `ROUTE_FILE` JSON, or the last exported route |
| `v` | Toggle GPS route following |
| `c` | Toggle camera recording |
| `x` | Exit |

## GPS Route Files

GPS points are read from `/<vehicle>/mavros/global_position/global` by default.
Use `GPS_TOPIC=/custom/navsatfix` to override it.

Export writes both files under `$RECORDING_DIR/gps`:

- `gps_route_YYYYMMDD_HHMMSS.json`
- `gps_route_YYYYMMDD_HHMMSS.csv`

Load an existing route before starting route follow:

```bash
ROUTE_FILE=~/workspace/projects/aerion-airsim/recordings/gps/gps_route_YYYYMMDD_HHMMSS.json \
bash scripts/run_manual_3drone_control.sh
```

The route follower uses GPS error to generate velocity commands. It does not
replace PX4 mission mode; it remains an OFFBOARD velocity controller.

For FCU-owned mission replay, upload the exported JSON through MAVROS mission
services instead:

```bash
ROUTE_FILE=~/workspace/projects/aerion-airsim/recordings/gps/gps_route_YYYYMMDD_HHMMSS.json \
MAVROS_NAMESPACE=drone1/mavros \
FIRMWARE=px4 \
ALTITUDE_MODE=relative_to_home \
TAKEOFF_FLAG=true \
TAKEOFF_ALTITUDE_SOURCE=first_waypoint \
START_FLAG=true \
bash scripts/run_gps_route_mission.sh
```

See `docs/gps_route_mission.md` for altitude mode and AUTO mode details.

## Camera Files

Camera frames are read from `/drone1/camera/image` by default. Use
`CAMERA_TOPIC=/custom/image` to override it.

Recording starts with `c` and stops with `c` again. The final file is named by
the stop time and written under `$RECORDING_DIR/camera`:

```text
camera_YYYYMMDD_HHMMSS.avi
```
