# GPS Route Mission Upload

This is the FCU-backed route replay path. AERION mission JSON is now exported
as GPS mission coordinates by default, then uploaded through the real
flight-controller mission interface. Legacy local ENU mission files still work;
the upload wrapper detects them and converts them before push.

For the team-lead MAVSDK-only local channel smoke from 2026-07-08, use
`scripts/run_mission_smoke.sh` instead. That smoke bypasses ROS2/MAVROS and
validates the PX4 mission channel (`udp://:14540` or the deployed AirSim
settings port) directly; see
[`docs/airsim/2026-07-08-airsim-local-mission-smoke-prep.md`](airsim/2026-07-08-airsim-local-mission-smoke-prep.md).

## Locked Successful Town10HD_Opt Flow

Status locked on 2026-07-07 after successful UE/CARLA + AirSim + PX4/MAVROS
mission execution.

Use this command for the current clicked mission:

```bash
cd ~/workspace/projects/aerion-airsim
bash scripts/run_clicked_mission_experiment.sh
```

The runner is GPS-first. It defaults to
`recordings/missions/clicked_mission_gps.json`, uploads without arming, and
does not start AUTO.MISSION unless you explicitly request it.

Mission editor only:

```bash
cd ~/workspace/projects/aerion-airsim
RUNNER_MODE=editor OPEN_EDITOR=true bash scripts/run_gps_route_mission_experiment.sh
```

After exporting a mission from the editor, verify the upload command without
launching the simulator:

```bash
RUNNER_MODE=dry-run SKIP_START_CONFIRM=true bash scripts/run_gps_route_mission_experiment.sh
```

Full guided stack, upload only:

```bash
bash scripts/run_clicked_mission_experiment.sh
```

Full guided stack, arm and start intentionally:

```bash
MISSION_ARM=true START_MISSION=true bash scripts/run_clicked_mission_experiment.sh
```

Current default inputs:

- Source mission: `recordings/missions/clicked_mission_gps.json`
- Legacy local source: `recordings/missions/clicked_mission.json`
- Prepared legacy local mission:
  `recordings/missions/clicked_mission_current_home_fcu_axes.json`
- AirSim NED origin: `recordings/maps/airsim_ned_origin.json`
- AirSim spawn offset: `X=240`, `Y=-170`, `Z=0.2`
- Actual map/FCU home computed from the current origin:
  `east=13.859121`, `north=67.803418`
- First waypoint relative to FCU home:
  `east=5.943`, `north=38.112`, distance about `38.57m`

Important: `X=240`, `Y=-170` is an AirSim global-NED spawn offset, not a
CARLA/OpenDRIVE map coordinate. The actual mission home must be computed as:

```text
map_home.east  = airsim_ned_origin.origin_meters.y + AirSim Vehicles.drone1.Y
map_home.north = airsim_ned_origin.origin_meters.x + AirSim Vehicles.drone1.X
```

The clicked runner does this automatically through
`scripts/compute_map_home_for_airsim_spawn.py`, then prepares the mission with
`MISSION_HOME_MODE=explicit`.

If UE PlayerStart, AirSim origin, or map loading behavior changes, regenerate
`recordings/maps/airsim_ned_origin.json` before running the mission:

```bash
cd ~/workspace/projects/aerion-airsim
bash scripts/run_clicked_mission_calibration.sh
```

The AirSim plugin must contain the `AERION_NED_ORIGIN_UE` and `AERION_SPAWN`
`UE_LOG` diagnostics in:

```text
/home/clrobur/airsim/Colosseum/Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp
```

After changing that plugin source, fully close UE and rebuild
`CarlaUnrealEditor`; an already-open UE process will keep the old plugin binary.

Recommended mission authoring flow:

1. Author or generate waypoints in the browser mission editor.
2. Export `Mission JSON`; the default file is `coordinate_frame: gps`.
3. Upload the GPS mission directly through MAVROS mission services.
4. Use `metadata.local_source` only for debugging or regenerating a local copy.

The checked-in/generated current mission is also available as
`recordings/missions/clicked_mission_gps.json`, converted from the last local
click export. This aligns the editor output with the PX4 mission protocol while
keeping the old local coordinates available for inspection. Legacy GPS
recordings from `manual_mavros_control` still work.

For the upload boundary and remote MAVROS flow, see
[`docs/mission_upload_logic.md`](mission_upload_logic.md).

## Flow

```text
recordings/missions/*.json or recordings/gps/gps_route_*.json
  -> aerion_gps_route_mission
  -> /drone1/mavros/mission/clear
  -> /drone1/mavros/mission/push
  -> /drone1/mavros/mission/set_current
  -> optional /drone1/mavros/cmd/arming
  -> optional /drone1/mavros/set_mode AUTO.MISSION
  -> PX4/ArduPilot SITL mission executor
```

## Example

Interactive CARLA/PX4/MAVROS experiment runner:

```bash
cd ~/workspace/projects/aerion-airsim

ROUTE_FILE=~/workspace/projects/aerion-airsim/recordings/missions/clicked_mission_gps.json \
START_BRIDGE=false \
bash scripts/run_gps_route_mission_experiment.sh
```

The runner deploys settings, launches the CARLA UE project, then waits for you
to press Enter after UE Play. It starts PX4 and MAVROS as background processes,
applies PX4 validation parameters, and then runs the mission uploader.

Manual upload command:

```bash
cd ~/workspace/projects/aerion-airsim
colcon build --packages-select airsim_ros2_bridge

MISSION_FILE=recordings/missions/clicked_mission_gps.json \
MAVROS_NAMESPACE=drone1/mavros \
START_MISSION=true \
bash scripts/upload_mission_json_to_mavros.sh
```

Legacy wrapper:

```bash
cd ~/workspace/projects/aerion-airsim

ROUTE_FILE=~/workspace/projects/aerion-airsim/recordings/gps/gps_route_YYYYMMDD_HHMMSS.json \
MAVROS_NAMESPACE=drone1/mavros \
FIRMWARE=px4 \
ALTITUDE_MODE=relative_to_home \
TAKEOFF_FLAG=true \
TAKEOFF_ALTITUDE_SOURCE=first_waypoint \
START_FLAG=true \
bash scripts/run_gps_route_mission.sh
```

Use `FIRMWARE=ardupilot` to default the start mode to `AUTO` instead of PX4's
`AUTO.MISSION`.

## Build Missions

Generate reusable local-meter missions:

```bash
cd ~/workspace/projects/aerion-airsim
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run airsim_ros2_bridge aerion_mission_builder \
  --pattern line \
  --count 4 \
  --spacing 12 \
  --altitude 5 \
  --out recordings/missions/line_4wp_5m.json

ros2 run airsim_ros2_bridge aerion_mission_builder \
  --pattern box \
  --north-size 30 \
  --east-size 20 \
  --altitude 6 \
  --out recordings/missions/box_30x20_6m.json

ros2 run airsim_ros2_bridge aerion_mission_builder \
  --pattern road \
  --road-file recordings/road_waypoints.json \
  --out recordings/missions/town10_road_shape.json
```

Then use the generated file as `ROUTE_FILE` for the experiment runner or manual
upload command.

## Complex Missions

For complex routes, keep the source route editable and generate a normalized
mission file from it. The preferred source format is local meters from the
current FCU home position:

```csv
east,north,up
0,0,8
8,12,8
22,22,9
40,22,9
```

Convert and validate the source route:

```bash
cd ~/workspace/projects/aerion-airsim
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run airsim_ros2_bridge aerion_mission_builder \
  --pattern import \
  --input recordings/missions/complex_s_curve.csv \
  --name complex_s_curve_8m \
  --resample-spacing 8 \
  --max-segment 10 \
  --min-altitude 2 \
  --max-altitude 30 \
  --out recordings/missions/complex_s_curve_8m.json
```

The generated mission can be uploaded through the FCU-backed runner:

```bash
cd ~/workspace/projects/aerion-airsim

ROUTE_FILE=~/workspace/projects/aerion-airsim/recordings/missions/complex_s_curve_8m.json \
ALTITUDE_MODE=relative_to_home \
TAKEOFF_ALTITUDE_SOURCE=first_waypoint \
PREARM_OFFBOARD=true \
MISSION_ARM=false \
START_MISSION=true \
START_BRIDGE=false \
CLEAN_START_STOP_UE=true \
SKIP_START_CONFIRM=true \
bash scripts/run_gps_route_mission_experiment.sh
```

Useful builder options:

- `--pattern import`: import CSV or JSON instead of generating a primitive path.
- `--resample-spacing 8`: split long route legs into about 8m mission segments.
- `--smooth-iterations 1`: smooth sharp corners before resampling.
- `--validate-only`: check a route without writing a mission file.
- `--max-segment`, `--min-altitude`, `--max-altitude`, `--max-waypoints`: catch
  routes that are risky or too large before upload.

Import accepts CSV headers such as `east,north,up`, `x,y,z`, or NED-style
`north,east,down` when `frame`/`coordinate_frame` is set to `local_ned` or
`airsim_ned`.

## Click Mission Editor

For drone routes, a road-only path is usually too restrictive. The practical
workflow is a standalone browser page generated from the CARLA OpenDRIVE map,
optionally combined with an AirSim mesh heightmap scan.

The current source of truth is the loaded CARLA map, `Town10HD_Opt`, and its
local OpenDRIVE file:

```text
~/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr
```

Do not use the Town10 web documentation image as a coordinate source. It is only
a visual reference and can differ from the local packaged map, loaded sublevels,
orientation, crop, or rendered props. Regenerate the editor/safety map from the
current CARLA world before authoring missions.

If Town10HD_Opt renders with missing or black materials, repair known migrated
material dependencies before capturing the top-down map:

```bash
cd ~/workspace/projects/aerion-airsim
bash scripts/run_town10_texture_repair.sh
```

For CARLA+AirSim Town10 runs, prefer the stable render profile:

```bash
cd ~/workspace/projects/aerion-airsim
bash scripts/run_ue_carla_town10.sh
```

```bash
cd ~/workspace/projects/aerion-airsim

SCAN_HEIGHTMAP=false \
bash scripts/build_mission_planning_assets.sh

google-chrome recordings/maps/mission_editor.html
```

If the CARLA UE project is running and the target map is open, build the editor
with a CARLA raycast height scan:

```bash
cd ~/workspace/projects/aerion-airsim

HEIGHTMAP_SCAN_METHOD=carla_raycast \
SCAN_HEIGHTMAP=true \
REQUIRE_HEIGHTMAP=true \
HEIGHTMAP_RESOLUTION=2 \
SCAN_TIMEOUT_SEC=7200 \
CARLA_TIMEOUT=60 \
CARLA_RAYCAST_WORKERS=2 \
CARLA_RAYCAST_PROGRESS_EVERY=10 \
MAP_BACKGROUND_IMAGE= \
TOPVIEW_ALIGNMENT= \
AUTO_TOPVIEW_ALIGNMENT=false \
DISPLAY_FLIP_Y=true \
bash scripts/build_mission_planning_assets.sh
```

This calls `scripts/scan_carla_raycast_heightmap.py`, which samples the CARLA
world with vertical raycasts and writes `recordings/maps/heightmap.json`. The
integrated builder also renders `recordings/maps/heightmap_map.png` and embeds
it as the editor's top-down safety layer. The editor uses the heightmap for
recommended waypoint altitude and segment clearance checks.

AirSim mesh scanning through `simGetMeshPositionVertexBuffers` is intentionally
not the default for Town10. On this UE/Vulkan setup it has caused Unreal to
terminate while enumerating the full map mesh. Only use
`HEIGHTMAP_SCAN_METHOD=airsim_mesh` with `ALLOW_UNSTABLE_AIRSIM_MESH_SCAN=true`
when you explicitly accept that risk.

For a full Town10 scan, use CARLA raycast directly with checkpoints:

```bash
cd ~/workspace/projects/aerion-airsim

python3 scripts/scan_carla_raycast_heightmap.py \
  --xodr ~/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr \
  --resolution 2.0 \
  --z-top 300 \
  --z-bottom -80 \
  --timeout 60 \
  --workers 6 \
  --progress-every 10 \
  --checkpoint-every 10 \
  --resume \
  --output recordings/maps/town10hdopt_heightmap_2m.json

python3 scripts/render_heightmap_map.py \
  --heightmap recordings/maps/town10hdopt_heightmap_2m.json \
  --output recordings/maps/town10hdopt_semantic_2m.png \
  --scale 3 \
  --mode semantic

HEIGHTMAP=recordings/maps/town10hdopt_heightmap_2m.json \
HEIGHTMAP_IMAGE=recordings/maps/town10hdopt_semantic_2m.png \
SCAN_HEIGHTMAP=false \
MAP_BACKGROUND_IMAGE= \
TOPVIEW_ALIGNMENT= \
AUTO_TOPVIEW_ALIGNMENT=false \
DISPLAY_FLIP_Y=true \
bash scripts/build_mission_planning_assets.sh
```

The `Town10HD_Opt` OpenDRIVE header bounds are much smaller than the older
`Mine_01` bounds. If the editor shows a very large outer area or unrelated
structures, rebuild the planning assets and verify that the XODR path is
`Town10HD_Opt.xodr`.

Do not rotate or flip the scan file itself. Keep scan coordinates in CARLA
OpenDRIVE map X/Y. If the aerial image orientation is different, solve it in the
top-view alignment step with control points; the affine alignment supports
rotation, scale, translation, and mild skew.

Generated planning assets:

```text
recordings/maps/heightmap.json      # scanned 2.5D obstacle/terrain heights
recordings/maps/heightmap_map.png   # visual top-down map rendered from scan
recordings/maps/home_origin.json    # queried AirSim vehicle origin, if available
recordings/maps/mission_editor.html # standalone click mission editor
```

The same integrated script also tries to query the current AirSim vehicle pose
with `scripts/query_airsim_home_origin.py` and prefill `Home map X/Y`. If the
query fails, the editor falls back to `0,0`.

The editor lets you click waypoints on the map, set a clearance altitude, use
the scanned heightmap plus an optional manual obstacle layer, validate segment
clearance, and download a GPS mission JSON that can be passed directly to the
mission runner. The original `local_enu` points are preserved under
`metadata.local_source`.

The editor opens in a clean `Photo` view. Use the layer presets when aligning or
checking a route:

- `Photo`: aerial image only.
- `Safety`: aerial image plus scanned height layer.

OpenDRIVE `planView` reference lines are not rendered as a map overlay because
they do not match the visible road surfaces in Town10. If a road/lane overlay is
needed later, generate it from CARLA lane waypoints or image segmentation rather
than reusing the raw OpenDRIVE geometry lines.

## Top-View Calibration

The aerial image from the CARLA documentation is useful, but it has to be
aligned to OpenDRIVE map meters before click missions can be trusted. The editor
has a built-in control-point workflow so you do not have to hand-write pixel
coordinates.

Build the editor with the aerial image and the current coarse alignment. Town10
uses the vertical-flipped coarse alignment because the aerial image orientation
matches the simulator view while the initial scan/map overlay was inverted on
the Y axis:

```bash
cd ~/workspace/projects/aerion-airsim

MAP_BACKGROUND_IMAGE=recordings/maps/town10_aerial.webp \
TOPVIEW_ALIGNMENT=recordings/maps/topview_alignment.json \
DISPLAY_FLIP_Y=true \
SCAN_HEIGHTMAP=false \
QUERY_HOME_ORIGIN=false \
bash scripts/build_mission_planning_assets.sh

google-chrome recordings/maps/mission_editor.html
```

Keep `DISPLAY_FLIP_Y=true` for the current Town10 editor view. This flips the
photo and safety scan together, preserving their overlay alignment while matching
the simulator's top-view orientation. The editor status panel should show
`display axes: screen up -North, screen right +East`; this is expected.
Use the default `Export axes` value, `Visual Screen: screen ↑->N, screen ->->E`,
for mission authoring. It compensates for the display flip so visual screen-up
exports as mission `+North`.

In the editor:

1. Press `Image Point`, then click a precise landmark on the aerial photo.
2. Press or wait for `Map Point`, then click the same landmark on the
   map-coordinate reference you are calibrating against.
3. Repeat with 6-10 landmarks spread across the map.
4. Press `Control JSON` and save the downloaded file as
   `recordings/maps/topview_control_points.json`.

Good landmarks are lane-center intersections, road corners, crosswalk corners,
and the centers of small round features. Avoid tree tops, shadows, parked cars,
and roof edges when possible.

Convert the downloaded control points into an alignment file:

```bash
cd ~/workspace/projects/aerion-airsim

python3 scripts/calibrate_topview_map.py \
  --method similarity \
  --control-points recordings/maps/topview_control_points.json \
  --image recordings/maps/town10_aerial.webp \
  --output recordings/maps/topview_alignment.json
```

Use `--method similarity` first. It allows translation, rotation, and uniform
scale but prevents skew, so the aerial image will not be visibly distorted. Use
`--method affine` only after you have many accurate control points and actually
need shear correction. If RMS error is larger than a few meters, re-pick the
control points instead of accepting the alignment.

Then rebuild the editor with the calibrated alignment:

```bash
MAP_BACKGROUND_IMAGE=recordings/maps/town10_aerial.webp \
TOPVIEW_ALIGNMENT=recordings/maps/topview_alignment.json \
SCAN_HEIGHTMAP=false \
QUERY_HOME_ORIGIN=false \
bash scripts/build_mission_planning_assets.sh
```

Check the reported RMS error. Under about 2-5m is usually enough for mission
sketching. If the error is larger, add or replace control points near the area
where you plan to fly.

Current Town10 editor alignment:

- `recordings/maps/topview_alignment.json` is a `similarity` alignment built
  from nine control points in `recordings/maps/topview_control_points.json`.
- RMS error is about `1.72m`, max error about `2.73m`.
- The aerial photo covers the central mission area, not the whole scanned CARLA
  map. This is intentional for central-area missions; use the scan layer outside
  the photo only as a safety/obstacle reference, not as a photo-aligned map.
- Keep `DISPLAY_FLIP_Y=true` with this alignment so the photo and safety scan are
  flipped together and remain matched.
- Keep `Export axes` at `Visual Screen: screen ↑->N, screen ->->E` for normal
  mission authoring. `Map ENU` and the rotation presets are only for diagnosing
  older/legacy exports.
- Do not flip only the image-space alignment. That breaks the photo/safety
  overlay. The correct visual fix is the whole-display flip above.

Current-world semantic editor:

When the CARLA documentation image does not match the running Unreal world,
build the editor from the current CARLA raycast semantic map instead of the
external photo:

```bash
cd ~/workspace/projects/aerion-airsim

python3 scripts/render_heightmap_map.py \
  --heightmap recordings/maps/heightmap.json \
  --output recordings/maps/carla_world_semantic_2m.png \
  --scale 3 \
  --mode semantic

HEIGHTMAP=recordings/maps/heightmap.json \
HEIGHTMAP_IMAGE=recordings/maps/carla_world_semantic_2m.png \
MAP_BACKGROUND_IMAGE= \
TOPVIEW_ALIGNMENT= \
AUTO_TOPVIEW_ALIGNMENT=false \
DISPLAY_FLIP_Y=true \
SCAN_HEIGHTMAP=false \
QUERY_HOME_ORIGIN=false \
bash scripts/build_mission_planning_assets.sh

google-chrome recordings/maps/mission_editor.html
```

This uses the same CARLA raycast data for the visible map and the safety layer,
so there is no photo/safety calibration mismatch. The map is less photorealistic
than the docs image, but it is tied to the running map geometry.

Important coordinate rule:

- The map uses OpenDRIVE X/Y.
- The exported mission uses GPS coordinates generated from the fixed
  `OriginGeopoint`.
- `Home map X` and `Home map Y` define the local source anchor that is preserved
  in mission metadata and used for GPS conversion.
- If AirSim is not running or the queried origin is wrong, override manually:

```bash
HOME_MAP_X=240 \
HOME_MAP_Y=-170 \
SCAN_HEIGHTMAP=false \
bash scripts/build_mission_planning_assets.sh
```

- If the first clicked point is the takeoff/home point, use `Home=First`.

The manual obstacle layer is intentionally simple and is additive on top of the
scanned heightmap:

```json
[
  {"type": "rect", "name": "building", "x": 20, "y": 20, "w": 24, "h": 18, "height": 14},
  {"type": "circle", "name": "tower", "x": 68, "y": -8, "r": 10, "height": 24}
]
```

If the editor status says `heightmap: not loaded`, the map is not scanned and
the safety model only contains the manual obstacle layer. In that mode, use it
for mission sketching only and keep clearance margins generous.

CLI validation can repeat the same heightmap clearance check before upload:

```bash
ros2 run airsim_ros2_bridge aerion_mission_builder \
  --pattern import \
  --input recordings/missions/clicked_mission.json \
  --heightmap recordings/maps/heightmap.json \
  --clearance 8 \
  --heightmap-unknown-policy error \
  --validate-only
```

If the input mission was exported by the editor, the local source home map
origin is read from `metadata.local_source.home_origin_map_xy`. For hand-written
local missions, pass `--home-map-x` and `--home-map-y`.

Use `--heightmap-unknown-policy error` before a real upload. It fails the
mission if any sampled route point falls outside scanned heightmap coverage.

## Mission File Frames

Supported waypoint frames:

- `global`, `gps`, `wgs84`: waypoint has `latitude`, `longitude`, `altitude`.
- `local_enu`: waypoint has `east`, `north`, `up` in meters from current FCU
  home.
- `local_ned`, `airsim_ned`: waypoint has `north`, `east`, `down` in meters
  from current FCU home.
- `carla_opendrive`: waypoint has `x` as east, `y` as north, `z` as up.
- legacy `waypoints_ned_rel`: existing `extract_road_waypoints.py` output.

Example legacy local mission:

```json
{
  "schema": "aerion_mission_v1",
  "name": "short_line",
  "coordinate_frame": "local_enu",
  "waypoints": [
    {"east": 0, "north": 0, "up": 5},
    {"east": 10, "north": 0, "up": 5},
    {"east": 20, "north": 0, "up": 5}
  ]
}
```

## Altitude Mode

- `relative`: write route `altitude` values as `FRAME_GLOBAL_REL_ALT`; this is
  the default for GPS editor exports.
- `absolute`: write route `altitude` values as `FRAME_GLOBAL`.
- `relative_from_first`: subtract the first recorded altitude from every route
  point and write the result as `FRAME_GLOBAL_REL_ALT`.
- `relative_to_home`: subtract the current MAVROS home altitude from each
  recorded absolute altitude and write the result as `FRAME_GLOBAL_REL_ALT`;
  this is the default for legacy local missions.

The old GPS recorder stores `NavSatFix.altitude`. In AirSim-direct bridge mode
that value is synthesized, so check the exported JSON before starting a real
mission. For PX4 SITL missions from `/mavros/global_position/global`,
`relative_to_home` is usually safer than blindly treating recorded values as
PX4 mission absolute altitude.

## Takeoff Altitude

When `--include-takeoff` is used, `--takeoff-altitude-source` controls the
prepended `MAV_CMD_NAV_TAKEOFF` altitude:

- `fixed`: use `--takeoff-altitude`.
- `first_waypoint`: use the first route waypoint after altitude conversion.
- `max_fixed_or_first`: use whichever is higher.

## Control Ownership

Only one mission or control writer should own the FCU at a time. Stop local
manual control and other setpoint publishers before starting AUTO mission mode.
