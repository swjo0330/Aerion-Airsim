# Town10HD_Opt Clicked Mission Success Handoff

Date: 2026-07-07

This document freezes the successful Town10HD_Opt clicked-mission execution
state for the next session.

## What Worked

The full UE/CARLA + AirSim + PX4 + MAVROS mission runner succeeded with:

```bash
cd ~/workspace/projects/aerion-airsim
bash scripts/run_clicked_mission_experiment.sh
```

The drone spawned at the expected initial position, moved in the correct
heading, and followed the clicked mission with the corrected coordinate home.

## Locked Inputs

- Source mission: `recordings/missions/clicked_mission.json`
- Prepared mission: `recordings/missions/clicked_mission_current_home_fcu_axes.json`
- AirSim NED origin: `recordings/maps/airsim_ned_origin.json`
- Mission runner: `scripts/run_clicked_mission_experiment.sh`
- Full-stack runner: `scripts/run_gps_route_mission_experiment.sh`

## Coordinate Facts

The known-good spawn is:

```text
AirSim Vehicles.drone1.X = 240
AirSim Vehicles.drone1.Y = -170
AirSim Vehicles.drone1.Z = 0.2
```

Those are AirSim global-NED offsets, not CARLA/OpenDRIVE map coordinates.

The AirSim NED origin from the successful UE session is:

```json
{
  "origin_meters": {
    "x": -172.19658203,
    "y": 183.85912109,
    "z": 27.63805176
  },
  "world_to_meters": 100.0
}
```

Therefore the real map home for mission preparation is:

```text
map_home.east  =  183.85912109 - 170  = 13.85912109
map_home.north = -172.19658203 + 240  = 67.80341797
```

The prepared mission currently uses:

```text
origin_map_enu.east  = 13.859121
origin_map_enu.north = 67.803418
axis_transform       = east_from:north, north_from:east
node1 relative       = east 5.942579, north 38.111839, distance 38.572354m
```

## Why This Matters

The main bug was mixing coordinate frames:

- AirSim settings `X/Y/Z` are global-NED offsets from AirSim's Play-time origin.
- Clicked mission points are map/OpenDRIVE-style east/north coordinates.
- The FCU mission must use waypoint deltas relative to the actual spawned home.

The fix is:

```text
actual map home = AirSim NED origin in meters + AirSim spawn offset
```

This is implemented by:

```bash
python3 scripts/compute_map_home_for_airsim_spawn.py \
  --origin recordings/maps/airsim_ned_origin.json \
  --airsim-x 240 \
  --airsim-y -170 \
  --airsim-z 0.2
```

The clicked mission runner performs this automatically in its default
`SPAWN_MODE=previous_home`.

## Diagnostics That Must Stay

The AirSim plugin has UE log diagnostics in:

```text
/home/clrobur/airsim/Colosseum/Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp
```

Expected log lines after UE Play:

```text
AERION_NED_ORIGIN_UE x=... y=... z=... world_to_meters=...
AERION_SPAWN vehicle=drone1 settings_ned_x=... settings_ned_y=... actual_ue_x=...
```

If these disappear, `recordings/maps/airsim_ned_origin.json` cannot be trusted.

## Recalibration

If UE PlayerStart, AirSim origin behavior, map loading, or the AirSim plugin
changes, regenerate the origin:

```bash
cd ~/workspace/projects/aerion-airsim
bash scripts/run_clicked_mission_calibration.sh
```

After plugin source changes, fully close UE and rebuild `CarlaUnrealEditor`.
An already-open UE process keeps the old AirSim plugin binary loaded.

## Validation Commands

Static/script checks:

```bash
cd ~/workspace/projects/aerion-airsim
bash -n scripts/run_clicked_mission_experiment.sh \
  scripts/run_clicked_mission_calibration.sh \
  scripts/run_gps_route_mission_experiment.sh \
  scripts/deploy_px4_1drone_lidar_settings.sh

python3 -m py_compile \
  scripts/compute_map_home_for_airsim_spawn.py \
  scripts/compute_airsim_spawn_for_mission_node.py \
  scripts/extract_airsim_ned_origin_from_ue_log.py \
  scripts/prepare_mission_for_current_fcu_home.py
```

Coordinate check:

```bash
python3 scripts/compute_map_home_for_airsim_spawn.py \
  --origin recordings/maps/airsim_ned_origin.json \
  --airsim-x 240 \
  --airsim-y -170 \
  --airsim-z 0.2
```
