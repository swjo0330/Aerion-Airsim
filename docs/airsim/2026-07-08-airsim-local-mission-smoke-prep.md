# AirSim Local Mission Smoke Prep

This note maps the 2026-07-08 team-lead PDF onto the current AERION repo.

## What The PDF Requires

- Validate the PX4 MAVLink mission channel directly with MAVSDK.
- Use the PX4 offboard/API UDP channel, normally `udp://:14540`.
- Do not use AirSim RPC movement commands or the ROS2 bridge for this smoke.
- Wait for heartbeat, GPS/home health, upload a GPS mission, download it back,
  compare item count/coordinates, arm, start `AUTO.MISSION`, watch progress,
  then RTL/disarm.
- Default route is home-relative northbound 1 km, split into 200 m mission
  items. `--wps` can provide absolute GPS destinations near the current
  `OriginGeopoint`.
- The team-lead zip uses `--url udpin://0.0.0.0:14540` and `--bearing`; the repo
  wrapper keeps compatible aliases.

## Current Repo Status

The existing completed work is the production/editor path:

```text
mission editor GPS JSON
  -> scripts/run_clicked_mission_experiment.sh
  -> scripts/run_gps_route_mission_experiment.sh
  -> MAVROS mission services
  -> PX4 AUTO.MISSION
```

That flow is documented in `docs/gps_route_mission.md` and
`docs/mission_upload_logic.md`. It remains valid, but it is not the PDF smoke:
it depends on ROS2/MAVROS. Legacy local JSON is still supported as a fallback,
but current editor export and `clicked_mission_gps.json` are already GPS
mission files.

The PDF smoke is now prepared as:

```text
scripts/mission_smoke.py
scripts/run_mission_smoke.sh
```

It uses MAVSDK only and is therefore a channel smoke for the later Mac
MAVProxy/MAVROS handoff.

## Run

Install MAVSDK once only when you are ready to run this smoke:

```bash
python3 -m pip install mavsdk
```

Start UE AirSim in Play, start PX4 SITL, and ensure no local MAVROS/MAVProxy is
already bound to the smoke port. Then run:

```bash
cd ~/workspace/projects/aerion-airsim
bash scripts/run_mission_smoke.sh
```

For absolute GPS targets near the current AirSim origin:

```bash
MISSION_WPS="37.5683,126.9780;37.5701,126.9780" \
bash scripts/run_mission_smoke.sh
```

If the active AirSim settings use a legacy PX4 API port, override it:

```bash
MAVSDK_PORT=15040 bash scripts/run_mission_smoke.sh
```

To run with the exact URL style from the zip:

```bash
MAVSDK_URL=udpin://0.0.0.0:14540 bash scripts/run_mission_smoke.sh
```

Note: `settings/px4_1drone_lidar.json` is aligned to the team-lead default
`ControlPortLocal=14540` and `ControlPortRemote=14580`. Use the port that
matches the deployed `~/Documents/AirSim/settings.json`.

## Pass Criteria

- S1 heartbeat connects.
- S2 global/home health becomes OK.
- S4 upload/download round trip matches the generated mission.
- S6 mission progress reaches `N/N`.
- UE view shows actual takeoff and waypoint movement.
- RTL is requested and disarm is observed, or `--no-rtl` was intentionally used.

## Relation To Mac Embodied Integration

After this smoke passes, the same PX4 API channel can be handed to MAVProxy:

```bash
mavproxy.py --master=udpin:0.0.0.0:14540 --out=udpout:<Mac_TS_IP>:14555 --daemon
```

Keep only one writer on the port. Stop local MAVSDK/MAVROS before running
MAVProxy for the Mac integration path.

## Current Pre-Integration Order

1. UE Play.
2. PX4 SITL.
3. Local ROS/MAVROS channel check.
4. GPS mission upload dry-run:

   ```bash
   DRY_RUN=true bash scripts/upload_mission_json_to_mavros.sh
   ```

5. GPS mission upload without starting:

   ```bash
   env -u CYCLONEDDS_URI RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
     MISSION_FILE=recordings/missions/clicked_mission_gps.json \
     START_MISSION=false \
     bash scripts/upload_mission_json_to_mavros.sh
   ```

6. After upload/download looks correct, arm and start intentionally:

   ```bash
   env -u CYCLONEDDS_URI RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
     MISSION_FILE=recordings/missions/clicked_mission_gps.json \
     MISSION_ARM=true \
     START_MISSION=true \
     bash scripts/upload_mission_json_to_mavros.sh
   ```
