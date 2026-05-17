#!/usr/bin/env bash
# Smoke-test the verified AirSim ROS2 bridge path from WSL.
set -euo pipefail

ROS_WS="${ROS_WS:-$HOME/aerion_ros2_ws}"
AIRSIM_IP="${AIRSIM_IP:-172.23.80.1}"
AIRSIM_PORT="${AIRSIM_PORT:-41451}"
AIRSIM_TIMEOUT_SEC="${AIRSIM_TIMEOUT_SEC:-2.0}"
VEHICLE_NAME="${VEHICLE_NAME:-Drone0}"
VEHICLE_INDEX="${VEHICLE_INDEX:-${VEHICLE_NAME##*[!0-9]}}"
VEHICLE_INDEX="${VEHICLE_INDEX:-0}"
MAVROS_NAMESPACE="${MAVROS_NAMESPACE:-mavros${VEHICLE_INDEX}}"
ARDU_COMPAT_VEHICLE="${ARDU_COMPAT_VEHICLE:-Drone0}"
if [ -z "${ENABLE_ARDU_COMPAT+x}" ]; then
    if [ "$VEHICLE_NAME" = "$ARDU_COMPAT_VEHICLE" ]; then
        ENABLE_ARDU_COMPAT=true
    else
        ENABLE_ARDU_COMPAT=false
    fi
fi
ROS_TOPIC_TIMEOUT="${ROS_TOPIC_TIMEOUT:-30}"
MOVE_SPEED_X="${MOVE_SPEED_X:-0.5}"
MOVE_DURATION_SEC="${MOVE_DURATION_SEC:-1.0}"
MIN_MOVE_DELTA="${MIN_MOVE_DELTA:-0.05}"
PROBE_TARGET_DX="${PROBE_TARGET_DX:-0.5}"
PROBE_TARGET_DY="${PROBE_TARGET_DY:-0.0}"
PROBE_TARGET_DZ="${PROBE_TARGET_DZ:-0.0}"
PROBE_TOLERANCE="${PROBE_TOLERANCE:-0.20}"
PROBE_MAX_DURATION_SEC="${PROBE_MAX_DURATION_SEC:-60.0}"
TAKEOFF_SERVICE="${TAKEOFF_SERVICE:-/${MAVROS_NAMESPACE}/cmd/takeoff}"
LAND_SERVICE="${LAND_SERVICE:-/${MAVROS_NAMESPACE}/cmd/land}"
TAKEOFF_ALTITUDE="${TAKEOFF_ALTITUDE:-2.0}"
TAKEOFF_SETTLE_SEC="${TAKEOFF_SETTLE_SEC:-2.0}"
LAND_SETTLE_SEC="${LAND_SETTLE_SEC:-2.0}"
RESET_ROS_DAEMON="${RESET_ROS_DAEMON:-false}"
CONTROL_BACKEND="${CONTROL_BACKEND:-px4_mavros}"
DISABLE_FASTDDS_SHM="${DISABLE_FASTDDS_SHM:-true}"
LAND_ON_EXIT=false
SMOKE_TMP_DIR="$(mktemp -d "${TMPDIR:-$HOME}/aerion_smoke.XXXXXX")"

tmp_path() {
    local name="$1"
    echo "${SMOKE_TMP_DIR}/${name}"
}

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: ROS2 Humble setup not found at /opt/ros/humble/setup.bash" >&2
    exit 1
fi

if [ ! -f "$ROS_WS/install/setup.bash" ]; then
    echo "ERROR: ROS workspace setup not found at $ROS_WS/install/setup.bash" >&2
    echo "Build first: cd $ROS_WS && colcon build --packages-select airsim_ros2_bridge" >&2
    exit 1
fi

cd "$ROS_WS"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

if [ "$DISABLE_FASTDDS_SHM" = "true" ]; then
    # Avoid FastDDS shared-memory port lock conflicts in long-lived WSL sessions.
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
fi

if [ "$RESET_ROS_DAEMON" = "true" ]; then
    ros2 daemon stop || true
    ros2 daemon start
    sleep 3
fi

pose_xyz() {
    python3 - "$AIRSIM_IP" "$AIRSIM_PORT" "$AIRSIM_TIMEOUT_SEC" "$VEHICLE_NAME" <<'PY'
import sys
import contextlib
import airsim

ip, port, timeout, vehicle = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
c = airsim.MultirotorClient(ip=ip, port=port, timeout_value=timeout)
with open("/dev/null", "w") as devnull, contextlib.redirect_stdout(devnull):
    c.confirmConnection()
state = c.client.call("getMultirotorState", vehicle)
try:
    pose = c.simGetVehiclePose(vehicle_name=vehicle)
    print(f"{pose.position.x_val} {pose.position.y_val} {pose.position.z_val}")
except Exception:
    state = c.client.call("getMultirotorState", vehicle)
    pos = state[1][0]
    print(f"{pos[0]} {pos[1]} {pos[2]}")
PY
}

mavros_pose_xyz() {
    local pose_file
    pose_file="$(tmp_path "${MAVROS_NAMESPACE}_pose.txt")"
    timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once "/${MAVROS_NAMESPACE}/local_position/pose" >"$pose_file"
    python3 - "$pose_file" <<'PY'
import re
import sys
from pathlib import Path
text = Path(sys.argv[1]).read_text()
vals = []
for axis in ("x", "y", "z"):
    m = re.search(rf"^\s*{axis}:\s*([-+0-9.eE]+)\s*$", text, flags=re.MULTILINE)
    vals.append(float(m.group(1)) if m else 0.0)
print(f"{vals[0]} {vals[1]} {vals[2]}")
PY
}

call_tol_service() {
    local service_name="$1"
    local label="$2"
    local response_file
    response_file="$(tmp_path "tol_response.txt")"

    echo "Calling ${label}: ${service_name}..."
    timeout "$ROS_TOPIC_TIMEOUT" ros2 service call "$service_name" mavros_msgs/srv/CommandTOL \
        "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: ${TAKEOFF_ALTITUDE}}" \
        >"$response_file"
    cat "$response_file"
    if ! grep -Eq "success=[Tt]rue|success: [Tt]rue" "$response_file"; then
        echo "ERROR: ${label} service did not report success=true" >&2
        exit 1
    fi
}

cleanup_land() {
    if [ "${LAND_ON_EXIT:-false}" != "true" ]; then
        return
    fi

    local cleanup_file
    cleanup_file="$(tmp_path "cleanup_land_response.txt")"
    echo "Cleanup: attempting land via ${LAND_SERVICE}..." >&2
    timeout "$ROS_TOPIC_TIMEOUT" ros2 service call "$LAND_SERVICE" mavros_msgs/srv/CommandTOL \
        "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: ${TAKEOFF_ALTITUDE}}" \
        >"$cleanup_file" 2>&1 || true
    cat "$cleanup_file" >&2 || true
}

cleanup_tmp_dir() {
    rm -rf "$SMOKE_TMP_DIR" 2>/dev/null || true
}

trap 'cleanup_land; cleanup_tmp_dir' EXIT

echo "Checking AirSim RPC at ${AIRSIM_IP}:${AIRSIM_PORT} for ${VEHICLE_NAME}..."
before_pose="$(pose_xyz)"
echo "Initial AirSim pose: $before_pose"
if [ "$CONTROL_BACKEND" = "px4_mavros" ]; then
    before_mavros_pose="$(mavros_pose_xyz)"
    echo "Initial MAVROS pose: $before_mavros_pose"
fi

if [ "$ENABLE_ARDU_COMPAT" = "true" ]; then
    echo "Checking /ap/status..."
    ap_status_file="$(tmp_path "ap_status.txt")"
    timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once /ap/status >"$ap_status_file"
    cat "$ap_status_file"
    if ! grep -q "api_control=true" "$ap_status_file"; then
        echo "ERROR: /ap/status does not report api_control=true" >&2
        exit 1
    fi

    echo "Checking /ap/pose/filtered..."
    ap_pose_file="$(tmp_path "ap_pose.txt")"
    timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once /ap/pose/filtered >"$ap_pose_file"
    head -n 20 "$ap_pose_file"

    echo "Checking /ap/twist/filtered..."
    ap_twist_file="$(tmp_path "ap_twist.txt")"
    timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once /ap/twist/filtered >"$ap_twist_file"
    head -n 20 "$ap_twist_file"
else
    echo "SKIP: /ap compatibility topics because ENABLE_ARDU_COMPAT=false for ${VEHICLE_NAME}"
fi

echo "Checking expanded MAVROS/Aerion compatibility topics..."
topic_check_file="$(tmp_path "topic_check.txt")"
while read -r topic msg_type; do
    timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once "$topic" "$msg_type" >"$topic_check_file"
    echo "OK: $topic"
done <<TOPICS
/${VEHICLE_NAME}/mavros/local_position/pose geometry_msgs/msg/PoseStamped
/${VEHICLE_NAME}/mavros/local_position/odom nav_msgs/msg/Odometry
/${VEHICLE_NAME}/mavros/local_position/velocity_local geometry_msgs/msg/TwistStamped
/${VEHICLE_NAME}/mavros/global_position/global sensor_msgs/msg/NavSatFix
/${VEHICLE_NAME}/mavros/global_position/rel_alt std_msgs/msg/Float64
/${VEHICLE_NAME}/mavros/imu/data sensor_msgs/msg/Imu
/${VEHICLE_NAME}/mavros/battery sensor_msgs/msg/BatteryState
/${MAVROS_NAMESPACE}/local_position/pose geometry_msgs/msg/PoseStamped
/${MAVROS_NAMESPACE}/imu/data sensor_msgs/msg/Imu
/${MAVROS_NAMESPACE}/global_position/global sensor_msgs/msg/NavSatFix
TOPICS

if [ "$ENABLE_ARDU_COMPAT" = "true" ]; then
    while read -r topic msg_type; do
        timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once "$topic" "$msg_type" >"$topic_check_file"
        echo "OK: $topic"
    done <<'TOPICS'
/ap/navsat sensor_msgs/msg/NavSatFix
/ap/imu/experimental/data sensor_msgs/msg/Imu
/ap/battery sensor_msgs/msg/BatteryState
/mavros/local_position/odom nav_msgs/msg/Odometry
/mavros/local_position/velocity_local geometry_msgs/msg/TwistStamped
/mavros/global_position/global sensor_msgs/msg/NavSatFix
/mavros/global_position/rel_alt std_msgs/msg/Float64
/mavros/imu/data sensor_msgs/msg/Imu
/mavros/battery sensor_msgs/msg/BatteryState
/odometry nav_msgs/msg/Odometry
/imu sensor_msgs/msg/Imu
/navsat sensor_msgs/msg/NavSatFix
/battery sensor_msgs/msg/BatteryState
TOPICS
fi

if ros2 interface show mavros_msgs/msg/State >/dev/null 2>&1; then
    timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once "/${MAVROS_NAMESPACE}/state" >"$(tmp_path "${MAVROS_NAMESPACE}_state.txt")"
    echo "OK: /${MAVROS_NAMESPACE}/state"
    if [ "$ENABLE_ARDU_COMPAT" = "true" ]; then
        timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once /mavros/state >"$(tmp_path "mavros_state.txt")"
        echo "OK: /mavros/state"
    fi
else
    echo "SKIP: /mavros state topics because mavros_msgs/msg/State is not installed"
fi

if ros2 interface show mavros_msgs/msg/ExtendedState >/dev/null 2>&1; then
    timeout "$ROS_TOPIC_TIMEOUT" ros2 topic echo --once "/${MAVROS_NAMESPACE}/extended_state" >"$(tmp_path "${MAVROS_NAMESPACE}_extended_state.txt")"
    echo "OK: /${MAVROS_NAMESPACE}/extended_state"
else
    echo "SKIP: /${MAVROS_NAMESPACE}/extended_state because mavros_msgs/msg/ExtendedState is not installed"
fi

if ros2 interface show mavros_msgs/srv/CommandBool >/dev/null 2>&1 && ros2 interface show mavros_msgs/srv/SetMode >/dev/null 2>&1; then
    for service_name in \
        "/${VEHICLE_NAME}/mavros/cmd/arming" "/${VEHICLE_NAME}/mavros/set_mode" \
        "/${MAVROS_NAMESPACE}/cmd/arming" "/${MAVROS_NAMESPACE}/set_mode"
    do
        if ros2 service list | grep -Fx "$service_name" >/dev/null 2>&1; then
            echo "OK: $service_name"
        else
            echo "ERROR: missing service $service_name" >&2
            exit 1
        fi
    done
    if [ "$ENABLE_ARDU_COMPAT" = "true" ]; then
        for service_name in /mavros/cmd/arming /mavros/set_mode; do
            if ros2 service list | grep -Fx "$service_name" >/dev/null 2>&1; then
                echo "OK: $service_name"
            else
                echo "ERROR: missing service $service_name" >&2
                exit 1
            fi
        done
    fi
else
    echo "SKIP: MAVROS service checks because mavros_msgs srv types are not installed"
fi

if ros2 interface show mavros_msgs/srv/CommandTOL >/dev/null 2>&1; then
    for service_name in \
        "/${VEHICLE_NAME}/mavros/cmd/takeoff" "/${VEHICLE_NAME}/mavros/cmd/land" \
        "/${MAVROS_NAMESPACE}/cmd/takeoff" "/${MAVROS_NAMESPACE}/cmd/land"
    do
        if ros2 service list | grep -Fx "$service_name" >/dev/null 2>&1; then
            echo "OK: $service_name"
        else
            echo "ERROR: missing service $service_name" >&2
            exit 1
        fi
    done
    if [ "$ENABLE_ARDU_COMPAT" = "true" ]; then
        for service_name in /mavros/cmd/takeoff /mavros/cmd/land; do
            if ros2 service list | grep -Fx "$service_name" >/dev/null 2>&1; then
                echo "OK: $service_name"
            else
                echo "ERROR: missing service $service_name" >&2
                exit 1
            fi
        done
    fi
else
    echo "SKIP: takeoff/land service checks because mavros_msgs/srv/CommandTOL is not installed"
fi

if ros2 interface show mavros_msgs/srv/CommandLong >/dev/null 2>&1; then
    for service_name in \
        "/${VEHICLE_NAME}/mavros/cmd/command" \
        "/${MAVROS_NAMESPACE}/cmd/command"
    do
        if ros2 service list | grep -Fx "$service_name" >/dev/null 2>&1; then
            echo "OK: $service_name"
        else
            echo "ERROR: missing service $service_name" >&2
            exit 1
        fi
    done
    if [ "$ENABLE_ARDU_COMPAT" = "true" ]; then
        if ros2 service list | grep -Fx /mavros/cmd/command >/dev/null 2>&1; then
            echo "OK: /mavros/cmd/command"
        else
            echo "ERROR: missing service /mavros/cmd/command" >&2
            exit 1
        fi
    fi
else
    echo "SKIP: command service checks because mavros_msgs/srv/CommandLong is not installed"
fi

if [ "$CONTROL_BACKEND" = "px4_mavros" ]; then
    call_tol_service "$TAKEOFF_SERVICE" "takeoff"
    LAND_ON_EXIT=true
    sleep "$TAKEOFF_SETTLE_SEC"
    takeoff_pose="$(pose_xyz)"
    takeoff_mavros_pose="$(mavros_pose_xyz)"
    echo "Post-takeoff AirSim pose: $takeoff_pose"
    echo "Post-takeoff MAVROS pose: $takeoff_mavros_pose"

    if [ "$ENABLE_ARDU_COMPAT" = "true" ]; then
        echo "Running /ap pose/cmd_vel closed-loop probe..."
        ros2 run airsim_ros2_bridge ap_pose_cmdvel_probe --ros-args \
            -p target_dx:="$PROBE_TARGET_DX" \
            -p target_dy:="$PROBE_TARGET_DY" \
            -p target_dz:="$PROBE_TARGET_DZ" \
            -p max_speed:="$MOVE_SPEED_X" \
            -p tolerance:="$PROBE_TOLERANCE" \
            -p max_duration_sec:="$PROBE_MAX_DURATION_SEC"
    else
        echo "Publishing /${MAVROS_NAMESPACE}/setpoint_velocity/cmd_vel_unstamped..."
        publish_count="$(python3 - "$MOVE_DURATION_SEC" <<'PY'
import math
import sys

duration = float(sys.argv[1])
print(max(1, math.ceil(duration * 5.0)))
PY
)"
        ros2 topic pub --times "$publish_count" -r 5 "/${MAVROS_NAMESPACE}/setpoint_velocity/cmd_vel_unstamped" geometry_msgs/msg/Twist \
            "{linear: {x: ${MOVE_SPEED_X}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    fi

    after_forward_pose="$(pose_xyz)"
    after_forward_mavros_pose="$(mavros_pose_xyz)"
    echo "Post-forward AirSim pose: $after_forward_pose"
    echo "Post-forward MAVROS pose: $after_forward_mavros_pose"

    call_tol_service "$LAND_SERVICE" "land"
    LAND_ON_EXIT=false
    sleep "$LAND_SETTLE_SEC"
else
    if [ "$ENABLE_ARDU_COMPAT" = "true" ]; then
        command_topic="/ap/cmd_vel"
    else
        command_topic="/${MAVROS_NAMESPACE}/setpoint_velocity/cmd_vel_unstamped"
    fi
    echo "Publishing ${command_topic}..."
    publish_count="$(python3 - "$MOVE_DURATION_SEC" <<'PY'
import math
import sys

duration = float(sys.argv[1])
print(max(1, math.ceil(duration * 5.0)))
PY
)"
    ros2 topic pub --times "$publish_count" -r 5 "$command_topic" geometry_msgs/msg/Twist \
        "{linear: {x: ${MOVE_SPEED_X}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
fi

after_pose="$(pose_xyz)"
echo "Final AirSim pose: $after_pose"

if [ "$CONTROL_BACKEND" = "px4_mavros" ]; then
    after_mavros_pose="$(mavros_pose_xyz)"
    echo "Final MAVROS pose: $after_mavros_pose"
    before_check_pose="$takeoff_mavros_pose"
    after_check_pose="$after_forward_mavros_pose"
else
    before_check_pose="$before_pose"
    after_check_pose="$after_pose"
fi

python3 - "$before_check_pose" "$after_check_pose" "$MIN_MOVE_DELTA" <<'PY'
import math
import sys

before = [float(v) for v in sys.argv[1].split()]
after = [float(v) for v in sys.argv[2].split()]
minimum = float(sys.argv[3])
delta = math.sqrt(sum((b - a) ** 2 for b, a in zip(before, after)))
print(f"Pose delta: {delta:.3f} m")
if delta < minimum:
    raise SystemExit(f"ERROR: pose delta {delta:.3f} m is below MIN_MOVE_DELTA={minimum}")
PY

echo "Smoke test passed."
