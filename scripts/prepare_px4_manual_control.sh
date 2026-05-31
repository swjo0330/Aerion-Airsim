#!/usr/bin/env bash
# Prepare PX4/MAVROS instances for manual velocity control in CARLA/AirSim.
set -euo pipefail

ROS_WS="${ROS_WS:-$HOME/workspace/projects/aerion-airsim}"
DRONE_COUNT="${DRONE_COUNT:-3}"
SETPOINT_RATE="${SETPOINT_RATE:-20}"
SETPOINT_WARMUP_SEC="${SETPOINT_WARMUP_SEC:-3}"
ARM_AFTER_PREP="${ARM_AFTER_PREP:-true}"

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

set_int_param() {
    local namespace="$1"
    local param="$2"
    local value="$3"

    timeout 5 ros2 service call "/${namespace}/param/set" mavros_msgs/srv/ParamSetV2 \
        "{force_set: true, param_id: '${param}', value: {type: 2, integer_value: ${value}}}" \
        >/tmp/"${namespace//\//_}_${param}.log" 2>&1 || true
}

set_double_param() {
    local namespace="$1"
    local param="$2"
    local value="$3"

    timeout 5 ros2 service call "/${namespace}/param/set" mavros_msgs/srv/ParamSetV2 \
        "{force_set: true, param_id: '${param}', value: {type: 3, double_value: ${value}}}" \
        >/tmp/"${namespace//\//_}_${param}.log" 2>&1 || true
}

warmup_setpoints() {
    local vehicle="$1"
    local duration="$2"

    timeout "$duration" ros2 topic pub --rate "$SETPOINT_RATE" \
        "/${vehicle}/mavros/setpoint_velocity/cmd_vel_unstamped" \
        geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        >/tmp/"${vehicle}_manual_warmup.log" 2>&1 &
}

for ((i = 1; i <= DRONE_COUNT; i++)); do
    vehicle="drone${i}"
    namespace="${vehicle}/mavros"
    echo "Preparing ${vehicle} PX4 params..."

    set_int_param "$namespace" COM_ARM_WO_GPS 1
    set_int_param "$namespace" CBRK_SUPPLY_CHK 894281
    set_int_param "$namespace" CBRK_USB_CHK 197848
    set_int_param "$namespace" CBRK_IO_SAFETY 22027
    set_int_param "$namespace" EKF2_MAG_TYPE 1
    set_int_param "$namespace" COM_PREARM_MODE 0
    set_int_param "$namespace" NAV_RCL_ACT 0
    set_int_param "$namespace" NAV_DLL_ACT 0
    set_int_param "$namespace" COM_RC_IN_MODE 4

    # Keep PX4 from disarming while the operator is settling into manual teleop.
    set_double_param "$namespace" COM_DISARM_PRFLT -1.0
    set_double_param "$namespace" COM_DISARM_LAND -1.0
    set_double_param "$namespace" COM_OF_LOSS_T 5.0
done

echo "Streaming neutral setpoints for ${SETPOINT_WARMUP_SEC}s..."
for ((i = 1; i <= DRONE_COUNT; i++)); do
    warmup_setpoints "drone${i}" "$SETPOINT_WARMUP_SEC"
done
wait

if [ "$ARM_AFTER_PREP" = "true" ]; then
    for ((i = 1; i <= DRONE_COUNT; i++)); do
        vehicle="drone${i}"
        namespace="${vehicle}/mavros"
        echo "Arming ${vehicle} in OFFBOARD..."
        ros2 service call "/${namespace}/set_mode" mavros_msgs/srv/SetMode \
            "{base_mode: 0, custom_mode: 'OFFBOARD'}" >/tmp/"${vehicle}_manual_mode.log" 2>&1 || true
        ros2 service call "/${namespace}/cmd/arming" mavros_msgs/srv/CommandBool \
            "{value: true}" >/tmp/"${vehicle}_manual_arm.log" 2>&1 || true
    done
fi

for ((i = 1; i <= DRONE_COUNT; i++)); do
    vehicle="drone${i}"
    echo "State ${vehicle}:"
    ros2 topic echo --once "/${vehicle}/mavros/state" || true
done
