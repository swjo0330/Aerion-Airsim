#!/usr/bin/env bash
# Launch two MAVROS instances against local PX4 SITL UDP endpoints.
set -euo pipefail

DRONE0_FCU_URL="${DRONE0_FCU_URL:-udp://:14540@127.0.0.1:14580}"
DRONE1_FCU_URL="${DRONE1_FCU_URL:-udp://:14541@127.0.0.1:14581}"
DRONE0_NAMESPACE="${DRONE0_NAMESPACE:-mavros0}"
DRONE1_NAMESPACE="${DRONE1_NAMESPACE:-mavros1}"

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

echo "Starting MAVROS for Drone0 ($DRONE0_NAMESPACE, $DRONE0_FCU_URL)..."
ros2 launch mavros px4.launch \
    fcu_url:="$DRONE0_FCU_URL" \
    tgt_system:=1 \
    namespace:="$DRONE0_NAMESPACE" \
    >"/tmp/mavros_drone0_px4.log" 2>&1 &
PID0=$!
echo "  PID: $PID0, log: /tmp/mavros_drone0_px4.log"

sleep 2

echo "Starting MAVROS for Drone1 ($DRONE1_NAMESPACE, $DRONE1_FCU_URL)..."
ros2 launch mavros px4.launch \
    fcu_url:="$DRONE1_FCU_URL" \
    tgt_system:=2 \
    namespace:="$DRONE1_NAMESPACE" \
    >"/tmp/mavros_drone1_px4.log" 2>&1 &
PID1=$!
echo "  PID: $PID1, log: /tmp/mavros_drone1_px4.log"

echo ""
echo "Both MAVROS instances running."
echo "  Drone0 topics: /$DRONE0_NAMESPACE/..."
echo "  Drone1 topics: /$DRONE1_NAMESPACE/..."
echo ""
echo "Press Ctrl+C to stop both instances."

trap "kill $PID0 $PID1 2>/dev/null; echo 'Stopped.'; exit 0" SIGINT SIGTERM
wait
