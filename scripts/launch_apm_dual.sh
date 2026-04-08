#!/bin/bash
# Launch 2 ArduPilot SITL instances for Colosseum
# ArduPilot --instance N applies N*10 port offset
# Instance 0: sensor 9003, control 9002
# Instance 1: sensor 9013, control 9012
set -e

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"

if [ ! -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]; then
    echo "ERROR: ArduPilot not found at $ARDUPILOT_DIR"
    echo "Set ARDUPILOT_DIR environment variable or clone ArduPilot to ~/ardupilot"
    exit 1
fi

# Copy APM settings
mkdir -p ~/Documents/AirSim
cp /home/clrobur/airsim/settings/apm_dual.json ~/Documents/AirSim/settings.json
echo "Copied APM settings to ~/Documents/AirSim/settings.json"

echo "Starting ArduPilot SITL instance 0 (Sensor:9003, Control:9002)..."
cd "$ARDUPILOT_DIR"
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -f airsim-copter -I0 &>/tmp/apm_sitl_0.log &
PID0=$!
echo "  PID: $PID0, log: /tmp/apm_sitl_0.log"

sleep 5

echo "Starting ArduPilot SITL instance 1 (Sensor:9013, Control:9012)..."
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -f airsim-copter -I1 &>/tmp/apm_sitl_1.log &
PID1=$!
echo "  PID: $PID1, log: /tmp/apm_sitl_1.log"

echo ""
echo "Both ArduPilot SITL instances running."
echo "  Instance 0: PID=$PID0, Sensor:9003, Control:9002"
echo "  Instance 1: PID=$PID1, Sensor:9013, Control:9012"
echo ""
echo "Press Ctrl+C to stop both instances."

trap "kill $PID0 $PID1 2>/dev/null; echo 'Stopped.'; exit 0" SIGINT SIGTERM
wait
