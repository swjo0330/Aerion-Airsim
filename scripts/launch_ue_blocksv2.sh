#!/bin/bash
# Launch Unreal Engine 5.6 with BlocksV2 + AirSim plugin
# Usage: ./launch_ue_blocksv2.sh [px4|apm]  (default: px4)
set -e

MODE="${1:-px4}"
UE_EDITOR="/home/clrobur/airsim/unreal-engine/Engine/Binaries/Linux/UnrealEditor"
BLOCKS_PROJECT="/home/clrobur/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject"
SETTINGS_DST="$HOME/Documents/AirSim/settings.json"

case "$MODE" in
    px4)
        SETTINGS_SRC="/home/clrobur/airsim/settings/px4_dual.json"
        ;;
    apm)
        SETTINGS_SRC="/home/clrobur/airsim/settings/apm_dual.json"
        ;;
    *)
        echo "Usage: $0 [px4|apm]"
        exit 1
        ;;
esac

mkdir -p "$HOME/Documents/AirSim"
cp "$SETTINGS_SRC" "$SETTINGS_DST"
echo "Copied $MODE settings to $SETTINGS_DST"

echo "Launching Unreal Engine with BlocksV2..."
"$UE_EDITOR" "$BLOCKS_PROJECT"
