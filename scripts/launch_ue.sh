#!/bin/bash
# Launch Unreal Engine with AirSim project
# Usage: ./launch_ue.sh [blocks|city] [px4|apm]  (default: blocks px4)
set -e

ENV="${1:-blocks}"
MODE="${2:-px4}"
UE_EDITOR="/home/clrobur/airsim/unreal-engine/Engine/Binaries/Linux/UnrealEditor"
SETTINGS_DST="$HOME/Documents/AirSim/settings.json"

case "$ENV" in
    blocks)
        PROJECT="/home/clrobur/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject"
        ;;
    city)
        PROJECT="/home/clrobur/airsim/Colosseum/Unreal/Environments/CityEnv/CityEnv.uproject"
        ;;
    *)
        echo "Usage: $0 [blocks|city] [px4|apm]"
        exit 1
        ;;
esac

# 설정 생성 + 배포
cd /home/clrobur/airsim
python3 scripts/generate_settings.py -f "$MODE" -n 2 --deploy

echo "Launching $ENV ($MODE mode)..."
"$UE_EDITOR" "$PROJECT"
