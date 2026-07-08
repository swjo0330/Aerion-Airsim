#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
UE_BIN="${UE_BIN:-$HOME/workspace/engines/UE5.5_carla/UnrealEngine/Engine/Binaries/Linux/UnrealEditor}"
UE_PROJECT="${UE_PROJECT:-$HOME/workspace/engines/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject}"
UE_RENDER_PROFILE="${UE_RENDER_PROFILE:-town10_stable}"
UE_QUALITY_LEVEL="${UE_QUALITY_LEVEL:-High}"

export UE_BIN
export UE_PROJECT
export UE_RENDER_PROFILE
export UE_QUALITY_LEVEL

exec "$WORKSPACE/scripts/run_ue_blocksv2.sh"
