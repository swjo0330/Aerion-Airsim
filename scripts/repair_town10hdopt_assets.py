#!/usr/bin/env python3
"""Repair known missing Town10HD_Opt material dependencies inside Unreal Editor.

Run with the CARLA UE project, for example:

  UnrealEditor CarlaUnreal.uproject -run=pythonscript -script=scripts/repair_town10hdopt_assets.py

The script uses UE asset tools so the copied assets get valid package names.
"""

from __future__ import annotations

import sys

import unreal


ASSET_COPIES = [
    (
        "/Engine/Functions/Engine_MaterialFunctions02/Utility/BreakOutFloat3Components",
        "/Game/Functions/Engine_MaterialFunctions02/Utility/BreakOutFloat3Components",
    ),
    (
        "/Engine/Functions/Engine_MaterialFunctions03/Blends/Blend_Overlay",
        "/Game/Functions/Engine_MaterialFunctions03/Blends/Blend_Overlay",
    ),
    (
        "/Engine/EngineMaterials/BlendFunc_DefBase",
        "/Game/EngineMaterials/BlendFunc_DefBase",
    ),
    (
        "/Engine/EngineMaterials/BlendFunc_DefBlend",
        "/Game/EngineMaterials/BlendFunc_DefBlend",
    ),
    (
        "/Game/Carla/Static/GenericMaterials/000_Masters/Textures/Noises/T_MacroVariation01",
        "/CarlaTools/Static/GenericMaterials/00_MastersOpt/Textures/T_MacroVariation",
    ),
]


def ensure_directory(asset_path: str) -> None:
    package_dir = asset_path.rsplit("/", 1)[0]
    if not unreal.EditorAssetLibrary.does_directory_exist(package_dir):
        unreal.EditorAssetLibrary.make_directory(package_dir)


def copy_asset(source: str, destination: str) -> bool:
    if not unreal.EditorAssetLibrary.does_asset_exist(source):
        unreal.log_error(f"[town10-repair] missing source asset: {source}")
        return False

    if unreal.EditorAssetLibrary.does_asset_exist(destination):
        unreal.log(f"[town10-repair] exists: {destination}")
        return True

    ensure_directory(destination)
    if not unreal.EditorAssetLibrary.duplicate_asset(source, destination):
        unreal.log_error(f"[town10-repair] duplicate failed: {source} -> {destination}")
        return False

    unreal.log(f"[town10-repair] repaired: {source} -> {destination}")
    return True


def save_repaired_assets() -> None:
    for _, destination in ASSET_COPIES:
        if unreal.EditorAssetLibrary.does_asset_exist(destination):
            unreal.EditorAssetLibrary.save_asset(destination, only_if_is_dirty=False)


def main() -> int:
    ok = True
    for source, destination in ASSET_COPIES:
        ok = copy_asset(source, destination) and ok

    save_repaired_assets()
    unreal.EditorAssetLibrary.save_directory("/Game/Functions", only_if_is_dirty=False, recursive=True)
    unreal.EditorAssetLibrary.save_directory("/Game/EngineMaterials", only_if_is_dirty=False, recursive=True)
    unreal.EditorAssetLibrary.save_directory(
        "/CarlaTools/Static/GenericMaterials/00_MastersOpt/Textures",
        only_if_is_dirty=False,
        recursive=True,
    )

    if ok:
        unreal.log("[town10-repair] done")
        return 0

    unreal.log_error("[town10-repair] completed with errors")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
