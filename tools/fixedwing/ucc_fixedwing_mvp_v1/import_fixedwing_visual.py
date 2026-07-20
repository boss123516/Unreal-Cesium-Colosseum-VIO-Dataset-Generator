#!/usr/bin/env python3
"""Import the PX4 RC Cessna body as a combined Unreal static mesh."""

from __future__ import annotations

import os
from pathlib import Path

import unreal


SOURCE_ENV = "UCC_FIXEDWING_FBX"
DESTINATION_PATH = "/Game/FixedWing"
DESTINATION_NAME = "SM_RCCessna"


def main() -> None:
    source_value = os.environ.get(SOURCE_ENV)
    if not source_value:
        raise RuntimeError(f"{SOURCE_ENV} is not set")
    source = Path(source_value).expanduser().resolve()
    if not source.is_file():
        raise RuntimeError(f"fixed-wing FBX not found: {source}")

    options = unreal.FbxImportUI()
    options.import_animations = False
    options.import_as_skeletal = False
    options.import_materials = True
    options.import_mesh = True
    options.import_textures = False
    options.mesh_type_to_import = unreal.FBXImportType.FBXIT_STATIC_MESH
    options.static_mesh_import_data.combine_meshes = True
    options.static_mesh_import_data.convert_scene = True
    options.static_mesh_import_data.convert_scene_unit = True
    options.static_mesh_import_data.force_front_x_axis = False
    options.static_mesh_import_data.generate_lightmap_u_vs = False
    options.static_mesh_import_data.import_translation = unreal.Vector(0.0, 0.0, 0.0)
    options.static_mesh_import_data.import_rotation = unreal.Rotator(0.0, 0.0, 0.0)
    options.static_mesh_import_data.import_uniform_scale = 1.0

    task = unreal.AssetImportTask()
    task.automated = True
    task.destination_name = DESTINATION_NAME
    task.destination_path = DESTINATION_PATH
    task.filename = str(source)
    task.options = options
    task.replace_existing = True
    task.replace_existing_settings = True
    task.save = True

    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    expected_path = f"{DESTINATION_PATH}/{DESTINATION_NAME}"
    asset = unreal.EditorAssetLibrary.load_asset(expected_path)
    if asset is None:
        raise RuntimeError(
            f"import did not produce {expected_path}; outputs={task.imported_object_paths}"
        )
    unreal.EditorAssetLibrary.save_loaded_asset(asset, only_if_is_dirty=False)
    print(f"[FIXEDWING_VISUAL_IMPORT] READY {expected_path}")


if __name__ == "__main__":
    main()
