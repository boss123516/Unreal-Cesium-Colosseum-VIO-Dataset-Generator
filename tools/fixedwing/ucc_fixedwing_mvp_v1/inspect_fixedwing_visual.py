#!/usr/bin/env python3
"""Print the imported RC Cessna static-mesh bounds for mount validation."""

import unreal


ASSET_PATH = "/Game/FixedWing/SM_RCCessna"


mesh = unreal.EditorAssetLibrary.load_asset(ASSET_PATH)
if mesh is None:
    raise RuntimeError(f"asset not found: {ASSET_PATH}")

bounds = mesh.get_bounding_box()
message = (
    "[FIXEDWING_VISUAL_BOUNDS] "
    f"min=({bounds.min.x},{bounds.min.y},{bounds.min.z}) "
    f"max=({bounds.max.x},{bounds.max.y},{bounds.max.z})"
)
unreal.log_warning(message)
