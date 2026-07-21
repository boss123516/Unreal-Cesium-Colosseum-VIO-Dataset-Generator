#!/usr/bin/env python3

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import apply_colosseum_fixedwing_patch as patcher


LEGACY_ROTATION = (
    "fixed_wing_visual_->SetRelativeRotation("
    "FRotator(0.0f, 180.0f, 90.0f));"
)
CORRECTED_ROTATION = (
    "fixed_wing_visual_->SetRelativeRotation("
    "FRotator(0.0f, 0.0f, 90.0f));"
)


class ColosseumFixedWingPatchTests(unittest.TestCase):
    def test_new_patch_uses_body_positive_x_as_nose(self) -> None:
        self.assertIn(CORRECTED_ROTATION, patcher.FIXED_WING_BEGIN_PLAY)
        self.assertNotIn(LEGACY_ROTATION, patcher.FIXED_WING_BEGIN_PLAY)

    def test_existing_install_is_migrated_and_then_idempotent(self) -> None:
        source = f'''#include "FlyingPawn.h"
#include "Components/StaticMeshComponent.h"
#include "common/AirSimSettings.hpp"

void AFlyingPawn::BeginPlay()
{{
    // [FIXEDWING_VISUAL] READY
    {LEGACY_ROTATION}
}}

void AFlyingPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{{
    fixed_wing_visual_ = nullptr;
}}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "FlyingPawn.cpp"
            path.write_text(source, encoding="utf-8")

            self.assertTrue(patcher.patch_flying_pawn_source(path))
            migrated = path.read_text(encoding="utf-8")
            self.assertIn(CORRECTED_ROTATION, migrated)
            self.assertNotIn(LEGACY_ROTATION, migrated)
            self.assertFalse(patcher.patch_flying_pawn_source(path))


if __name__ == "__main__":
    unittest.main()
