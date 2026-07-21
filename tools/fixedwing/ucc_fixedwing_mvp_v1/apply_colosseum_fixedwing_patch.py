#!/usr/bin/env python3
"""Apply the UCC fixed-wing runtime patch to an installed AirSim plugin."""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--project",
        type=Path,
        required=True,
        help="UCCVioDatasetSim project root containing Plugins/AirSim",
    )
    return parser.parse_args()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one source match, found {count}")
    return text.replace(old, new, 1)


def write_changed(path: Path, text: str) -> bool:
    current = path.read_text(encoding="utf-8")
    if current == text:
        return False
    path.write_text(text, encoding="utf-8")
    return True


def patch_physics_body(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    if "std::unique_lock<std::mutex> scopedLock()" in text:
        return False
    old = """        void unlock()
        {
            mutex_.unlock();
        }
"""
    new = old + """        std::unique_lock<std::mutex> scopedLock()
        {
            return std::unique_lock<std::mutex>(mutex_);
        }
"""
    return write_changed(path, replace_once(text, old, new, path.name))


def patch_external_physics(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    if "auto body_lock = body_ptr->scopedLock();" in text:
        return False
    old = """            for (PhysicsBody* body_ptr : *this) {
                body_ptr->updateKinematics();
                body_ptr->update();
"""
    new = """            for (PhysicsBody* body_ptr : *this) {
                auto body_lock = body_ptr->scopedLock();
                body_ptr->updateKinematics();
                body_ptr->update();
"""
    return write_changed(path, replace_once(text, old, new, path.name))


FIXED_WING_OBSERVER_BLOCK = r'''
    if (msr::airlib::AirSimSettings::singleton().physics_engine_name == "ExternalPhysicsEngine") {
        // Follow aircraft heading from behind while keeping the horizon stable.
        // Roll and pitch remain visible instead of being cancelled by an
        // identically rotating camera.
        SpringArm->bInheritPitch = false;
        SpringArm->bInheritYaw = true;
        SpringArm->bInheritRoll = false;
        SpringArm->bEnableCameraLag = false;
        UE_LOG(
            LogTemp,
            Display,
            TEXT("[FIXEDWING_OBSERVER] horizon-stable chase camera enabled"));
    }
'''


def patch_camera_manager(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    original = text
    if '#include "common/AirSimSettings.hpp"' not in text:
        text = replace_once(
            text,
            '#include "AirBlueprintLib.h"\n',
            '#include "AirBlueprintLib.h"\n#include "common/AirSimSettings.hpp"\n',
            f"{path.name} settings include",
        )
    if "[FIXEDWING_OBSERVER]" not in text:
        anchor = """    manual_pose_controller_->initializeForPlay();
"""
        text = replace_once(
            text,
            anchor,
            anchor + FIXED_WING_OBSERVER_BLOCK,
            f"{path.name} observer setup",
        )
    return write_changed(path, text) if text != original else False


def patch_flying_pawn_header(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    original = text
    if "class UStaticMeshComponent;" not in text:
        text = replace_once(
            text,
            '#include "FlyingPawn.generated.h"\n\n',
            '#include "FlyingPawn.generated.h"\n\nclass UStaticMeshComponent;\n\n',
            f"{path.name} forward declaration",
        )
    if "UStaticMeshComponent* fixed_wing_visual_" not in text:
        text = replace_once(
            text,
            """private: //variables
    //Unreal components
    UPROPERTY()
    APIPCamera* camera_front_left_;
""",
            """private: //variables
    //Unreal components
    UPROPERTY()
    UStaticMeshComponent* fixed_wing_visual_ = nullptr;

    UPROPERTY()
    APIPCamera* camera_front_left_;
""",
            f"{path.name} visual member",
        )
    return write_changed(path, text) if text != original else False


FIXED_WING_BEGIN_PLAY = r'''void AFlyingPawn::BeginPlay()
{
    Super::BeginPlay();

    if (msr::airlib::AirSimSettings::singleton().physics_engine_name == "ExternalPhysicsEngine") {
        static const TCHAR* quad_mesh_components[] = {
            TEXT("BodyMesh"),
            TEXT("Prop0"),
            TEXT("Prop1"),
            TEXT("Prop2"),
            TEXT("Prop3"),
        };
        for (const TCHAR* component_name : quad_mesh_components) {
            if (auto* component = UAirBlueprintLib::GetActorComponent<UStaticMeshComponent>(this, component_name)) {
                component->SetVisibility(false, false);
                component->SetHiddenInGame(true, false);
            }
        }

        const FString visual_setting =
            FPlatformMisc::GetEnvironmentVariable(TEXT("UCC_FIXEDWING_VISUAL"));
        UStaticMesh* fixed_wing_mesh = visual_setting == TEXT("0")
            ? nullptr
            : LoadObject<UStaticMesh>(
                  nullptr,
                  TEXT("/Game/FixedWing/SM_RCCessna.SM_RCCessna"));
        if (fixed_wing_mesh != nullptr && GetRootComponent() != nullptr) {
            fixed_wing_visual_ = NewObject<UStaticMeshComponent>(this, TEXT("FixedWingVisual"));
            fixed_wing_visual_->SetStaticMesh(fixed_wing_mesh);
            fixed_wing_visual_->SetupAttachment(GetRootComponent());
            fixed_wing_visual_->SetAbsolute(false, false, true);
            fixed_wing_visual_->SetRelativeLocation(FVector(7.0f, 0.0f, -8.0f));
            // The imported FBX already uses +X as the aircraft nose direction.
            // Remap only its Y-up / Z-lateral axes into Unreal Z-up / Y-right.
            fixed_wing_visual_->SetRelativeRotation(FRotator(0.0f, 0.0f, 90.0f));
            fixed_wing_visual_->SetRelativeScale3D(FVector(0.1f));
            fixed_wing_visual_->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            fixed_wing_visual_->SetHiddenInSceneCapture(true);
            AddInstanceComponent(fixed_wing_visual_);
            fixed_wing_visual_->RegisterComponent();
            UE_LOG(
                LogTemp,
                Display,
                TEXT("[FIXEDWING_VISUAL] READY root_scale=%s visual_scale=%s extent_cm=%s"),
                *GetRootComponent()->GetComponentScale().ToString(),
                *fixed_wing_visual_->GetComponentScale().ToString(),
                *fixed_wing_visual_->Bounds.BoxExtent.ToString());
        }
        else if (visual_setting == TEXT("0")) {
            UE_LOG(
                LogTemp,
                Display,
                TEXT("[FIXEDWING_VISUAL] disabled by UCC_FIXEDWING_VISUAL=0"));
        }
        else {
            UAirBlueprintLib::LogMessage(
                TEXT("Fixed-wing visual unavailable:"),
                TEXT("/Game/FixedWing/SM_RCCessna"),
                LogDebugLevel::Failure);
        }
    }
}
'''


def patch_flying_pawn_source(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    original = text
    if '#include "Components/StaticMeshComponent.h"' not in text:
        text = replace_once(
            text,
            '#include "FlyingPawn.h"\n',
            '#include "FlyingPawn.h"\n#include "Components/StaticMeshComponent.h"\n',
            f"{path.name} mesh include",
        )
    if '#include "common/AirSimSettings.hpp"' not in text:
        text = replace_once(
            text,
            '#include "AirBlueprintLib.h"\n',
            '#include "AirBlueprintLib.h"\n#include "common/AirSimSettings.hpp"\n',
            f"{path.name} settings include",
        )
    if "[FIXEDWING_VISUAL] READY" not in text:
        text = replace_once(
            text,
            """void AFlyingPawn::BeginPlay()
{
    Super::BeginPlay();
}
""",
            FIXED_WING_BEGIN_PLAY,
            f"{path.name} BeginPlay",
        )
    if "    fixed_wing_visual_ = nullptr;" not in text:
        text = replace_once(
            text,
            """void AFlyingPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
""",
            """void AFlyingPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    fixed_wing_visual_ = nullptr;
""",
            f"{path.name} EndPlay",
        )
    legacy_visual_rotation = (
        "fixed_wing_visual_->SetRelativeRotation("
        "FRotator(0.0f, 180.0f, 90.0f));"
    )
    corrected_visual_rotation = (
        "fixed_wing_visual_->SetRelativeRotation("
        "FRotator(0.0f, 0.0f, 90.0f));"
    )
    if legacy_visual_rotation in text:
        text = replace_once(
            text,
            legacy_visual_rotation,
            corrected_visual_rotation,
            f"{path.name} fixed-wing nose direction",
        )
        text = text.replace(
            "// Imported FBX axes are -X forward, +Z right, +Y up. Rotate them\n"
            "            // into AirSim / Unreal body axes: +X forward, +Y right, +Z up.",
            "// The imported FBX already uses +X as the aircraft nose direction.\n"
            "            // Remap only its Y-up / Z-lateral axes into Unreal Z-up / Y-right.",
            1,
        )
    if corrected_visual_rotation not in text:
        raise RuntimeError(f"{path.name}: fixed-wing visual rotation is missing")
    return write_changed(path, text) if text != original else False


CAMERA_SETUP_FUNCTION = r'''void PawnSimApi::setupCamerasFromSettings(const common_utils::UniqueValueMap<std::string, APIPCamera*>& cameras)
{
    //add cameras that already exists in pawn
    cameras_.clear();
    for (const auto& p : cameras.getMap())
        cameras_.insert_or_assign(p.first, p.second);

    //create or replace cameras specified in settings
    createCamerasFromSettings();

    // Configure explicitly named cameras first, then apply defaults once to
    // every remaining physical camera. Several AirSim aliases can point to the
    // same APIPCamera, so configuring by alias would otherwise overwrite the
    // fixed-wing cam0 settings with a later default alias.
    const auto& camera_defaults = AirSimSettings::singleton().camera_defaults;
    const bool is_external_physics =
        AirSimSettings::singleton().physics_engine_name == "ExternalPhysicsEngine";
    std::set<APIPCamera*> configured_cameras;

    for (const auto& pair : getVehicleSetting()->cameras) {
        APIPCamera* camera = cameras_.findOrDefault(pair.first, nullptr);
        if (camera == nullptr)
            continue;

        const auto& camera_setting = pair.second;
        camera->setupCameraFromSettings(camera_setting, getNedTransform());

        if (is_external_physics && pair.first == "cam0") {
            Pose camera_pose = camera->getPose();
            if (!std::isnan(camera_setting.position.x()))
                camera_pose.position.x() = camera_setting.position.x();
            if (!std::isnan(camera_setting.position.y()))
                camera_pose.position.y() = camera_setting.position.y();
            if (!std::isnan(camera_setting.position.z()))
                camera_pose.position.z() = camera_setting.position.z();

            real_T pitch, roll, yaw;
            VectorMath::toEulerianAngle(camera_pose.orientation, pitch, roll, yaw);
            if (!std::isnan(camera_setting.rotation.pitch))
                pitch = Utils::degreesToRadians(camera_setting.rotation.pitch);
            if (!std::isnan(camera_setting.rotation.roll))
                roll = Utils::degreesToRadians(camera_setting.rotation.roll);
            if (!std::isnan(camera_setting.rotation.yaw))
                yaw = Utils::degreesToRadians(camera_setting.rotation.yaw);
            camera_pose.orientation = VectorMath::toQuaternion(pitch, roll, yaw);
            camera->setCameraPose(camera_pose);
        }

        configured_cameras.insert(camera);
    }

    for (APIPCamera* camera : cameras_) {
        if (camera != nullptr && configured_cameras.count(camera) == 0)
            camera->setupCameraFromSettings(camera_defaults, getNedTransform());
    }

    // Keep the carrier body out of fixed-wing sensor images without hiding the
    // visual from the normal player/chase camera.
    if (is_external_physics) {
        for (APIPCamera* camera : cameras_) {
            const int image_count = static_cast<int>(Utils::toNumeric(APIPCamera::ImageType::Count));
            for (int image_type = 0; image_type < image_count; ++image_type) {
                if (USceneCaptureComponent2D* capture = camera->getCaptureComponent(
                        static_cast<APIPCamera::ImageType>(image_type), false)) {
                    capture->HideActorComponents(params_.pawn);
                }
            }
        }
    }
}

'''


CAMERA_REUSE_BLOCK = r'''
        // Runtime-spawned BP_PIPCamera instances do not render the Cesium Scene
        // pass reliably in this project. Reuse the proven front-center camera
        // for the fixed-wing dataset alias, then apply cam0's own pose and
        // capture settings in setupCamerasFromSettings().
        if (AirSimSettings::singleton().physics_engine_name == "ExternalPhysicsEngine" &&
            camera_setting_pair.first == "cam0") {
            APIPCamera* camera = cameras_.findOrDefault("front_center", nullptr);
            if (camera != nullptr) {
                camera->AttachToComponent(bodyMesh, FAttachmentTransformRules::KeepWorldTransform);
                cameras_.insert_or_assign(camera_setting_pair.first, camera);
                UE_LOG(
                    LogTemp,
                    Display,
                    TEXT("[FIXEDWING_CAMERA] cam0 reuses the built-in front-center capture"));
                continue;
            }
        }
'''


def patch_pawn_sim_api(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    original = text
    if "#include <set>" not in text:
        text = replace_once(
            text,
            '#include "DrawDebugHelpers.h"\n',
            '#include "DrawDebugHelpers.h"\n\n#include <set>\n',
            f"{path.name} set include",
        )
    if "std::set<APIPCamera*> configured_cameras;" not in text:
        start = text.index("void PawnSimApi::setupCamerasFromSettings(")
        end = text.index("void PawnSimApi::createCamerasFromSettings()", start)
        text = text[:start] + CAMERA_SETUP_FUNCTION + text[end:]
    if "[FIXEDWING_CAMERA] cam0 reuses" not in text:
        anchor = """        const auto& setting = camera_setting_pair.second;
"""
        text = replace_once(
            text,
            anchor,
            anchor + CAMERA_REUSE_BLOCK,
            f"{path.name} camera reuse",
        )
    return write_changed(path, text) if text != original else False


def main() -> int:
    project = args.project.expanduser().resolve()
    source = project / "Plugins" / "AirSim" / "Source"
    targets = {
        source / "AirLib/include/physics/PhysicsBody.hpp": patch_physics_body,
        source / "AirLib/include/physics/ExternalPhysicsEngine.hpp": patch_external_physics,
        source / "CameraManager.cpp": patch_camera_manager,
        source / "Vehicles/Multirotor/FlyingPawn.h": patch_flying_pawn_header,
        source / "Vehicles/Multirotor/FlyingPawn.cpp": patch_flying_pawn_source,
        source / "PawnSimApi.cpp": patch_pawn_sim_api,
    }
    missing = [str(path) for path in targets if not path.is_file()]
    if missing:
        raise SystemExit("[ERROR] AirSim plugin files missing:\n" + "\n".join(missing))

    changed = []
    for path, patcher in targets.items():
        if patcher(path):
            changed.append(path.relative_to(project).as_posix())

    if changed:
        for relative in changed:
            print(f"[PATCHED] {relative}")
    else:
        print("[OK] Colosseum fixed-wing patch already applied")
    return 0


if __name__ == "__main__":
    args = parse_args()
    raise SystemExit(main())
