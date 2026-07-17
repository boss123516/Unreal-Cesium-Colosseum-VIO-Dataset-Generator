#include "CesiumAirSimCameraBridgeSubsystem.h"

#include "Cesium3DTileset.h"
#include "CesiumCamera.h"
#include "CesiumCameraManager.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "PIPCamera.h"

namespace
{
constexpr int32 PreferredWidth = 640;
constexpr int32 PreferredHeight = 480;
constexpr int64 TwoGiB = 2LL * 1024LL * 1024LL * 1024LL;

FIntPoint GetCaptureSize(const USceneCaptureComponent2D* Capture)
{
    if (Capture && Capture->TextureTarget)
    {
        return FIntPoint(
            FMath::Max(1, Capture->TextureTarget->SizeX),
            FMath::Max(1, Capture->TextureTarget->SizeY));
    }

    return FIntPoint(PreferredWidth, PreferredHeight);
}
} // namespace

void UCesiumAirSimCameraBridgeSubsystem::Initialize(
    FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    UE_LOG(
        LogTemp,
        Display,
        TEXT("[CESIUM_CAM0_BRIDGE] subsystem initialized; waiting for AirSim cam0."));
}

void UCesiumAirSimCameraBridgeSubsystem::Deinitialize()
{
    RemoveBridgeCameras();

    SensorCapture.Reset();
    SensorCameraActor.Reset();
    CameraManager.Reset();

    UE_LOG(
        LogTemp,
        Display,
        TEXT("[CESIUM_CAM0_BRIDGE] subsystem deinitialized."));

    Super::Deinitialize();
}

bool UCesiumAirSimCameraBridgeSubsystem::DoesSupportWorldType(
    EWorldType::Type WorldType) const
{
    return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

TStatId UCesiumAirSimCameraBridgeSubsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(
        UCesiumAirSimCameraBridgeSubsystem,
        STATGROUP_Tickables);
}

void UCesiumAirSimCameraBridgeSubsystem::Tick(float DeltaTime)
{
    UWorld* World = GetWorld();
    if (!World || World->bIsTearingDown)
    {
        return;
    }

    const double Now = World->GetTimeSeconds();

    const bool bNeedDiscovery =
        !CameraManager.IsValid() ||
        !SensorCapture.IsValid() ||
        BridgeCameraStartIndex == INDEX_NONE;

    if (bNeedDiscovery && Now - LastDiscoverySeconds >= DiscoveryPeriodSeconds)
    {
        LastDiscoverySeconds = Now;
        DiscoverCameraAndManager();
    }

    if (Now - LastTilesetTuneSeconds >= TilesetTunePeriodSeconds)
    {
        LastTilesetTuneSeconds = Now;
        TuneTilesets();
    }

    UpdateBridgeCameras();
}

int32 UCesiumAirSimCameraBridgeSubsystem::ScoreCamera(
    APIPCamera* Camera,
    USceneCaptureComponent2D* Capture) const
{
    if (!Camera || !Capture)
    {
        return MIN_int32;
    }

    const FString CombinedName = FString::Printf(
        TEXT("%s %s %s"),
        *Camera->GetName(),
        *Capture->GetName(),
        Camera->GetAttachParentActor()
            ? *Camera->GetAttachParentActor()->GetName()
            : TEXT(""))
                                     .ToLower();

    if (CombinedName.Contains(TEXT("external")))
    {
        return MIN_int32;
    }

    int32 Score = 0;

    if (Camera->GetAttachParentActor())
    {
        Score += 250;
    }

    if (CombinedName.Contains(TEXT("drone1")))
    {
        Score += 100;
    }

    if (CombinedName.Contains(TEXT("cam0")))
    {
        Score += 1500;
    }

    const FIntPoint Size = GetCaptureSize(Capture);
    if (Size.X == PreferredWidth && Size.Y == PreferredHeight)
    {
        Score += 1200;
    }

    const float Pitch =
        FRotator::NormalizeAxis(Capture->GetComponentRotation().Pitch);

    // Dataset cam0 is expected to be downward-facing by roughly 45 degrees.
    if (Pitch <= -10.0f && Pitch >= -85.0f)
    {
        Score += 350;
    }

    if (Capture->FOVAngle >= 20.0f && Capture->FOVAngle <= 140.0f)
    {
        Score += 20;
    }

    return Score;
}

void UCesiumAirSimCameraBridgeSubsystem::DiscoverCameraAndManager()
{
    UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    ACesiumCameraManager* NewManager =
        ACesiumCameraManager::GetDefaultCameraManager(World);

    if (!NewManager)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[CESIUM_CAM0_BRIDGE] CesiumCameraManager is not available yet."));
        return;
    }

    APIPCamera* BestActor = nullptr;
    USceneCaptureComponent2D* BestCapture = nullptr;
    int32 BestScore = MIN_int32;

    for (TActorIterator<APIPCamera> It(World); It; ++It)
    {
        APIPCamera* Candidate = *It;
        if (!IsValid(Candidate))
        {
            continue;
        }

        USceneCaptureComponent2D* Capture =
            Candidate->getCaptureComponent(APIPCamera::ImageType::Scene, false);

        const int32 Score = ScoreCamera(Candidate, Capture);
        if (Score > BestScore)
        {
            BestScore = Score;
            BestActor = Candidate;
            BestCapture = Capture;
        }
    }

    if (!BestActor || !BestCapture)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[CESIUM_CAM0_BRIDGE] no eligible AirSim APIPCamera found yet."));
        return;
    }

    const bool bSelectionChanged =
        CameraManager.Get() != NewManager ||
        SensorCameraActor.Get() != BestActor ||
        SensorCapture.Get() != BestCapture;

    if (!bSelectionChanged && BridgeCameraStartIndex != INDEX_NONE)
    {
        return;
    }

    RemoveBridgeCameras();

    CameraManager = NewManager;
    SensorCameraActor = BestActor;
    SensorCapture = BestCapture;

    RegisterBridgeCameras();

    const FIntPoint Size = GetCaptureSize(BestCapture);
    const AActor* Parent = BestActor->GetAttachParentActor();

    UE_LOG(
        LogTemp,
        Display,
        TEXT(
            "[CESIUM_CAM0_BRIDGE] selected actor=%s capture=%s parent=%s "
            "size=%dx%d fov=%.2f score=%d"),
        *BestActor->GetName(),
        *BestCapture->GetName(),
        Parent ? *Parent->GetName() : TEXT("<none>"),
        Size.X,
        Size.Y,
        BestCapture->FOVAngle,
        BestScore);
}

void UCesiumAirSimCameraBridgeSubsystem::RegisterBridgeCameras()
{
    ACesiumCameraManager* Manager = CameraManager.Get();
    if (!Manager || !SensorCapture.IsValid())
    {
        return;
    }

    Manager->UsePlayerCameras = true;

    BridgeCameraStartIndex = Manager->AdditionalCameras.Num();
    BridgeCameraCount = 2;
    Manager->AdditionalCameras.AddDefaulted(BridgeCameraCount);

    UpdateBridgeCameras();
}

void UCesiumAirSimCameraBridgeSubsystem::RemoveBridgeCameras()
{
    ACesiumCameraManager* Manager = CameraManager.Get();

    if (Manager &&
        BridgeCameraStartIndex != INDEX_NONE &&
        BridgeCameraCount > 0 &&
        Manager->AdditionalCameras.IsValidIndex(BridgeCameraStartIndex) &&
        Manager->AdditionalCameras.Num() >=
            BridgeCameraStartIndex + BridgeCameraCount)
    {
        Manager->AdditionalCameras.RemoveAt(
            BridgeCameraStartIndex,
            BridgeCameraCount,
            EAllowShrinking::No);
    }

    BridgeCameraStartIndex = INDEX_NONE;
    BridgeCameraCount = 0;
    bLoggedReady = false;
}

void UCesiumAirSimCameraBridgeSubsystem::UpdateBridgeCameras()
{
    ACesiumCameraManager* Manager = CameraManager.Get();
    USceneCaptureComponent2D* Capture = SensorCapture.Get();

    if (!Manager || !Capture ||
        BridgeCameraStartIndex == INDEX_NONE ||
        BridgeCameraCount != 2 ||
        Manager->AdditionalCameras.Num() <
            BridgeCameraStartIndex + BridgeCameraCount)
    {
        return;
    }

    const FIntPoint Size = GetCaptureSize(Capture);
    const FVector2D ViewportSize(
        static_cast<double>(Size.X),
        static_cast<double>(Size.Y));

    const FVector Location = Capture->GetComponentLocation();
    const FRotator Rotation = Capture->GetComponentRotation();
    const double Fov = FMath::Clamp(
        static_cast<double>(Capture->FOVAngle),
        20.0,
        140.0);

    const FVector LookAheadLocation =
        Location + Rotation.Vector() * LookAheadDistanceCm;

    Manager->AdditionalCameras[BridgeCameraStartIndex] =
        FCesiumCamera(
            ViewportSize,
            Location,
            Rotation,
            Fov);

    Manager->AdditionalCameras[BridgeCameraStartIndex + 1] =
        FCesiumCamera(
            ViewportSize,
            LookAheadLocation,
            Rotation,
            FMath::Max(Fov, LookAheadFovDegrees));

    if (!bLoggedReady)
    {
        bLoggedReady = true;

        UE_LOG(
            LogTemp,
            Display,
            TEXT(
                "[CESIUM_CAM0_BRIDGE] READY: current cam0 + 90m look-ahead "
                "camera are feeding Cesium tile selection."));
    }
}

void UCesiumAirSimCameraBridgeSubsystem::TuneTilesets()
{
    UWorld* World = GetWorld();
    ACesiumCameraManager* Manager = CameraManager.Get();

    if (!World)
    {
        return;
    }

    for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
    {
        ACesium3DTileset* Tileset = *It;
        if (!IsValid(Tileset))
        {
            continue;
        }

        Tileset->PreloadAncestors = true;
        Tileset->PreloadSiblings = true;
        Tileset->ForbidHoles = true;

        Tileset->MaximumScreenSpaceError = 16.0;
        Tileset->MaximumSimultaneousTileLoads = 64;
        Tileset->MaximumCachedBytes = TwoGiB;
        Tileset->LoadingDescendantLimit = 20;

        // Once cam0 is explicitly registered, frustum culling can stay enabled.
        Tileset->EnableFrustumCulling = true;
        Tileset->EnableFogCulling = false;
        Tileset->EnableOcclusionCulling = false;

        if (Manager && Tileset->ResolveCameraManager() != Manager)
        {
            Tileset->SetCameraManager(
                TSoftObjectPtr<ACesiumCameraManager>(Manager));
            Tileset->InvalidateResolvedCameraManager();
            Tileset->ResolveCameraManager();
        }
    }
}
