#include "KauBuildingsTilesetSubsystem.h"

#include "Cesium3DTileset.h"
#include "CesiumGeoreference.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Misc/CommandLine.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Parse.h"

namespace
{
const FName KauTilesetTag(TEXT("KAU_LOD1_BUILDINGS"));
const FName KauTilesetActorName(TEXT("KAU_Buildings_5km"));
constexpr int64 OneGiB = 1024LL * 1024LL * 1024LL;
} // namespace

void UKauBuildingsTilesetSubsystem::Initialize(
    FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    GConfig->GetBool(
        TEXT("KAUBuildings"),
        TEXT("Enabled"),
        bEnabled,
        GGameIni);
    GConfig->GetString(
        TEXT("KAUBuildings"),
        TEXT("TilesetUrl"),
        TilesetUrl,
        GGameIni);
    GConfig->GetString(
        TEXT("KAUBuildings"),
        TEXT("RequiredMapName"),
        RequiredMapName,
        GGameIni);
    GConfig->GetDouble(
        TEXT("KAUBuildings"),
        TEXT("MaximumScreenSpaceError"),
        MaximumScreenSpaceError,
        GGameIni);
    GConfig->GetBool(
        TEXT("KAUBuildings"),
        TEXT("RecenterGeoreference"),
        bRecenterGeoreference,
        GGameIni);
    GConfig->GetBool(
        TEXT("KAUBuildings"),
        TEXT("EnableCollision"),
        bEnableCollision,
        GGameIni);
    GConfig->GetDouble(
        TEXT("KAUBuildings"),
        TEXT("OriginLongitude"),
        OriginLongitude,
        GGameIni);
    GConfig->GetDouble(
        TEXT("KAUBuildings"),
        TEXT("OriginLatitude"),
        OriginLatitude,
        GGameIni);
    GConfig->GetDouble(
        TEXT("KAUBuildings"),
        TEXT("OriginHeight"),
        OriginHeight,
        GGameIni);

    if (FParse::Param(
            FCommandLine::Get(),
            TEXT("KAUNoBuildingCollision")))
    {
        bEnableCollision = false;
    }
    else if (FParse::Param(
                 FCommandLine::Get(),
                 TEXT("KAUBuildingCollision")))
    {
        bEnableCollision = true;
    }

    TilesetUrl.TrimStartAndEndInline();
    RequiredMapName.TrimStartAndEndInline();
    MaximumScreenSpaceError =
        FMath::Clamp(MaximumScreenSpaceError, 1.0, 128.0);
    OriginLongitude = FMath::Clamp(OriginLongitude, -180.0, 180.0);
    OriginLatitude = FMath::Clamp(OriginLatitude, -90.0, 90.0);

    UE_LOG(
        LogTemp,
        Display,
        TEXT(
            "[KAU_BUILDINGS] subsystem initialized enabled=%s map=%s url=%s "
            "recenter=%s origin=(%.8f,%.8f,%.3f) collision=%s"),
        bEnabled ? TEXT("true") : TEXT("false"),
        RequiredMapName.IsEmpty() ? TEXT("<any>") : *RequiredMapName,
        TilesetUrl.IsEmpty() ? TEXT("<empty>") : *TilesetUrl,
        bRecenterGeoreference ? TEXT("true") : TEXT("false"),
        OriginLongitude,
        OriginLatitude,
        OriginHeight,
        bEnableCollision ? TEXT("true") : TEXT("false"));
}

void UKauBuildingsTilesetSubsystem::Deinitialize()
{
    BuildingsTileset.Reset();
    Super::Deinitialize();
}

bool UKauBuildingsTilesetSubsystem::DoesSupportWorldType(
    EWorldType::Type WorldType) const
{
    return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

TStatId UKauBuildingsTilesetSubsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(
        UKauBuildingsTilesetSubsystem,
        STATGROUP_Tickables);
}

void UKauBuildingsTilesetSubsystem::Tick(float DeltaTime)
{
    if (!bEnabled)
    {
        return;
    }

    UWorld* World = GetWorld();
    if (!World || World->bIsTearingDown)
    {
        return;
    }

    if (!RequiredMapName.IsEmpty() &&
        !World->GetMapName().Contains(RequiredMapName))
    {
        if (!bLoggedMapMismatch)
        {
            bLoggedMapMismatch = true;
            UE_LOG(
                LogTemp,
                Display,
                TEXT(
                    "[KAU_BUILDINGS] current map %s does not match %s; "
                    "automatic tileset creation skipped."),
                *World->GetMapName(),
                *RequiredMapName);
        }
        return;
    }

    if (!EnsureKauGeoreference())
    {
        return;
    }

    if (BuildingsTileset.IsValid())
    {
        return;
    }

    const double Now = World->GetTimeSeconds();
    if (Now - LastAttemptSeconds < RetryPeriodSeconds)
    {
        return;
    }
    LastAttemptSeconds = Now;
    TryCreateTileset();
}

bool UKauBuildingsTilesetSubsystem::EnsureKauGeoreference()
{
    if (bGeoreferenceConfigured)
    {
        return true;
    }

    UWorld* World = GetWorld();
    if (!World)
    {
        return false;
    }

    ACesiumGeoreference* Georeference =
        ACesiumGeoreference::GetDefaultGeoreference(World);
    if (!IsValid(Georeference))
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[KAU_BUILDINGS] default CesiumGeoreference unavailable; retrying."));
        return false;
    }

    const FVector Previous =
        Georeference->GetOriginLongitudeLatitudeHeight();
    if (bRecenterGeoreference)
    {
        Georeference->SetOriginPlacement(
            EOriginPlacement::CartographicOrigin);
        Georeference->SetOriginLongitudeLatitudeHeight(
            FVector(OriginLongitude, OriginLatitude, OriginHeight));
    }

    const FVector Current =
        Georeference->GetOriginLongitudeLatitudeHeight();
    bGeoreferenceConfigured = true;

    UE_LOG(
        LogTemp,
        Display,
        TEXT(
            "[KAU_BUILDINGS] GEOREFERENCE_READY actor=%s "
            "previous=(%.8f,%.8f,%.3f) current=(%.8f,%.8f,%.3f)"),
        *Georeference->GetName(),
        Previous.X,
        Previous.Y,
        Previous.Z,
        Current.X,
        Current.Y,
        Current.Z);
    return true;
}

ACesium3DTileset*
UKauBuildingsTilesetSubsystem::FindExistingTileset() const
{
    UWorld* World = GetWorld();
    if (!World)
    {
        return nullptr;
    }

    for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
    {
        ACesium3DTileset* Candidate = *It;
        if (IsValid(Candidate) &&
            (Candidate->ActorHasTag(KauTilesetTag) ||
             Candidate->GetFName() == KauTilesetActorName))
        {
            return Candidate;
        }
    }
    return nullptr;
}

void UKauBuildingsTilesetSubsystem::TryCreateTileset()
{
    UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    if (ACesium3DTileset* Existing = FindExistingTileset())
    {
        BuildingsTileset = Existing;
        UE_LOG(
            LogTemp,
            Display,
            TEXT("[KAU_BUILDINGS] using existing actor %s."),
            *Existing->GetName());
        return;
    }

    if (TilesetUrl.IsEmpty())
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("[KAU_BUILDINGS] TilesetUrl is empty; actor not created."));
        bEnabled = false;
        return;
    }

    FActorSpawnParameters SpawnParameters;
    SpawnParameters.Name = KauTilesetActorName;
    SpawnParameters.SpawnCollisionHandlingOverride =
        ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    ACesium3DTileset* Tileset = World->SpawnActor<ACesium3DTileset>(
        FVector::ZeroVector,
        FRotator::ZeroRotator,
        SpawnParameters);

    if (!IsValid(Tileset))
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[KAU_BUILDINGS] failed to spawn Cesium3DTileset; retrying."));
        return;
    }

    Tileset->Tags.AddUnique(KauTilesetTag);
    Tileset->SetTilesetSource(ETilesetSource::FromUrl);
    Tileset->SetUrl(TilesetUrl);
    Tileset->SetMaximumScreenSpaceError(MaximumScreenSpaceError);
    Tileset->SetCreatePhysicsMeshes(bEnableCollision);
    Tileset->SetEnableDoubleSidedCollisions(bEnableCollision);
    Tileset->SetActorEnableCollision(bEnableCollision);

    Tileset->PreloadAncestors = true;
    Tileset->PreloadSiblings = true;
    Tileset->ForbidHoles = true;
    Tileset->EnableFrustumCulling = true;
    Tileset->EnableFogCulling = false;
    Tileset->EnableOcclusionCulling = false;
    Tileset->MaximumSimultaneousTileLoads = 64;
    Tileset->MaximumCachedBytes = OneGiB;
    Tileset->LoadingDescendantLimit = 20;

    Tileset->ResolveGeoreference();
    BuildingsTileset = Tileset;

    UE_LOG(
        LogTemp,
        Display,
        TEXT(
            "[KAU_BUILDINGS] READY actor=%s url=%s sse=%.1f "
            "collision=%s physics_meshes=%s"),
        *Tileset->GetName(),
        *TilesetUrl,
        MaximumScreenSpaceError,
        bEnableCollision ? TEXT("true") : TEXT("false"),
        bEnableCollision ? TEXT("true") : TEXT("false"));
}
