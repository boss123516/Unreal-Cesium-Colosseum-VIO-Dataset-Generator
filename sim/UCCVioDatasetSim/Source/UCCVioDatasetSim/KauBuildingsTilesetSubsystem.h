#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "KauBuildingsTilesetSubsystem.generated.h"

class ACesium3DTileset;
class ACesiumGeoreference;

/**
 * Adds the locally generated KAU 5 km LoD1 buildings to HighAltitudeCity.
 *
 * The actor is created at runtime so the binary map does not need to carry a
 * machine-specific localhost URL. Cesium places the tile content from its ECEF
 * transforms, using the level's existing CesiumGeoreference.
 */
UCLASS()
class UCCVIODATASETSIM_API UKauBuildingsTilesetSubsystem
    : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;
    virtual bool DoesSupportWorldType(EWorldType::Type WorldType) const override;

    virtual void Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override;

private:
    void TryCreateTileset();
    ACesium3DTileset* FindExistingTileset() const;
    bool EnsureKauGeoreference();

private:
    TWeakObjectPtr<ACesium3DTileset> BuildingsTileset;

    FString TilesetUrl;
    FString RequiredMapName;
    double MaximumScreenSpaceError = 16.0;
    double OriginLongitude = 126.86519;
    double OriginLatitude = 37.60025;
    double OriginHeight = 37.630;
    double LastAttemptSeconds = -1000.0;
    bool bEnabled = true;
    bool bEnableCollision = true;
    bool bRecenterGeoreference = true;
    bool bGeoreferenceConfigured = false;
    bool bLoggedMapMismatch = false;

    static constexpr double RetryPeriodSeconds = 2.0;
};
