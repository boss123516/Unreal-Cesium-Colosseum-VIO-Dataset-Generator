#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "CesiumAirSimCameraBridgeSubsystem.generated.h"

class ACesium3DTileset;
class ACesiumCameraManager;
class APIPCamera;
class USceneCaptureComponent2D;

/**
 * Registers the AirSim scene camera with Cesium's tile-selection camera manager.
 *
 * The subsystem is created automatically for Game / PIE worlds. It selects the
 * most likely dataset camera (preferring 640x480, attached, downward-facing
 * APIPCamera scene captures), publishes the current camera and a 90 m look-ahead
 * camera to Cesium, and applies high-speed terrain streaming safeguards.
 */
UCLASS()
class UCCVIODATASETSIM_API UCesiumAirSimCameraBridgeSubsystem
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
    void DiscoverCameraAndManager();
    void RegisterBridgeCameras();
    void RemoveBridgeCameras();
    void UpdateBridgeCameras();
    void TuneTilesets();
    int32 ScoreCamera(APIPCamera* Camera, USceneCaptureComponent2D* Capture) const;

private:
    TWeakObjectPtr<ACesiumCameraManager> CameraManager;
    TWeakObjectPtr<APIPCamera> SensorCameraActor;
    TWeakObjectPtr<USceneCaptureComponent2D> SensorCapture;

    int32 BridgeCameraStartIndex = INDEX_NONE;
    int32 BridgeCameraCount = 0;

    double LastDiscoverySeconds = -1000.0;
    double LastTilesetTuneSeconds = -1000.0;
    bool bLoggedReady = false;

    static constexpr double DiscoveryPeriodSeconds = 1.0;
    static constexpr double TilesetTunePeriodSeconds = 2.0;
    static constexpr double LookAheadDistanceCm = 9000.0; // 30 m/s x 3 s
    static constexpr double LookAheadFovDegrees = 95.0;
};
