#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HavokVehicle.generated.h"

USTRUCT(BlueprintType)
struct FCarSetup
{
    GENERATED_BODY()

    // Spawn //
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector SpawnLocation = FVector(0.0f, 0.0f, 0.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Mass = 750.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float maxSpeed = 130.0f;

    // Engine //
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Torque = 500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float minRPM = 1000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float maxRPM = 7500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float optRPM = 5500.0f;

    // Transmission //
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float DownShiftAtRPM = 3500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float UpShiftAtRPM = 6500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float ReverseGearRatio = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<float> GearRatios;


    // Wheel //
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelRadius = 0.4f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelThickness = 0.2f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelFriction = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelMass = 10.0f;

    // Brake //
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float BrakeTorque = 1500.0f;

    // Aerodynamics //
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float liftCoefficient = -0.3f;

    // Steer //
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float maxSteer = 35.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool SteerBackWheels = false;

    // Suspension //
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float SuspensionLength = 0.35f;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float SuspensionStrength = 50.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float U = -0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float F = 1.3f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float B = 1.1f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float L = 1.1f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bUseCustomCar;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bUseRayTrace;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float bBodyScale;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float NormalSpinDamping = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float CollisionSpinDamping = 4.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float CollisionThreshold = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bUseCustomWheel;

};

USTRUCT(BlueprintType)
struct FMeshData
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
     TArray<float> X;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
     TArray<float> Y;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
     TArray<float> Z;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
     TArray<int> Triangles;
};

UCLASS(BlueprintType)
class AHK_Vehicle : public AActor
{
    GENERATED_BODY()

public:

    AHK_Vehicle();


    //UFUNCTION(BlueprintCallable)
    //void BuildCarMesh(const TArray<float>& X, const TArray<float>& Y, const TArray<float>& Z, const TArray<int>& Triangles);

    UFUNCTION(BlueprintCallable)
    void SpawnCar(FCarSetup SetupData, const TArray<float>& X, const TArray<float>& Y, const TArray<float>& Z, const TArray<int>& Triangles, FMeshData WheelMesh);

    UFUNCTION(BlueprintCallable)
    FVector GetCarLoc();

    UFUNCTION(BlueprintCallable)
    FRotator GetCarRot();

    UFUNCTION(BlueprintCallable)
    void StepCar(float WS, float AD, bool R, bool B, float T);

    UFUNCTION(BlueprintCallable)
    void UpdateWheels(FVector& T1L, FQuat& T1Q, FVector& T2L, FQuat& T2Q, FVector& T3L, FQuat& T3Q, FVector& T4L, FQuat& T4Q);

    UFUNCTION(BlueprintCallable)
    void GetCarStats(float& RPM, float& KMPH, int& Gear, bool &isReverse, float &Torque, float &SteerAngle);

    UFUNCTION(BlueprintCallable)
    void ResetCar();

    UFUNCTION(BlueprintCallable)
    void UpdateCar(FCarSetup SetupData);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bUseCustomCar;

    UFUNCTION(BlueprintCallable)
    FCarSetup GetCatSetup();

private:
    class HK_Car* Car;
};