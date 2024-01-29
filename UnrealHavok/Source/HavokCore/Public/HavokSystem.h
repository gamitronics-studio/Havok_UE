#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "HavokSystem.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FWorldCleared);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FPhysicsOn);

USTRUCT(BlueprintType)
struct FHardwareData
{
	GENERATED_BODY();

	UPROPERTY(BlueprintReadWrite)
	FString ArchName;

	UPROPERTY(BlueprintReadOnly)
	FString CPU;

	UPROPERTY(BlueprintReadOnly)
	int NumOfCores = 0;

	UPROPERTY(BlueprintReadOnly)
	int64 RAM = 0;
};


UCLASS()
class HAVOKCORE_API UHavokSystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

private:

	bool bPhysicsIsOn;

public:

	UFUNCTION(BlueprintCallable)
	FHardwareData GetHardwareInfo();

	UFUNCTION(BlueprintCallable)
	void PhysicsStart(FVector gravity, float fps);

	UFUNCTION(BlueprintCallable)
	void PhysicsStop();

	UFUNCTION(BlueprintCallable)
	void PhysicsQuit();

	UFUNCTION(BlueprintCallable)
	int ClearWorld();

	UFUNCTION(BlueprintCallable)
	void AddFloor(float X, float Y);

	UFUNCTION(BlueprintCallable)
	void AddBlock(float X, float Y, float Z, FVector Loc, FQuat Rot);

	UFUNCTION(BlueprintCallable)
	void CreateMesh(const TArray<float>& X, const TArray<float>& Y, const TArray<float>& Z, const TArray<int>& Triangles, FVector Location, FQuat Rotation);

	UFUNCTION(BlueprintCallable)
	void TestMesh();

	//____ Dispatchers ____//

	UPROPERTY(BlueprintAssignable)
	FPhysicsOn OnPhysicsOn;

	UPROPERTY(BlueprintAssignable)
	FWorldCleared OnWorldCleared;

};