#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HavokBasicBody.generated.h"

UCLASS(BlueprintType)
class AHK_Body : public AActor
{
    GENERATED_BODY()

public:
    
    AHK_Body();

    UFUNCTION(BlueprintCallable)
    bool RemoveBody();

    UFUNCTION(BlueprintCallable)
    void CreateSphere(float Radius, float Mass, float Friction, FVector Location);

    UFUNCTION(BlueprintCallable)
    void CreateBox(float L, float W, float H, float Mass, float Friction, FVector Location, FQuat Rot);

    UFUNCTION(BlueprintCallable)
    void AddMesh(const TArray<float>& X, const  TArray<float>& Y, const  TArray<float>& Z, int VertCount, const TArray<int>& Triangles, int TriCount, FVector Location, FQuat Rotation);

    UFUNCTION(BlueprintCallable)
    FVector GetLocation();

    UFUNCTION(BlueprintCallable)
    FRotator GetRotation();

    UFUNCTION(BlueprintCallable)
    int GetID();

private:
    class HK_Body* Body;
};