#pragma once  

#define DLL_EXPORT __declspec(dllexport)

struct HardwareData
{
    int NumOfCores;
    const char* ArchName;
    const char* CpuName;
    unsigned long long TotalRAM;
};

struct Gravity
{
    float x;
    float y;
    float z;
};

struct HK_Location
{
    float x;
    float y;
    float z;
};

struct HK_Quaternion
{
    float x;
    float y;
    float z;
    float w;

};


#ifdef __cplusplus 
extern "C"
{
#endif

    struct HardwareData;
    HardwareData DLL_EXPORT HardwareInfo();

    struct Gravity;

    void DLL_EXPORT StartPhysics(Gravity, float);
    void DLL_EXPORT StopPhysics();
    void DLL_EXPORT QuitPhysics();
    int DLL_EXPORT HK_ClearWorld();
    void DLL_EXPORT HK_AddFloor(float, float);
    void DLL_EXPORT HK_AddBlock(float, float, float, HK_Location, HK_Quaternion);
    void DLL_EXPORT HK_AddMesh(const float[], const float[], const float[], int, const int[], int, HK_Location, HK_Quaternion);
    void DLL_EXPORT HK_TestMesh();

#ifdef __cplusplus
}
#endif

#include <Physics/Physics/Collide/Shape/hknpShape.h>
#include <Physics/Physics/Dynamics/Material/hknpMaterialLibrary.h>
#include <Physics/Physics/Dynamics/World/hknpWorld.h>
#include <Physics/Physics/Dynamics/World/hknpWorldCinfo.h>

hknpBodyId AddBody(hknpBodyCinfo);
bool RemoveBody(hknpBodyId);
hknpMaterialId GetMaterial(hknpMaterial);
HK_Location GetBodyLocation(hknpBodyId);
HK_Quaternion GetBodyRotation(hknpBodyId);
hkRefPtr<hknpWorld> GetWorld();
float GetBodyAngle(hknpBodyId);


