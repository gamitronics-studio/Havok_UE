#pragma once  

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

                //____ Physics Init ____//

#ifdef __cplusplus 
extern "C"
{
#endif

    struct HardwareData;
    struct Gravity;

    void StartPhysics(Gravity, float);
    void StopPhysics();
    void QuitPhysics();

    int HK_ClearWorld();
    void HK_AddFloor(float, float);
    void HK_AddBlock(float, float, float, HK_Location, HK_Quaternion);
    void HK_AddMesh(const float[], const float[], const float[], int, const int[], int, HK_Location, HK_Quaternion);
    void HK_TestMesh();
    HardwareData HardwareInfo();

#ifdef __cplusplus
}
#endif

                //____ Basic Body ____//

class __declspec(dllimport) HK_Body
{
private:

    int BodyIndex;
    struct hknpBodyId* BodyID;

public:
    HK_Body();
    bool HK_RemoveBody();
    void HK_AddSphere(float, float, float, HK_Location);
    void HK_AddBox(float, float, float, float, float, HK_Location, HK_Quaternion);
    void HK_AddMesh(const float[], const float[], const float[], int, const int[], int, HK_Location, HK_Quaternion);

    HK_Location HK_GetLocation();
    HK_Quaternion HK_GetRotation();
    int HK_GetID();
};

                //____ Car ____//

struct HK_CarSetup
{
    HK_Location SpawnLocation;

    float HK_Mass = 750.0f;
    float HK_maxSpeed = 130.0f;

    float HK_Torque = 500.0f;
    float HK_minRPM = 1000.0f;
    float HK_maxRPM = 7500.0f;
    float HK_optRPM = 5500.0f;

    int HK_Gears = 4;
    float HK_DownShiftAtRPM = 3500.0f;
    float HK_UpShiftAtRPM = 6500.0f;
    float HK_ReverseGearRatio = 1.0f;
    float HK_GearRatios[4] = { 2.0f, 1.5f, 1.0f, 0.75f };

    float HK_WheelFriction = 1.5f;
    float HK_WheelRadius = 0.4f;
    float HK_WheelThickness = 0.25f;
    float HK_WheelMass = 10.0f;

    float HK_BrakeTorque = 1500.0f;

    float HK_liftCoefficient = -0.3f;

    float HK_maxSteer = 35.0f;
    bool HK_SteerBackWheels = false;

    float HK_SuspensionLength = 0.35f;
    float HK_SuspensionStrength = 50.0f;

    float Up = -0.05f, Front = 1.3f, Back = 1.1f, Lateral = 1.1f;
};

struct HK_CarStats
{
    float KMPH;
    float RPM;
};

class __declspec(dllimport) HK_Car
{

private:
    struct hknpBodyId* BodyID;
    class hknpShape* BodyShape;
    HK_CarSetup CarData;

public:
    HK_Car();
    void HK_BuildCarMesh(const float[], const float[], const float[], int, const int[], int);
    int HK_BuildCar(HK_CarSetup, const float X[], const float Y[], const float Z[], int VertCount, const int Triangles[], int TriCount);
    HK_Location HK_GetCarLocation();
    HK_Quaternion HK_GetCarRotation();
    float HK_GetCarAngle();
    void HK_UpdateWheels(HK_Location&, HK_Quaternion&, HK_Location&, HK_Quaternion&, HK_Location&, HK_Quaternion&, HK_Location&, HK_Quaternion&);

    void HK_Step(float, float, bool, bool, float);
    HK_CarStats HK_GetCarStats();
    void HK_ResetCar();
};