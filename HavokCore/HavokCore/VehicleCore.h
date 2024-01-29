#pragma once

#define HK_CALL __cdecl

#include "HK_Vehicle.h"

#include <Physics/Physics/Extensions/Vehicle/hknpVehicleInstance.h>
#include <Physics/Physics/Extensions/Vehicle/AeroDynamics/Default/hknpVehicleDefaultAerodynamics.h>
#include <Physics/Physics/Extensions/Vehicle/DriverInput/Default/hknpVehicleDefaultAnalogDriverInput.h>
#include <Physics/Physics/Extensions/Vehicle/Brake/Default/hknpVehicleDefaultBrake.h>
#include <Physics/Physics/Extensions/Vehicle/Engine/Default/hknpVehicleDefaultEngine.h>
#include <Physics/Physics/Extensions/Vehicle/VelocityDamper/Default/hknpVehicleDefaultVelocityDamper.h>
#include <Physics/Physics/Extensions/Vehicle/Steering/Default/hknpVehicleDefaultSteering.h>
#include <Physics/Physics/Extensions/Vehicle/Suspension/Default/hknpVehicleDefaultSuspension.h>
#include <Physics/Physics/Extensions/Vehicle/Transmission/Default/hknpVehicleDefaultTransmission.h>
#include <Physics/Physics/Extensions/Vehicle/WheelCollide/RayCast/hknpVehicleRayCastWheelCollide.h>
#include <Physics/Physics/Extensions/Vehicle/WheelCollide/LinearCast/hknpVehicleLinearCastWheelCollide.h>
#include <Physics/Physics/Extensions/Vehicle/TyreMarks/hknpTyremarksInfo.h>

#include <Physics/Physics/Collide/Shape/Convex/hknpConvexShape.h>
#include <Physics/Physics/Extensions/Vehicle/hknpVehicleInstance.h>
#include <Physics/Physics/Extensions/Vehicle/Camera/hknp1dAngularFollowCam.h>

class NpVehicleSetup : public hkReferencedObject
{
public:
    HK_DECLARE_CLASS(NpVehicleSetup, New);

    NpVehicleSetup() {}

    HK_CarSetup SetupData;

    virtual void buildVehicle(const hknpWorld* world, hknpVehicleInstance& vehicle, const hkRotation& chassisOrientation, bool useRaycast, bool initVehicle = true);

public:
    virtual void setupVehicleData(const hknpWorld* world, const hkRotation& chassisOrientation, hknpVehicleData& data);
    virtual void setupComponent(const hknpVehicleData& data, hknpVehicleDefaultAnalogDriverInput& driverInput);
    virtual void setupComponent(const hknpVehicleData& data, hknpVehicleDefaultSteering& steering);
    virtual void setupComponent(const hknpVehicleData& data, hknpVehicleDefaultEngine& engine);
    virtual void setupComponent(const hknpVehicleData& data, hknpVehicleDefaultTransmission& transmission);
    virtual void setupComponent(const hknpVehicleData& data, hknpVehicleDefaultBrake& brake);
    virtual void setupComponent(const hknpVehicleData& data, hknpVehicleDefaultSuspension& suspension);
    virtual void setupComponent(const hknpVehicleData& data, hkVector4Parameter gravity, hknpVehicleDefaultAerodynamics& aerodynamics);
    virtual void setupComponent(const hknpVehicleData& data, hknpVehicleDefaultVelocityDamper& velocityDamper);

    virtual void setupWheelCollide(const hknpWorld* world, const hknpVehicleInstance& vehicle, hknpVehicleLinearCastWheelCollide& wheelCollide);
    virtual void setupWheelCollide(const hknpWorld* world, const hknpVehicleInstance& vehicle, hknpVehicleRayCastWheelCollide& wheelCollide);
    virtual void setupTyremarks(const hknpVehicleData& data, hknpTyremarksInfo& tyremarkscontroller);

    virtual hkReal getMass() { return 750.0f; }
};

hkRefPtr<hknpShape> HK_CALL makeCarChassisShape(const hkVector4& fwd = { 1, 0, 0 }, const hkVector4& up = { 0, 1, 0 }, const hkVector4& right = { 0, 0, 1 });

struct DisplayMeterInfo
{
    float m_value;
    float m_maxValue;
    float m_stepSize;

    int m_posX;
    int m_posY;
    int m_radius;
    int m_pointerRadius;

    hkUint32 m_scaleColor;
    hkUint32 m_pointerColor;
};

struct VehicleDataAndDisplayInfo
{
    hknpVehicleInstance* m_vehicle;
    hkReal m_lastRPM;
};

void HK_CALL createDisplayWheel(hkArray<hkRefPtr<hkDisplayGeometry>>& displayGeometry, const hkVector4& fwd, const hkVector4& right, hkReal radius, hkReal thickness);

void HK_CALL createDisplayWheels(hkUint64 displayId, hkReal radius = 0.4f, hkReal thickness = 0.2f, const hkVector4& fwd = hkVector4(1, 0, 0), const hkVector4& right = hkVector4(0, 0, 1));

hkRefPtr<hknpShape> HK_CALL makeDisc(const hkVector4& fwd, const hkVector4& right, hkReal radius, hkReal thickness, int numSides);

void HK_CALL syncDisplayWheels(hknpVehicleInstance& vehicle, const hkArray<hkUint64>& wheels, hkVdbCmdTag tag, const hkVector4& up = hkVector4(0, 1, 0));