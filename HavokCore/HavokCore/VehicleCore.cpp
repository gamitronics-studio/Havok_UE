#include "pch.h" 
#include "Core.h"
#include "VehicleCore.h"

#include <Physics/Physics/Extensions/Vehicle/hknpVehicleInstance.h>
#include <Physics/Physics/Extensions/Vehicle/DriverInput/Default/hknpVehicleDefaultAnalogDriverInput.h>
#include <Physics/Physics/Extensions/Vehicle/Camera/hknp1dAngularFollowCam.h>
#include <Physics/Physics/Collide/Shape/Convex/hknpConvexShape.h>
#include <Physics/Physics/Collide/Shape/hknpShapeUtil.h>
#include <Physics/Physics/Extensions/Viewers/Shape/hknpShapeViewer.h>

#include <Common/Visualize/Handlers/hkDoublePrecisionDebugDisplayHandlerWrapper.h>

#include <Common/Base/Visualize/Shape/hkDisplayGeometry.h>
#include <Common/Base/Types/Color/hkColor.h>
#include <Geometry/Collide/GeometryProcessing/ConvexHull/hkGeometryUtility.h>



#include "pch.h" 
#include "Core.h"
#include "VehicleCore.h"

#include <Physics/Physics/Extensions/Vehicle/hknpVehicleInstance.h>
#include <Physics/Physics/Extensions/Vehicle/DriverInput/Default/hknpVehicleDefaultAnalogDriverInput.h>
#include <Physics/Physics/Extensions/Vehicle/Camera/hknp1dAngularFollowCam.h>
#include <Physics/Physics/Collide/Shape/Convex/hknpConvexShape.h>
#include <Physics/Physics/Collide/Shape/hknpShapeUtil.h>
#include <Physics/Physics/Extensions/Viewers/Shape/hknpShapeViewer.h>

#include <Common/Visualize/Handlers/hkDoublePrecisionDebugDisplayHandlerWrapper.h>

#include <Common/Base/Visualize/Shape/hkDisplayGeometry.h>
#include <Common/Base/Types/Color/hkColor.h>
#include <Geometry/Collide/GeometryProcessing/ConvexHull/hkGeometryUtility.h>

#define COMPILE

void NpVehicleSetup::buildVehicle(const hknpWorld* world, hknpVehicleInstance& vehicle, const hkRotation& chassisOrientation, bool useRaycast, bool initVehicle)
{
    vehicle.m_data = new hknpVehicleData;
    vehicle.m_driverInput = new hknpVehicleDefaultAnalogDriverInput;
    vehicle.m_steering = new hknpVehicleDefaultSteering;
    vehicle.m_engine = new hknpVehicleDefaultEngine;
    vehicle.m_transmission = new hknpVehicleDefaultTransmission;
    vehicle.m_brake = new hknpVehicleDefaultBrake;
    vehicle.m_suspension = new hknpVehicleDefaultSuspension;
    vehicle.m_aerodynamics = new hknpVehicleDefaultAerodynamics;
    vehicle.m_velocityDamper = new hknpVehicleDefaultVelocityDamper;

    if (useRaycast)
    {
        vehicle.m_wheelCollide = new hknpVehicleRayCastWheelCollide;
    }
    else
    {
        vehicle.m_wheelCollide = new hknpVehicleLinearCastWheelCollide;
    }

    setupVehicleData(world, chassisOrientation, *vehicle.m_data);

    vehicle.m_tyreMarks = new hknpTyremarksInfo(*vehicle.m_data, 128);

    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultAnalogDriverInput*>(vehicle.m_driverInput));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultSteering*>(vehicle.m_steering));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultEngine*>(vehicle.m_engine));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultTransmission*>(vehicle.m_transmission));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultBrake*>(vehicle.m_brake));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultSuspension*>(vehicle.m_suspension));
    setupComponent(*vehicle.m_data, world->getGravity(), *static_cast<hknpVehicleDefaultAerodynamics*>(vehicle.m_aerodynamics));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultVelocityDamper*>(vehicle.m_velocityDamper));

    if (useRaycast)
    {
        setupWheelCollide(world, vehicle, *static_cast<hknpVehicleRayCastWheelCollide*>(vehicle.m_wheelCollide));
    }
    else
    {
        setupWheelCollide(world, vehicle, *static_cast<hknpVehicleLinearCastWheelCollide*>(vehicle.m_wheelCollide));
    }

    setupTyremarks(*vehicle.m_data, *static_cast<hknpTyremarksInfo*>(vehicle.m_tyreMarks));

    vehicle.m_deviceStatus = new hknpVehicleDriverInputAnalogStatus;
    hknpVehicleDriverInputAnalogStatus* deviceStatus = (hknpVehicleDriverInputAnalogStatus*)vehicle.m_deviceStatus;
    deviceStatus->m_positionY = -0.4f;
    deviceStatus->m_positionX = 0.3f;

    deviceStatus->m_handbrakeButtonPressed = false;
    deviceStatus->m_reverseButtonPressed = false;

    if (initVehicle)
    {
        vehicle.init();
    }
}

#ifdef COMPILE

void NpVehicleSetup::setupVehicleData(const hknpWorld* world, const hkRotation& chassisOrientation, hknpVehicleData& data)
{
    data.m_gravity = world->getGravity();

    data.m_chassisOrientation = chassisOrientation;

    data.m_frictionEqualizer = 0.5f;

    data.m_torqueRollFactor = SetupData.HK_TorqueRollFactor; // 0.625f;
    data.m_torquePitchFactor = SetupData.HK_TorquePitchFactor; // 0.5f;
    data.m_torqueYawFactor = SetupData.HK_TorqueYawFactor; // 0.35f;

    data.m_chassisUnitInertiaYaw = 1.0f;
    data.m_chassisUnitInertiaRoll = 1.0f;
    data.m_chassisUnitInertiaPitch = 1.0f;

    data.m_extraTorqueFactor = SetupData.HK_extraTorque; // -0.5f;
    data.m_maxVelocityForPositionalFriction = 0.0f;

    data.m_numWheels = 4;

    data.m_wheelParams.setSize(data.m_numWheels);

    data.m_wheelParams[0].m_axle = 0;
    data.m_wheelParams[1].m_axle = 0;
    data.m_wheelParams[2].m_axle = 1;
    data.m_wheelParams[3].m_axle = 1;

    data.m_wheelParams[0].m_friction = SetupData.HK_WheelFrictionF; // 1.5f;
    data.m_wheelParams[1].m_friction = SetupData.HK_WheelFrictionF; // 1.5f;
    data.m_wheelParams[2].m_friction = SetupData.HK_WheelFrictionB; // 1.5f;
    data.m_wheelParams[3].m_friction = SetupData.HK_WheelFrictionB; // 1.5f;

    data.m_wheelParams[0].m_slipAngle = SetupData.HK_SlipAngle;
    data.m_wheelParams[1].m_slipAngle = SetupData.HK_SlipAngle;
    data.m_wheelParams[2].m_slipAngle = SetupData.HK_SlipAngle;
    data.m_wheelParams[3].m_slipAngle = SetupData.HK_SlipAngle;

    for (int i = 0; i < data.m_numWheels; i++)
    {
        data.m_wheelParams[i].m_radius = SetupData.HK_WheelRadius; // 0.4f;
        data.m_wheelParams[i].m_width = SetupData.HK_WheelThickness; // 0.25f;
        data.m_wheelParams[i].m_mass = SetupData.HK_WheelMass; // 10.0f;

        data.m_wheelParams[i].m_viscosityFriction = SetupData.HK_ViscosityFriction;
        data.m_wheelParams[i].m_maxFriction = 2.0f * data.m_wheelParams[i].m_friction;
        data.m_wheelParams[i].m_forceFeedbackMultiplier = 0.1f;
        data.m_wheelParams[i].m_maxContactBodyAcceleration = hkReal(data.m_gravity.length3()) * 2;
    }
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultAnalogDriverInput& driverInput)
{
    driverInput.m_slopeChangePointX = 0.8f;
    driverInput.m_initialSlope = 0.7f;
    driverInput.m_deadZone = 0.0f;
    driverInput.m_autoReverse = true;
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultSteering& steering)
{
    steering.m_doesWheelSteer.setSize(data.m_numWheels);

    steering.m_maxSteeringAngle = SetupData.HK_maxSteer * (HK_REAL_PI / 180);

    steering.m_maxSpeedFullSteeringAngle = SetupData.HK_MaxSpeedFullSteer * (1.605f / 3.6f);
    steering.m_doesWheelSteer[0] = true;
    steering.m_doesWheelSteer[1] = true;
    steering.m_doesWheelSteer[2] = SetupData.HK_SteerBackWheels;
    steering.m_doesWheelSteer[3] = SetupData.HK_SteerBackWheels;
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultEngine& engine)
{
    engine.m_maxTorque = SetupData.HK_maxTorque; // 500.0f;
    engine.m_minRPM = SetupData.HK_minRPM; // 1000.0f;
    engine.m_optRPM = SetupData.HK_optRPM; // 5500.0f;
    engine.m_maxRPM = SetupData.HK_maxRPM; // 7500.0f

    engine.m_torqueFactorAtMinRPM = 0.8f;
    engine.m_torqueFactorAtMaxRPM = 0.8f;

    engine.m_resistanceFactorAtMinRPM = 0.05f;
    engine.m_resistanceFactorAtOptRPM = 0.1f;
    engine.m_resistanceFactorAtMaxRPM = 0.3f;
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultTransmission& transmission)
{
    int numGears = SetupData.HK_Gears;

    transmission.m_gearsRatio.setSize(numGears);
    transmission.m_wheelsTorqueRatio.setSize(data.m_numWheels);

    transmission.m_downshiftRPM = SetupData.HK_DownShiftAtRPM; // 3500.0f;
    transmission.m_upshiftRPM = SetupData.HK_UpShiftAtRPM;// 6500.0f;

    transmission.m_clutchDelayTime = 0.0f;
    transmission.m_reverseGearRatio = SetupData.HK_ReverseGearRatio; // 1.0f;

    transmission.m_gearsRatio[0] = SetupData.HK_GearRatios[0]; // 2.0f;
    transmission.m_gearsRatio[1] = SetupData.HK_GearRatios[1]; // 1.5f;
    transmission.m_gearsRatio[2] = SetupData.HK_GearRatios[2]; // 1.0f;
    transmission.m_gearsRatio[3] = SetupData.HK_GearRatios[3]; // 0.75f;
    transmission.m_gearsRatio[4] = SetupData.HK_GearRatios[4]; // 0.75f;
    transmission.m_gearsRatio[5] = SetupData.HK_GearRatios[5]; // 0.75f;

    transmission.m_wheelsTorqueRatio[0] = 0.2f;
    transmission.m_wheelsTorqueRatio[1] = 0.2f;
    transmission.m_wheelsTorqueRatio[2] = 0.3f;
    transmission.m_wheelsTorqueRatio[3] = 0.3f;

    const hkReal vehicleTopSpeed = SetupData.HK_maxSpeed; // 130.0f
    const hkReal wheelRadius = SetupData.HK_WheelRadius; // 0.4f;
    const hkReal maxEngineRpm = SetupData.HK_maxRPM; // 7500.0f
    transmission.m_primaryTransmissionRatio = hknpVehicleDefaultTransmission::calculatePrimaryTransmissionRatio(vehicleTopSpeed, wheelRadius, maxEngineRpm, transmission.m_gearsRatio[numGears - 1]);
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultBrake& brake)
{
    brake.m_wheelBrakingProperties.setSize(data.m_numWheels);

    const float bt = SetupData.HK_BrakeTorque; // 1500
    brake.m_wheelBrakingProperties[0].m_maxBreakingTorque = bt;
    brake.m_wheelBrakingProperties[1].m_maxBreakingTorque = bt;
    brake.m_wheelBrakingProperties[2].m_maxBreakingTorque = bt;
    brake.m_wheelBrakingProperties[3].m_maxBreakingTorque = bt;

    brake.m_wheelBrakingProperties[0].m_isConnectedToHandbrake = false;
    brake.m_wheelBrakingProperties[1].m_isConnectedToHandbrake = false;
    brake.m_wheelBrakingProperties[2].m_isConnectedToHandbrake = true;
    brake.m_wheelBrakingProperties[3].m_isConnectedToHandbrake = true;
    brake.m_wheelBrakingProperties[0].m_minPedalInputToBlock = 0.9f;
    brake.m_wheelBrakingProperties[1].m_minPedalInputToBlock = 0.9f;
    brake.m_wheelBrakingProperties[2].m_minPedalInputToBlock = 0.9f;
    brake.m_wheelBrakingProperties[3].m_minPedalInputToBlock = 0.9f;
    brake.m_wheelsMinTimeToBlock = 1000.0f;
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultSuspension& suspension)
{
    suspension.m_wheelParams.setSize(data.m_numWheels);
    suspension.m_wheelSpringParams.setSize(data.m_numWheels);

    hkReal Length = SetupData.HK_SuspensionLength; //0.35f
    suspension.m_wheelParams[0].m_length = Length;
    suspension.m_wheelParams[1].m_length = Length;
    suspension.m_wheelParams[2].m_length = Length;
    suspension.m_wheelParams[3].m_length = Length;

    const float str = SetupData.HK_SuspensionStrength;
    suspension.m_wheelSpringParams[0].m_strength = str;
    suspension.m_wheelSpringParams[1].m_strength = str;
    suspension.m_wheelSpringParams[2].m_strength = str;
    suspension.m_wheelSpringParams[3].m_strength = str;

    const float wd = 3.0f;
    suspension.m_wheelSpringParams[0].m_dampingCompression = wd;
    suspension.m_wheelSpringParams[1].m_dampingCompression = wd;
    suspension.m_wheelSpringParams[2].m_dampingCompression = wd;
    suspension.m_wheelSpringParams[3].m_dampingCompression = wd;

    suspension.m_wheelSpringParams[0].m_dampingRelaxation = wd;
    suspension.m_wheelSpringParams[1].m_dampingRelaxation = wd;
    suspension.m_wheelSpringParams[2].m_dampingRelaxation = wd;
    suspension.m_wheelSpringParams[3].m_dampingRelaxation = wd;

    const hkVector4 upDir = data.m_chassisOrientation.getColumn<0>();
    const hkVector4 fwdDir = data.m_chassisOrientation.getColumn<1>();
    const hkVector4 latDir = data.m_chassisOrientation.getColumn<2>();

    {
        hkVector4 vUp;
        vUp.setMul(upDir, hkSimdReal::fromFloat(SetupData.Up));
        hkVector4 vFront;
        vFront.setAddMul(vUp, fwdDir, hkSimdReal::fromFloat(SetupData.Front));
        hkVector4 vBack;
        vBack.setSubMul(vUp, fwdDir, hkSimdReal::fromFloat(SetupData.Back));
        hkVector4 vLateral;
        vLateral.setMul(latDir, hkSimdReal::fromFloat(SetupData.Lateral));

        suspension.m_wheelParams[0].m_hardpointChassisSpace.setSub(vFront, vLateral);  // ( hardPointFrontX, hardPointY, -hardPointZ);
        suspension.m_wheelParams[1].m_hardpointChassisSpace.setAdd(vFront, vLateral);  // ( hardPointFrontX, hardPointY,  hardPointZ);
        suspension.m_wheelParams[2].m_hardpointChassisSpace.setSub(vBack, vLateral);   // ( hardPointBackX, hardPointY, -hardPointZ);
        suspension.m_wheelParams[3].m_hardpointChassisSpace.setAdd(vBack, vLateral);   // ( hardPointBackX, hardPointY,  hardPointZ);
    }

    const hkVector4 downDir(0.0f, -1.0f, 0.0f);
    suspension.m_wheelParams[0].m_directionChassisSpace = downDir;
    suspension.m_wheelParams[1].m_directionChassisSpace = downDir;
    suspension.m_wheelParams[2].m_directionChassisSpace = downDir;
    suspension.m_wheelParams[3].m_directionChassisSpace = downDir;
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hkVector4Parameter gravity, hknpVehicleDefaultAerodynamics& aerodynamics)
{
    aerodynamics.m_airDensity = 1.3f;
    aerodynamics.m_frontalArea = SetupData.HK_FrontArea; // 1.0f;

    aerodynamics.m_dragCoefficient = SetupData.HK_DragCoefficient; // 0.7f
    aerodynamics.m_liftCoefficient = SetupData.HK_liftCoefficient; // -0.3f;

    hkVector4 dir = gravity;
    dir.normalize<3>();

    aerodynamics.m_extraGravityws.setMul(dir, hkSimdReal::fromFloat(SetupData.HK_ExtraGravity));
}

#endif

#ifndef COMPILE

void NpVehicleSetup::setupVehicleData(const hknpWorld* world, const hkRotation& chassisOrientation, hknpVehicleData& data)
{
    data.m_gravity = world->getGravity();

    data.m_chassisOrientation = chassisOrientation;

    data.m_frictionEqualizer = 0.5f;

    data.m_torqueRollFactor = 0.325f;
    data.m_torquePitchFactor = 0.5f;
    data.m_torqueYawFactor = 0.325f;

    data.m_chassisUnitInertiaYaw = 1.0f;
    data.m_chassisUnitInertiaRoll = 1.0f;
    data.m_chassisUnitInertiaPitch = 1.0f;

    data.m_extraTorqueFactor = -40000.f;
    data.m_maxVelocityForPositionalFriction = 10.0f;

    data.m_numWheels = 4;

    data.m_wheelParams.setSize(data.m_numWheels);

    data.m_wheelParams[0].m_axle = 0;
    data.m_wheelParams[1].m_axle = 0;
    data.m_wheelParams[2].m_axle = 1;
    data.m_wheelParams[3].m_axle = 1;

    data.m_wheelParams[0].m_friction = 4.7f;
    data.m_wheelParams[1].m_friction = 4.7f;
    data.m_wheelParams[2].m_friction = 4.6f;
    data.m_wheelParams[3].m_friction = 4.6f;

    data.m_wheelParams[0].m_slipAngle = 0.05f;
    data.m_wheelParams[1].m_slipAngle = 0.05f;
    data.m_wheelParams[2].m_slipAngle = 0.02f;
    data.m_wheelParams[3].m_slipAngle = 0.02f;

    for (int i = 0; i < data.m_numWheels; i++)
    {
        data.m_wheelParams[i].m_radius = 0.4f;
        data.m_wheelParams[i].m_width = 0.2f;
        data.m_wheelParams[i].m_mass = 20.0f;

        data.m_wheelParams[i].m_viscosityFriction = 0.0f;
        data.m_wheelParams[i].m_maxFriction = 2.0f * data.m_wheelParams[i].m_friction;
        data.m_wheelParams[i].m_forceFeedbackMultiplier = 0.1f;
        data.m_wheelParams[i].m_maxContactBodyAcceleration = hkReal(data.m_gravity.length3()) * 2;
    }
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultAnalogDriverInput& driverInput)
{

    driverInput.m_slopeChangePointX = 0.8f;
    driverInput.m_initialSlope = 0.7f;
    driverInput.m_deadZone = 0.0f;
    driverInput.m_autoReverse = true;
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultSteering& steering)
{
    steering.m_doesWheelSteer.setSize(data.m_numWheels);

    steering.m_maxSteeringAngle = 35 * (HK_REAL_PI / 180);

    steering.m_maxSpeedFullSteeringAngle = 70.0f * (1.605f / 3.6f);
    steering.m_doesWheelSteer[0] = true;
    steering.m_doesWheelSteer[1] = true;
    steering.m_doesWheelSteer[2] = false;
    steering.m_doesWheelSteer[3] = false;
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultEngine& engine)
{
    engine.m_maxTorque = 3000.0f;

    engine.m_minRPM = 1000.0f;
    engine.m_optRPM = 7500.0f;

    engine.m_maxRPM = 8500.0f;

    engine.m_torqueFactorAtMinRPM = 0.8f;
    engine.m_torqueFactorAtMaxRPM = 0.8f;

    engine.m_resistanceFactorAtMinRPM = 0.05f;
    engine.m_resistanceFactorAtOptRPM = 0.1f;
    engine.m_resistanceFactorAtMaxRPM = 0.3f;
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultTransmission& transmission)
{
    int numGears = 5;

    transmission.m_gearsRatio.setSize(numGears);
    transmission.m_wheelsTorqueRatio.setSize(data.m_numWheels);

    transmission.m_downshiftRPM = 3500.0f;
    transmission.m_upshiftRPM = 7500.0f;

    transmission.m_clutchDelayTime = 0.0f;
    transmission.m_reverseGearRatio = 1.0f;
    transmission.m_gearsRatio[0] = 6.0f;
    transmission.m_gearsRatio[1] = 4.0f;
    transmission.m_gearsRatio[2] = 2.5f;
    transmission.m_gearsRatio[3] = 1.5f;
    transmission.m_gearsRatio[4] = 1.f;

    transmission.m_wheelsTorqueRatio[0] = 0.1f;
    transmission.m_wheelsTorqueRatio[1] = 0.1f;
    transmission.m_wheelsTorqueRatio[2] = 0.4f;
    transmission.m_wheelsTorqueRatio[3] = 0.4f;

    const hkReal vehicleTopSpeed = 185.0f;
    const hkReal wheelRadius = 0.4f;
    const hkReal maxEngineRpm = 8500.0f;
    transmission.m_primaryTransmissionRatio = hknpVehicleDefaultTransmission::calculatePrimaryTransmissionRatio(vehicleTopSpeed,
        wheelRadius,
        maxEngineRpm,
        transmission.m_gearsRatio[numGears - 1]);
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultBrake& brake)
{
    brake.m_wheelBrakingProperties.setSize(data.m_numWheels);

    const float bt = 1500.0f;
    brake.m_wheelBrakingProperties[0].m_maxBreakingTorque = bt;
    brake.m_wheelBrakingProperties[1].m_maxBreakingTorque = bt;
    brake.m_wheelBrakingProperties[2].m_maxBreakingTorque = bt;
    brake.m_wheelBrakingProperties[3].m_maxBreakingTorque = bt;

    // Handbrake is attached to rear wheels only.
    brake.m_wheelBrakingProperties[0].m_isConnectedToHandbrake = false;
    brake.m_wheelBrakingProperties[1].m_isConnectedToHandbrake = false;
    brake.m_wheelBrakingProperties[2].m_isConnectedToHandbrake = true;
    brake.m_wheelBrakingProperties[3].m_isConnectedToHandbrake = true;
    brake.m_wheelBrakingProperties[0].m_minPedalInputToBlock = 0.9f;
    brake.m_wheelBrakingProperties[1].m_minPedalInputToBlock = 0.9f;
    brake.m_wheelBrakingProperties[2].m_minPedalInputToBlock = 0.9f;
    brake.m_wheelBrakingProperties[3].m_minPedalInputToBlock = 0.9f;
    brake.m_wheelsMinTimeToBlock = 1000.0f;
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultSuspension& suspension)
{
    suspension.m_wheelParams.setSize(data.m_numWheels);
    suspension.m_wheelSpringParams.setSize(data.m_numWheels);

    const hkReal hardPoint = 0.1f;
    suspension.m_wheelParams[0].m_length = 0.4f + hardPoint;
    suspension.m_wheelParams[1].m_length = 0.4f + hardPoint;
    suspension.m_wheelParams[2].m_length = 0.4f + hardPoint;
    suspension.m_wheelParams[3].m_length = 0.4f + hardPoint;

    hkReal m_suspensionStrength = 100.0f;
    suspension.m_wheelSpringParams[0].m_strength = m_suspensionStrength;
    suspension.m_wheelSpringParams[1].m_strength = m_suspensionStrength;
    suspension.m_wheelSpringParams[2].m_strength = m_suspensionStrength;
    suspension.m_wheelSpringParams[3].m_strength = m_suspensionStrength;

    const float compress = 3.0f;
    suspension.m_wheelSpringParams[0].m_dampingCompression = compress;
    suspension.m_wheelSpringParams[1].m_dampingCompression = compress;
    suspension.m_wheelSpringParams[2].m_dampingCompression = compress;
    suspension.m_wheelSpringParams[3].m_dampingCompression = compress;

    const float relax = 3.0f;
    suspension.m_wheelSpringParams[0].m_dampingRelaxation = relax;
    suspension.m_wheelSpringParams[1].m_dampingRelaxation = relax;
    suspension.m_wheelSpringParams[2].m_dampingRelaxation = relax;
    suspension.m_wheelSpringParams[3].m_dampingRelaxation = relax;

    const hkVector4 upDir = data.m_chassisOrientation.getColumn<0>();
    const hkVector4 fwdDir = data.m_chassisOrientation.getColumn<1>();
    const hkVector4 latDir = data.m_chassisOrientation.getColumn<2>();

    {
        hkVector4 vUp;
        vUp.setMul(upDir, hkSimdReal::fromFloat(-0.05f));
        hkVector4 vFront;
        vFront.setAddMul(vUp, fwdDir, hkSimdReal::fromFloat(1.3f));
        hkVector4 vBack;
        vBack.setSubMul(vUp, fwdDir, hkSimdReal::fromFloat(1.1f));
        hkVector4 vLateral;
        vLateral.setMul(latDir, hkSimdReal::fromFloat(1.1f));

        suspension.m_wheelParams[0].m_hardpointChassisSpace.setSub(vFront, vLateral);
        suspension.m_wheelParams[1].m_hardpointChassisSpace.setAdd(vFront, vLateral);
        suspension.m_wheelParams[2].m_hardpointChassisSpace.setSub(vBack, vLateral);
        suspension.m_wheelParams[3].m_hardpointChassisSpace.setAdd(vBack, vLateral);
    }

    hkVector4 downDir;
    downDir.setNeg<3>(upDir);
    suspension.m_wheelParams[0].m_directionChassisSpace = downDir;
    suspension.m_wheelParams[1].m_directionChassisSpace = downDir;
    suspension.m_wheelParams[2].m_directionChassisSpace = downDir;
    suspension.m_wheelParams[3].m_directionChassisSpace = downDir;
}

void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hkVector4Parameter gravity, hknpVehicleDefaultAerodynamics& aerodynamics)
{
    aerodynamics.m_airDensity = 1.3f;
    // In m^2.
    aerodynamics.m_frontalArea = 0.6f;

    aerodynamics.m_dragCoefficient = 0.2f;
    aerodynamics.m_liftCoefficient = -0.3f;

    hkVector4 dir = gravity;
    dir.normalize<3>();

    aerodynamics.m_extraGravityws.setMul(dir, hkSimdReal::fromFloat(25.0f));
}

#endif


void NpVehicleSetup::setupComponent(const hknpVehicleData& data, hknpVehicleDefaultVelocityDamper& velocityDamper)
{
    velocityDamper.m_normalSpinDamping = 0.0f;
    velocityDamper.m_collisionSpinDamping = 4.0f;
    velocityDamper.m_collisionThreshold = 1.0f;
}

void NpVehicleSetup::setupWheelCollide(const hknpWorld* world, const hknpVehicleInstance& vehicle, hknpVehicleRayCastWheelCollide& wheelCollide)
{
    // Set the wheels to have the same collision filter info as the chassis.
    wheelCollide.m_wheelCollisionFilterInfo = world->getBody(vehicle.m_bodyId).m_collisionFilterInfo;
}

void NpVehicleSetup::setupWheelCollide(const hknpWorld* world, const hknpVehicleInstance& vehicle, hknpVehicleLinearCastWheelCollide& wheelCollide)
{
    // Set the wheels to have the same collision filter info as the chassis.
    wheelCollide.m_wheelCollisionFilterInfo = world->getBody(vehicle.m_bodyId).m_collisionFilterInfo;
}

void NpVehicleSetup::setupTyremarks(const hknpVehicleData& data, hknpTyremarksInfo& tyreMarks)
{
    tyreMarks.m_minTyremarkEnergy = 100.0f;
    tyreMarks.m_maxTyremarkEnergy = 1000.0f;
}



void NpVehicleSetup::UpdateCar(const hknpWorld* world, hknpVehicleInstance& vehicle)
{
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultAnalogDriverInput*>(vehicle.m_driverInput));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultSteering*>(vehicle.m_steering));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultEngine*>(vehicle.m_engine));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultTransmission*>(vehicle.m_transmission));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultBrake*>(vehicle.m_brake));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultSuspension*>(vehicle.m_suspension));
    setupComponent(*vehicle.m_data, world->getGravity(), *static_cast<hknpVehicleDefaultAerodynamics*>(vehicle.m_aerodynamics));
    setupComponent(*vehicle.m_data, *static_cast<hknpVehicleDefaultVelocityDamper*>(vehicle.m_velocityDamper));
}


hkRefPtr<hknpShape> HK_CALL makeCarChassisShape(float scale, const hkVector4& fwd, const hkVector4& up, const hkVector4& right)
{
    hkReal xSize = 1.75f * scale;
    hkReal ySize = 0.25f * scale;
    hkReal zSize = 1.1f * scale;

    hkReal xBumper = 1.9f * scale;
    hkReal yBumper = 0.15f * scale;
    hkReal zBumper = 1.0f * scale;

    hkReal xRoofFront = 0.4f * scale;
    hkReal xRoofBack = -1.0f * scale;
    hkReal yRoof = ySize + 0.45f * scale;
    hkReal zRoof = 0.7f * scale;

    hkReal xDoorFront = xRoofFront;
    hkReal xDoorBack = xRoofBack;
    hkReal yDoor = ySize;
    hkReal zDoor = zSize + 0.1f * scale;

    int numVertices = 22;

    // 16 = 4 (size of "each float group", 3 for x,y,z, 1 for padding) * 4 (size of float).
    int stride = sizeof(hkReal) * 4;

    HK_ALIGN_REAL(hkReal vertices[]) = {
        xSize, ySize, zSize, 0.0f,     // v0
        xSize, ySize, -zSize, 0.0f,    // v1
        xSize, -ySize, zSize, 0.0f,    // v2
        xSize, -ySize, -zSize, 0.0f,   // v3
        -xSize, -ySize, zSize, 0.0f,   // v4
        -xSize, -ySize, -zSize, 0.0f,  // v5

        xBumper, yBumper, zBumper, 0.0f,    // v6
        xBumper, yBumper, -zBumper, 0.0f,   // v7
        -xBumper, yBumper, zBumper, 0.0f,   // v8
        -xBumper, yBumper, -zBumper, 0.0f,  // v9

        xRoofFront, yRoof, zRoof, 0.0f,   // v10
        xRoofFront, yRoof, -zRoof, 0.0f,  // v11
        xRoofBack, yRoof, zRoof, 0.0f,    // v12
        xRoofBack, yRoof, -zRoof, 0.0f,   // v13

        xDoorFront, yDoor, zDoor, 0.0f,    // v14
        xDoorFront, yDoor, -zDoor, 0.0f,   // v15
        xDoorFront, -yDoor, zDoor, 0.0f,   // v16
        xDoorFront, -yDoor, -zDoor, 0.0f,  // v17

        xDoorBack, yDoor, zDoor, 0.0f,    // v18
        xDoorBack, yDoor, -zDoor, 0.0f,   // v19
        xDoorBack, -yDoor, zDoor, 0.0f,   // v20
        xDoorBack, -yDoor, -zDoor, 0.0f,  // v21
    };

    for (hkReal e : vertices)
    {
        e = e * 5;
    }

    // Rotate chassis vertices from x-fwd, y-up, z-right
    hkRotation orientation;
    orientation.setCols(fwd, up, right);
    for (int i = 0; i < numVertices; ++i)
    {
        hkVector4 v(vertices[i * 4], vertices[i * 4 + 1], vertices[i * 4 + 2]);
        v.setRotatedDir(orientation, v);
        vertices[i * 4] = v(0);
        vertices[i * 4 + 1] = v(1);
        vertices[i * 4 + 2] = v(2);
    }

    //
    // SHAPE CONSTRUCTION.
    //

    hkStridedVertices stridedVerts;
    stridedVerts.m_numVertices = numVertices;
    stridedVerts.m_striding = stride;
    stridedVerts.m_vertices = vertices;

    return hknpConvexShape::makeFromVertices(stridedVerts, 0.1f);
}

void HK_CALL createDisplayWheel(hkArray<hkRefPtr<hkDisplayGeometry>>& displayGeometry, const hkVector4& fwd, const hkVector4& right, hkReal radius, hkReal thickness)
{
    hkRefPtr<hknpShape> wheelShape = makeDisc(fwd, right, radius, thickness, 10);
    hknpShape::BuildSurfaceGeometryConfig config;
    config.m_radiusMode = hknpShape::CONVEX_RADIUS_DISPLAY_ROUNDED;
    hknpShape::SurfaceGeometry output;
    wheelShape->buildSurfaceGeometry(config, output);
    displayGeometry = output.m_surfaces;
}

void HK_CALL createDisplayWheels(hkUint64 displayId, hkReal radius, hkReal thickness, const hkVector4& fwd, const hkVector4& right)
{
    hkInplaceArray<hkRefPtr<hkDisplayGeometry>, 1> displayGeometry;

    createDisplayWheel(displayGeometry, fwd, right, radius, thickness);


    hkInplaceArray<hkDisplayGeometry*, 1> geomsAsPointers;
    for (const hkRefPtr<hkDisplayGeometry>& dg : displayGeometry)
    {
        geomsAsPointers.pushBack(dg.val());
    }
}

void HK_CALL createCustomWheels(const float X[], const float Y[], const float Z[], int VertCount, const int Triangles[], int TriCount)
{

    hkInplaceArray<hkRefPtr<hkDisplayGeometry>, 1> displayGeometry;

    hkGeometry geometry;
    geometry.clear();

    for (int i = 0; i < VertCount; i++)
        geometry.m_vertices.expandOne().set(X[i], Y[i], Z[i]);

    for (int i = 0; i < TriCount; i += 3)
        geometry.m_triangles.expandOne().set(Triangles[i], Triangles[i + 1], Triangles[i + 2]);

    hkRefPtr<hknpShape> wheelShape = hknpShape::makeMesh(geometry);
    hknpShape::BuildSurfaceGeometryConfig config;
    config.m_radiusMode = hknpShape::CONVEX_RADIUS_DISPLAY_ROUNDED;
    hknpShape::SurfaceGeometry output;
    wheelShape->buildSurfaceGeometry(config, output);
    displayGeometry = output.m_surfaces;


    hkInplaceArray<hkDisplayGeometry*, 1> geomsAsPointers;
    for (const hkRefPtr<hkDisplayGeometry>& dg : displayGeometry)
    {
        geomsAsPointers.pushBack(dg.val());
    }
}

hkRefPtr<hknpShape> HK_CALL makeDisc(const hkVector4& fwd, const hkVector4& right, hkReal radius, hkReal thickness, int numSides)
{
    hkArray<hkVector4> vertices;
    for (int i = 0; i < numSides; i++)
    {
        hkReal angle = HK_REAL_PI * 2 * i / (hkReal)numSides;
        hkQuaternion q(right, angle);
        vertices.expandOne().setRotatedDir(q, fwd * radius - right * thickness * 0.5f);
        vertices.expandOne().setRotatedDir(q, fwd * radius * 0.75f + right * thickness * 0.5f);
    }

    return hknpConvexShape::makeFromVertices(vertices);
}

void HK_CALL syncDisplayWheels(hknpVehicleInstance& vehicle, const hkArray<hkUint64>& wheels, hkVdbCmdTag tag, const hkVector4& up)
{
    hkQuaternion wheelFromDisplayWheel;
    wheelFromDisplayWheel.setAxisAngle(up, HK_REAL_PI);
    const hkVector4& chassisRight = vehicle.m_data->m_chassisOrientation.getColumn<2>();

    for (int i = 0; i < wheels.getSize(); i++)
    {
        hkVector4 worldFromWheelPos;
        hkQuaternion worldFromWheelRot;

        vehicle.calcCurrentPositionAndRotation(vehicle.getChassisTransform(),
            vehicle.m_suspension,
            i,
            worldFromWheelPos, worldFromWheelRot);

        // 'isLeftWheel' if on back-side of plane with 'right' as normal
        const hkVector4& hardPointChassis = vehicle.m_suspension->m_wheelParams[i].m_hardpointChassisSpace;
        const bool isLeftWheel = chassisRight.dot<3>(hardPointChassis) < 0;
        if (isLeftWheel)
        {
            worldFromWheelRot.mul(wheelFromDisplayWheel);
        }

        hkTransform worldFromDisplayWheel(worldFromWheelRot, worldFromWheelPos);
        //environment->m_displayHandler->updateGeometryTransform(wheels[i], worldFromDisplayWheel);
    }
}