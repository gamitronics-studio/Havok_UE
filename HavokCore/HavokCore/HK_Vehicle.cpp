#include "pch.h" 
#include "HK_Vehicle.h"

#include "VehicleCore.h"

#include <Physics/Physics/hknpProductFeatures.h>
#include <Physics/Physics/Collide/Shape/hknpShape.h>
#include <Physics/Physics/Dynamics/Material/hknpMaterialLibrary.h>
#include <Physics/Physics/Dynamics/World/hknpWorld.h>
#include <Physics/Physics/Dynamics/World/hknpWorldCinfo.h>
#include <Physics/Physics/Dynamics/Simulation/Multithreaded/hknpTaskGraph.h>
#include <Physics/Physics/Extensions/Viewers/hknpProcessContext.h>
#include <Physics/Physics/Extensions/Viewers/hknpViewerNames.h>

#include <Physics/Physics/Extensions/Actions/hknpActionManager.h>
#include <Physics\Physics\Extensions\Vehicle\Transmission\hknpVehicleTransmission.h>

hknpVehicleInstance* m_vehicle;

hkRefPtr<hknpWorld> m_npWorld;

hkArray<hkUint64> m_displayWheelId;

HK_Location hkLocationToFVector(hkVector4 tempL);
HK_Quaternion QuatToHKQuat(hkQuaternion tempQ);

HK_Car::HK_Car()
{

}

void HK_Car::HK_BuildCarMesh(const float X[], const float Y[], const float Z[], int VertCount, const int Triangles[], int TriCount)
{
    hkGeometry geometry;
    geometry.clear();

    for (int i = 0; i < VertCount; i++)
        geometry.m_vertices.expandOne().set(X[i], Y[i], Z[i]);

    for (int i = 0; i < TriCount; i += 3)
        geometry.m_triangles.expandOne().set(Triangles[i], Triangles[i + 1], Triangles[i + 2]);

    hkVector4 halfExtents(1, 1, 1);
    hkRefPtr<hknpShape> boxShape = hknpShape::makeBoxFromHalfExtents(halfExtents);

    //BodyShape = hknpShape::makeMesh(geometry);

    BodyShape = boxShape;
}

int HK_Car::HK_BuildCar(HK_CarSetup SetupData, const float X[], const float Y[], const float Z[], int VertCount, const int Triangles[], int TriCount)
{
    m_npWorld = GetWorld();

    CarData = SetupData;

    //____ Car Body____//    
    {
        hknpMaterialId chassisMaterialId;
        {
            hknpMaterial chassisMaterial;
            {
                chassisMaterial.m_dynamicFriction = 0.5f;
                chassisMaterial.m_restitution = 0.25f;
            }

            chassisMaterialId = GetMaterial(chassisMaterial);
        }

        hknpBodyCinfo chassisBodyInfo;
        {
            chassisBodyInfo.m_position = hkVector4(SetupData.SpawnLocation.x, SetupData.SpawnLocation.y, SetupData.SpawnLocation.z);

            hkGeometry geometry;
            geometry.clear();

            for (int i = 0; i < VertCount; i++)
                geometry.m_vertices.expandOne().set(X[i], Y[i], Z[i]);

            for (int i = 0; i < TriCount; i += 3)
                geometry.m_triangles.expandOne().set(Triangles[i], Triangles[i + 1], Triangles[i + 2]);

            hkVector4 halfExtents(1, 1, 1);
            hkRefPtr<hknpShape> boxShape = hknpShape::makeBoxFromHalfExtents(halfExtents);

            chassisBodyInfo.m_shape = bCustomCar ? hknpShape::makeMesh(geometry): makeCarChassisShape();
            chassisBodyInfo.m_materialId = chassisMaterialId;
            chassisBodyInfo.m_mass = SetupData.HK_Mass;
            chassisBodyInfo.m_motionType = hknpMotionType::DYNAMIC;
        }
         BodyID = new hknpBodyId();
        *BodyID = AddBody(chassisBodyInfo);
        m_npWorld->setBodyActivationControl(*BodyID, hknpActivationControl::ALWAYS_ACTIVE);
    }
 

    //____ Car Create____//
    {
        m_vehicle = new hknpVehicleInstance(*BodyID, m_npWorld);
        NpVehicleSetup setup;

        setup.SetupData = SetupData;

        hkRotation rot;
        rot.setCols(hkVector4(0, 1, 0), hkVector4(1, 0, 0), hkVector4(0, 0, 1));

        const bool useRayCast = true;
        setup.buildVehicle(m_npWorld, *m_vehicle, rot, useRayCast);

        m_npWorld->getActionManager()->addAction(m_vehicle);
        m_vehicle->removeReference();
    }

    //____Wheels____//
    hkReal radius = SetupData.HK_WheelRadius;
    hkReal thickness = SetupData.HK_WheelThickness;
    { 
        m_displayWheelId.setSize(4);

        for (int i = 0; i < m_displayWheelId.getSize(); i++)
        {
            hkUint64 displayId = (i * 2) + 301;
            const hknpVehicleData::WheelComponentParams& params = m_vehicle->m_data->m_wheelParams[i];

            //____Wheel____//
            hkInplaceArray<hkRefPtr<hkDisplayGeometry>, 1> displayGeometry;
                        
            createDisplayWheels(displayId, params.m_radius, params.m_width, hkVector4(1,0,0), hkVector4(0,0,1));
            
            hkInplaceArray<hkDisplayGeometry*, 1> geomsAsPointers;
            for (const hkRefPtr<hkDisplayGeometry>& dg : displayGeometry)
            {
                geomsAsPointers.pushBack(dg.val());
            }
            //____ ____//

            m_displayWheelId[i] = displayId;
        }
    }

    //____ Car Add to World ____//
    hkVector4 m_vehiclePosition = hkVector4(CarData.SpawnLocation.x, CarData.SpawnLocation.y, CarData.SpawnLocation.z);
    m_npWorld->setBodyPosition(*BodyID, m_vehiclePosition);

    return 200;
}

hknpVehicleDriverInputAnalogStatus* deviceStatus;

void HK_Car::HK_Step(float Move, float Steer, bool Reverse, bool HBrake, float T)
{
    ((hknpVehicleDriverInputAnalogStatus*)m_vehicle->m_deviceStatus)->m_positionY = Move;
    ((hknpVehicleDriverInputAnalogStatus*)m_vehicle->m_deviceStatus)->m_positionX = Steer;

    ((hknpVehicleDriverInputAnalogStatus*)m_vehicle->m_deviceStatus)->m_reverseButtonPressed = Reverse;

    ((hknpVehicleDriverInputAnalogStatus*)m_vehicle->m_deviceStatus)->m_handbrakeButtonPressed = HBrake;
}

void HK_Car::HK_ResetCar()
{
    hkVector4 ResetLoc = hkVector4(CarData.SpawnLocation.x, CarData.SpawnLocation.y, CarData.SpawnLocation.z);
    hkTransform Reset;
    const hkRotation Rot = hkRotation();
    Reset.setTranslation(hkVector4(CarData.SpawnLocation.x, CarData.SpawnLocation.y, CarData.SpawnLocation.z));
    Reset.setRotation(Rot);
    //hkRotationf ResetRot;
    //ActivationMode mode = 
    //m_npWorld->setBodyTransform(*BodyID, ResetLoc, ResetRot, ActivationMode::ACTIVATE);
    GetWorld()->setBodyTransform(*BodyID, Reset);
}

HK_Location HK_Car::HK_GetCarLocation()
{
    return GetBodyLocation(*BodyID);
}

HK_Quaternion HK_Car::HK_GetCarRotation()
{
    return GetBodyRotation(*BodyID);
}

void HK_Car::HK_UpdateWheels(HK_Location& T1_L, HK_Quaternion& T1_Q, HK_Location& T2_L, HK_Quaternion& T2_Q, HK_Location& T3_L, HK_Quaternion& T3_Q, HK_Location& T4_L, HK_Quaternion& T4_Q)
{
   hkVector4 T1L, T2L, T3L, T4L;
   hkQuaternion T1Q, T2Q, T3Q, T4Q;

   m_vehicle->calcCurrentPositionAndRotation(m_vehicle->getChassisTransform(), m_vehicle->m_suspension, 0, T1L, T1Q);
   m_vehicle->calcCurrentPositionAndRotation(m_vehicle->getChassisTransform(), m_vehicle->m_suspension, 1, T2L, T2Q);
   m_vehicle->calcCurrentPositionAndRotation(m_vehicle->getChassisTransform(), m_vehicle->m_suspension, 2, T3L, T3Q);
   m_vehicle->calcCurrentPositionAndRotation(m_vehicle->getChassisTransform(), m_vehicle->m_suspension, 3, T4L, T4Q);

   T1_L = hkLocationToFVector(T1L);
   T1_L = hkLocationToFVector(T1L);
   T1_L = hkLocationToFVector(T1L);
   T1_L = hkLocationToFVector(T1L);

   T1_Q = QuatToHKQuat(T1Q);
   T2_Q = QuatToHKQuat(T2Q);
   T3_Q = QuatToHKQuat(T3Q);
   T4_Q = QuatToHKQuat(T4Q);

}

HK_CarStats HK_Car::HK_GetCarStats()
{
    HK_CarStats stats;
    stats.RPM = m_vehicle->calcRPM();
    stats.KMPH = m_vehicle->calcKMPH();
    stats.Gear = m_vehicle->m_currentGear;
    stats.isReverse = m_vehicle->m_isReversing;
    stats.Torque = m_vehicle->m_torque;
    //stats.SteerAngle = m_vehicle->m_wheelsSteeringAngle;

    return stats;
}

HK_Location hkLocationToFVector(hkVector4 tempL)
{
    HK_Location Loc;
    Loc.x = tempL(0);
    Loc.y = tempL(1);
    Loc.z = tempL(2);
    return Loc;
}

HK_Quaternion QuatToHKQuat(hkQuaternion tempQ)
{
    HK_Quaternion Q;
    Q.x = tempQ(0);
    Q.y = tempQ(1);
    Q.z = tempQ(2);
    Q.w = tempQ(3);

    return Q;
}

