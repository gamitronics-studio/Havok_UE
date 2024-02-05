#pragma once
#include "pch.h" 

#include "Core.h"

#include "string.h"
#include<iostream>

#pragma region HavokHeaders

#include <Common/Base/hkBase.h>
#include <Common/Base/System/hkBaseSystem.h>
#include <Common/Base/System/Hardware/hkHardwareInfo.h>
#include <Common/Base/System/Init/PlatformInit.h>
#include <Common/Base/Container/BlockStream/Allocator/Dynamic/hkDynamicBlockStreamAllocator.h>
#include <Common/Base/Thread/Thread/hkThread.h>
#include <Common/Base/Thread/Semaphore/hkSemaphore.h>
#include <Common/Base/Thread/TaskQueue/Default/hkDefaultTaskQueue.h>
#include <Common/Visualize/hkVisualDebugger.h>
#include <Common/Visualize/Process/hkCommonProcessContext.h>

#include <Physics/Physics/hknpProductFeatures.h>
#include <Physics/Physics/Collide/Shape/hknpShape.h>
#include <Physics/Physics/Dynamics/Material/hknpMaterialLibrary.h>
#include <Physics/Physics/Dynamics/World/hknpWorld.h>
#include <Physics/Physics/Dynamics/World/hknpWorldCinfo.h>
#include <Physics/Physics/Dynamics/Simulation/Multithreaded/hknpTaskGraph.h>
#include <Physics/Physics/Extensions/Viewers/hknpProcessContext.h>
#include <Physics/Physics/Extensions/Viewers/hknpViewerNames.h>

#include <Common/Base/System/Main/hkMain.cxx>
#include <Common/Base/System/Error/hkErrorReport.cxx>

#include <Common/Base/hkKeycodeManager.cxx>

#pragma endregion

bool SimulationOn = false;

int hkMain(int argc, const char** argv)
{
    return 0;
}

HardwareData HardwareInfo()
{
    HardwareData HardData;

    hkBaseSystem::Cinfo baseSystemCinfo = hkBaseSystem::Cinfo::getDefault(ErrorReport);
    hkBaseSystem::init(baseSystemCinfo);

    hkHardwareInfo hwInfo;

    HardData.NumOfCores = hwInfo.getNumHardwareThreads();
    HardData.ArchName = hwInfo.getArchitectureName();
    HardData.CpuName = hwInfo.getCpuName();
    HardData.TotalRAM = hwInfo.getTotalRam();

    return HardData;
}

#pragma region WorldControls

#define msize _msize

hkLog::RegisteredOrigin s_demoLogOrigin("demo");

void CoreLoop(hkRefPtr<hknpWorld>, hkReal, hknpBodyId);

hkRefPtr<hkCommonProcessContext> commonContext;
hkRefPtr<hknpProcessContext> npContext;
hkRefPtr<hkVisualDebugger> vdbServer;

hkScopedPtr<hkThread> m_thread;
hkTaskQueue* m_taskQueue;
hkTaskQueue::Handle m_handleToWaitFor;
hkSemaphore m_workAvailable;
hkSemaphore m_workFinished;
hk::atomic<hkMonitorStream*> m_monitorStream{ nullptr };

hkRefPtr<hknpWorld> world;

struct WorkerThread
{
public:
    HK_DECLARE_CLASS(WorkerThread, New);

    WorkerThread(hkTaskQueue* taskQueue) : m_taskQueue(taskQueue)
    {
        m_thread = hkThread::make([this]
            {
                m_monitorStream.store(hkMonitorStream::getInstancePtr(), hk::memory_order_relaxed);

                while (1)
                {
                    m_workAvailable.acquire();
                    if (m_taskQueue)
                    {
                        m_taskQueue->processUntilFinished(m_handleToWaitFor);
                    }
                    else
                    {
                        break;
                    }
                    m_workFinished.release();
                } });


        while (!m_monitorStream.load(hk::memory_order_relaxed))
        {
        }
    }

    ~WorkerThread()
    {
        m_taskQueue = nullptr;
        m_workAvailable.release();
    }

    hkMonitorStream* getMonitorStream() const
    {
        return m_monitorStream.load(hk::memory_order_relaxed);
    }


    void processTasks(const hkTaskQueue::Handle& finalHandle)
    {
        m_handleToWaitFor = finalHandle;
        m_workAvailable.release();
    }

    void waitUntilFinished()
    {
        m_workFinished.acquire();
    }

protected:
    hkScopedPtr<hkThread> m_thread;
    hkTaskQueue* m_taskQueue;
    hkTaskQueue::Handle m_handleToWaitFor;
    hkSemaphore m_workAvailable;
    hkSemaphore m_workFinished;
    hk::atomic<hkMonitorStream*> m_monitorStream{ nullptr };
};

#pragma endregion

void StartPhysics(Gravity g, float fps)
{
    SimulationOn = true;

    const hkReal duration = 100.0f;

    hkBaseSystem::Cinfo baseSystemCinfo = hkBaseSystem::Cinfo::getDefault(ErrorReport);
    hkBaseSystem::init(baseSystemCinfo);

    hkProductFeatures::Physics::All::request();

    hkDynamicBlockStreamAllocator blockStreamAllocator(1024 * 1024, hkDynamicBlockStreamAllocator::KEEP_BLOCKS);

    //____ World____//
    
    {
        hknpWorldCinfo info;
        info.m_gravity.set(g.x, g.y, g.z);
        info.m_blockStreamAllocator = &blockStreamAllocator;

        world = hk::makeRef<hknpWorld>(info);
    }

    //____Visual Debugger____//
    hkRefPtr<hkCommonProcessContext> commonContext;
    hkRefPtr<hknpProcessContext> npContext;
    hkRefPtr<hkVisualDebugger> vdbServer;
    {
        hknpProcessContext::registerAllProcesses();

        commonContext = hk::makeRef<hkCommonProcessContext>();
        npContext = hk::makeRef<hknpProcessContext>();
        npContext->addWorld(world);

        hkArray<hkProcessContext*> contexts;
        contexts.pushBack(commonContext);
        contexts.pushBack(npContext);
        vdbServer = hk::makeRef<hkVisualDebugger>(contexts);
        vdbServer->addDefaultProcess(HKNP_SHAPE_VIEWER_NAME);  // enable the shape viewer by default

        hkVisualDebugger::ServeInput vdbServeInput;
        vdbServeInput.m_advertiseDescription = "PhysicsExample";
        vdbServer->serve(vdbServeInput);
    }

    //____Worker Thread Setup____//
    hkArray<hkScopedPtr<WorkerThread>> workerThreads;
    hkDefaultTaskQueue taskQueue;
    const int numThreads = hkMath::min2(hkHardwareInfo::getNumHardwareThreads(), taskQueue.getMaxNumSupportedThreads());
    for (int i = 0; i < numThreads - 1; i++)  // skip main thread
    {
        workerThreads.emplaceBack(hk::makeUnique<WorkerThread>(&taskQueue));
    }

    hkArray<hkMonitorStream*> monitorStreams;
    monitorStreams.pushBack(hkMonitorStream::getInstancePtr());
    for (hkScopedPtr<WorkerThread>& workerThread : workerThreads)
    {
        monitorStreams.pushBack(workerThread->getMonitorStream());
    }

    for (hkMonitorStream* monitorStream : monitorStreams)
    {
        monitorStream->resize(100 * 1024);  // 100KiB capacity
    }

    hknpStepInput stepInput;
    stepInput.m_deltaTime = 1.0f / fps;                 
    stepInput.m_numThreads = workerThreads.getSize() + 1;

    hkStopwatch stopWatch;
    stopWatch.start();
    hkReal lastTime = stopWatch.getElapsedSeconds();

    //____ Simulation loop____//
    const int numSteps = int(duration / stepInput.m_deltaTime);
    while (SimulationOn)
    {
        // Clear all profiling data
        for (hkMonitorStream* monitorStream : monitorStreams)
        {
            monitorStream->reset();
        }

        {
            {
                hknpTaskGraph taskGraph;
                world->generateCollideTasks(stepInput, taskGraph);

                hkTaskQueue::Handle finalHandle = taskGraph.submitToTaskQueue(taskQueue);
                for (hkScopedPtr<WorkerThread>& workerThread : workerThreads)
                {
                    workerThread->processTasks(finalHandle);
                }
                taskQueue.processUntilFinished(finalHandle);
                for (hkScopedPtr<WorkerThread>& workerThread : workerThreads)
                {
                    workerThread->waitUntilFinished();
                }
                taskGraph.freeTaskQueueHandles(taskQueue);
            };

            // Perform solving and integration.
            {
                hknpTaskGraph taskGraph;
                world->generateSolveTasks(taskGraph);

                hkTaskQueue::Handle finalHandle = taskGraph.submitToTaskQueue(taskQueue);
                for (hkScopedPtr<WorkerThread>& workerThread : workerThreads)
                {
                    workerThread->processTasks(finalHandle);
                }
                taskQueue.processUntilFinished(finalHandle);
                for (hkScopedPtr<WorkerThread>& workerThread : workerThreads)
                {
                    workerThread->waitUntilFinished();
                }
                taskGraph.freeTaskQueueHandles(taskQueue);
            }
        }

        {
            commonContext->m_monitorStreams.setSize(monitorStreams.getSize());
            for (int i = 0; i < monitorStreams.getSize(); i++)
            {
                commonContext->m_monitorStreams[i] = monitorStreams[i]->getDataView();
            }
            vdbServer->step(stepInput.m_deltaTime * 1000.0f);
        }

        // Wait until the actual time has passed
        while (stopWatch.getElapsedSeconds() < lastTime + stepInput.m_deltaTime)
        {
        }
        lastTime += stepInput.m_deltaTime;
    }
  stopWatch.stop();
  HK_ClearWorld();
}

void StopPhysics()
{
    SimulationOn = false;
}

void QuitPhysics()
{
    hkBaseSystem::quit();
}

#include <Common/Base/System/Init/PlatformInit.cxx>

//____Floor____//
void HK_AddFloor(float X, float Z)
{

    hkVector4 halfExtents(X, 0.5f, Z);
    hkRefPtr<hknpShape> boxShape = hknpShape::makeBoxFromHalfExtents(halfExtents);

    hknpBodyCinfo bodyCinfo;
    bodyCinfo.m_shape = boxShape;

    hknpBodyId bodyId = world->allocateBody(bodyCinfo);
    world->addBody(bodyId);

}

void HK_AddBlock(float X, float Y, float Z, HK_Location loc, HK_Quaternion quat)
{
    hkVector4 halfExtents(X, Y, Z);
    hkRefPtr<hknpShape> boxShape = hknpShape::makeBoxFromHalfExtents(halfExtents);

    hknpBodyCinfo bodyCinfo;
    bodyCinfo.m_shape = boxShape;


    bodyCinfo.m_position.set(loc.x, loc.y, loc.z);
    bodyCinfo.m_orientation.set(quat.x, quat.y, quat.z, quat.w);

    hknpBodyId bodyId = world->allocateBody(bodyCinfo);

    world->addBody(bodyId);
}

void HK_AddMesh(const float X[], const float Y[], const float Z[], int VertCount, const int Triangles[], int TriCount, HK_Location Location, HK_Quaternion Rotation)
{
    hkGeometry geometry;
    geometry.clear();

    for (int i = 0; i < VertCount; i++)
        geometry.m_vertices.expandOne().set(X[i], Y[i], Z[i]);

    for (int i = 0; i < TriCount; i += 3)
        geometry.m_triangles.expandOne().set(Triangles[i], Triangles[i + 1], Triangles[i + 2]);

    hknpBodyCinfo Cinfo;
    {
        Cinfo.m_position.set(Location.x, Location.y, Location.z);
        Cinfo.m_orientation.set(Rotation.x, Rotation.y, Rotation.z, Rotation.w);
        Cinfo.m_shape = hknpShape::makeMesh(geometry);
    }

    hknpBodyId bodyId = world->allocateBody(Cinfo);
    world->addBody(bodyId);
}

void DLL_EXPORT HK_TestMesh()
{
    hkGeometry geometry;
    geometry.m_vertices.setSize(4);

    geometry.m_vertices[0] = hkVector4(40.0f, 40.0f, 0.0f);
    geometry.m_vertices[1] = hkVector4(-40.0f, 40.0f, 0.0f);
    geometry.m_vertices[2] = hkVector4(-40.0f, -40.0f, 0.0f);
    geometry.m_vertices[3] = hkVector4(40.0f, -40.0f, 0.0f);

    geometry.m_triangles.setSize(2);
    geometry.m_triangles[0].set(0, 1, 2);
    geometry.m_triangles[1].set(0, 2, 3);

    hknpBodyCinfo info;
    info.m_position.set(1, 1, 1);
    info.m_shape = hknpShape::makeMesh(geometry);

    hknpBodyId bodyId = world->allocateBody(info);
    world->addBody(bodyId);
}

hknpBodyId AddBody(hknpBodyCinfo BodyInfo)
{
    hknpBodyId BodyId = world->allocateBody(BodyInfo);
    world->addBody(BodyId);

    return BodyId;
}

bool RemoveBody(hknpBodyId)
{
    return false;
}

int HK_ClearWorld()
{
    int Removed = -1;
    hknpBodyIdRange IDs = world->getBodyIds();;
    hkUint32 NumOfBodies = world->getNumBodies();

    for (const hknpBodyId& body : IDs)
    {
        world->destroyBodies(&body, 1);
    }

    return NumOfBodies;
}

hknpMaterialId GetMaterial(hknpMaterial Material)
{
    return world->accessMaterialLibrary()->addEntry(Material);
}

HK_Location GetBodyLocation(hknpBodyId BodyID)
{
    HK_Location Loc = HK_Location(0.0f, 0.0f, 0.0f);;
    hkVector4 pos = world->getBodyTransform(BodyID).getTranslation();

    Loc.x = pos(0);
    Loc.y = pos(2);
    Loc.z = pos(1);

    return Loc;
}

HK_Quaternion GetBodyRotation(hknpBodyId BodyID)
{
    hkQuaternion Q;
    HK_Quaternion Rot;
    hkRotation rot = world->getBodyTransform(BodyID).getRotation();
    Q.set(rot);

    //Q.getAxis(Rot);

    Rot.x = Q(0);
    Rot.y = Q(2);
    Rot.z = Q(1);
    Rot.w = Q(3);

    return Rot;
}

hkRefPtr<hknpWorld> GetWorld()
{
    return world;
}

float GetBodyAngle(hknpBodyId BodyID)
{
    float Angle;
    hkQuaternion Q;
    hkRotation rot = world->getBodyTransform(BodyID).getRotation();
    Q.set(rot);
    Angle =  Q.getAngle();

    return Angle;
}
