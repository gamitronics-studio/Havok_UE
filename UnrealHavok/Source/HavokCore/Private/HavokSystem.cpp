#include "HavokSystem.h"
THIRD_PARTY_INCLUDES_START
#include "../ThirdParty/Core.h"
THIRD_PARTY_INCLUDES_END



class FHavokThread : public FRunnable
{
    FRunnableThread* Thread;
    bool bRunThread;

    Gravity worldGravity;
    float FPS;

public:

    FHavokThread()
    {
        Thread = FRunnableThread::Create(this, TEXT("MyThread"));
        UE_LOG(LogTemp, Warning, TEXT("Threading...."));
    };

    FHavokThread(FVector gravity, float fps)
    {
        worldGravity.x = gravity.X;
        worldGravity.y = gravity.Y;
        worldGravity.x = gravity.Z;
        FPS = fps;

        Thread = FRunnableThread::Create(this, TEXT("MyThread"));
        UE_LOG(LogTemp, Warning, TEXT("Threading with Gravity...."));
    };


    virtual uint32 Run() override
    {
        while (!bShutdown)
        {
            UE_LOG(LogTemp, Warning, TEXT("Thread running...."));
            UE_LOG(LogTemp, Warning, TEXT("Physics -- Start --"));
            StartPhysics(worldGravity, FPS);
        }
        return 0;
    }

    virtual void Stop() override
    {
        StopPhysics();
        bShutdown = true;
        UE_LOG(LogTemp, Warning, TEXT("Thread stopped..."));
    }

    bool bShutdown = false;
};


FHardwareData UHavokSystem::GetHardwareInfo()
{
    FHardwareData HardwareD;

    HardwareData HD = HardwareInfo();

    HardwareD.ArchName = HD.ArchName;
    HardwareD.CPU = HD.CpuName;
    HardwareD.NumOfCores = HD.NumOfCores;
    HardwareD.RAM = (HD.TotalRAM)/(1024*1024*1024);

	return HardwareD;
}

FHavokThread* Thread;

void UHavokSystem::PhysicsStart(FVector gravity, float fps)
{
    if (!bPhysicsIsOn)
    {
        UE_LOG(LogTemp, Warning, TEXT("Physics -- Init --"));
        bPhysicsIsOn = true;
        Thread = new FHavokThread(gravity, fps);
    }
    OnPhysicsOn.Broadcast();
}

void UHavokSystem::PhysicsStop()
{
    UE_LOG(LogTemp, Warning, TEXT("Physics -- Stopping --"));
    StopPhysics();

    return;
    if (Thread != nullptr)
    {
        UE_LOG(LogTemp, Warning, TEXT("Thread stopping..."));
        //Thread->WaitForCompletion();
        Thread->bShutdown = true;
        delete Thread;
        Thread = nullptr;
    }
}

void UHavokSystem::PhysicsQuit()
{
    void QuitPhysics();
}

int UHavokSystem::ClearWorld()
{
    OnWorldCleared.Broadcast();
    return HK_ClearWorld();
}

void UHavokSystem::AddFloor(float X, float Y)
{
    HK_AddFloor(X, Y);
}

void UHavokSystem::AddBlock(float X, float Y, float Z, FVector Loc, FQuat Rot)
{
    HK_Location tempLoc;
    tempLoc.x = Loc.X;
    tempLoc.y = Loc.Z;
    tempLoc.z = Loc.Y;

    HK_Quaternion tempQ;
    tempQ.x = Rot.X;
    tempQ.y = Rot.Z;
    tempQ.z = Rot.Y;
    tempQ.w = Rot.W;

    HK_AddBlock(X, Y, Z, tempLoc, tempQ);
}

void UHavokSystem::CreateMesh(const TArray<float>& X, const TArray<float>& Y, const TArray<float>& Z, const TArray<int>& Triangles, FVector Location, FQuat Rotation)
{
    
    HK_Location tempLoc;
    tempLoc.x = Location.X;
    tempLoc.y = Location.Z;
    tempLoc.z = Location.Y;

    HK_Quaternion tempQ;
    tempQ.x = Rotation.X;
    tempQ.y = Rotation.Z;
    tempQ.z = Rotation.Y;
    tempQ.w = Rotation.W;

    HK_AddMesh(X.GetData(), Z.GetData(), Y.GetData(), X.Num(), Triangles.GetData(), Triangles.Num(), tempLoc, tempQ);
}

void UHavokSystem::TestMesh()
{
    HK_TestMesh();
}
