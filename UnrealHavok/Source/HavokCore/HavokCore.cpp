#include "HavokCore.h"

IMPLEMENT_MODULE(FHavokCoreModule, HavokCore);

void FHavokCoreModule::StartupModule()
{
	const FString LibPath = FPaths::Combine(FPaths::ProjectDir(), TEXT("Binaries/Win64/HavokCore.dll"));

	DynamicLibExampleHandle = FPlatformProcess::GetDllHandle(*LibPath);

	if (DynamicLibExampleHandle != nullptr)
	{
		UE_LOG(LogTemp, Log, TEXT("HavokCore.dll loaded successfully! %s"), *LibPath);
	}
	else
	{
		UE_LOG(LogTemp, Fatal, TEXT("HavokCore.dll failed to load! %s"), *LibPath);
	}
}

void FHavokCoreModule::ShutdownModule()
{
	FPlatformProcess::FreeDllHandle(DynamicLibExampleHandle);
	DynamicLibExampleHandle = nullptr;

	UE_LOG(LogTemp, Fatal, TEXT("HavokCore.dll unloaded"));
}