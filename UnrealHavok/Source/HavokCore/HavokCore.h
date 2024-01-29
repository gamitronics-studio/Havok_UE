#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"


class FHavokCoreModule : public IModuleInterface
{
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

    void* DynamicLibExampleHandle;
};