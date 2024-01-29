using UnrealBuildTool;

public class UnrealHavok : ModuleRules
{
	public UnrealHavok(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore" });
        PublicDependencyModuleNames.Add("HavokCore");

        PrivateDependencyModuleNames.AddRange(new string[] {  });

	}
}
