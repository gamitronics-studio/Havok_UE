using System.IO;
using UnrealBuildTool;

public class HavokCore : ModuleRules
{
    private void CopyToBinaries(string Filepath, ReadOnlyTargetRules Target)
    {
        string DirBinaries = Path.Combine(Directory.GetParent(ModuleDirectory).Parent.ToString(), "Binaries", Target.Platform.ToString());
        string filename = Path.GetFileName(Filepath);

        if (!Directory.Exists(DirBinaries))
            Directory.CreateDirectory(DirBinaries);

        if (!File.Exists(Path.Combine(DirBinaries, filename)))
            File.Copy(Filepath, Path.Combine(DirBinaries, filename), true);
    }

    public HavokCore(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core", "CoreUObject", "Engine",
		});

        PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "ThirdParty", "HavokCore.lib"));

		string Path_Havok = (Path.Combine(ModuleDirectory, "ThirdParty", "HavokCore.lib"));
        RuntimeDependencies.Add(Path_Havok);

        PublicDelayLoadDLLs.Add("HavokCore.dll");

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "ThirdParty"));
        PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "ThirdParty", "HavokCore.lib"));

        CopyToBinaries(Path.Combine(ModuleDirectory, "ThirdParty", "HavokCore.dll"), Target);

    }
}