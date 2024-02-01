#include "HavokVehicle.h"
THIRD_PARTY_INCLUDES_START
#include "../ThirdParty/Core.h"
THIRD_PARTY_INCLUDES_END

FVector hkLocationToFVector(HK_Location);
FQuat hkQuatToQuat(HK_Quaternion);

HK_CarSetup UEtoHK_CarSetup(FCarSetup);
FCarSetup HKtoUE_CarSetup(HK_CarSetup);

AHK_Vehicle::AHK_Vehicle()
{

}

void AHK_Vehicle::SpawnCar(FCarSetup SetupData, const TArray<float>& X, const TArray<float>& Y, const TArray<float>& Z, const TArray<int>& Triangles, FMeshData WheelMesh)
{
	Car = new HK_Car();
	
	HK_CarSetup hkSetup; // = UEtoHK_CarSetup(SetupData);




	//Car->bCustomCar = SetupData.bUseCustomCar;
	hkSetup.HK_bCustomCar = true; // SetupData.bUseCustomCar;

	Car->HK_BuildCar(hkSetup, X.GetData(), Z.GetData(), Y.GetData(), X.Num(), Triangles.GetData(), Triangles.Num(), WheelMesh.X.GetData(), WheelMesh.Z.GetData(), WheelMesh.Y.GetData(), WheelMesh.X.Num(), WheelMesh.Triangles.GetData(), WheelMesh.Triangles.Num());
}

FVector AHK_Vehicle::GetCarLoc()
{
	FVector Loc;
	HK_Location tempLoc = Car->HK_GetCarLocation();
	Loc.X = tempLoc.x;
	Loc.Y = tempLoc.y;
	Loc.Z = tempLoc.z;
	return Loc;
}

FRotator AHK_Vehicle::GetCarRot()
{
	FRotator Rot;
	HK_Quaternion FQ = Car->HK_GetCarRotation();

	FQuat Q(FQ.x, FQ.y, FQ.z, FQ.w);

	return Q.Rotator();
}

void AHK_Vehicle::StepCar(float WS, float AD, bool R, bool B, float T)
{
	Car->HK_Step(WS, AD, R, B, T);
}

void AHK_Vehicle::UpdateWheels(FVector& T1L, FQuat& T1Q, FVector& T2L, FQuat& T2Q, FVector& T3L, FQuat& T3Q, FVector& T4L, FQuat& T4Q)
{
	HK_Location T1_L, T2_L, T3_L, T4_L;
	HK_Quaternion T1_Q, T2_Q, T3_Q, T4_Q;
	Car->HK_UpdateWheels(T1_L, T1_Q, T2_L, T2_Q, T3_L, T3_Q, T4_L, T4_Q);

	T1Q = hkQuatToQuat(T1_Q);
	T2Q = hkQuatToQuat(T2_Q);
	T3Q = hkQuatToQuat(T3_Q);
	T4Q = hkQuatToQuat(T4_Q);

	T1L = hkLocationToFVector(T1_L);
	T2L = hkLocationToFVector(T2_L);
	T3L = hkLocationToFVector(T3_L);
	T4L = hkLocationToFVector(T4_L);
}

void AHK_Vehicle::GetCarStats(float& RPM, float& KMPH, int& Gear, bool& isReverse, float& Torque, float& SteerAngle)
{
	HK_CarStats Stats = Car->HK_GetCarStats();
	RPM = Stats.RPM;
	KMPH = Stats.KMPH;
	Gear = Stats.Gear;
	isReverse = Stats.isReverse;
	Torque = Stats.Torque;
	SteerAngle = Stats.SteerAngle;
}

void AHK_Vehicle::ResetCar()
{
	Car->HK_ResetCar();
}

void AHK_Vehicle::UpdateCar(FCarSetup SetupData)
{
	HK_CarSetup hkSetup = UEtoHK_CarSetup(SetupData);;

	Car->UpdateCar(hkSetup);
}

FCarSetup AHK_Vehicle::GetCatSetup()
{
	return HKtoUE_CarSetup(Car->HK_GetCarSetup());
}

FQuat hkQuatToQuat(HK_Quaternion FQ)
{
	return FQuat(FQ.x, FQ.y, FQ.z, FQ.w);
}

FVector hkLocationToFVector(HK_Location tempLoc)
{
	FVector Loc;
	Loc.X = tempLoc.x;
	Loc.Y = tempLoc.y;
	Loc.Z = tempLoc.z;
	return Loc;
}

HK_CarSetup UEtoHK_CarSetup(FCarSetup SetupData)
{
	HK_CarSetup hkSetup;

	hkSetup.SpawnLocation.x = SetupData.SpawnLocation.X;
	hkSetup.SpawnLocation.y = SetupData.SpawnLocation.Z;
	hkSetup.SpawnLocation.z = SetupData.SpawnLocation.Y;

	hkSetup.HK_Mass = SetupData.Mass;
	hkSetup.HK_maxSpeed = SetupData.maxSpeed;

	hkSetup.HK_Torque = SetupData.Torque;

	hkSetup.HK_minRPM = SetupData.minRPM;
	hkSetup.HK_maxRPM = SetupData.maxRPM;
	hkSetup.HK_optRPM = SetupData.optRPM;

	hkSetup.HK_DownShiftAtRPM = SetupData.DownShiftAtRPM;
	hkSetup.HK_UpShiftAtRPM = SetupData.UpShiftAtRPM;
	hkSetup.HK_ReverseGearRatio = SetupData.ReverseGearRatio;
	hkSetup.HK_GearRatios[0] = SetupData.GearRatios[0];
	hkSetup.HK_GearRatios[1] = SetupData.GearRatios[1];
	hkSetup.HK_GearRatios[2] = SetupData.GearRatios[2];
	hkSetup.HK_GearRatios[3] = SetupData.GearRatios[3];
	hkSetup.HK_GearRatios[4] = SetupData.GearRatios[4];
	hkSetup.HK_GearRatios[5] = SetupData.GearRatios[5];

	hkSetup.HK_WheelRadius = SetupData.WheelRadius;
	hkSetup.HK_WheelThickness = SetupData.WheelThickness;
	hkSetup.HK_WheelFriction = SetupData.WheelFriction;
	hkSetup.HK_WheelMass = SetupData.WheelMass;

	hkSetup.HK_BrakeTorque = SetupData.BrakeTorque;

	hkSetup.HK_liftCoefficient = SetupData.liftCoefficient;

	hkSetup.HK_maxSteer = SetupData.maxSteer;
	hkSetup.HK_SteerBackWheels = SetupData.SteerBackWheels;

	hkSetup.HK_SuspensionLength = SetupData.SuspensionLength;
	hkSetup.HK_SuspensionStrength = SetupData.SuspensionStrength;

	hkSetup.Up = SetupData.U; hkSetup.Front = SetupData.F; hkSetup.Back = SetupData.B; hkSetup.Lateral = SetupData.L;

	hkSetup.RTWheels = SetupData.bUseRayTrace;
	hkSetup.BodyScale = SetupData.bBodyScale;

	hkSetup.HK_normalSpinDamping = SetupData.NormalSpinDamping;
	hkSetup.HK_collisionSpinDamping = SetupData.CollisionSpinDamping;
	hkSetup.HK_collisionThreshold = SetupData.CollisionThreshold;

	hkSetup.UseCustomWheel = SetupData.bUseCustomWheel;
	hkSetup.HK_bCustomCar = SetupData.bUseCustomCar;

	return hkSetup;
}

FCarSetup HKtoUE_CarSetup(HK_CarSetup hkSetup)
{
	FCarSetup SetupData;

	SetupData.SpawnLocation.X = hkSetup.SpawnLocation.x;
	SetupData.SpawnLocation.Z = hkSetup.SpawnLocation.y;
	SetupData.SpawnLocation.Y = hkSetup.SpawnLocation.z;

	SetupData.Mass = hkSetup.HK_Mass;
	SetupData.maxSpeed = hkSetup.HK_maxSpeed;

	SetupData.Torque = hkSetup.HK_Torque;

	SetupData.minRPM = hkSetup.HK_minRPM;
	SetupData.maxRPM = hkSetup.HK_maxRPM;
	SetupData.optRPM = hkSetup.HK_optRPM;

	SetupData.DownShiftAtRPM = hkSetup.HK_DownShiftAtRPM;
	SetupData.UpShiftAtRPM = hkSetup.HK_UpShiftAtRPM;
	SetupData.ReverseGearRatio = hkSetup.HK_ReverseGearRatio;

	/*
	SetupData.GearRatios[0] = hkSetup.HK_GearRatios[0];
	SetupData.GearRatios[1] = hkSetup.HK_GearRatios[1];
	SetupData.GearRatios[2] = hkSetup.HK_GearRatios[2];
	SetupData.GearRatios[3] = hkSetup.HK_GearRatios[3];
	SetupData.GearRatios[4] = hkSetup.HK_GearRatios[4];
	SetupData.GearRatios[5] = hkSetup.HK_GearRatios[5];

	*/

	SetupData.WheelRadius = hkSetup.HK_WheelRadius;
	SetupData.WheelThickness = hkSetup.HK_WheelThickness;
	SetupData.WheelFriction = hkSetup.HK_WheelFriction;
	SetupData.WheelMass = hkSetup.HK_WheelMass;

	SetupData.BrakeTorque = hkSetup.HK_BrakeTorque;

	SetupData.liftCoefficient = hkSetup.HK_liftCoefficient;

	SetupData.maxSteer = hkSetup.HK_maxSteer;
	SetupData.SteerBackWheels = hkSetup.HK_SteerBackWheels;

	SetupData.SuspensionLength = hkSetup.HK_SuspensionLength;
	SetupData.SuspensionStrength = hkSetup.HK_SuspensionStrength;

	SetupData.U = hkSetup.Up; SetupData.F = hkSetup.Front; SetupData.B = hkSetup.Back; SetupData.L = hkSetup.Lateral;

	SetupData.bUseRayTrace = hkSetup.RTWheels;
	SetupData.bBodyScale = hkSetup.BodyScale;

	SetupData.NormalSpinDamping = hkSetup.HK_normalSpinDamping;
	SetupData.CollisionSpinDamping = hkSetup.HK_collisionSpinDamping;
	SetupData.CollisionThreshold = hkSetup.HK_collisionThreshold;

	SetupData.bUseCustomWheel = hkSetup.UseCustomWheel;
	SetupData.bUseCustomCar = hkSetup.HK_bCustomCar;

	return SetupData;
}