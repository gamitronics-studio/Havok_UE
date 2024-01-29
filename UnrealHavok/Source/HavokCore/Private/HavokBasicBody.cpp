#include "HavokBasicBody.h"
THIRD_PARTY_INCLUDES_START
#include "../ThirdParty/Core.h"
THIRD_PARTY_INCLUDES_END

AHK_Body::AHK_Body()
{

}

bool AHK_Body::RemoveBody()
{
	return Body->HK_RemoveBody();
}

void AHK_Body::CreateSphere(float Radius, float Mass, float Friction, FVector Location)
{
	Body = new HK_Body();
	HK_Location tempLoc;
	tempLoc.x = Location.X;
	tempLoc.y = Location.Z;
	tempLoc.z = Location.Y;

	Body->HK_AddSphere(Radius, Mass, Friction, tempLoc);
}

void AHK_Body::CreateBox(float L, float W, float H, float Mass, float Friction, FVector Location, FQuat Rot)
{
	Body = new HK_Body();
	HK_Location tempLoc;
	tempLoc.x = Location.X;
	tempLoc.y = Location.Z;
	tempLoc.z = Location.Y;

	HK_Quaternion tempQ;
	tempQ.x = Rot.X;
	tempQ.y = Rot.Z;
	tempQ.z = Rot.Y;
	tempQ.w = Rot.W;


	Body->HK_AddBox(L, H, W, Mass, Friction, tempLoc, tempQ);
}

void AHK_Body::AddMesh(const TArray<float>& X, const TArray<float>& Y, const TArray<float>& Z, int VertCount, const TArray<int>& Triangles, int TriCount, FVector Location, FQuat Rotation)
{
	Body = new HK_Body();
	HK_Location tempLoc;
	tempLoc.x = Location.X;
	tempLoc.y = Location.Z;
	tempLoc.z = Location.Y;

	HK_Quaternion tempQ;
	tempQ.x = Rotation.X;
	tempQ.y = Rotation.Z;
	tempQ.z = Rotation.Y;
	tempQ.w = Rotation.W;

	Body->HK_AddMesh(X.GetData(), Z.GetData(), Y.GetData(), VertCount, Triangles.GetData(), TriCount, tempLoc, tempQ);

}

FVector AHK_Body::GetLocation()
{
	FVector Loc;
	HK_Location tempLoc = Body->HK_GetLocation();
	Loc.X = tempLoc.x;
	Loc.Y = tempLoc.z;
	Loc.Z = tempLoc.y;
	return Loc;
}

FRotator AHK_Body::GetRotation()
{
	FRotator Rot;
	HK_Quaternion FQ = Body->HK_GetRotation();

	FQuat Q(FQ.x, FQ.y, FQ.z, FQ.w);

	return Q.Rotator();
}

int AHK_Body::GetID()
{
	return Body->HK_GetID();
}