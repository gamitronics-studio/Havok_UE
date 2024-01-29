#include "pch.h" 
#include "HK_Body.h"
#include <Physics/Physics/Collide/Shape/hknpShape.h>
#include <Physics/Physics/Dynamics/Material/hknpMaterialLibrary.h>

#include <Common/Base/Types/Color/hkColor.h>
#include <Common/Base/Visualize/Shape/hkDisplayGeometry.h>

hknpBodyId BodyID;

HK_Body::HK_Body()
{

}

bool HK_Body::HK_RemoveBody()
{
	GetWorld()->destroyBodies(BodyID, 1);
	return true;
}

HK_Location HK_Body::HK_GetLocation()
{
	return GetBodyLocation(*BodyID);
}

HK_Quaternion HK_Body::HK_GetRotation()
{
	return GetBodyRotation(*BodyID);
}

int HK_Body::HK_GetID()
{
	return BodyID->asBodyIndex().indexUnchecked();
}

void HK_Body::HK_AddSphere(float Radius, float Mass, float Friction, HK_Location loc)
{
	hkReal radius = Radius;
	hkRefPtr<hknpShape> sphereShape = hknpShape::makeSphere(radius);

	hknpMaterialId materialId;
	{
		hknpMaterial material;
		material.m_restitution = Friction;
		materialId = GetMaterial(material);
	}

	hknpBodyCinfo bodyCinfo;
	bodyCinfo.m_shape = sphereShape;
	bodyCinfo.m_materialId = materialId;
	bodyCinfo.m_position.set(loc.x, loc.z, loc.y);
	bodyCinfo.m_mass = Mass;
	bodyCinfo.m_motionType = hknpMotionType::DYNAMIC;


	BodyID = new hknpBodyId();
	*BodyID = AddBody(bodyCinfo);
}

void HK_Body::HK_AddBox(float L, float W, float H, float Mass, float Friction, HK_Location loc, HK_Quaternion quat)
{
	hkVector4 halfExtents(L, W, H);
	hkRefPtr<hknpShape> boxShape = hknpShape::makeBoxFromHalfExtents(halfExtents);

	hknpMaterialId materialId;
	{
		hknpMaterial material;
		material.m_restitution = Friction;
		materialId = GetMaterial(material);
	}

	hknpBodyCinfo bodyCinfo;
	bodyCinfo.m_shape = boxShape;
	bodyCinfo.m_materialId = materialId;
	bodyCinfo.m_position.set(loc.x, loc.y, loc.z);
	bodyCinfo.m_orientation.set(quat.x, quat.y, quat.z, quat.w);

	bodyCinfo.m_mass = Mass;
	bodyCinfo.m_motionType = hknpMotionType::DYNAMIC;

	BodyID = new hknpBodyId();
	*BodyID = AddBody(bodyCinfo);
}

void HK_Body::HK_AddMesh(const float X[], const float Y[], const float Z[], int VertCount, const int Triangles[], int TriCount, HK_Location Location, HK_Quaternion Rotation)
{
	hkGeometry geometry;
	geometry.clear();

	for (int i = 0; i < VertCount; i++)
		geometry.m_vertices.expandOne().set(X[i], Y[i], Z[i]);

	int Tri = 0;
	for (int i = 0; i < TriCount; i += 3)
		geometry.m_triangles.expandOne().set(i, i + 1, i + 2);

	hknpBodyId BodyId;

	hknpBodyCinfo Cinfo;
	{
		Cinfo.m_position.set(Location.x, Location.y, Location.z);
		Cinfo.m_orientation.set(Rotation.x, Rotation.y, Rotation.z, Rotation.w);
		Cinfo.m_shape = hknpShape::makeMesh(geometry);
	}

	BodyID = new hknpBodyId();
	*BodyID = AddBody(Cinfo);
}
