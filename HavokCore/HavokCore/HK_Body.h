#pragma once
#include "Core.h"

#ifdef HAVOKCORE_EXPORTS
#define HAVOK_API __declspec(dllexport)
#else
#define HAVOK_API __declspec(dllimport)
#endif

#ifdef __cplusplus 
extern "C"
{
#endif

   

#ifdef __cplusplus
}
#endif

class HAVOK_API HK_Body
{
private:

    int BodyIndex;
    struct hknpBodyId* BodyID;
    
public:
    HK_Body();
    bool HK_RemoveBody();
    void HK_AddSphere(float, float, float, HK_Location);
    void HK_AddBox(float, float, float, float, float, HK_Location, HK_Quaternion);
    void HK_AddMesh(const float[], const float[], const float[], int, const int[], int, HK_Location, HK_Quaternion);
    HK_Location HK_GetLocation();
    HK_Quaternion HK_GetRotation();
    int HK_GetID();
};