#include "L0_MAX_Body_Kinematics.h"
#include "FXMatrix.h"
#include "PointSet.h"
#include <cstdlib>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

extern "C"
{

	EXPORT CFxKineMAX *L0_CreateFxKineMAX()
	{
		return new CFxKineMAX();
	}

	EXPORT FX_VOID L0_DestroyCFxKineMAX(CFxKineMAX *obj)
	{
		delete obj;
	}

	EXPORT FX_VOID L0_OnLogSwitch_KineMAX(CFxKineMAX *obj, FX_INT32 log_tag)
	{
		if (obj)
			obj->L0_OnLogSwitch(log_tag);
	}

	EXPORT FX_VOID L0_OnSetCondition(CFxKineMAX *obj,
					  Vect3 std_Body,
					  Vect3 k_Body,
					  FX_DOUBLE std_L_len,
					  FX_DOUBLE k_L, 
					  FX_DOUBLE std_R_len, 
					  FX_DOUBLE k_R)
	{
		if (obj)
		{
			obj->L0_OnSetCondition(std_Body, k_Body, std_L_len, k_L, std_R_len, k_R);
		}
	}

	EXPORT FX_VOID L0_OnKineLR(CFxKineMAX *obj,
							Vect3 jv, 
							Matrix4 pgL, 
							Matrix4 pgR)
	{
		if (obj)
		{
			Vect3 jv_body={0};
			for (FX_INT32 i = 0; i < 3; ++i)
			{
				jv_body[i] = jv[i];
			}
			obj->L0_OnKineLR(jv_body,pgL,pgR);
		}
	}

	EXPORT FX_VOID L0_OnCalBody(CFxKineMAX *obj,
							  Vect3 tpos1, 
							  Vect3 tpos2, 
							  Vect3 ret_pos)
	{
		if (obj)
		{
			Vect3 tpos1_inside={0};
			Vect3 tpos2_inside={0};

			for (FX_INT32 i = 0; i < 3; ++i)
			{
				tpos1_inside[i] = tpos1[i];
				tpos2_inside[i] = tpos2[i];
			}

			obj->L0_OnCalBody(tpos1_inside,tpos2_inside,ret_pos);
		}
	}

	EXPORT FX_VOID L0_OnCalBody_withRef(CFxKineMAX *obj,
							Vect3 refjv, 
							Vect3 tpos1, 
							Vect3 tpos2, 
							Vect3 ret_pos)
	{
		if (obj)
		{
			obj->L0_OnCalBody_withref(refjv,tpos1,tpos2,ret_pos);
		}
	}
}