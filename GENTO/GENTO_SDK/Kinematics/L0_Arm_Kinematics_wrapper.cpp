#include "L0_Arm_Kinematics.h"
#include "PointSet.h"
#include <cstdlib>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

extern "C"
{

	EXPORT CFxKineIF *L0_CreateFxKineIF()
	{
		return new CFxKineIF();
	}

	EXPORT FX_VOID L0_DestroyFxKineIF(CFxKineIF *obj)
	{
		delete obj;
	}

	EXPORT FX_VOID L0_OnLogSwitch_KineIF(CFxKineIF *obj, FX_INT32 log_tag)
	{
		if (obj)
			obj->L0_OnLogSwitch(log_tag);
	}

	EXPORT FX_INT32 L0_OnInitEnv_KineIF(CFxKineIF *obj, FX_CHAR *env_path, FX_INT32 RobotSerial)
	{
		if (!obj)
			return 0;
		return obj->L0_OnInitEnv(env_path, RobotSerial) ? 1 : 0;
	}

	EXPORT FX_INT32 L0_OnSetTool(CFxKineIF *obj, Matrix4 tool)
	{
		if (!obj)
			return 0;
		return obj->L0_OnSetTool(tool) ? 1 : 0;
	}

	EXPORT FX_VOID L0_OnRmvTool(CFxKineIF *obj)
	{
		if (obj) 
		{
			obj->L0_OnRmvTool();
		}
	}

	EXPORT FX_INT32 L0_OnSolveArmFK(CFxKineIF *obj,
							Vect7 joints,
							Matrix4 pgos)
	{
		if (!obj)
			return 0;
		Matrix4 mat;
		FX_INT32 ret = obj->L0_OnSolveArmFK(joints, mat) ? 1 : 0;
		if (ret)
		{
			for (FX_INT32 i = 0; i < 4; ++i)
				for (FX_INT32 j = 0; j < 4; ++j)
					pgos[i][j] = mat[i][j];
		}
		return ret;
	}

	EXPORT FX_INT32 L0_OnSolveArmJcb(CFxKineIF *obj,
							 Vect7 joints,
							 FX_DOUBLE jcb[6][7])
	{
		if (!obj)
			return 0;
		return obj->L0_OnSolveArmJcb(joints, jcb) ? 1 : 0;
	}

	EXPORT FX_INT32 L0_OnSolveArmIK(CFxKineIF *obj, FX_VOID *solve_para)
	{
		if (!obj)
			return 0;
		return obj->L0_OnSolveArmIK((FX_InvKineSolvePara *)solve_para) ? 1 : 0;
	}

	EXPORT FX_INT32 L0_OnIdenDynaLoad(CFxKineIF *obj, FX_CHAR* path,FX_DOUBLE* mass, Vect3 mr, Vect6 I)
	{
		if (!obj)
			return 0;
		return obj->L0_OnIdenDynaLoad(path,mass,mr,I) ? 1 : 0;
	}

}