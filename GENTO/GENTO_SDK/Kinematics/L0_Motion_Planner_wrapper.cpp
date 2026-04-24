#include "L0_Motion_Planner.h"
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

	EXPORT CFxPln *L0_CreateFxPln()
	{
		return new CFxPln();
	}

	EXPORT FX_VOID L0_DestroyFxPln(CFxPln *obj)
	{
		delete obj;
	}

	EXPORT FX_VOID L0_OnLogSwitch_Pln(CFxPln *obj, FX_INT32 log_tag)
	{
		if (obj)
			obj->L0_OnLogSwitch(log_tag);
	}

	EXPORT FX_INT32 L0_OnInitEnv_SingleArm_Pln(CFxPln *obj,FX_CHAR * env_path,FX_INT32 RobotSerial)
	{
		if (!obj)
			return 0;
		return obj->L0_OnInitEnv_SingleArm(env_path, RobotSerial) ? 1 : 0;
	}

	EXPORT FX_INT32 L0_OnInitEnv_DualArm_Pln(CFxPln *obj,FX_CHAR * env_path)
	{
		if (!obj)
			return 0;
		return obj->L0_OnInitEnv_DualArm(env_path) ? 1 : 0;
	}

	EXPORT FX_INT32 L0_OnMovJ(CFxPln *obj,
					  Vect7 start_joint, 
					  Vect7 end_joint, 
					  FX_DOUBLE vel_ratio, 
					  FX_DOUBLE acc_ratio,
					  FX_VOID *pset)
	{
		if (!obj)
			return 0;
		Vect7 start, end;
		for (FX_INT32 i = 0; i < 7; ++i)
		{
			start[i] = start_joint[i];
			end[i] = end_joint[i];
		}
		return obj->L0_OnMovJ(start, end, vel_ratio, acc_ratio, (CPointSet *)pset) ? 1 : 0;
	}

	EXPORT FX_INT32 L0_OnMovL(CFxPln *obj,
					  Vect6 start_xyzabc,
					  Vect6 end_xyzabc,
					  Vect7 ref_joints,
					  FX_DOUBLE vel,
					  FX_DOUBLE acc,
					  FX_INT32 freq,
					  FX_VOID *pset)
	{
		if (!obj)
			return 0;
		Vect6 start, end;
		Vect7 ref;
		for (FX_INT32 i = 0; i < 6; ++i)
		{
			start[i] = start_xyzabc[i];
			end[i] = end_xyzabc[i];
		}
		for (FX_INT32 i = 0; i < 7; ++i)
			ref[i] = ref_joints[i];
		return obj->L0_OnMovL(start, end, ref, vel, acc, freq,(CPointSet *)pset) ? 1 : 0;
	}

	EXPORT FX_INT32 L0_OnMovL_KeepJ(CFxPln *obj,
							Vect7 start_joints,
							Vect7 stop_joints,
							FX_DOUBLE vel,
							FX_DOUBLE acc,
							FX_INT32 freq,
							FX_VOID *pset)
	{
		if (!obj)
			return 0;
		Vect7 start, stop;
		for (FX_INT32 i = 0; i < 7; ++i)
		{
			start[i] = start_joints[i];
			stop[i] = stop_joints[i];
		}
		return obj->L0_OnMovL_KeepJ(start, stop, vel, acc, freq, (CPointSet *)pset) ? 1 : 0;
	}

	EXPORT FX_INT32 L0_MultiPoints_Set_MovL_Start(CFxPln *obj,
									Vect7 ref_joints,
									Vect6 start_xyzabc,
									Vect6 end_xyzabc,
									FX_DOUBLE allow_range,
									FX_INT32 zsp_type,
									Vect6 zsp_para,
									FX_DOUBLE vel,
									FX_DOUBLE acc,
									FX_INT32 freq)
	{
		if (!obj)
			return 0;
		Vect7 refJ;
		Vect6 start, end;
		Vect6 zsp;
		for (FX_INT32 i = 0; i < 6; ++i)
		{
			start[i] = start_xyzabc[i];
			end[i] = end_xyzabc[i];
			zsp[i] = zsp_para[i];
		}
		for (FX_INT32 i = 0; i < 7; ++i)
			refJ[i] = ref_joints[i];
		return obj->L0_MultiPoints_Set_MovL_Start(refJ, start, end, allow_range,
										 zsp_type, zsp, vel, acc, freq)
				   ? 1
				   : 0;
	}

	EXPORT FX_INT32 L0_MultiPoints_Set_MovL_NextPoints(CFxPln *obj,
										 Vect6 next_xyzabc,
										 FX_DOUBLE allow_range,
										 FX_INT32 zsp_type,
										 Vect6 zsp_para,
										 FX_DOUBLE vel,
										 FX_DOUBLE acc)
	{
		if (!obj)
			return 0;
		Vect6 next, zsp;
		for (FX_INT32 i = 0; i < 6; ++i)
		{
			next[i] = next_xyzabc[i];
			zsp[i] = zsp_para[i];
		}
		return obj->L0_MultiPoints_Set_MovL_NextPoints(next, allow_range,
											  zsp_type, zsp, vel, acc)
				   ? 1
				   : 0;
	}

	EXPORT FX_INT32 L0_MultiPoints_Get_MovL_Path(CFxPln *obj, FX_VOID *pset)
	{
		if (!obj)
			return 0;
		return obj->L0_MultiPoints_Get_MovL_Path((CPointSet *)pset) ? 1 : 0;
	}

	EXPORT FX_INT32 L0_OnMovL_DualArm_FixBody(CFxPln *obj, DualArm_FixedBody* DA_FB,FX_VOID *pset_left,FX_VOID *pset_right)
	{
		if (!obj)
			return 0;
		return obj->L0_OnMovL_DualArm_FixBody(DA_FB, (CPointSet *)pset_left, (CPointSet *)pset_right) ? 1 : 0;
	}

	EXPORT FX_VOID L0_XYZABC2Matrix4_DEG(Vect6 xyzabc, Matrix4 m)
	{
		FX_DOUBLE angx = xyzabc[3];
		FX_DOUBLE angy = xyzabc[4];
		FX_DOUBLE angz = xyzabc[5];
		FX_DOUBLE sa = 0.0, sb = 0.0, sr = 0.0;
		FX_DOUBLE ca = 0.0, cb = 0.0, cr = 0.0;

		// 使用与原始库相同的宏
		FX_SIN_COS_DEG(angx, &sr, &cr);
		FX_SIN_COS_DEG(angy, &sb, &cb);
		FX_SIN_COS_DEG(angz, &sa, &ca);

		m[0][0] = ca * cb;
		m[0][1] = ca * sb * sr - sa * cr;
		m[0][2] = ca * sb * cr + sa * sr;

		m[1][0] = sa * cb;
		m[1][1] = sa * sb * sr + ca * cr;
		m[1][2] = sa * sb * cr - ca * sr;

		m[2][0] = -sb;
		m[2][1] = cb * sr;
		m[2][2] = cb * cr;

		m[0][3] = xyzabc[0];
		m[1][3] = xyzabc[1];
		m[2][3] = xyzabc[2];

		m[3][0] = 0;
		m[3][1] = 0;
		m[3][2] = 0;
		m[3][3] = 1;
	}

	EXPORT FX_VOID L0_Matrix42XYZABC_DEG(Matrix4 m, Vect6 xyzabc)
	{
		FX_DOUBLE r = FX_Sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0]);
		xyzabc[4] = FX_ATan2(-m[2][0], r);
		if (r <= 1e-10 && r >= -1e-10) // 使用 FXARM_EPS 的值，通常为 1e-10
		{
			xyzabc[5] = 0;
			if (xyzabc[4] > 0)
				xyzabc[3] = FX_ATan2(m[0][1], m[1][1]);
			else
				xyzabc[3] = -FX_ATan2(m[0][1], m[1][1]);
		}
		else
		{
			xyzabc[5] = FX_ATan2(m[1][0], m[0][0]);
			xyzabc[3] = FX_ATan2(m[2][1], m[2][2]);
		}
		xyzabc[0] = m[0][3];
		xyzabc[1] = m[1][3];
		xyzabc[2] = m[2][3];

		xyzabc[3] *= FXARM_R2D;
		xyzabc[4] *= FXARM_R2D;
		xyzabc[5] *= FXARM_R2D;
	}

	EXPORT FX_VOID *L0_FX_CPointSet_Create()
	{
		return new CPointSet();
	}

	EXPORT FX_VOID L0_FX_CPointSet_Destroy(FX_VOID *pset)
	{
		if (pset)
		{
			delete static_cast<CPointSet *>(pset);
		}
	}

	EXPORT FX_BOOL L0_FX_CPointSet_OnInit(FX_VOID *pset, FX_INT32 ptype)
	{
		if (!pset)
			return FX_FALSE;
		CPointSet *pointSet = static_cast<CPointSet *>(pset);
		return pointSet->OnInit(static_cast<PoinType>(ptype)) ? FX_TRUE : FX_FALSE;
	}

	EXPORT FX_INT32 L0_FX_CPointSet_OnGetPointNum(FX_VOID *pset)
	{
		if (!pset)
			return 0;
		CPointSet *pointSet = static_cast<CPointSet *>(pset);
		return pointSet->OnGetPointNum();
	}

	EXPORT FX_DOUBLE *L0_FX_CPointSet_OnGetPoint(FX_VOID *pset, FX_INT32 pos)
	{
		if (!pset)
			return nullptr;
		CPointSet *pointSet = static_cast<CPointSet *>(pset);
		return pointSet->OnGetPoint(pos);
	}

	EXPORT FX_BOOL L0_FX_CPointSet_OnSetPoint(FX_VOID *pset, FX_DOUBLE point_value[])
	{
		if (!pset)
			return FX_FALSE;
		CPointSet *pointSet = static_cast<CPointSet *>(pset);
		return pointSet->OnSetPoint(point_value) ? FX_TRUE : FX_FALSE;
	}
}