#include "L1Robot/L1Robot.h"
#include "Common/FXCommon.h"
#include "Common/FXType.h"
#include "Common/FXErrorCode.h"
#include <windows.h>

int main(int argc, char** argv)
{
	int sdk_version = 0;
	int controller_version = 0;
	unsigned int system_errorcode = 0;
	FXStateType obj_state = FX_STATE_UNKNOWN;
	double k[7] = { 3,3,3,2,1,1,1 };
	double d[7] = { 0.2,0.2,0.2,0.2,0.2,0.2,0.2 };
	double vel_ratio = 10;
	double acc_ratio = 10;
	double pos1[7] = { 0,0,0,0,0,0,0 };
	double pos2[7] = { 10,10,10,90,10,10,10 };

	sdk_version = FX_L1_System_GetSDKVersion();
	printf("SDK version is 0x%08x\n", sdk_version);

	if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0)
	{
		printf("Failed to link system\n");
		goto WAIT_EXIT;
	}

	controller_version = FX_L1_System_GetControllerVersion();
	printf("Controller version is 0x%08x\n", controller_version);

	obj_state = FX_L1_Fbk_CurrentState(FX_OBJ_ARM0);
	if (obj_state == FX_STATE_ERROR)
	{
		printf("Arm0 is in STATE_ERROR state now, press any key to reset error\n");
		getchar();
		if (FX_L1_State_ResetError(FX_OBJ_ARM0, 1000, &system_errorcode) == FUNC_RET_SUCCESS)
		{
			printf("Reset arm0 error success, arm0 is now in STATE_IDLE state\n");
		}
		else
		{
			printf("Failed to reset arm0 error, errorcode = 0x%08x\n", system_errorcode);
			goto WAIT_EXIT;
		}
	}
	else if (obj_state != FX_STATE_IDLE)
	{
		printf("Arm0 is not in STATE_IDLE state now, press any key to transfer to STATE_IDLE state\n");
		getchar();
		if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
		{
			printf("Failed to transfer arm0 to STATE_IDLE state\n");
			goto WAIT_EXIT;
		}
	}

	printf("Arm0 is in STATE_IDLE state now, press any key to transfer to STATE_IMP_JOINT state\n");
	getchar();
	if (FX_L1_State_SwitchToImpJointMode(FX_OBJ_ARM0, 2000, vel_ratio, acc_ratio, k, d) != FUNC_RET_SUCCESS)
	{
		printf("Failed to transfer arm0 to STATE_IMP_JOINT state\n");
		goto WAIT_EXIT;
	}

	printf("Arm0 is in STATE_IMP_JOINT state now\n");
	printf("Press any key to let arm0 move to target position {%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
		pos1[0], pos1[1], pos1[2], pos1[3], pos1[4], pos1[5], pos1[6]);
	getchar();

	if (FX_L1_Comm_Clear(500) != FUNC_RET_SUCCESS
		|| FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM0, pos1) != FUNC_RET_SUCCESS
		|| FX_L1_Comm_Send() != FUNC_RET_SUCCESS)
	{
		printf("Failed to set arm0's target position\n");
		goto WAIT_EXIT;
	}
	
	printf("Set arm0's target positon success, arm0 is moving to the target position...\n");
	printf("Press any key to let arm0 move to target position {%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
		pos2[0], pos2[1], pos2[2], pos2[3], pos2[4], pos2[5], pos2[6]);
	getchar();

	if (FX_L1_Comm_Clear(500) != FUNC_RET_SUCCESS
		|| FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM0, pos2) != FUNC_RET_SUCCESS
		|| FX_L1_Comm_Send() != FUNC_RET_SUCCESS)
	{
		printf("Failed to set arm0's target position\n");
		goto WAIT_EXIT;
	}
	printf("Set arm0's target positon success, arm0 is moving to the target position...\n");
	printf("Press any key to transfer to STATE_IDLE state\n");
	getchar();
	if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
	{
		printf("Failed to transfer arm0 to STATE_IDLE state\n");
		goto WAIT_EXIT;
	}
	printf("Arm0 is in STATE_IDLE state now\n");

WAIT_EXIT:
	printf("Press any key to exit\n");
	getchar();
	return 0;
}
