#include "L1Robot/L1Robot.h"
#include "Common/FXCommon.h"
#include "Common/FXType.h"
#include "Common/FXErrorCode.h"
#include <windows.h>

const char* StateType2Str(FXStateType state_type)
{
	switch (state_type)
	{
	case FX_STATE_IDLE: return "STATE_IDLE";
	case FX_STATE_POSITION: return "STATE_POSITION";
	case FX_STATE_IMP_JOINT: return "STATE_IMP_JOINT";
	case FX_STATE_IMP_CART: return "STATE_IMP_CART";
	case FX_STATE_IMP_FORCE: return "STATE_IMP_FORCE";
	case FX_STATE_DRAG_JOINT: return "STATE_DRAG_JOINT";
	case FX_STATE_DRAG_CART_X: return "STATE_DRAG_CART_X";
	case FX_STATE_DRAG_CART_Y: return "STATE_DRAG_CART_Y";
	case FX_STATE_DRAG_CART_Z: return "STATE_DRAG_CART_Z";
	case FX_STATE_DRAG_CART_R: return "STATE_DRAG_CART_R";
	case FX_STATE_RELEASE: return "STATE_RELEASE";
	case FX_STATE_PD: return "STATE_PD";
	case FX_STATE_ERROR: return "STATE_ERROR";
	case FX_STATE_TRANSFERRING: return "STATE_TRANSFERRING";
	case FX_STATE_UNKNOWN:
	default: return "STATE_UNKNOWN";
	}
}

int main(int argc, char** argv)
{
	int sdk_version = 0;
	int controller_version = 0;
	unsigned int system_errorcode = 0;
	FXStateType obj_state = FX_STATE_UNKNOWN;
	double vel_ratio = 10;
	double acc_ratio = 10;
	double pos1[7] = { 0,0,0,0,0,0,0 };
	double pos2[7] = { 10,10,10,90,10,10,10 };
	FXStateType cur_state = FX_STATE_UNKNOWN;

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

	printf("Arm0 is in STATE_IDLE state now, press any key to transfer to STATE_POSITION state\n");
	getchar();
	if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 2000, vel_ratio, acc_ratio) != FUNC_RET_SUCCESS)
	{
		printf("Failed to transfer arm0 to STATE_POSITION state\n");
		goto WAIT_EXIT;
	}

	printf("Arm0 is in STATE_POSITION state now\n");
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
	printf("Press any key to trigger arm0's emergency\n");
	getchar();

	if (FX_L1_Comm_Clear(500) != FUNC_RET_SUCCESS
		|| FX_L1_Runtime_EmergencyStop(FX_OBJ_ARM0_FLAG) != FX_OBJ_ARM0_FLAG
		|| FX_L1_Comm_Send() != FUNC_RET_SUCCESS)
	{
		printf("Failed to trigger arm0's emergency stop\n");
		goto WAIT_EXIT;
	}
	printf("Arm0's emergency stop is triggered\n");
	Sleep(200);
	cur_state = FX_L1_Fbk_CurrentState(FX_OBJ_ARM0);
	printf("Arm0 is in %s state now\n", StateType2Str(cur_state));

WAIT_EXIT:
	printf("Press any key to exit\n");
	getchar();
	return 0;
}
