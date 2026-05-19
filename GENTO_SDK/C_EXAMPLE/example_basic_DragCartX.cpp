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
	double k[7] = { 2000,2000,2000,100,100,100,50 };
	double d[7] = { 0.1,0.1,0.1,0.1,0.1,0.1,1 };

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

	printf("Arm0 is in STATE_IDLE state now, press any key to transfer to STATE_DRAG_CART state\n");
	getchar();
	if (FX_L1_State_SwitchToDragCartX(FX_OBJ_ARM0, 2000, k, d) != FUNC_RET_SUCCESS)
	{
		printf("Failed to transfer arm0 to STATE_DRAG_CART state\n");
		goto WAIT_EXIT;
	}

	printf("Arm0 is in STATE_DRAG_CART state now, please press the drag button on arm0's terminal to drag the arm\n");
	printf("Press any key to let arm0 transfer to STATE_IDLE state.\n");
	getchar();
	if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
	{
		printf("Failed to transfer arm0 to STATE_IDLE state\n");
		goto WAIT_EXIT;
	}
WAIT_EXIT:
	printf("Press any key to exit\n");
	getchar();
	return 0;
}
