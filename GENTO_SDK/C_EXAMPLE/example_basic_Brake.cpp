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
		printf("Brake lock/unlock must be run in FX_STATE_IDLE state, try to reset error\n");
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
		if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
		{
			printf("Failed to transfer arm0 to STATE_IDLE state\n");
			goto WAIT_EXIT;
		}
	}
	
	printf("Press any key to unlock all brakes of arm0, please hold the arm carefully or it will drop!\n");
	getchar();
	if (FX_L1_Config_SetBrakeUnlock(FX_OBJ_ARM0, 0xFF) != FUNC_RET_SUCCESS)
	{
		printf("Unlock all brakes of arm0 failed\n");
	}

	printf("Press any key to lock all brakes of arm0\n");
	getchar();
	if (FX_L1_Config_SetBrakeLock(FX_OBJ_ARM0, 0xFF) != FUNC_RET_SUCCESS)
	{
		printf("Lock all brakes of arm0 failed\n");
	}

WAIT_EXIT:
	printf("Press any key to exit\n");
	getchar();
	return 0;
}
