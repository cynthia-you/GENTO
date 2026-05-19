#include "L1Robot/L1Robot.h"
#include "Common/FXCommon.h"
#include "Common/FXType.h"
#include "common/FXErrorCode.h"
#include <windows.h>

int main(int argc, char** argv)
{
	int sdk_version = 0;
	int controller_version = 0;
	FXStateType obj_state = FX_STATE_UNKNOWN;
	const ROBOT_RT* rt_ptr = NULL;
	unsigned int system_errorcode = 0;
	unsigned int servo_errorcode[7] = { 0 };

	sdk_version = FX_L1_System_GetSDKVersion();
	printf("SDK version is 0x%08x\n", sdk_version);

	if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0)
	{
		printf("Failed to link system\n");
		goto WAIT_EXIT;
	}
	
	controller_version = FX_L1_System_GetControllerVersion();
	printf("Controller version is 0x%08x\n", controller_version);

	obj_state = FX_L1_Fbk_CurrentState(FX_OBJ_ARM1);
	if (obj_state == FX_STATE_UNKNOWN)
	{
		printf("Arm0 is in unknown state\n");
		goto WAIT_EXIT;
	}
	else if (obj_state == FX_STATE_ERROR)
	{
		printf("Arm0 is in error state now\n");
	}
	else
	{
		printf("Arm0 is not in error, press any key to trigger an emergency error\n");
		getchar();
		if (FX_L1_Comm_Clear(500) != FUNC_RET_SUCCESS
			|| FX_L1_Runtime_EmergencyStop(FX_OBJ_ARM1_FLAG) != FX_OBJ_ARM1_FLAG
			|| FX_L1_Comm_SendAndWait(500) < 0)
		{
			printf("Failed to trigger an emergency error for arm0\n");
			goto WAIT_EXIT;
		}
		Sleep(100);
		if (FX_L1_Fbk_CurrentState(FX_OBJ_ARM1) != FX_STATE_ERROR)
		{
			printf("Arm0 is not in error state!\n");
			goto WAIT_EXIT;
		}
	}

	rt_ptr = FX_L1_Fbk_GetRT();
	if (rt_ptr == NULL)
	{
		printf("Failed to get ROBOT_RT data pointer\n");
		goto WAIT_EXIT;
	}
	system_errorcode = rt_ptr->m_ARMS[1].m_ARM_State.m_ERRCode;
	printf("Arm0's errorcode is %d\n", system_errorcode);
	if (system_errorcode == ERR_Servo)
	{
		if (FX_L1_State_GetServoErrorCode(FX_OBJ_ARM1, servo_errorcode) != FUNC_RET_SUCCESS)
		{
			printf("Failed to get servo errorcode for arm0\n");
			goto WAIT_EXIT;
		}
		else
		{
			printf("Arm0 is showing a servo error, servo errorcode is {0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x}\n",
				servo_errorcode[0], servo_errorcode[1], servo_errorcode[2], servo_errorcode[3], servo_errorcode[4], servo_errorcode[5], servo_errorcode[6]);
		}
	}
	printf("Press any key to reset error for arm0\n");
	getchar();
	if (FX_L1_State_ResetError(FX_OBJ_ARM1, 1000, &system_errorcode) == FUNC_RET_SUCCESS)
	{
		printf("Reset arm0 error success, arm0 is now in STATE_IDLE state\n");
	}
	else
	{
		printf("Failed to reset arm0 error. Arm0's errorcode is %d\n", system_errorcode);
		if (system_errorcode == ERR_Servo)
		{
			if (FX_L1_State_GetServoErrorCode(FX_OBJ_ARM1, servo_errorcode) != FUNC_RET_SUCCESS)
			{
				printf("Failed to get servo errorcode for arm0\n");
				goto WAIT_EXIT;
			}
			else
			{
				printf("Arm0 is showing a servo error, servo errorcode is {0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x}\n",
					servo_errorcode[0], servo_errorcode[1], servo_errorcode[2], servo_errorcode[3], servo_errorcode[4], servo_errorcode[5], servo_errorcode[6]);
			}
		}
	}

WAIT_EXIT:
	printf("Press any key to exit\n");
	getchar();
	return 0;
}
