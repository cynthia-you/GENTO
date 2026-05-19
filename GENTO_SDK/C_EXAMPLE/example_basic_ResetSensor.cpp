#include "L1Robot/L1Robot.h"
#include "Common/FXCommon.h"
#include "Common/FXType.h"
#include "Common/FXErrorCode.h"
#include <windows.h>

void ReadArm0SensorFbk(double sensor_fbk[7])
{
	const ROBOT_RT* rt_ptr = FX_L1_Fbk_GetRT();
	if (rt_ptr == NULL)
	{
		return;
	}

	for (int i = 0; i < 7; i++)
	{
		sensor_fbk[i] = rt_ptr->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor[i];
	}
}

int main(int argc, char** argv)
{
	int sdk_version = 0;
	int controller_version = 0;
	FXStateType obj_state = FX_STATE_UNKNOWN;
	unsigned int system_errorcode = 0;
	double zero_position[7] = { 0 };
	double sensor_fbk[7] = { 0 };

	sdk_version = FX_L1_System_GetSDKVersion();
	printf("SDK version is 0x%08x\n", sdk_version);

	printf("-----------------------------------------IMPORTANT------------------------------------------\n");
	printf("Before running this example, please ensure that no tool is mounted on the Arm0 end effector.\n");
	printf("Otherwise, Arm0 will not operate correctly in torque control mode!\n");
	printf("--------------------------------------------------------------------------------------------\n");

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
		printf("Reset sensor offset must be run in FX_STATE_IDLE state, try to reset error\n");
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
	
	printf("Press any key to transfer arm0 to FX_STATE_POSITION state\n");
	getchar();
	if (FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 2000, 10, 10) != FUNC_RET_SUCCESS)
	{
		printf("Transfer arm0 to FX_STATE_POSITION state failed\n");
		goto WAIT_EXIT;
	}

	printf("Press any key to let arm0 move to zero position\n");
	getchar();
	if (FX_L1_Comm_Clear(500) != FUNC_RET_SUCCESS
		|| FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM0, zero_position) != FUNC_RET_SUCCESS
		|| FX_L1_Comm_SendAndWait(500) < 0)
	{
		printf("Failed to set arm0's target position\n");
		goto WAIT_EXIT;
	}

	printf("Please wait until arm0 goes to zero position, and press any key to transfer arm0 to FX_STATE_IDLE state\n");
	getchar();
	if (FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 1000) != FUNC_RET_SUCCESS)
	{
		printf("Transfer arm0 to FX_STATE_IDLE state failed\n");
	}

	ReadArm0SensorFbk(sensor_fbk);
	printf("Current arm0's sensor feedback: {%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
		sensor_fbk[0], sensor_fbk[1], sensor_fbk[2], sensor_fbk[3], sensor_fbk[4], sensor_fbk[5], sensor_fbk[6]);

	printf("Press any key to clear sensor offset to zero\n");
	getchar();
	if (FX_L1_Config_ResetSensorOffset(FX_OBJ_ARM0) != FUNC_RET_SUCCESS)
	{
		printf("Clear arm0's sensor offset to zero failed\n");
		goto WAIT_EXIT;
	}

	Sleep(10);
	ReadArm0SensorFbk(sensor_fbk);
	printf("Clear sensor offset success, current arm0's sensor feedback: {%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
		sensor_fbk[0], sensor_fbk[1], sensor_fbk[2], sensor_fbk[3], sensor_fbk[4], sensor_fbk[5], sensor_fbk[6]);
	getchar();
WAIT_EXIT:
	printf("Press any key to exit\n");
	getchar();
	return 0;
}
