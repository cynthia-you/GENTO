#include "L1Robot/L1Robot.h"
#include "Common/FXCommon.h"
#include "Common/FXType.h"
#include "Common/FXErrorCode.h"
#include <windows.h>

void ReadArm0EncoderFbk(double motor_encoder_fbk[7], double external_encoder_fbk[7])
{
	const ROBOT_RT* rt_ptr = FX_L1_Fbk_GetRT();
	const ROBOT_SG* sg_ptr = FX_L1_Fbk_GetSG();
	if (rt_ptr == NULL || sg_ptr == NULL)
	{
		return;
	}

	for (int i = 0; i < 7; i++)
	{
		motor_encoder_fbk[i] = rt_ptr->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
		external_encoder_fbk[i] = sg_ptr->m_ARMS[0].m_ARM_GET.m_ARM_FBK_Joint_ExtPos[i];
	}
}

int main(int argc, char** argv)
{
	int sdk_version = 0;
	int controller_version = 0;
	FXStateType obj_state = FX_STATE_UNKNOWN;
	unsigned int system_errorcode = 0;
	double motor_encoder_fbk[7] = { 0 };
	double external_encoder_fbk[7] = { 0 };

	sdk_version = FX_L1_System_GetSDKVersion();
	printf("SDK version is 0x%08x\n", sdk_version);

	printf("---------------------------------IMPORTANT----------------------------------------\n");
	printf("Before running this sample, please ensure Arm0 is at the mechanical zero position.\n");
	printf("Otherwise, Arm0 will not operate correctly in all control mode!\n");
	printf("For each controlled object, the encoder reset operation can only be performed once "
		   "per power cycle. Performing this operation multiple times will result in abnormal "
		   "encoder position data upon the next power-up.\n");
	printf("----------------------------------------------------------------------------------\n");

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

	ReadArm0EncoderFbk(motor_encoder_fbk, external_encoder_fbk);
	printf("Current arm0's motor encoder feedback: {%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
		motor_encoder_fbk[0], motor_encoder_fbk[1], motor_encoder_fbk[2], motor_encoder_fbk[3], motor_encoder_fbk[4], motor_encoder_fbk[5], motor_encoder_fbk[6]);
	printf("Current arm0's external encoder feedback: {%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
		external_encoder_fbk[0], external_encoder_fbk[1], external_encoder_fbk[2], external_encoder_fbk[3], external_encoder_fbk[4], external_encoder_fbk[5], external_encoder_fbk[6]);
	printf("Press any key to reset motor encoder and external encoder feedbacks to zero\n");
	getchar();
	if (FX_L1_Config_ResetEncOffset(FX_OBJ_ARM0, 0xFF) != FUNC_RET_SUCCESS)
	{
		printf("Reset arm0's motor encoder and external encoder feedbacks to zero failed\n");
		goto WAIT_EXIT;
	}
	Sleep(100);
	printf("Reset arm0's motor encoder and external encoder feedbacks to zero success\n");
	ReadArm0EncoderFbk(motor_encoder_fbk, external_encoder_fbk);
	printf("Current arm0's motor encoder feedback: {%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
		motor_encoder_fbk[0], motor_encoder_fbk[1], motor_encoder_fbk[2], motor_encoder_fbk[3], motor_encoder_fbk[4], motor_encoder_fbk[5], motor_encoder_fbk[6]);
	printf("Current arm0's external encoder feedback: {%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf}\n",
		external_encoder_fbk[0], external_encoder_fbk[1], external_encoder_fbk[2], external_encoder_fbk[3], external_encoder_fbk[4], external_encoder_fbk[5], external_encoder_fbk[6]);

	printf("Upon successful encoder reset, the motor encoder position feedback will remain unchanged until after a reboot, whereas the external encoder position feedback updates immediately.\n");
	printf("Do not execute any motion control commands after resetting the encoder; please power cycle​ the system.\n");
	getchar();
WAIT_EXIT:
	printf("Press any key to exit\n");
	getchar();
	return 0;
}
