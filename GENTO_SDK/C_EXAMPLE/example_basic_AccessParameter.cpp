#include "L1Robot/L1Robot.h"
#include "Common/FXCommon.h"
#include "Common/FXType.h"
#include "common/FXErrorCode.h"
#include <windows.h>

int main(int argc, char** argv)
{
	int sdk_version = 0;
	int controller_version = 0;
	char robot_name[30] = { 0 };
	int bus_freq = 0;
	float new_enc_error_valve = 0;
	float old_enc_error_valve = 0;

	sdk_version = FX_L1_System_GetSDKVersion();
	printf("SDK version is 0x%08x\n", sdk_version);

	if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0)
	{
		printf("Failed to link system\n");
		goto WAIT_EXIT;
	}

	controller_version = FX_L1_System_GetControllerVersion();
	printf("Controller version is 0x%08x\n", controller_version);

	if (FX_L1_Param_GetString("R.BASIC.Name", robot_name) != FUNC_RET_SUCCESS)
	{
		printf("Failed to get string parameter: R.BASIC.Name\n");
	}

	if (FX_L1_Param_GetInt32("R.BASIC.BusFreq", &bus_freq) != 0)
	{
		printf("Failed to get int parameter: R.BASIC.BusFreq\n");
	}

	if (FX_L1_Param_GetFloat("R.A0.BASIC.EncErrorValve", &old_enc_error_valve) != FUNC_RET_SUCCESS)
	{
		printf("Failed to get float parameter: R.A0.BASIC.EncErrorValve\n");
	}

	printf("Press any key to set float parameter R.A0.BASIC.EncErrorValve=2.5\n");
	getchar();
	if (FX_L1_Param_SetFloat("R.A0.BASIC.EncErrorValve", 2.5) != FUNC_RET_SUCCESS)
	{
		printf("Failed to set float parameter R.A0.BASIC.EncErrorValve\n");
	}

	printf("Press any key to get float parameter R.A0.BASIC.EncErrorValve\n");
	getchar();
	if (FX_L1_Param_GetFloat("R.A0.BASIC.EncErrorValve", &new_enc_error_valve) != FUNC_RET_SUCCESS)
	{
		printf("Failed to get float parameter: R.A0.BASIC.EncErrorValve\n");
	}

	printf("Press any key to set float parameter R.A0.BASIC.EncErrorValve=%.4f\n", old_enc_error_valve);
	getchar();
	if (FX_L1_Param_SetFloat("R.A0.BASIC.EncErrorValve", old_enc_error_valve) != FUNC_RET_SUCCESS)
	{
		printf("Failed to set float parameter R.A0.BASIC.EncErrorValve\n");
	}

WAIT_EXIT:
	printf("Press any key to exit\n");
	getchar();
	return 0;
}