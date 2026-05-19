#include "L1Robot/L1Robot.h"
#include "Common/FXCommon.h"
#include "Common/FXType.h"
#include "Common/FXErrorCode.h"
#include <windows.h>

int main(int argc, char** argv)
{
	int sdk_version = 0;
	int controller_version = 0;

	sdk_version = FX_L1_System_GetSDKVersion();
	printf("SDK version is 0x%08x\n", sdk_version);

	if (FX_L1_System_Link(6, 6, 7, 190, FX_LOG_ALL_FLAG) < 0)
	{
		printf("Failed to link system\n");
		goto WAIT_EXIT;
	}

	controller_version = FX_L1_System_GetControllerVersion();
	printf("Controller version is 0x%08x\n", controller_version);

	printf("Press any key to upload config file robot.ini to local\n");
	getchar();
	if (FX_L1_System_RecvFile("./robot.ini", "/home/FUSION/Config/cfg/robot.ini") != FUNC_RET_SUCCESS)
	{
		printf("Failed to transfer robot.ini to local path\n");
	}
	else
	{
		printf("Transfer robot.ini to local path success\n");
	}
	printf("Press any key to download config file to remote path /home/FUSION/Tmp/robot.ini\n");
	getchar();
	if (FX_L1_System_SendFile("./robot.ini", "/home/FUSION/Tmp/robot.ini") != FUNC_RET_SUCCESS)
	{
		printf("Failed to transfer robot.ini to remote path\n");
	}
	else
	{
		printf("Transfer robot.ini to remote path success\n");
	}

WAIT_EXIT:
	printf("Press any key to exit\n");
	getchar();
	return 0;
}