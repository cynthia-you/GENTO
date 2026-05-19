#include "L1Robot.h"
#include "FXCmplOpt.h"
#include "FXErrorCode.h"
#include "L0Robot.h"
#include "FXFileClient.h"
#include <cassert>
#include <math.h>

#define FX_COMM_MAX_TIMEOUT 2000

unsigned int FX_LOG_L1_LEVEL = 0;
char FX_ROBOT_NAME[30] = {0};
int FX_ARM0_DOF = 0;
int FX_ARM1_DOF = 0;
int FX_HEAD_DOF = 0;
int FX_BODY_DOF = 0;
int FX_LIFT_DOF = 0;
int FX_LINK_TAG = 0;
unsigned char FX_LINK_IP[4] = {0};
FXRobotType FX_ROBOT_TYPE = FX_ROBOT_NULL;

const char *_FX_ObjType2Str(FXObjType obj_type)
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
        return "Arm0";
    case FX_OBJ_ARM1:
        return "Arm1";
    case FX_OBJ_HEAD:
        return "Head";
    case FX_OBJ_BODY:
        return "Body";
    case FX_OBJ_LIFT:
        return "Lift";
    default:
        return "UnknownObj";
    }
}

const char *_FX_StateType2Str(FXStateType state_type)
{
    switch (state_type)
    {
    case FX_STATE_IDLE:
        return "STATE_IDLE";
    case FX_STATE_POSITION:
        return "STATE_POSITION";
    case FX_STATE_IMP_JOINT:
        return "STATE_IMP_JOINT";
    case FX_STATE_IMP_CART:
        return "STATE_IMP_CART";
    case FX_STATE_IMP_FORCE:
        return "STATE_IMP_FORCE";
    case FX_STATE_DRAG_JOINT:
        return "STATE_DRAG_JOINT";
    case FX_STATE_DRAG_CART_X:
        return "STATE_DRAG_CART_X";
    case FX_STATE_DRAG_CART_Y:
        return "STATE_DRAG_CART_Y";
    case FX_STATE_DRAG_CART_Z:
        return "STATE_DRAG_CART_Z";
    case FX_STATE_DRAG_CART_R:
        return "STATE_DRAG_CART_R";
    case FX_STATE_RELEASE:
        return "STATE_RELEASE";
    case FX_STATE_PD:
        return "STATE_PD";
    case FX_STATE_ERROR:
        return "STATE_ERROR";
    case FX_STATE_TRANSFERRING:
        return "STATE_TRANSFERRING";
    case FX_STATE_UNKNOWN:
    default:
        return "STATE_UNKNOWN";
    }
}

void _FX_DEBG(const FX_CHAR *fmt, ...)
{
    if ((FX_LOG_L1_LEVEL & FX_LOG_DEBG_FLAG) == 0)
    {
        return;
    }

    FX_CHAR fmt_str[512] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(fmt_str, 511, fmt, args);
    va_end(args);
    printf("[DEBG][L1] %s\n", fmt_str);
}

void _FX_INFO(const FX_CHAR *fmt, ...)
{
    if ((FX_LOG_L1_LEVEL & FX_LOG_INFO_FLAG) == 0)
    {
        return;
    }

    FX_CHAR fmt_str[512] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(fmt_str, 511, fmt, args);
    va_end(args);
    printf("[INFO][L1] %s\n", fmt_str);
}

void _FX_WARN(const FX_CHAR *fmt, ...)
{
    if ((FX_LOG_L1_LEVEL & FX_LOG_WARN_FLAG) == 0)
    {
        return;
    }

    FX_CHAR fmt_str[512] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(fmt_str, 511, fmt, args);
    va_end(args);
    printf("[WARN][L1] %s\n", fmt_str);
}

void _FX_ERRO(const FX_CHAR *fmt, ...)
{
    if ((FX_LOG_L1_LEVEL & FX_LOG_ERROR_FLAG) == 0)
    {
        return;
    }

    FX_CHAR fmt_str[512] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(fmt_str, 511, fmt, args);
    va_end(args);
    printf("[ERRO][L1] %s\n", fmt_str);
}

int FX_L1_System_Link(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4, unsigned int log_level)
{
    assert(ip1 >= 0 && ip1 <= 255);
    assert(ip2 >= 0 && ip2 <= 255);
    assert(ip3 >= 0 && ip3 <= 255);
    assert(ip4 >= 0 && ip4 <= 255);

    FX_LOG_L1_LEVEL = log_level;
    FX_ROBOT_TYPE = FX_ROBOT_NULL;
    _FX_DEBG("%s: Set L1 API log level 0x%08x", __FUNCTION__, log_level);

    if ((ip1 == 0 && ip2 == 0 && ip3 == 0 && ip4 == 0) || (ip1 == 255 && ip2 == 255 && ip3 == 255 && ip4 == 255) || ip1 == 127)
    {
        _FX_ERRO("%s: Invalid IP address: %u.%u.%u.%u", __FUNCTION__, ip1, ip2, ip3, ip4);
        return FUNC_RET_INVALID_INPUT_ARG;
    }

    if (FX_L0_System_Link(ip1, ip2, ip3, ip4) != 0)
    {
        _FX_ERRO("%s: Link robot failed, local port 3721/3722/3723/3724 may be used by other processes", __FUNCTION__);
        FX_L0_System_Unlink();
        return FUNC_RET_LINK_FAILED;
    }

    int ret = FX_L0_System_Testconnect();
    if (ret < 0)
    {
        _FX_ERRO("%s: Link robot failed, no response in 1000ms", __FUNCTION__);
        FX_L0_System_Unlink();
        return FUNC_RET_LINK_NO_RESPONSE;
    }

    if (FX_L0_System_CheckVersion() != 0)
    {
        _FX_ERRO("%s: SDK version is not supported by robot system", __FUNCTION__);
        FX_L0_System_Unlink();
        return FUNC_RET_VERSION_INCOMPATIABLE;
    }
    _FX_INFO("%s: Link success, communication delay is %dms", __FUNCTION__, ret);

    char robot_name[30] = {0};
    char dof_arm0_name[30] = {0};
    char dof_arm1_name[30] = {0};
    char dof_head_name[30] = {0};
    char dof_body_name[30] = {0};
    char dof_lift_name[30] = {0};
    sprintf(robot_name, "R.BASIC.Name");
    if (FX_L1_Param_GetString(robot_name, FX_ROBOT_NAME) != FUNC_RET_SUCCESS)
    {
        _FX_WARN("%s: Failed to get parameter: %s", __FUNCTION__, robot_name);
        strcpy(FX_ROBOT_NAME, "UnknownRobot");
        FX_ROBOT_TYPE = FX_ROBOT_NULL;
        FX_L0_System_Unlink();
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }
    else
    {
        if (strcmp(FX_ROBOT_NAME, "MarvinProM3") == 0)
        {
            FX_ROBOT_TYPE = FX_ROBOT_MARVIN_PRO_M3;
        }
        else if (strcmp(FX_ROBOT_NAME, "MarvinProM6") == 0)
        {
            FX_ROBOT_TYPE = FX_ROBOT_MARVIN_PRO_M6;
        }
        else if (strcmp(FX_ROBOT_NAME, "GentoSkye") == 0)
        {
            FX_ROBOT_TYPE = FX_ROBOT_GENTO_SKYE;
        }
        else if (strcmp(FX_ROBOT_NAME, "GentoLuna") == 0)
        {
            FX_ROBOT_TYPE = FX_ROBOT_GENTO_LUNA;
        }
        else
        {
            _FX_WARN("%s: Link failed, invalid robot name: %s=%s", __FUNCTION__, robot_name, FX_ROBOT_NAME);
            strcpy(FX_ROBOT_NAME, "UnknownRobot");
            FX_ROBOT_TYPE = FX_ROBOT_NULL;
            FX_L0_System_Unlink();
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
    }
    sprintf(dof_arm0_name, "R.A0.BASIC.Dof");
    if (FX_L1_Param_GetInt32(dof_arm0_name, &FX_ARM0_DOF) != FUNC_RET_SUCCESS)
    {
        _FX_WARN("%s: Failed to get parameter: %s", __FUNCTION__, dof_arm0_name);
        FX_ARM0_DOF = 0;
    }
    sprintf(dof_arm1_name, "R.A1.BASIC.Dof");
    if (FX_L1_Param_GetInt32(dof_arm1_name, &FX_ARM1_DOF) != FUNC_RET_SUCCESS)
    {
        _FX_WARN("%s: Failed to get parameter: %s", __FUNCTION__, dof_arm1_name);
        FX_ARM1_DOF = 0;
    }
    sprintf(dof_body_name, "R.B.BASIC.Dof");
    if (FX_L1_Param_GetInt32(dof_body_name, &FX_BODY_DOF) != FUNC_RET_SUCCESS)
    {
        _FX_WARN("%s: Failed to get parameter: %s", __FUNCTION__, dof_body_name);
        FX_BODY_DOF = 0;
    }
    sprintf(dof_head_name, "R.H.BASIC.Dof");
    if (FX_L1_Param_GetInt32(dof_head_name, &FX_HEAD_DOF) != FUNC_RET_SUCCESS)
    {
        _FX_WARN("%s: Failed to get parameter: %s", __FUNCTION__, dof_head_name);
        FX_HEAD_DOF = 0;
    }
    sprintf(dof_lift_name, "R.L.BASIC.Dof");
    if (FX_L1_Param_GetInt32(dof_lift_name, &FX_LIFT_DOF) != FUNC_RET_SUCCESS)
    {
        _FX_WARN("%s: Failed to get parameter: %s", __FUNCTION__, dof_lift_name);
        FX_LIFT_DOF = 0;
    }
    _FX_INFO("%s: DOF of [%s]: arm0=%d, arm1=%d, head=%d, body=%d, lift=%d", __FUNCTION__, FX_ROBOT_NAME, FX_ARM0_DOF, FX_ARM1_DOF, FX_HEAD_DOF, FX_BODY_DOF, FX_LIFT_DOF);

    FX_LINK_IP[0] = ip1;
    FX_LINK_IP[1] = ip2;
    FX_LINK_IP[2] = ip3;
    FX_LINK_IP[3] = ip4;
    FX_LINK_TAG = 1;

    return ret;
}

void FX_L1_System_Unlink()
{
    FX_L0_System_Unlink();
    FX_ROBOT_TYPE = FX_ROBOT_NULL;
    FX_LINK_TAG = 0;
}

void FX_L1_System_SetLogLevel(unsigned int level)
{
    FX_LOG_L1_LEVEL = level & 0x0000000F;
}

unsigned int FX_L1_System_GetLogLevel()
{
    return FX_LOG_L1_LEVEL;
}

int FX_L1_System_GetControllerVersion()
{
    int version = FX_L0_System_GetControllerVersion();
    if (version < 0)
    {
        _FX_ERRO("%s: Failed to execute, return %d", __FUNCTION__, version);
        return FUNC_RET_OPERATION_FAILED;
    }
    else
    {
        _FX_INFO("%s: Controller version is 0x%08x", __FUNCTION__, version);
        return version;
    }
}

int FX_L1_System_GetSDKVersion()
{
    int version = FX_L0_System_GetSdkVersion();
    if (version < 0)
    {
        _FX_ERRO("%s: Failed to execute, return %d", __FUNCTION__, version);
        return FUNC_RET_OPERATION_FAILED;
    }
    else
    {
        _FX_INFO("%s: SDK version is 0x%08x", __FUNCTION__, version);
        return version;
    }
}

int FX_L1_System_Reboot()
{
    int can_reboot = 1;
    unsigned short cur_obj_state = 0;
    if (FX_ARM0_DOF > 0)
    {
        cur_obj_state = FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState;
        if (cur_obj_state != ARM_STATE_IDLE && cur_obj_state != ARM_STATE_ERROR)
        {
            _FX_WARN("%s: Arm0 is not in idle or error state, reject reboot", __FUNCTION__);
            can_reboot = 0;
        }
    }
    if (FX_ARM1_DOF > 0)
    {
        cur_obj_state = FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState;
        if (cur_obj_state != ARM_STATE_IDLE && cur_obj_state != ARM_STATE_ERROR)
        {
            _FX_WARN("%s: Arm1 is not in idle or error state, reject reboot", __FUNCTION__);
            can_reboot = 0;
        }
    }
    if (FX_HEAD_DOF > 0)
    {
        cur_obj_state = FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState;
        if (cur_obj_state != HEAD_STATE_IDLE && cur_obj_state != HEAD_STATE_ERROR)
        {
            _FX_WARN("%s: Head is not in idle or error state, reject reboot", __FUNCTION__);
            can_reboot = 0;
        }
    }
    if (FX_BODY_DOF > 0)
    {
        cur_obj_state = FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState;
        if (cur_obj_state != BODY_STATE_IDLE && cur_obj_state != BODY_STATE_ERROR)
        {
            _FX_WARN("%s: Body is not in idle or error state, reject reboot", __FUNCTION__);
            can_reboot = 0;
        }
    }
    if (FX_LIFT_DOF > 0)
    {
        cur_obj_state = FX_L0_GetRobotRT()->m_LIFT.m_LIFT_State.m_CurState;
        if (cur_obj_state != LIFT_STATE_IDLE && cur_obj_state != LIFT_STATE_ERROR)
        {
            _FX_WARN("%s: Lift is not in idle or error state, reject reboot", __FUNCTION__);
            can_reboot = 0;
        }
    }
    if (!can_reboot)
    {
        return FUNC_RET_INVALID_CONDITION;
    }

    if (FX_L0_System_Reboot() != 0)
    {
        _FX_ERRO("%s: Failed to reboot controller", __FUNCTION__);
        return FUNC_RET_OPERATION_FAILED;
    }
    _FX_INFO("%s: Controller system is going to reboot in seconds", __FUNCTION__);
    return FUNC_RET_SUCCESS;
}

int FX_L1_System_Update(char *update_file_path, char *ini_file_path)
{
    if (update_file_path == NULL && ini_file_path == NULL)
    {
        _FX_ERRO("%s: Update file path and ini file path are all invalid", __FUNCTION__);
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    if (update_file_path != NULL)
    {
        if (FX_L1_System_SendFile(update_file_path, (char *)"/home/FUSION/Tmp/update_package.UPDATE") != 0)
        {
            _FX_ERRO("%s: Failed to transfer update file to the controller", __FUNCTION__);
            return FUNC_RET_SEND_FILE_FAILED;
        }
    }
    if (ini_file_path != NULL)
    {
        if (FX_L1_System_SendFile(ini_file_path, (char *)"/home/FUSION/Tmp/robot.ini.UPDATE") != 0)
        {
            _FX_ERRO("%s: Failed to transfer ini file to the controller", __FUNCTION__);
            return FUNC_RET_RECV_FILE_FAILED;
        }
    }
    if (FX_L0_System_Update() != 0)
    {
        _FX_ERRO("%s: Failed to set controller update flag", __FUNCTION__);
        return FUNC_RET_OPERATION_FAILED;
    }
    _FX_INFO("%s: Update preparations are done, please reboot the controller to finish the update operation", __FUNCTION__);
    return FUNC_RET_SUCCESS;
}

int FX_L1_System_SendFile(char *local_file_path, char *remote_file_path)
{
    int ret = FX_FileClient_SendFile(FX_LINK_IP[0], FX_LINK_IP[1], FX_LINK_IP[2], FX_LINK_IP[3], local_file_path, remote_file_path);
    if (ret == 0)
    {
        _FX_INFO("%s: Transfer Local[%s]--->Controller[%s] success", __FUNCTION__, local_file_path, remote_file_path);
        return FUNC_RET_SUCCESS;
    }
    else
    {
        _FX_ERRO("%s: Transfer Local[%s]--->Controller[%s] failed, return %d", __FUNCTION__, local_file_path, remote_file_path, ret);
        return FUNC_RET_SEND_FILE_FAILED;
    }
}

int FX_L1_System_RecvFile(char *local_file_path, char *remote_file_path)
{
    int ret = FX_FileClient_RecvFile(FX_LINK_IP[0], FX_LINK_IP[1], FX_LINK_IP[2], FX_LINK_IP[3], local_file_path, remote_file_path);
    if (ret == 0)
    {
        _FX_INFO("%s: Transfer Controller[%s]--->Local[%s] success", __FUNCTION__, remote_file_path, local_file_path);
        return FUNC_RET_SUCCESS;
    }
    else
    {
        _FX_ERRO("%s: Transfer Controller[%s]--->Local[%s] failed, return %d", __FUNCTION__, remote_file_path, local_file_path, ret);
        return FUNC_RET_RECV_FILE_FAILED;
    }
}

int FX_L1_Comm_Clear(unsigned int timeout)
{
    if (FX_L0_Communication_Clear(timeout) == 0)
    {
        return FUNC_RET_SUCCESS;
    }
    else
    {
        _FX_ERRO("%s: Wait command ready to send timeout", __FUNCTION__);
        return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
    }
}

int FX_L1_Comm_Send()
{
    if (FX_L0_Communication_Send() == 0)
    {
        return FUNC_RET_SUCCESS;
    }
    else
    {
        _FX_ERRO("%s: Send command failed", __FUNCTION__);
        return FUNC_RET_COMM_SEND_FAILED;
    }
}

int FX_L1_Comm_SendAndWait(unsigned int timeout)
{
    int ret = FX_L0_Communication_SendWaitResponse(timeout);
    if (ret < 0)
    {
        _FX_ERRO("%s: Wait command reply timeout", __FUNCTION__);
        return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
    }
    else
    {
        return ret;
    }
}

int FX_L1_Fbk_GetCtrlObjDof(FXObjType obj_type)
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
        return FX_ARM0_DOF;
    case FX_OBJ_ARM1:
        return FX_ARM1_DOF;
    case FX_OBJ_HEAD:
        return FX_HEAD_DOF;
    case FX_OBJ_BODY:
        return FX_BODY_DOF;
    case FX_OBJ_LIFT:
        return FX_LIFT_DOF;
    default:
        return -1;
    }
}

void FX_L1_Fbk_GetCtrlObjServoVersion(FXObjType obj_type, char version[7][30])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        for (FX_INT32 i = 0; i < 7; i++)
        {
            if (i < FX_ARM0_DOF)
            {
                if (FX_L0_Arm0_State_GetServoVersion(i, version[i]) != 0)
                {
                    _FX_WARN("%s: Failed to get %s.Axis%d servo version", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                    version[i][0] = 0;
                }
            }
            else
            {
                version[i][0] = 0;
            }
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        for (FX_INT32 i = 0; i < 7; i++)
        {
            if (i < FX_ARM1_DOF)
            {
                if (FX_L0_Arm1_State_GetServoVersion(i, version[i]) != 0)
                {
                    _FX_WARN("%s: Failed to get %s.Axis%d servo version", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                    version[i][0] = 0;
                }
            }
            else
            {
                version[i][0] = 0;
            }
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        for (FX_INT32 i = 0; i < 7; i++)
        {
            if (i < FX_HEAD_DOF)
            {
                if (FX_L0_Head_State_GetServoVersion(i, version[i]) != 0)
                {
                    _FX_WARN("%s: Failed to get %s.Axis%d servo version", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                    version[i][0] = 0;
                }
            }
            else
            {
                version[i][0] = 0;
            }
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        for (FX_INT32 i = 0; i < 7; i++)
        {
            if (i < FX_BODY_DOF)
            {
                if (FX_L0_Body_State_GetServoVersion(i, version[i]) != 0)
                {
                    _FX_WARN("%s: Failed to get %s.Axis%d servo version", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                    version[i][0] = 0;
                }
            }
            else
            {
                version[i][0] = 0;
            }
        }
        break;
    }
    case FX_OBJ_LIFT:
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        version[0][0] = 0;
        version[1][0] = 0;
        version[2][0] = 0;
        version[3][0] = 0;
        version[4][0] = 0;
        version[5][0] = 0;
        version[6][0] = 0;
    }
    }
}

void FX_L1_Fbk_GetCtrlObjSensorVersionAndSerial(FXObjType obj_type, FX_INT32 version[7], FX_INT32 serial[7])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        for (FX_INT32 i = 0; i < 7; i++)
        {
            if (i < FX_ARM0_DOF)
            {
                if (FX_L0_Arm0_State_GetSensorVersion(i, &version[i]) != 0)
                {
                    _FX_WARN("%s: Failed to get %s.Axis%d sensor version", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                    version[i] = 0;
                }
                if (FX_L0_Arm0_State_GetSensorSerial(i, &serial[i]) != 0)
                {
                    _FX_WARN("%s: Failed to get %s.Axis%d sensor serial", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                    serial[i] = 0;
                }
            }
            else
            {
                version[i] = 0;
                serial[i] = 0;
            }
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        for (FX_INT32 i = 0; i < 7; i++)
        {
            if (i < FX_ARM1_DOF)
            {
                if (FX_L0_Arm1_State_GetSensorVersion(i, &version[i]) != 0)
                {
                    _FX_WARN("%s: Failed to get %s.Axis%d sensor version", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                    version[i] = 0;
                }
                if (FX_L0_Arm1_State_GetSensorSerial(i, &serial[i]) != 0)
                {
                    _FX_WARN("%s: Failed to get %s.Axis%d sensor serial", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                    serial[i] = 0;
                }
            }
            else
            {
                version[i] = 0;
                serial[i] = 0;
            }
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        for (FX_INT32 i = 0; i < 7; i++)
        {
            if (i < FX_BODY_DOF)
            {
                if (FX_L0_Body_State_GetSensorVersion(i, &version[i]) != 0)
                {
                    _FX_WARN("%s: Failed to get %s.Axis%d sensor version", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                    version[i] = 0;
                }
                if (FX_L0_Body_State_GetSensorSerial(i, &serial[i]) != 0)
                {
                    _FX_WARN("%s: Failed to get %s.Axis%d sensor serial", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                    serial[i] = 0;
                }
            }
            else
            {
                version[i] = 0;
                serial[i] = 0;
            }
        }
        break;
    }
    case FX_OBJ_HEAD:
    case FX_OBJ_LIFT:
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        for (FX_INT32 i = 0; i < 7; i++)
        {
            version[i] = 0;
            serial[i] = 0;
        }
    }
    }
}

int FX_L1_Fbk_GetCtrlObjPhysicalState(FXObjType obj_type, int *physical_state)
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_State_GetPhysicalState(physical_state) != 0)
        {
            _FX_ERRO("%s: Failed to get %s physical state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_State_GetPhysicalState(physical_state) != 0)
        {
            _FX_ERRO("%s: Failed to get %s physical state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_L0_Head_State_GetPhysicalState(physical_state) != 0)
        {
            _FX_ERRO("%s: Failed to get %s physical state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_Body_State_GetPhysicalState(physical_state) != 0)
        {
            _FX_ERRO("%s: Failed to get %s physical state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_LIFT:
    {
        if (FX_L0_Lift_State_GetPhysicalState(physical_state) != 0)
        {
            _FX_ERRO("%s: Failed to get %s physical state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }

    return FUNC_RET_SUCCESS;
}

FXRobotType FX_L1_Fbk_GetRobotType()
{
    return FX_ROBOT_TYPE;
}

FXStateType FX_L1_Fbk_CurrentState(FXObjType obj_type)
{
    int cur_state = 0;
    int drag_type = 0;
    const ROBOT_RT *rt = FX_L0_GetRobotRT();
    const ROBOT_SG *sg = FX_L0_GetRobotSG();

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        cur_state = rt->m_ARMS[0].m_ARM_State.m_CurState;
        if (cur_state == ARM_STATE_IDLE)
        {
            return FX_STATE_IDLE;
        }
        else if (cur_state == ARM_STATE_POSITION)
        {
            return FX_STATE_POSITION;
        }
        else if (cur_state == ARM_STATE_TORQUE)
        {
            drag_type = rt->m_ARMS[0].m_ARM_IN.m_ARM_CMD_Ctrl_DragType;
            switch (sg->m_ARMS[0].m_ARM_SET.m_ARM_Ctrl_ImpType)
            {
            case FX_IMP_TYPE_NULL:
                break;
            case FX_IMP_TYPE_JOINT:
            {
                if (drag_type == FX_DRAG_TYPE_JOINT)
                {
                    return FX_STATE_DRAG_JOINT;
                }
                else
                {
                    return FX_STATE_IMP_JOINT;
                }
            }
            case FX_IMP_TYPE_CART:
            {
                if (drag_type == FX_DRAG_TYPE_CART_X)
                {
                    return FX_STATE_DRAG_CART_X;
                }
                else if (drag_type == FX_DRAG_TYPE_CART_Y)
                {
                    return FX_STATE_DRAG_CART_Y;
                }
                else if (drag_type == FX_DRAG_TYPE_CART_Z)
                {
                    return FX_STATE_DRAG_CART_Z;
                }
                else if (drag_type == FX_DRAG_TYPE_CART_R)
                {
                    return FX_STATE_DRAG_CART_R;
                }
                else
                {
                    return FX_STATE_IMP_CART;
                }
            }
            case FX_IMP_TYPE_FORCE:
                return FX_STATE_IMP_FORCE;
            default:
                return FX_STATE_UNKNOWN;
            }
        }
        else if (cur_state == ARM_STATE_RELEASE)
        {
            return FX_STATE_RELEASE;
        }
        else if (cur_state == ARM_STATE_ERROR)
        {
            return FX_STATE_ERROR;
        }
        else if (cur_state == ARM_STATE_TRANS_TO_POSITION || cur_state == ARM_STATE_TRANS_TO_TORQUE || cur_state == ARM_STATE_TRANS_TO_RELEASE || cur_state == ARM_STATE_TRANS_TO_IDLE)
        {
            return FX_STATE_TRANSFERRING;
        }
        else
        {
            return FX_STATE_UNKNOWN;
        }
    }
    case FX_OBJ_ARM1:
    {
        cur_state = rt->m_ARMS[1].m_ARM_State.m_CurState;
        if (cur_state == ARM_STATE_IDLE)
        {
            return FX_STATE_IDLE;
        }
        else if (cur_state == ARM_STATE_POSITION)
        {
            return FX_STATE_POSITION;
        }
        else if (cur_state == ARM_STATE_TORQUE)
        {
            drag_type = rt->m_ARMS[1].m_ARM_IN.m_ARM_CMD_Ctrl_DragType;
            switch (sg->m_ARMS[1].m_ARM_SET.m_ARM_Ctrl_ImpType)
            {
            case FX_IMP_TYPE_NULL:
                break;
            case FX_IMP_TYPE_JOINT:
            {
                if (drag_type == FX_DRAG_TYPE_JOINT)
                {
                    return FX_STATE_DRAG_JOINT;
                }
                else
                {
                    return FX_STATE_IMP_JOINT;
                }
            }
            case FX_IMP_TYPE_CART:
            {
                if (drag_type == FX_DRAG_TYPE_CART_X)
                {
                    return FX_STATE_DRAG_CART_X;
                }
                else if (drag_type == FX_DRAG_TYPE_CART_Y)
                {
                    return FX_STATE_DRAG_CART_Y;
                }
                else if (drag_type == FX_DRAG_TYPE_CART_Z)
                {
                    return FX_STATE_DRAG_CART_Z;
                }
                else if (drag_type == FX_DRAG_TYPE_CART_R)
                {
                    return FX_STATE_DRAG_CART_R;
                }
                else
                {
                    return FX_STATE_IMP_CART;
                }
            }
            case FX_IMP_TYPE_FORCE:
                return FX_STATE_IMP_FORCE;
            default:
                return FX_STATE_UNKNOWN;
            }
        }
        else if (cur_state == ARM_STATE_RELEASE)
        {
            return FX_STATE_RELEASE;
        }
        else if (cur_state == ARM_STATE_ERROR)
        {
            return FX_STATE_ERROR;
        }
        else if (cur_state == ARM_STATE_TRANS_TO_POSITION || cur_state == ARM_STATE_TRANS_TO_TORQUE || cur_state == ARM_STATE_TRANS_TO_RELEASE || cur_state == ARM_STATE_TRANS_TO_IDLE)
        {
            return FX_STATE_TRANSFERRING;
        }
        else
        {
            return FX_STATE_UNKNOWN;
        }
    }
    case FX_OBJ_HEAD:
    {
        cur_state = rt->m_HEAD.m_HEAD_State.m_CurState;
        if (cur_state == HEAD_STATE_IDLE)
        {
            return FX_STATE_IDLE;
        }
        else if (cur_state == HEAD_STATE_POSITION)
        {
            return FX_STATE_POSITION;
        }
        else if (cur_state == HEAD_STATE_ERROR)
        {
            return FX_STATE_ERROR;
        }
        else if (cur_state == HEAD_STATE_TRANS_TO_POSITION || cur_state == HEAD_STATE_TRANS_TO_IDLE)
        {
            return FX_STATE_TRANSFERRING;
        }
        else
        {
            return FX_STATE_UNKNOWN;
        }
    }
    case FX_OBJ_BODY:
    {
        cur_state = rt->m_BODY.m_BODY_State.m_CurState;
        if (cur_state == BODY_STATE_IDLE)
        {
            return FX_STATE_IDLE;
        }
        else if (cur_state == BODY_STATE_POSITION)
        {
            return FX_STATE_POSITION;
        }
        else if (cur_state == BODY_STATE_TORQUE)
        {
            return FX_STATE_IMP_JOINT;
        }
        else if (cur_state == BODY_STATE_RELEASE)
        {
            return FX_STATE_RELEASE;
        }
        else if (cur_state == BODY_STATE_ERROR)
        {
            return FX_STATE_ERROR;
        }
        else if (cur_state == BODY_STATE_TRANS_TO_POSITION || cur_state == BODY_STATE_TRANS_TO_TORQUE || cur_state == BODY_STATE_TRANS_TO_RELEASE || cur_state == BODY_STATE_TRANS_TO_IDLE)
        {
            return FX_STATE_TRANSFERRING;
        }
        else
        {
            return FX_STATE_UNKNOWN;
        }
    }
    case FX_OBJ_LIFT:
    {
        cur_state = rt->m_LIFT.m_LIFT_State.m_CurState;
        if (cur_state == LIFT_STATE_IDLE)
        {
            return FX_STATE_IDLE;
        }
        else if (cur_state == LIFT_STATE_POSITION)
        {
            return FX_STATE_POSITION;
        }
        else if (cur_state == LIFT_STATE_ERROR)
        {
            return FX_STATE_ERROR;
        }
        else if (cur_state == LIFT_STATE_TRANS_TO_POSITION || cur_state == LIFT_STATE_TRANS_TO_IDLE)
        {
            return FX_STATE_TRANSFERRING;
        }
        else
        {
            return FX_STATE_UNKNOWN;
        }
    }
    default:
    {
        return FX_STATE_UNKNOWN;
    }
    }
}

const ROBOT_RT *FX_L1_Fbk_GetRT()
{
    return FX_L0_GetRobotRT();
}

const ROBOT_SG *FX_L1_Fbk_GetSG()
{
    return FX_L0_GetRobotSG();
}

int FX_L1_State_GetServoErrorCode(FXObjType obj_type, unsigned int error_code[7])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        for (int i = 0; i < FX_ARM0_DOF; i++)
        {
            if (FX_L0_Arm0_State_GetServoErrorCode(i, &error_code[i]) != 0)
            {
                _FX_ERRO("%s: Failed to get %s.Axis%d servo error code", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                return FUNC_RET_OPERATION_FAILED;
            }
        }
        break;
    }

    case FX_OBJ_ARM1:
    {
        for (int i = 0; i < FX_ARM1_DOF; i++)
        {
            if (FX_L0_Arm1_State_GetServoErrorCode(i, &error_code[i]) != 0)
            {
                _FX_ERRO("%s: Failed to get %s.Axis%d servo error code", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                return FUNC_RET_OPERATION_FAILED;
            }
        }
        break;
    }

    case FX_OBJ_HEAD:
    {
        for (int i = 0; i < FX_HEAD_DOF; i++)
        {
            if (FX_L0_Head_State_GetServoErrorCode(i, &error_code[i]) != 0)
            {
                _FX_ERRO("%s: Failed to get %s.Axis%d servo error code", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                return FUNC_RET_OPERATION_FAILED;
            }
        }
        break;
    }

    case FX_OBJ_BODY:
    {
        for (int i = 0; i < FX_BODY_DOF; i++)
        {
            if (FX_L0_Body_State_GetServoErrorCode(i, &error_code[i]) != 0)
            {
                _FX_ERRO("%s: Failed to get %s.Axis%d servo error code", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                return FUNC_RET_OPERATION_FAILED;
            }
        }
        break;
    }

    case FX_OBJ_LIFT:
    {
        for (int i = 0; i < FX_LIFT_DOF; i++)
        {
            if (FX_L0_Lift_State_GetServoErrorCode(i, &error_code[i]) != 0)
            {
                _FX_ERRO("%s: Failed to get %s.Axis%d servo error code", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                return FUNC_RET_OPERATION_FAILED;
            }
        }
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_State_ResetError(FXObjType obj_type, unsigned int timeout, unsigned int *system_errorcode)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    const FX_UINT32 *errorcode_ptr = NULL;
    *system_errorcode = 0;
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_State_Reset() != 0)
        {
            _FX_ERRO("%s: Failed to reset %s error", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        errorcode_ptr = &(FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_ERRCode);
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_State_Reset() != 0)
        {
            _FX_ERRO("%s: Failed to reset %s error", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        errorcode_ptr = &(FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_ERRCode);
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_Head_State_Reset() != 0)
        {
            _FX_ERRO("%s: Failed to reset %s error", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        errorcode_ptr = &(FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_ERRCode);
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_Body_State_Reset() != 0)
        {
            _FX_ERRO("%s: Failed to reset %s error", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        errorcode_ptr = &(FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_ERRCode);
        break;
    }
    case FX_OBJ_LIFT:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_Lift_State_Reset() != 0)
        {
            _FX_ERRO("%s: Failed to reset %s error", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        errorcode_ptr = &(FX_L0_GetRobotRT()->m_LIFT.m_LIFT_State.m_ERRCode);
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }

    if (timeout == 0) // non-block
    {
        _FX_INFO("%s: %s processes the operation success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    while (timeout > 0)
    {
        CUtility::UniMilliSleep(1);
        if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_IDLE)
        {
            _FX_INFO("%s: %s processes the operation success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        timeout--;
    }
    *system_errorcode = *errorcode_ptr;
    _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
    return FUNC_RET_OPERATION_TIMEOUT;
}

int FX_L1_State_SwitchToIdle(FXObjType obj_type, unsigned int timeout)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_IDLE:
    {
        _FX_INFO("%s: %s state is already in STATE_IDLE", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    case FX_STATE_POSITION:
    case FX_STATE_IMP_JOINT:
    case FX_STATE_IMP_CART:
    case FX_STATE_IMP_FORCE:
    case FX_STATE_DRAG_JOINT:
    case FX_STATE_DRAG_CART_X:
    case FX_STATE_DRAG_CART_Y:
    case FX_STATE_DRAG_CART_Z:
    case FX_STATE_DRAG_CART_R:
    case FX_STATE_RELEASE:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (FX_L0_Arm0_Runtime_SetState(ARM_STATE_IDLE) != 0)
            {
                _FX_ERRO("%s: %s format command Runtime_SetState failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (FX_L0_Arm1_Runtime_SetState(ARM_STATE_IDLE) != 0)
            {
                _FX_ERRO("%s: %s format command Runtime_SetState failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        {
            if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
            {
                return FUNC_RET_INVALID_ROBOT_TYPE;
            }
            if (FX_L0_Head_Runtime_SetState(HEAD_STATE_IDLE) != 0)
            {
                _FX_ERRO("%s: %s format command Runtime_SetState failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_BODY:
        {
            if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
            {
                return FUNC_RET_INVALID_ROBOT_TYPE;
            }
            if (FX_L0_Body_Runtime_SetState(BODY_STATE_IDLE) != 0)
            {
                _FX_ERRO("%s: %s format command Runtime_SetState failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_LIFT:
        {
            if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE)
            {
                return FUNC_RET_INVALID_ROBOT_TYPE;
            }
            if (FX_L0_Lift_Runtime_SetState(LIFT_STATE_IDLE) != 0)
            {
                _FX_ERRO("%s: %s format command Runtime_SetState failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        default:
        {
            _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command Runtime_InitTraj reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            _FX_INFO("%s: %s transfer to STATE_IDLE success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        while (timeout > 0)
        {
            CUtility::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_IDLE)
            {
                _FX_INFO("%s: %s transfer to STATE_IDLE success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_SUCCESS;
            }
            timeout--;
        }
        _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_OPERATION_TIMEOUT;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        _FX_ERRO("%s: %s is not allowed to do the operation, current state is %s", __FUNCTION__, _FX_ObjType2Str(obj_type), _FX_StateType2Str(cur_state));
        return FUNC_RET_INVALID_CONDITION;
    }
    }
}

int FX_L1_State_SwitchToPositionMode(FXObjType obj_type, unsigned int timeout, double vel_ratio, double acc_ratio)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    if (vel_ratio < 1)
    {
        vel_ratio = 1;
    }
    else if (vel_ratio > 100)
    {
        vel_ratio = 100;
    }

    if (acc_ratio < 1)
    {
        acc_ratio = 1;
    }
    else if (acc_ratio > 100)
    {
        acc_ratio = 100;
    }

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_POSITION:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        {
            if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
            {
                return FUNC_RET_INVALID_ROBOT_TYPE;
            }
            if (FX_L0_Head_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Head_Runtime_SetAccRatio(acc_ratio) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_BODY:
        {
            if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
            {
                return FUNC_RET_INVALID_ROBOT_TYPE;
            }
            if (FX_L0_Body_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Body_Runtime_SetAccRatio(acc_ratio) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_LIFT:
        {
            if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE)
            {
                return FUNC_RET_INVALID_ROBOT_TYPE;
            }
            if (FX_L0_Lift_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Lift_Runtime_SetAccRatio(acc_ratio) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        _FX_INFO("%s: %s state is already in STATE_POSITION", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    case FX_STATE_IDLE:
    case FX_STATE_IMP_JOINT:
    case FX_STATE_IMP_CART:
    case FX_STATE_IMP_FORCE:
    case FX_STATE_DRAG_JOINT:
    case FX_STATE_DRAG_CART_X:
    case FX_STATE_DRAG_CART_Y:
    case FX_STATE_DRAG_CART_Z:
    case FX_STATE_DRAG_CART_R:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Arm0_Runtime_SetState(ARM_STATE_POSITION) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Arm1_Runtime_SetState(ARM_STATE_POSITION) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        {
            if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
            {
                return FUNC_RET_INVALID_ROBOT_TYPE;
            }
            if (FX_L0_Head_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Head_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Head_Runtime_SetState(HEAD_STATE_POSITION) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_BODY:
        {
            if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
            {
                return FUNC_RET_INVALID_ROBOT_TYPE;
            }
            if (FX_L0_Body_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Body_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Body_Runtime_SetState(BODY_STATE_POSITION) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_LIFT:
        {
            if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE)
            {
                return FUNC_RET_INVALID_ROBOT_TYPE;
            }
            if (FX_L0_Lift_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Lift_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Lift_Runtime_SetState(LIFT_STATE_POSITION) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            _FX_INFO("%s: %s transfer to STATE_POSITION success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        while (timeout > 0)
        {
            CUtility::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_POSITION)
            {
                CUtility::UniMilliSleep(250); // delay 250ms to let servo get ready to follow position command in CSP mode
                _FX_INFO("%s: %s transfer to STATE_POSITION success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_SUCCESS;
            }
            timeout--;
        }
        _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_OPERATION_TIMEOUT;
    }
    case FX_STATE_ERROR:
    case FX_STATE_RELEASE:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        _FX_ERRO("%s: %s is not allowed to do the operation, current state is %s", __FUNCTION__, _FX_ObjType2Str(obj_type), _FX_StateType2Str(cur_state));
        return FUNC_RET_INVALID_CONDITION;
    }
    }
}

int FX_L1_State_SwitchToImpJointMode(FXObjType obj_type, unsigned int timeout, double vel_ratio, double acc_ratio, double k[7], double d[7])
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    if (vel_ratio < 1)
    {
        vel_ratio = 1;
    }
    else if (vel_ratio > 100)
    {
        vel_ratio = 100;
    }

    if (acc_ratio < 1)
    {
        acc_ratio = 1;
    }
    else if (acc_ratio > 100)
    {
        acc_ratio = 100;
    }

    for (int i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_IMP_JOINT:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_BODY:
        {
            if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
            {
                return FUNC_RET_INVALID_ROBOT_TYPE;
            }
            if (FX_L0_Body_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Body_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Body_Runtime_SetPDP(k) != 0 || FX_L0_Body_Runtime_SetPDD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        _FX_INFO("%s: %s state is already in STATE_IMP_JOINT", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    case FX_STATE_IDLE:
    case FX_STATE_POSITION:
    case FX_STATE_IMP_CART:
    case FX_STATE_IMP_FORCE:
    case FX_STATE_DRAG_JOINT:
    case FX_STATE_DRAG_CART_X:
    case FX_STATE_DRAG_CART_Y:
    case FX_STATE_DRAG_CART_Z:
    case FX_STATE_DRAG_CART_R:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0 || FX_L0_Arm0_Runtime_SetDragType(FX_DRAG_TYPE_NULL) != 0 || FX_L0_Arm0_Runtime_SetImpType(FX_IMP_TYPE_JOINT) != 0 || FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0 || FX_L0_Arm1_Runtime_SetDragType(FX_DRAG_TYPE_NULL) != 0 || FX_L0_Arm1_Runtime_SetImpType(FX_IMP_TYPE_JOINT) != 0 || FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_BODY:
        {
            if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
            {
                return FUNC_RET_INVALID_ROBOT_TYPE;
            }
            if (FX_L0_Body_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Body_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Body_Runtime_SetPDP(k) != 0 || FX_L0_Body_Runtime_SetPDD(d) != 0 || FX_L0_Body_Runtime_SetState(BODY_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            _FX_INFO("%s: %s transfer to STATE_IMP_JOINT success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        while (timeout > 0)
        {
            CUtility::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_IMP_JOINT)
            {
                _FX_INFO("%s: %s transfer to STATE_IMP_JOINT success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_SUCCESS;
            }
            timeout--;
        }
        _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_OPERATION_TIMEOUT;
    }
    case FX_STATE_ERROR:
    case FX_STATE_RELEASE:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        _FX_ERRO("%s: %s is not allowed to do the operation, current state is %s", __FUNCTION__, _FX_ObjType2Str(obj_type), _FX_StateType2Str(cur_state));
        return FUNC_RET_INVALID_CONDITION;
    }
    }
}

int FX_L1_State_SwitchToImpCartMode(FXObjType obj_type, unsigned int timeout, double vel_ratio, double acc_ratio, double k[7], double d[7])
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    if (vel_ratio < 1)
    {
        vel_ratio = 1;
    }
    else if (vel_ratio > 100)
    {
        vel_ratio = 100;
    }

    if (acc_ratio < 1)
    {
        acc_ratio = 1;
    }
    else if (acc_ratio > 100)
    {
        acc_ratio = 100;
    }

    for (int i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_IMP_CART:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Arm0_Runtime_SetCartK(k) != 0 || FX_L0_Arm0_Runtime_SetCartD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Arm1_Runtime_SetCartK(k) != 0 || FX_L0_Arm1_Runtime_SetCartD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        _FX_INFO("%s: %s state is already in STATE_IMP_CART", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    case FX_STATE_IDLE:
    case FX_STATE_POSITION:
    case FX_STATE_IMP_JOINT:
    case FX_STATE_IMP_FORCE:
    case FX_STATE_DRAG_JOINT:
    case FX_STATE_DRAG_CART_X:
    case FX_STATE_DRAG_CART_Y:
    case FX_STATE_DRAG_CART_Z:
    case FX_STATE_DRAG_CART_R:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Arm0_Runtime_SetCartK(k) != 0 || FX_L0_Arm0_Runtime_SetCartD(d) != 0 || FX_L0_Arm0_Runtime_SetDragType(FX_DRAG_TYPE_NULL) != 0 || FX_L0_Arm0_Runtime_SetImpType(FX_IMP_TYPE_CART) != 0 || FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                return -7;
            }
            if (FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) != 0 || FX_L0_Arm1_Runtime_SetCartK(k) != 0 || FX_L0_Arm1_Runtime_SetCartD(d) != 0 || FX_L0_Arm1_Runtime_SetDragType(FX_DRAG_TYPE_NULL) != 0 || FX_L0_Arm1_Runtime_SetImpType(FX_IMP_TYPE_CART) != 0 || FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            _FX_INFO("%s: %s transfer to STATE_IMP_CART success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        while (timeout > 0)
        {
            CUtility::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_IMP_CART)
            {
                _FX_INFO("%s: %s transfer to STATE_IMP_CART success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_SUCCESS;
            }
            timeout--;
        }
        _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_OPERATION_TIMEOUT;
    }
    case FX_STATE_ERROR:
    case FX_STATE_RELEASE:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        _FX_ERRO("%s: %s is not allowed to do the operation, current state is %s", __FUNCTION__, _FX_ObjType2Str(obj_type), _FX_StateType2Str(cur_state));
        return FUNC_RET_INVALID_CONDITION;
    }
    }
}

int FX_L1_State_SwitchToImpForceMode(FXObjType obj_type, unsigned int timeout, double force_ctrl[FX_FORCE_DEF_NUM], double torque_ctrl[FX_TORQUE_DEF_NUM])
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    double force_dir_vector_len = force_ctrl[FX_FORCE_DIR_X] * force_ctrl[FX_FORCE_DIR_X] + force_ctrl[FX_FORCE_DIR_Y] * force_ctrl[FX_FORCE_DIR_Y] + force_ctrl[FX_FORCE_DIR_Z] * force_ctrl[FX_FORCE_DIR_Z];
    if (force_dir_vector_len < 0.1)
    {
        return -8;
    }
    if (force_ctrl[FX_FORCE_VALUE] < 0)
    {
        force_ctrl[FX_FORCE_VALUE] = 0;
    }
    if (force_ctrl[FX_FORCE_DISTANCE] < 0)
    {
        force_ctrl[FX_FORCE_DISTANCE] = 0;
    }

    double torque_dir_vector_len = torque_ctrl[FX_TORQUE_DIR_A] * torque_ctrl[FX_TORQUE_DIR_A] + torque_ctrl[FX_TORQUE_DIR_B] * torque_ctrl[FX_TORQUE_DIR_B] + torque_ctrl[FX_TORQUE_DIR_C] * torque_ctrl[FX_TORQUE_DIR_C];
    if (torque_dir_vector_len < 0.1)
    {
        return -8;
    }
    if (torque_ctrl[FX_TORQUE_VALUE] < 0)
    {
        torque_ctrl[FX_TORQUE_VALUE] = 0;
    }
    if (torque_ctrl[FX_TORQUE_ANGLE] < 0)
    {
        torque_ctrl[FX_TORQUE_ANGLE] = 0;
    }

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_IMP_FORCE:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (FX_L0_Arm0_Runtime_SetForceCtrl(force_ctrl) != 0 || FX_L0_Arm0_Runtime_SetTorqueCtrl(torque_ctrl) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (FX_L0_Arm1_Runtime_SetForceCtrl(force_ctrl) != 0 || FX_L0_Arm1_Runtime_SetTorqueCtrl(torque_ctrl) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        _FX_INFO("%s: %s state is already in STATE_IMP_FORCE", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    case FX_STATE_IDLE:
    case FX_STATE_POSITION:
    case FX_STATE_IMP_JOINT:
    case FX_STATE_IMP_CART:
    case FX_STATE_DRAG_JOINT:
    case FX_STATE_DRAG_CART_X:
    case FX_STATE_DRAG_CART_Y:
    case FX_STATE_DRAG_CART_Z:
    case FX_STATE_DRAG_CART_R:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm0_Runtime_SetForceCtrl(force_ctrl) != 0 || FX_L0_Arm0_Runtime_SetTorqueCtrl(torque_ctrl) != 0 || FX_L0_Arm0_Runtime_SetDragType(FX_DRAG_TYPE_NULL) != 0 || FX_L0_Arm0_Runtime_SetImpType(FX_IMP_TYPE_FORCE) != 0 || FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm1_Runtime_SetForceCtrl(force_ctrl) != 0 || FX_L0_Arm1_Runtime_SetTorqueCtrl(torque_ctrl) != 0 || FX_L0_Arm1_Runtime_SetDragType(FX_DRAG_TYPE_NULL) != 0 || FX_L0_Arm1_Runtime_SetImpType(FX_IMP_TYPE_FORCE) != 0 || FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            _FX_INFO("%s: %s transfer to STATE_IMP_FORCE success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        while (timeout > 0)
        {
            CUtility::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_IMP_FORCE)
            {
                _FX_INFO("%s: %s transfer to STATE_IMP_FORCE success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_SUCCESS;
            }
            timeout--;
        }
        _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_OPERATION_TIMEOUT;
    }
    case FX_STATE_ERROR:
    case FX_STATE_RELEASE:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        _FX_ERRO("%s: %s is not allowed to do the operation, current state is %s", __FUNCTION__, _FX_ObjType2Str(obj_type), _FX_StateType2Str(cur_state));
        return FUNC_RET_INVALID_CONDITION;
    }
    }
}

int FX_L1_State_SwitchToDragJoint(FXObjType obj_type, unsigned int timeout, double k[7], double d[7])
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    for (int i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_DRAG_JOINT:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_BODY:
        case FX_OBJ_HEAD:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        _FX_INFO("%s: %s state is already in STATE_DRAG_JOINT", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    case FX_STATE_IDLE:
    case FX_STATE_POSITION:
    case FX_STATE_IMP_JOINT:
    case FX_STATE_IMP_CART:
    case FX_STATE_IMP_FORCE:
    case FX_STATE_DRAG_CART_X:
    case FX_STATE_DRAG_CART_Y:
    case FX_STATE_DRAG_CART_Z:
    case FX_STATE_DRAG_CART_R:
    case FX_STATE_RELEASE:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm0_Runtime_SetDragType(FX_DRAG_TYPE_JOINT) != 0 || FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0 || FX_L0_Arm0_Runtime_SetImpType(FX_IMP_TYPE_JOINT) != 0 || FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm1_Runtime_SetDragType(FX_DRAG_TYPE_JOINT) != 0 || FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0 || FX_L0_Arm1_Runtime_SetImpType(FX_IMP_TYPE_JOINT) != 0 || FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            _FX_INFO("%s: %s transfer to STATE_DRAG_JOINT success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        while (timeout > 0)
        {
            CUtility::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_DRAG_JOINT)
            {
                _FX_INFO("%s: %s transfer to STATE_DRAG_JOINT success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_SUCCESS;
            }
            timeout--;
        }
        _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_OPERATION_TIMEOUT;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        _FX_ERRO("%s: %s is not allowed to do the operation, current state is %s", __FUNCTION__, _FX_ObjType2Str(obj_type), _FX_StateType2Str(cur_state));
        return FUNC_RET_INVALID_CONDITION;
    }
    }
}

int FX_L1_State_SwitchToDragCartX(FXObjType obj_type, unsigned int timeout, double k[7], double d[7])
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    for (int i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_DRAG_CART_X:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_BODY:
        case FX_OBJ_HEAD:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        _FX_INFO("%s: %s state is already in STATE_DRAG_X", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    case FX_STATE_IDLE:
    case FX_STATE_POSITION:
    case FX_STATE_IMP_JOINT:
    case FX_STATE_IMP_CART:
    case FX_STATE_IMP_FORCE:
    case FX_STATE_DRAG_JOINT:
    case FX_STATE_DRAG_CART_Y:
    case FX_STATE_DRAG_CART_Z:
    case FX_STATE_DRAG_CART_R:
    case FX_STATE_RELEASE:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm0_Runtime_SetDragType(FX_DRAG_TYPE_CART_X) != 0 || FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0 || FX_L0_Arm0_Runtime_SetImpType(FX_IMP_TYPE_CART) != 0 || FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm1_Runtime_SetDragType(FX_DRAG_TYPE_CART_X) != 0 || FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0 || FX_L0_Arm1_Runtime_SetImpType(FX_IMP_TYPE_CART) != 0 || FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            _FX_INFO("%s: %s transfer to STATE_DRAG_X success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        while (timeout > 0)
        {
            CUtility::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_DRAG_CART_X)
            {
                _FX_INFO("%s: %s transfer to STATE_DRAG_X success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_SUCCESS;
            }
            timeout--;
        }
        _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_OPERATION_TIMEOUT;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        _FX_ERRO("%s: %s is not allowed to do the operation, current state is %s", __FUNCTION__, _FX_ObjType2Str(obj_type), _FX_StateType2Str(cur_state));
        return FUNC_RET_INVALID_CONDITION;
    }
    }
}

int FX_L1_State_SwitchToDragCartY(FXObjType obj_type, unsigned int timeout, double k[7], double d[7])
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    for (int i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_DRAG_CART_Y:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_BODY:
        case FX_OBJ_HEAD:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        _FX_INFO("%s: %s state is already in STATE_DRAG_Y", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    case FX_STATE_IDLE:
    case FX_STATE_POSITION:
    case FX_STATE_IMP_JOINT:
    case FX_STATE_IMP_CART:
    case FX_STATE_IMP_FORCE:
    case FX_STATE_DRAG_JOINT:
    case FX_STATE_DRAG_CART_X:
    case FX_STATE_DRAG_CART_Z:
    case FX_STATE_DRAG_CART_R:
    case FX_STATE_RELEASE:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm0_Runtime_SetDragType(FX_DRAG_TYPE_CART_Y) != 0 || FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0 || FX_L0_Arm0_Runtime_SetImpType(FX_IMP_TYPE_CART) != 0 || FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm1_Runtime_SetDragType(FX_DRAG_TYPE_CART_Y) != 0 || FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0 || FX_L0_Arm1_Runtime_SetImpType(FX_IMP_TYPE_CART) != 0 || FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            _FX_INFO("%s: %s transfer to STATE_DRAG_Y success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        while (timeout > 0)
        {
            CUtility::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_DRAG_CART_Y)
            {
                _FX_INFO("%s: %s transfer to STATE_DRAG_Y success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_SUCCESS;
            }
            timeout--;
        }
        _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_OPERATION_TIMEOUT;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        _FX_ERRO("%s: %s is not allowed to do the operation, current state is %s", __FUNCTION__, _FX_ObjType2Str(obj_type), _FX_StateType2Str(cur_state));
        return FUNC_RET_INVALID_CONDITION;
    }
    }
}

int FX_L1_State_SwitchToDragCartZ(FXObjType obj_type, unsigned int timeout, double k[7], double d[7])
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    for (int i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_DRAG_CART_Z:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_BODY:
        case FX_OBJ_HEAD:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        _FX_INFO("%s: %s state is already in STATE_DRAG_Z", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    case FX_STATE_IDLE:
    case FX_STATE_POSITION:
    case FX_STATE_IMP_JOINT:
    case FX_STATE_IMP_CART:
    case FX_STATE_IMP_FORCE:
    case FX_STATE_DRAG_JOINT:
    case FX_STATE_DRAG_CART_X:
    case FX_STATE_DRAG_CART_Y:
    case FX_STATE_DRAG_CART_R:
    case FX_STATE_RELEASE:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm0_Runtime_SetDragType(FX_DRAG_TYPE_CART_Z) != 0 || FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0 || FX_L0_Arm0_Runtime_SetImpType(FX_IMP_TYPE_CART) != 0 || FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm1_Runtime_SetDragType(FX_DRAG_TYPE_CART_Z) != 0 || FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0 || FX_L0_Arm1_Runtime_SetImpType(FX_IMP_TYPE_CART) != 0 || FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            _FX_INFO("%s: %s transfer to STATE_DRAG_Z success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        while (timeout > 0)
        {
            CUtility::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_DRAG_CART_Z)
            {
                _FX_INFO("%s: %s transfer to STATE_DRAG_Z success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_SUCCESS;
            }
            timeout--;
        }
        _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_OPERATION_TIMEOUT;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        _FX_ERRO("%s: %s is not allowed to do the operation, current state is %s", __FUNCTION__, _FX_ObjType2Str(obj_type), _FX_StateType2Str(cur_state));
        return FUNC_RET_INVALID_CONDITION;
    }
    }
}

int FX_L1_State_SwitchToDragCartR(FXObjType obj_type, unsigned int timeout, double k[7], double d[7])
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    for (int i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_DRAG_CART_R:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_BODY:
        case FX_OBJ_HEAD:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        _FX_INFO("%s: %s state is already in STATE_DRAG_R", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    case FX_STATE_IDLE:
    case FX_STATE_POSITION:
    case FX_STATE_IMP_JOINT:
    case FX_STATE_IMP_CART:
    case FX_STATE_IMP_FORCE:
    case FX_STATE_DRAG_JOINT:
    case FX_STATE_DRAG_CART_X:
    case FX_STATE_DRAG_CART_Y:
    case FX_STATE_DRAG_CART_Z:
    case FX_STATE_RELEASE:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm0_Runtime_SetDragType(FX_DRAG_TYPE_CART_R) != 0 || FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0 || FX_L0_Arm0_Runtime_SetImpType(FX_IMP_TYPE_CART) != 0 || FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm1_Runtime_SetDragType(FX_DRAG_TYPE_CART_R) != 0 || FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0 || FX_L0_Arm1_Runtime_SetImpType(FX_IMP_TYPE_CART) != 0 || FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            _FX_INFO("%s: %s transfer to STATE_DRAG_R success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        while (timeout > 0)
        {
            CUtility::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_DRAG_CART_R)
            {
                _FX_INFO("%s: %s transfer to STATE_DRAG_R success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_SUCCESS;
            }
            timeout--;
        }
        _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_OPERATION_TIMEOUT;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        _FX_ERRO("%s: %s is not allowed to do the operation, current state is %s", __FUNCTION__, _FX_ObjType2Str(obj_type), _FX_StateType2Str(cur_state));
        return FUNC_RET_INVALID_CONDITION;
    }
    }
}

int FX_L1_State_SwitchToCollaborativeRelease(FXObjType obj_type, unsigned int timeout)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_RELEASE:
    {
        _FX_INFO("%s: %s state is already in STATE_RELEASE", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_SUCCESS;
    }
    case FX_STATE_IDLE:
    {
        // send cmd
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm0_Runtime_SetState(ARM_STATE_RELEASE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_FBK_LowSpdFlag != 1)
            {
                _FX_ERRO("%s: %s is not allowed to do the operation, it is moving", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_INVALID_CONDITION;
            }
            if (FX_L0_Arm1_Runtime_SetState(ARM_STATE_RELEASE) != 0)
            {
                _FX_ERRO("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_OBJ;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            _FX_INFO("%s: %s transfer to STATE_RELEASE success [non-block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_SUCCESS;
        }
        while (timeout > 0)
        {
            CUtility::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_RELEASE)
            {
                _FX_INFO("%s: %s transfer to STATE_RELEASE success [block]", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_SUCCESS;
            }
            timeout--;
        }
        _FX_ERRO("%s: %s processes the operation timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_OPERATION_TIMEOUT;
    }
    case FX_STATE_POSITION:
    case FX_STATE_IMP_JOINT:
    case FX_STATE_IMP_CART:
    case FX_STATE_IMP_FORCE:
    case FX_STATE_DRAG_JOINT:
    case FX_STATE_DRAG_CART_X:
    case FX_STATE_DRAG_CART_Y:
    case FX_STATE_DRAG_CART_Z:
    case FX_STATE_DRAG_CART_R:
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        _FX_ERRO("%s: %s is not allowed to do the operation, current state is %s", __FUNCTION__, _FX_ObjType2Str(obj_type), _FX_StateType2Str(cur_state));
        return FUNC_RET_INVALID_CONDITION;
    }
    }
}

int FX_L1_Param_SetInt32(char *name, int value)
{
    if (name == NULL || strlen(name) > 29)
    {
        _FX_ERRO("%s: Parameter name is not valid", __FUNCTION__);
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    if (FX_L0_Param_SetInt(name, value) != 0)
    {
        _FX_ERRO("%s: Failed to set %s=%d", __FUNCTION__, name, value);
        return FUNC_RET_SET_PARAM_FAILED;
    }
    if (FX_L0_Param_Save() != 0)
    {
        _FX_ERRO("%s: Failed to save %s=%d to controller config file", __FUNCTION__, name, value);
        return FUNC_RET_SAVE_PARAM_FAILED;
    }
    _FX_INFO("%s: Set %s=%d success", __FUNCTION__, name, value);
    return FUNC_RET_SUCCESS;
}

int FX_L1_Param_SetFloat(char *name, float value)
{
    if (name == NULL || strlen(name) > 29)
    {
        _FX_ERRO("%s: Parameter name is not valid", __FUNCTION__);
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    if (FX_L0_Param_SetFloat(name, value) != 0)
    {
        _FX_ERRO("%s: Failed to set %s=%.4f", __FUNCTION__, name, value);
        return FUNC_RET_SET_PARAM_FAILED;
    }
    if (FX_L0_Param_Save() != 0)
    {
        _FX_ERRO("%s: Failed to save %s=%.4f to controller config file", __FUNCTION__, name, value);
        return FUNC_RET_SAVE_PARAM_FAILED;
    }
    _FX_INFO("%s: Set %s=%.4f success", __FUNCTION__, name, value);
    return FUNC_RET_SUCCESS;
}

int FX_L1_Param_GetInt32(char *name, int *value)
{
    if (name == NULL || strlen(name) > 29)
    {
        _FX_ERRO("%s: Parameter name is not valid", __FUNCTION__);
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    if (FX_L0_Param_GetInt(name, value) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name);
        return FUNC_RET_GET_PARAM_FAILED;
    }
    _FX_DEBG("%s: Get %s=%d success", __FUNCTION__, name, *value);
    return FUNC_RET_SUCCESS;
}

int FX_L1_Param_GetFloat(char *name, float *value)
{
    if (name == NULL || strlen(name) > 29)
    {
        _FX_ERRO("%s: Parameter name is not valid", __FUNCTION__);
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    if (FX_L0_Param_GetFloat(name, value) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name);
        return FUNC_RET_GET_PARAM_FAILED;
    }
    _FX_DEBG("%s: Get %s=%.4f success", __FUNCTION__, name, *value);
    return FUNC_RET_SUCCESS;
}

int FX_L1_Param_GetString(char *name, char *value)
{
    if (name == NULL || strlen(name) > 29)
    {
        _FX_ERRO("%s: Parameter name is not valid", __FUNCTION__);
        return FUNC_RET_INVALID_INPUT_ARG;
    }
    if (FX_L0_Param_GetString(name, value) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name);
        return FUNC_RET_GET_PARAM_FAILED;
    }
    _FX_DEBG("%s: Get %s=%s success", __FUNCTION__, name, value);
    return FUNC_RET_SUCCESS;
}

int FX_L1_Terminal_ClearData(FXTerminalType terminal_type)
{
    switch (terminal_type)
    {
    case FX_TERMINAL_ARM0:
        return FX_L0_Arm0_Terminal_ClearData();
    case FX_TERMINAL_ARM1:
        return FX_L0_Arm1_Terminal_ClearData();
    default:
        return FUNC_RET_INVALID_TERMINAL;
    }
}

int FX_L1_Terminal_GetData(FXTerminalType terminal_type, FXChnType *chn_type, unsigned char data[64])
{
    switch (terminal_type)
    {
    case FX_TERMINAL_ARM0:
        return FX_L0_Arm0_Terminal_GetData((int *)chn_type, data);
    case FX_TERMINAL_ARM1:
        return FX_L0_Arm1_Terminal_GetData((int *)chn_type, data);
    default:
        return FUNC_RET_INVALID_TERMINAL;
    }
}

int FX_L1_Terminal_SetData(FXTerminalType terminal_type, FXChnType chn_type, unsigned char data[64], unsigned int data_len)
{
    switch (terminal_type)
    {
    case FX_TERMINAL_ARM0:
        return FX_L0_Arm0_Terminal_SetData(chn_type, data, data_len);
    case FX_TERMINAL_ARM1:
        return FX_L0_Arm1_Terminal_SetData(chn_type, data, data_len);
    default:
        return FUNC_RET_INVALID_TERMINAL;
    }
}

int FX_L1_Config_SetBrakeLock(FXObjType obj_type, unsigned char axis_mask)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm0_Config_SetBrakeLock(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm1_Config_SetBrakeLock(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState != HEAD_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Head_Config_SetBrakeLock(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Body_Config_SetBrakeLock(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    _FX_INFO("%s: %s does the operation success", __FUNCTION__, _FX_ObjType2Str(obj_type));
    return FUNC_RET_SUCCESS;
}

int FX_L1_Config_SetBrakeUnlock(FXObjType obj_type, unsigned char axis_mask)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm0_Config_SetBrakeUnlock(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm1_Config_SetBrakeUnlock(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState != HEAD_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Head_Config_SetBrakeUnlock(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Body_Config_SetBrakeUnlock(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    _FX_INFO("%s: %s does the operation success", __FUNCTION__, _FX_ObjType2Str(obj_type));
    return FUNC_RET_SUCCESS;
}

int FX_L1_Config_ResetEncOffset(FXObjType obj_type, unsigned char axis_mask)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm0_Config_ResetEncMultiTurn(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to reset encoder multi-turn for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        CUtility::UniMilliSleep(100);
        if (FX_L0_Arm0_Config_ResetEncSingleTurn(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to reset encoder single-turn for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm1_Config_ResetEncMultiTurn(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to reset encoder multi-turn for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        CUtility::UniMilliSleep(100);
        if (FX_L0_Arm1_Config_ResetEncSingleTurn(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to reset encoder single-turn for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState != HEAD_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Head_Config_ResetEncMultiTurn(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to reset encoder multi-turn for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        CUtility::UniMilliSleep(100);
        if (FX_L0_Head_Config_ResetEncSingleTurn(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to reset encoder single-turn for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Body_Config_ResetEncMultiTurn(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to reset encoder multi-turn for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        CUtility::UniMilliSleep(100);
        if (FX_L0_Body_Config_ResetEncSingleTurn(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to reset encoder single-turn for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_LIFT:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_LIFT.m_LIFT_State.m_CurState != LIFT_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Lift_Config_ResetEncOffset(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to reset encoder offset for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    _FX_INFO("%s: %s does the operation success for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
    return FUNC_RET_SUCCESS;
}

int FX_L1_Config_ClearEncError(FXObjType obj_type, unsigned char axis_mask)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm0_Config_ClearEncError(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm1_Config_ClearEncError(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState != HEAD_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Head_Config_ClearEncError(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Body_Config_ClearEncError(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    _FX_INFO("%s: %s does the operation success for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
    return FUNC_RET_SUCCESS;
}

int FX_L1_Config_ResetAxisSensorOffset(FXObjType obj_type, unsigned int axis_id)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    int offset = 0;
    // compute offset
    char SensorK_name[30] = {0};
    char SensorDir_name[30] = {0};
    float sensor_value = 0;
    const ROBOT_RT *rt = FX_L0_GetRobotRT();
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (axis_id >= (unsigned int)FX_ARM0_DOF)
        {
            _FX_ERRO("%s: %s doesn't support axis_id=%u", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_id);
            return FUNC_RET_INVALID_INPUT_ARG;
        }
        sprintf(SensorK_name, "R.A0.L%d.BASIC.SensorK", axis_id);
        sprintf(SensorDir_name, "R.A0.L%d.BASIC.SensorDir", axis_id);
        sensor_value = rt->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor[axis_id];
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (axis_id >= (unsigned int)FX_ARM1_DOF)
        {
            _FX_ERRO("%s: %s doesn't support axis_id=%u", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_id);
            return FUNC_RET_INVALID_INPUT_ARG;
        }
        sprintf(SensorK_name, "R.A1.L%d.BASIC.SensorK", axis_id);
        sprintf(SensorDir_name, "R.A1.L%d.BASIC.SensorDir", axis_id);
        sensor_value = rt->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor[axis_id];
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (axis_id >= (unsigned int)FX_BODY_DOF)
        {
            _FX_ERRO("%s: %s doesn't support axis_id=%u", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_id);
            return FUNC_RET_INVALID_INPUT_ARG;
        }
        sprintf(SensorK_name, "R.B.L%d.BASIC.SensorK", axis_id);
        sprintf(SensorDir_name, "R.B.L%d.BASIC.SensorDir", axis_id);
        sensor_value = rt->m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_SensorTor[axis_id];
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    float sensor_k = 0;
    int sensor_dir = 0;
    if (FX_L0_Param_GetFloat(SensorK_name, &sensor_k) != 0 || FX_L0_Param_GetInt(SensorDir_name, &sensor_dir) != 0)
    {
        _FX_ERRO("%s: Failed to get parameter %s or %s", __FUNCTION__, SensorK_name, SensorDir_name);
        return FUNC_RET_GET_PARAM_FAILED;
    }
    if (sensor_k < 0.00001)
    {
        _FX_ERRO("%s: Invalid parameter setting, get %s=%.6f, value is too small", __FUNCTION__, SensorK_name, sensor_k);
        return FUNC_RET_INVALID_PARAM_SETTING;
    }
    if (sensor_dir == 0)
    {
        offset = sensor_value / sensor_k;
    }
    else
    {
        offset = -sensor_value / sensor_k;
    }
    _FX_DEBG("%s: Get %s=%.6f, %s=%d ===> offset = %d", __FUNCTION__, SensorK_name, sensor_k, SensorDir_name, sensor_dir, offset);

    // set offset
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm0_Config_SetSensorOffset(axis_id, offset) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_id=%u", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_id);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm1_Config_SetSensorOffset(axis_id, offset) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_id=%u", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_id);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Body_Config_SetSensorOffset(axis_id, offset) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_id=%u", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_id);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    _FX_INFO("%s: %s does the operation success for axis_id=%u", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_id);
    return FUNC_RET_SUCCESS;
}

int FX_L1_Config_ResetSensorOffset(FXObjType obj_type)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    int offset[8] = {0};
    // compute offset
    char SensorK_name[8][30] = {{0}};
    char SensorDir_name[8][30] = {{0}};
    float sensor_value[8] = {0};
    int dof = 0;
    const ROBOT_RT *rt = FX_L0_GetRobotRT();
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        dof = FX_ARM0_DOF;
        for (int i = 0; i < dof; i++)
        {
            sprintf(SensorK_name[i], "R.A0.L%d.BASIC.SensorK", i);
            sprintf(SensorDir_name[i], "R.A0.L%d.BASIC.SensorDir", i);
            sensor_value[i] = rt->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor[i];
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        dof = FX_ARM1_DOF;
        for (int i = 0; i < dof; i++)
        {
            sprintf(SensorK_name[i], "R.A1.L%d.BASIC.SensorK", i);
            sprintf(SensorDir_name[i], "R.A1.L%d.BASIC.SensorDir", i);
            sensor_value[i] = rt->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor[i];
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        dof = FX_BODY_DOF;
        for (int i = 0; i < dof; i++)
        {
            sprintf(SensorK_name[i], "R.B.L%d.BASIC.SensorK", i);
            sprintf(SensorDir_name[i], "R.B.L%d.BASIC.SensorDir", i);
            sensor_value[i] = rt->m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_SensorTor[i];
        }
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    float sensor_k[8] = {0};
    int sensor_dir[8] = {0};
    for (int i = 0; i < dof; i++)
    {
        if (FX_L0_Param_GetFloat(SensorK_name[i], &sensor_k[i]) != 0 || FX_L0_Param_GetInt(SensorDir_name[i], &sensor_dir[i]) != 0)
        {
            _FX_ERRO("%s: Failed to get parameter %s or %s", __FUNCTION__, SensorK_name[i], SensorDir_name[i]);
            return FUNC_RET_GET_PARAM_FAILED;
        }
        if (sensor_k[i] < 0.00001)
        {
            _FX_ERRO("%s: Invalid parameter setting, get %s=%.6f, value is too small", __FUNCTION__, SensorK_name[i], sensor_k[i]);
            return FUNC_RET_INVALID_PARAM_SETTING;
        }
        if (sensor_dir[i] == 0)
        {
            offset[i] = sensor_value[i] / sensor_k[i];
        }
        else
        {
            offset[i] = -sensor_value[i] / sensor_k[i];
        }
    }

    // set offset
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        for (int i = 0; i < FX_ARM0_DOF; i++)
        {
            if (FX_L0_Arm0_Config_SetSensorOffset(i, offset[i]) != 0)
            {
                _FX_ERRO("%s: %s failed to do the operation for axis_id=%u", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                return FUNC_RET_OPERATION_FAILED;
            }
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        for (int i = 0; i < FX_ARM1_DOF; i++)
        {
            if (FX_L0_Arm1_Config_SetSensorOffset(i, offset[i]) != 0)
            {
                _FX_ERRO("%s: %s failed to do the operation for axis_id=%u", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                return FUNC_RET_OPERATION_FAILED;
            }
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        for (int i = 0; i < FX_BODY_DOF; i++)
        {
            if (FX_L0_Body_Config_SetSensorOffset(i, offset[i]) != 0)
            {
                _FX_ERRO("%s: %s failed to do the operation for axis_id=%u", __FUNCTION__, _FX_ObjType2Str(obj_type), i);
                return FUNC_RET_OPERATION_FAILED;
            }
        }
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    _FX_INFO("%s: %s does the operation success", __FUNCTION__, _FX_ObjType2Str(obj_type));
    return 0;
}

int FX_L1_Config_DisableSoftLimit(FXObjType obj_type, unsigned char axis_mask)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm0_Config_DisableSoftLimit(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Arm1_Config_DisableSoftLimit(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState != HEAD_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Head_Config_DisableSoftLimit(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE && FX_ROBOT_TYPE != FX_ROBOT_GENTO_LUNA)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Body_Config_DisableSoftLimit(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    case FX_OBJ_LIFT:
    {
        if (FX_ROBOT_TYPE != FX_ROBOT_GENTO_SKYE)
        {
            return FUNC_RET_INVALID_ROBOT_TYPE;
        }
        if (FX_L0_GetRobotRT()->m_LIFT.m_LIFT_State.m_CurState != LIFT_STATE_IDLE)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_IDLE state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (FX_L0_Lift_Config_DisableSoftLimit(axis_mask) != 0)
        {
            _FX_ERRO("%s: %s failed to do the operation for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
            return FUNC_RET_OPERATION_FAILED;
        }
        break;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    _FX_INFO("%s: %s does the operation success for axis_mask=0x%02x", __FUNCTION__, _FX_ObjType2Str(obj_type), axis_mask);
    return FUNC_RET_SUCCESS;
}

int FX_L1_Config_SetTraj(FXObjType obj_type, unsigned int point_num, double *point_data)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (RobotCtrl::GetIns()->m_RobotRT.m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_POSITION)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_POSITION state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState >= 3)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it is tracking a planned trajectory now", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        // InitTraj
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command Runtime_InitTraj ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        if (FX_L0_Arm0_Runtime_InitTraj(point_num) != 0)
        {
            _FX_ERRO("%s: %s format command Runtime_InitTraj failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command Runtime_InitTraj reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        int wait_count = 10;
        do
        {
            CUtility::UniMilliSleep(1);
            wait_count--;
            if (wait_count == 0)
            {
                _FX_ERRO("%s: %s processes the operation Runtime_InitTraj timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_OPERATION_TIMEOUT;
            }
        } while (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState != 1);
        // SetTraj
        unsigned int full_frame_num = point_num / 50;
        unsigned int relic_point_num = point_num % 50;
        for (unsigned int i = 0; i < full_frame_num; i++)
        {
            if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
            {
                _FX_ERRO("%s: %s wait command Runtime_SetTraj ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
            }
            if (FX_L0_Arm0_Runtime_SetTraj(i, 50, &point_data[350 * i]) != 0)
            {
                _FX_ERRO("%s: %s format command Runtime_SetTraj failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
            {
                _FX_ERRO("%s: %s wait command Runtime_SetTraj reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
            }
        }
        if (relic_point_num != 0)
        {
            if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
            {
                _FX_ERRO("%s: %s wait command Runtime_SetTraj ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
            }
            if (FX_L0_Arm0_Runtime_SetTraj(full_frame_num, relic_point_num, &point_data[350 * full_frame_num]) != 0)
            {
                _FX_ERRO("%s: %s format command Runtime_SetTraj failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
            {
                _FX_ERRO("%s: %s wait command Runtime_SetTraj reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
            }
        }
        wait_count = 10;
        do
        {
            CUtility::UniMilliSleep(1);
            wait_count--;
            if (wait_count == 0)
            {
                _FX_ERRO("%s: %s processes the operation Runtime_SetTraj timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_OPERATION_TIMEOUT;
            }
        } while (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[0].m_ARM_GET.m_ARM_FBK_TrajState != 2);

        _FX_INFO("%s: %s set a trajectory with %d points success", __FUNCTION__, _FX_ObjType2Str(obj_type), point_num);
        return FUNC_RET_SUCCESS;
    }
    case FX_OBJ_ARM1:
    {
        if (RobotCtrl::GetIns()->m_RobotRT.m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_POSITION)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it should be in STATE_POSITION state", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        if (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[1].m_ARM_GET.m_ARM_FBK_TrajState >= 3)
        {
            _FX_ERRO("%s: %s is not allowed to do the operation, it is tracking a planned trajectory now", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_INVALID_CONDITION;
        }
        // InitTraj
        if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
        {
            _FX_ERRO("%s: %s wait command Runtime_InitTraj ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
        }
        if (FX_L0_Arm1_Runtime_InitTraj(point_num) != 0)
        {
            _FX_ERRO("%s: %s format command Runtime_InitTraj failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
        {
            _FX_ERRO("%s: %s wait command Runtime_InitTraj reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
        }
        int wait_count = 10;
        do
        {
            CUtility::UniMilliSleep(1);
            wait_count--;
            if (wait_count == 0)
            {
                _FX_ERRO("%s: %s processes the operation Runtime_InitTraj timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_OPERATION_TIMEOUT;
            }
        } while (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[1].m_ARM_GET.m_ARM_FBK_TrajState != 1);
        // SetTraj
        unsigned int full_frame_num = point_num / 50;
        unsigned int relic_point_num = point_num % 50;
        for (unsigned int i = 0; i < full_frame_num; i++)
        {
            if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
            {
                _FX_ERRO("%s: %s wait command Runtime_SetTraj ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
            }
            if (FX_L0_Arm1_Runtime_SetTraj(i, 50, &point_data[350 * i]) != 0)
            {
                _FX_ERRO("%s: %s format command Runtime_SetTraj failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
            {
                _FX_ERRO("%s: %s wait command Runtime_SetTraj reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
            }
        }
        if (relic_point_num != 0)
        {
            if (FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT) != 0)
            {
                _FX_ERRO("%s: %s wait command Runtime_SetTraj ready to send timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_COMM_WAIT_READY_TIMEOUT;
            }
            if (FX_L0_Arm1_Runtime_SetTraj(full_frame_num, relic_point_num, &point_data[350 * full_frame_num]) != 0)
            {
                _FX_ERRO("%s: %s format command Runtime_SetTraj failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_FORMAT_CMD_FAILED;
            }
            if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) < 0)
            {
                _FX_ERRO("%s: %s wait command Runtime_SetTraj reply timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_COMM_WAIT_REPLY_TIMEOUT;
            }
        }
        wait_count = 10;
        do
        {
            CUtility::UniMilliSleep(1);
            wait_count--;
            if (wait_count == 0)
            {
                _FX_ERRO("%s: %s processes the operation Runtime_SetTraj timeout", __FUNCTION__, _FX_ObjType2Str(obj_type));
                return FUNC_RET_OPERATION_TIMEOUT;
            }
        } while (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[1].m_ARM_GET.m_ARM_FBK_TrajState != 2);

        _FX_INFO("%s: %s set a trajectory with %d points success", __FUNCTION__, _FX_ObjType2Str(obj_type), point_num);
        return FUNC_RET_SUCCESS;
    }
    default:
    {
        _FX_ERRO("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
}

unsigned int FX_L1_Runtime_EmergencyStop(unsigned int obj_mask)
{
    unsigned int ret_obj_mask = 0;
    if ((obj_mask & FX_OBJ_ARM0_FLAG) != 0)
    {
        if (FX_L0_Arm0_Runtime_EmergencyStop() == 0)
        {
            ret_obj_mask |= FX_OBJ_ARM0_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_ARM1_FLAG) != 0)
    {
        if (FX_L0_Arm1_Runtime_EmergencyStop() == 0)
        {
            ret_obj_mask |= FX_OBJ_ARM1_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_HEAD_FLAG) != 0)
    {
        if (FX_L0_Head_Runtime_EmergencyStop() == 0)
        {
            ret_obj_mask |= FX_OBJ_HEAD_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_BODY_FLAG) != 0)
    {
        if (FX_L0_Body_Runtime_EmergencyStop() == 0)
        {
            ret_obj_mask |= FX_OBJ_BODY_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_LIFT_FLAG) != 0)
    {
        if (FX_L0_Lift_Runtime_EmergencyStop() == 0)
        {
            ret_obj_mask |= FX_OBJ_LIFT_FLAG;
        }
    }
    _FX_INFO("%s: Set command Runtime_EmergencyStop success with obj_mask=0x%02x", __FUNCTION__, ret_obj_mask);
    return ret_obj_mask;
}

int FX_L1_Runtime_SetJointPosCmd(FXObjType obj_type, double pos_cmd[7])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetJointPosCmd(pos_cmd) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetJointPosCmd(pos_cmd) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_L0_Head_Runtime_SetJointPosCmd(pos_cmd) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_Body_Runtime_SetJointPosCmd(pos_cmd) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_LIFT:
    {
        if (FX_L0_Lift_Runtime_SetJointPosCmd(pos_cmd) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetForceCtrl(FXObjType obj_type, double force_ctrl[FX_FORCE_DEF_NUM])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetForceCtrl(force_ctrl) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetForceCtrl(force_ctrl) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetTorqueCtrl(FXObjType obj_type, double torque_ctrl[FX_TORQUE_DEF_NUM])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetTorqueCtrl(torque_ctrl) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetTorqueCtrl(torque_ctrl) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetVelRatio(FXObjType obj_type, double vel_ratio)
{
    if (vel_ratio < 1)
    {
        vel_ratio = 1;
    }
    else if (vel_ratio > 100)
    {
        vel_ratio = 100;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_L0_Head_Runtime_SetVelRatio(vel_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_Body_Runtime_SetVelRatio(vel_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_LIFT:
    {
        if (FX_L0_Lift_Runtime_SetVelRatio(vel_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetAccRatio(FXObjType obj_type, double acc_ratio)
{
    if (acc_ratio < 1)
    {
        acc_ratio = 1;
    }
    else if (acc_ratio > 100)
    {
        acc_ratio = 100;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_L0_Head_Runtime_SetAccRatio(acc_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_Body_Runtime_SetAccRatio(acc_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_LIFT:
    {
        if (FX_L0_Lift_Runtime_SetAccRatio(acc_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetSpeedRatio(FXObjType obj_type, double vel_ratio, double acc_ratio)
{
    if (vel_ratio < 1)
    {
        vel_ratio = 1;
    }
    else if (vel_ratio > 100)
    {
        vel_ratio = 100;
    }

    if (acc_ratio < 1)
    {
        acc_ratio = 1;
    }
    else if (acc_ratio > 100)
    {
        acc_ratio = 100;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_L0_Head_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Head_Runtime_SetAccRatio(acc_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_Body_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Body_Runtime_SetAccRatio(acc_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_LIFT:
    {
        if (FX_L0_Lift_Runtime_SetVelRatio(vel_ratio) != 0 || FX_L0_Lift_Runtime_SetAccRatio(acc_ratio) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetJointK(FXObjType obj_type, double k[7])
{
    for (int i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetJointK(k) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetJointK(k) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetJointD(FXObjType obj_type, double d[7])
{
    for (int i = 0; i < 7; i++)
    {
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetJointD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetJointD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetJointKD(FXObjType obj_type, double k[7], double d[7])
{
    for (int i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetJointK(k) != 0 || FX_L0_Arm0_Runtime_SetJointD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetJointK(k) != 0 || FX_L0_Arm1_Runtime_SetJointD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetCartK(FXObjType obj_type, double k[7])
{
    for (int i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetCartK(k) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetCartK(k) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetCartD(FXObjType obj_type, double d[7])
{
    for (int i = 0; i < 7; i++)
    {
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetJointD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetJointD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetCartKD(FXObjType obj_type, double k[7], double d[7])
{
    for (int i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetCartK(k) != 0 || FX_L0_Arm0_Runtime_SetCartD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetCartK(k) != 0 || FX_L0_Arm1_Runtime_SetCartD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetToolK(FXObjType obj_type, double k[6])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetToolK(k) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetToolK(k) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetToolD(FXObjType obj_type, double d[10])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetToolD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetToolD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetToolKD(FXObjType obj_type, double k[6], double d[10])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_Arm0_Runtime_SetToolK(k) != 0 || FX_L0_Arm0_Runtime_SetToolD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_Arm1_Runtime_SetToolK(k) != 0 || FX_L0_Arm1_Runtime_SetToolD(d) != 0)
        {
            _FX_WARN("%s: %s format command failed", __FUNCTION__, _FX_ObjType2Str(obj_type));
            return FUNC_RET_FORMAT_CMD_FAILED;
        }
        break;
    }
    default:
    {
        _FX_WARN("%s: %s doesn't support the operation", __FUNCTION__, _FX_ObjType2Str(obj_type));
        return FUNC_RET_INVALID_OBJ;
    }
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetBodyPDP(double p[6])
{
    for (int i = 0; i < 6; i++)
    {
        if (p[i] < 0)
        {
            p[i] = 0;
        }
    }

    if (FX_L0_Body_Runtime_SetPDP(p) != 0)
    {
        _FX_WARN("%s: Body format command failed", __FUNCTION__);
        return FUNC_RET_FORMAT_CMD_FAILED;
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetBodyPDD(double d[6])
{
    for (int i = 0; i < 6; i++)
    {
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    if (FX_L0_Body_Runtime_SetPDD(d) != 0)
    {
        _FX_WARN("%s: Body format command failed", __FUNCTION__);
        return FUNC_RET_FORMAT_CMD_FAILED;
    }
    return FUNC_RET_SUCCESS;
}

int FX_L1_Runtime_SetBodyPD(double p[6], double d[6])
{
    for (int i = 0; i < 6; i++)
    {
        if (p[i] < 0)
        {
            p[i] = 0;
        }
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    if (FX_L0_Body_Runtime_SetPDP(p) != 0 || FX_L0_Body_Runtime_SetPDD(d) != 0)
    {
        _FX_WARN("%s: Body format command failed", __FUNCTION__);
        return FUNC_RET_FORMAT_CMD_FAILED;
    }
    return FUNC_RET_SUCCESS;
}

unsigned int FX_L1_Runtime_RunTraj(unsigned int obj_mask)
{
    unsigned int ret_obj_mask = 0;
    if ((obj_mask & FX_OBJ_ARM0_FLAG) != 0)
    {
        if (FX_L0_Arm0_Runtime_RunTraj() == 0)
        {
            ret_obj_mask |= FX_OBJ_ARM0_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_ARM1_FLAG) != 0)
    {
        if (FX_L0_Arm1_Runtime_RunTraj() == 0)
        {
            ret_obj_mask |= FX_OBJ_ARM1_FLAG;
        }
    }
    _FX_INFO("%s: Set command Runtime_RunTraj success with obj_mask=0x%02x", __FUNCTION__, ret_obj_mask);
    return ret_obj_mask;
}

unsigned int FX_L1_Runtime_StopTraj(unsigned int obj_mask)
{
    unsigned int ret_obj_mask = 0;
    if ((obj_mask & FX_OBJ_ARM0_FLAG) != 0)
    {
        if (FX_L0_Arm0_Runtime_StopTraj())
        {
            ret_obj_mask |= FX_OBJ_ARM0_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_ARM1_FLAG) != 0)
    {
        if (FX_L0_Arm1_Runtime_StopTraj())
        {
            ret_obj_mask |= FX_OBJ_ARM1_FLAG;
        }
    }
    _FX_INFO("%s: Set command Runtime_StopTraj success with obj_mask=0x%02x", __FUNCTION__, ret_obj_mask);
    return ret_obj_mask;
}

FX_MotionHandle FX_L1_Kinematics_Create(void)
{
    return FX_L0_Kinematics_create();
}

void FX_L1_Kinematics_Destroy(FX_MotionHandle handle)
{
    FX_L0_Kinematics_destroy(handle);
}

void FX_L1_Kinematics_SetLogLevel(unsigned int log_level)
{
    FX_L0_Kinematics_set_log_level(log_level);
}

int FX_L1_Kinematics_InitSingleArm_ByInputParams(FX_MotionHandle handle, int RobotSerial, int *type, double DH[8][4], double PNVA[8][4], double BOUND[4][3],
                                                 double GRV[3], double MASS[7], double MCP[7][3], double I[7][6])
{
    return (FX_L0_Kinematics_init_single_arm(handle, RobotSerial, type, DH, PNVA, BOUND, GRV, MASS, MCP, I) == FX_MOTION_OK) ? FUNC_RET_SUCCESS : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_InitSingleArm_ByIniConfig(FX_MotionHandle handle, int RobotSerial)
{
    if (FX_ROBOT_TYPE == FX_ROBOT_NULL)
    {
        return FUNC_RET_INVALID_ROBOT_TYPE;
    }

    _FX_DEBG("%s: Initializing Kinematics for Robot Serial: %d\n", __FUNCTION__, RobotSerial);
    char name_[64] = {0};
    int i = 0;
    int j = 0;

    // Check Connect Situation
    int ret = FX_L0_System_Testconnect();
    if (ret < 0)
    {
        _FX_ERRO("%s: Link robot failed, no response in 1000ms", __FUNCTION__);
        return FUNC_RET_LINK_NO_RESPONSE;
    }

    // Get Robot Type
    int type_ = 0;
    snprintf(name_, sizeof(name_), "R.A%d.BASIC.Type", RobotSerial);
    if (FX_L1_Param_GetInt32(name_, &type_) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
        return FUNC_RET_GET_PARAM_FAILED;
    }
    else
    {
        _FX_DEBG("%s: Get %s=%d success", __FUNCTION__, name_, type_);
    }

    // Get Robot Dof
    int dof_ = 0;
    snprintf(name_, sizeof(name_), "R.A%d.BASIC.Dof", RobotSerial);
    if (FX_L1_Param_GetInt32(name_, &dof_) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
        return FUNC_RET_GET_PARAM_FAILED;
    }
    else
    {
        _FX_DEBG("%s: Get %s=%d success", __FUNCTION__, name_, dof_);
    }

    // Get DH Parameters
    float dh_[8][4] = {{0}};
    for (i = 0; i < dof_; i++)
    {
        //// alpha
        snprintf(name_, sizeof(name_), "R.A%d.L%d.DH.Alpha", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &dh_[i][0]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }
        //// a
        snprintf(name_, sizeof(name_), "R.A%d.L%d.DH.A", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &dh_[i][1]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }
        //// d
        snprintf(name_, sizeof(name_), "R.A%d.L%d.DH.D", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &dh_[i][2]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }
        //// theta
        snprintf(name_, sizeof(name_), "R.A%d.L%d.DH.Theta", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &dh_[i][3]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }
    }
    //// Get Flange DH Parameters
    snprintf(name_, sizeof(name_), "R.A%d.FLANGE.Alpha", RobotSerial);
    if (FX_L1_Param_GetFloat(name_, &dh_[7][0]) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
        return FUNC_RET_GET_PARAM_FAILED;
    }
    ////// a
    snprintf(name_, sizeof(name_), "R.A%d.FLANGE.A", RobotSerial);
    if (FX_L1_Param_GetFloat(name_, &dh_[7][1]) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
        return FUNC_RET_GET_PARAM_FAILED;
    }
    ////// d
    snprintf(name_, sizeof(name_), "R.A%d.FLANGE.D", RobotSerial);
    if (FX_L1_Param_GetFloat(name_, &dh_[7][2]) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
        return FUNC_RET_GET_PARAM_FAILED;
    }
    ////// theta
    snprintf(name_, sizeof(name_), "R.A%d.FLANGE.Theta", RobotSerial);
    if (FX_L1_Param_GetFloat(name_, &dh_[7][3]) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
        return FUNC_RET_GET_PARAM_FAILED;
    }

    // Get PNVA
    float pnva_[8][4] = {{0}};
    for (i = 0; i < dof_; i++)
    {
        //// limit_positive
        snprintf(name_, sizeof(name_), "R.A%d.L%d.BASIC.LimitPos", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &pnva_[i][0]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        //// limit_negtive
        snprintf(name_, sizeof(name_), "R.A%d.L%d.BASIC.LimitNeg", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &pnva_[i][1]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        //// limit_velocity
        snprintf(name_, sizeof(name_), "R.A%d.L%d.BASIC.VelMax", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &pnva_[i][2]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        //// limit_accelaration
        snprintf(name_, sizeof(name_), "R.A%d.L%d.BASIC.AccMax", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &pnva_[i][3]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }
    }

    // Get Bound
    float bound_[4][3] = {{0}};
    for (i = 0; i < 3; i++)
    {
        // First Quadrant
        snprintf(name_, sizeof(name_), "R.A%d.CTRL.BD67PP%d", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &bound_[0][i]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        // Second Quadrant
        snprintf(name_, sizeof(name_), "R.A%d.CTRL.BD67NP%d", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &bound_[1][i]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        // Third Quadrant
        snprintf(name_, sizeof(name_), "R.A%d.CTRL.BD67NN%d", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &bound_[2][i]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        // Fourth Quadrant
        snprintf(name_, sizeof(name_), "R.A%d.CTRL.BD67PN%d", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &bound_[3][i]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }
    }

    // Get Dynamic Parameters
    float gravity_[3] = {0};
    float mass_[7] = {0};
    float mcp_[7][3] = {{0}};
    float inertia_[7][6] = {{0}};

    //// Gravity
    snprintf(name_, sizeof(name_), "R.A%d.BASIC.GravityX", RobotSerial);
    if (FX_L1_Param_GetFloat(name_, &gravity_[0]) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
        return FUNC_RET_GET_PARAM_FAILED;
    }

    snprintf(name_, sizeof(name_), "R.A%d.BASIC.GravityY", RobotSerial);
    if (FX_L1_Param_GetFloat(name_, &gravity_[1]) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
        return FUNC_RET_GET_PARAM_FAILED;
    }

    snprintf(name_, sizeof(name_), "R.A%d.BASIC.GravityZ", RobotSerial);
    if (FX_L1_Param_GetFloat(name_, &gravity_[2]) != 0)
    {
        _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
        return FUNC_RET_GET_PARAM_FAILED;
    }

    for (i = 0; i < dof_; i++)
    {
        //// Mass
        snprintf(name_, sizeof(name_), "R.A%d.L%d.DYNAMIC.M", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &mass_[i]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        //// MCP
        snprintf(name_, sizeof(name_), "R.A%d.L%d.DYNAMIC.MRX", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &mcp_[i][0]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        snprintf(name_, sizeof(name_), "R.A%d.L%d.DYNAMIC.MRY", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &mcp_[i][1]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        snprintf(name_, sizeof(name_), "R.A%d.L%d.DYNAMIC.MRZ", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &mcp_[i][2]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        //// Inertia
        snprintf(name_, sizeof(name_), "R.A%d.L%d.DYNAMIC.InertiaXX", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &inertia_[i][0]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        snprintf(name_, sizeof(name_), "R.A%d.L%d.DYNAMIC.InertiaXY", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &inertia_[i][1]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        snprintf(name_, sizeof(name_), "R.A%d.L%d.DYNAMIC.InertiaXZ", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &inertia_[i][2]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        snprintf(name_, sizeof(name_), "R.A%d.L%d.DYNAMIC.InertiaYY", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &inertia_[i][3]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        snprintf(name_, sizeof(name_), "R.A%d.L%d.DYNAMIC.InertiaYZ", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &inertia_[i][4]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }

        snprintf(name_, sizeof(name_), "R.A%d.L%d.DYNAMIC.InertiaZZ", RobotSerial, i);
        if (FX_L1_Param_GetFloat(name_, &inertia_[i][5]) != 0)
        {
            _FX_ERRO("%s: Failed to get %s", __FUNCTION__, name_);
            return FUNC_RET_GET_PARAM_FAILED;
        }
    }

    // float to double
    double dh_input[8][4] = {{0}};
    double pnva_input[8][4] = {{0}};
    double bound_input[4][3] = {{0}};
    double gravity_input[3] = {0};
    double mass_input[7] = {0};
    double mcp_input[7][3] = {{0}};
    double inertia_input[7][6] = {{0}};

    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 4; j++)
        {
            dh_input[i][j] = dh_[i][j];
            pnva_input[i][j] = pnva_[i][j];
        }
    }

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 3; j++)
        {
            bound_input[i][j] = bound_[i][j];
        }
    }

    for (i = 0; i < 3; i++)
    {
        gravity_input[i] = gravity_[i];
    }

    for (i = 0; i < 7; i++)
    {
        mass_input[i] = mass_[i];
        for (j = 0; j < 3; j++)
        {
            mcp_input[i][j] = mcp_[i][j];
        }
        for (j = 0; j < 6; j++)
        {
            inertia_input[i][j] = inertia_[i][j];
        }
    }
    // Initialization
    return (FX_L0_Kinematics_init_single_arm(handle, RobotSerial, &type_, dh_input, pnva_input, bound_input,
                                             gravity_input, mass_input, mcp_input, inertia_input) == FX_MOTION_OK)
               ? FUNC_RET_SUCCESS
               : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_SetTool(FX_MotionHandle handle, int robot_serial, double tool[4][4])
{
    return (FX_L0_Kinematics_set_tool(handle, robot_serial, tool) == FX_MOTION_OK) ? FUNC_RET_SUCCESS : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_RemoveTool(FX_MotionHandle handle, int robot_serial)
{
    return (FX_L0_Kinematics_remove_tool(handle, robot_serial) == FX_MOTION_OK) ? FUNC_RET_SUCCESS : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_ForwardKinematics(FX_MotionHandle handle, int robot_serial,
                                       double joints[7], double pose_matrix[4][4])
{
    return (FX_L0_Kinematics_forward_kinematics(handle, robot_serial, joints, pose_matrix) == FX_MOTION_OK) ? FUNC_RET_SUCCESS : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_Jacobian(FX_MotionHandle handle, int robot_serial,
                              double joints[7], double jacobian[6][7])
{
    return (FX_L0_Kinematics_jacobian(handle, robot_serial, joints, jacobian) == FX_MOTION_OK) ? FUNC_RET_SUCCESS : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_InverseKinematics(FX_MotionHandle handle, int robot_serial,
                                       FX_InvKineSolvePara *params)
{
    return (FX_L0_Kinematics_inverse_kinematics(handle, robot_serial, params) == FX_MOTION_OK) ? FUNC_RET_SUCCESS : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_SetSkyeBodyCondition(FX_MotionHandle handle,
                                          double std_body[3], double k_body[3],
                                          double std_arm0_len, double k_arm0,
                                          double std_arm1_len, double k_arm1)
{
    return (FX_L0_Kinematics_set_body_condition(handle, std_body, k_body,
                                                std_arm0_len, k_arm0, std_arm1_len, k_arm1) == FX_MOTION_OK)
               ? FUNC_RET_SUCCESS
               : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_SkyeBodyForwardKinematics(FX_MotionHandle handle, double jv[3],
                                               double arm0_shoulder_matrix[4][4], double arm1_shoulder_matrix[4][4])
{
    return (FX_L0_Kinematics_body_forward(handle, jv, arm0_shoulder_matrix, arm1_shoulder_matrix) == FX_MOTION_OK) ? FUNC_RET_SUCCESS : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_SkyeBodyInverseKinematics(FX_MotionHandle handle,
                                               double arm0_tcp[3], double arm1_tcp[3],
                                               double out_body_joints[3])
{
    return (FX_L0_Kinematics_calc_body_position(handle, arm0_tcp, arm1_tcp, out_body_joints) == FX_MOTION_OK) ? FUNC_RET_SUCCESS : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_SkyeBodyInverseKinematicsWithRef(FX_MotionHandle handle,
                                                      double ref_body_joints[3],
                                                      double arm0_tcp[3], double arm1_tcp[3],
                                                      double out_body_joints[3])
{
    return (FX_L0_Kinematics_calc_body_position_with_ref(handle, ref_body_joints, arm0_tcp, arm1_tcp, out_body_joints) == FX_MOTION_OK)

               ? FUNC_RET_SUCCESS
               : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_PlanJointMove(FX_MotionHandle handle, int robot_serial,
                                   double start_joints[7], double end_joints[7],
                                   double vel_ratio, double acc_ratio, int freq,
                                   double *point_set_handle, int *point_num)
{
    return (FX_L0_Kinematics_plan_joint_move(handle, robot_serial, start_joints, end_joints,
                                             vel_ratio, acc_ratio, freq, point_set_handle, point_num) == FX_MOTION_OK)
               ? FUNC_RET_SUCCESS
               : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_PlanLinearMove(FX_MotionHandle handle, int robot_serial,
                                    double start_xyzabc[6], double end_xyzabc[6],
                                    double ref_joints[7],
                                    double vel, double acc, int freq,
                                    double *point_set_handle, int *point_num)
{
    return (FX_L0_Kinematics_plan_linear_move(handle, robot_serial, start_xyzabc, end_xyzabc,
                                              ref_joints, vel, acc, freq, point_set_handle, point_num) == FX_MOTION_OK)
               ? FUNC_RET_SUCCESS
               : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_PlanLinearMoveKeepJoints(FX_MotionHandle handle, int robot_serial,
                                              double start_joints[7], double end_joints[7],
                                              double vel, double acc, int freq,
                                              double *point_set_handle, int *point_num)
{
    return (FX_L0_Kinematics_plan_linear_keep_joints(handle, robot_serial, start_joints, end_joints,
                                                     vel, acc, freq, point_set_handle, point_num) == FX_MOTION_OK)
               ? FUNC_RET_SUCCESS
               : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetStart(FX_MotionHandle handle, int robot_serial,
                                                         double ref_joints[7],
                                                         double start_xyzabc[6], double end_xyzabc[6],
                                                         double allow_range, int zsp_type,
                                                         double zsp_para[6],
                                                         double vel, double acc, int freq)
{
    return (FX_L0_Kinematics_multi_points_set_movl_start(handle, robot_serial, ref_joints,
                                                         start_xyzabc, end_xyzabc, allow_range,
                                                         zsp_type, zsp_para, vel, acc, freq) == FX_MOTION_OK)
               ? FUNC_RET_SUCCESS
               : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetNextPoints(FX_MotionHandle handle, int robot_serial,
                                                              double next_xyzabc[6],
                                                              double allow_range, int zsp_type,
                                                              double zsp_para[6],
                                                              double vel, double acc)
{
    return (FX_L0_Kinematics_multi_points_set_movl_next_points(handle, robot_serial, next_xyzabc, allow_range,
                                                               zsp_type, zsp_para, vel, acc) == FX_MOTION_OK)
               ? FUNC_RET_SUCCESS
               : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_PlanLinearMove_MultiPoints_GetPoints(FX_MotionHandle handle,
                                                          double *point_set_handle, int *point_num)
{
    return (FX_L0_Kinematics_multi_points_get_movl_path(handle, point_set_handle, point_num) == FX_MOTION_OK)
               ? FUNC_RET_SUCCESS
               : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_ArmsSynchronousPlanning(FX_MotionHandle handle,
                                             ArmsSynchronousPlanningParams *params,
                                             double *arm0_point_set, double *arm1_point_set, int *point_num)
{
    if (!handle || !params)
        return 0;
    return (FX_L0_Kinematics_plan_dual_arm_fixed_body(handle, params, arm0_point_set, arm1_point_set, point_num) == FX_MOTION_OK)
               ? FUNC_RET_SUCCESS
               : FUNC_RET_OPERATION_FAILED;
}

int FX_L1_Kinematics_DynamicsIdentification(int robot_type, char *file_path, double *mass, double mr[3], double inertia[6])
{
    return (FX_L0_Kinematics_dynamics_identification(robot_type, file_path, mass, mr, inertia) == FX_MOTION_OK)
               ? FUNC_RET_SUCCESS
               : FUNC_RET_OPERATION_FAILED;
}

void FX_L1_XYZABC2Matrix(double xyzabc[6], double matrix[4][4])
{
    FX_L0_XYZABC2Matrix(xyzabc, matrix);
}

void FX_L1_Matrix2XYZABC(double matrix[4][4], double xyzabc[6])
{
    FX_L0_Matrix2XYZABC(matrix, xyzabc);
}
