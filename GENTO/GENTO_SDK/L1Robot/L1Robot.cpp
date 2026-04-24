#include "L1Robot.h"
#include <math.h>

#define FX_COMM_MAX_TIMEOUT 50

FX_BOOL local_log_tag = FX_TRUE;

FX_INT32 FX_ARM0_DOF = 0;
FX_INT32 FX_ARM1_DOF = 0;
FX_INT32 FX_HEAD_DOF = 0;
FX_INT32 FX_BODY_DOF = 0;
FX_INT32 FX_LIFT_DOF = 0;

static const ROBOT_RT *g_robot_rt = NULL;
static const ROBOT_SG *g_robot_sg = NULL;

FX_INT32 RobotCache_Init()
{
    if (g_robot_rt == NULL)
    {
        g_robot_rt = FX_L0_GetRobotRT();
    }
    if (g_robot_sg == NULL)
    {
        g_robot_sg = FX_L0_GetRobotSG();
    }

    if (g_robot_rt == NULL || g_robot_sg == NULL)
    {
        return -1;
    }
    return 0;
}

const ROBOT_RT *RobotCache_GetRT()
{
    if (g_robot_rt == NULL)
    {
        RobotCache_Init();
    }
    return g_robot_rt;
}

const ROBOT_SG *RobotCache_GetSG()
{
    if (g_robot_sg == NULL)
    {
        RobotCache_Init();
    }
    return g_robot_sg;
}
FX_VOID RobotCache_Cleanup()
{
    g_robot_rt = NULL;
    g_robot_sg = NULL;
}

FX_INT32 FX_L1_System_GetVersion()
{
    FX_INT32 version = FX_L0_System_GetVersion();
    if (local_log_tag == FX_TRUE)
    {
        printf("[INFO] FX_L1_System_GetVersion :%d.\n", version);
    }
    return version;
}

FX_INT32 FX_L1_System_Link(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4, FX_INT32 *version, FX_UINT32 logSwitch)
{
    assert(ip1 >= 0 && ip1 <= 255);
    assert(ip2 >= 0 && ip2 <= 255);
    assert(ip3 >= 0 && ip3 <= 255);
    assert(ip4 >= 0 && ip4 <= 255);
    if (ip1 == 0 && ip2 == 0 && ip3 == 0 && ip4 == 0)
    {
        printf("[ERROR] Connect: Invalid IP address: 0.0.0.0");
        return -3;
    }
    if (ip1 == 255 && ip2 == 255 && ip3 == 255 && ip4 == 255)
    {
        printf("[ERROR] Connect: Invalid IP address: broadcast address");
        return -3;
    }
    if (ip1 == 127)
    {
        printf("[ERROR] Connect: Loopback address not allowed");
        return -3;
    }
    if (!FX_L0_System_connect(ip1, ip2, ip3, ip4))
    {
        printf("[ERROR] link robot failed, port is occupied");
        return -1;
    }

    FX_INT32 re = FX_L0_System_Testconnect();
    if (re < 0)
    {
        printf("[ERROR] robot link no response in 1000ms ---\n");
        return -2;
    }

    if (logSwitch == 0)
    {
        FX_L0_System_LocalLogOff();
        printf("[INFO] The log has been closed.\n");
    }
    if (logSwitch == 1)
    {
        FX_L0_System_LocalLogOn();
        printf("[INFO] The log has been open.\n");
    }

    if (version != NULL)
    {
        *version = FX_L1_System_GetVersion();
        printf("[INFO] System version: %d.\n", *version);
    }

    FX_CHAR dof_arm0_name[30] = {0};
    FX_CHAR dof_arm1_name[30] = {0};
    FX_CHAR dof_head_name[30] = {0};
    FX_CHAR dof_body_name[30] = {0};
    FX_CHAR dof_lift_name[30] = {0};
    sprintf(dof_arm0_name, "R.A0.BASIC.Dof");
    if (!FX_L1_Param_GetInt32(dof_arm0_name, &FX_ARM0_DOF))
    {
        printf("[WARNING] Failed to get parameter: %s\n", dof_arm0_name);
        FX_ARM0_DOF = 0;
    }
    sprintf(dof_arm1_name, "R.A1.BASIC.Dof");
    if (!FX_L1_Param_GetInt32(dof_arm1_name, &FX_ARM1_DOF))
    {
        printf("[WARNING] Failed to get parameter: %s\n", dof_arm1_name);
        FX_ARM1_DOF = 0;
    }
    sprintf(dof_body_name, "R.B.BASIC.Dof");
    if (!FX_L1_Param_GetInt32(dof_body_name, &FX_BODY_DOF))
    {
        printf("[WARNING] Failed to get parameter: %s\n", dof_body_name);
        FX_BODY_DOF = 0;
    }
    sprintf(dof_head_name, "R.H.BASIC.Dof");
    if (!FX_L1_Param_GetInt32(dof_head_name, &FX_HEAD_DOF))
    {
        printf("[WARNING] Failed to get parameter: %s\n", dof_head_name);
        FX_HEAD_DOF = 0;
    }
    sprintf(dof_lift_name, "R.L.BASIC.Dof");
    if (!FX_L1_Param_GetInt32(dof_lift_name, &FX_LIFT_DOF))
    {
        printf("[WARNING] Failed to get parameter: %s\n", dof_lift_name);
        FX_LIFT_DOF = 0;
    }
    printf("[INFO] DOF of modules: arm0=%d, arm1=%d, head=%d, body=%d, lift=%d\n", FX_ARM0_DOF, FX_ARM1_DOF, FX_HEAD_DOF, FX_BODY_DOF, FX_LIFT_DOF);
    return re;
}

FX_BOOL FX_L1_System_Reboot()
{
    FX_BOOL can_reboot = FX_TRUE;
    FX_UINT16 cur_obj_state = 0;
    if (FX_ARM0_DOF > 0)
    {
        cur_obj_state = FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState;
        if (cur_obj_state != ARM_STATE_IDLE && cur_obj_state != ARM_STATE_ERROR)
        {
            printf("[WARNING] Arm0 is not in idle or error state, reject reboot\n");
            can_reboot = FX_FALSE;
        }
    }
    if (FX_ARM1_DOF > 0)
    {
        cur_obj_state = FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState;
        if (cur_obj_state != ARM_STATE_IDLE && cur_obj_state != ARM_STATE_ERROR)
        {
            printf("[WARNING] Arm1 is not in idle or error state, reject reboot\n");
            can_reboot = FX_FALSE;
        }
    }
    if (FX_HEAD_DOF > 0)
    {
        cur_obj_state = FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState;
        if (cur_obj_state != HEAD_STATE_IDLE && cur_obj_state != HEAD_STATE_ERROR)
        {
            printf("[WARNING] Head is not in idle or error state, reject reboot\n");
            can_reboot = FX_FALSE;
        }
    }
    if (FX_BODY_DOF > 0)
    {
        cur_obj_state = FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState;
        if (cur_obj_state != BODY_STATE_IDLE && cur_obj_state != BODY_STATE_ERROR)
        {
            printf("[WARNING] Body is not in idle or error state, reject reboot\n");
            can_reboot = FX_FALSE;
        }
    }
    if (FX_LIFT_DOF > 0)
    {
        cur_obj_state = FX_L0_GetRobotRT()->m_LIFT.m_LIFT_State.m_CurState;
        if (cur_obj_state != LIFT_STATE_IDLE && cur_obj_state != LIFT_STATE_ERROR)
        {
            printf("[WARNING] Lift is not in idle or error state, reject reboot\n");
            can_reboot = FX_FALSE;
        }
    }
    if (!can_reboot)
    {
        return FX_FALSE;
    }

    if (!FX_L0_System_Reboot())
    {
        printf("[ERROR] Reboot failed\n");
        return FX_FALSE;
    }
    if (local_log_tag == FX_TRUE)
    {
        printf("[INFO] Reboot sucessed.\n");
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_SendFile(FX_CHAR *local_file_path, FX_CHAR *remote_file_path)
{
    return FX_FileClient_SendFile(local_file_path, remote_file_path);
}

FX_BOOL FX_L1_RecvFile(FX_CHAR *local_file_path, FX_CHAR *remote_file_path)
{
    return FX_FileClient_RecvFile(local_file_path, remote_file_path);
}

FX_BOOL FX_L1_Comm_Clear(FX_UINT32 timeout)
{
    return FX_L0_Communication_Clear(timeout);
}

FX_BOOL FX_L1_Comm_Send()
{
    return FX_L0_Communication_Send();
}

FX_BOOL FX_L1_Comm_SendAndWait(FX_UINT32 timeout)
{
    return ((FX_L0_Communication_SendWaitResponse(timeout) <= 0) ? FX_FALSE : FX_TRUE);
}

FX_INT32 FX_L1_Fbk_GetCtrlObjDof(FXObjType obj_type)
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

FXStateType FX_L1_Fbk_CurrentState(FXObjType obj_type)
{
    FX_INT32 cur_state = 0;
    FX_INT32 drag_type = 0;
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
            drag_type = rt->m_ARMS[0].m_ARM_IN.m_ARM_CMD_Drag_Type;
            switch (sg->m_ARMS[0].m_ARM_SET.m_ARM_SET_ImpType)
            {
            case IMP_TYPE_NULL:
                break;
            case IMP_TYPE_JOINT:
            {
                if (drag_type == DRAG_TYPE_JOINT)
                {
                    return FX_STATE_DRAG_JOINT;
                }
                else
                {
                    return FX_STATE_IMP_JOINT;
                }
            }
            case IMP_TYPE_CART:
            {
                if (drag_type == DRAG_TYPE_CART_X)
                {
                    return FX_STATE_DRAG_CART_X;
                }
                else if (drag_type == DRAG_TYPE_CART_Y)
                {
                    return FX_STATE_DRAG_CART_Y;
                }
                else if (drag_type == DRAG_TYPE_CART_Z)
                {
                    return FX_STATE_DRAG_CART_Z;
                }
                else if (drag_type == DRAG_TYPE_CART_R)
                {
                    return FX_STATE_DRAG_CART_R;
                }
                else
                {
                    return FX_STATE_IMP_CART;
                }
            }
            case IMP_TYPE_FORCE:
                return FX_STATE_IMP_FORCE;
            default:
                FX_STATE_UNKNOWN;
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
            drag_type = rt->m_ARMS[1].m_ARM_IN.m_ARM_CMD_Drag_Type;
            switch (sg->m_ARMS[1].m_ARM_SET.m_ARM_SET_ImpType)
            {
            case IMP_TYPE_NULL:
                break;
            case IMP_TYPE_JOINT:
            {
                if (drag_type == DRAG_TYPE_JOINT)
                {
                    return FX_STATE_DRAG_JOINT;
                }
                else
                {
                    return FX_STATE_IMP_JOINT;
                }
            }
            case IMP_TYPE_CART:
            {
                if (drag_type == DRAG_TYPE_CART_X)
                {
                    return FX_STATE_DRAG_CART_X;
                }
                else if (drag_type == DRAG_TYPE_CART_Y)
                {
                    return FX_STATE_DRAG_CART_Y;
                }
                else if (drag_type == DRAG_TYPE_CART_Z)
                {
                    return FX_STATE_DRAG_CART_Z;
                }
                else if (drag_type == DRAG_TYPE_CART_R)
                {
                    return FX_STATE_DRAG_CART_R;
                }
                else
                {
                    return FX_STATE_IMP_CART;
                }
            }
            case IMP_TYPE_FORCE:
                return FX_STATE_IMP_FORCE;
            default:
                FX_STATE_UNKNOWN;
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

FX_BOOL FX_L1_State_GetServoErrorCode(FXObjType obj_type, FX_INT32 error_code[7])
{
    if (!error_code)
    {
        return FX_FALSE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        for (FX_INT32 i = 0; i < FX_ARM0_DOF; i++)
        {
            if (!FX_L0_Arm0_State_GetServoErrorCode(i, &error_code[i]))
            {
                return FX_FALSE;
            }
        }
        break;
    }

    case FX_OBJ_ARM1:
    {
        for (FX_INT32 i = 0; i < FX_ARM1_DOF; i++)
        {
            if (!FX_L0_Arm1_State_GetServoErrorCode(i, &error_code[i]))
                return FX_FALSE;
        }
        break;
    }

    case FX_OBJ_HEAD:
    {
        for (FX_INT32 i = 0; i < FX_HEAD_DOF; i++)
        {
            if (!FX_L0_Head_State_GetServoErrorCode(i, &error_code[i]))
                return FX_FALSE;
        }
        break;
    }

    case FX_OBJ_BODY:
    {
        for (FX_INT32 i = 0; i < FX_BODY_DOF; i++)
        {
            if (!FX_L0_Body_State_GetServoErrorCode(i, &error_code[i]))
                return FX_FALSE;
        }
        break;
    }

    case FX_OBJ_LIFT:
    {
        for (FX_INT32 i = 0; i < FX_LIFT_DOF; i++)
        {
            if (!FX_L0_Lift_State_GetServoErrorCode(i, &error_code[i]))
                return FX_FALSE;
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }
    return FX_TRUE;
}

FX_UINT32 FX_L1_State_ResetError(FX_UINT32 obj_mask)
{
    if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
    {
        return 0;
    }

    FX_UINT32 ret_obj_mask = 0;
    if ((obj_mask & FX_OBJ_ARM0_FLAG) != 0)
    {
        if (FX_L0_Arm0_State_Reset())
        {
            ret_obj_mask |= FX_OBJ_ARM0_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_ARM1_FLAG) != 0)
    {
        if (FX_L0_Arm1_State_Reset())
        {
            ret_obj_mask |= FX_OBJ_ARM1_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_HEAD_FLAG) != 0)
    {
        if (FX_L0_Head_State_Reset())
        {
            ret_obj_mask |= FX_OBJ_HEAD_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_BODY_FLAG) != 0)
    {
        if (FX_L0_Body_State_Reset())
        {
            ret_obj_mask |= FX_OBJ_BODY_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_LIFT_FLAG) != 0)
    {
        if (FX_L0_Lift_State_Reset())
        {
            ret_obj_mask |= FX_OBJ_LIFT_FLAG;
        }
    }

    if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
    {
        return 0;
    }
    return ret_obj_mask;
}

FX_UINT32 FX_L1_State_EmergencyStop(FX_UINT32 obj_mask)
{
    if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
    {
        return 0;
    }

    FX_UINT32 ret_obj_mask = 0;
    if ((obj_mask & FX_OBJ_ARM0_FLAG) != 0)
    {
        if (FX_L0_Arm0_State_EmergencyStop())
        {
            ret_obj_mask |= FX_OBJ_ARM0_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_ARM1_FLAG) != 0)
    {
        if (FX_L0_Arm1_State_EmergencyStop())
        {
            ret_obj_mask |= FX_OBJ_ARM1_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_HEAD_FLAG) != 0)
    {
        if (FX_L0_Head_State_EmergencyStop())
        {
            ret_obj_mask |= FX_OBJ_HEAD_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_BODY_FLAG) != 0)
    {
        if (FX_L0_Body_State_EmergencyStop())
        {
            ret_obj_mask |= FX_OBJ_BODY_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_LIFT_FLAG) != 0)
    {
        if (FX_L0_Lift_State_EmergencyStop())
        {
            ret_obj_mask |= FX_OBJ_LIFT_FLAG;
        }
    }

    if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
    {
        return 0;
    }
    return ret_obj_mask;
}

FX_INT32 FX_L1_State_SwitchToIdle(FXObjType obj_type, FX_UINT32 timeout)
{
    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_IDLE:
    {
        return 0;
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (!FX_L0_Arm0_Runtime_SetState(ARM_STATE_IDLE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (!FX_L0_Arm1_Runtime_SetState(ARM_STATE_IDLE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        {
            if (!FX_L0_Head_Runtime_SetState(HEAD_STATE_IDLE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_BODY:
        {
            if (!FX_L0_Body_Runtime_SetState(BODY_STATE_IDLE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_LIFT:
        {
            if (!FX_L0_Lift_Runtime_SetState(LIFT_STATE_IDLE))
            {
                return -3;
            }
            break;
        }
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            return 0;
        }
        while (timeout > 0)
        {
            CFXULT::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_IDLE)
            {
                return 0;
            }
            timeout--;
        }
        return -5;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        return -6;
    }
    }
}

FX_INT32 FX_L1_State_SwitchToPositionMode(FXObjType obj_type, FX_UINT32 timeout, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio)
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

    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_POSITION:
    {
        // send cmd
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (!FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (!FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        {
            if (!FX_L0_Head_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Head_Runtime_SetAccRatio(acc_ratio))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_BODY:
        {
            if (!FX_L0_Body_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Body_Runtime_SetAccRatio(acc_ratio))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_LIFT:
        {
            if (!FX_L0_Lift_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Lift_Runtime_SetAccRatio(acc_ratio))
            {
                return -3;
            }
            break;
        }
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        return 0;
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Arm0_Runtime_SetState(ARM_STATE_POSITION))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Arm1_Runtime_SetState(ARM_STATE_POSITION))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        {
            if (!FX_L0_Head_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Head_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Head_Runtime_SetState(HEAD_STATE_POSITION))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_BODY:
        {
            if (!FX_L0_Body_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Body_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Body_Runtime_SetState(BODY_STATE_POSITION))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_LIFT:
        {
            if (!FX_L0_Lift_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Lift_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Lift_Runtime_SetState(LIFT_STATE_POSITION))
            {
                return -3;
            }
            break;
        }
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            return 0;
        }
        while (timeout > 0)
        {
            CFXULT::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_POSITION)
            {
                CFXULT::UniMilliSleep(250); // FIXME
                return 0;
            }
            timeout--;
        }
        return -5;
    }
    case FX_STATE_ERROR:
    case FX_STATE_RELEASE:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        return -6;
    }
    }
}

FX_INT32 FX_L1_State_SwitchToImpJointMode(FXObjType obj_type, FX_UINT32 timeout, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio, FX_DOUBLE k[7], FX_DOUBLE d[7])
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

    for (FX_INT32 i = 0; i < 7; i++)
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (!FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Arm0_Runtime_SetJointK(k) || !FX_L0_Arm0_Runtime_SetJointD(d))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (!FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Arm1_Runtime_SetJointK(k) || !FX_L0_Arm1_Runtime_SetJointD(d))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_BODY:
        {
            if (!FX_L0_Body_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Body_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Body_Runtime_SetPDP(k) || !FX_L0_Body_Runtime_SetPDD(d))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        return 0;
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Arm0_Runtime_SetJointK(k) || !FX_L0_Arm0_Runtime_SetJointD(d) || !FX_L0_Arm0_Runtime_SetDragType(DRAG_TYPE_NULL) || !FX_L0_Arm0_Runtime_SetImpType(IMP_TYPE_JOINT) || !FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Arm1_Runtime_SetJointK(k) || !FX_L0_Arm1_Runtime_SetJointD(d) || !FX_L0_Arm1_Runtime_SetDragType(DRAG_TYPE_NULL) || !FX_L0_Arm1_Runtime_SetImpType(IMP_TYPE_JOINT) || !FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_BODY:
        {
            if (!FX_L0_Body_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Body_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Body_Runtime_SetPDP(k) || !FX_L0_Body_Runtime_SetPDD(d) || !FX_L0_Arm1_Runtime_SetState(BODY_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            return 0;
        }
        while (timeout > 0)
        {
            CFXULT::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_IMP_JOINT)
            {
                CFXULT::UniMilliSleep(100); // FIXME
                return 0;
            }
            timeout--;
        }
        return -5;
    }
    case FX_STATE_ERROR:
    case FX_STATE_RELEASE:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        return -6;
    }
    }
}

FX_INT32 FX_L1_State_SwitchToImpCartMode(FXObjType obj_type, FX_UINT32 timeout, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio, FX_DOUBLE k[7], FX_DOUBLE d[7])
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

    for (FX_INT32 i = 0; i < 7; i++)
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (!FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Arm0_Runtime_SetCartK(k) || !FX_L0_Arm0_Runtime_SetCartD(d))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (!FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Arm1_Runtime_SetCartK(k) || !FX_L0_Arm1_Runtime_SetCartD(d))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        return 0;
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Arm0_Runtime_SetCartK(k) || !FX_L0_Arm0_Runtime_SetCartD(d) || !FX_L0_Arm0_Runtime_SetDragType(DRAG_TYPE_NULL) || !FX_L0_Arm0_Runtime_SetImpType(IMP_TYPE_CART) || !FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio) || !FX_L0_Arm1_Runtime_SetCartK(k) || !FX_L0_Arm1_Runtime_SetCartD(d) || !FX_L0_Arm1_Runtime_SetDragType(DRAG_TYPE_NULL) || !FX_L0_Arm1_Runtime_SetImpType(IMP_TYPE_CART) || !FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            return 0;
        }
        while (timeout > 0)
        {
            CFXULT::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_IMP_CART)
            {
                return 0;
            }
            timeout--;
        }
        return -5;
    }
    case FX_STATE_ERROR:
    case FX_STATE_RELEASE:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        return -6;
    }
    }
}

FX_INT32 FX_L1_State_SwitchToImpForceMode(FXObjType obj_type, FX_UINT32 timeout, FX_DOUBLE force_ctrl[FX_FORCE_DEF_NUM], FX_DOUBLE torque_ctrl[FX_TORQUE_DEF_NUM])
{
    FX_DOUBLE force_dir_vector_len = force_ctrl[FX_FORCE_DIR_X] * force_ctrl[FX_FORCE_DIR_X] + force_ctrl[FX_FORCE_DIR_Y] * force_ctrl[FX_FORCE_DIR_Y] + force_ctrl[FX_FORCE_DIR_Z] * force_ctrl[FX_FORCE_DIR_Z];
    if (force_dir_vector_len < 0.1)
    {
        return -8;
    }
    if (force_ctrl[FX_FORCE_VALUE] < 0)
    {
        force_ctrl[FX_FORCE_VALUE] = 0;
    }
    if (force_ctrl[FX_FORCE_DISTANCE < 0])
    {
        force_ctrl[FX_FORCE_DISTANCE] = 0;
    }

    FX_DOUBLE torque_dir_vector_len = torque_ctrl[FX_FORCE_DIR_X] * torque_ctrl[FX_FORCE_DIR_X] + torque_ctrl[FX_FORCE_DIR_Y] * torque_ctrl[FX_FORCE_DIR_Y] + torque_ctrl[FX_FORCE_DIR_Z] * torque_ctrl[FX_FORCE_DIR_Z];
    if (torque_dir_vector_len < 0.1)
    {
        return -8;
    }
    if (torque_ctrl[FX_TORQUE_VALUE] < 0)
    {
        torque_ctrl[FX_TORQUE_VALUE] = 0;
    }
    if (torque_ctrl[FX_TORQUE_ANGLE < 0])
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            if (!FX_L0_Arm0_Runtime_SetForceCtrl(force_ctrl) || !FX_L0_Arm0_Runtime_SetTorqueCtrl(torque_ctrl))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            if (!FX_L0_Arm1_Runtime_SetForceCtrl(force_ctrl) || !FX_L0_Arm1_Runtime_SetTorqueCtrl(torque_ctrl))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        return 0;
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm0_Runtime_SetForceCtrl(force_ctrl) || !FX_L0_Arm0_Runtime_SetTorqueCtrl(torque_ctrl) || !FX_L0_Arm0_Runtime_SetImpType(IMP_TYPE_FORCE) || !FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm1_Runtime_SetForceCtrl(force_ctrl) || !FX_L0_Arm1_Runtime_SetTorqueCtrl(torque_ctrl) || !FX_L0_Arm1_Runtime_SetImpType(IMP_TYPE_FORCE) || !FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            return 0;
        }
        while (timeout > 0)
        {
            CFXULT::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_IMP_FORCE)
            {
                return 0;
            }
            timeout--;
        }
        return -5;
    }
    case FX_STATE_ERROR:
    case FX_STATE_RELEASE:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        return -6;
    }
    }
}

FX_INT32 FX_L1_State_SwitchToDragJoint(FXObjType obj_type, FX_UINT32 timeout)
{
    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_DRAG_JOINT:
    {
        return 0;
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm0_Runtime_SetDragType(DRAG_TYPE_JOINT) || !FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm1_Runtime_SetDragType(DRAG_TYPE_JOINT) || !FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            return 0;
        }
        while (timeout > 0)
        {
            CFXULT::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_DRAG_JOINT)
            {
                return 0;
            }
            timeout--;
        }
        return -5;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        return -6;
    }
    }
}

FX_INT32 FX_L1_State_SwitchToDragCartX(FXObjType obj_type, FX_UINT32 timeout)
{
    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_DRAG_CART_X:
    {
        return 0;
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm0_Runtime_SetDragType(DRAG_TYPE_CART_X) || !FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm1_Runtime_SetDragType(DRAG_TYPE_CART_X) || !FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            return 0;
        }
        while (timeout > 0)
        {
            CFXULT::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_DRAG_CART_X)
            {
                return 0;
            }
            timeout--;
        }
        return -5;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        return -6;
    }
    }
}

FX_INT32 FX_L1_State_SwitchToDragCartY(FXObjType obj_type, FX_UINT32 timeout)
{
    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_DRAG_CART_Y:
    {
        return 0;
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm0_Runtime_SetDragType(DRAG_TYPE_CART_Y) || !FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm1_Runtime_SetDragType(DRAG_TYPE_CART_Y) || !FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            return 0;
        }
        while (timeout > 0)
        {
            CFXULT::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_DRAG_CART_Y)
            {
                return 0;
            }
            timeout--;
        }
        return -5;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        return -6;
    }
    }
}

FX_INT32 FX_L1_State_SwitchToDragCartZ(FXObjType obj_type, FX_UINT32 timeout)
{
    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_DRAG_CART_Z:
    {
        return 0;
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm0_Runtime_SetDragType(DRAG_TYPE_CART_Z) || !FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm1_Runtime_SetDragType(DRAG_TYPE_CART_Z) || !FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            return 0;
        }
        while (timeout > 0)
        {
            CFXULT::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_DRAG_CART_Z)
            {
                return 0;
            }
            timeout--;
        }
        return -5;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        return -6;
    }
    }
}

FX_INT32 FX_L1_State_SwitchToDragCartR(FXObjType obj_type, FX_UINT32 timeout)
{
    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_DRAG_CART_R:
    {
        return 0;
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
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm0_Runtime_SetDragType(DRAG_TYPE_CART_R) || !FX_L0_Arm0_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm1_Runtime_SetDragType(DRAG_TYPE_CART_R) || !FX_L0_Arm1_Runtime_SetState(ARM_STATE_TORQUE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            return 0;
        }
        while (timeout > 0)
        {
            CFXULT::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_DRAG_CART_R)
            {
                return 0;
            }
            timeout--;
        }
        return -5;
    }
    case FX_STATE_ERROR:
    case FX_STATE_TRANSFERRING:
    case FX_STATE_UNKNOWN:
    default:
    {
        return -6;
    }
    }
}

FX_INT32 FX_L1_State_SwitchToCollaborativeRelease(FXObjType obj_type, FX_UINT32 timeout)
{
    FXStateType cur_state = FX_STATE_UNKNOWN;
    cur_state = FX_L1_Fbk_CurrentState(obj_type);
    switch (cur_state)
    {
    case FX_STATE_RELEASE:
    {
        return 0;
    }
    case FX_STATE_IDLE:
    {
        // send cmd
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return -1;
        }
        switch (obj_type)
        {
        case FX_OBJ_ARM0:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[0].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm0_Runtime_SetState(ARM_STATE_RELEASE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_ARM1:
        {
            const ROBOT_SG *sg = FX_L0_GetRobotSG();
            if (sg->m_ARMS[1].m_ARM_GET.m_ARM_GET_LowSpdFlag != 1)
            {
                return -7;
            }
            if (!FX_L0_Arm1_Runtime_SetState(ARM_STATE_RELEASE))
            {
                return -3;
            }
            break;
        }
        case FX_OBJ_HEAD:
        case FX_OBJ_BODY:
        case FX_OBJ_LIFT:
        default:
        {
            return -4;
        }
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return -2;
        }
        // wait response
        if (timeout == 0) // non-block
        {
            return 0;
        }
        while (timeout > 0)
        {
            CFXULT::UniMilliSleep(1);
            if (FX_L1_Fbk_CurrentState(obj_type) == FX_STATE_RELEASE)
            {
                return 0;
            }
            timeout--;
        }
        return -5;
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
        return -6;
    }
    }
}

FX_BOOL FX_L1_Param_SetInt32(FX_CHAR *name, FX_INT32 value)
{
    if (name == NULL || strlen(name) > 29)
    {
        return FX_FALSE;
    }
    if (!FX_L0_Param_SetInt(name, value))
    {
        return FX_FALSE;
    }
    return FX_L0_Param_Save();
}

FX_BOOL FX_L1_Param_SetFloat(FX_CHAR *name, FX_FLOAT value)
{
    if (name == NULL || strlen(name) > 29)
    {
        return FX_FALSE;
    }
    if (!FX_L0_Param_SetFloat(name, value))
    {
        return FX_FALSE;
    }
    return FX_L0_Param_Save();
}

FX_BOOL FX_L1_Param_GetInt32(FX_CHAR *name, FX_INT32 *value)
{
    if (name == NULL || strlen(name) > 29)
    {
        return FX_FALSE;
    }
    return FX_L0_Param_GetInt(name, value);
}

FX_BOOL FX_L1_Param_GetFloat(FX_CHAR *name, FX_FLOAT *value)
{
    if (name == NULL || strlen(name) > 29)
    {
        return FX_FALSE;
    }
    return FX_L0_Param_GetFloat(name, value);
}

FX_BOOL FX_L1_Terminal_ClearData(FXTerminalType terminal_type)
{
    switch (terminal_type)
    {
    case FX_TERMINAL_ARM0:
        return FX_L0_Arm0_Terminal_ClearData();
    case FX_TERMINAL_ARM1:
        return FX_L0_Arm1_Terminal_ClearData();
    default:
        return FX_FALSE;
    }
}

FX_INT32 FX_L1_Terminal_GetData(FXTerminalType terminal_type, FXChnType *chn_type, FX_UCHAR data[64])
{
    switch (terminal_type)
    {
    case FX_TERMINAL_ARM0:
        return FX_L0_Arm0_Terminal_GetData((FX_INT32 *)chn_type, data);
    case FX_TERMINAL_ARM1:
        return FX_L0_Arm1_Terminal_GetData((FX_INT32 *)chn_type, data);
    default:
        return -1;
    }
}

FX_BOOL FX_L1_Terminal_SetData(FXTerminalType terminal_type, FXChnType chn_type, FX_UCHAR data[64], FX_UINT32 data_len)
{
    switch (terminal_type)
    {
    case FX_TERMINAL_ARM0:
        return FX_L0_Arm0_Terminal_SetData(chn_type, data, data_len);
    case FX_TERMINAL_ARM1:
        return FX_L0_Arm1_Terminal_SetData(chn_type, data, data_len);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Config_SetBrakeLock(FXObjType obj_type, FX_UINT8 axis_mask)
{
    if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
    {
        return FX_FALSE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm0_Config_SetBrakeLock(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm1_Config_SetBrakeLock(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState != HEAD_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Head_Config_SetBrakeLock(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Body_Config_SetBrakeLock(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }

    if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
    {
        return FX_FALSE;
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_Config_SetBrakeUnlock(FXObjType obj_type, FX_UINT8 axis_mask)
{
    if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
    {
        return FX_FALSE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm0_Config_SetBrakeUnlock(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm1_Config_SetBrakeUnlock(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState != HEAD_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Head_Config_SetBrakeUnlock(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Body_Config_SetBrakeUnlock(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }

    if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
    {
        return FX_FALSE;
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_Config_ResetEncOffset(FXObjType obj_type, FX_UINT8 axis_mask)
{
    if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
    {
        return FX_FALSE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm0_Config_ResetEncOffset(axis_mask) || !FX_L0_Arm0_Config_ResetExtEncOffset(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm1_Config_ResetEncOffset(axis_mask) || !FX_L0_Arm1_Config_ResetExtEncOffset(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState != HEAD_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Head_Config_ResetEncOffset(axis_mask) || !FX_L0_Head_Config_ResetExtEncOffset(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Body_Config_ResetEncOffset(axis_mask) || !FX_L0_Body_Config_ResetExtEncOffset(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_LIFT:
    {
        if (FX_L0_GetRobotRT()->m_LIFT.m_LIFT_State.m_CurState != LIFT_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Lift_Config_ResetEncOffset(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }

    if (FX_L0_Communication_SendWaitResponse(5000) <= 0)
    {
        return FX_FALSE;
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_Config_ClearEncError(FXObjType obj_type, FX_UINT8 axis_mask)
{
    if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
    {
        return FX_FALSE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm0_Config_ClearEncError(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm1_Config_ClearEncError(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState != HEAD_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Head_Config_ClearEncError(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Body_Config_ClearEncError(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }

    if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
    {
        return FX_FALSE;
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_Config_ClearAxisSensorOffset(FXObjType obj_type, FX_UINT32 axis_id)
{
    FX_INT32 offset = 0;
    // compute offset
    FX_CHAR SensorK_name[30] = {0};
    FX_CHAR SensorDir_name[30] = {0};
    FX_FLOAT sensor_value = 0;
    const ROBOT_RT *rt = FX_L0_GetRobotRT();
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (axis_id >= FX_ARM0_DOF)
        {
            return FX_FALSE;
        }
        sprintf(SensorK_name, "R.A0.L%d.BASIC.SensorK", axis_id);
        sprintf(SensorDir_name, "R.A0.L%d.BASIC.SensorDir", axis_id);
        sensor_value = rt->m_ARMS[0].m_ARM_OUT.m_ARM_FB_Joint_SToq[axis_id];
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (axis_id >= FX_ARM1_DOF)
        {
            return FX_FALSE;
        }
        sprintf(SensorK_name, "R.A1.L%d.BASIC.SensorK", axis_id);
        sprintf(SensorDir_name, "R.A1.L%d.BASIC.SensorDir", axis_id);
        sensor_value = rt->m_ARMS[1].m_ARM_OUT.m_ARM_FB_Joint_SToq[axis_id];
        break;
    }
    case FX_OBJ_BODY:
    {
        if (axis_id >= FX_BODY_DOF)
        {
            return FX_FALSE;
        }
        sprintf(SensorK_name, "R.B.L%d.BASIC.SensorK", axis_id);
        sprintf(SensorDir_name, "R.B.L%d.BASIC.SensorDir", axis_id);
        sensor_value = rt->m_BODY.m_BODY_Out.m_BODY_FB_STorq[axis_id];
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }
    FX_FLOAT sensor_k = 0;
    FX_INT32 sensor_dir = 0;
    if (!FX_L0_Param_GetFloat(SensorK_name, &sensor_k) || !FX_L0_Param_GetInt(SensorDir_name, &sensor_dir))
    {
        return FX_FALSE;
    }
    if (sensor_k < 0.00001)
    {
        return FX_FALSE;
    }
    if (sensor_dir == 0)
    {
        offset = sensor_value / sensor_k;
    }
    else
    {
        offset = -sensor_value / sensor_k;
    }

    // set offset
    if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
    {
        return FX_FALSE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm0_Config_SetSensorOffset(axis_id, offset))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm1_Config_SetSensorOffset(axis_id, offset))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Body_Config_SetSensorOffset(axis_id, offset))
        {
            return FX_FALSE;
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }

    if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
    {
        return FX_FALSE;
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_Config_ClearSensorOffset(FXObjType obj_type)
{
    FX_INT32 offset[8] = {0};
    // compute offset
    FX_CHAR SensorK_name[8][30] = {{0}};
    FX_CHAR SensorDir_name[8][30] = {{0}};
    FX_FLOAT sensor_value[8] = {0};
    FX_INT32 dof = 0;
    const ROBOT_RT *rt = FX_L0_GetRobotRT();
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        dof = FX_ARM0_DOF;
        for (FX_INT32 i = 0; i < dof; i++)
        {
            sprintf(SensorK_name[i], "R.A0.L%d.BASIC.SensorK", i);
            sprintf(SensorDir_name[i], "R.A0.L%d.BASIC.SensorDir", i);
            sensor_value[i] = rt->m_ARMS[0].m_ARM_OUT.m_ARM_FB_Joint_SToq[i];
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        dof = FX_ARM1_DOF;
        for (FX_INT32 i = 0; i < dof; i++)
        {
            sprintf(SensorK_name[i], "R.A1.L%d.BASIC.SensorK", i);
            sprintf(SensorDir_name[i], "R.A1.L%d.BASIC.SensorDir", i);
            sensor_value[i] = rt->m_ARMS[1].m_ARM_OUT.m_ARM_FB_Joint_SToq[i];
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        dof = FX_BODY_DOF;
        for (FX_INT32 i = 0; i < dof; i++)
        {
            sprintf(SensorK_name[i], "R.B.L%d.BASIC.SensorK", i);
            sprintf(SensorDir_name[i], "R.B.L%d.BASIC.SensorDir", i);
            sensor_value[i] = rt->m_BODY.m_BODY_Out.m_BODY_FB_STorq[i];
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }
    FX_FLOAT sensor_k[8] = {0};
    FX_INT32 sensor_dir[8] = {0};
    for (FX_INT32 i = 0; i < dof; i++)
    {
        if (!FX_L0_Param_GetFloat(SensorK_name[i], &sensor_k[i]) || !FX_L0_Param_GetInt(SensorDir_name[i], &sensor_dir[i]))
        {
            return FX_FALSE;
        }
        if (sensor_k[i] < 0.00001)
        {
            return FX_FALSE;
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
    if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
    {
        return FX_FALSE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        for (FX_INT32 i = 0; i < FX_ARM0_DOF; i++)
        {
            if (!FX_L0_Arm0_Config_SetSensorOffset(i, offset[i]))
            {
                return FX_FALSE;
            }
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        for (FX_INT32 i = 0; i < FX_ARM1_DOF; i++)
        {
            if (!FX_L0_Arm1_Config_SetSensorOffset(i, offset[i]))
            {
                return FX_FALSE;
            }
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            return FX_FALSE;
        }
        for (FX_INT32 i = 0; i < FX_BODY_DOF; i++)
        {
            if (!FX_L0_Body_Config_SetSensorOffset(i, offset[i]))
            {
                return FX_FALSE;
            }
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }

    if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
    {
        return FX_FALSE;
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_Config_DisableSoftLimit(FXObjType obj_type, FX_UINT8 axis_mask)
{
    if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
    {
        return FX_FALSE;
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm0_Config_DisableSoftLimit(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (FX_L0_GetRobotRT()->m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm1_Config_DisableSoftLimit(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (FX_L0_GetRobotRT()->m_HEAD.m_HEAD_State.m_CurState != HEAD_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Head_Config_DisableSoftLimit(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (FX_L0_GetRobotRT()->m_BODY.m_BODY_State.m_CurState != BODY_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Body_Config_DisableSoftLimit(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_LIFT:
    {
        if (FX_L0_GetRobotRT()->m_LIFT.m_LIFT_State.m_CurState != LIFT_STATE_IDLE)
        {
            return FX_FALSE;
        }
        if (!FX_L0_Lift_Config_DisableSoftLimit(axis_mask))
        {
            return FX_FALSE;
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }

    if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
    {
        return FX_FALSE;
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_Config_SetTraj(FXObjType obj_type, FX_UINT32 point_num, FX_DOUBLE *point_data)
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (RobotCtrl::GetIns()->m_RobotRT.m_ARMS[0].m_ARM_State.m_CurState != ARM_STATE_POSITION)
        {
            return FX_FALSE;
        }
        if (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[0].m_ARM_GET.m_ARM_GET_TrajState >= 3)
        {
            return FX_FALSE;
        }
        // InitTraj
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm0_Runtime_InitTraj(point_num))
        {
            return FX_FALSE;
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return FX_FALSE;
        }
        FX_INT32 wait_count = 10;
        do
        {
            CFXULT::UniMilliSleep(1);
            wait_count--;
            if (wait_count == 0)
            {
                return FX_FALSE;
            }
        } while (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[0].m_ARM_GET.m_ARM_GET_TrajState != 1);
        // SetTraj
        FX_UINT32 full_frame_num = point_num / 50;
        FX_UINT32 relic_point_num = point_num % 50;
        for (FX_INT32 i = 0; i < full_frame_num; i++)
        {
            if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
            {
                return FX_FALSE;
            }
            if (!FX_L0_Arm0_Runtime_SetTraj(i, 50, &point_data[350 * i]))
            {
                return FX_FALSE;
            }
            if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
            {
                return FX_FALSE;
            }
        }
        if (relic_point_num != 0)
        {
            if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
            {
                return FX_FALSE;
            }
            if (!FX_L0_Arm0_Runtime_SetTraj(full_frame_num, relic_point_num, &point_data[350 * full_frame_num]))
            {
                return FX_FALSE;
            }
            if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
            {
                return FX_FALSE;
            }
        }
        wait_count = 10;
        do
        {
            CFXULT::UniMilliSleep(1);
            wait_count--;
            if (wait_count == 0)
            {
                return FX_FALSE;
            }
        } while (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[0].m_ARM_GET.m_ARM_GET_TrajState != 2);
        return FX_TRUE;
    }
    case FX_OBJ_ARM1:
    {
        if (RobotCtrl::GetIns()->m_RobotRT.m_ARMS[1].m_ARM_State.m_CurState != ARM_STATE_POSITION)
        {
            return FX_FALSE;
        }
        if (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[1].m_ARM_GET.m_ARM_GET_TrajState >= 3)
        {
            return FX_FALSE;
        }
        // InitTraj
        if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
        {
            return FX_FALSE;
        }
        if (!FX_L0_Arm1_Runtime_InitTraj(point_num))
        {
            return FX_FALSE;
        }
        if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
        {
            return FX_FALSE;
        }
        FX_INT32 wait_count = 10;
        do
        {
            CFXULT::UniMilliSleep(1);
            wait_count--;
            if (wait_count == 0)
            {
                return FX_FALSE;
            }
        } while (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[1].m_ARM_GET.m_ARM_GET_TrajState != 1);
        // SetTraj
        FX_UINT32 full_frame_num = point_num / 50;
        FX_UINT32 relic_point_num = point_num % 50;
        for (FX_INT32 i = 0; i < full_frame_num; i++)
        {
            if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
            {
                return FX_FALSE;
            }
            if (!FX_L0_Arm1_Runtime_SetTraj(i, 50, &point_data[350 * i]))
            {
                return FX_FALSE;
            }
            if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
            {
                return FX_FALSE;
            }
        }
        if (relic_point_num != 0)
        {
            if (!FX_L0_Communication_Clear(FX_COMM_MAX_TIMEOUT))
            {
                return FX_FALSE;
            }
            if (!FX_L0_Arm1_Runtime_SetTraj(full_frame_num, relic_point_num, &point_data[350 * full_frame_num]))
            {
                return FX_FALSE;
            }
            if (FX_L0_Communication_SendWaitResponse(FX_COMM_MAX_TIMEOUT) <= 0)
            {
                return FX_FALSE;
            }
        }
        wait_count = 10;
        do
        {
            CFXULT::UniMilliSleep(1);
            wait_count--;
            if (wait_count == 0)
            {
                return FX_FALSE;
            }
        } while (RobotCtrl::GetIns()->m_RobotSG.m_ARMS[1].m_ARM_GET.m_ARM_GET_TrajState != 2);
        return FX_TRUE;
    }
    default:
    {
        return FX_FALSE;
    }
    }
}

FX_BOOL FX_L1_Runtime_SetJointPosCmd(FXObjType obj_type, FX_DOUBLE pos_cmd[7])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
        return FX_L0_Arm0_Runtime_SetJointPosCmd(pos_cmd);
    case FX_OBJ_ARM1:
        return FX_L0_Arm1_Runtime_SetJointPosCmd(pos_cmd);
    case FX_OBJ_HEAD:
        return FX_L0_Head_Runtime_SetJointPosCmd(pos_cmd);
    case FX_OBJ_BODY:
        return FX_L0_Body_Runtime_SetJointPosCmd(pos_cmd);
    case FX_OBJ_LIFT:
        return FX_L0_Lift_Runtime_SetJointPosCmd(pos_cmd);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Runtime_SetForceCtrl(FXObjType obj_type, FX_DOUBLE force_ctrl[FX_FORCE_DEF_NUM])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
        return FX_L0_Arm0_Runtime_SetForceCtrl(force_ctrl);
    case FX_OBJ_ARM1:
        return FX_L0_Arm1_Runtime_SetForceCtrl(force_ctrl);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Runtime_SetTorqueCtrl(FXObjType obj_type, FX_DOUBLE torque_ctrl[FX_TORQUE_DEF_NUM])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
        return FX_L0_Arm0_Runtime_SetTorqueCtrl(torque_ctrl);
    case FX_OBJ_ARM1:
        return FX_L0_Arm1_Runtime_SetTorqueCtrl(torque_ctrl);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Runtime_SetVelRatio(FXObjType obj_type, FX_DOUBLE vel_ratio)
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
        return FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio);
    case FX_OBJ_ARM1:
        return FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio);
    case FX_OBJ_HEAD:
        return FX_L0_Head_Runtime_SetVelRatio(vel_ratio);
    case FX_OBJ_BODY:
        return FX_L0_Body_Runtime_SetVelRatio(vel_ratio);
    case FX_OBJ_LIFT:
        return FX_L0_Lift_Runtime_SetVelRatio(vel_ratio);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Runtime_SetAccRatio(FXObjType obj_type, FX_DOUBLE acc_ratio)
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
        return FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio);
    case FX_OBJ_ARM1:
        return FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio);
    case FX_OBJ_HEAD:
        return FX_L0_Head_Runtime_SetAccRatio(acc_ratio);
    case FX_OBJ_BODY:
        return FX_L0_Body_Runtime_SetAccRatio(acc_ratio);
    case FX_OBJ_LIFT:
        return FX_L0_Lift_Runtime_SetAccRatio(acc_ratio);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Runtime_SetSpeedRatio(FXObjType obj_type, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio)
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
        if (!FX_L0_Arm0_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm0_Runtime_SetAccRatio(acc_ratio))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (!FX_L0_Arm1_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Arm1_Runtime_SetAccRatio(acc_ratio))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_HEAD:
    {
        if (!FX_L0_Head_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Head_Runtime_SetAccRatio(acc_ratio))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_BODY:
    {
        if (!FX_L0_Body_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Body_Runtime_SetAccRatio(acc_ratio))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_LIFT:
    {
        if (!FX_L0_Lift_Runtime_SetVelRatio(vel_ratio) || !FX_L0_Lift_Runtime_SetAccRatio(acc_ratio))
        {
            return FX_FALSE;
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_Runtime_SetJointK(FXObjType obj_type, FX_DOUBLE k[7])
{
    for (FX_INT32 i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
        return FX_L0_Arm0_Runtime_SetJointK(k);
    case FX_OBJ_ARM1:
        return FX_L0_Arm1_Runtime_SetJointK(k);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Runtime_SetJointD(FXObjType obj_type, FX_DOUBLE d[7])
{
    for (FX_INT32 i = 0; i < 7; i++)
    {
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
        return FX_L0_Arm0_Runtime_SetJointD(d);
    case FX_OBJ_ARM1:
        return FX_L0_Arm1_Runtime_SetJointD(d);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Runtime_SetJointKD(FXObjType obj_type, FX_DOUBLE k[7], FX_DOUBLE d[7])
{
    for (FX_INT32 i = 0; i < 7; i++)
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
        if (!FX_L0_Arm0_Runtime_SetJointK(k) || !FX_L0_Arm0_Runtime_SetJointD(d))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (!FX_L0_Arm1_Runtime_SetJointK(k) || !FX_L0_Arm1_Runtime_SetJointD(d))
        {
            return FX_FALSE;
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_Runtime_SetCartK(FXObjType obj_type, FX_DOUBLE k[7])
{
    for (FX_INT32 i = 0; i < 7; i++)
    {
        if (k[i] < 0)
        {
            k[i] = 0;
        }
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
        return FX_L0_Arm0_Runtime_SetCartK(k);
    case FX_OBJ_ARM1:
        return FX_L0_Arm1_Runtime_SetCartK(k);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Runtime_SetCartD(FXObjType obj_type, FX_DOUBLE d[7])
{
    for (FX_INT32 i = 0; i < 7; i++)
    {
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    switch (obj_type)
    {
    case FX_OBJ_ARM0:
        return FX_L0_Arm0_Runtime_SetCartD(d);
    case FX_OBJ_ARM1:
        return FX_L0_Arm1_Runtime_SetCartD(d);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Runtime_SetCartKD(FXObjType obj_type, FX_DOUBLE k[7], FX_DOUBLE d[7])
{
    for (FX_INT32 i = 0; i < 7; i++)
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
        if (!FX_L0_Arm0_Runtime_SetCartK(k) || !FX_L0_Arm0_Runtime_SetCartD(d))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (!FX_L0_Arm1_Runtime_SetCartK(k) || !FX_L0_Arm1_Runtime_SetCartD(d))
        {
            return FX_FALSE;
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_Runtime_SetToolK(FXObjType obj_type, FX_DOUBLE k[6])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
        return FX_L0_Arm0_Runtime_SetToolK(k);
    case FX_OBJ_ARM1:
        return FX_L0_Arm1_Runtime_SetToolK(k);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Runtime_SetToolD(FXObjType obj_type, FX_DOUBLE d[10])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
        return FX_L0_Arm0_Runtime_SetToolD(d);
    case FX_OBJ_ARM1:
        return FX_L0_Arm1_Runtime_SetToolD(d);
    default:
        return FX_FALSE;
    }
}

FX_BOOL FX_L1_Runtime_SetToolKD(FXObjType obj_type, FX_DOUBLE k[6], FX_DOUBLE d[10])
{
    switch (obj_type)
    {
    case FX_OBJ_ARM0:
    {
        if (!FX_L0_Arm0_Runtime_SetToolK(k) || !FX_L0_Arm0_Runtime_SetToolD(d))
        {
            return FX_FALSE;
        }
        break;
    }
    case FX_OBJ_ARM1:
    {
        if (!FX_L0_Arm1_Runtime_SetToolK(k) || !FX_L0_Arm1_Runtime_SetToolD(d))
        {
            return FX_FALSE;
        }
        break;
    }
    default:
    {
        return FX_FALSE;
    }
    }
    return FX_TRUE;
}

FX_BOOL FX_L1_Runtime_SetBodyPDP(FX_DOUBLE p[6])
{
    for (FX_INT32 i = 0; i < 6; i++)
    {
        if (p[i] < 0)
        {
            p[i] = 0;
        }
    }

    return FX_L0_Body_Runtime_SetPDP(p);
}

FX_BOOL FX_L1_Runtime_SetBodyPDD(FX_DOUBLE d[6])
{
    for (FX_INT32 i = 0; i < 6; i++)
    {
        if (d[i] < 0)
        {
            d[i] = 0;
        }
    }

    return FX_L0_Body_Runtime_SetPDD(d);
}

FX_BOOL FX_L1_Runtime_SetBodyPD(FX_DOUBLE p[6], FX_DOUBLE d[6])
{
    for (FX_INT32 i = 0; i < 6; i++)
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

    if (!FX_L0_Body_Runtime_SetPDP(p) || !FX_L0_Body_Runtime_SetPDD(d))
    {
        return FX_FALSE;
    }
    return FX_TRUE;
}

FX_UINT32 FX_L1_Runtime_RunTraj(FX_UINT32 obj_mask)
{
    FX_UINT32 ret_obj_mask = 0;
    if ((obj_mask & FX_OBJ_ARM0_FLAG) != 0)
    {
        if (FX_L0_Arm0_Runtime_RunTraj())
        {
            ret_obj_mask |= FX_OBJ_ARM0_FLAG;
        }
    }
    if ((obj_mask & FX_OBJ_ARM1_FLAG) != 0)
    {
        if (FX_L0_Arm1_Runtime_RunTraj())
        {
            ret_obj_mask |= FX_OBJ_ARM1_FLAG;
        }
    }
    return ret_obj_mask;
}

FX_UINT32 FX_L1_Runtime_StopTraj(FX_UINT32 obj_mask)
{
    FX_UINT32 ret_obj_mask = 0;
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
    return ret_obj_mask;
}

/*=============================================================================
 * 运动学与规划接口
 *============================================================================*/

FX_MotionHandle FX_L1_Kinematics_Create(FX_VOID)
{
    return FX_L0_Kinematics_create();
}

FX_VOID FX_L1_Kinematics_Destroy(FX_MotionHandle handle)
{
    FX_L0_Kinematics_destroy(handle);
}

FX_VOID FX_L1_Kinematics_LogSwitch(FX_MotionHandle handle, FX_INT32 on)
{
    FX_L0_Kinematics_log_switch(handle, on);
}

FX_BOOL FX_L1_Kinematics_InitSingleArm(FX_MotionHandle handle, const FX_CHAR *env_path, FX_INT32 robot_serial)
{
    return (FX_L0_Kinematics_init_single_arm(handle, env_path, robot_serial) == FX_MOTION_OK) ? FX_TRUE : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_InitDualArm(FX_MotionHandle handle, const FX_CHAR *env_path)
{
    return (FX_L0_Kinematics_init_dual_arm(handle, env_path) == FX_MOTION_OK) ? FX_TRUE : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_ForwardKinematics(FX_MotionHandle handle, FX_INT32 robot_serial,
                                           const FX_DOUBLE joints[7], FX_DOUBLE pose_matrix[16])
{
    return (FX_L0_Kinematics_forward_kinematics(handle, robot_serial, joints, pose_matrix) == FX_MOTION_OK) ? FX_TRUE : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_Jacobian(FX_MotionHandle handle, FX_INT32 robot_serial,
                                  const FX_DOUBLE joints[7], FX_DOUBLE jacobian[42])
{
    return (FX_L0_Kinematics_jacobian(handle, robot_serial, joints, jacobian) == FX_MOTION_OK) ? FX_TRUE : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_InverseKinematics(FX_MotionHandle handle, FX_INT32 robot_serial,
                                           FX_InvKineSolverParams *params)
{
    return (FX_L0_Kinematics_inverse_kinematics(handle, robot_serial, params) == FX_MOTION_OK) ? FX_TRUE : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_GetJointLimits(FX_MotionHandle handle, FX_INT32 robot_serial,
                                        FX_INT32 *robot_type,
                                        FX_DOUBLE pos_limit[7], FX_DOUBLE neg_limit[7],
                                        FX_DOUBLE vel_limit[7], FX_DOUBLE acc_limit[7])
{
    return (FX_L0_Kinematics_get_joint_limits(handle, robot_serial, robot_type,
                                              pos_limit, neg_limit, vel_limit, acc_limit) == FX_MOTION_OK)
               ? FX_TRUE
               : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_SetBodyCondition(FX_MotionHandle handle,
                                          const FX_DOUBLE std_body[3], const FX_DOUBLE k_body[3],
                                          FX_DOUBLE std_left_len, FX_DOUBLE k_left,
                                          FX_DOUBLE std_right_len, FX_DOUBLE k_right)
{
    return (FX_L0_Kinematics_set_body_condition(handle, std_body, k_body,
                                                std_left_len, k_left, std_right_len, k_right) == FX_MOTION_OK)
               ? FX_TRUE
               : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_BodyForward(FX_MotionHandle handle, const FX_DOUBLE jv[3],
                                     FX_DOUBLE left_shoulder_matrix[16], FX_DOUBLE right_shoulder_matrix[16])
{
    return (FX_L0_Kinematics_body_forward(handle, jv, left_shoulder_matrix, right_shoulder_matrix) == FX_MOTION_OK) ? FX_TRUE : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_CalcBodyPosition(FX_MotionHandle handle,
                                          const FX_DOUBLE left_tcp[3], const FX_DOUBLE right_tcp[3],
                                          FX_DOUBLE out_body_joints[3])
{
    return (FX_L0_Kinematics_calc_body_position(handle, left_tcp, right_tcp, out_body_joints) == FX_MOTION_OK) ? FX_TRUE : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_CalcBodyPositionWithRef(FX_MotionHandle handle,
                                                 const FX_DOUBLE ref_body_joints[3],
                                                 const FX_DOUBLE left_tcp[3], const FX_DOUBLE right_tcp[3],
                                                 FX_DOUBLE out_body_joints[3])
{
    return (FX_L0_Kinematics_calc_body_position_with_ref(handle, ref_body_joints, left_tcp, right_tcp, out_body_joints) == FX_MOTION_OK) ? FX_TRUE : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_PlanJointMove(FX_MotionHandle handle, FX_INT32 robot_serial,
                                       const FX_DOUBLE start_joints[7], const FX_DOUBLE end_joints[7],
                                       FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio,
                                       FX_DOUBLE *point_set_handle, FX_INT32 *point_num)
{
    return (FX_L0_Kinematics_plan_joint_move(handle, robot_serial, start_joints, end_joints,
                                             vel_ratio, acc_ratio, point_set_handle, point_num) == FX_MOTION_OK)
               ? FX_TRUE
               : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_PlanLinearMove(FX_MotionHandle handle, FX_INT32 robot_serial,
                                        const FX_DOUBLE start_xyzabc[6], const FX_DOUBLE end_xyzabc[6],
                                        const FX_DOUBLE ref_joints[7],
                                        FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq,
                                        FX_DOUBLE *point_set_handle, FX_INT32 *point_num)
{
    return (FX_L0_Kinematics_plan_linear_move(handle, robot_serial, start_xyzabc, end_xyzabc,
                                              ref_joints, vel, acc, freq, point_set_handle, point_num) == FX_MOTION_OK)
               ? FX_TRUE
               : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_PlanLinearKeepJoints(FX_MotionHandle handle, FX_INT32 robot_serial,
                                              const FX_DOUBLE start_joints[7], const FX_DOUBLE end_joints[7],
                                              FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq,
                                              FX_DOUBLE *point_set_handle, FX_INT32 *point_num)
{
    return (FX_L0_Kinematics_plan_linear_keep_joints(handle, robot_serial, start_joints, end_joints,
                                                     vel, acc, freq, point_set_handle, point_num) == FX_MOTION_OK)
               ? FX_TRUE
               : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetStart(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                             const FX_DOUBLE ref_joints[7],
                                                             const FX_DOUBLE start_xyzabc[6], const FX_DOUBLE end_xyzabc[6],
                                                             FX_DOUBLE allow_range, FX_INT32 zsp_type,
                                                             const FX_DOUBLE zsp_para[6],
                                                             FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq)
{
    return (FX_L0_Kinematics_multi_points_set_movl_start(handle, robot_serial, ref_joints,
                                                         start_xyzabc, end_xyzabc, allow_range,
                                                         zsp_type, zsp_para, vel, acc, freq) == FX_MOTION_OK)
               ? FX_TRUE
               : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetNextPoints(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                                  const FX_DOUBLE next_xyzabc[6],
                                                                  FX_DOUBLE allow_range, FX_INT32 zsp_type,
                                                                  const FX_DOUBLE zsp_para[6],
                                                                  FX_DOUBLE vel, FX_DOUBLE acc)
{
    return (FX_L0_Kinematics_multi_points_set_movl_next_points(handle, robot_serial, next_xyzabc, allow_range,
                                                               zsp_type, zsp_para, vel, acc) == FX_MOTION_OK)
               ? FX_TRUE
               : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_PlanLinearMove_MultiPoints_GetPath(FX_MotionHandle handle,
                                                            FX_DOUBLE *point_set_handle, FX_INT32 *point_num)
{
    return (FX_L0_Kinematics_multi_points_get_movl_path(handle, point_set_handle, point_num) == FX_MOTION_OK)
               ? FX_TRUE
               : FX_FALSE;
}

FX_BOOL FX_L1_Kinematics_PlanDualArmFixedBody(FX_MotionHandle handle,
                                              const DualArmFixedBodyParams *params,
                                              FX_DOUBLE *left_point_set, FX_DOUBLE *right_point_set, FX_INT32 *point_num)
{
    if (!handle || !params)
        return FX_FALSE;

    return (FX_L0_Kinematics_plan_dual_arm_fixed_body(handle, params, left_point_set, right_point_set, point_num) == FX_MOTION_OK) ? FX_TRUE : FX_FALSE;
}

FX_VOID FX_L1_XYZABC2Matrix(const FX_DOUBLE xyzabc[6], FX_DOUBLE matrix[16])
{
    FX_L0_XYZABC2Matrix(xyzabc, matrix);
}

FX_VOID FX_L1_Matrix2XYZABC(const FX_DOUBLE matrix[16], FX_DOUBLE xyzabc[6])
{
    FX_L0_Matrix2XYZABC(matrix, xyzabc);
}

FX_VOID *FX_L1_CPointSet_Create()
{
    return FX_L0_CPointSet_Create();
}

FX_VOID FX_L1_CPointSet_Destroy(FX_VOID *pset)
{
    FX_L0_CPointSet_Destroy(pset);
}

FX_BOOL FX_L1_CPointSet_OnInit(FX_VOID *pset, FX_INT32 ptype)
{
    return FX_L0_CPointSet_OnInit(pset, ptype);
}

FX_INT32 FX_L1_CPointSet_OnGetPointNum(FX_VOID *pset)
{
    return FX_L0_CPointSet_OnGetPointNum(pset);
}

FX_DOUBLE *FX_L1_CPointSet_OnGetPoint(FX_VOID *pset, FX_INT32 pos)
{
    return FX_L0_CPointSet_OnGetPoint(pset, pos);
}

FX_BOOL FX_L1_CPointSet_OnSetPoint(FX_VOID *pset, FX_DOUBLE point_value[])
{
    return FX_L0_CPointSet_OnSetPoint(pset, point_value);
}
