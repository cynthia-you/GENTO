#include "L0Robot.h"
#include <cstdio>
#include <cstring>

static FX_BOOL l0_local_log_tag = FX_TRUE;

/*=============================================================================
 * 日志开关
 *============================================================================*/
FX_VOID FX_L0_System_LocalLogOn(FX_VOID)
{
    l0_local_log_tag = FX_TRUE;
    RobotCtrl::OnLocalLogOn();
}

FX_VOID FX_L0_System_LocalLogOff(FX_VOID)
{
    l0_local_log_tag = FX_FALSE;
    RobotCtrl::OnLocalLogOff();
}

/*=============================================================================
 * 连接处理
 *============================================================================*/
FX_BOOL FX_L0_System_connect(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4)
{
    return RobotCtrl::Link(ip1, ip2, ip3, ip4);
}

FX_INT32 FX_L0_System_Testconnect(FX_VOID)
{
    return RobotCtrl::TestLink();
}

/*=============================================================================
 * 组包发送机制
 *============================================================================*/
FX_BOOL FX_L0_Communication_Clear(FX_UINT32 timeout)
{
    return RobotCtrl::ClearSend(timeout);
}

FX_BOOL FX_L0_Communication_Send(FX_VOID)
{
    return RobotCtrl::SetSend();
}

FX_INT32 FX_L0_Communication_SendWaitResponse(FX_UINT32 time_out)
{
    return RobotCtrl::WaitSend(time_out);
}

/*=============================================================================
 * 系统操作
 *============================================================================*/
FX_INT32 FX_L0_System_GetVersion(FX_VOID)
{
    return RobotCtrl::System_GetVersion();
}

FX_BOOL FX_L0_System_Reboot(FX_VOID)
{
    return RobotCtrl::System_Reboot();
}

/*=============================================================================
 * 参数读写
 *============================================================================*/
FX_BOOL FX_L0_Param_GetInt(FX_CHAR name[30], FX_INT32 *ret_value)
{
    FX_INT32 val;
    if (RobotCtrl::Para_GetInt(name, &val))
    {
        *ret_value = val;
        return FX_TRUE;
    }
    return FX_FALSE;
}

FX_BOOL FX_L0_Param_GetFloat(FX_CHAR name[30], FX_FLOAT *ret_value)
{
    FX_FLOAT val;
    if (RobotCtrl::Para_GetFloat(name, &val))
    {
        *ret_value = val;
        return FX_TRUE;
    }
    return FX_FALSE;
}

FX_BOOL FX_L0_Param_SetInt(FX_CHAR name[30], FX_INT32 target_value)
{
    return RobotCtrl::Para_SetInt(name, (FX_INT32)target_value);
}

FX_BOOL FX_L0_Param_SetFloat(FX_CHAR name[30], FX_FLOAT target_value)
{
    return RobotCtrl::Para_SetFloat(name, (FX_FLOAT)target_value);
}

FX_BOOL FX_L0_Param_Save(FX_VOID)
{
    return RobotCtrl::Para_Save();
}

/*=============================================================================
 * 末端透传数据通道
 *============================================================================*/
FX_BOOL FX_L0_Arm0_Terminal_ClearData(FX_VOID)
{
    return RobotCtrl::Arm0_Terminal_ClearData();
}

FX_INT32 FX_L0_Arm0_Terminal_GetData(FX_INT32 *channel_type_ptr, FX_UCHAR data_ptr[64])
{
    return RobotCtrl::Arm0_Terminal_GetData(channel_type_ptr, data_ptr);
}

FX_BOOL FX_L0_Arm0_Terminal_SetData(FX_INT32 channel_type, FX_UCHAR *data_ptr, FX_INT32 data_len)
{
    return RobotCtrl::Arm0_Terminal_SetData(channel_type, data_ptr, data_len);
}

FX_BOOL FX_L0_Arm1_Terminal_ClearData(FX_VOID)
{
    return RobotCtrl::Arm1_Terminal_ClearData();
}

FX_INT32 FX_L0_Arm1_Terminal_GetData(FX_INT32 *channel_type_ptr, FX_UCHAR data_ptr[64])
{
    return RobotCtrl::Arm1_Terminal_GetData(channel_type_ptr, data_ptr);
}

FX_BOOL FX_L0_Arm1_Terminal_SetData(FX_INT32 channel_type, FX_UCHAR *data_ptr, FX_INT32 data_len)
{
    return RobotCtrl::Arm1_Terminal_SetData(channel_type, data_ptr, data_len);
}

/*=============================================================================
 * 状态接口
 *============================================================================*/
FX_BOOL FX_L0_Arm0_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code)
{
    return RobotCtrl::Arm0_State_GetServoErrorCode(axis_id, error_code);
}

FX_BOOL FX_L0_Arm1_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code)
{
    return RobotCtrl::Arm1_State_GetServoErrorCode(axis_id, error_code);
}

FX_BOOL FX_L0_Head_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code)
{
    return RobotCtrl::Head_State_GetServoErrorCode(axis_id, error_code);
}

FX_BOOL FX_L0_Body_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code)
{
    return RobotCtrl::Body_State_GetServoErrorCode(axis_id, error_code);
}

FX_BOOL FX_L0_Lift_State_GetServoErrorCode(FX_INT32 axis_id, FX_INT32 *error_code)
{
    return RobotCtrl::Lift_State_GetServoErrorCode(axis_id, error_code);
}

FX_BOOL FX_L0_Arm0_State_Reset(FX_VOID)
{
    return RobotCtrl::Arm0_State_Reset();
}

FX_BOOL FX_L0_Arm0_State_EmergencyStop(FX_VOID)
{
    return RobotCtrl::Arm0_State_EmergencyStop();
}

FX_BOOL FX_L0_Arm1_State_Reset(FX_VOID)
{
    return RobotCtrl::Arm1_State_Reset();
}

FX_BOOL FX_L0_Arm1_State_EmergencyStop(FX_VOID)
{
    return RobotCtrl::Arm1_State_EmergencyStop();
}

FX_BOOL FX_L0_Head_State_Reset(FX_VOID)
{
    return RobotCtrl::Head_State_Reset();
}

FX_BOOL FX_L0_Head_State_EmergencyStop(FX_VOID)
{
    return RobotCtrl::Head_State_EmergencyStop();
}

FX_BOOL FX_L0_Body_State_Reset(FX_VOID)
{
    return RobotCtrl::Body_State_Reset();
}

FX_BOOL FX_L0_Body_State_EmergencyStop(FX_VOID)
{
    return RobotCtrl::Body_State_EmergencyStop();
}

FX_BOOL FX_L0_Lift_State_Reset(FX_VOID)
{
    return RobotCtrl::Lift_State_Reset();
}

FX_BOOL FX_L0_Lift_State_EmergencyStop(FX_VOID)
{
    return RobotCtrl::Lift_State_EmergencyStop();
}

/*=============================================================================
 * 配置接口
 *============================================================================*/
// Arm0
FX_BOOL FX_L0_Arm0_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm0_Config_SetBrakeLock(axis_mask);
}

FX_BOOL FX_L0_Arm0_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm0_Config_SetBrakeUnlock(axis_mask);
}

FX_BOOL FX_L0_Arm0_Config_ResetEncOffset(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm0_Config_ResetEncOffset(axis_mask);
}

FX_BOOL FX_L0_Arm0_Config_ClearEncError(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm0_Config_ClearEncError(axis_mask);
}

FX_BOOL FX_L0_Arm0_Config_ResetExtEncOffset(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm0_Config_ResetExtEncOffset(axis_mask);
}

FX_BOOL FX_L0_Arm0_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm0_Config_DisableSoftLimit(axis_mask);
}

FX_BOOL FX_L0_Arm0_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset)
{
    return RobotCtrl::Arm0_Config_SetSensorOffset(axis_id, offset);
}

// Arm1
FX_BOOL FX_L0_Arm1_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm1_Config_SetBrakeLock(axis_mask);
}

FX_BOOL FX_L0_Arm1_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm1_Config_SetBrakeUnlock(axis_mask);
}

FX_BOOL FX_L0_Arm1_Config_ResetEncOffset(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm1_Config_ResetEncOffset(axis_mask);
}

FX_BOOL FX_L0_Arm1_Config_ClearEncError(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm1_Config_ClearEncError(axis_mask);
}

FX_BOOL FX_L0_Arm1_Config_ResetExtEncOffset(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm1_Config_ResetExtEncOffset(axis_mask);
}

FX_BOOL FX_L0_Arm1_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
    return RobotCtrl::Arm1_Config_DisableSoftLimit(axis_mask);
}

FX_BOOL FX_L0_Arm1_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset)
{
    return RobotCtrl::Arm1_Config_SetSensorOffset(axis_id, offset);
}

// Head
FX_BOOL FX_L0_Head_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
    return RobotCtrl::Head_Config_SetBrakeLock(axis_mask);
}

FX_BOOL FX_L0_Head_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
    return RobotCtrl::Head_Config_SetBrakeUnlock(axis_mask);
}

FX_BOOL FX_L0_Head_Config_ResetEncOffset(FX_UINT8 axis_mask)
{
    return RobotCtrl::Head_Config_ResetEncOffset(axis_mask);
}

FX_BOOL FX_L0_Head_Config_ClearEncError(FX_UINT8 axis_mask)
{
    return RobotCtrl::Head_Config_ClearEncError(axis_mask);
}

FX_BOOL FX_L0_Head_Config_ResetExtEncOffset(FX_UINT8 axis_mask)
{
    return RobotCtrl::Head_Config_ResetExtEncOffset(axis_mask);
}

FX_BOOL FX_L0_Head_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
    return RobotCtrl::Head_Config_DisableSoftLimit(axis_mask);
}

// Body
FX_BOOL FX_L0_Body_Config_SetBrakeLock(FX_UINT8 axis_mask)
{
    return RobotCtrl::Body_Config_SetBrakeLock(axis_mask);
}

FX_BOOL FX_L0_Body_Config_SetBrakeUnlock(FX_UINT8 axis_mask)
{
    return RobotCtrl::Body_Config_SetBrakeUnlock(axis_mask);
}

FX_BOOL FX_L0_Body_Config_ResetEncOffset(FX_UINT8 axis_mask)
{
    return RobotCtrl::Body_Config_ResetEncOffset(axis_mask);
}

FX_BOOL FX_L0_Body_Config_ClearEncError(FX_UINT8 axis_mask)
{
    return RobotCtrl::Body_Config_ClearEncError(axis_mask);
}

FX_BOOL FX_L0_Body_Config_ResetExtEncOffset(FX_UINT8 axis_mask)
{
    return RobotCtrl::Body_Config_ResetExtEncOffset(axis_mask);
}

FX_BOOL FX_L0_Body_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
    return RobotCtrl::Body_Config_DisableSoftLimit(axis_mask);
}

FX_BOOL FX_L0_Body_Config_SetSensorOffset(FX_INT32 axis_id, FX_INT32 offset)
{
    return RobotCtrl::Body_Config_SetSensorOffset(axis_id, offset);
}

// Lift
FX_BOOL FX_L0_Lift_Config_ResetEncOffset(FX_UINT8 axis_mask)
{
    return RobotCtrl::Lift_Config_ResetEncOffset(axis_mask);
}

FX_BOOL FX_L0_Lift_Config_DisableSoftLimit(FX_UINT8 axis_mask)
{
    return RobotCtrl::Lift_Config_DisableSoftLimit(axis_mask);
}

/*=============================================================================
 * 运行时接口
 *============================================================================*/
// Arm0
FX_BOOL FX_L0_Arm0_Runtime_SetState(FX_INT32 state)
{
    return RobotCtrl::Arm0_Runtime_SetState(state);
}

FX_BOOL FX_L0_Arm0_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[7])
{
    return RobotCtrl::Arm0_Runtime_SetJointPosCmd(joint_pos);
}

FX_BOOL FX_L0_Arm0_Runtime_SetJointTorCmd(FX_DOUBLE joint_tor[7])
{
    return RobotCtrl::Arm0_Runtime_SetJointTorCmd(joint_tor);
}

FX_BOOL FX_L0_Arm0_Runtime_SetForceCtrl(FX_DOUBLE force_ctrl[5])
{
    return RobotCtrl::Arm0_Runtime_SetForceCtrl(force_ctrl);
}

FX_BOOL FX_L0_Arm0_Runtime_SetTorqueCtrl(FX_DOUBLE torque_ctrl[5])
{
    return RobotCtrl::Arm0_Runtime_SetTorqueCtrl(torque_ctrl);
}

FX_BOOL FX_L0_Arm0_Runtime_SetVelRatio(FX_DOUBLE vel_ratio)
{
    return RobotCtrl::Arm0_Runtime_SetVelRatio(vel_ratio);
}

FX_BOOL FX_L0_Arm0_Runtime_SetAccRatio(FX_DOUBLE acc_ratio)
{
    return RobotCtrl::Arm0_Runtime_SetAccRatio(acc_ratio);
}

FX_BOOL FX_L0_Arm0_Runtime_SetJointK(FX_DOUBLE k[7])
{
    return RobotCtrl::Arm0_Runtime_SetJointK(k);
}

FX_BOOL FX_L0_Arm0_Runtime_SetJointD(FX_DOUBLE d[7])
{
    return RobotCtrl::Arm0_Runtime_SetJointD(d);
}

FX_BOOL FX_L0_Arm0_Runtime_SetCartK(FX_DOUBLE k[7])
{
    return RobotCtrl::Arm0_Runtime_SetCartK(k);
}

FX_BOOL FX_L0_Arm0_Runtime_SetCartD(FX_DOUBLE d[7])
{
    return RobotCtrl::Arm0_Runtime_SetCartD(d);
}

FX_BOOL FX_L0_Arm0_Runtime_SetToolK(FX_DOUBLE k[6])
{
    return RobotCtrl::Arm0_Runtime_SetToolK(k);
}

FX_BOOL FX_L0_Arm0_Runtime_SetToolD(FX_DOUBLE d[10])
{
    return RobotCtrl::Arm0_Runtime_SetToolD(d);
}

FX_BOOL FX_L0_Arm0_Runtime_SetImpType(FX_INT32 imp_type)
{
    return RobotCtrl::Arm0_Runtime_SetImpType(imp_type);
}

FX_BOOL FX_L0_Arm0_Runtime_SetDragType(FX_INT32 drag_type)
{
    return RobotCtrl::Arm0_Runtime_SetDragType(drag_type);
}

FX_BOOL FX_L0_Arm0_Runtime_InitTraj(FX_INT32 point_num)
{
    return RobotCtrl::Arm0_Runtime_InitTraj(point_num);
}

FX_BOOL FX_L0_Arm0_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
{
    return RobotCtrl::Arm0_Runtime_SetTraj(serial, point_num, point_data);
}

FX_BOOL FX_L0_Arm0_Runtime_RunTraj(FX_VOID)
{
    return RobotCtrl::Arm0_Runtime_RunTraj();
}

FX_BOOL FX_L0_Arm0_Runtime_StopTraj(FX_VOID)
{
    return RobotCtrl::Arm0_Runtime_StopTraj();
}

// Arm1
FX_BOOL FX_L0_Arm1_Runtime_SetState(FX_INT32 state)
{
    return RobotCtrl::Arm1_Runtime_SetState(state);
}

FX_BOOL FX_L0_Arm1_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[7])
{
    return RobotCtrl::Arm1_Runtime_SetJointPosCmd(joint_pos);
}

FX_BOOL FX_L0_Arm1_Runtime_SetJointTorCmd(FX_DOUBLE joint_tor[7])
{
    return RobotCtrl::Arm1_Runtime_SetJointTorCmd(joint_tor);
}

FX_BOOL FX_L0_Arm1_Runtime_SetForceCtrl(FX_DOUBLE force_ctrl[5])
{
    return RobotCtrl::Arm1_Runtime_SetForceCtrl(force_ctrl);
}

FX_BOOL FX_L0_Arm1_Runtime_SetTorqueCtrl(FX_DOUBLE torque_ctrl[5])
{
    return RobotCtrl::Arm1_Runtime_SetTorqueCtrl(torque_ctrl);
}

FX_BOOL FX_L0_Arm1_Runtime_SetVelRatio(FX_DOUBLE vel_ratio)
{
    return RobotCtrl::Arm1_Runtime_SetVelRatio(vel_ratio);
}

FX_BOOL FX_L0_Arm1_Runtime_SetAccRatio(FX_DOUBLE acc_ratio)
{
    return RobotCtrl::Arm1_Runtime_SetAccRatio(acc_ratio);
}

FX_BOOL FX_L0_Arm1_Runtime_SetJointK(FX_DOUBLE k[7])
{
    return RobotCtrl::Arm1_Runtime_SetJointK(k);
}

FX_BOOL FX_L0_Arm1_Runtime_SetJointD(FX_DOUBLE d[7])
{
    return RobotCtrl::Arm1_Runtime_SetJointD(d);
}

FX_BOOL FX_L0_Arm1_Runtime_SetCartK(FX_DOUBLE k[7])
{
    return RobotCtrl::Arm1_Runtime_SetCartK(k);
}

FX_BOOL FX_L0_Arm1_Runtime_SetCartD(FX_DOUBLE d[7])
{
    return RobotCtrl::Arm1_Runtime_SetCartD(d);
}

FX_BOOL FX_L0_Arm1_Runtime_SetToolK(FX_DOUBLE k[6])
{
    return RobotCtrl::Arm1_Runtime_SetToolK(k);
}

FX_BOOL FX_L0_Arm1_Runtime_SetToolD(FX_DOUBLE d[10])
{
    return RobotCtrl::Arm1_Runtime_SetToolD(d);
}

FX_BOOL FX_L0_Arm1_Runtime_SetImpType(FX_INT32 imp_type)
{
    return RobotCtrl::Arm1_Runtime_SetImpType(imp_type);
}

FX_BOOL FX_L0_Arm1_Runtime_SetDragType(FX_INT32 drag_type)
{
    return RobotCtrl::Arm1_Runtime_SetDragType(drag_type);
}

FX_BOOL FX_L0_Arm1_Runtime_InitTraj(FX_INT32 point_num)
{
    return RobotCtrl::Arm1_Runtime_InitTraj(point_num);
}

FX_BOOL FX_L0_Arm1_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
{
    return RobotCtrl::Arm1_Runtime_SetTraj(serial, point_num, point_data);
}

FX_BOOL FX_L0_Arm1_Runtime_RunTraj(FX_VOID)
{
    return RobotCtrl::Arm1_Runtime_RunTraj();
}

FX_BOOL FX_L0_Arm1_Runtime_StopTraj(FX_VOID)
{
    return RobotCtrl::Arm1_Runtime_StopTraj();
}

// Head
FX_BOOL FX_L0_Head_Runtime_SetState(FX_INT32 state)
{
    return RobotCtrl::Head_Runtime_SetState(state);
}

FX_BOOL FX_L0_Head_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[3])
{
    return RobotCtrl::Head_Runtime_SetJointPosCmd(joint_pos);
}

FX_BOOL FX_L0_Head_Runtime_SetVelRatio(FX_DOUBLE vel_ratio)
{
    return RobotCtrl::Head_Runtime_SetVelRatio(vel_ratio);
}

FX_BOOL FX_L0_Head_Runtime_SetAccRatio(FX_DOUBLE acc_ratio)
{
    return RobotCtrl::Head_Runtime_SetAccRatio(acc_ratio);
}

// Body
FX_BOOL FX_L0_Body_Runtime_SetState(FX_INT32 state)
{
    return RobotCtrl::Body_Runtime_SetState(state);
}

FX_BOOL FX_L0_Body_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[6])
{
    return RobotCtrl::Body_Runtime_SetJointPosCmd(joint_pos);
}

FX_BOOL FX_L0_Body_Runtime_SetVelRatio(FX_DOUBLE vel_ratio)
{
    return RobotCtrl::Body_Runtime_SetVelRatio(vel_ratio);
}

FX_BOOL FX_L0_Body_Runtime_SetAccRatio(FX_DOUBLE acc_ratio)
{
    return RobotCtrl::Body_Runtime_SetAccRatio(acc_ratio);
}

FX_BOOL FX_L0_Body_Runtime_SetPDP(FX_DOUBLE p[6])
{
    return RobotCtrl::Body_Runtime_SetPDP(p);
}

FX_BOOL FX_L0_Body_Runtime_SetPDD(FX_DOUBLE d[6])
{
    return RobotCtrl::Body_Runtime_SetPDD(d);
}

FX_BOOL FX_L0_Body_Runtime_InitTraj(FX_INT32 point_num)
{
    return RobotCtrl::Body_Runtime_InitTraj(point_num);
}

FX_BOOL FX_L0_Body_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
{
    return RobotCtrl::Body_Runtime_SetTraj(serial, point_num, point_data);
}

FX_BOOL FX_L0_Body_Runtime_RunTraj(FX_VOID)
{
    return RobotCtrl::Body_Runtime_RunTraj();
}

FX_BOOL FX_L0_Body_Runtime_StopTraj(FX_VOID)
{
    return RobotCtrl::Body_Runtime_StopTraj();
}
// Lift
FX_BOOL FX_L0_Lift_Runtime_SetState(FX_INT32 state)
{
    return RobotCtrl::Lift_Runtime_SetState(state);
}

FX_BOOL FX_L0_Lift_Runtime_SetJointPosCmd(FX_DOUBLE joint_pos[2])
{
    return RobotCtrl::Lift_Runtime_SetJointPosCmd(joint_pos);
}

FX_BOOL FX_L0_Lift_Runtime_SetVelRatio(FX_DOUBLE vel_ratio)
{
    return RobotCtrl::Lift_Runtime_SetVelRatio(vel_ratio);
}

FX_BOOL FX_L0_Lift_Runtime_SetAccRatio(FX_DOUBLE acc_ratio)
{
    return RobotCtrl::Lift_Runtime_SetAccRatio(acc_ratio);
}

FX_BOOL FX_L0_Lift_Runtime_InitTraj(FX_INT32 point_num)
{
    return RobotCtrl::Lift_Runtime_InitTraj(point_num);
}

FX_BOOL FX_L0_Lift_Runtime_SetTraj(FX_INT32 serial, FX_INT32 point_num, FX_DOUBLE *point_data)
{
    return RobotCtrl::Lift_Runtime_SetTraj(serial, point_num, point_data);
}

FX_BOOL FX_L0_Lift_Runtime_RunTraj(FX_VOID)
{
    return RobotCtrl::Lift_Runtime_RunTraj();
}

FX_BOOL FX_L0_Lift_Runtime_StopTraj(FX_VOID)
{
    return RobotCtrl::Lift_Runtime_StopTraj();
}
/*=============================================================================
 * 通讯数据结构获取
 *============================================================================*/
const ROBOT_RT *FX_L0_GetRobotRT(FX_VOID)
{
    RobotCtrl *ctrl = RobotCtrl::GetIns();
    if (ctrl == NULL)
        return NULL;
    return &(ctrl->m_RobotRT);
}

const ROBOT_SG *FX_L0_GetRobotSG(FX_VOID)
{
    RobotCtrl *ctrl = RobotCtrl::GetIns();
    if (ctrl == NULL)
        return NULL;
    return &(ctrl->m_RobotSG);
}
