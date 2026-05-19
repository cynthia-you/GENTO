#include "L0Robot.h"
#include <cstdio>
#include <cstring>

static unsigned char l0_local_log_tag = 1;

/*=============================================================================
 * 日志开关
 *============================================================================*/
void FX_L0_System_LocalLogOn(void)
{
    l0_local_log_tag = 1;
    RobotCtrl::OnLocalLogOn();
}

void FX_L0_System_LocalLogOff(void)
{
    l0_local_log_tag = 0;
    RobotCtrl::OnLocalLogOff();
}

/*=============================================================================
 * 连接处理
 *============================================================================*/
int FX_L0_System_Link(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4)
{
    return ((RobotCtrl::Link(ip1, ip2, ip3, ip4) == FX_TRUE) ? 0 : -1);
}

void FX_L0_System_Unlink()
{
    RobotCtrl::Unlink();
}

int FX_L0_System_Testconnect(void)
{
    return RobotCtrl::TestLink();
}

int FX_L0_System_CheckVersion(void)
{
    return ((RobotCtrl::System_CheckVersion() == FX_TRUE) ? 0 : -1);
}

int FX_L0_System_GetControllerVersion(void)
{
    return RobotCtrl::System_GetControllerVersion();
}

int FX_L0_System_GetSdkVersion(void)
{
    return RobotCtrl::System_GetSdkVersion();
}

int FX_L0_System_Reboot(void)
{
    return ((RobotCtrl::System_Reboot() == FX_TRUE) ? 0 : -1);
}

int FX_L0_System_Update(void)
{
    return ((RobotCtrl::System_Update() == FX_TRUE) ? 0 : -1);
}

/*=============================================================================
 * 组包发送机制
 *============================================================================*/
int FX_L0_Communication_Clear(unsigned int timeout)
{
    return ((RobotCtrl::ClearSend(timeout) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Communication_Send(void)
{
    return ((RobotCtrl::SetSend() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Communication_SendWaitResponse(unsigned int time_out)
{
    return RobotCtrl::WaitSend(time_out);
}

/*=============================================================================
 * 参数读写
 *============================================================================*/
int FX_L0_Param_GetInt(char name[30], int *ret_value)
{
    FX_INT32 val = 0;
    if (RobotCtrl::Para_GetInt(name, &val) == FX_TRUE)
    {
        *ret_value = val;
        return 0;
    }
    else
    {
        return -1;
    }
}

int FX_L0_Param_GetFloat(char name[30], float *ret_value)
{
    FX_FLOAT val = 0;
    if (RobotCtrl::Para_GetFloat(name, &val) == FX_TRUE)
    {
        *ret_value = val;
        return 0;
    }
    else
    {
        return -1;
    }
}

int FX_L0_Param_GetString(char name[30], char ret_value[30])
{
    if (RobotCtrl::Para_GetString(name, ret_value) == FX_TRUE)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

int FX_L0_Param_SetInt(char name[30], int target_value)
{
    return ((RobotCtrl::Para_SetInt(name, (int)target_value) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Param_SetFloat(char name[30], float target_value)
{
    return ((RobotCtrl::Para_SetFloat(name, (float)target_value) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Param_Save(void)
{
    return ((RobotCtrl::Para_Save() == FX_TRUE) ? 0 : -1);
}

/*=============================================================================
 * 末端透传数据通道
 *============================================================================*/
int FX_L0_Arm0_Terminal_ClearData(void)
{
    return ((RobotCtrl::Arm0_Terminal_ClearData() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Terminal_GetData(int *channel_type_ptr, unsigned char data_ptr[64])
{
    return RobotCtrl::Arm0_Terminal_GetData(channel_type_ptr, data_ptr);
}

int FX_L0_Arm0_Terminal_SetData(int channel_type, unsigned char *data_ptr, int data_len)
{
    return ((RobotCtrl::Arm0_Terminal_SetData(channel_type, data_ptr, data_len) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Terminal_ClearData(void)
{
    return ((RobotCtrl::Arm1_Terminal_ClearData() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Terminal_GetData(int *channel_type_ptr, unsigned char data_ptr[64])
{
    return RobotCtrl::Arm1_Terminal_GetData(channel_type_ptr, data_ptr);
}

int FX_L0_Arm1_Terminal_SetData(int channel_type, unsigned char *data_ptr, int data_len)
{
    return ((RobotCtrl::Arm1_Terminal_SetData(channel_type, data_ptr, data_len) == FX_TRUE) ? 0 : -1);
}

/*=============================================================================
 * 状态接口
 *============================================================================*/
int FX_L0_Arm0_State_GetServoErrorCode(int axis_id, unsigned int *error_code)
{
    return ((RobotCtrl::Arm0_State_GetServoErrorCode(axis_id, error_code) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_State_GetServoErrorCode(int axis_id, unsigned int *error_code)
{
    return ((RobotCtrl::Arm1_State_GetServoErrorCode(axis_id, error_code) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_State_GetServoErrorCode(int axis_id, unsigned int *error_code)
{
    return ((RobotCtrl::Head_State_GetServoErrorCode(axis_id, error_code) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_State_GetServoErrorCode(int axis_id, unsigned int *error_code)
{
    return ((RobotCtrl::Body_State_GetServoErrorCode(axis_id, error_code) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_State_GetServoErrorCode(int axis_id, unsigned int *error_code)
{
    return ((RobotCtrl::Lift_State_GetServoErrorCode(axis_id, error_code) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_State_GetServoVersion(int axis_id, char version[30])
{
    return ((RobotCtrl::Arm0_State_GetServoVersion(axis_id, version) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_State_GetServoVersion(int axis_id, char version[30])
{
    return ((RobotCtrl::Arm1_State_GetServoVersion(axis_id, version) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_State_GetServoVersion(int axis_id, char version[30])
{
    return ((RobotCtrl::Head_State_GetServoVersion(axis_id, version) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_State_GetServoVersion(int axis_id, char version[30])
{
    return ((RobotCtrl::Body_State_GetServoVersion(axis_id, version) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_State_GetSensorVersion(int axis_id, int *version)
{
    return ((RobotCtrl::Arm0_State_GetSensorVersion(axis_id, version) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_State_GetSensorVersion(int axis_id, int *version)
{
    return ((RobotCtrl::Arm1_State_GetSensorVersion(axis_id, version) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_State_GetSensorVersion(int axis_id, int *version)
{
    return ((RobotCtrl::Body_State_GetSensorVersion(axis_id, version) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_State_GetSensorSerial(int axis_id, int *serial)
{
    return ((RobotCtrl::Arm0_State_GetSensorSerial(axis_id, serial) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_State_GetSensorSerial(int axis_id, int *serial)
{
    return ((RobotCtrl::Arm1_State_GetSensorSerial(axis_id, serial) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_State_GetSensorSerial(int axis_id, int *serial)
{
    return ((RobotCtrl::Body_State_GetSensorSerial(axis_id, serial) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_State_GetPhysicalState(int *physical_state)
{
    return ((RobotCtrl::Arm0_State_GetPhyscialState(physical_state) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_State_GetPhysicalState(int *physical_state)
{
    return ((RobotCtrl::Arm1_State_GetPhyscialState(physical_state) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_State_GetPhysicalState(int *physical_state)
{
    return ((RobotCtrl::Head_State_GetPhyscialState(physical_state) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_State_GetPhysicalState(int *physical_state)
{
    return ((RobotCtrl::Body_State_GetPhyscialState(physical_state) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_State_GetPhysicalState(int *physical_state)
{
    return ((RobotCtrl::Lift_State_GetPhyscialState(physical_state) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_State_Reset(void)
{
    return ((RobotCtrl::Arm0_State_Reset() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_State_Reset(void)
{
    return ((RobotCtrl::Arm1_State_Reset() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_State_Reset(void)
{
    return ((RobotCtrl::Head_State_Reset() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_State_Reset(void)
{
    return ((RobotCtrl::Body_State_Reset() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_State_Reset(void)
{
    return ((RobotCtrl::Lift_State_Reset() == FX_TRUE) ? 0 : -1);
}

/*=============================================================================
 * 配置接口
 *============================================================================*/
// Arm0
int FX_L0_Arm0_Config_SetBrakeLock(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm0_Config_SetBrakeLock(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Config_SetBrakeUnlock(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm0_Config_SetBrakeUnlock(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Config_ResetEncSingleTurn(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm0_Config_ResetEncSingleTurn(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Config_ClearEncError(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm0_Config_ClearEncError(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Config_ResetEncMultiTurn(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm0_Config_ResetEncMultiTurn(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Config_DisableSoftLimit(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm0_Config_DisableSoftLimit(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Config_SetSensorOffset(int axis_id, int offset)
{
    return ((RobotCtrl::Arm0_Config_SetSensorOffset(axis_id, offset) == FX_TRUE) ? 0 : -1);
}

// Arm1
int FX_L0_Arm1_Config_SetBrakeLock(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm1_Config_SetBrakeLock(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Config_SetBrakeUnlock(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm1_Config_SetBrakeUnlock(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Config_ResetEncSingleTurn(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm1_Config_ResetEncSingleTurn(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Config_ClearEncError(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm1_Config_ClearEncError(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Config_ResetEncMultiTurn(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm1_Config_ResetEncMultiTurn(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Config_DisableSoftLimit(unsigned char axis_mask)
{
    return ((RobotCtrl::Arm1_Config_DisableSoftLimit(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Config_SetSensorOffset(int axis_id, int offset)
{
    return ((RobotCtrl::Arm1_Config_SetSensorOffset(axis_id, offset) == FX_TRUE) ? 0 : -1);
}

// Head
int FX_L0_Head_Config_SetBrakeLock(unsigned char axis_mask)
{
    return ((RobotCtrl::Head_Config_SetBrakeLock(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_Config_SetBrakeUnlock(unsigned char axis_mask)
{
    return ((RobotCtrl::Head_Config_SetBrakeUnlock(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_Config_ResetEncSingleTurn(unsigned char axis_mask)
{
    return ((RobotCtrl::Head_Config_ResetEncSingleTurn(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_Config_ClearEncError(unsigned char axis_mask)
{
    return ((RobotCtrl::Head_Config_ClearEncError(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_Config_ResetEncMultiTurn(unsigned char axis_mask)
{
    return ((RobotCtrl::Head_Config_ResetEncMultiTurn(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_Config_DisableSoftLimit(unsigned char axis_mask)
{
    return ((RobotCtrl::Head_Config_DisableSoftLimit(axis_mask) == FX_TRUE) ? 0 : -1);
}

// Body
int FX_L0_Body_Config_SetBrakeLock(unsigned char axis_mask)
{
    return ((RobotCtrl::Body_Config_SetBrakeLock(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Config_SetBrakeUnlock(unsigned char axis_mask)
{
    return ((RobotCtrl::Body_Config_SetBrakeUnlock(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Config_ResetEncSingleTurn(unsigned char axis_mask)
{
    return ((RobotCtrl::Body_Config_ResetEncSingleTurn(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Config_ClearEncError(unsigned char axis_mask)
{
    return ((RobotCtrl::Body_Config_ClearEncError(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Config_ResetEncMultiTurn(unsigned char axis_mask)
{
    return ((RobotCtrl::Body_Config_ResetEncMultiTurn(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Config_DisableSoftLimit(unsigned char axis_mask)
{
    return ((RobotCtrl::Body_Config_DisableSoftLimit(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Config_SetSensorOffset(int axis_id, int offset)
{
    return ((RobotCtrl::Body_Config_SetSensorOffset(axis_id, offset) == FX_TRUE) ? 0 : -1);
}

// Lift
int FX_L0_Lift_Config_ResetEncOffset(unsigned char axis_mask)
{
    return ((RobotCtrl::Lift_Config_ResetEncOffset(axis_mask) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_Config_DisableSoftLimit(unsigned char axis_mask)
{
    return ((RobotCtrl::Lift_Config_DisableSoftLimit(axis_mask) == FX_TRUE) ? 0 : -1);
}

/*=============================================================================
 * 运行时接口
 *============================================================================*/
// Arm0
int FX_L0_Arm0_Runtime_EmergencyStop(void)
{
    return ((RobotCtrl::Arm0_Runtime_EmergencyStop() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetState(int state)
{
    return ((RobotCtrl::Arm0_Runtime_SetState(state) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetJointPosCmd(double joint_pos[7])
{
    return ((RobotCtrl::Arm0_Runtime_SetJointPosCmd(joint_pos) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetJointTorCmd(double joint_tor[7])
{
    return ((RobotCtrl::Arm0_Runtime_SetJointTorCmd(joint_tor) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetForceCtrl(double force_ctrl[5])
{
    return ((RobotCtrl::Arm0_Runtime_SetForceCtrl(force_ctrl) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetTorqueCtrl(double torque_ctrl[5])
{
    return ((RobotCtrl::Arm0_Runtime_SetTorqueCtrl(torque_ctrl) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetVelRatio(double vel_ratio)
{
    return ((RobotCtrl::Arm0_Runtime_SetVelRatio(vel_ratio) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetAccRatio(double acc_ratio)
{
    return ((RobotCtrl::Arm0_Runtime_SetAccRatio(acc_ratio) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetJointK(double k[7])
{
    return ((RobotCtrl::Arm0_Runtime_SetJointK(k) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetJointD(double d[7])
{
    return ((RobotCtrl::Arm0_Runtime_SetJointD(d) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetCartK(double k[7])
{
    return ((RobotCtrl::Arm0_Runtime_SetCartK(k) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetCartD(double d[7])
{
    return ((RobotCtrl::Arm0_Runtime_SetCartD(d) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetToolK(double k[6])
{
    return ((RobotCtrl::Arm0_Runtime_SetToolK(k) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetToolD(double d[10])
{
    return ((RobotCtrl::Arm0_Runtime_SetToolD(d) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetImpType(int imp_type)
{
    return ((RobotCtrl::Arm0_Runtime_SetImpType(imp_type) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetDragType(int drag_type)
{
    return ((RobotCtrl::Arm0_Runtime_SetDragType(drag_type) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_InitTraj(int point_num)
{
    return ((RobotCtrl::Arm0_Runtime_InitTraj(point_num) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_SetTraj(int serial, int point_num, double *point_data)
{
    return ((RobotCtrl::Arm0_Runtime_SetTraj(serial, point_num, point_data) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_RunTraj(void)
{
    return ((RobotCtrl::Arm0_Runtime_RunTraj() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm0_Runtime_StopTraj(void)
{
    return ((RobotCtrl::Arm0_Runtime_StopTraj() == FX_TRUE) ? 0 : -1);
}

// Arm1
int FX_L0_Arm1_Runtime_EmergencyStop(void)
{
    return ((RobotCtrl::Arm1_Runtime_EmergencyStop() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetState(int state)
{
    return ((RobotCtrl::Arm1_Runtime_SetState(state) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetJointPosCmd(double joint_pos[7])
{
    return ((RobotCtrl::Arm1_Runtime_SetJointPosCmd(joint_pos) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetJointTorCmd(double joint_tor[7])
{
    return ((RobotCtrl::Arm1_Runtime_SetJointTorCmd(joint_tor) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetForceCtrl(double force_ctrl[5])
{
    return ((RobotCtrl::Arm1_Runtime_SetForceCtrl(force_ctrl) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetTorqueCtrl(double torque_ctrl[5])
{
    return ((RobotCtrl::Arm1_Runtime_SetTorqueCtrl(torque_ctrl) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetVelRatio(double vel_ratio)
{
    return ((RobotCtrl::Arm1_Runtime_SetVelRatio(vel_ratio) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetAccRatio(double acc_ratio)
{
    return ((RobotCtrl::Arm1_Runtime_SetAccRatio(acc_ratio) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetJointK(double k[7])
{
    return ((RobotCtrl::Arm1_Runtime_SetJointK(k) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetJointD(double d[7])
{
    return ((RobotCtrl::Arm1_Runtime_SetJointD(d) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetCartK(double k[7])
{
    return ((RobotCtrl::Arm1_Runtime_SetCartK(k) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetCartD(double d[7])
{
    return ((RobotCtrl::Arm1_Runtime_SetCartD(d) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetToolK(double k[6])
{
    return ((RobotCtrl::Arm1_Runtime_SetToolK(k) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetToolD(double d[10])
{
    return ((RobotCtrl::Arm1_Runtime_SetToolD(d) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetImpType(int imp_type)
{
    return ((RobotCtrl::Arm1_Runtime_SetImpType(imp_type) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetDragType(int drag_type)
{
    return ((RobotCtrl::Arm1_Runtime_SetDragType(drag_type) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_InitTraj(int point_num)
{
    return ((RobotCtrl::Arm1_Runtime_InitTraj(point_num) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_SetTraj(int serial, int point_num, double *point_data)
{
    return ((RobotCtrl::Arm1_Runtime_SetTraj(serial, point_num, point_data) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_RunTraj(void)
{
    return ((RobotCtrl::Arm1_Runtime_RunTraj() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Arm1_Runtime_StopTraj(void)
{
    return ((RobotCtrl::Arm1_Runtime_StopTraj() == FX_TRUE) ? 0 : -1);
}

// Head
int FX_L0_Head_Runtime_EmergencyStop(void)
{
    return ((RobotCtrl::Head_Runtime_EmergencyStop() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_Runtime_SetState(int state)
{
    return ((RobotCtrl::Head_Runtime_SetState(state) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_Runtime_SetJointPosCmd(double joint_pos[3])
{
    return ((RobotCtrl::Head_Runtime_SetJointPosCmd(joint_pos) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_Runtime_SetVelRatio(double vel_ratio)
{
    return ((RobotCtrl::Head_Runtime_SetVelRatio(vel_ratio) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Head_Runtime_SetAccRatio(double acc_ratio)
{
    return ((RobotCtrl::Head_Runtime_SetAccRatio(acc_ratio) == FX_TRUE) ? 0 : -1);
}

// Body
int FX_L0_Body_Runtime_EmergencyStop(void)
{
    return ((RobotCtrl::Body_Runtime_EmergencyStop() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Runtime_SetState(int state)
{
    return ((RobotCtrl::Body_Runtime_SetState(state) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Runtime_SetJointPosCmd(double joint_pos[6])
{
    return ((RobotCtrl::Body_Runtime_SetJointPosCmd(joint_pos) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Runtime_SetVelRatio(double vel_ratio)
{
    return ((RobotCtrl::Body_Runtime_SetVelRatio(vel_ratio) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Runtime_SetAccRatio(double acc_ratio)
{
    return ((RobotCtrl::Body_Runtime_SetAccRatio(acc_ratio) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Runtime_SetPDP(double p[6])
{
    return ((RobotCtrl::Body_Runtime_SetPDP(p) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Runtime_SetPDD(double d[6])
{
    return ((RobotCtrl::Body_Runtime_SetPDD(d) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Runtime_InitTraj(int point_num)
{
    return ((RobotCtrl::Body_Runtime_InitTraj(point_num) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Runtime_SetTraj(int serial, int point_num, double *point_data)
{
    return ((RobotCtrl::Body_Runtime_SetTraj(serial, point_num, point_data) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Runtime_RunTraj(void)
{
    return ((RobotCtrl::Body_Runtime_RunTraj() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Body_Runtime_StopTraj(void)
{
    return ((RobotCtrl::Body_Runtime_StopTraj() == FX_TRUE) ? 0 : -1);
}
// Lift

int FX_L0_Lift_Runtime_EmergencyStop(void)
{
    return ((RobotCtrl::Lift_Runtime_EmergencyStop() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_Runtime_SetState(int state)
{
    return ((RobotCtrl::Lift_Runtime_SetState(state) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_Runtime_SetJointPosCmd(double joint_pos[2])
{
    return ((RobotCtrl::Lift_Runtime_SetJointPosCmd(joint_pos) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_Runtime_SetVelRatio(double vel_ratio)
{
    return ((RobotCtrl::Lift_Runtime_SetVelRatio(vel_ratio) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_Runtime_SetAccRatio(double acc_ratio)
{
    return ((RobotCtrl::Lift_Runtime_SetAccRatio(acc_ratio) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_Runtime_InitTraj(int point_num)
{
    return ((RobotCtrl::Lift_Runtime_InitTraj(point_num) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_Runtime_SetTraj(int serial, int point_num, double *point_data)
{
    return ((RobotCtrl::Lift_Runtime_SetTraj(serial, point_num, point_data) == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_Runtime_RunTraj(void)
{
    return ((RobotCtrl::Lift_Runtime_RunTraj() == FX_TRUE) ? 0 : -1);
}

int FX_L0_Lift_Runtime_StopTraj(void)
{
    return ((RobotCtrl::Lift_Runtime_StopTraj() == FX_TRUE) ? 0 : -1);
}
/*=============================================================================
 * 通讯数据结构获取
 *============================================================================*/
const ROBOT_RT *FX_L0_GetRobotRT(void)
{
    RobotCtrl *ctrl = RobotCtrl::GetIns();
    if (ctrl == NULL)
    {
        return NULL;
    }
    return &ctrl->m_RobotRT;
}

const ROBOT_SG *FX_L0_GetRobotSG(void)
{
    RobotCtrl *ctrl = RobotCtrl::GetIns();
    if (ctrl == NULL)
    {
        return NULL;
    }
    return &ctrl->m_RobotSG;
}
