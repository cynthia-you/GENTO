#ifndef FX_L1ROBOT_H_
#define FX_L1ROBOT_H_
#include <stdio.h>
#include <cassert>
#include "L0Robot.h"
#include "FXCommon.h"
#include "L0Kinematics.h"
#include "FXFileClient.h"

// #include "L0KinematicsSDK/FxKineIF.h"

#if defined(_WIN32) || defined(_WIN64)
#ifdef L1_SDK_EXPORTS
#define FX_L1_SDK_API __declspec(dllexport)
#else
#define FX_L1_SDK_API __declspec(dllimport)
#endif
#elif defined(__linux__)
#ifdef L1_SDK_EXPORTS
#define FX_L1_SDK_API __attribute__((visibility("default")))
#else
#define FX_L1_SDK_API
#endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 连接到机器人控制器，获取版本、配置日志并读取自由度参数.
     *        Connect to robot controller, get version, config log and read DOF parameters.
     * @param ip1~ip4 IP地址四段 (0-255)，禁止0.0.0.0、广播地址和127.x.x.x
     * @param version 输出参数，返回系统版本号；可传NULL跳过
     * @param logSwitch 日志开关：0-关闭，1-开启
     * @return >0: 连接测试返回延时时间;
     *         -1: 端口被占用; -2: 1000ms无响应;
     *         -3: IP为0.0.0.0; -4: IP为广播地址; -5: IP为环回地址
     */
    FX_L1_SDK_API FX_INT32 FX_L1_System_Link(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4, FX_INT32 *version, FX_UINT32 logSwitch = 0);

    /**
     * @brief 控制器重启. 必须检查状态在idle和error时可以重启，运动过程不可重启！
     * @return FX_TRUE/FX_FALSE
     */
    FX_L1_SDK_API FX_BOOL FX_L1_System_Reboot();
    /*=============================================================================
     * 通讯组包工具
     *============================================================================*/
    FX_L1_SDK_API FX_BOOL FX_L1_Comm_Clear(FX_UINT32 timeout);
    FX_L1_SDK_API FX_BOOL FX_L1_Comm_Send();
    FX_L1_SDK_API FX_BOOL FX_L1_Comm_SendAndWait(FX_UINT32 timeout);

    /*=============================================================================
     * 状态设置
     *============================================================================*/
    FX_L1_SDK_API FX_INT32 FX_L1_Fbk_GetCtrlObjDof(FXObjType obj_type);
    FX_L1_SDK_API FXStateType FX_L1_Fbk_CurrentState(FXObjType obj_type);

    FX_L1_SDK_API FX_BOOL FX_L1_State_GetServoErrorCode(FXObjType obj_type, FX_INT32 error_code[7]);
    FX_L1_SDK_API FX_UINT32 FX_L1_State_ResetError(FX_UINT32 obj_mask);
    FX_L1_SDK_API FX_UINT32 FX_L1_State_EmergencyStop(FX_UINT32 obj_mask);
    // return 0: success
    //       -1: clear set timeout
    //       -2: send and wait timeout
    //       -3: format cmd failed, cmd list is too long
    //       -4: invalid obj type
    //       -5: controller execution timeout
    //       -6: in state that is not allowed to transfer to target state
    //       -7: object is moving, not allowed to transfer state
    //       -8: invalid input parameters
    FX_L1_SDK_API FX_INT32 FX_L1_State_SwitchToIdle(FXObjType obj_type, FX_UINT32 timeout);
    FX_L1_SDK_API FX_INT32 FX_L1_State_SwitchToPositionMode(FXObjType obj_type, FX_UINT32 timeout, FX_DOUBLE vel, FX_DOUBLE acc);
    FX_L1_SDK_API FX_INT32 FX_L1_State_SwitchToImpJointMode(FXObjType obj_type, FX_UINT32 timeout, FX_DOUBLE vel, FX_DOUBLE acc, FX_DOUBLE k[7], FX_DOUBLE d[7]);
    FX_L1_SDK_API FX_INT32 FX_L1_State_SwitchToImpCartMode(FXObjType obj_type, FX_UINT32 timeout, FX_DOUBLE vel, FX_DOUBLE acc, FX_DOUBLE k[7], FX_DOUBLE d[7]);
    FX_L1_SDK_API FX_INT32 FX_L1_State_SwitchToImpForceMode(FXObjType obj_type, FX_UINT32 timeout, FX_DOUBLE force_ctrl[FX_FORCE_DEF_NUM], FX_DOUBLE torque_ctrl[FX_TORQUE_DEF_NUM]);
    FX_L1_SDK_API FX_INT32 FX_L1_State_SwitchToDragJoint(FXObjType obj_type, FX_UINT32 timeout);
    FX_L1_SDK_API FX_INT32 FX_L1_State_SwitchToDragCartX(FXObjType obj_type, FX_UINT32 timeout);
    FX_L1_SDK_API FX_INT32 FX_L1_State_SwitchToDragCartY(FXObjType obj_type, FX_UINT32 timeout);
    FX_L1_SDK_API FX_INT32 FX_L1_State_SwitchToDragCartZ(FXObjType obj_type, FX_UINT32 timeout);
    FX_L1_SDK_API FX_INT32 FX_L1_State_SwitchToDragCartR(FXObjType obj_type, FX_UINT32 timeout);
    FX_L1_SDK_API FX_INT32 FX_L1_State_SwitchToCollaborativeRelease(FXObjType obj_type, FX_UINT32 timeout);

    /*=============================================================================
     * 设置和获取配置参数（参数名参考ini）
     *============================================================================*/
    FX_L1_SDK_API FX_BOOL FX_L1_Param_SetInt32(FX_CHAR *name, FX_INT32 value);
    FX_L1_SDK_API FX_BOOL FX_L1_Param_SetFloat(FX_CHAR *name, FX_FLOAT value);
    FX_L1_SDK_API FX_BOOL FX_L1_Param_GetInt32(FX_CHAR *name, FX_INT32 *value);
    FX_L1_SDK_API FX_BOOL FX_L1_Param_GetFloat(FX_CHAR *name, FX_FLOAT *value);

    /*=============================================================================
     * 末端工具通讯
     *============================================================================*/
    FX_L1_SDK_API FX_BOOL FX_L1_Terminal_ClearData(FXTerminalType terminal_type);
    FX_L1_SDK_API FX_INT32 FX_L1_Terminal_GetData(FXTerminalType terminal_type, FXChnType *chn_type, FX_UCHAR data[64]);
    FX_L1_SDK_API FX_BOOL FX_L1_Terminal_SetData(FXTerminalType terminal_type, FXChnType chn_type, FX_UCHAR data[64], FX_UINT32 data_len);

    /*=============================================================================
     * 机器人配置：抱闸 松闸 重置内部编码器偏移  清除内部编码器错误  设置轴传感器偏移 设置轨迹
     *============================================================================*/
    FX_L1_SDK_API FX_BOOL FX_L1_Config_SetBrakeLock(FXObjType obj_type, FX_UINT8 axis_mask);
    FX_L1_SDK_API FX_BOOL FX_L1_Config_SetBrakeUnlock(FXObjType obj_type, FX_UINT8 axis_mask);
    FX_L1_SDK_API FX_BOOL FX_L1_Config_ResetEncOffset(FXObjType obj_type, FX_UINT8 axis_mask);
    FX_L1_SDK_API FX_BOOL FX_L1_Config_ClearEncError(FXObjType obj_type, FX_UINT8 axis_mask);
    FX_L1_SDK_API FX_BOOL FX_L1_Config_SetAxisSensorOffset(FXObjType obj_type, FX_UINT32 axis_id, FX_INT32 offset);
    FX_L1_SDK_API FX_BOOL FX_L1_Config_DisableSoftLimit(FXObjType obj_type, FX_UINT8 axis_mask);
    FX_L1_SDK_API FX_BOOL FX_L1_Config_SetSensorOffset(FXObjType obj_type, FX_INT32 offset[7]);
    FX_L1_SDK_API FX_BOOL FX_L1_Config_SetTraj(FXObjType obj_type, FX_UINT32 point_num, FX_DOUBLE *point_data);

    /*=============================================================================
     * 实时通讯，所有指令必须在FX_L1_Comm_Clear()和FX_L1_Comm_Send()之间调用。
     * 可多条指令在FX_L1_Comm_Clear()和FX_L1_Comm_Send()之间调用实现同时生效。
     *============================================================================*/
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetJointPosCmd(FXObjType obj_type, FX_DOUBLE pos_cmd[7]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetForceCtrl(FXObjType obj_type, FX_DOUBLE force_ctrl[FX_FORCE_DEF_NUM]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetTorqueCtrl(FXObjType obj_type, FX_DOUBLE torque_ctrl[FX_TORQUE_DEF_NUM]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetVelRatio(FXObjType obj_type, FX_DOUBLE vel_ratio);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetAccRatio(FXObjType obj_type, FX_DOUBLE acc_ratio);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetSpeedRatio(FXObjType obj_type, FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetJointK(FXObjType obj_type, FX_DOUBLE k[7]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetJointD(FXObjType obj_type, FX_DOUBLE d[7]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetJointKD(FXObjType obj_type, FX_DOUBLE k[7], FX_DOUBLE d[7]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetCartK(FXObjType obj_type, FX_DOUBLE k[7]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetCartD(FXObjType obj_type, FX_DOUBLE d[7]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetCartKD(FXObjType obj_type, FX_DOUBLE k[7], FX_DOUBLE d[7]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetToolK(FXObjType obj_type, FX_DOUBLE k[7]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetToolD(FXObjType obj_type, FX_DOUBLE d[7]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetToolKD(FXObjType obj_type, FX_DOUBLE k[7], FX_DOUBLE d[7]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetBodyPDP(FX_DOUBLE p[6]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetBodyPDD(FX_DOUBLE d[6]);
    FX_L1_SDK_API FX_BOOL FX_L1_Runtime_SetBodyPD(FX_DOUBLE p[6], FX_DOUBLE d[6]);
    FX_L1_SDK_API FX_UINT32 FX_L1_Runtime_RunTraj(FX_UINT32 obj_mask);
    FX_L1_SDK_API FX_UINT32 FX_L1_Runtime_StopTraj(FX_UINT32 obj_mask);

    /*=============================================================================
     * RT SG 数据接口
     *============================================================================*/
    FX_L1_SDK_API FX_INT32 RobotCache_Init();
    FX_L1_SDK_API const ROBOT_RT *RobotCache_GetRT();
    FX_L1_SDK_API const ROBOT_SG *RobotCache_GetSG();
    FX_L1_SDK_API FX_VOID RobotCache_Cleanup();

    /*=============================================================================
     * 运动学与规划接口
     *============================================================================*/

    /* 不透明句柄 */
    typedef struct FX_MotionContext *FX_MotionHandle;

    /* 生命周期管理 */
    FX_L1_SDK_API FX_MotionHandle FX_L1_Kinematics_Create(FX_VOID);
    FX_L1_SDK_API FX_VOID FX_L1_Kinematics_Destroy(FX_MotionHandle handle);
    FX_L1_SDK_API FX_VOID FX_L1_Kinematics_LogSwitch(FX_MotionHandle handle, FX_INT32 on);

    /* 初始化 */
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_InitSingleArm(FX_MotionHandle handle, const FX_CHAR *env_path, FX_INT32 robot_serial);
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_InitDualArm(FX_MotionHandle handle, const FX_CHAR *env_path);

    /* 单臂运动学 */
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_ForwardKinematics(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                             const FX_DOUBLE joints[7], FX_DOUBLE pose_matrix[16]);
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_Jacobian(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                    const FX_DOUBLE joints[7], FX_DOUBLE jacobian[42]);
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_InverseKinematics(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                             FX_InvKineSolverParams *params);

    /* 关节极限 */
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_GetJointLimits(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                          FX_INT32 *robot_type,
                                                          FX_DOUBLE pos_limit[7], FX_DOUBLE neg_limit[7],
                                                          FX_DOUBLE vel_limit[7], FX_DOUBLE acc_limit[7]);

    /* 身体运动学 */
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_SetBodyCondition(FX_MotionHandle handle,
                                                            const FX_DOUBLE std_body[3], const FX_DOUBLE k_body[3],
                                                            FX_DOUBLE std_left_len, FX_DOUBLE k_left,
                                                            FX_DOUBLE std_right_len, FX_DOUBLE k_right);
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_BodyForward(FX_MotionHandle handle, const FX_DOUBLE jv[3],
                                                       FX_DOUBLE left_shoulder_matrix[16], FX_DOUBLE right_shoulder_matrix[16]);
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_CalcBodyPosition(FX_MotionHandle handle,
                                                            const FX_DOUBLE left_tcp[3], const FX_DOUBLE right_tcp[3],
                                                            FX_DOUBLE out_body_joints[3]);
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_CalcBodyPositionWithRef(FX_MotionHandle handle,
                                                                   const FX_DOUBLE ref_body_joints[3],
                                                                   const FX_DOUBLE left_tcp[3], const FX_DOUBLE right_tcp[3],
                                                                   FX_DOUBLE out_body_joints[3]);

    /* 运动规划（单臂） */
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_PlanJointMove(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                         const FX_DOUBLE start_joints[7], const FX_DOUBLE end_joints[7],
                                                         FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio,
                                                         FX_DOUBLE *point_set_handle, FX_INT32 *point_num);
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_PlanLinearMove(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                          const FX_DOUBLE start_xyzabc[6], const FX_DOUBLE end_xyzabc[6],
                                                          const FX_DOUBLE ref_joints[7],
                                                          FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq,
                                                          FX_DOUBLE *point_set_handle, FX_INT32 *point_num);
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_PlanLinearKeepJoints(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                                const FX_DOUBLE start_joints[7], const FX_DOUBLE end_joints[7],
                                                                FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq,
                                                                FX_DOUBLE *point_set_handle, FX_INT32 *point_num);
    /* 多端直线运动规划（单臂） */
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetStart(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                                               const FX_DOUBLE ref_joints[7],
                                                                               const FX_DOUBLE start_xyzabc[6], const FX_DOUBLE end_xyzabc[6],
                                                                               FX_DOUBLE allow_range, FX_INT32 zsp_type,
                                                                               const FX_DOUBLE zsp_para[6],
                                                                               FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq);
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_PlanLinearMove_MultiPoints_SetNextPoints(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                                                    const FX_DOUBLE next_xyzabc[6],
                                                                                    FX_DOUBLE allow_range, FX_INT32 zsp_type,
                                                                                    const FX_DOUBLE zsp_para[6],
                                                                                    FX_DOUBLE vel, FX_DOUBLE acc);
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_PlanLinearMove_MultiPoints_GetPath(FX_MotionHandle handle,
                                                                              FX_DOUBLE *point_set_handle, FX_INT32 *point_num);

    /* 双臂同步规划（固定身体） */
    FX_L1_SDK_API FX_BOOL FX_L1_Kinematics_PlanDualArmFixedBody(FX_MotionHandle handle,
                                                                const DualArmFixedBodyParams *params,
                                                                FX_DOUBLE *left_point_set, FX_DOUBLE *right_point_set, FX_INT32 *point_num);

    /* 辅助工具 */
    FX_L1_SDK_API FX_VOID FX_L1_XYZABC2Matrix(const FX_DOUBLE xyzabc[6], FX_DOUBLE matrix[16]);
    FX_L1_SDK_API FX_VOID FX_L1_Matrix2XYZABC(const FX_DOUBLE matrix[16], FX_DOUBLE xyzabc[6]);

    /* PointSet 管理 */
    FX_L1_SDK_API FX_VOID *FX_L1_CPointSet_Create();
    FX_L1_SDK_API FX_VOID FX_L1_CPointSet_Destroy(FX_VOID *pset);
    FX_L1_SDK_API FX_BOOL FX_L1_CPointSet_OnInit(FX_VOID *pset, FX_INT32 ptype);
    FX_L1_SDK_API FX_INT32 FX_L1_CPointSet_OnGetPointNum(FX_VOID *pset);
    FX_L1_SDK_API FX_DOUBLE *FX_L1_CPointSet_OnGetPoint(FX_VOID *pset, FX_INT32 pos);
    FX_L1_SDK_API FX_BOOL FX_L1_CPointSet_OnSetPoint(FX_VOID *pset, FX_DOUBLE point_value[]);

    /*=============================================================================
     * 文件传输服务
     *============================================================================*/
    FX_L1_SDK_API FX_BOOL FX_L1_SendFile(FX_CHAR *local_file_path, FX_CHAR *remote_file_path);
    FX_L1_SDK_API FX_BOOL FX_L1_RecvFile(FX_CHAR *local_file_path, FX_CHAR *remote_file_path);

#ifdef __cplusplus
}
#endif

#endif
