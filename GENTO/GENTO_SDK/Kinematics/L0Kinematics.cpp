#include "L0Kinematics.h"

/* 内部上下文结构体，持有所有 C++ 对象 */
struct FX_MotionContext
{
    CFxKineIF kine_left;
    CFxKineIF kine_right;
    CFxKineMAX body_kine;
    CFxPln planner;
};

static FX_VOID array_to_matrix44(const FX_DOUBLE arr[16], Matrix4 mat)
{
    for (FX_INT32 i = 0; i < 4; ++i)
        for (FX_INT32 j = 0; j < 4; ++j)
            mat[i][j] = arr[i * 4 + j];
}

static FX_VOID matrix44_to_array(const Matrix4 mat, FX_DOUBLE arr[16])
{
    for (FX_INT32 i = 0; i < 4; ++i)
        for (FX_INT32 j = 0; j < 4; ++j)
            arr[i * 4 + j] = mat[i][j];
}

static FX_VOID copy_vect7(const FX_DOUBLE src[7], Vect7 dst)
{
    for (FX_INT32 i = 0; i < 7; ++i)
        dst[i] = src[i];
}

static FX_VOID copy_vect6(const FX_DOUBLE src[6], Vect6 dst)
{
    for (FX_INT32 i = 0; i < 6; ++i)
        dst[i] = src[i];
}

/* ==================== 生命周期 ==================== */
FX_MotionHandle FX_L0_Kinematics_create(FX_VOID)
{
    FX_MotionContext *ctx = new FX_MotionContext();
    if (!ctx)
        return nullptr;
    return ctx;
}

FX_VOID FX_L0_Kinematics_destroy(FX_MotionHandle handle)
{
    if (handle)
        delete handle;
}

FX_VOID FX_L0_Kinematics_log_switch(FX_MotionHandle handle, FX_INT32 on)
{
    if (!handle)
        return;
    handle->kine_left.L0_OnLogSwitch(on);
    handle->kine_right.L0_OnLogSwitch(on);
    handle->body_kine.L0_OnLogSwitch(on);
    handle->planner.L0_OnLogSwitch(on);
}

/* ==================== 初始化 ==================== */
FX_INT32 FX_L0_Kinematics_init_single_arm(FX_MotionHandle handle, const char *env_path, FX_INT32 robot_serial)
{
    if (!handle)
        return FX_MOTION_ERROR;
    if (robot_serial == 0)
    {
        if (!handle->kine_left.L0_OnInitEnv(const_cast<char *>(env_path), robot_serial))
            return FX_MOTION_ERROR;
    }
    else if (robot_serial == 1)
    {
        if (!handle->kine_right.L0_OnInitEnv(const_cast<char *>(env_path), robot_serial))
            return FX_MOTION_ERROR;
    }
    else
    {
        return FX_MOTION_ERROR;
    }
    if (!handle->planner.L0_OnInitEnv_SingleArm(const_cast<char *>(env_path), robot_serial))
        return FX_MOTION_ERROR;
    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_init_dual_arm(FX_MotionHandle handle, const char *env_path)
{
    if (!handle)
        return FX_MOTION_ERROR;
    if (!handle->planner.L0_OnInitEnv_DualArm(const_cast<char *>(env_path)))
        return FX_MOTION_ERROR;
    // if (!handle->kine_left.L0_OnInitEnv(const_cast<char *>(env_path), 0))
    //     return FX_MOTION_ERROR;
    // if (!handle->kine_right.L0_OnInitEnv(const_cast<char *>(env_path), 1))
    //     return FX_MOTION_ERROR;
    return FX_MOTION_OK;
}

/* ==================== 单臂运动学 ==================== */
FX_INT32 FX_L0_Kinematics_forward_kinematics(FX_MotionHandle handle, FX_INT32 robot_serial,
                                             const FX_DOUBLE joints[7], FX_DOUBLE pose_matrix[16])
{
    if (!handle || !joints || !pose_matrix)
        return FX_MOTION_ERROR;
    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;

    Vect7 jv;
    copy_vect7(joints, jv);
    Matrix4 mat;
    if (!kine->L0_OnSolveArmFK(jv, mat))
        return FX_MOTION_ERROR;
    matrix44_to_array(mat, pose_matrix);
    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_jacobian(FX_MotionHandle handle, FX_INT32 robot_serial,
                                   const FX_DOUBLE joints[7], FX_DOUBLE jacobian[42])
{
    if (!handle || !joints || !jacobian)
        return FX_MOTION_ERROR;
    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;

    Vect7 jv;
    copy_vect7(joints, jv);
    FX_DOUBLE jcb[6][7];
    if (!kine->L0_OnSolveArmJcb(jv, jcb))
        return FX_MOTION_ERROR;
    FX_INT32 idx = 0;
    for (FX_INT32 i = 0; i < 6; ++i)
        for (FX_INT32 j = 0; j < 7; ++j)
            jacobian[idx++] = jcb[i][j];
    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_inverse_kinematics(FX_MotionHandle handle, FX_INT32 robot_serial,
                                             FX_InvKineSolverParams *params)
{
    if (!handle || !params)
        return FX_MOTION_ERROR;
    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;

    FX_InvKineSolvePara para;
    memset(&para, 0, sizeof(para));

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            para.m_Input_IK_TargetTCP[i][j] = params->target_pose[i * 4 + j];
        }
    }

    for (int i = 0; i < 7; ++i)
    {
        para.m_Input_IK_RefJoint[i] = params->ref_joints[i];
    }

    if (!kine->L0_OnSolveArmIK(&para))
        return FX_MOTION_ERROR;

    for (int i = 0; i < 7; ++i)
    {
        params->solution[i] = para.m_Output_RetJoint[i];
    }
    params->solution_valid = 1;
    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_get_joint_limits(FX_MotionHandle handle, FX_INT32 robot_serial,
                                           FX_INT32 *robot_type,
                                           FX_DOUBLE pos_limit[7], FX_DOUBLE neg_limit[7],
                                           FX_DOUBLE vel_limit[7], FX_DOUBLE acc_limit[7])
{
    if (!handle)
        return FX_MOTION_ERROR;
    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;

    FX_DOUBLE lmt_neg[8] = {0}, lmt_pos[8] = {0}, lmt_vel[8] = {0}, lmt_acc[8] = {0};
    FX_INT32 type = 0;
    if (!kine->OnGetArmLmt(type, lmt_neg, lmt_pos, lmt_vel, lmt_acc))
        return FX_MOTION_ERROR;
    if (robot_type)
        *robot_type = type;
    for (FX_INT32 i = 0; i < 7; ++i)
    {
        if (neg_limit)
            neg_limit[i] = lmt_neg[i];
        if (pos_limit)
            pos_limit[i] = lmt_pos[i];
        if (vel_limit)
            vel_limit[i] = lmt_vel[i];
        if (acc_limit)
            acc_limit[i] = lmt_acc[i];
    }
    return FX_MOTION_OK;
}

/* ==================== MAX 身体运动学 ==================== */
FX_INT32 FX_L0_Kinematics_set_body_condition(FX_MotionHandle handle,
                                             const FX_DOUBLE std_body[3], const FX_DOUBLE k_body[3],
                                             FX_DOUBLE std_left_len, FX_DOUBLE k_left,
                                             FX_DOUBLE std_right_len, FX_DOUBLE k_right)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect3 sb, kb;
    for (FX_INT32 i = 0; i < 3; ++i)
    {
        sb[i] = std_body[i];
        kb[i] = k_body[i];
    }
    handle->body_kine.L0_OnSetCondition(sb, kb, std_left_len, k_left, std_right_len, k_right);
    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_body_forward(FX_MotionHandle handle, const FX_DOUBLE jv[3], FX_DOUBLE left_shoulder_matrix[16], FX_DOUBLE right_shoulder_matrix[16])
{
    if (!jv || !left_shoulder_matrix || !right_shoulder_matrix)
        return FX_MOTION_ERROR;
    Vect3 jv3 = {jv[0], jv[1], jv[2]};
    Matrix4 left, right;
    handle->body_kine.L0_OnKineLR(jv3, left, right);
    matrix44_to_array(left, left_shoulder_matrix);
    matrix44_to_array(right, right_shoulder_matrix);
    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_calc_body_position(FX_MotionHandle handle, const FX_DOUBLE left_tcp[3], const FX_DOUBLE right_tcp[3],
                                             FX_DOUBLE out_body_joints[3])
{
    if (!left_tcp || !right_tcp || !out_body_joints)
        return FX_MOTION_ERROR;
    Vect3 lt = {left_tcp[0], left_tcp[1], left_tcp[2]};
    Vect3 rt = {right_tcp[0], right_tcp[1], right_tcp[2]};
    Vect3 out;
    handle->body_kine.L0_OnCalBody(lt, rt, out);
    out_body_joints[0] = out[0];
    out_body_joints[1] = out[1];
    out_body_joints[2] = out[2];
    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_calc_body_position_with_ref(FX_MotionHandle handle, const FX_DOUBLE ref_body_joints[3],
                                                      const FX_DOUBLE left_tcp[3], const FX_DOUBLE right_tcp[3],
                                                      FX_DOUBLE out_body_joints[3])
{
    if (!ref_body_joints || !left_tcp || !right_tcp || !out_body_joints)
        return FX_MOTION_ERROR;
    Vect3 ref = {ref_body_joints[0], ref_body_joints[1], ref_body_joints[2]};
    Vect3 lt = {left_tcp[0], left_tcp[1], left_tcp[2]};
    Vect3 rt = {right_tcp[0], right_tcp[1], right_tcp[2]};
    Vect3 out;
    handle->body_kine.L0_OnCalBody_withref(ref, lt, rt, out);
    out_body_joints[0] = out[0];
    out_body_joints[1] = out[1];
    out_body_joints[2] = out[2];
    return FX_MOTION_OK;
}

/* ==================== 运动规划（单臂） ==================== */
FX_INT32 FX_L0_Kinematics_plan_joint_move(FX_MotionHandle handle, FX_INT32 robot_serial,
                                          const FX_DOUBLE start_joints[7], const FX_DOUBLE end_joints[7],
                                          FX_DOUBLE vel_ratio, FX_DOUBLE acc_ratio,
                                          FX_DOUBLE *point_set_handle, FX_INT32 *point_num)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect7 s, e;
    copy_vect7(start_joints, s);
    copy_vect7(end_joints, e);
    FX_VOID *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!handle->planner.L0_OnMovJ(s, e, vel_ratio, acc_ratio, pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    *point_num = pset->OnGetPointNum();
    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_c);

    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_plan_linear_move(FX_MotionHandle handle, FX_INT32 robot_serial,
                                           const FX_DOUBLE start_xyzabc[6], const FX_DOUBLE end_xyzabc[6],
                                           const FX_DOUBLE ref_joints[7],
                                           FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq,
                                           FX_DOUBLE *point_set_handle, FX_INT32 *point_num)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect6 s, e;
    copy_vect6(start_xyzabc, s);
    copy_vect6(end_xyzabc, e);
    Vect7 ref;
    copy_vect7(ref_joints, ref);
    FX_VOID *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!handle->planner.L0_OnMovL(s, e, ref, vel, acc, freq, pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    *point_num = pset->OnGetPointNum();
    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_c);

    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_plan_linear_keep_joints(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                  const FX_DOUBLE start_joints[7], const FX_DOUBLE end_joints[7],
                                                  FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq,
                                                  FX_DOUBLE *point_set_handle, FX_INT32 *point_num)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect7 s, e;
    copy_vect7(start_joints, s);
    copy_vect7(end_joints, e);
    FX_VOID *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!handle->planner.L0_OnMovL_KeepJ(s, e, vel, acc, freq, pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    *point_num = pset->OnGetPointNum();
    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_c);

    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_multi_points_set_movl_start(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                      const FX_DOUBLE ref_joints[7],
                                                      const FX_DOUBLE start_xyzabc[6], const FX_DOUBLE end_xyzabc[6],
                                                      FX_DOUBLE allow_range, FX_INT32 zsp_type,
                                                      const FX_DOUBLE zsp_para[6],
                                                      FX_DOUBLE vel, FX_DOUBLE acc, FX_INT32 freq)
{
    if (!handle || !ref_joints || !start_xyzabc || !end_xyzabc || !zsp_para)
        return FX_MOTION_ERROR;

    Vect7 ref;
    Vect6 start, end, zsp;
    copy_vect7(ref_joints, ref);
    copy_vect6(start_xyzabc, start);
    copy_vect6(end_xyzabc, end);
    copy_vect6(zsp_para, zsp);

    if (!handle->planner.L0_MultiPoints_Set_MovL_Start(ref, start, end, allow_range, zsp_type, zsp, vel, acc, freq))
        return FX_MOTION_ERROR;

    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_multi_points_set_movl_next_points(FX_MotionHandle handle, FX_INT32 robot_serial,
                                                            const FX_DOUBLE next_xyzabc[6],
                                                            FX_DOUBLE allow_range, FX_INT32 zsp_type,
                                                            const FX_DOUBLE zsp_para[6],
                                                            FX_DOUBLE vel, FX_DOUBLE acc)
{
    if (!handle || !next_xyzabc || !zsp_para)
        return FX_MOTION_ERROR;

    Vect6 next, zsp;
    copy_vect6(next_xyzabc, next);
    copy_vect6(zsp_para, zsp);

    if (!handle->planner.L0_MultiPoints_Set_MovL_NextPoints(next, allow_range, zsp_type, zsp, vel, acc))
        return FX_MOTION_ERROR;

    return FX_MOTION_OK;
}

FX_INT32 FX_L0_Kinematics_multi_points_get_movl_path(FX_MotionHandle handle,
                                                     FX_DOUBLE *point_set_handle, FX_INT32 *point_num)
{
    if (!handle || !point_set_handle || !point_num)
        return FX_MOTION_ERROR;

    FX_VOID *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!pset)
        return FX_MOTION_ERROR;

    if (!handle->planner.L0_MultiPoints_Get_MovL_Path(pset))
    {
        FX_L0_CPointSet_Destroy(pset_c);
        return FX_MOTION_ERROR;
    }

    *point_num = pset->OnGetPointNum();
    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
    {
        FX_L0_CPointSet_Destroy(pset_c);
        return FX_MOTION_ERROR;
    }

    FX_L0_CPointSet_Destroy(pset_c);
    return FX_MOTION_OK;
}

/* ==================== 双臂同步规划 ==================== */
FX_INT32 FX_L0_Kinematics_plan_dual_arm_fixed_body(FX_MotionHandle handle,
                                                   const DualArmFixedBodyParams *params,
                                                   FX_DOUBLE *left_point_set, FX_DOUBLE *right_point_set, FX_INT32 *point_num)
{
    if (!handle || !params)
        return FX_MOTION_ERROR;

    DualArm_FixedBody da;

    da.World_Co_Flag = params->world_co_flag;

    for (int i = 0; i < 6; ++i)
    {
        da.Left_Arm_Start_XYZABC[i] = params->left_start_xyzabc[i];
        da.Left_Arm_End_XYZABC[i] = params->left_end_xyzabc[i];
        da.Left_Arm_ZSP_Para[i] = params->left_zsp_para[i];
    }
    for (int i = 0; i < 7; ++i)
    {
        da.Left_Arm_Ref_Joints[i] = params->left_ref_joints[i];
    }
    da.Left_Arm_ZSP_Type = params->left_zsp_type;

    for (int i = 0; i < 6; ++i)
    {
        da.Right_Arm_Start_XYZABC[i] = params->right_start_xyzabc[i];
        da.Right_Arm_End_XYZABC[i] = params->right_end_xyzabc[i];
        da.Right_Arm_ZSP_Para[i] = params->right_zsp_para[i];
    }
    for (int i = 0; i < 7; ++i)
    {
        da.Right_Arm_Ref_Joints[i] = params->right_ref_joints[i];
    }
    da.Right_Arm_ZSP_Type = params->right_zsp_type;

    for (int i = 0; i < 3; ++i)
    {
        da.Max_Body_Start_PRR[i] = params->body_start_prr[i];
    }

    da.Vel = params->vel;
    da.Acc = params->acc;
    da.Freq = params->freq;
    da.Sync_Type = params->sync_type;

    FX_VOID *pset_left = FX_L0_CPointSet_Create();
    FX_VOID *pset_right = FX_L0_CPointSet_Create();
    CPointSet *left_pset = reinterpret_cast<CPointSet *>(pset_left);
    CPointSet *right_pset = reinterpret_cast<CPointSet *>(pset_right);

    if (!handle->planner.L0_OnMovL_DualArm_FixBody(&da, left_pset, right_pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    int left_num = left_pset->OnGetPointNum();
    int right_num = right_pset->OnGetPointNum();

    if(left_num!=right_num)
    {
        // 点数不一致，说明规划失败
        return FX_MOTION_ERROR;
    }
    else
    {
        *point_num = left_num;
    }
    
    if (!FX_L0_CPointSet_OnAppendPoint(pset_left, left_point_set) || !FX_L0_CPointSet_OnAppendPoint(pset_right, right_point_set))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_left);
    FX_L0_CPointSet_Destroy(pset_right);

    return FX_MOTION_OK;
}
/* ===================== pointset ==================*/
FX_VOID *FX_L0_CPointSet_Create()
{
    return new CPointSet();
}

FX_VOID FX_L0_CPointSet_Destroy(FX_VOID *pset)
{
    if (pset)
    {
        delete static_cast<CPointSet *>(pset);
    }
}

FX_BOOL FX_L0_CPointSet_OnInit(FX_VOID *pset, FX_INT32 ptype)
{
    if (!pset)
        return FX_FALSE;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnInit(static_cast<PoinType>(ptype)) ? FX_TRUE : FX_FALSE;
}

FX_INT32 FX_L0_CPointSet_OnGetPointNum(FX_VOID *pset)
{
    if (!pset)
        return 0;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnGetPointNum();
}

FX_DOUBLE *FX_L0_CPointSet_OnGetPoint(FX_VOID *pset, FX_INT32 pos)
{
    if (!pset)
        return nullptr;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnGetPoint(pos);
}

FX_BOOL FX_L0_CPointSet_OnSetPoint(FX_VOID *pset, FX_DOUBLE point_value[])
{
    if (!pset)
        return FX_FALSE;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnSetPoint(point_value) ? FX_TRUE : FX_FALSE;
}

FX_BOOL FX_L0_CPointSet_OnAppendPoint(FX_VOID *pset, FX_DOUBLE *point_value)
{
    if (!pset)
        return FX_FALSE;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    int num = pointSet->OnGetPointNum();

    int i = 0;
    for (i = 0; i < num; i++)
    {
        // pointset中的数据换成double序列
        double *p = pointSet->OnGetPoint(i);
        if (!p)
            return FX_FALSE;
            
        for (int j = 0; j < 7; j++)
        {
            point_value[i * 7 + j] = p[j];
        }
    }

    return FX_TRUE;
}
/* ==================== 辅助工具 ==================== */
FX_VOID FX_L0_XYZABC2Matrix(const FX_DOUBLE xyzabc[6], FX_DOUBLE matrix[16])
{
    CFxPln pln;
    Matrix4 m;
    FX_DOUBLE arr[6];
    for (FX_INT32 i = 0; i < 6; ++i)
        arr[i] = xyzabc[i];
    pln.L0_XYZABC2Matrix4_DEG(arr, m);
    matrix44_to_array(m, matrix);
}

FX_VOID FX_L0_Matrix2XYZABC(const FX_DOUBLE matrix[16], FX_DOUBLE xyzabc[6])
{
    CFxPln pln;
    Matrix4 m;
    array_to_matrix44(matrix, m);
    FX_DOUBLE out[6];
    pln.L0_Matrix42XYZABC_DEG(m, out);
    for (FX_INT32 i = 0; i < 6; ++i)
        xyzabc[i] = out[i];
}