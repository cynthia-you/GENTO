#include "L0KineMotion.h"
#include "KineCommon/FXLog.h"

/* Internal context structure holding all C++ objects */
struct FX_MotionContext
{
    CFxKineIF kine_left;
    CFxKineIF kine_right;
    CFxKineMAX body_kine;
    CFxPln planner;
};

static void copy_vect7(double src[7], Vect7 dst)
{
    for (int i = 0; i < 7; ++i)
        dst[i] = src[i];
}

static void copy_vect6(double src[6], Vect6 dst)
{
    for (int i = 0; i < 6; ++i)
        dst[i] = src[i];
}

/* ==================== Lifecycle ==================== */
FX_MotionHandle FX_L0_Kinematics_create(void)
{
    FX_MotionContext *ctx = new FX_MotionContext();
    if (!ctx)
        return nullptr;
    return ctx;
}

void FX_L0_Kinematics_destroy(FX_MotionHandle handle)
{
    if (handle)
        delete handle;
}

void FX_L0_Kinematics_set_log_level(unsigned int log_level)
{
    CLog::SetLogLevel(log_level);
}

/* ==================== Initialization ==================== */
int FX_L0_Kinematics_init_single_arm(FX_MotionHandle handle,
                                     int RobotSerial, int *type, double DH[8][4], double PNVA[8][4], double BOUND[4][3],
                                     double GRV[3], double MASS[7], double MCP[7][3], double I[7][6])
{
    if (!handle)
        return FX_MOTION_ERROR;
    if (RobotSerial == 0)
    {
        if (!handle->kine_left.OnInitEnv(0, type, DH, PNVA, BOUND, GRV, MASS, MCP, I))
            return FX_MOTION_ERROR;
    }
    else if (RobotSerial == 1)
    {
        if (!handle->kine_right.OnInitEnv(1, type, DH, PNVA, BOUND, GRV, MASS, MCP, I))
            return FX_MOTION_ERROR;
    }
    else
    {
        return FX_MOTION_ERROR;
    }
    if (!handle->planner.OnInitEnv_SingleArm(RobotSerial, type, DH, PNVA, BOUND))
        return FX_MOTION_ERROR;

    FX_LOG_INFO("Init Arm%d success, robot type = %d", RobotSerial, *type);
    return FX_MOTION_OK;
}

/* ==================== Tool Setting ==================== */
int FX_L0_Kinematics_set_tool(FX_MotionHandle handle, int robot_serial, double tool[4][4])
{
    if (!handle)
        return FX_MOTION_ERROR;
    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;

    if (!kine->OnSetTool(robot_serial, tool))
    {
        return FX_MOTION_ERROR;
    }

    if (!handle->planner.OnPlnSetTool(robot_serial, tool))
    {
        return FX_MOTION_ERROR;
    }

    FX_LOG_INFO("Set Arm%d tool Success", robot_serial);
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_remove_tool(FX_MotionHandle handle, int robot_serial)
{
    if (!handle)
        return FX_MOTION_ERROR;

    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;

    kine->OnRmvTool(robot_serial);
    handle->planner.OnPlnRemoveTool(0);

    FX_LOG_INFO("Reamove Arm%d tool Success", robot_serial);
    return FX_MOTION_OK;
}

/* ==================== Single-Arm Kinematics ==================== */
int FX_L0_Kinematics_forward_kinematics(FX_MotionHandle handle, int robot_serial,
                                        double joints[7], double pose_matrix[4][4])
{
    if (!handle || !joints || !pose_matrix)
        return FX_MOTION_ERROR;
    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;
    Vect7 jv;
    copy_vect7(joints, jv);
    if (!kine->OnSolveArmFK(jv, pose_matrix))
        return FX_MOTION_ERROR;
    
    FX_LOG_INFO("Calculate Arm%d forward kinematics Success", robot_serial);
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_jacobian(FX_MotionHandle handle, int robot_serial,
                              double joints[7], double jacobian[6][7])
{
    if (!handle || !joints || !jacobian)
        return FX_MOTION_ERROR;
    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;

    Vect7 jv;
    copy_vect7(joints, jv);
    if (!kine->OnSolveArmJcb(jv, jacobian))
        return FX_MOTION_ERROR;

    FX_LOG_INFO("Calculate Arm%d jacobian matrix success", robot_serial);
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_inverse_kinematics(FX_MotionHandle handle, int robot_serial,
                                        FX_InvKineSolvePara *params) // FX_InvKineSolverParams *params)
{
    if (!handle || !params)
        return FX_MOTION_ERROR;
    CFxKineIF *kine = (robot_serial == 0) ? &handle->kine_left : &handle->kine_right;
    if (!kine->m_InitTag)
        return FX_MOTION_ERROR;

    if (!kine->OnSolveArmIK(params))
        return FX_MOTION_ERROR;

    FX_LOG_INFO("Calculate Arm%d inverse kinematics success", robot_serial);
    return FX_MOTION_OK;
}

/* ==================== MAX Body Kinematics ==================== */
int FX_L0_Kinematics_set_body_condition(FX_MotionHandle handle,
                                        double std_body[3], double k_body[3],
                                        double std_left_len, double k_left,
                                        double std_right_len, double k_right)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect3 sb, kb;
    for (int i = 0; i < 3; ++i)
    {
        sb[i] = std_body[i];
        kb[i] = k_body[i];
    }
    handle->body_kine.OnSetCondition(sb, kb, std_left_len, k_left, std_right_len, k_right);

    FX_LOG_INFO("Set Skye body condition success");
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_body_forward(FX_MotionHandle handle, double jv[3], double left_shoulder_matrix[4][4], double right_shoulder_matrix[4][4])
{
    if (!jv || !left_shoulder_matrix || !right_shoulder_matrix)
        return FX_MOTION_ERROR;
    Vect3 jv3 = {jv[0], jv[1], jv[2]};
    handle->body_kine.OnKineLR(jv3, left_shoulder_matrix, right_shoulder_matrix);
    FX_LOG_INFO("Calculate Skye body forward kinematics success");
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_calc_body_position(FX_MotionHandle handle, double left_tcp[3], double right_tcp[3],
                                        double out_body_joints[3])
{
    if (!left_tcp || !right_tcp || !out_body_joints)
        return FX_MOTION_ERROR;
    Vect3 lt = {left_tcp[0], left_tcp[1], left_tcp[2]};
    Vect3 rt = {right_tcp[0], right_tcp[1], right_tcp[2]};
    Vect3 out;
    handle->body_kine.OnCalBody(lt, rt, out);
    out_body_joints[0] = out[0];
    out_body_joints[1] = out[1];
    out_body_joints[2] = out[2];
    FX_LOG_INFO("Calculate Skye body position success");
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_calc_body_position_with_ref(FX_MotionHandle handle, double ref_body_joints[3],
                                                 double left_tcp[3], double right_tcp[3],
                                                 double out_body_joints[3])
{
    if (!ref_body_joints || !left_tcp || !right_tcp || !out_body_joints)
        return FX_MOTION_ERROR;
    Vect3 ref = {ref_body_joints[0], ref_body_joints[1], ref_body_joints[2]};
    Vect3 lt = {left_tcp[0], left_tcp[1], left_tcp[2]};
    Vect3 rt = {right_tcp[0], right_tcp[1], right_tcp[2]};
    Vect3 out;
    handle->body_kine.OnCalBody_withref(ref, lt, rt, out);
    out_body_joints[0] = out[0];
    out_body_joints[1] = out[1];
    out_body_joints[2] = out[2];
    FX_LOG_INFO("Calculate Skye body position with reference success");
    return FX_MOTION_OK;
}

/* ==================== Motion Planning (Single Arm) ==================== */
int FX_L0_Kinematics_plan_joint_move(FX_MotionHandle handle, int robot_serial,
                                     double start_joints[7], double end_joints[7],
                                     double vel_ratio, double acc_ratio, int freq,
                                     double *point_set_handle, int *point_num)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect7 s, e;
    copy_vect7(start_joints, s);
    copy_vect7(end_joints, e);
    void *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!handle->planner.OnMovJ(s, e, vel_ratio, acc_ratio, freq, pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    *point_num = pset->OnGetPointNum();
    if (*point_num > FX_MOTION_MAX_POINT_NUM)
    {
        FX_LOG_ERRO("FX_L0_Kinematics_plan_joint_move: motion planning output points exceed the maximum allowed number");
        return FX_MOTION_ERROR;
    }
    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_c);
    FX_LOG_INFO("Plan joint move success");
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_plan_linear_move(FX_MotionHandle handle, int robot_serial,
                                      double start_xyzabc[6], double end_xyzabc[6],
                                      double ref_joints[7],
                                      double vel, double acc, int freq,
                                      double *point_set_handle, int *point_num)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect6 s, e;
    copy_vect6(start_xyzabc, s);
    copy_vect6(end_xyzabc, e);
    Vect7 ref;
    copy_vect7(ref_joints, ref);
    void *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!handle->planner.OnMovL(s, e, ref, vel, acc, freq, pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    *point_num = pset->OnGetPointNum();
    if (*point_num > FX_MOTION_MAX_POINT_NUM)
    {
        FX_LOG_ERRO("FX_L0_Kinematics_plan_linear_move: motion planning output points exceed the maximum allowed number");
        return FX_MOTION_ERROR;
    }

    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_c);
    FX_LOG_INFO("Plan linear move (OnMovL)success");
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_plan_linear_keep_joints(FX_MotionHandle handle, int robot_serial,
                                             double start_joints[7], double end_joints[7],
                                             double vel, double acc, int freq,
                                             double *point_set_handle, int *point_num)
{
    if (!handle)
        return FX_MOTION_ERROR;
    Vect7 s, e;
    copy_vect7(start_joints, s);
    copy_vect7(end_joints, e);
    void *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!handle->planner.OnMovL_KeepJ(s, e, vel, acc, freq, pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    *point_num = pset->OnGetPointNum();
    if (*point_num > FX_MOTION_MAX_POINT_NUM)
    {
        FX_LOG_ERRO("FX_L0_Kinematics_plan_linear_keep_joints: motion planning output points exceed the maximum allowed number");
        return FX_MOTION_ERROR;
    }

    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_c);
    FX_LOG_INFO("Plan linear move (OnMovL-KeepJ)success");
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_multi_points_set_movl_start(FX_MotionHandle handle, int robot_serial,
                                                 double ref_joints[7],
                                                 double start_xyzabc[6], double end_xyzabc[6],
                                                 double allow_range, int zsp_type,
                                                 double zsp_para[6],
                                                 double vel, double acc, int freq)
{
    if (!handle || !ref_joints || !start_xyzabc || !end_xyzabc || !zsp_para)
        return FX_MOTION_ERROR;

    Vect7 ref;
    Vect6 start, end, zsp;
    copy_vect7(ref_joints, ref);
    copy_vect6(start_xyzabc, start);
    copy_vect6(end_xyzabc, end);
    copy_vect6(zsp_para, zsp);

    if (!handle->planner.MultiPoints_Set_MovL_Start(ref, start, end, allow_range, zsp_type, zsp, vel, acc, freq))
        return FX_MOTION_ERROR;

    FX_LOG_INFO("Set liner move first point success");
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_multi_points_set_movl_next_points(FX_MotionHandle handle, int robot_serial,
                                                       double next_xyzabc[6],
                                                       double allow_range, int zsp_type,
                                                       double zsp_para[6],
                                                       double vel, double acc)
{
    if (!handle || !next_xyzabc || !zsp_para)
        return FX_MOTION_ERROR;

    Vect6 next, zsp;
    copy_vect6(next_xyzabc, next);
    copy_vect6(zsp_para, zsp);

    if (!handle->planner.MultiPoints_Set_MovL_NextPoints(next, allow_range, zsp_type, zsp, vel, acc))
        return FX_MOTION_ERROR;

    FX_LOG_INFO("Set liner move next point success");
    return FX_MOTION_OK;
}

int FX_L0_Kinematics_multi_points_get_movl_path(FX_MotionHandle handle,
                                                double *point_set_handle, int *point_num)
{
    if (!handle || !point_set_handle || !point_num)
        return FX_MOTION_ERROR;

    void *pset_c = FX_L0_CPointSet_Create();
    CPointSet *pset = reinterpret_cast<CPointSet *>(pset_c);
    if (!pset)
        return FX_MOTION_ERROR;

    if (!handle->planner.MultiPoints_Get_MovL_Path(pset))
    {
        FX_L0_CPointSet_Destroy(pset_c);
        return FX_MOTION_ERROR;
    }

    *point_num = pset->OnGetPointNum();
    if (*point_num > FX_MOTION_MAX_POINT_NUM)
    {
        FX_LOG_ERRO("FX_L0_Kinematics_multi_points_get_movl_path: motion planning output points exceed the maximum allowed number");
        return FX_MOTION_ERROR;
    }

    if (!FX_L0_CPointSet_OnAppendPoint(pset_c, point_set_handle))
    {
        FX_L0_CPointSet_Destroy(pset_c);
        return FX_MOTION_ERROR;
    }

    FX_L0_CPointSet_Destroy(pset_c);
    FX_LOG_INFO("Get Multi-Points liner move result success");
    return FX_MOTION_OK;
}

/* ==================== Dual-Arm Synchronized Planning ==================== */
int FX_L0_Kinematics_plan_dual_arm_fixed_body(FX_MotionHandle handle,
                                              ArmsSynchronousPlanningParams *params,
                                              double *arm0_point_set, double *arm1_point_set, int *point_num)
{
    if (!handle || !params)
        return FX_MOTION_ERROR;

    void *pset_left = FX_L0_CPointSet_Create();
    void *pset_right = FX_L0_CPointSet_Create();
    CPointSet *left_pset = reinterpret_cast<CPointSet *>(pset_left);
    CPointSet *right_pset = reinterpret_cast<CPointSet *>(pset_right);

    if (!handle->planner.OnMovL_DualArm_FixBody(params, left_pset, right_pset))
        return FX_MOTION_ERROR;

    // transfer pointset to double array
    int left_num = left_pset->OnGetPointNum();
    int right_num = right_pset->OnGetPointNum();

    if (left_num != right_num)
    {
        // Mismatched point counts indicate planning failure
        FX_LOG_ERRO("FX_L0_Kinematics_plan_dual_arm_fixed_body: point count mismatch, left=%d, right=%d", left_num, right_num);
        return FX_MOTION_ERROR;
    }
    else
    {
        *point_num = left_num;
    }

    if (*point_num > FX_MOTION_MAX_POINT_NUM)
    {
        FX_LOG_ERRO("FX_L0_Kinematics_plan_dual_arm_fixed_body: motion planning output points exceed the maximum allowed number");
        return FX_MOTION_ERROR;
    }

    if (!FX_L0_CPointSet_OnAppendPoint(pset_left, arm0_point_set) || !FX_L0_CPointSet_OnAppendPoint(pset_right, arm1_point_set))
        return FX_MOTION_ERROR;

    FX_L0_CPointSet_Destroy(pset_left);
    FX_L0_CPointSet_Destroy(pset_right);

    FX_LOG_INFO("Dual-Arm Fixed-Body linear move success");
    return FX_MOTION_OK;
}

/* ==================== Dynamic Parameters Identification ==================== */
int FX_L0_Kinematics_dynamics_identification(
    int robot_type, char *file_path, double *mass, double mr[3], double inertia[6])
{
    LoadDynamicPara DynPara;

    int type = 0;
    if (robot_type == FX_ROBOT_TYPE_PILOT_CCS)
    {
        type = 1;
    }
    else if (robot_type == FX_ROBOT_TYPE_PILOT_SRS)
    {
        type = 2;
    }
    else
    {
        return FX_MOTION_ERROR;
    }

    int ret = OnCalLoadDyn(&DynPara, type, file_path);

    *mass = DynPara.m;

    mr[0] = DynPara.r[0];
    mr[1] = DynPara.r[1];
    mr[2] = DynPara.r[2];

    inertia[0] = DynPara.I[0];
    inertia[1] = DynPara.I[1];
    inertia[2] = DynPara.I[2];
    inertia[3] = DynPara.I[3];
    inertia[4] = DynPara.I[4];
    inertia[5] = DynPara.I[5];

    if (ret != 0)
    {
        FX_LOG_ERRO("FX_L0_Kinematics_dynamics_identification: failed with error code=%d", ret);
        return FX_MOTION_ERROR;
    }

    FX_LOG_INFO("Dynamic Parameters Identification success");
    return FX_MOTION_OK;
}

/* ===================== pointset ==================*/
void *FX_L0_CPointSet_Create()
{
    return new CPointSet();
}

void FX_L0_CPointSet_Destroy(void *pset)
{
    if (pset)
    {
        delete static_cast<CPointSet *>(pset);
    }
}

int FX_L0_CPointSet_OnInit(void *pset, int ptype)
{
    if (!pset)
        return 0;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnInit(static_cast<PoinType>(ptype)) ? 1 : 0;
}

int FX_L0_CPointSet_OnGetPointNum(void *pset)
{
    if (!pset)
        return 0;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnGetPointNum();
}

double *FX_L0_CPointSet_OnGetPoint(void *pset, int pos)
{
    if (!pset)
        return nullptr;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnGetPoint(pos);
}

int FX_L0_CPointSet_OnSetPoint(void *pset, double point_value[])
{
    if (!pset)
        return 0;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    return pointSet->OnSetPoint(point_value) ? 1 : 0;
}

int FX_L0_CPointSet_OnAppendPoint(void *pset, double *point_value)
{
    if (!pset)
        return 0;
    CPointSet *pointSet = static_cast<CPointSet *>(pset);
    int num = pointSet->OnGetPointNum();

    int i = 0;
    for (i = 0; i < num; i++)
    {
        // Convert PointSet data into a double array
        double *p = pointSet->OnGetPoint(i);
        if (!p)
            return 0;

        for (int j = 0; j < 7; j++)
        {
            point_value[i * 7 + j] = p[j];
        }
    }

    return 1;
}
/* ==================== Helper Utilities ==================== */
void FX_L0_XYZABC2Matrix(double xyzabc[6], double matrix[4][4])
{
    FX_DOUBLE angx = xyzabc[3];
    FX_DOUBLE angy = xyzabc[4];
    FX_DOUBLE angz = xyzabc[5];
    FX_DOUBLE sa = 0.0;
    FX_DOUBLE sb = 0.0;
    FX_DOUBLE sr = 0.0;
    FX_DOUBLE ca = 0.0;
    FX_DOUBLE cb = 0.0;
    FX_DOUBLE cr = 0.0;

    FX_SIN_COS_DEG(angx, &sr, &cr);
    FX_SIN_COS_DEG(angy, &sb, &cb);
    FX_SIN_COS_DEG(angz, &sa, &ca);

    matrix[0][0] = ca * cb;
    matrix[0][1] = ca * sb * sr - sa * cr;
    matrix[0][2] = ca * sb * cr + sa * sr;

    matrix[1][0] = sa * cb;
    matrix[1][1] = sa * sb * sr + ca * cr;
    matrix[1][2] = sa * sb * cr - ca * sr;

    matrix[2][0] = -sb;
    matrix[2][1] = cb * sr;
    matrix[2][2] = cb * cr;

    matrix[0][3] = xyzabc[0];
    matrix[1][3] = xyzabc[1];
    matrix[2][3] = xyzabc[2];

    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
}

void FX_L0_Matrix2XYZABC(double matrix[4][4], double xyzabc[6])
{
    CFxPln pln;
    double out[6];
    pln.Matrix42XYZABC_DEG(matrix, out);
    for (int i = 0; i < 6; ++i)
        xyzabc[i] = out[i];
}
