#include "internal/arm_internal.h"
#include "t170/motion/pose_solver.h"
#include "t170/motion/joint_motion.h"
#include "t170/common/paths.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "t170/common/servo_trace.h"
#include "t170/skills/legacy/profiles.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cctype>
#include <limits>
#include <stdexcept>
using namespace Eigen;
using namespace std;
using namespace t170::motion::detail;
namespace t170::motion::detail {
int arm_hand_side(const Robot_Arm &arm)
{
    if (!arm.can_id_list.empty() && arm.can_id_list[0] >= 23)
        return 0;
    return 1;
}
}


namespace t170::motion::detail {
double encoder_cnt_to_rad(int cnt)
{
    const double deg = static_cast<double>(cnt) * 360.0 / 65536.0 / 4.0;
    return deg * M_PI / 180.0;
}
}


namespace t170::motion::detail {
int rad_to_encoder_cnt(double q_rad)
{
    const double deg = q_rad * 180.0 / M_PI;
    return static_cast<int>(deg * 65536.0 * 4.0 / 360.0);
}
}


namespace {
Matrix<double, 1, 6> arm_flange_tool(const Robot_Arm &arm)
{
    Matrix<double, 1, 6> tool;
    if (arm_hand_side(arm) == 1)
        tool << 0.26, 0, 0, 0, 0, 0;
    else
        tool << 0.26, 0, 0, 0, 0, 0;
    return tool;
}
}


namespace {
struct T170Mdh
{
    double alpha[7]{};
    double a[7]{};
    double d[7]{};
    double dh_off[7]{};
    double mdl_dir[7]{};
    double mdl_off[7]{};
    Eigen::Matrix<double, 1, 6> tool = Eigen::Matrix<double, 1, 6>::Zero();
};
}


namespace {
T170Mdh t170_mdh_from_yaml(bool is_right)
{
    const double h = M_PI / 2.0;
    T170Mdh m;
    const double al[7] = {h, h, h, -h, h, -h, -h};
    const double dho[7] = {h, -h, h, 0, 0, -h, 0};
    for (int i = 0; i < 7; ++i)
    {
        m.alpha[i] = al[i];
        m.a[i] = 0.0;
        m.dh_off[i] = dho[i];
        m.mdl_dir[i] = 1.0;
        m.mdl_off[i] = 0.0;
    }
    m.d[0] = is_right ? 0.191 : -0.191;
    m.d[2] = 0.270;
    m.d[4] = 0.250;
    if (is_right)
        m.mdl_off[1] = h;
    else
    {
        m.mdl_dir[0] = -1.0;
        m.mdl_dir[3] = -1.0;
        m.mdl_dir[5] = -1.0;
        m.mdl_off[1] = -h;
    }
    m.tool << 0.260, 0, 0, 0, 0, 0;
    return m;
}
}


namespace {
double t170_q_model(const T170Mdh &m, int i, double q_motor, int conv)
{
    if (conv == 1)
        return q_motor * m.mdl_dir[i] + m.mdl_off[i];
    const double d = m.mdl_dir[i];
    return (q_motor - m.mdl_off[i]) / (std::abs(d) < 1e-9 ? 1.0 : d);
}
}


namespace {
Eigen::Matrix4d t170_accum(const T170Mdh &m, const Eigen::RowVectorXd &q, int conv, int n_joints)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const int n = std::min(n_joints, 7);
    for (int i = 0; i < n; ++i)
    {
        const double th = t170_q_model(m, i, q(i), conv) + m.dh_off[i];
        T *= MDHTrans(m.alpha[i], m.a[i], m.d[i], th);
    }
    return T;
}
}


namespace {
double t170_tcp_err(Robot_Arm &arm, const T170Mdh &m, const Eigen::RowVectorXd &q, int conv)
{
    const Eigen::Vector3d sdk = arm.Forward_Kinematics(q).block<3, 1>(0, 3);
    const Eigen::Matrix4d T = t170_accum(m, q, conv, 7) * TR(m.tool);
    return (T.block<3, 1>(0, 3) - sdk).norm();
}
}


namespace {
struct T170MdhPick
{
    T170Mdh m;
    int conv = 0;
    double err = 1e9;
};
}


namespace {
T170MdhPick t170_pick_mdh(Robot_Arm &arm, const Eigen::RowVectorXd &q)
{
    const bool is_right = (arm_hand_side(arm) == 1);
    const T170Mdh cand = t170_mdh_from_yaml(is_right);
    T170MdhPick best;
    best.m = cand;
    for (int conv = 0; conv < 2; ++conv)
    {
        const double err = t170_tcp_err(arm, cand, q, conv);
        if (err < best.err)
        {
            best.conv = conv;
            best.err = err;
        }
    }
    static bool logged[2] = {false, false};
    const int side = is_right ? 1 : 0;
    if (!logged[side])
    {
        logged[side] = true;
        std::cout << "[fk] " << (is_right ? "右臂" : "左臂")
                  << " MDH conv=" << best.conv << " tcp_err=" << std::fixed << std::setprecision(5)
                  << best.err << " m（肘/腕用 yaml MDH 累乘，TCP 用 SDK）\n";
        if (best.err > 0.02)
            std::cerr << "[fk][WARN] 本地 MDH TCP 与 SDK 差 " << best.err
                      << " m，肘/腕坐标可能偏\n";
    }
    return best;
}
}


namespace {
ArmLinkFk compute_link_fk(Robot_Arm &arm, const Eigen::RowVectorXd &q)
{
    ArmLinkFk out;
    out.tcp = arm.Forward_Kinematics(q).block<3, 1>(0, 3);
    const T170MdhPick pick = t170_pick_mdh(arm, q);
    out.tcp_mdh_err = pick.err;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 7; ++i)
    {
        const double th = t170_q_model(pick.m, i, q(i), pick.conv) + pick.m.dh_off[i];
        T *= MDHTrans(pick.m.alpha[i], pick.m.a[i], pick.m.d[i], th);
        const Eigen::Vector3d p = T.block<3, 1>(0, 3);
        if (i == 1)
            out.shoulder = p;
        else if (i == 3)
            out.elbow = p;
        else if (i == 4)
            out.wrist = p;
        else if (i == 6)
            out.flange = p;
    }
    return out;
}
}


ArmLinkFk arm_fk_links(Robot_Arm &arm, const Eigen::Ref<const Eigen::RowVectorXd> &q_motor)
{
    return compute_link_fk(arm, q_motor);
}

ArmLinkFk arm_get_link_fk(Robot_Arm &arm)
{
    const int hand = arm_hand_side(arm);
    return arm_fk_links(arm, arm_read_motor_joints(hand).row(0));
}



namespace wrist_servo {
int hand_index(const Robot_Arm &arm) { return arm_hand_side(arm); }
}


namespace wrist_servo {
Eigen::Matrix4d tool_transform(const Robot_Arm &arm) { return TR(arm_flange_tool(arm)); }
}


namespace wrist_servo {
bool solve_local_pose(Robot_Arm &arm,const Joints &seed,const Vec3 &target,
                      const Eigen::Matrix3d &rotation,const Config &cfg,
                      Joints &solution,double &xyz_residual,double &rotation_residual)
{
    std::lock_guard<std::mutex> lock(g_arm_kin_mu[arm_hand_side(arm)]);
    const ForwardModel forward=[&](const Joints &q) { return arm.Forward_Kinematics(Eigen::RowVectorXd(q)); };
    const Joints origin=g_servo_joint_origin[arm_hand_side(arm)];
    const JointLimitCheck limits=[&](const Joints &q) {
        return (q-origin).cwiseAbs().maxCoeff()>cfg.max_joint_excursion_rad ||
            (cfg.enforce_sdk_joint_limits && arm.Check_Joint_Limit(Eigen::RowVectorXd(q)));
    };
    return solve_pose_with_fk(seed,target,rotation,cfg,forward,limits,solution,xyz_residual,rotation_residual);
}
}


Eigen::Matrix4d arm_fk_tcp_from_encoders(Robot_Arm &arm)
{
    const int hand = arm_hand_side(arm);
    const Eigen::MatrixXd q = arm_read_motor_joints(hand);
    return arm.Forward_Kinematics(q.row(0));
}

Matrix<double, 1, 6> arm_get_tcp_pos(Robot_Arm &arm)
{
    return T2PosEulerAngles(arm_fk_tcp_from_encoders(arm));
}

Matrix<double, 1, 6> compose_base_visual(
    Robot_Arm &arm,
    const Eigen::Matrix4d &T_tcp,
    Matrix<double, 1, 6> &falan_visualPos)
{
    const Matrix<double, 1, 6> flange_tool = arm_flange_tool(arm);
    const Eigen::Matrix4d T_base_visual = T_tcp * TR(flange_tool).inverse() * TR(falan_visualPos);
    return T2PosEulerAngles(T_base_visual);
}

Matrix<double, 1, 6> arm_get_base_visual_pos(Robot_Arm &arm, Matrix<double, 1, 6> &falan_visualPos)
{
    return compose_base_visual(arm, arm_fk_tcp_from_encoders(arm), falan_visualPos);
}



Matrix<double, 1, 6> arm_tcp_from_joints(
    Robot_Arm &arm,
    const Eigen::Ref<const Eigen::RowVectorXd> &q_motor)
{
    std::lock_guard<std::mutex> kin(g_arm_kin_mu[arm_hand_side(arm) == 1 ? 1 : 0]);
    return T2PosEulerAngles(arm.Forward_Kinematics(q_motor));
}


