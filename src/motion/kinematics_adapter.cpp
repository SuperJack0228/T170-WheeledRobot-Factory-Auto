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
int arm_hand_side(const Arm &arm)
{
    return arm.definition().hand;
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
Matrix<double,1,6> arm_flange_tool(const Arm &arm) { return arm.definition().tool; }
ArmLinkFk compute_link_fk(Arm &arm,const Eigen::RowVectorXd &q) {
    ArmLinkFk out;out.tcp=arm.Forward_Kinematics(q).topRightCorner<3,1>();
    Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
    const auto &d=arm.definition();
    for(int i=0;i<7;++i) {
        t*=MDHTrans(d.mdh(i,0),d.mdh(i,1),d.mdh(i,2),q(i)+d.mdh(i,3));
        if(i==1)out.shoulder=t.topRightCorner<3,1>();
        if(i==3)out.elbow=t.topRightCorner<3,1>();
        if(i==5)out.wrist=t.topRightCorner<3,1>();
    }
    out.flange=arm.Flange_Transform(q).topRightCorner<3,1>();
    const Eigen::Matrix4d tcp=t*TR(d.tool);out.tcp_mdh_err=(tcp.topRightCorner<3,1>()-out.tcp).norm();
    return out;
}
}

ArmLinkFk arm_fk_links(Arm &arm, const Eigen::Ref<const Eigen::RowVectorXd> &q_model)
{
    return compute_link_fk(arm, q_model);
}

ArmLinkFk arm_get_link_fk(Arm &arm)
{
    const int hand = arm_hand_side(arm);
    return arm_fk_links(arm, arm_read_model_joints(hand).row(0));
}



namespace wrist_servo {
int hand_index(const Arm &arm) { return arm_hand_side(arm); }
}


namespace wrist_servo {
Eigen::Matrix4d tool_transform(const Arm &arm) { return TR(arm_flange_tool(arm)); }
}


namespace wrist_servo {
bool solve_local_pose(Arm &arm,const Joints &seed,const Vec3 &target,
                      const Eigen::Matrix3d &rotation,const Config &cfg,
                      Joints &solution,double &xyz_residual,double &rotation_residual)
{
    std::lock_guard<std::mutex> lock(g_arm_kin_mu[arm_hand_side(arm)]);
    const ForwardModel forward=[&](const Joints &q) { return arm.Forward_Kinematics(Eigen::RowVectorXd(q)); };
    const Joints origin=g_servo_joint_origin[arm_hand_side(arm)];
    const JointLimitCheck limits=[&](const Joints &q) {
        return (q-origin).cwiseAbs().maxCoeff()>cfg.max_joint_excursion_rad ||
            arm.Check_Joint_Limit(Eigen::RowVectorXd(q));
    };
    Eigen::Matrix4d goal=Eigen::Matrix4d::Identity();goal.topRightCorner<3,1>()=target;goal.topLeftCorner<3,3>()=rotation;
    auto policy=arm.ik_policy(goal);policy.iterations=12;policy.step_rad=.05;policy.null_step_rad=.001;policy.posture_iterations=1;
    policy.minimum=policy.minimum.cwiseMax(origin-Joints::Constant(cfg.max_joint_excursion_rad));
    policy.maximum=policy.maximum.cwiseMin(origin+Joints::Constant(cfg.max_joint_excursion_rad));
    return solve_pose_with_fk(seed,target,rotation,cfg,forward,limits,solution,xyz_residual,rotation_residual,&policy);
}
}


Eigen::Matrix4d arm_fk_tcp_from_encoders(Arm &arm)
{
    const int hand = arm_hand_side(arm);
    const Eigen::MatrixXd q = arm_read_model_joints(hand);
    return arm.Forward_Kinematics(q.row(0));
}

Matrix<double, 1, 6> arm_get_tcp_pos(Arm &arm)
{
    return T2PosEulerAngles(arm_fk_tcp_from_encoders(arm));
}

Matrix<double, 1, 6> compose_base_visual(
    Arm &arm,
    const Eigen::Matrix4d &T_tcp,
    Matrix<double, 1, 6> &falan_visualPos)
{
    const Matrix<double, 1, 6> flange_tool = arm_flange_tool(arm);
    const Eigen::Matrix4d T_base_visual = T_tcp * TR(flange_tool).inverse() * TR(falan_visualPos);
    return T2PosEulerAngles(T_base_visual);
}

Matrix<double, 1, 6> arm_get_base_visual_pos(Arm &arm, Matrix<double, 1, 6> &falan_visualPos)
{
    return compose_base_visual(arm, arm_fk_tcp_from_encoders(arm), falan_visualPos);
}



Matrix<double, 1, 6> arm_tcp_from_joints(
    Arm &arm,
    const Eigen::Ref<const Eigen::RowVectorXd> &q_model)
{
    std::lock_guard<std::mutex> kin(g_arm_kin_mu[arm_hand_side(arm) == 1 ? 1 : 0]);
    return T2PosEulerAngles(arm.Forward_Kinematics(q_model));
}


