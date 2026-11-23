#pragma once
#include <Eigen/Dense>
#include <functional>
#include <string>
#include <limits>

namespace t170::motion {
using IKJoints = Eigen::Matrix<double,1,7>;
using IKJacobian = Eigen::Matrix<double,6,7>;
using IKForward = std::function<Eigen::Matrix4d(const IKJoints &)>;
struct IKPolicy {
    IKJoints minimum=IKJoints::Constant(-1e3), maximum=IKJoints::Constant(1e3);
    IKJoints preferred=IKJoints::Zero();
    std::function<IKJacobian(const IKJoints &)> jacobian;
    std::function<Eigen::Vector3d(const IKJoints &)> elbow;
    Eigen::Vector3d elbow_target=Eigen::Vector3d::Zero();
    int iterations=100, posture_iterations=4;
    double step_rad=.10, null_step_rad=.025;
    double posture_gain=.15, elbow_gain=8, limit_gain=.002;
    double wrist_margin_rad=.14,wrist_gain=2.0;
    double position_tolerance_m=.0005, rotation_tolerance_rad=.005;
};
struct IKReport {
    int iterations=0, bound_joint=-1;
    double position_error=std::numeric_limits<double>::infinity(), rotation_error=std::numeric_limits<double>::infinity();
    std::string reason;
};
// Shared posture objective for endpoint selection and differential IK.
Eigen::Matrix<double,7,1> posture_gradient(const IKJoints &,const IKPolicy &);
double wrist_penalty(const IKJoints &,const IKPolicy &);
// Strictly convex 7-variable box QP, active bounds are reoptimized/released by KKT.
bool solve_box_qp(const Eigen::Matrix<double,7,7> &h,const Eigen::Matrix<double,7,1> &g,
                  const IKJoints &lower,const IKJoints &upper,IKJoints &solution);
bool solve_bounded_ik(const IKJoints &seed,const Eigen::Matrix4d &target,
    const IKForward &forward,const IKPolicy &policy,
    const std::function<bool(const IKJoints &)> &reject,IKJoints &solution,IKReport &report);
}
