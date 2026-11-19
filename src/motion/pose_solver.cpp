#include "t170/motion/pose_solver.h"
#include "t170/motion/bounded_ik.h"
#include <algorithm>
#include <cmath>

namespace wrist_servo {
bool solve_pose_with_fk(const Joints &seed,const Vec3 &target,const Eigen::Matrix3d &rotation,
                        const Config &cfg,const ForwardModel &forward,const JointLimitCheck &out_of_limits,
                        Joints &solution,double &xyz_residual,double &rotation_residual,
                        const t170::motion::IKPolicy *policy)
{
    xyz_residual=rotation_residual=std::numeric_limits<double>::infinity();
    if(!forward || !seed.allFinite() || !target.allFinite() || !rotation.allFinite())return false;
    auto settings=policy?*policy:t170::motion::IKPolicy{};
    if(!policy){settings.preferred=seed;settings.posture_iterations=0;settings.iterations=12;settings.step_rad=.05;}
    settings.position_tolerance_m=cfg.solver_position_tolerance_m;
    settings.rotation_tolerance_rad=cfg.solver_rotation_tolerance_rad;
    if(policy) {
        // Coarse planning establishes the elbow posture. Online secondary motion
        // must shrink with the requested TCP increment and fit a small fraction
        // of a single tick's acceleration budget, including repeated IK iterations.
        if(!std::isfinite(cfg.period_s) || cfg.period_s<=0 || !std::isfinite(cfg.max_joint_accel_rad_s2) ||
           cfg.max_joint_accel_rad_s2<=0)return false;
        const auto current=forward(seed);
        if(!current.allFinite())return false;
        const double request=(target-current.topRightCorner<3,1>()).norm()+
            .15*Eigen::AngleAxisd(rotation*current.topLeftCorner<3,3>().transpose()).angle();
        settings.null_step_rad=std::min({settings.null_step_rad,.1*request,
            .05*cfg.max_joint_accel_rad_s2*cfg.period_s*cfg.period_s/std::max(1,settings.posture_iterations)});
    }
    Eigen::Matrix4d goal=Eigen::Matrix4d::Identity();goal.topRightCorner<3,1>()=target;goal.topLeftCorner<3,3>()=rotation;
    t170::motion::IKReport report;
    const bool ok=t170::motion::solve_bounded_ik(seed,goal,forward,settings,out_of_limits,solution,report);
    xyz_residual=report.position_error;rotation_residual=report.rotation_error;return ok;
}
}
