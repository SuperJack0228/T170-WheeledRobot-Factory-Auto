#include "servo/visual_servo.h"
#include <cmath>

namespace wrist_servo {
bool solve_pose_with_fk(const Joints &seed,const Vec3 &target,const Eigen::Matrix3d &rotation,
                        const Config &cfg,const ForwardModel &forward,const JointLimitCheck &out_of_limits,
                        Joints &solution,double &xyz_residual,double &rotation_residual)
{
    if(!seed.allFinite() || !target.allFinite() || !rotation.allFinite()) return false;
    Joints q=seed;
    constexpr double h=1e-5, rotation_weight=0.12;
    auto rotation_vector=[](const Eigen::Matrix3d &R) -> Eigen::Vector3d {
        const Eigen::AngleAxisd angle(R);
        return angle.angle()*angle.axis();
    };
    for(int iteration=0;iteration<6;++iteration) {
        const Eigen::Matrix4d T=forward(q);
        if(!T.allFinite()) return false;
        const Vec3 e=target-T.block<3,1>(0,3);
        const Vec3 er=rotation_vector(rotation*T.block<3,3>(0,0).transpose());
        xyz_residual=e.norm(); rotation_residual=er.norm();
        if(xyz_residual<=1e-6 && rotation_residual<=1e-5) {
            if(out_of_limits(q)) return false;
            solution=q; return true;
        }
        Eigen::Matrix<double,6,7> J;
        for(int j=0;j<7;++j) {
            Joints qp=q; qp(j)+=h;
            const Eigen::Matrix4d Tp=forward(qp);
            if(!Tp.allFinite()) return false;
            J.block<3,1>(0,j)=(Tp.block<3,1>(0,3)-T.block<3,1>(0,3))/h;
            J.block<3,1>(3,j)=rotation_weight*rotation_vector(Tp.block<3,3>(0,0)*T.block<3,3>(0,0).transpose())/h;
        }
        Eigen::Matrix<double,6,1> error; error << e,rotation_weight*er;
        Eigen::Matrix<double,6,6> A=J*J.transpose();
        A.diagonal().array()+=1e-6;
        Eigen::Matrix<double,7,1> step=J.transpose()*A.ldlt().solve(error);
        if(!step.allFinite()) return false;
        const double largest=step.cwiseAbs().maxCoeff();
        if(largest>0.05) step*=0.05/largest;
        bool accepted=false;
        for(double scale=1.0;scale>=0.03125;scale*=0.5) {
            Joints trial=q+scale*step.transpose();
            if(trial.allFinite() && !out_of_limits(trial)) { q=trial; accepted=true; break; }
        }
        if(!accepted) return false;
    }
    const Eigen::Matrix4d T=forward(q);
    xyz_residual=(target-T.block<3,1>(0,3)).norm();
    rotation_residual=rotation_vector(rotation*T.block<3,3>(0,0).transpose()).norm();
    if(!T.allFinite() || xyz_residual>cfg.solver_position_tolerance_m ||
       rotation_residual>cfg.solver_rotation_tolerance_rad || out_of_limits(q)) return false;
    solution=q; return true;
}
} // namespace wrist_servo
