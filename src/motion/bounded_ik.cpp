#include "t170/motion/bounded_ik.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace t170::motion {
bool solve_box_qp(const Eigen::Matrix<double,7,7> &h,const Eigen::Matrix<double,7,1> &g,
                  const IKJoints &lower,const IKJoints &upper,IKJoints &solution) {
    if(!h.allFinite() || !g.allFinite() || !lower.allFinite() || !upper.allFinite() ||
       (lower.array()>upper.array()).any())return false;
    Eigen::Matrix<double,7,1> x=Eigen::Matrix<double,7,1>::Zero();
    std::array<int,7> active{};
    for(int j=0;j<7;++j) {
        x(j)=std::clamp(0.,lower(j),upper(j));
        if(upper(j)-lower(j)<1e-12)active[j]=2;
        else if(x(j)<=lower(j)+1e-12)active[j]=-1;
        else if(x(j)>=upper(j)-1e-12)active[j]=1;
    }
    for(int iteration=0;iteration<100;++iteration) {
        const Eigen::Matrix<double,7,1> gradient=h*x+g;
        std::vector<int> free;for(int j=0;j<7;++j)if(!active[j])free.push_back(j);
        Eigen::Matrix<double,7,1> direction=Eigen::Matrix<double,7,1>::Zero();
        if(!free.empty()) {
            Eigen::MatrixXd hf(free.size(),free.size());Eigen::VectorXd gf(free.size());
            for(std::size_t a=0;a<free.size();++a) {
                gf(a)=-gradient(free[a]);
                for(std::size_t b=0;b<free.size();++b)hf(a,b)=h(free[a],free[b]);
            }
            const Eigen::VectorXd step=hf.ldlt().solve(gf);if(!step.allFinite())return false;
            for(std::size_t a=0;a<free.size();++a)direction(free[a])=step(a);
        }
        if(direction.cwiseAbs().maxCoeff()<1e-10) {
            double violation=1e-9;int release=-1;
            for(int j=0;j<7;++j) {
                const double v=active[j]==-1?-gradient(j):active[j]==1?gradient(j):0;
                if(v>violation){violation=v;release=j;}
            }
            if(release<0){solution=x.transpose();return true;}
            active[release]=0;continue;
        }
        double alpha=1;int hit=-1;
        for(int j:free) {
            const double bound=direction(j)>1e-14?(upper(j)-x(j))/direction(j):
                direction(j)<-1e-14?(lower(j)-x(j))/direction(j):2;
            if(bound<alpha){alpha=std::max(0.,bound);hit=j;}
        }
        x+=alpha*direction;
        for(int j=0;j<7;++j)x(j)=std::clamp(x(j),lower(j),upper(j));
        if(hit>=0)active[hit]=direction(hit)>0?1:-1;
    }
    return false;
}

bool solve_bounded_ik(const IKJoints &seed,const Eigen::Matrix4d &target,
    const IKForward &forward,const IKPolicy &p,const std::function<bool(const IKJoints &)> &reject,
    IKJoints &solution,IKReport &report) {
    report={};report.reason="invalid_input";
    if(!seed.allFinite() || !target.allFinite() || !forward || !p.minimum.allFinite() ||
       !p.maximum.allFinite() || !p.preferred.allFinite() || p.iterations<1 ||
       p.posture_iterations<0 || p.posture_iterations>p.iterations ||
       !std::isfinite(p.step_rad) || p.step_rad<=0 || !std::isfinite(p.null_step_rad) || p.null_step_rad<0 ||
       !std::isfinite(p.position_tolerance_m) || p.position_tolerance_m<=0 ||
       !std::isfinite(p.rotation_tolerance_rad) || p.rotation_tolerance_rad<=0 ||
       !std::isfinite(p.posture_gain) || p.posture_gain<0 || !std::isfinite(p.elbow_gain) || p.elbow_gain<0 ||
       !std::isfinite(p.limit_gain) || p.limit_gain<0 || !p.elbow_target.allFinite() ||
       (p.minimum.array()>p.maximum.array()).any())return false;
    if((seed.array()<p.minimum.array()-1e-8).any() || (seed.array()>p.maximum.array()+1e-8).any()) {
        report.reason="seed_outside_effective_limits";return false;
    }
    IKJoints q=seed;constexpr double weight=.15,h=1e-6,damping=1e-6;
    auto error=[&](const IKJoints &v) -> Eigen::Matrix<double,6,1> {
        const auto t=forward(v);const Eigen::AngleAxisd aa(target.topLeftCorner<3,3>()*t.topLeftCorner<3,3>().transpose());
        Eigen::Matrix<double,6,1> e;e.head<3>()=target.topRightCorner<3,1>()-t.topRightCorner<3,1>();
        e.tail<3>()=weight*aa.angle()*aa.axis();return e;
    };
    for(int iteration=0;iteration<p.iterations;++iteration) {
        report.iterations=iteration+1;const auto e=error(q);if(!e.allFinite()){report.reason="nonfinite_fk";return false;}
        report.position_error=e.head<3>().norm();report.rotation_error=e.tail<3>().norm()/weight;
        if(report.position_error<1e-5 && report.rotation_error<1e-4 &&
           (iteration==0 || iteration>=p.posture_iterations)) {
            if(!reject || !reject(q)){solution=q;report.reason="converged";return true;}
        }
        IKJacobian j;
        if(p.jacobian)j=p.jacobian(q);
        else {
            const auto t=forward(q);
            for(int axis=0;axis<7;++axis) {
                auto perturbed=q;perturbed(axis)+=h;const auto next=forward(perturbed);
                j.block<3,1>(0,axis)=(next.topRightCorner<3,1>()-t.topRightCorner<3,1>())/h;
                const Eigen::AngleAxisd aa(next.topLeftCorner<3,3>()*t.topLeftCorner<3,3>().transpose());
                j.block<3,1>(3,axis)=aa.angle()*aa.axis()/h;
            }
        }
        j.bottomRows<3>()*=weight;if(!j.allFinite()){report.reason="nonfinite_jacobian";return false;}
        Eigen::Matrix<double,7,1> desired=Eigen::Matrix<double,7,1>::Zero();
        if(iteration<p.posture_iterations) {
            Eigen::Matrix<double,7,1> gradient=p.posture_gain*(q-p.preferred).transpose();
            if(p.elbow) {
                const Eigen::Vector3d elbow=p.elbow(q);Eigen::Matrix<double,3,7> je;
                for(int axis=0;axis<7;++axis){auto next=q;next(axis)+=h;je.col(axis)=(p.elbow(next)-elbow)/h;}
                gradient+=p.elbow_gain*je.transpose()*(elbow-p.elbow_target);
            }
            for(int axis=0;axis<7;++axis) {
                const double low=std::max(.02,q(axis)-p.minimum(axis)),high=std::max(.02,p.maximum(axis)-q(axis));
                gradient(axis)+=std::clamp(p.limit_gain*(1/(high*high)-1/(low*low)),-2.,2.);
            }
            Eigen::Matrix<double,6,6> jj=j*j.transpose();jj.diagonal().array()+=1e-8;
            const Eigen::Matrix<double,7,7> nullspace=Eigen::Matrix<double,7,7>::Identity()-j.transpose()*jj.ldlt().solve(j);
            desired=-nullspace*gradient;
            const double largest=desired.cwiseAbs().maxCoeff();if(largest>p.null_step_rad)desired*=p.null_step_rad/largest;
        }
        Eigen::Matrix<double,7,7> hessian=j.transpose()*j;hessian.diagonal().array()+=damping;
        const Eigen::Matrix<double,7,1> linear=-j.transpose()*e-damping*desired;
        const IKJoints lower=(p.minimum-q).cwiseMax(IKJoints::Constant(-p.step_rad));
        const IKJoints upper=(p.maximum-q).cwiseMin(IKJoints::Constant(p.step_rad));
        IKJoints step;
        if(!solve_box_qp(hessian,linear,lower,upper,step)){report.reason="box_qp_failed";return false;}
        bool accepted=false;
        for(double scale=1;scale>=1./128;scale*=.5) {
            const IKJoints candidate=q+scale*step;
            if(reject && reject(candidate))continue;
            const auto next_error=error(candidate);
            // Secondary null-space motion may create a small second-order pose error;
            // primary-only polishing follows, without relaxing final residual checks.
            const double allowance=iteration<p.posture_iterations?2e-7:1e-16;
            if(next_error.allFinite() && next_error.squaredNorm()<e.squaredNorm()+allowance) {
                q=candidate;accepted=true;break;
            }
        }
        if(!accepted || step.cwiseAbs().maxCoeff()<1e-9)break;
    }
    const auto e=error(q);report.position_error=e.head<3>().norm();report.rotation_error=e.tail<3>().norm()/weight;
    const auto margin=(q-p.minimum).cwiseMin(p.maximum-q);Eigen::Index index=0;margin.minCoeff(&index);report.bound_joint=static_cast<int>(index);
    if(e.allFinite() && report.position_error<=p.position_tolerance_m && report.rotation_error<=p.rotation_tolerance_rad &&
       (!reject || !reject(q))) {solution=q;report.reason="within_tolerance";return true;}
    report.reason="not_converged";return false;
}
}
