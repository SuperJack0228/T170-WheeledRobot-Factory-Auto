#include "t170/motion/differential_ik.h"
#include <algorithm>
#include <cmath>
namespace t170::motion {
bool differential_step(const IKJoints &q,const Eigen::Matrix4d &target,const IKJoints &previous,
    double dt,const IKPolicy &p,const DifferentialLimits &limits,const IKForward &fk,
    IKJoints &next,DifferentialReport &report) {
    report={};report.reason="invalid_dynamic_input";
    if(!q.allFinite() || !previous.allFinite() || !target.allFinite() || !fk || !p.jacobian ||
       !std::isfinite(dt) || dt<=0 || dt>.15 || !std::isfinite(limits.velocity) || !std::isfinite(limits.acceleration) ||
       !(limits.velocity>0) || !(limits.acceleration>0))return false;
    const auto current=fk(q);auto j=p.jacobian(q);
    if(!current.allFinite() || !j.allFinite())return false;
    const Eigen::AngleAxisd rotation(target.topLeftCorner<3,3>()*current.topLeftCorner<3,3>().transpose());
    Eigen::Matrix<double,6,1> desired;desired.head<3>()=(target.topRightCorner<3,1>()-current.topRightCorner<3,1>())/dt;
    desired.tail<3>()=.15*rotation.angle()*rotation.axis()/dt;j.bottomRows<3>()*=.15;
    IKJoints low,high;
    for(int k=0;k<7;++k) {
        const double below=q(k)-p.minimum(k),above=p.maximum(k)-q(k),a=limits.acceleration;
        if(below<-1e-8 || above<-1e-8){report.reason="command_outside_effective_limits";return false;}
        // Include this integration tick in the stopping-distance budget.
        const auto brake=[&](double distance){return std::sqrt(a*a*dt*dt+2*a*std::max(0.,distance))-a*dt;};
        low(k)=std::max({-limits.velocity,previous(k)-a*dt,-std::max(0.,below)/dt,-brake(below)});
        high(k)=std::min({limits.velocity,previous(k)+a*dt,std::max(0.,above)/dt,brake(above)});
        if(low(k)>high(k)+1e-10){report.reason="dynamic_bounds_infeasible";return false;}
        if(low(k)>high(k))low(k)=high(k);
    }
    Eigen::Matrix<double,6,6> jj=j*j.transpose();jj.diagonal().array()+=1e-8;
    const auto n=(Eigen::Matrix<double,7,7>::Identity()-j.transpose()*jj.ldlt().solve(j)).eval();
    Eigen::Matrix<double,7,1> posture=-n*posture_gradient(q,p);
    const double secondary_speed=.20*std::min(1.,desired.norm()/.01);
    if(posture.cwiseAbs().maxCoeff()>secondary_speed && posture.cwiseAbs().maxCoeff()>0)
        posture*=secondary_speed/posture.cwiseAbs().maxCoeff();
    constexpr double regularization=1e-4;
    Eigen::Matrix<double,7,7> h=j.transpose()*j;h.diagonal().array()+=regularization;
    const Eigen::Matrix<double,7,1> g=-j.transpose()*desired-regularization*posture;
    IKJoints v;
    if(!solve_box_qp(h,g,low,high,v)){report.reason="dynamic_qp_failed";return false;}
    // One Cartesian safety-plane inequality, solved through its Lagrange
    // multiplier while retaining all seven joint box constraints.
    const auto normal=j.row(2).eval();
    double required=(limits.floor_z-current(2,3))/dt;
    if(std::isfinite(required)) {
        const double a=std::max(1e-6,limits.acceleration*normal.cwiseAbs().sum());
        const double distance=std::max(0.,current(2,3)-limits.floor_z);
        required=std::max(required,-std::sqrt(a*a*dt*dt+2*a*distance)+a*dt);
    }
    if(std::isfinite(required) && normal.dot(v)<required-1e-10) {
        double maximum=0;for(int k=0;k<7;++k)maximum+=normal(k)*(normal(k)>=0?high(k):low(k));
        if(maximum<required-1e-9){report.reason="floor_braking_infeasible";return false;}
        double lo=0,hi=1;
        bool found=false;
        for(int i=0;i<30;++i,hi*=2) {
            if(!solve_box_qp(h,g-hi*normal.transpose(),low,high,v))return false;
            if(normal.dot(v)>=required-1e-10){found=true;break;}
        }
        if(!found){report.reason="floor_qp_failed";return false;}
        for(int i=0;i<32;++i) {
            const double mid=.5*(lo+hi);IKJoints trial;
            if(!solve_box_qp(h,g-mid*normal.transpose(),low,high,trial))return false;
            if(normal.dot(trial)>=required){hi=mid;v=trial;}else lo=mid;
        }
    }
    const IKJoints result=q+dt*v;const auto actual=fk(result);
    if(!actual.allFinite() || actual(2,3)<limits.floor_z-.00005) {report.reason="nonlinear_floor_residual";return false;}
    report.position_error=(actual.topRightCorner<3,1>()-target.topRightCorner<3,1>()).norm();
    report.rotation_error=Eigen::AngleAxisd(target.topLeftCorner<3,3>()*actual.topLeftCorner<3,3>().transpose()).angle();
    report.velocity_peak=v.cwiseAbs().maxCoeff();report.acceleration_peak=((v-previous)/dt).cwiseAbs().maxCoeff();
    report.reason="dynamic_qp_accepted";next=result;return true;
}
}
