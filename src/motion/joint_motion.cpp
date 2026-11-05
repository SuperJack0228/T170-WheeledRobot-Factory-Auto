#include "t170/motion/joint_motion.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace t170::motion {
namespace {
bool positive(double x) { return std::isfinite(x) && x > 0; }
}
Eigen::MatrixXd time_joint_path(const Eigen::MatrixXd &points, double dt,
                               const JointMotionLimits &limit, double minimum_duration) {
    if(points.rows()<2 || points.cols()!=7 || !points.allFinite() || !positive(dt) ||
       !positive(limit.velocity) || !positive(limit.acceleration) || !positive(limit.jerk) ||
       !std::isfinite(minimum_duration) || minimum_duration<0) return {};
    std::vector<JointVector> q{points.row(0)};
    std::vector<double> s{0};
    double maximum_joint_length=0;
    for(int i=1;i<points.rows();++i) {
        const double length=(points.row(i)-q.back()).norm();
        if(length<1e-10) continue;
        maximum_joint_length+=(points.row(i)-q.back()).cwiseAbs().maxCoeff();
        s.push_back(s.back()+length); q.push_back(points.row(i));
    }
    if(q.size()==1) return Eigen::MatrixXd::Constant(2,7,0).rowwise()+q.front();
    const int n=static_cast<int>(q.size());
    const double length=s.back();
    for(auto &value:s) value/=length;
    // Natural spline second derivatives. Positive tridiagonal system.
    std::vector<double> diag(n,1), upper(n,0), lower(n,0);
    std::vector<JointVector> second(n,JointVector::Zero());
    for(int i=1;i<n-1;++i) {
        const double a=s[i]-s[i-1], b=s[i+1]-s[i];
        lower[i]=a; diag[i]=2*(a+b); upper[i]=b;
        second[i]=6*((q[i+1]-q[i])/b-(q[i]-q[i-1])/a);
    }
    for(int i=1;i<n;++i) {
        const double factor=lower[i]/diag[i-1];
        diag[i]-=factor*upper[i-1]; second[i]-=factor*second[i-1];
    }
    second.back()/=diag.back();
    for(int i=n-2;i>=0;--i) second[i]=(second[i]-upper[i]*second[i+1])/diag[i];
    double duration=std::max({0.25,minimum_duration,1.9*maximum_joint_length/limit.velocity});
    for(int attempt=0;attempt<20 && duration<=60;++attempt) {
        const int ticks=std::max(2,static_cast<int>(std::ceil(duration/dt)));
        if(ticks>200000) return {};
        duration=ticks*dt;
        Eigen::MatrixXd out(ticks+1,7);
        int interval=0;
        for(int k=0;k<=ticks;++k) {
            const double t=static_cast<double>(k)/ticks;
            const double u=t*t*t*(10+t*(-15+6*t));
            while(interval<n-2 && u>s[interval+1]) ++interval;
            const double h=s[interval+1]-s[interval];
            const double a=std::clamp((s[interval+1]-u)/h,0.0,1.0), b=1-a;
            out.row(k)=a*q[interval]+b*q[interval+1]+
                ((a*a*a-a)*second[interval]+(b*b*b-b)*second[interval+1])*(h*h/6);
        }
        out.row(0)=q.front(); out.bottomRows(1)=q.back();
        JointVector previous=q.front(), v=JointVector::Zero(), a=v;
        double pv=0,pa=0,pj=0;
        for(int k=0;k<out.rows()+3;++k) {
            const JointVector current=out.row(std::min(k,static_cast<int>(out.rows())-1));
            const JointVector next_v=(current-previous)/dt, next_a=(next_v-v)/dt;
            pv=std::max(pv,next_v.cwiseAbs().maxCoeff());
            pa=std::max(pa,next_a.cwiseAbs().maxCoeff());
            pj=std::max(pj,((next_a-a)/dt).cwiseAbs().maxCoeff());
            previous=current; v=next_v; a=next_a;
        }
        const double scale=std::max({pv/limit.velocity,std::sqrt(pa/limit.acceleration),
                                     std::cbrt(pj/limit.jerk)});
        if(scale<=1.000001) return out;
        duration*=std::max(1.05,1.03*scale);
    }
    return {};
}

int run_joint_motion(const Eigen::MatrixXd &path,double dt,const MotionFeedbackConfig &cfg,const MotionIO &io) {
    if(path.rows()<2 || path.cols()!=7 || !path.allFinite() || !positive(dt) ||
       !positive(cfg.tolerance_rad) || !positive(cfg.settle_timeout_s) ||
       !positive(cfg.feedback_timeout_s) || cfg.stable_samples<1 ||
       !io.read || !io.send || !io.stop_code || !io.now || !io.wait) return -10;
    JointVector actual;
    auto emit=[&](const char *event,int row,const JointVector &command,const JointVector *measured,double lag) {
        if(io.observe) io.observe(event,row,command,measured,lag);
    };
    if(!io.read(actual) || !actual.allFinite()) {
        emit("initial_feedback_failed",0,path.row(0),nullptr,0);return -32;
    }
    double last_feedback=io.now(),next_read=last_feedback;
    emit("start",0,path.row(0),&actual,(actual-path.row(0)).cwiseAbs().maxCoeff());
    int stable=0;
    double deadline=0;
    for(int row=0;;) {
        const int stopped=io.stop_code();
        if(stopped) {emit("abort",std::min(row,static_cast<int>(path.rows())-1),
                        path.row(std::min(row,static_cast<int>(path.rows())-1)),nullptr,0);return stopped;}
        const double tick=io.now();
        const int index=std::min(row,static_cast<int>(path.rows())-1);
        const JointVector command=path.row(index);
        if(!io.send(command)) { emit("write_failed",index,command,nullptr,0); return -2; }
        if(tick>=next_read) {
            next_read=tick+0.05;
            if(io.read(actual) && actual.allFinite()) {
                last_feedback=io.now();
                const double lag=(actual-command).cwiseAbs().maxCoeff();
                emit("feedback",index,command,&actual,lag);
                if(row>=path.rows()) {
                    stable=lag<=cfg.tolerance_rad && (!io.arrived || io.arrived(actual)) ? stable+1 : 0;
                    if(stable>=cfg.stable_samples) {
                        emit("measured_arrival",index,command,&actual,lag); return 0;
                    }
                }
            } else stable=0;
        }
        if(io.now()-last_feedback>cfg.feedback_timeout_s) {
            emit("feedback_timeout",index,command,nullptr,0); return -32;
        }
        if(row==path.rows()-1) {
            emit("command_complete",index,command,nullptr,0);
            deadline=io.now()+cfg.settle_timeout_s;
        }
        if(row>=path.rows() && io.now()>deadline) {
            emit("arrival_timeout",index,command,&actual,(actual-command).cwiseAbs().maxCoeff()); return -8;
        }
        // No catch-up bursts after a slow hardware call. Keep the current target alive.
        io.wait(std::max(0.0,dt-(io.now()-tick)));
        if(row<path.rows()) ++row;
    }
}
} // namespace t170::motion
