#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>

namespace wrist_servo {
// Limits apply to connectors, not to the SDK's already planned cruise segments.
struct ConnectorLimits {
    double speed = 1.5;       // rad/s
    double acceleration = 6; // rad/s^2
    double jerk = 80;        // rad/s^3
    double max_duration = 0.40;
};
struct JointBoundary {
    Eigen::RowVectorXd q, v, a;
};
inline JointBoundary trajectory_boundary(const Eigen::MatrixXd &q, int row, double dt, bool outgoing) {
    JointBoundary b{q.row(row),Eigen::RowVectorXd::Zero(q.cols()),Eigen::RowVectorXd::Zero(q.cols())};
    if(outgoing && row+1<q.rows()) {
        b.v=(q.row(row+1)-q.row(row))/dt;
        if(row+2<q.rows()) {
            b.v=(-3*q.row(row)+4*q.row(row+1)-q.row(row+2))/(2*dt);
            b.a=(q.row(row+2)-2*q.row(row+1)+q.row(row))/(dt*dt);
        }
        if(row+3<q.rows()) b.a=(2*q.row(row)-5*q.row(row+1)+4*q.row(row+2)-q.row(row+3))/(dt*dt);
    } else if(!outgoing && row>0) {
        b.v=(q.row(row)-q.row(row-1))/dt;
        if(row>1) {
            b.v=(3*q.row(row)-4*q.row(row-1)+q.row(row-2))/(2*dt);
            b.a=(q.row(row)-2*q.row(row-1)+q.row(row-2))/(dt*dt);
        }
        if(row>2) b.a=(2*q.row(row)-5*q.row(row-1)+4*q.row(row-2)-q.row(row-3))/(dt*dt);
    }
    return b;
}

// Quintic Hermite connector: q, dq and ddq match at BOTH ends. The validity
// callback lets the hardware adapter check TCP clearance without linking the SDK here.
inline Eigen::MatrixXd connect_joint_boundaries(
    const JointBoundary &from, const JointBoundary &to, double dt, const ConnectorLimits &limits,
    const std::function<bool(const Eigen::RowVectorXd &)> &valid = {}) {
    const auto n=from.q.size();
    if(n==0 || to.q.size()!=n || from.v.size()!=n || from.a.size()!=n || to.v.size()!=n || to.a.size()!=n ||
       !from.q.allFinite() || !to.q.allFinite() || !from.v.allFinite() || !to.v.allFinite() ||
       !from.a.allFinite() || !to.a.allFinite() || !std::isfinite(dt) || dt<=0 ||
       !std::isfinite(limits.speed) || limits.speed<=0 || !std::isfinite(limits.acceleration) || limits.acceleration<=0 ||
       !std::isfinite(limits.jerk) || limits.jerk<=0 || !std::isfinite(limits.max_duration) || limits.max_duration<dt)
        return {};
    if(std::max(from.v.cwiseAbs().maxCoeff(),to.v.cwiseAbs().maxCoeff())>limits.speed ||
       std::max(from.a.cwiseAbs().maxCoeff(),to.a.cwiseAbs().maxCoeff())>limits.acceleration) return {};
    const int maximum=static_cast<int>(std::floor(limits.max_duration/dt));
    // Search in sample-time increments. No change to the caller's playback period.
    for(int steps=std::max(2,static_cast<int>(std::ceil(.02/dt)));steps<=maximum;steps+=2) {
        const double T=steps*dt;
        const Eigen::RowVectorXd c0=from.q, c1=T*from.v, c2=.5*T*T*from.a;
        const Eigen::RowVectorXd D=to.q-c0-c1-c2, V=T*to.v-c1-2*c2, A=T*T*to.a-2*c2;
        const Eigen::RowVectorXd c3=10*D-4*V+.5*A, c4=-15*D+7*V-A, c5=6*D-3*V+.5*A;
        bool acceptable=true;
        // Check derivatives between command samples too, rather than only positions.
        for(int k=0;k<=steps*4;++k) {
            const double u=double(k)/(steps*4);
            const Eigen::RowVectorXd v=(c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/T;
            const Eigen::RowVectorXd a=(2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(T*T);
            const Eigen::RowVectorXd j=(6*c3+u*(24*c4+u*60*c5))/(T*T*T);
            if(v.cwiseAbs().maxCoeff()>limits.speed*.98 || a.cwiseAbs().maxCoeff()>limits.acceleration*.98 ||
               j.cwiseAbs().maxCoeff()>limits.jerk*.98) { acceptable=false; break; }
        }
        if(!acceptable) continue;
        Eigen::MatrixXd points(steps+1,n);
        for(int k=0;k<=steps;++k) {
            const double u=double(k)/steps;
            points.row(k)=c0+u*(c1+u*(c2+u*(c3+u*(c4+u*c5))));
            if(valid && !valid(points.row(k))) { acceptable=false; break; }
        }
        if(acceptable) { points.row(0)=from.q; points.row(steps)=to.q; return points; }
    }
    return {};
}

// Start at the held reference even for a sub-degree discrepancy. A stale large
// mismatch is rejected instead of moving a gripped object back to an old pose.
inline Eigen::MatrixXd connect_held_start(const Eigen::MatrixXd &path, const Eigen::RowVectorXd &held,
                                        double dt, const ConnectorLimits &limits,
                                        const std::function<bool(const Eigen::RowVectorXd &)> &valid = {}) {
    if(path.rows()<2 || path.cols()!=held.size() || !path.allFinite() || !held.allFinite() ||
       !std::isfinite(dt) || dt<=0 || !std::isfinite(limits.speed) || limits.speed<=0 ||
       !std::isfinite(limits.acceleration) || limits.acceleration<=0 ||
       !std::isfinite(limits.jerk) || limits.jerk<=0 ||
       !std::isfinite(limits.max_duration) || limits.max_duration<dt) return {};
    const double gap=(path.row(0)-held).cwiseAbs().maxCoeff();
    if(gap>.05) return {};
    const JointBoundary from{held,Eigen::RowVectorXd::Zero(held.size()),Eigen::RowVectorXd::Zero(held.size())};
    const auto initial=trajectory_boundary(path,0,dt,true);
    // Do not reject/retime an SDK-native acceleration profile that already
    // starts at the held position with effectively zero velocity.
    if(gap<1e-9 && initial.v.cwiseAbs().maxCoeff()<.02) {
        Eigen::MatrixXd out=path; out.row(0)=held; return out;
    }
    // A forced row zero can hide an IK change in row one. Skip only a short
    // startup prefix, then join the real SDK branch with zero initial v/a.
    const int last=std::min(static_cast<int>(path.rows())-2,static_cast<int>(std::ceil(.10/dt)));
    for(int row=0;row<=last;++row) {
        if((path.row(row)-held).cwiseAbs().maxCoeff()>.05) return {};
        const auto bridge=connect_joint_boundaries(from,trajectory_boundary(path,row,dt,true),dt,limits,valid);
        if(bridge.rows()==0) continue;
        const int tail=static_cast<int>(path.rows())-row-1;
        Eigen::MatrixXd out(bridge.rows()+tail,path.cols());
        out.topRows(bridge.rows())=bridge;
        out.bottomRows(tail)=path.bottomRows(tail);
        return out;
    }
    return {};
}
} // namespace wrist_servo
