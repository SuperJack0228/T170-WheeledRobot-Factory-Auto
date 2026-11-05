#include "t170/motion/joint_motion.h"
#include <cmath>
#include <future>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace t170::motion;
void require(bool ok,const char *message) {if(!ok) throw std::runtime_error(message);}
void check_limits(const Eigen::MatrixXd &path,double dt,const JointMotionLimits &limits) {
    require(path.rows()>2,"missing timed path");
    JointVector previous=path.row(0),v=JointVector::Zero(),a=v;
    for(int i=0;i<path.rows()+3;++i) {
        JointVector q=path.row(std::min(i,static_cast<int>(path.rows())-1));
        JointVector nv=(q-previous)/dt,na=(nv-v)/dt,nj=(na-a)/dt;
        require(nv.cwiseAbs().maxCoeff()<=limits.velocity*1.00001,"velocity bound");
        require(na.cwiseAbs().maxCoeff()<=limits.acceleration*1.00001,"acceleration bound");
        require(nj.cwiseAbs().maxCoeff()<=limits.jerk*1.00001,"jerk bound including stop");
        previous=q;v=nv;a=na;
    }
}
struct Plant {
    double time=0, response=.03;
    JointVector actual=JointVector::Zero(),command=actual;
    bool feedback=true,writes=true,abort=false,extra_clearance=true;
    std::vector<std::string> events;
    MotionIO io() {
        MotionIO result;
        result.read=[&](JointVector &q){q=actual;return feedback || time<.1;};
        result.send=[&](const JointVector &q){command=q;return writes;};
        result.stop_code=[&]{return abort && time>.1?-4:0;};
        result.now=[&]{return time;};
        result.wait=[&](double dt){time+=dt;actual+=response*(command-actual);};
        result.arrived=[&](const auto &){return extra_clearance;};
        result.observe=[&](const char *event,int,const auto &,const auto *,double){events.emplace_back(event);};
        return result;
    }
};
int main() {
    try {
        const double dt=.005;const JointMotionLimits limits{.9,2.5,20};
        Eigen::MatrixXd knots(4,7);knots.setZero();
        knots.row(1)<<.12,-.10,.08,.05,-.04,.02,0;
        knots.row(2)<<.25,-.18,.13,.11,-.06,.05,.02;
        knots.row(3)<<.4,-.3,.2,.16,-.08,.07,.03;
        const auto path=time_joint_path(knots,dt,limits);
        check_limits(path,dt,limits);
        require((path.row(0)-knots.row(0)).norm()==0,"start mismatch");
        require((path.bottomRows(1)-knots.bottomRows(1)).norm()==0,"end mismatch");
        Eigen::MatrixXd direct(2,7);direct.row(0)=knots.row(0);direct.row(1)=knots.row(3);
        const auto line=time_joint_path(direct,dt,limits);
        check_limits(line,dt,limits);
        for(int i=0;i<line.rows();++i)
            require((line.row(i)-line(i,0)/.4*direct.row(1)).norm()<1e-10,"joint return changed path");
        require(time_joint_path(direct,dt,{0,1,1}).rows()==0,"invalid limits accepted");
        Eigen::MatrixXd still=Eigen::MatrixXd::Zero(2,7);
        require(time_joint_path(still,dt,limits).rows()>=2,"zero-length path");
        const MotionFeedbackConfig cfg{.03,4,.5,3};
        auto hand=[&] {
            Plant plant;
            const int code=run_joint_motion(line,dt,cfg,plant.io());
            require(code==0,"lagged plant failed to arrive");
            require((plant.actual-line.bottomRows(1)).cwiseAbs().maxCoeff()<=cfg.tolerance_rad,"false arrival");
            require(plant.events.back()=="measured_arrival","missing measured event");
            return code;
        };
        auto right=std::async(std::launch::async,hand);
        require(hand()==0 && right.get()==0,"dual hand cross-talk");
        Plant jammed;jammed.response=0;
        require(run_joint_motion(line,dt,cfg,jammed.io())==-8,"jam reported success");
        Plant blind;blind.feedback=false;
        require(run_joint_motion(line,dt,cfg,blind.io())==-32,"feedback loss ignored");
        Plant disconnected;disconnected.writes=false;
        require(run_joint_motion(line,dt,cfg,disconnected.io())==-2,"write failure ignored");
        Plant stopped;stopped.abort=true;
        require(run_joint_motion(line,dt,cfg,stopped.io())==-4,"stop ignored");
        Plant not_clear;not_clear.extra_clearance=false;
        require(run_joint_motion(line,dt,cfg,not_clear.io())==-8,"joint arrival bypassed lift clearance");
        std::cout<<"joint_motion: smooth bounds, exact endpoints, feedback, dual hands and failure cases passed\n";
        return 0;
    } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
