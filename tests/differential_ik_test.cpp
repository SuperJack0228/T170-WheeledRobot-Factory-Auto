#include "t170/motion/differential_ik.h"
#include "t170/motion/feedback_filter.h"
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <vector>
#include <filesystem>
using namespace t170::motion;
void check(bool b,const char *m){if(!b)throw std::runtime_error(m);}
int main(int argc,char **argv){try {
    JointFeedbackFilter filter;IKJoints low=IKJoints::Constant(-3.14),high=-low,q;
    q<<.45,-.09,-.19,1.68,-1.06,-.89,1.70;
    check(filter.accept(q,1,low,high,.9),"initial feedback");
    auto corrupt=q;corrupt.tail<4>().setZero();
    check(!filter.accept(corrupt,1.01,low,high,.9),"four zero joints entered feedback history");
    check(filter.accept(q,1.013,low,high,.9),"valid frame after dropout rejected");
    JointFeedbackFilter zero;check(zero.accept(IKJoints::Zero(),1,low,high,.9),"legitimate zero rejected");
    auto nan=q;nan(0)=NAN;check(!filter.accept(nan,1.02,low,high,.9),"nonfinite feedback accepted");
    if(argc>1) {
        const auto path=std::filesystem::path(argv[1])/"field_logs/20261123_1411_session.FVo4L83G/logs/visual_servo/servo_1795414356475500_20333.csv";
        std::ifstream input(path);check(bool(input),"field feedback fixture missing");std::string line;
        auto split=[](const std::string &s){std::vector<std::string> a;std::istringstream stream(s);std::string v;while(std::getline(stream,v,','))a.push_back(v);return a;};
        std::getline(input,line);const auto columns=split(line);
        auto col=[&](const std::string &name){auto it=std::find(columns.begin(),columns.end(),name);check(it!=columns.end(),"CSV column missing");return size_t(it-columns.begin());};
        JointFeedbackFilter replay;int rejected=0,accepted=0;
        while(std::getline(input,line)) {
            const auto row=split(line);
            if(row.at(col("session"))!="2" || row.at(col("hand"))!="1")continue;
            const auto event=row.at(col("event"));if(event!="handoff" && event!="control" && event!="end" && event!="feedback_step")continue;
            IKJoints measured;for(int j=0;j<7;++j)measured(j)=std::stod(row.at(col("actual_q"+std::to_string(j))));
            const bool good=replay.accept(measured,std::stod(row.at(col("steady_s"))),low,high,.9);
            const bool known_corruption=measured.tail<4>().isZero(0);
            check(good!=known_corruption,"field feedback validity regression");if(good)++accepted;else ++rejected;
        }
        check(rejected==2 && accepted>200,"field outlier/recovery replay incomplete");
        std::cout<<"Field CSV replay accepted="<<accepted<<" rejected="<<rejected<<" (same corrupt frame logged twice)\n";
    }
    IKPolicy policy;policy.minimum=IKJoints::Constant(-.5);policy.maximum=IKJoints::Constant(.5);
    policy.elbow_gain=policy.posture_gain=policy.limit_gain=policy.wrist_gain=0;
    policy.jacobian=[](const IKJoints &){IKJacobian j=IKJacobian::Zero();j.leftCols<6>().setIdentity();return j;};
    const auto fk=[](const IKJoints &v){Eigen::Matrix4d t=Eigen::Matrix4d::Identity();t.topRightCorner<3,1>()=v.head<3>().transpose();
        t.topLeftCorner<3,3>()=(Eigen::AngleAxisd(v(3),Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(v(4),Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(v(5),Eigen::Vector3d::UnitZ())).toRotationMatrix();return t;};
    q.setZero();IKJoints previous=IKJoints::Zero();DifferentialReport report;
    for(int k=0;k<300;++k) {
        const double dt=k%2?.009:.011;auto target=fk(q);target(0,3)=.7;
        IKJoints next;check(differential_step(q,target,previous,dt,policy,{.45,2,-INFINITY},fk,next,report),"bounded motion failed");
        auto velocity=((next-q)/dt).eval();
        check(velocity.cwiseAbs().maxCoeff()<=.45000001,"velocity bound");
        check(((velocity-previous)/dt).cwiseAbs().maxCoeff()<=2.0000001,"acceleration bound");
        check(next(0)<=.50000001,"joint limit crossed");q=next;previous=velocity;
    }
    check(q(0)>.49,"dynamics stalled below joint endpoint");
    q.setZero();q(2)=.05;previous.setZero();
    for(int k=0;k<200;++k) {
        auto target=fk(q);target(2,3)=-.1;IKJoints next;
        check(differential_step(q,target,previous,.01,policy,{.3,2,0},fk,next,report),"floor QP infeasible");
        previous=(next-q)/.01;q=next;check(q(2)>=-1e-8,"Cartesian floor crossed");
    }
    check(q(2)<.001,"floor was not approached");
    std::cout<<"Differential QP: variable dt, position/speed/acceleration/braking bounds, Cartesian floor and bad feedback passed\n";
    return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
