#include "t170/motion/arm.h"
#include <chrono>
#include <iostream>
#include <iomanip>
using t170::motion::ArmJoints;
using Pose=Eigen::Matrix<double,1,6>;
struct ReferenceBackend final : t170::motion::ArmBackend {
    t170::motion::ArmDefinition d;
    explicit ReferenceBackend(t170::motion::ArmDefinition v):d(std::move(v)){}
    Eigen::Matrix4d flange(const ArmJoints &q) const override {
        Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
        for(int i=0;i<7;++i) {
            const double al=d.mdh(i,0),a=d.mdh(i,1),di=d.mdh(i,2),th=q(i)+d.mdh(i,3);
            Eigen::Matrix4d m;
            m<<cos(th),-sin(th),0,a,sin(th)*cos(al),cos(th)*cos(al),-sin(al),-sin(al)*di,
               sin(th)*sin(al),cos(th)*sin(al),cos(al),cos(al)*di,0,0,0,1;
            t*=m;
        }return t;
    }
    Eigen::Matrix4d forward(const ArmJoints &q) const override {return flange(q)*t170::motion::arm_pose_transform(d.tool);}
    // Deliberately unavailable: native Linux SDK cannot run on macOS.
    int inverse(const Eigen::Matrix4d &,ArmJoints &,std::optional<double>) override {return -99;}
};
Pose pose(const Eigen::Matrix4d &t) {
    Pose p;p.head<3>()=t.topRightCorner<3,1>().transpose();
    p(4)=atan2(-t(2,0),hypot(t(0,0),t(1,0)));p(3)=atan2(t(2,1),t(2,2));p(5)=atan2(t(1,0),t(0,0));return p;
}
void replay(const char *file,const char *id,const char *side,ArmJoints degrees,Pose goal,bool relative=false,bool endpoint_only=false) {
    auto d=t170::motion::load_arm_definition(file,side);
    Arm arm(d,std::make_shared<ReferenceBackend>(d));
    ArmJoints seed=degrees*M_PI/180;
    const auto start=arm.Forward_Kinematics(seed);auto initial=pose(start);
    if(relative) {initial.head<3>()=goal.head<3>();goal=initial;}
    const auto begin=std::chrono::steady_clock::now();
    Eigen::MatrixXd trajectory;arm.Cart_Linear_Velocity=.2;
    const int rc=endpoint_only?0:arm.Line_Trajectory(seed,goal,trajectory);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    std::cout<<"CASE "<<id<<" side="<<side<<" endpoint_only="<<endpoint_only;
    if(!endpoint_only)std::cout<<" line_rc="<<rc<<" rows="<<trajectory.rows()<<" ms="<<ms;
    std::cout<<" start_pose="<<pose(start)<<" goal="<<goal<<'\n';
    if(trajectory.rows())std::cout<<"END q_deg="<<trajectory.bottomRows(1)*180/M_PI<<" duration_s="<<(trajectory.rows()-1)*d.period<<'\n';
    const auto target=t170::motion::arm_pose_transform(goal);
    const Eigen::Quaterniond ra(start.topLeftCorner<3,3>()),rb(target.topLeftCorner<3,3>());
    const double distance=(start.topRightCorner<3,1>()-target.topRightCorner<3,1>()).norm();
    const double angle=Eigen::AngleAxisd(ra.toRotationMatrix()*rb.toRotationMatrix().transpose()).angle();
    const int count=std::max(1,int(std::ceil(std::max(distance/.002,angle/.01))));
    Eigen::RowVectorXd q=seed;
    for(int i=1;!endpoint_only && i<=count;++i) {
        const double u=double(i)/count;Eigen::Matrix4d intermediate=Eigen::Matrix4d::Identity();
        intermediate.topRightCorner<3,1>()=(1-u)*start.topRightCorner<3,1>()+u*target.topRightCorner<3,1>();
        intermediate.topLeftCorner<3,3>()=ra.slerp(u,rb).toRotationMatrix();
        auto p=pose(intermediate);const int code=arm.Inverse_Kinematics_Numeric(p,q);
        if(code) {
            const ArmJoints margin=((q-d.minimum).array().min((d.maximum-q).array())).matrix();
            std::cout<<"FAIL_KNOT "<<i<<'/'<<count<<" code="<<code<<" last_q_deg="<<q*180/M_PI<<" limit_margin_deg="<<margin*180/M_PI<<'\n';break;
        }
    }
    Eigen::RowVectorXd direct=seed;auto p=goal;
    std::cout<<"ENDPOINT_NUMERIC rc="<<arm.Inverse_Kinematics_Numeric(p,direct)<<" q_deg="<<direct*180/M_PI<<'\n';
    // Search deterministic interior seeds for endpoint existence only, not safe paths.
    int solutions=0;
    for(int k=0;k<40;++k) {
        Eigen::RowVectorXd s(7);
        for(int j=0;j<7;++j) {const double u=.1+.8*(.5+.5*sin((k+1)*(j+3)*1.618));s(j)=d.minimum(j)+u*(d.maximum(j)-d.minimum(j));}
        p=goal;
        if(arm.Inverse_Kinematics_Numeric(p,s)==0) {
            if(solutions++==0)std::cout<<"ALTERNATE_ENDPOINT q_deg="<<s*180/M_PI<<'\n';
        }
    }
    std::cout<<"ALTERNATE_ENDPOINT solutions="<<solutions<<"/40 (not a trajectory or hardware validation)\n";
}
int main(int argc,char **argv) {
    if(argc!=2)return 2;
    std::cout<<std::fixed<<std::setprecision(6);
    // The first Home uses endpoint IK plus a JOINT trajectory, not a TCP line.
    replay(argv[1],"first_home_right","right",(ArmJoints()<<-8.8,-8.2,6.3,-6.1,7.7,-3.4,2.1).finished(),(Pose()<<.45,-.35,-.1,0,0,0).finished(),false,true);
    replay(argv[1],"first_home_left","left",(ArmJoints()<<-5.2,3.1,1.4,7.5,.8,-22.2,15.7).finished(),(Pose()<<.45,.35,-.1,0,0,0).finished(),false,true);
    replay(argv[1],"belt_xy_right","right",(ArmJoints()<<56.9,-64,-39.4,82,21.1,-57.4,65.3).finished(),(Pose()<<.45,-.25,-.2031,0,0,0).finished(),true);
    replay(argv[1],"belt_xy_left","left",(ArmJoints()<<52.2,60.6,30.6,82.2,-10.1,-58.8,-73.1).finished(),(Pose()<<.45,.25,-.2030,0,0,0).finished(),true);
    replay(argv[1],"second_home_lift_left","left",(ArmJoints()<<52.9,60.8,30.8,82.2,-10,-59,-73.5).finished(),(Pose()<<.4513,.2992,-.1,0,0,0).finished(),true);
    replay(argv[1],"second_home_z_left","left",(ArmJoints()<<55.1,63.8,32.3,73.1,-9.2,-49.5,-78.1).finished(),(Pose()<<.45,.35,-.1,0,0,0).finished(),true);
    replay(argv[1],"second_home_rotate_right","right",(ArmJoints()<<89,-61,-56.6,57.6,4.3,-45,97.6).finished(),(Pose()<<.4499,-.3501,-.1,0,0,0).finished());
    replay(argv[1],"second_home_rotate_left","left",(ArmJoints()<<81,63,44.8,66.9,3.6,-53.6,-89.9).finished(),(Pose()<<.4713,.3778,-.1,0,0,0).finished());
}
