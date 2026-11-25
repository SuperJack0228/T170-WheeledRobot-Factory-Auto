#include "support/reference_arm.h"
#include "t170/motion/joint_motion.h"
#include "t170/motion/cartesian_arrival.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <unistd.h>
using namespace t170::motion;
using Pose=Eigen::Matrix<double,1,6>;
void require(bool b,const std::string &s){if(!b)throw std::runtime_error(s);}
Pose pose(const Eigen::Matrix4d &t) {
    Pose p;p.head<3>()=t.topRightCorner<3,1>().transpose();
    p.tail<3>()<<atan2(t(2,1),t(2,2)),atan2(-t(2,0),hypot(t(0,0),t(1,0))),atan2(t(1,0),t(0,0));return p;
}
void field_limits(ArmDefinition &d) {
    ArmJoints low,high;
    if(d.hand==1) {low<<-179.9087,-179.9544,-179.9087,-45,-179.9087,-59.5876,-89.9544;
        high<<179.9087,-3.0002,179.9087,138.999,179.9087,59.5876,99.9989;}
    else {low<<-179.9087,3.0002,-179.9087,-45,-179.9087,-59.5876,-90;
        high<<179.9087,179.9544,179.9087,138.999,179.9087,59.5876,89.9544;}
    std::lock_guard lock(d.limits->mutex);
    d.limits->bounds={d.minimum.cwiseMax(ArmJoints(low*M_PI/180)),d.maximum.cwiseMin(ArmJoints(high*M_PI/180)),true};
}
std::ofstream summary,trace;
void check_path(Arm &arm,const std::string &name,ArmJoints q,Pose target,bool transit,double speed,double budget) {
    Eigen::MatrixXd path;arm.Cart_Linear_Velocity=speed;
    int code=transit?arm.Approach_Trajectory(Eigen::MatrixXd(q),target,path,speed):arm.Line_Trajectory(Eigen::MatrixXd(q),target,path);
    const auto &d=arm.definition();
    require(code==0 && path.rows()>1,name+" planning failed code="+std::to_string(code));
    const double seconds=(path.rows()-1)*d.period;
    const auto peaks=joint_path_peaks(path,d.period);
    require(seconds<budget,name+" excessive duration="+std::to_string(seconds));
    require(peaks.velocity<=d.velocity*1.00001 && peaks.acceleration<=d.acceleration*1.00001 && peaks.jerk<=d.jerk*1.00001,name+" derivative limits");
    require((path.row(0)-q).norm()<1e-12,name+" changed first reference");
    const auto start=arm.Forward_Kinematics(q),goal=arm_pose_transform(target);
    const Eigen::Vector3d delta=goal.topRightCorner<3,1>()-start.topRightCorner<3,1>();
    const Eigen::Quaterniond ra(start.topLeftCorner<3,3>()),rb(goal.topLeftCorner<3,3>());
    double deviation=0,orientation=0,peak_tcp=0;Eigen::Vector3d previous=start.topRightCorner<3,1>();
    for(int k=0;k<path.rows();++k) {
        require(!arm.Check_Joint_Limit(path.row(k)),name+" effective limit");
        const auto t=arm.Forward_Kinematics(path.row(k));const Eigen::Vector3d p=t.topRightCorner<3,1>();
        peak_tcp=std::max(peak_tcp,(p-previous).norm()/d.period);previous=p;
        if(!transit) {
            const double u=delta.squaredNorm()>1e-14?std::clamp((p-start.topRightCorner<3,1>()).dot(delta)/delta.squaredNorm(),0.,1.):0;
            deviation=std::max(deviation,(p-start.topRightCorner<3,1>()-u*delta).norm());
            orientation=std::max(orientation,Eigen::AngleAxisd(ra.slerp(u,rb).toRotationMatrix()*t.topLeftCorner<3,3>().transpose()).angle());
        }
        if(trace) {trace<<d.model_id<<','<<d.side<<','<<name<<','<<k*d.period;
            for(int j=0;j<7;++j)trace<<','<<path(k,j);
            trace<<','<<p.x()<<','<<p.y()<<','<<p.z()<<'\n';}
    }
    require(peak_tcp<=speed*1.0002,name+" Cartesian speed exceeded");
    require(transit || (deviation<=.001 && orientation<=.01),name+" Cartesian path error");
    const auto final=arm.Forward_Kinematics(path.bottomRows(1));
    const double error=(final.topRightCorner<3,1>()-goal.topRightCorner<3,1>()).norm();
    require(error<=.0005 && Eigen::AngleAxisd(goal.topLeftCorner<3,3>()*final.topLeftCorner<3,3>().transpose()).angle()<=.005,name+" endpoint");
    if(name.find("jog")!=std::string::npos)require(error<=teach_jog_tolerance(delta.norm())*.5,name+" micro jog lost displacement");
    std::cout<<d.model_id<<' '<<d.side<<' '<<name<<" seconds="<<seconds<<" peak_v/a/j="<<peaks.velocity<<'/'<<peaks.acceleration<<'/'<<peaks.jerk<<" xyz_deviation_m="<<deviation<<" endpoint_m="<<error<<'\n';
    if(summary)summary<<d.model_id<<','<<d.side<<','<<name<<','<<seconds<<','<<peaks.velocity<<','<<peaks.acceleration<<','<<peaks.jerk<<','<<peak_tcp<<','<<deviation<<','<<error<<'\n';
}
void feedback_arrival_test(Arm &arm,const ArmJoints &seed) {
    Pose goal=pose(arm.Forward_Kinematics(seed));goal(0)+=.005;
    Eigen::MatrixXd path;arm.Cart_Linear_Velocity=.04;
    require(arm.Line_Trajectory(Eigen::MatrixXd(seed),goal,path)==0,"feedback regression path");
    auto run=[&](bool jammed,bool tcp_gate) {
        ArmJoints actual=seed,command=seed;double now=0;
        CartesianArrivalGate gate(arm_pose_transform(goal),teach_jog_tolerance(.005));
        MotionIO io;io.now=[&]{return now;};io.stop_code=[]{return 0;};
        io.read=[&](ArmJoints &q){q=actual;return true;};io.send=[&](const ArmJoints &q){command=q;return true;};
        io.wait=[&](double dt){now+=dt;if(!jammed)actual+=(1-std::exp(-dt/.04))*(command-actual);};
        if(tcp_gate)io.arrived=[&](const ArmJoints &q){return gate(arm.Forward_Kinematics(q));};
        const int code=run_joint_motion(path,.005,{.12,1,.5,3},io);
        if(!jammed && code==0)require((arm.Forward_Kinematics(actual).topRightCorner<3,1>()-goal.head<3>().transpose()).norm()<.00075,"simulated actual arrival");
        return code;
    };
    require(run(true,false)==0,"test no longer reproduces coarse joint arrival defect");
    require(run(true,true)==-8,"TCP gate falsely accepted jammed micro jog");
    require(run(false,true)==0,"lagged motion did not reach TCP gate");
}
void arrival_tests() {
    auto jog_goal=Eigen::Matrix4d::Identity().eval();jog_goal(0,3)=.002;
    CartesianArrivalGate jog(jog_goal,teach_jog_tolerance(.002),.03,.00025,.003);
    auto no_op=Eigen::Matrix4d::Identity().eval();
    require(!jog(no_op)&&!jog(no_op),"2mm jog no-op accepted");
    auto near=jog_goal;near(0,3)=.0014;
    require(!jog(near),"moving jog accepted before stability");
    near(0,3)=.0015;
    require(jog(near),"completed 2mm jog rejected by submillimetre feedback jitter");
    auto reversed=no_op;reversed(0,3)=-.002;
    require(!jog(reversed)&&!jog(reversed),"reverse jog accepted");
    auto missing=jog_goal;missing(0,3)=NAN;
    require(!jog(missing),"invalid jog feedback accepted");
    auto target=Eigen::Matrix4d::Identity().eval();target(0,3)=.005;
    CartesianArrivalGate gate(target,teach_jog_tolerance(.005));
    auto missed=target;missed(0,3)=.0007;
    require(!gate(missed) && !gate(missed),"4.3 mm residual falsely arrived");
    require(!gate(target),"moving sample falsely stable");
    require(gate(target),"stationary in-tolerance sample rejected");
    auto wrong_rotation=target;wrong_rotation.topLeftCorner<3,3>()=Eigen::AngleAxisd(.03,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    require(!gate(wrong_rotation),"rotation error accepted");
    target(0,3)=.0002;CartesianArrivalGate fine(target,teach_jog_tolerance(.0002));
    auto original=Eigen::Matrix4d::Identity().eval();require(!fine(original)&&!fine(original),"0.2 mm jog no-op accepted");
    require(!fine(target)&&fine(target),"0.2 mm jog actual arrival rejected");
    auto coarse_goal=Eigen::Matrix4d::Identity().eval();coarse_goal(0,3)=.2;
    CartesianArrivalGate coarse(coarse_goal,.05,.0873);
    for(double x:{.16,.165,.17}) {
        auto moving=coarse_goal;moving(0,3)=x;
        require(!coarse(moving),"moving arm accepted merely because coarse pose band was reached");
    }
    auto settled=coarse_goal;settled(0,3)=.17;
    require(coarse(settled),"coarse preparation stability gate became too strict");
    auto invalid=target;invalid(0,3)=NAN;require(!fine(invalid),"nonfinite feedback accepted");
}
int main(int argc,char **argv) {try {
    require(argc>=3,"runtime and repository required");arrival_tests();
    if(argc>3) {std::filesystem::create_directories(argv[3]);summary.open(std::filesystem::path(argv[3])/"timing.csv");
        trace.open(std::filesystem::path(argv[3])/"joint_paths.csv");
        summary<<"model,hand,case,nominal_s,peak_v_rad_s,peak_a_rad_s2,peak_j_rad_s3,peak_tcp_m_s,line_deviation_m,endpoint_error_m\n";
        summary<<std::setprecision(12);trace<<std::setprecision(17);
        trace<<"model,hand,case,time_s,j1_rad,j2_rad,j3_rad,j4_rad,j5_rad,j6_rad,j7_rad,tcp_x_m,tcp_y_m,tcp_z_m\n";}
    const std::filesystem::path runtime=argv[1],root=argv[2];
    for(const std::string id:{"t170c_v2_3kg","t170c_v2_5kg"}) {
        const auto temp=std::filesystem::temp_directory_path()/("t170-timing-"+id+"-"+std::to_string(getpid()));
        std::filesystem::create_directories(temp);
        auto model=YAML::LoadFile((root/"config/robot_models"/id/"Robot_Model.yaml").string());
        auto actual=YAML::LoadFile((runtime/"config/Robot_Model.yaml").string());
        for(const char *side:{"right","left"})model["T7_170"][side]["tool"]=id=="t170c_v2_3kg"?YAML::Clone(actual["T7_170"][side]["tool"]):YAML::Load("[0.26,0,0,0,0,0]");
        auto binding=YAML::LoadFile((runtime/"config/arm_binding.yaml").string());binding["model_id"]=id;
        binding["motion_policy"]=YAML::LoadFile((root/"config/robot_models"/id/"motion_policy.yaml").string());
        {std::ofstream f(temp/"Robot_Model.yaml");f<<model;}{std::ofstream f(temp/"arm_binding.yaml");f<<binding;}
        for(const char *side:{"right","left"}) {
            auto d=load_arm_definition((temp/"Robot_Model.yaml").string(),side);field_limits(d);
            Arm arm(d,std::make_shared<ReferenceBackend>(d));
            Pose ready;ready<<.45,d.hand?-.30:.30,-.25,0,55*M_PI/180,(d.hand?40:-40)*M_PI/180;
            check_path(arm,"home_to_ready",d.home,ready,true,.4,8);
            Eigen::RowVectorXd q=d.home;require(arm.Inverse_Kinematics(ready,q)==0,"ready inverse");
            feedback_arrival_test(arm,q);
            Pose lift=pose(arm.Forward_Kinematics(q));lift(2)+=.05;
            check_path(arm,"ready_lift",q,lift,false,.2,6);
            for(double mm:{.2,2.,5.}) for(int axis:{0,1}) {
                Pose jog=pose(arm.Forward_Kinematics(q));jog(axis)+=mm*.001;
                check_path(arm,"jog_"+std::to_string(mm)+"_axis"+std::to_string(axis),q,jog,false,.04,3);
            }
            if(id=="t170c_v2_3kg") {
                // Rounded MODEL-angle snapshots and TCP goals from startup.log,
                // session.3xwUhf8C; no motor/device angles are used as IK seeds.
                ArmJoints start;Pose goal;
                if(d.hand) {
                    start<<52.8,-38.3,-70.9,68.7,49.2,-38,22;start*=M_PI/180;
                    goal=pose(arm.Forward_Kinematics(start));goal(2)=-.25;
                    check_path(arm,"field_right_lift",start,goal,false,.4,5);
                    start<<71.2,-46.2,-69.4,84.5,62.3,-59.5,20.4;start*=M_PI/180;
                    goal=pose(arm.Forward_Kinematics(start));goal(0)=.45;goal(1)=-.3;
                    check_path(arm,"field_right_xy",start,goal,true,.4,6);
                    Pose calibration_ready;calibration_ready<<.45,-.3,-.25,5*M_PI/180,55*M_PI/180,50*M_PI/180;
                    Eigen::RowVectorXd cq=d.home;
                    require(arm.Inverse_Kinematics(calibration_ready,cq)==0,"calibration ready inverse");
                    Pose high;high<<.4323,-.1265,-.2983,5*M_PI/180,55*M_PI/180,50*M_PI/180;
                    check_path(arm,"calibration_approach",cq,high,true,.35,6);
                    require(arm.Inverse_Kinematics(high,cq)==0,"calibration hover inverse");
                    Pose dpoint=high;dpoint(2)=-.4083;
                    check_path(arm,"calibration_descent",cq,dpoint,false,.18,6);
                    require(arm.Inverse_Kinematics(dpoint,cq)==0,"calibration D inverse");
                    feedback_arrival_test(arm,cq);
                    for(double mm:{.2,5.}) for(int axis:{0,1}) for(double sign:{-1.,1.}) {
                        auto jog=pose(arm.Forward_Kinematics(cq));jog(axis)+=sign*mm*.001;
                        check_path(arm,"field_D_jog_"+std::to_string(sign*mm)+"_axis"+std::to_string(axis),cq,jog,false,.04,3);
                    }
                    start<<25.9,-52.3,-3.6,83.7,7.2,-46.3,68.1;
                    goal<<.474458666293,-.041357897538,-.293264690538,5*M_PI/180,55*M_PI/180,50*M_PI/180;
                } else {start<<25.9,53,4.4,83.7,-8.7,-46,-67.3;
                    goal<<.484826201255,.173403118252,-.293264690538,5*M_PI/180,55*M_PI/180,-50*M_PI/180;}
                start*=M_PI/180;
                for(double offset:{-.1,0.,.1}) {
                    auto perturbed=start;perturbed(0)+=offset*M_PI/180;
                    check_path(arm,"field_approach_"+std::to_string(offset),perturbed,goal,true,.4,6);
                }
                check_path(arm,"field_approach_strict_line",start,goal,false,.4,8);
            }
        }
        std::filesystem::remove_all(temp);
    }
    return 0;
} catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}}
