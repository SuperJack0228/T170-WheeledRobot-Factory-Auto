#include "support/reference_arm.h"
#include "t170/motion/differential_ik.h"
#include "t170/control/visual_servo.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
using namespace t170::motion;
using Pose=Eigen::Matrix<double,1,6>;
void check(bool b,const std::string &m){if(!b)throw std::runtime_error(m);}
void run(Arm &arm,Pose guide,Pose finish,bool grasp,double lag=0,const char *scenario="tray") {
    Eigen::MatrixXd approach;
    Pose ready;ready<<.45,arm.definition().hand?-.3:.3,-.25,0,45*M_PI/180,(arm.definition().hand?40:-40)*M_PI/180;
    Eigen::MatrixXd preparation;
    check(arm.Approach_Trajectory(arm.definition().home,ready,preparation,.4)==0,"preparation failed");
    check((preparation.rows()-1)*.005<8,"preparation too slow");
    check(arm.Approach_Stroke(preparation.bottomRows(1),guide,finish,approach,.4)==0,"whole stroke preflight failed "+arm.definition().side);
    check((approach.rows()-1)*.005<8,"approach too slow");
    ArmJoints q=approach.bottomRows(1),v=ArmJoints::Zero();
    const auto initial=arm.Forward_Kinematics(q);auto state=wrist_servo::State{};
    state.position=initial.topRightCorner<3,1>();state.rotation=initial.topLeftCorner<3,3>();state.joints=q;state.time_s=1;
    wrist_servo::Config cfg;cfg.tray_place.enabled=true;wrist_servo::Controller controller;
    controller.reset(cfg,state,finish(2),.1,finish.head<2>().transpose(),grasp,!grasp);
    if(!grasp)controller.set_observation_z(state.position.z());
    // A held workpiece hides the target in placement. Its neighbour estimator's
    // virtual target enters through exactly the same controller/velocity QP.
    const wrist_servo::Vec2 xy=finish.head<2>().transpose()+wrist_servo::Vec2(.001,arm.definition().hand?.012:-.012);
    unsigned frame=0;double elapsed=0,minimum_margin=100,minimum_outward=100;auto reference=state.position;int failures=0;
    for(int k=0;k<2000;++k) {
        const double dt=k%2?.009:.011;elapsed+=dt;state.time_s=1+elapsed;
        if(k%10==0){wrist_servo::PixelObservation obs;obs.frame=++frame;obs.capture_s=state.time_s-.08;obs.confidence=.99;obs.clock_uncertainty_s=.06;obs.virtual_target=!grasp;
            wrist_servo::Mapping map;map.valid=true;map.remaining_m=xy-state.position.head<2>();std::string reason;
            controller.observe(obs,state,map,state.time_s,reason);}
        auto out=controller.tick(state,dt);
        if(out.phase==wrist_servo::Phase::Done) {
            check((xy-state.position.head<2>()).norm()<.0061,"false XY completion");
            check(state.position.z()>=finish(2)-.001,"false Z completion");
            check(elapsed<10,"abnormally slow closed loop");
            check(minimum_outward>.04,"elbow tucked toward torso during stroke");
            check(minimum_margin>2*M_PI/180,"wrist comfort was lost along the stroke");
            if(grasp) {
                auto lift=finish;lift.head<2>()=state.position.head<2>().transpose();lift(2)=guide(2);
                Eigen::MatrixXd exit;arm.Cart_Linear_Velocity=.25;
                check(arm.Line_Trajectory(Eigen::MatrixXd(q),lift,exit)==0,"post-grasp vertical exit failed");
                check((exit.rows()-1)*.005<8,"vertical exit timing regression");
                Eigen::MatrixXd home;check(arm.Joint_Trajectory(exit.bottomRows(1),arm.definition().home,home)==0,"joint return failed");
            }
            std::cout<<arm.definition().model_id<<' '<<arm.definition().side<<' '<<scenario<<(grasp?" grasp":" place")<<" lag_s="<<lag<<" controller_done_s="<<elapsed<<" approach_s="<<(approach.rows()-1)*.005<<" minimum_outward_mm="<<minimum_outward*1000<<" minimum_wrist_margin_deg="<<minimum_margin*180/M_PI<<'\n';return;
        }
        check(out.phase!=wrist_servo::Phase::Failed,"controller failed "+out.reason);
        auto target=arm_pose_transform(finish);target.topRightCorner<3,1>()=out.reference;
        auto policy=arm.ik_policy(target);DifferentialReport report;ArmJoints next;
        const double floor=finish(2)+(grasp?cfg.grasp_height_margin_m:cfg.place_height_margin_m)-cfg.solver_position_tolerance_m;
        check(differential_step(q,target,v,dt,policy,{cfg.max_joint_speed_rad_s,cfg.max_joint_accel_rad_s2,floor},
            [&](const ArmJoints &j){return arm.Forward_Kinematics(j);},next,report),"dynamic step failed "+report.reason);
        const auto velocity=((next-q)/dt).eval();
        check(velocity.cwiseAbs().maxCoeff()<=.450001,"stroke velocity");
        check(((velocity-v)/dt).cwiseAbs().maxCoeff()<=2.000001,"stroke acceleration");
        check(!arm.Check_Joint_Limit(next),"stroke effective bounds");
        const auto t=arm.Forward_Kinematics(next);const auto applied=((t.topRightCorner<3,1>()-reference)/dt).eval();
        reference=t.topRightCorner<3,1>();
        controller.feedback_reference(t.topRightCorner<3,1>(),applied);
        const auto actual_q=(state.joints+(lag>0 ? 1-std::exp(-dt/lag) : 1.)*(next-state.joints)).eval();
        const auto actual=arm.Forward_Kinematics(actual_q);
        const auto actual_velocity=((actual.topRightCorner<3,1>()-state.position)/dt).eval();
        state.velocity=.4*actual_velocity+.6*state.velocity;state.position=actual.topRightCorner<3,1>();state.rotation=actual.topLeftCorner<3,3>();state.joints=actual_q;
        const auto bounds=arm.effective_limits();for(int j:{4,5,6})minimum_margin=std::min({minimum_margin,next(j)-bounds.minimum(j),bounds.maximum(j)-next(j)});
        const auto elbow=arm.elbow_position(next);
        minimum_outward=std::min(minimum_outward,(arm.definition().hand?-1.:1.)*elbow.y()-std::abs(arm.definition().mdh(0,2)));
        q=next;v=velocity;
    }
    throw std::runtime_error("whole stroke timed out");
}
int main(int argc,char **argv){try{
    check(argc==3,"runtime/repo required");const std::filesystem::path runtime=argv[1],root=argv[2];
    for(std::string id:{"t170c_v2_3kg","t170c_v2_5kg"}) {
        const auto temp=std::filesystem::temp_directory_path()/("t170-stroke-"+id+std::to_string(getpid()));std::filesystem::create_directories(temp);
        auto model=YAML::LoadFile((root/"config/robot_models"/id/"Robot_Model.yaml").string());
        auto resolved=YAML::LoadFile((runtime/"config/Robot_Model.yaml").string());
        for(const char *side:{"right","left"})model["T7_170"][side]["tool"]=id=="t170c_v2_3kg"?YAML::Clone(resolved["T7_170"][side]["tool"]):YAML::Load("[0.26,0,0,0,0,0]");
        auto binding=YAML::LoadFile((runtime/"config/arm_binding.yaml").string());binding["model_id"]=id;binding["motion_policy"]=YAML::LoadFile((root/"config/robot_models"/id/"motion_policy.yaml").string());
        {std::ofstream f(temp/"Robot_Model.yaml");f<<model;}{std::ofstream f(temp/"arm_binding.yaml");f<<binding;}
        for(std::string side:{"right","left"}) {
            auto d=load_arm_definition((temp/"Robot_Model.yaml").string(),side);ArmJoints low=d.minimum,high=d.maximum;
            low(5)=std::max(low(5),-1.04);high(5)=std::min(high(5),1.04);
            if(d.hand){high(1)=-3.0002*M_PI/180;high(6)=99.9989*M_PI/180;low(6)=-89.9544*M_PI/180;}else{low(1)=3.0002*M_PI/180;low(6)=-M_PI/2;}
            {std::lock_guard lock(d.limits->mutex);d.limits->bounds={low,high,true};}
            Arm arm(d,std::make_shared<ReferenceBackend>(d));
            Pose b;b<<(d.hand?.402673178447:.412925185882),d.hand?-.046147641802:.168618673164,-.293270837787,0,55*M_PI/180,(d.hand?60:-60)*M_PI/180;
            Pose c=b;c(2)=d.hand?-.418270837787:-.414270837787;
            run(arm,b,c,true);run(arm,b,c,true,.18);
            // Current tray placement uses +/-50 deg yaw rather than the
            // near-row grasp's +/-60 deg, and observes neighbours at B.
            Pose place_b=b;place_b(5)=(d.hand?50:-50)*M_PI/180;
            Pose place_c=place_b;place_c(2)=-.421270837787;
            run(arm,place_b,place_c,false);run(arm,place_b,place_c,false,.18);
            Eigen::MatrixXd rejected;auto unreachable=c;unreachable(2)=-1.2;
            check(arm.Approach_Stroke(d.home,b,unreachable,rejected,.4)!=0 && rejected.rows()==0,
                "unreachable finish accepted or partial path retained");
            Pose belt;b <<.5,d.hand?-.25:.25,-.2,0,45*M_PI/180,(d.hand?40:-40)*M_PI/180;belt=b;belt(2)-=.1;
            run(arm,b,belt,true,0,"belt");
        }
        std::filesystem::remove_all(temp);
    }
    return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
