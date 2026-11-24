// Production skill implementations with synthetic motion/gripper ports, never real hardware.
#include "t170/skills/conveyor.h"
#include "t170/skills/legacy/runtime.h"
#include "t170/motion/servo.h"
#include <iostream>
#include <stdexcept>
MoveBoxConfig g_move_cfg;
namespace {
bool aborted=false,io_fault=false,grasp_empty=false,opened=true;
int approach_code=0,lift_code=0,return_code=0,moves=0,closes=0,opens=0;
double used_approach=0,used_descent=0,used_hover=0;
Eigen::Matrix<double,1,7> returned_joints=Eigen::Matrix<double,1,7>::Zero();
void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
void reset(){aborted=io_fault=grasp_empty=false;opened=true;approach_code=lift_code=return_code=0;moves=closes=opens=0;}
}
Arm::Arm(const std::string &,const std::string &,const std::string &side,const std::string &){can_id_list={side=="left"?23:16};}
bool hardware_abort_requested(){return aborted;}
bool can_io_faulted(){return io_fault;}
bool right_arm_motors_locked(){return false;}
void hardware_abort_sleep_ms(int){}
void hand_shadow_end(int){}
void arm_discard_line_then_line_plan(int){}
Eigen::Matrix<double,1,6> arm_get_tcp_pos(Arm &){return Eigen::Matrix<double,1,6>::Zero();}
ArmLineMoveResult arm_dual_line_then_line_nostop(Arm &,const Eigen::Matrix<double,1,6> &hover,
 const Eigen::Matrix<double,1,6> &goal,bool right,Arm &,const Eigen::Matrix<double,1,6> &,
 const Eigen::Matrix<double,1,6> &,bool left,double reach,double descent,const std::function<void()> &,bool,
 const char *,bool,bool,int,const std::function<void()> &){
    ++moves;used_approach=reach;used_descent=descent;used_hover=hover(2)-goal(2);
    return {right?approach_code:0,left?approach_code:0};
}
int arm_vertical_lift(Arm &,double height,double,const std::function<void(double)> &progress){
    ++moves;if(progress)progress(height);return lift_code;
}
int arm_return_after_lift(Arm &,const Eigen::Matrix<double,1,7> &ready,const std::function<void()> &on_return,double,double){
    ++moves;returned_joints=ready;if(on_return)on_return();return return_code;
}
namespace wrist_servo {
void stage_belt_grasp_reference(int,bool){}
void start_final_hold_repeat(){}
bool ready_for_gripper(Arm &){return true;}
void task_event(int,const char *,const char *){}
}
namespace gripper {
struct Gripper::SideImpl {};
Gripper::Gripper(const Config &c):config_(c){}
Gripper::~Gripper()=default;
GraspFeedback Gripper::graspAndWait(Side,double,std::chrono::milliseconds){++closes;return {grasp_empty?GraspResult::PositionReached:GraspResult::TorqueLimit,grasp_empty?0.:1.,0};}
bool Gripper::closeAndWait(Side,double,double,std::chrono::milliseconds){++opens;return opened;}
}
namespace move_box {
bool gripper_grasp_empty(gripper::Gripper &,gripper::Side,const gripper::GraspFeedback &f,const char *){return f.result==gripper::GraspResult::PositionReached;}
}
int main() {
    using namespace t170::skills;
    try {
        Arm right("synthetic","synthetic","right"),left("synthetic","synthetic","left");gripper::Gripper g;
        ConveyorStationConfig station;station.grasp_hover_above_m=.08;
        station.grasp_approach_vel_m_s=.12;station.grasp_descend_vel_m_s=.035;
        ConveyorContext context{g,right,left,station,{}};
        g_move_cfg.cameras.hand_descent_correct=true;g_move_cfg.visual_servo.belt_grasp_enabled=true;
        ConveyorGraspRequest request;request.goal<<.4,-.2,.15,0,0,0;
        reset();auto result=execute_conveyor_grasp(context,request);
        require(result.ok() && result.hands[1].part==PartState::Holding && result.hands[1].action_completed &&
                !result.hands[1].retreat_completed && moves==1 && closes==1,"grasp contact completion is wrong");
        require(std::abs(used_hover-.08)<1e-10 && used_descent==.035 && used_approach==.12,
                "station motion parameters changed");
        reset();grasp_empty=true;result=execute_conveyor_grasp(context,request);
        require(!result.ok() && result.error==Error::EmptyGrasp && result.native_code==11 &&
                result.hands[1].part==PartState::Empty,"grasp_empty grasp was reported holding");
        reset();approach_code=-16;result=execute_conveyor_grasp(context,request);
        require(result.native_code==-16 && result.failure_stage==Stage::AlignDescend && closes==0,
                "motion failure closed a claw or lost its native code");
        reset();Control cancel;cancel.progress=[&](const Progress &p){if(p.stage==Stage::AlignDescend)aborted=true;};
        result=execute_conveyor_grasp(context,request,cancel);
        require(result.status==Status::Cancelled && moves==0 && closes==0,"cancel at boundary issued motion");
        reset();ConveyorGraspRequest invalid;result=execute_conveyor_grasp(context,invalid);
        require(result.error==Error::InvalidRequest && moves==0,"non-finite target reached planner");
        reset();ConveyorPlaceRequest place;place.goal=request.goal;place.initial_part=PartState::Holding;
        int released=0;context.hooks.on_released=[&]{++released;};
        result=execute_conveyor_place(context,place);
        require(result.ok() && result.hands[1].part==PartState::Empty && released==1 && opens==1,
                "place result did not preserve legacy open behavior");
        reset();opened=false;result=execute_conveyor_place(context,place);
        require(result.error==Error::ReleaseUnconfirmed && result.native_code==0 &&
                result.hands[1].part==PartState::Unknown && !result.hands[1].action_completed,
                "unconfirmed release was marked complete");
        reset();ConveyorRetreatRequest retreat;retreat.initial_part=PartState::Holding;lift_code=-16;
        result=execute_conveyor_retreat(context,retreat);
        require(result.error==Error::Motion && result.hands[1].part==PartState::Holding &&
                !result.hands[1].retreat_completed && moves==1,"failed lift lost held state or continued returning");
        reset();retreat.retry_empty=true;retreat.retry_ready=Joints::Constant(.3);
        result=execute_conveyor_retreat(context,retreat);
        require(result.ok() && result.hands[1].retreat_completed && returned_joints.isApprox(*retreat.retry_ready),
                "grasp_empty retry did not use pre-approach measured joints");
        reset();retreat.retry_empty=false;retreat.retry_ready.reset();
        bool peer_ok=false,same_busy=false;
        context.hooks.on_return_started=[&] {
            auto peer=request;peer.hand=Hand::Left;
            peer_ok=execute_conveyor_grasp(context,peer).ok();
            same_busy=execute_conveyor_grasp(context,request).error==Error::Busy;
        };
        result=execute_conveyor_retreat(context,retreat);
        require(result.ok() && peer_ok && same_busy,"arm lease broke overlapped retreat or allowed a double writer");
        std::cout<<"conveyor skills: production contact/retreat, native errors, release uncertainty, cancellation and peer overlap passed\n";
    }catch(const std::exception &ex){std::cerr<<ex.what()<<'\n';return 1;}
}
