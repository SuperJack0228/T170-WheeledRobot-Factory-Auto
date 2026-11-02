// Production phase coordinator, synthetic SDK adapters: no hardware is accessed.
#include "move_box_runtime.h"
#include "servo/runtime.h"
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>

namespace {
std::atomic<bool> abort_phase_test{false},prepared{false},hold_started{false},parallel{false};
std::atomic<int> closes[2]{},stopped_watches[2]{},plans{0};
bool ready[2]={true,true};
std::mutex phase_test_mu;std::condition_variable wake;
int throw_hand=-1;
void require(bool ok,const char *reason) {if(!ok) throw std::runtime_error(reason);}
void reset() {
    abort_phase_test=false;prepared=false;hold_started=false;parallel=false;plans=0;throw_hand=-1;
    for(int h=0;h<2;++h) {closes[h]=0;stopped_watches[h]=0;ready[h]=true;}
}
}
Robot_Arm::Robot_Arm(const std::string &,const std::string &,const std::string &side,const std::string &) {
    can_id_list={side=="left" ? 23 : 16};
}
bool hardware_abort_requested() {return abort_phase_test.load();}
bool can_io_faulted() {return false;}
void hand_shadow_end(int hand) {++stopped_watches[hand];}
namespace wrist_servo {
void start_final_hold_repeat() {hold_started=true;}
bool ready_for_gripper(Robot_Arm &arm) {return ready[arm.can_id_list[0]==23 ? 0 : 1];}
void task_event(int,const char *,const char *) {}
}
namespace gripper {
struct Gripper::SideImpl {};
Gripper::Gripper(const Config &c):config_(c) {}
Gripper::~Gripper()=default;
GraspFeedback Gripper::graspAndWait(Side side,double,std::chrono::milliseconds) {
    const int hand=side==Side::Right ? 1 : 0;
    require(hold_started && prepared,"close occurred before hold/retreat preparation");
    require(stopped_watches[hand]==1,"vision watch not stopped before closing");
    ++closes[hand];wake.notify_all();
    if(throw_hand==hand) throw std::runtime_error("synthetic claw failure");
    if(parallel.load()) {
        std::unique_lock<std::mutex> lock(phase_test_mu);
        require(wake.wait_for(lock,std::chrono::seconds(1),[]{return closes[0]>0 && closes[1]>0;}),
                "dual grippers were serialized");
    }
    return {hand==0 ? GraspResult::PositionReached : GraspResult::TorqueLimit,hand==0 ? 0.0 : 1.0,0};
}
}
namespace move_box {
bool gripper_grasp_empty(gripper::Gripper &,gripper::Side,const gripper::GraspFeedback &f,const char *) {
    return f.result==gripper::GraspResult::PositionReached;
}
}
int main() {
    try {
        Robot_Arm right("synthetic","synthetic","right"),left("synthetic","synthetic","left");
        gripper::Gripper g;bool empty_r=false,empty_l=false;
        auto prepare=[]{++plans;prepared=true;};
        reset();
        auto result=move_box::finish_servo_grasp(g,right,left,false,false,empty_r,empty_l,"none",prepare);
        require(!hold_started && plans==0 && closes[0]==0 && closes[1]==0,"inactive hands were commanded");
        reset();
        result=move_box::finish_servo_grasp(g,right,left,true,false,empty_r,empty_l,"belt",prepare);
        require(result.ret_r==0 && closes[1]==1 && closes[0]==0 && plans==1 && !empty_r,"single right grasp failed");
        reset();ready[0]=false;
        result=move_box::finish_servo_grasp(g,right,left,true,true,empty_r,empty_l,"tray",prepare);
        require(result.ret_r==-36 && result.ret_l==-36 && plans==0 && closes[0]==0 && closes[1]==0,
                "one failed gate allowed a peer claw command");
        reset();parallel=true;
        result=move_box::finish_servo_grasp(g,right,left,true,true,empty_r,empty_l,"tray",prepare);
        require(result.ret_r==0 && result.ret_l==0 && plans==1 && closes[0]==1 && closes[1]==1 &&
                !empty_r && empty_l,"dual feedback/empty-result routing failed");
        reset();
        result=move_box::finish_servo_grasp(g,right,left,false,true,empty_r,empty_l,"belt",prepare);
        require(result.ret_l==0 && closes[0]==1 && closes[1]==0 && empty_l,"single left or empty feedback failed");
        reset();
        result=move_box::finish_servo_grasp(g,right,left,true,false,empty_r,empty_l,"belt",[]{throw std::runtime_error("plan failure");});
        require(result.ret_r==-32 && closes[1]==0,"preparation failure closed a claw");
        reset();
        result=move_box::finish_servo_grasp(g,right,left,true,false,empty_r,empty_l,"belt",[]{abort_phase_test=true;});
        require(result.ret_r==-4 && closes[1]==0,"STOP during preparation did not prevent closing");
        reset();throw_hand=1;
        result=move_box::finish_servo_grasp(g,right,left,true,false,empty_r,empty_l,"belt",prepare);
        require(result.ret_r==-32,"claw exception was reported as success");
        std::cout<<"grasp_execution: shared tray/belt phase ordering, selected hands, dual close, empty feedback and failures passed\n";
    } catch(const std::exception &ex) {std::cerr<<"grasp_execution FAIL: "<<ex.what()<<'\n';return 1;}
}
