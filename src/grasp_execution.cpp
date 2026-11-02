#include "move_box_runtime.h"
#include "servo/runtime.h"
#include "hand_shadow.h"
#include "Ti5_socketcan.h"
#include <iostream>
#include <thread>

namespace move_box {
ArmLineMoveResult finish_servo_grasp(
    gripper::Gripper &g,Robot_Arm &arm_r,Robot_Arm &arm_l,
    bool move_r,bool move_l,bool &empty_r,bool &empty_l,
    const char *tag,const std::function<void()> &before_close) {
    ArmLineMoveResult out;
    if(!move_r && !move_l) return out;
    if(move_r) empty_r=false;
    if(move_l) empty_l=false;
    auto fail=[&](int code) { return ArmLineMoveResult{move_r ? code : 0,move_l ? code : 0}; };
    if(hardware_abort_requested() || can_io_faulted()) return fail(hardware_abort_requested() ? -4 : -2);
    // Keep the final corrected joint command alive until retract takes ownership.
    wrist_servo::start_final_hold_repeat();
    if(move_r) wrist_servo::task_event(1,"both_ready","checking_before_gripper");
    if(move_l) wrist_servo::task_event(0,"both_ready","checking_before_gripper");
    const bool ready_r=!move_r || wrist_servo::ready_for_gripper(arm_r);
    const bool ready_l=!move_l || wrist_servo::ready_for_gripper(arm_l);
    if(!ready_r || !ready_l) {
        std::cerr<<"[grasp] "<<(tag ? tag : "grasp")<<" 合爪前位置复核失败\n";
        return fail(-36);
    }
    try { if(before_close) before_close(); }
    catch(const std::exception &ex) {
        std::cerr<<"[grasp] 收手预规划启动失败: "<<ex.what()<<'\n';return fail(-32);
    }
    if(hardware_abort_requested() || can_io_faulted()) return fail(hardware_abort_requested() ? -4 : -2);
    if(move_r) hand_shadow_end(1);
    if(move_l) hand_shadow_end(0);
    auto close=[&](int hand,int &code,bool &empty) {
        wrist_servo::task_event(hand,"gripper_start","close");
        try {
            const auto side=hand==1 ? gripper::Side::Right : gripper::Side::Left;
            const auto feedback=g.graspAndWait(side);
            wrist_servo::task_event(hand,"gripper_end","feedback_received");
            empty=gripper_grasp_empty(g,side,feedback,tag ? tag : "grasp");
        } catch(const std::exception &ex) {
            wrist_servo::task_event(hand,"gripper_end","exception");
            std::cerr<<"[grasp] hand="<<hand<<" 合爪异常: "<<ex.what()<<'\n';code=-32;
        }
    };
    if(move_r && move_l) {
        std::thread right([&]{close(1,out.ret_r,empty_r);});
        close(0,out.ret_l,empty_l);right.join();
    } else if(move_r) close(1,out.ret_r,empty_r);
    else close(0,out.ret_l,empty_l);
    if(hardware_abort_requested() || can_io_faulted()) return fail(hardware_abort_requested() ? -4 : -2);
    return out;
}
}
