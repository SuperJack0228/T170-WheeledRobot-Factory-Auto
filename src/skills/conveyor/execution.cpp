#include "t170/skills/conveyor.h"
#include "../internal/execution.h"
#include "t170/skills/legacy/runtime.h"
#include "t170/motion/servo.h"
#include "t170/hardware/device_bus.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
namespace t170::skills {
namespace {
struct Binding { gripper::Gripper *g; Arm *arm_r,*arm_l; bool last_grasped_r=false,last_grasped_l=false; };
int contact(ConveyorContext &context,const ConveyorGraspRequest &request,detail::Execution &run) {
    Binding binding{&context.gripper,&context.right,&context.left};auto *g_hw=&binding;
    const bool is_right=request.hand==Hand::Right;
    const auto &c=context.station;
    auto aborted=[&]{return hardware_abort_requested() || can_io_faulted() || run.cancelled();};
    const char *name=request.log_tag.c_str();
    const auto &station=request.metadata.workcell_id;
    constexpr int kBeltGraspEmpty=11;

        if (is_right && right_arm_motors_locked())
        {
            std::cout << "[orch] belt_grasp 右臂锁定，跳过右手抓取\n";
            return 0;
        }
        Eigen::Matrix<double,1,6> goal=request.goal;
        const char *hand = is_right ? "右手" : "左手";
        Eigen::Matrix<double, 1, 6> hover = goal;
        const double hover_z = c.grasp_hover_above_m > 1e-9 ? c.grasp_hover_above_m : 0.08;
        hover(2) += hover_z;
        const double reach_mps = c.grasp_approach_vel_m_s;
        double kDescentMps = c.grasp_descend_vel_m_s;
        if (!std::isfinite(kDescentMps) || kDescentMps <= 0)
            return -30;
        const bool belt_grasp_servo =
            g_move_cfg.cameras.hand_descent_correct &&
            g_move_cfg.visual_servo.belt_grasp_enabled;
        struct ClearBeltReference {
            int hand=-1;
            ~ClearBeltReference(){ if(hand>=0) wrist_servo::stage_belt_grasp_reference(hand,false); }
        } clear_belt_reference;
        if(belt_grasp_servo) {
            clear_belt_reference.hand=is_right?1:0;
            wrist_servo::stage_belt_grasp_reference(clear_belt_reference.hand,true);
        }
        std::cout << std::fixed << std::setprecision(2)
                  << "[orch] belt_grasp " << hand << " " << reach_mps
                  << "m/s 先到正上方 " << (hover_z * 100.0)
                  << " cm，再竖直 " << kDescentMps
                  << " m/s 下降。";
        if (belt_grasp_servo)
            std::cout << "下降段使用双轴闭环、延迟补偿与预测制动\n";
        else
            std::cout << "传送带抓取开环偏置（码+offset），不进手相机伺服\n";
        const std::string descent_tag=std::string(name)+"/"+station+"/grasp";
        run.stage(Stage::AlignDescend,{!is_right,is_right});
        if(aborted()) return -4;
        const ArmLineMoveResult approach = arm_dual_line_then_line_nostop(
            *g_hw->arm_r, hover, goal, is_right,
            *g_hw->arm_l, hover, goal, !is_right,
            reach_mps, kDescentMps, {}, false, descent_tag.c_str(), false,
            belt_grasp_servo, 0);
        const int approach_rc = is_right ? approach.ret_r : approach.ret_l;
        run.native(approach_rc,{!is_right,is_right},Stage::AlignDescend);
        if (approach_rc != 0)
        {
            std::cerr << "[orch] belt_grasp " << hand
                      << " 伸手失败 code=" << approach_rc << "\n";
            return aborted() ? -4 : approach_rc;
        }
        if (aborted()) return -4;
        run.stage(Stage::Gripper,{!is_right,is_right});
        if(aborted()) return -4;
        bool empty_r=false,empty_l=false;
        if(belt_grasp_servo) {
            const auto closed=move_box::finish_servo_grasp(
                *g_hw->g,*g_hw->arm_r,*g_hw->arm_l,is_right,!is_right,empty_r,empty_l,name);
            const int code=is_right ? closed.ret_r : closed.ret_l;
            if(code!=0) return code;
        } else {
            // Open-loop belt grasp: settle then close; no wrist_servo hold/gate.
            hardware_abort_sleep_ms(500);
            if(aborted()) return -4;
            const auto side=is_right ? gripper::Side::Right : gripper::Side::Left;
            const auto feedback=g_hw->g->graspAndWait(side);
            (is_right ? empty_r : empty_l)=move_box::gripper_grasp_empty(*g_hw->g,side,feedback,name);
        }
        detail::report_action(is_right,(is_right?empty_r:empty_l)?PartState::Empty:PartState::Holding,Evidence::LegacyPolicy,!(is_right?empty_r:empty_l));
        if(is_right ? empty_r : empty_l) return aborted() ? -4 : kBeltGraspEmpty;
        if(is_right) g_hw->last_grasped_r=true;
        else g_hw->last_grasped_l=true;
        return aborted() ? -4 : 0;
    
}
int contact(ConveyorContext &context,const ConveyorPlaceRequest &request,detail::Execution &run) {
    Binding binding{&context.gripper,&context.right,&context.left};auto *g_hw=&binding;
    const bool is_right=request.hand==Hand::Right;
    const auto &c=context.station;
    auto aborted=[&]{return hardware_abort_requested() || can_io_faulted() || run.cancelled();};
    auto log_gripper_qs=[&](const char *tag){if(context.hooks.log_gripper) context.hooks.log_gripper(tag);};

        if (is_right && right_arm_motors_locked())
        {
            std::cout << "[orch] belt 右臂锁定，跳过右手放置\n";
            return 0;
        }
        Eigen::Matrix<double,1,6> goal=request.goal;
        const double place_mps = c.place_approach_vel_m_s;
        double kDescentMps = g_move_cfg.head_grasp.descend_vel_m_s;
        if (!std::isfinite(kDescentMps) || kDescentMps <= 0)
            return -30;
        std::cout << "[orch] belt " << (is_right ? "右手" : "左手")
                  << " " << std::fixed << std::setprecision(2) << place_mps
                  << "m/s 先到上方，再竖直 " << kDescentMps << " m/s 放下\n";
        Eigen::Matrix<double, 1, 6> hover = goal;
        if (c.place_hover_above_m > 1e-9)
            hover(2) += c.place_hover_above_m;
        if(!g_move_cfg.visual_servo.belt_place_enabled)
            std::cout << "[servo] 传送带放置使用既有开环目标；未定义可观测特征，不计入视觉闭环验收\n";
        run.stage(Stage::AlignDescend,{!is_right,is_right});
        if(aborted()) return -4;
        const ArmLineMoveResult approach = arm_dual_line_then_line_nostop(
            *g_hw->arm_r, hover, goal, is_right,
            *g_hw->arm_l, hover, goal, !is_right,
            place_mps, kDescentMps, {}, false, "belt放置", false,
            g_move_cfg.visual_servo.belt_place_enabled, 1);
        const int approach_rc = is_right ? approach.ret_r : approach.ret_l;
        run.native(approach_rc,{!is_right,is_right},Stage::AlignDescend);
        if (approach_rc != 0)
        {
            std::cerr << "[orch] belt " << (is_right ? "右手" : "左手")
                      << " 放置伸手失败 code=" << approach_rc << "\n";
            return aborted() ? -4 : -5;
        }
        if (aborted())
            return -4;
        run.stage(Stage::Gripper,{!is_right,is_right});
        if(aborted()) return -4;
        if (g_hw->g != nullptr)
        {
            std::cout << "[orch] belt " << (is_right ? "右手" : "左手")
                      << " 放置点松爪到 0.2（0全开 1全合）\n";
            log_gripper_qs("放置松爪前");
            const bool opened = g_hw->g->closeAndWait(
                is_right ? gripper::Side::Right : gripper::Side::Left,
                0.2, 0.2, std::chrono::milliseconds(800));
            log_gripper_qs(opened ? "放置松爪到位" : "放置松爪已下发，不等开到位");
            detail::report_action(is_right,opened?PartState::Empty:PartState::Unknown,Evidence::LegacyPolicy,opened);
            if(!opened) {
                auto &state=run.result.hands[index(request.hand)];
                state.error=Error::ReleaseUnconfirmed;state.failure_stage=Stage::Gripper;
                run.fail(Error::ReleaseUnconfirmed,"belt open wait did not confirm release");
            }
            if(context.hooks.on_released) context.hooks.on_released();
            if (aborted())
                return -4;
        }
        return aborted() ? -4 : 0;
    
}
int withdraw(ConveyorContext &context,const ConveyorRetreatRequest &request,detail::Execution &run) {
    Binding binding{&context.gripper,&context.right,&context.left};auto *g_hw=&binding;
    const bool is_right=request.hand==Hand::Right;
    const auto &c=context.station;
    auto aborted=[&]{return hardware_abort_requested() || can_io_faulted() || run.cancelled();};
    const char *name=request.log_tag.c_str();
    const char *back_name_default="固定关节中转准备位";
    const bool to_ready=request.retry_empty;
    const auto &progress=context.hooks.on_measured_lift;
    const auto &on_return=context.hooks.on_return_started;
    Joints retry_ready[2]={Joints::Zero(),Joints::Zero()};
    bool retry_ready_ok[2]={false,false};
    if(request.retry_ready) {retry_ready[index(request.hand)]=*request.retry_ready;retry_ready_ok[index(request.hand)]=true;}

        if (is_right && right_arm_motors_locked())
            return 0;
        if (aborted())
            return -4;
        Arm &arm = is_right ? *g_hw->arm_r : *g_hw->arm_l;
        const char *hand = is_right ? "右手" : "左手";
        const char *back_name = to_ready ? "伸手前实测关节（空抓）" : back_name_default;
        if (aborted())
            return -4;
        if(to_ready) arm_discard_line_then_line_plan(is_right ? 1 : 0);
        const double speed=c.grasp_retract_vel_m_s;
        std::cout<<"[orch] "<<name<<' '<<hand<<" 合爪后先竖直抬离，再规划返回，目标="<<back_name
                 <<" speed="<<speed<<"m/s\n";
        const double lift_height=c.grasp_hover_above_m>1e-9 ? c.grasp_hover_above_m : g_move_cfg.head_grasp.lift_after_grasp_z;
        run.stage(Stage::Lift,{!is_right,is_right});
        if(aborted()) return -4;
        int code=arm_vertical_lift(arm,lift_height,speed,progress);
        run.native(code,{!is_right,is_right},Stage::Lift);
        if(code==0) {
            if(to_ready && !retry_ready_ok[is_right?1:0]) return -32;
            const auto &m=g_move_cfg.retract_motion;
            const wrist_servo::Joints ready=to_ready?retry_ready[is_right?1:0]:
                (is_right?m.ready_right:m.ready_left);
            run.stage(Stage::Return,{!is_right,is_right});
        if(aborted()) return -4;
            code=arm_return_after_lift(arm,ready,on_return,on_return?0.40:-1.0,1.0,speed);
        }
        if(code!=0) {
            std::cerr<<"[orch] "<<name<<' '<<hand<<" 收手失败 code="<<code<<"，保留夹爪状态\n";
            return aborted() ? -4 : code;
        }
        return aborted() ? -4 : 0;
    
}
int withdraw_place(ConveyorContext &context,const ConveyorRetreatRequest &request,detail::Execution &run) {
    Binding binding{&context.gripper,&context.right,&context.left};auto *g_hw=&binding;
    const bool is_right=request.hand==Hand::Right;
    const auto &c=context.station;
    auto aborted=[&]{return hardware_abort_requested() || can_io_faulted() || run.cancelled();};

        if (is_right && right_arm_motors_locked())
            return 0;
        if (aborted())
            return -4;
        Arm &arm = is_right ? *g_hw->arm_r : *g_hw->arm_l;
        constexpr double kPlaceNudgeUpM = 0.03;
        const double place_mps = c.place_retract_vel_m_s;
        Eigen::Matrix<double, 1, 6> lift = arm_get_tcp_pos(arm);
        const double z0 = lift(2);
        lift(2) = z0 + kPlaceNudgeUpM;
        Eigen::Matrix<double, 1, 6> ready = is_right ? c.tcp.right : c.tcp.left;
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] belt " << (is_right ? "右手" : "左手")
                  << " " << std::setprecision(2) << place_mps
                  << "m/s 松爪后抬 " << std::setprecision(4) << kPlaceNudgeUpM
                  << "m 后不停，直接回准备\n";
        run.stage(Stage::Return,{!is_right,is_right});
        if(aborted()) return -4;
        const ArmLineMoveResult back = arm_dual_line_then_line_nostop(
            *g_hw->arm_r, lift, ready, is_right,
            *g_hw->arm_l, lift, ready, !is_right,
            place_mps, place_mps, {}, false, "放置收手", false, false, 0,
            context.hooks.on_return_started);
        const int back_rc = is_right ? back.ret_r : back.ret_l;
        run.native(back_rc,{!is_right,is_right},Stage::Return);
        if (back_rc != 0)
        {
            std::cerr << "[orch] belt " << (is_right ? "右手" : "左手")
                      << " 放置收手失败 code=" << back_rc << "\n";
            return aborted() ? -4 : -5;
        }
        return aborted() ? -4 : 0;
    
}
template<class Request> bool validate(detail::Execution &run,const Request &request) {
    if(!run.entered()) return false;
    if((request.hand!=Hand::Left && request.hand!=Hand::Right)) {
        run.fail(Error::InvalidRequest,"valid hand is required");return false;
    }
    if(hardware_abort_requested() || run.cancelled()) {
        run.fail(Error::Cancelled,"aborted before conveyor action",-4);return false;
    }
    if(can_io_faulted()) {run.fail(Error::Device,"CAN feedback unavailable",-2);return false;}
    return true;
}
unsigned resource(Hand hand) {return hand==Hand::Left?1u:hand==Hand::Right?2u:0u;}

} // namespace
Result execute_conveyor_grasp(ConveyorContext &context,const ConveyorGraspRequest &request,const Control &control) {
    detail::Execution run(Skill::ConveyorGrasp,request.metadata,{request.hand==Hand::Left,request.hand==Hand::Right},control,resource(request.hand));
    if(!validate(run,request)) return run.finish();
    if(!request.goal.allFinite()) {run.fail(Error::InvalidRequest,"goal contains non-finite coordinates");return run.finish();}
    NominalTarget target;target.frame="arm_base";
    for(int i=0;i<6;++i) target.pose[i]=request.goal(i);
    run.result.hands[index(request.hand)].nominal_target=target;
    detail::ExecutionScope scope(run);
    try {
        const int code=contact(context,request,run);run.result.legacy_status=code;
        if(!run.result.native_code) run.result.native_code=code;
        run.native(code,{request.hand==Hand::Left,request.hand==Hand::Right},run.result.stage);
    }
    catch(const std::exception &ex) {run.fail(Error::Exception,ex.what(),-32);}
    return run.finish();
}
Result execute_conveyor_place(ConveyorContext &context,const ConveyorPlaceRequest &request,const Control &control) {
    detail::Execution run(Skill::ConveyorPlace,request.metadata,{request.hand==Hand::Left,request.hand==Hand::Right},control,resource(request.hand));
    if(!validate(run,request)) return run.finish();
    if(!request.goal.allFinite()) {run.fail(Error::InvalidRequest,"goal contains non-finite coordinates");return run.finish();}
    auto &hand=run.result.hands[index(request.hand)];hand.part=request.initial_part;
    if(hand.part!=PartState::Unknown) hand.evidence=Evidence::CallerReported;
    NominalTarget target;target.frame="arm_base";
    for(int i=0;i<6;++i) target.pose[i]=request.goal(i);
    run.result.hands[index(request.hand)].nominal_target=target;
    detail::ExecutionScope scope(run);
    try {
        const int code=contact(context,request,run);run.result.legacy_status=code;
        if(!run.result.native_code) run.result.native_code=code;
        run.native(code,{request.hand==Hand::Left,request.hand==Hand::Right},run.result.stage);
    }
    catch(const std::exception &ex) {run.fail(Error::Exception,ex.what(),-32);}
    return run.finish();
}
Result execute_conveyor_retreat(ConveyorContext &context,const ConveyorRetreatRequest &request,const Control &control) {
    detail::Execution run(Skill::ConveyorRetreat,request.metadata,{request.hand==Hand::Left,request.hand==Hand::Right},control,resource(request.hand));
    if(!validate(run,request)) return run.finish();
    if(request.retry_ready && !request.retry_ready->allFinite()) {run.fail(Error::InvalidRequest,"retry joints contain non-finite values");return run.finish();}
    auto &hand=run.result.hands[index(request.hand)];hand.part=request.initial_part;
    if(hand.part!=PartState::Unknown) hand.evidence=Evidence::CallerReported;
    detail::ExecutionScope scope(run);
    try {
        const int code=request.after_place?withdraw_place(context,request,run):withdraw(context,request,run);
        run.result.legacy_status=code;if(!run.result.native_code) run.result.native_code=code;run.native(code,{request.hand==Hand::Left,request.hand==Hand::Right},run.result.stage);
        hand.action_completed=hand.retreat_completed=code==0;
    } catch(const std::exception &ex) {run.fail(Error::Exception,ex.what(),-32);}
    return run.finish();
}
} // namespace t170::skills
