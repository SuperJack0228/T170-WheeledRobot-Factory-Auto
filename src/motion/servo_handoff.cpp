#include "internal/arm_internal.h"
#include "t170/vision/wrist_observer.h"
#include "t170/motion/pose_solver.h"
#include "t170/motion/joint_motion.h"
#include "t170/common/paths.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "t170/common/servo_trace.h"
#include "t170/skills/legacy/profiles.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cctype>
#include <limits>
#include <stdexcept>
using namespace Eigen;
using namespace std;
using namespace t170::motion::detail;
namespace {
int execute_descent_table(const Eigen::MatrixXd &traj, int hand, int descent_row,
                          Arm &arm, const Eigen::Matrix<double,1,6> &goal,
                          double descent_speed, int target_class,const char *tag)
{
    if (traj.rows()<2 || traj.cols()!=7 || descent_row<1 || descent_row>traj.rows()) return -30;
    if (!wrist_servo::begin_session(arm,goal,target_class,tag)) {
        wrist_servo::end_session(hand,false); return -30;
    }
    struct Guard {
        int hand; bool success=false;
        ~Guard() { wrist_servo::end_session(hand,success); }
    } guard{hand};
    wrist_servo::ObservationContext place_context;
    wrist_servo::observation_context(hand,place_context);
    const bool stop_at_B=target_class==1 && place_context.tray_target.valid &&
        place_context.config.tray_place.enabled && place_context.config.tray_place.observe_at_B;
    wrist_servo::Joints previous=traj.row(0), command=previous;
    double last_sample=0;
    for(int i=0;i<descent_row;++i) {
        const auto cycle=std::chrono::steady_clock::now();
        if(hardware_abort_requested()) return -4;
        if(can_io_faulted()) return -2;
        if(wrist_servo::group_is_failed()) return -31;
        if(wrist_servo::monotonic_seconds()-last_sample>=0.02) {
            wrist_servo::sample_approach_state(arm);
            last_sample=wrist_servo::monotonic_seconds();
        }
        previous=command; command=traj.row(i);
        wrist_servo::send_joint_reference(hand,command);
        std::this_thread::sleep_until(cycle+std::chrono::microseconds(5000));
        // B is a guide, not an exact stop. Transfer during the SDK deceleration
        // when the remaining guide distance and joint/Cartesian speed are bounded.
        if(!stop_at_B && i>=2 && i+1<descent_row) {
            const auto &cfg=g_move_cfg.visual_servo;
            const auto pose=arm_tcp_from_joints(arm,command);
            const auto prior=arm_tcp_from_joints(arm,previous);
            const auto guide=arm_tcp_from_joints(arm,wrist_servo::Joints(traj.row(descent_row-1)));
            const double v=(pose.head<3>()-prior.head<3>()).norm()/0.005;
            const double qv=((command-previous)/0.005).cwiseAbs().maxCoeff();
            const double angle=Eigen::AngleAxisd(TR(goal).block<3,3>(0,0)*
                TR(pose).block<3,3>(0,0).transpose()).angle();
            if((pose.head<3>()-guide.head<3>()).norm()<=cfg.handoff_distance_m &&
               v<=cfg.handoff_speed_m_s &&
               qv<=cfg.max_joint_speed_rad_s && angle<=0.002 &&
               pose(2)>goal(2)+cfg.freeze_height_m) {
                wrist_servo::handoff_event(arm,command,previous,"blended_before_B");
                break;
            }
        }
        if(i+1==descent_row) wrist_servo::handoff_event(arm,command,previous,
            stop_at_B?"place_stop_at_B":"SDK_endpoint_fallback");
    }
    g_servo_joint_origin[hand]=command;
    wrist_servo::task_event(hand,"joint_policy","velocity_QP_position_speed_acceleration_braking_elbow_wrist");
    const int result=wrist_servo::run_descent(arm,goal,command,previous,descent_speed);
    guard.success=result==0;
    return result;
}
}


bool descent_commanded_xy(int hand,double &x,double &y) {
    return wrist_servo::completed_xy(hand,x,y);
}

ArmLineMoveResult arm_dual_line_then_line_nostop(
    Arm &arm_r,
    const Eigen::Matrix<double, 1, 6> &mid_r,
    const Eigen::Matrix<double, 1, 6> &goal_r,
    const bool move_r,
    Arm &arm_l,
    const Eigen::Matrix<double, 1, 6> &mid_l,
    const Eigen::Matrix<double, 1, 6> &goal_l,
    const bool move_l,
    const double vel_first_m_s,
    const double vel_second_m_s,
    const std::function<void()> &before_move,
    const bool use_prepared,
    const char *log_tag,
    const bool snap_reach,
    const bool correct_descent,
    const int hand_target_class)
{
    const bool visual = correct_descent && g_move_cfg.cameras.hand_descent_correct;
    if(move_r) wrist_servo::invalidate_result(1);
    if(move_l) wrist_servo::invalidate_result(0);
    Eigen::Matrix<double,1,6> approach_r=mid_r, approach_l=mid_l;
    if(visual) {
        approach_r.tail<3>()=goal_r.tail<3>();
        approach_l.tail<3>()=goal_l.tail<3>();
    }
    ArmLineMoveResult out;
    Eigen::MatrixXd traj_r, traj_l;
    const bool prepared_r =
        use_prepared && !correct_descent && move_r &&
        take_prepared_line(arm_hand_side(arm_r), traj_r, out.ret_r);
    const bool prepared_l =
        use_prepared && !correct_descent && move_l &&
        take_prepared_line(arm_hand_side(arm_l), traj_l, out.ret_l);
    auto plan=[&](Arm &arm,Eigen::Matrix<double,1,6> mid,
                  const Eigen::Matrix<double,1,6> &goal,Eigen::MatrixXd &trajectory) {
        if(!visual) return plan_line_then_line(
            arm,mid,goal,vel_first_m_s,vel_second_m_s,trajectory,log_tag,nullptr,use_prepared);
        wrist_servo::Joints q;
        if(!wrist_servo::read_joint_feedback(arm_hand_side(arm),q)) return -32;
        // Continue from the held command, not a lagged encoder sample; otherwise
        // the player must insert a second connector or refuse a stale handoff.
        wrist_servo::peek_final_hold(arm_hand_side(arm),q);
        const int code=arm.Approach_Stroke(Eigen::MatrixXd(q),mid,goal,trajectory,vel_first_m_s);
        if(code==0) g_planned_descent_row[arm_hand_side(arm)]=static_cast<int>(trajectory.rows());
        // Execute the SDK approach up to its bounded handoff region; the local
        // reference controller rounds the guide and owns all subsequent motion.
        return code;
    };
    if (move_r && !prepared_r) out.ret_r=plan(arm_r,approach_r,goal_r,traj_r);
    if (move_l && !prepared_l) out.ret_l=plan(arm_l,approach_l,goal_l,traj_l);
    if ((move_r && out.ret_r != 0) || (move_l && out.ret_l != 0))
    {
        if (move_r && out.ret_r == 0)
            out.ret_r = -14;
        if (move_l && out.ret_l == 0)
            out.ret_l = -14;
        return out;
    }
    if (before_move)
        before_move();
    if (visual)
        wrist_servo::prepare_group(move_r,move_l);
    const auto watch = [&](Arm &arm, const Eigen::Matrix<double, 1, 6> &goal) {
        const int hand = arm_hand_side(arm);
        if (visual)
            hand_shadow_ensure(
                hand, &arm, goal, log_tag != nullptr ? log_tag : "下降", hand_target_class);
    };
    if (move_r)
        watch(arm_r, goal_r);
    if (move_l)
        watch(arm_l, goal_l);
    auto run_one = [&](Arm &arm, const Eigen::MatrixXd &traj,
                       const Eigen::Matrix<double, 1, 6> &goal, int &ret) {
        const int hand = arm_hand_side(arm);
        try {
            if (visual)
                ret = execute_descent_table(traj, hand, g_planned_descent_row[hand], arm, goal,
                                            vel_second_m_s, hand_target_class,log_tag);
            else
                ret = execute_arm_trajectory(traj, hand, k_arm_traj_dt, use_prepared ? &arm : nullptr);
        } catch(const std::exception &ex) {
            std::cerr << "[servo] hand=" << hand << " execution exception: " << ex.what() << std::endl;
            wrist_servo::ObservationContext context;
            if(visual && wrist_servo::observation_context(hand,context)) {
                wrist_servo::observation_event(hand,context.session,ex.what());
                wrist_servo::end_session(hand,false);
            }
            ret=-32;
        }
    };
    if (move_r && move_l)
    {
        std::thread th_r([&]() { run_one(arm_r, traj_r, goal_r, out.ret_r); });
        std::thread th_l([&]() { run_one(arm_l, traj_l, goal_l, out.ret_l); });
        th_r.join();
        th_l.join();
    }
    else if (move_r)
        run_one(arm_r, traj_r, goal_r, out.ret_r);
    else if (move_l)
        run_one(arm_l, traj_l, goal_l, out.ret_l);
    if(visual && move_r && move_l && (out.ret_r!=0 || out.ret_l!=0)) {
        // No gripper command has been issued yet. A locally finished peer is not a successful grasp.
        for(int hand=0;hand<2;++hand) {
            wrist_servo::ObservationContext context;
            wrist_servo::observation_context(hand,context);
            wrist_servo::observation_event(hand,context.session,"peer_failed_before_gripper");
            wrist_servo::invalidate_result(hand);
        }
        if(out.ret_r==0) out.ret_r=-35;
        if(out.ret_l==0) out.ret_l=-35;
    }
    if (correct_descent && move_r)
        hand_shadow_end(arm_hand_side(arm_r));
    if (correct_descent && move_l)
        hand_shadow_end(arm_hand_side(arm_l));
    const char *done = correct_descent ? "下降到位" : (snap_reach ? "伸手到位" : "收手终点");
    auto arrival=[&](Arm &arm,const Eigen::Matrix<double,1,6> &goal) {
        Eigen::Matrix<double,1,6> corrected=goal;
        double x=0,y=0;
        if(visual && descent_commanded_xy(arm_hand_side(arm),x,y)) { corrected(0)=x; corrected(1)=y; }
        if(visual) {
            // The closed-loop descent has already passed its measured terminal
            // gates. A second generic encoder wait can add up to 15 seconds.
            record_arm_cartesian_snapshot(arm,corrected,done,true);return 0;
        }
        if(record_arm_cartesian_arrival(arm,corrected,done,g_move_cfg.retract_motion.settle_timeout_s))return 0;
        if(hardware_abort_requested())return -4;
        if(can_io_faulted())return -2;
        const auto measured=arm_get_joint_pos(arm);
        if(!arm.Check_Joint_Limit(measured))wrist_servo::hold_joint_reference(arm_hand_side(arm),measured);
        return -8;
    };
    if (move_r && out.ret_r == 0) out.ret_r=arrival(arm_r,goal_r);
    if (move_l && out.ret_l == 0) out.ret_l=arrival(arm_l,goal_l);
    return out;
}
