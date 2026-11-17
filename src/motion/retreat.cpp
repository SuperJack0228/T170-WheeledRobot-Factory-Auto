#include "internal/arm_internal.h"
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
std::mutex g_staged_motion_mu[2];
}


namespace {
int plan_seeded_vertical_lift(Robot_Arm &arm, const wrist_servo::Joints &grasp, double lift_m,
                              double speed, Eigen::MatrixXd &trajectory, double &rise_m, std::string &detail)
{
    detail="initial_fk_or_height";
    const int hand = arm_hand_side(arm);
    Eigen::Matrix4d start_T;
    {
        std::lock_guard<std::mutex> lock(g_arm_kin_mu[hand]);
        start_T = arm.Forward_Kinematics(Eigen::RowVectorXd(grasp));
    }
    if (!start_T.allFinite())
        return -10;
    const Eigen::Matrix3d rotation = start_T.block<3, 3>(0, 0);
    const double x = start_T(0, 3);
    const double y = start_T(1, 3);
    double z = start_T(2, 3);
    const double z_goal = std::min(z + lift_m, g_move_cfg.grasp_valid.z_max);
    if (!(z_goal > z + 0.001))
        return -10;
    struct Waypoint
    {
        wrist_servo::Joints q;
        double z = 0;
    };
    std::vector<Waypoint> waypoints{{grasp, z}};
    wrist_servo::Joints q = grasp;
    double step = 0.010;
    while (z < z_goal - 1e-4)
    {
        if (hardware_abort_requested() || can_io_faulted())
            return hardware_abort_requested() ? -4 : -2;
        double dz = std::min(step, z_goal - z);
        wrist_servo::Joints next;
        bool solved = false;
        while (dz >= 0.002)
        {
            std::lock_guard<std::mutex> lock(g_arm_kin_mu[hand]);
            const wrist_servo::ForwardModel forward = [&](const wrist_servo::Joints &sample) {
                return arm.Forward_Kinematics(Eigen::RowVectorXd(sample));
            };
            const wrist_servo::JointLimitCheck limits = [&](const wrist_servo::Joints &sample) {
                return (sample - grasp).cwiseAbs().maxCoeff() > 1.5 ||
                       (sample - q).cwiseAbs().maxCoeff() > 0.20;
            };
            wrist_servo::Vec3 target;
            target << x, y, z + dz;
            double position_error = 0;
            double rotation_error = 0;
            if (!wrist_servo::solve_pose_with_fk(
                    q, target, rotation, g_move_cfg.visual_servo, forward, limits, next,
                    position_error, rotation_error))
            {
                dz *= 0.5;
                continue;
            }
            const Eigen::Matrix4d solved_T = arm.Forward_Kinematics(Eigen::RowVectorXd(next));
            if (!solved_T.allFinite() || solved_T(2, 3) < z + std::min(0.001, 0.5 * dz))
            {
                dz *= 0.5;
                continue;
            }
            z = solved_T(2, 3);
            q = next;
            solved = true;
            break;
        }
        if (!solved)
            break;
        waypoints.push_back({q, z});
        step = 0.010;
    }
    rise_m = waypoints.back().z - waypoints.front().z;
    detail="local_ik requested="+std::to_string(z_goal-start_T(2,3))+" solved="+std::to_string(rise_m);
    if (waypoints.size() < 2 || rise_m < 0.002)
        return -16;
    // A partial 2 mm rise must not authorize a horizontal return beside the tray.
    if(z_goal-waypoints.back().z>g_move_cfg.head_grasp.retract_arrival_tolerance_m)
        return -16;
    Eigen::MatrixXd path(waypoints.size(),7);
    for(std::size_t i=0;i<waypoints.size();++i) path.row(i)=waypoints[i].q;
    const auto &c=g_move_cfg.retract_motion;
    detail="time_parameterization";
    trajectory=t170::motion::time_joint_path(path,k_arm_traj_dt,
        {c.max_joint_velocity_rad_s,c.max_joint_acceleration_rad_s2,c.max_joint_jerk_rad_s3},2*rise_m/speed);
    if(trajectory.rows()<2) return -16;
    // Check the interpolated path as well as IK knots near the loaded tray.
    for(int i=0;i<trajectory.rows();++i) {
        std::lock_guard<std::mutex> lock(g_arm_kin_mu[hand]);
        const auto T=arm.Forward_Kinematics(Eigen::RowVectorXd(trajectory.row(i)));
        const double angle=Eigen::AngleAxisd(rotation*T.block<3,3>(0,0).transpose()).angle();
        if(!T.allFinite() || T(2,3)<start_T(2,3)-.002 ||
           T(2,3)>z_goal+.002 ||
           (T.block<2,1>(0,3)-start_T.block<2,1>(0,3)).norm()>.008 || angle>.08) {
            detail="interpolated_geometry row="+std::to_string(i);return -16;
        }
    }
    detail="continuous_ik_timed";
    return 0;
}
}


namespace {
int execute_staged_motion(Robot_Arm &arm,const Eigen::MatrixXd &path,const char *stage,
                          const std::function<void(const wrist_servo::Joints &)> &progress={},
                          const std::function<bool(const wrist_servo::Joints &)> &arrived={},
                          double notify_at=-1,const std::function<void()> &on_notify={})
{
    const int hand=arm_hand_side(arm);
    if(path.rows()<2 || path.cols()!=7 || !path.allFinite()) return -16;
    wrist_servo::Joints transferred;
    if(wrist_servo::take_final_hold(hand,transferred) &&
       (transferred-path.row(0)).cwiseAbs().maxCoeff()>1e-6) {
        wrist_servo::hold_joint_reference(hand,transferred);
        wrist_servo::task_event(hand,"retract_handoff_failed","reference_changed_during_planning");
        return -16;
    }
    const auto &cfg=g_move_cfg.retract_motion;
    const auto goal=arm_tcp_from_joints(arm,wrist_servo::Joints(path.bottomRows(1)));
    std::cout<<"[retract] hand="<<hand<<" stage="<<stage<<" duration_s="<<(path.rows()-1)*k_arm_traj_dt
             <<" joint_limits="<<cfg.max_joint_velocity_rad_s<<","<<cfg.max_joint_acceleration_rad_s2
             <<","<<cfg.max_joint_jerk_rad_s3<<" ready_tolerance_rad="<<cfg.arrival_joint_tolerance_rad<<'\n';
    double last_print=-1;
    bool notified=false;
    t170::motion::MotionIO io;
    io.read=[&](auto &q){return wrist_servo::read_joint_feedback(hand,q);};
    io.arrived=arrived;
    io.send=[&](const auto &q) {
        try {wrist_servo::send_joint_reference(hand,q);return true;}
        catch(const std::exception &e) {std::cerr<<"[retract] write: "<<e.what()<<'\n';return false;}
    };
    io.stop_code=[] {return hardware_abort_requested()?-4:can_io_faulted()?-2:0;};
    io.now=[] {return wrist_servo::monotonic_seconds();};
    io.wait=[](double seconds) {
        if(seconds>0) std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    };
    io.observe=[&](const char *event,int index,const wrist_servo::Joints &command,const wrist_servo::Joints *actual,double lag) {
        const double now=wrist_servo::monotonic_seconds();
        wrist_servo::ObservationContext context;wrist_servo::observation_context(hand,context);
        wrist_servo::TraceRow row;row.time_s=now;row.hand=hand;row.session=context.session;
        row.task_tag=context.task_tag;row.target_class=context.target_class;
        row.final_z=context.final_z;row.event=std::string(stage)+"_"+event;
        row.reason="joint_motion_v2";row.trajectory_row=index;row.commanded=command;
        auto q_at=[&](int i)->wrist_servo::Joints {return path.row(std::max(0,i));};
        const wrist_servo::Joints velocity=(q_at(index)-q_at(index-1))/k_arm_traj_dt;
        const wrist_servo::Joints acceleration=(q_at(index)-2*q_at(index-1)+q_at(index-2))/(k_arm_traj_dt*k_arm_traj_dt);
        row.command_step_rad=(q_at(index)-q_at(index-1)).cwiseAbs().maxCoeff();
        row.command_speed_rad_s=velocity.cwiseAbs().maxCoeff();
        row.command_accel_rad_s2=acceleration.cwiseAbs().maxCoeff();
        row.command_jerk_rad_s3=((q_at(index)-3*q_at(index-1)+3*q_at(index-2)-q_at(index-3))/
            std::pow(k_arm_traj_dt,3)).cwiseAbs().maxCoeff();
        row.reference=goal.head<3>().transpose();row.joint_lead_rad=lag;
        row.joint_feedback_valid=actual!=nullptr;
        row.measured.joints.setConstant(NAN);row.measured.position.setConstant(NAN);
        if(actual) {
            row.measured.joints=*actual;row.measured.position=arm_tcp_from_joints(arm,*actual).head<3>().transpose();
            if(progress) progress(*actual);
        }
        wrist_servo::trace_writer().push(std::move(row));
        if(on_notify && !notified && path.rows()>1 &&
           (index>=static_cast<int>(std::ceil(std::clamp(notify_at,0.0,1.0)*(path.rows()-1))) ||
            std::string(event)=="command_complete")) {
            notified=true;on_notify();
        }
        if(std::string(event)!="feedback" || now-last_print>=.5) {
            last_print=now;
            std::cout<<"[retract] hand="<<hand<<" stage="<<stage<<" event="<<event
                     <<" row="<<index<<"/"<<path.rows()-1<<" lag_deg="<<lag*180/M_PI;
            if(actual) std::cout<<" actual_q="<<*actual<<" command_q="<<command;
            std::cout<<'\n';
        }
    };
    const int result=t170::motion::run_joint_motion(path,k_arm_traj_dt,
        {cfg.arrival_joint_tolerance_rad,cfg.settle_timeout_s,cfg.feedback_timeout_s,cfg.stable_samples},io);
    // On arrival keep the terminal reference alive. On failure keep measured
    // position instead of continuing to push a distant target; STOP/CAN never writes.
    if(hardware_abort_requested() || can_io_faulted()) return result;
    wrist_servo::Joints final;
    if(result==0) final=path.bottomRows(1);
    else if(!wrist_servo::read_joint_feedback(hand,final)) return result;
    wrist_servo::hold_joint_reference(hand,final);
    wrist_servo::task_event(hand,result==0 ? (std::string(stage)+"_done").c_str() :
        (std::string(stage)+"_failed").c_str(),result==0?"measured_joint_arrival":std::to_string(result).c_str());
    return result;
}
}


namespace {
int play_seeded_vertical_lift(Robot_Arm &arm,double lift_m,double speed,
                              const std::function<void(double)> &on_progress)
{
    const int hand=arm_hand_side(arm);
    wrist_servo::Joints actual,start;
    if(!wrist_servo::read_joint_feedback(hand,actual)) return -32;
    start=actual;wrist_servo::peek_final_hold(hand,start);
    Eigen::MatrixXd path;double rise=0;std::string detail;
    const int result=plan_seeded_vertical_lift(arm,start,lift_m,speed,path,rise,detail);
    const std::string reason="continuous_ik rise="+std::to_string(rise)+" rows="+
        std::to_string(path.rows())+" code="+std::to_string(result)+" detail="+detail;
    wrist_servo::task_event(hand,"lift_plan",reason.c_str());
    std::cout<<"[retract] hand="<<hand<<' '<<reason<<'\n';
    if(result!=0) return result;
    const auto origin=arm_tcp_from_joints(arm,actual);
    const auto progress=[&](const wrist_servo::Joints &q) {
        const auto p=arm_tcp_from_joints(arm,q);
        if(on_progress && (p.head<2>()-origin.head<2>()).norm()<=.015) on_progress(p(2)-origin(2));
    };
    const auto clearance=[&](const wrist_servo::Joints &q) {
        const auto p=arm_tcp_from_joints(arm,q);
        return p(2)>=origin(2)+rise-g_move_cfg.head_grasp.retract_arrival_tolerance_m &&
               (p.head<2>()-origin.head<2>()).norm()<=.015;
    };
    return execute_staged_motion(arm,path,"lift",progress,clearance);
}
}


namespace {
int play_joint_return(Robot_Arm &arm,const wrist_servo::Joints &ready,
                      const std::function<void()> &on_started = {},
                      double notify_at = -1,double limit_scale = 1)
{
    const int hand=arm_hand_side(arm);
    wrist_servo::Joints start;
    if(!wrist_servo::read_joint_feedback(hand,start)) return -32;
    wrist_servo::peek_final_hold(hand,start);
    Eigen::MatrixXd knots(2,7);knots.row(0)=start;knots.row(1)=ready;
    const auto &c=g_move_cfg.retract_motion;
    const double scale=std::isfinite(limit_scale) && limit_scale>1 ? limit_scale : 1;
    const auto path=t170::motion::time_joint_path(knots,k_arm_traj_dt,
        {c.max_joint_velocity_rad_s*scale,c.max_joint_acceleration_rad_s2*scale,
         c.max_joint_jerk_rad_s3*scale});
    if(path.rows()<2) return -16;
    std::cout<<"[retract] hand="<<hand<<" fixed_ready_motor_rad="<<ready
             <<" duration_s="<<(path.rows()-1)*k_arm_traj_dt
             <<" limit_scale="<<scale<<'\n';
    wrist_servo::task_event(hand,"return_plan","fixed_motor_joints_no_IK");
    if(!(notify_at>=0 && notify_at<=1)) {
        if(on_started) on_started();
        return execute_staged_motion(arm,path,"return");
    }
    return execute_staged_motion(arm,path,"return",{},{},notify_at,on_started);
}
}


int arm_vertical_lift(Robot_Arm &arm,double lift_m,double speed,
                      const std::function<void(double)> &on_progress) {
    const int hand=arm_hand_side(arm);
    if(hand==1 && right_arm_motors_locked()) return 0;
    if(!std::isfinite(lift_m) || lift_m<=0 || !std::isfinite(speed) || speed<=0) return -10;
    arm_discard_line_then_line_plan(hand);
    std::lock_guard<std::mutex> owner(g_staged_motion_mu[hand]);
    orch::TimingSpan timer("arm_vertical_lift");const int code=play_seeded_vertical_lift(arm,lift_m,speed,on_progress);timer.set_ok(code==0);return code;
}

int arm_return_after_lift(Robot_Arm &arm,const Eigen::Matrix<double,1,7> &ready,
                          const std::function<void()> &on_started,
                          double notify_at,double limit_scale) {
    if(arm_hand_side(arm)==1 && right_arm_motors_locked()) return 0;
    if(!ready.allFinite()) return -10;
    arm_discard_line_then_line_plan(arm_hand_side(arm));
    std::lock_guard<std::mutex> owner(g_staged_motion_mu[arm_hand_side(arm)]);
    orch::TimingSpan timer("arm_return");const int code=play_joint_return(arm,ready,on_started,notify_at,limit_scale);timer.set_ok(code==0);return code;
}
