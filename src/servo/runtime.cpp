#include "servo/runtime.h"
#include "servo/trace.h"
#include "function.h"
#include "Ti5_socketcan.h"
#include "hand_shadow.h"
#include "move_box_config.h"
#include "seg_pose_bridge.h"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>

namespace wrist_servo {
namespace {
struct Runtime {
    std::mutex mutex;
    ObservationContext context;
    StateHistory history;
    std::optional<PixelObservation> pending;
    Eigen::Matrix4d tcp_to_camera = Eigen::Matrix4d::Identity();
    bool succeeded = false;
    Vec2 completed_goal = Vec2::Zero();
};
Runtime slots[2];
std::atomic<std::uint64_t> next_session{1};
std::atomic<bool> group_failed{false};

State measured(Robot_Arm &arm, Runtime &slot, double &read_ms) {
    const double before = monotonic_seconds();
    Joints q;
    if(!read_joint_feedback(hand_index(arm),q)) throw std::runtime_error("joint_feedback_read_failed");
    const double after = monotonic_seconds();
    State s; s.time_s = 0.5*(before+after); s.joints=q;
    s.sampling_uncertainty_s=0.5*(after-before)+slot.context.config.feedback_age_budget_s;
    const auto pose = arm_tcp_from_joints(arm,q);
    if(!pose.allFinite()) throw std::runtime_error("nonfinite_forward_kinematics");
    s.position = pose.head<3>().transpose();
    s.rotation = TR(pose).block<3,3>(0,0);
    read_ms=(after-before)*1000;
    std::lock_guard<std::mutex> lock(slot.mutex);
    if (slot.context.latest.time_s > 0) {
        const double dt=s.time_s-slot.context.latest.time_s;
        if (dt>1e-5 && dt<0.2) {
            const Vec3 raw=(s.position-slot.context.latest.position)/dt;
            s.velocity=0.4*raw+0.6*slot.context.latest.velocity;
        }
    }
    slot.history.push(s); slot.context.latest=s;
    Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0)=s.rotation; T.block<3,1>(0,3)=s.position;
    const Eigen::Matrix4d camera=T*slot.tcp_to_camera;
    slot.context.camera_origin=camera.block<3,1>(0,3);
    slot.context.camera_rotation=camera.block<3,3>(0,0);
    return s;
}

bool pair_too_close(int hand, const State &s, const Vec2 &goal, double clearance) {
    Runtime &other=slots[1-hand];
    std::lock_guard<std::mutex> lock(other.mutex);
    if (!other.context.active || !other.context.descending) return false;
    if (std::abs(other.context.latest.position.z()-s.position.z())>0.08) return false;
    return (other.context.latest.position.head<2>()-s.position.head<2>()).norm()<clearance ||
           (other.context.latest.position.head<2>()-goal).norm()<clearance;
}

void send(int hand, const Joints &q) {
    send_joint_reference(hand,q);
}
}

void prepare_group() { group_failed.store(false); }
bool group_is_failed() { return group_failed.load(); }

bool begin_session(Robot_Arm &arm, const Eigen::Matrix<double,1,6> &goal, int cls,const char *tag) {
    const int hand=hand_index(arm);
    if(hand<0 || hand>1) return false;
    if (!trace_writer().start((std::filesystem::path(project_root_dir())/"logs"/"visual_servo").string())) {
        std::cerr << "[servo] 无法创建调试日志，取消视觉动作\n"; return false;
    }
    const Config cfg=g_move_cfg.visual_servo;
    const auto &profile=profile_for(cfg,hand,cls);
    Eigen::Matrix4d eye=Eigen::Matrix4d::Identity();
    if (profile.mode=="geometry") {
        std::array<double,16> raw; std::string error;
        const auto path=hand==1?default_right_hand_to_robot_yaml_path():default_left_hand_to_robot_yaml_path();
        if(!load_cam2robot_matrix(path,raw,error)) { std::cerr << "[servo] " << error << '\n'; return false; }
        for(int r=0;r<4;++r) for(int c=0;c<4;++c) eye(r,c)=raw[r*4+c];
    }
    Runtime &slot=slots[hand];
    {
        std::lock_guard<std::mutex> lock(slot.mutex);
        slot.context={}; slot.history.clear(); slot.pending.reset(); slot.succeeded=false;
        slot.context.active=true; slot.context.session=next_session.fetch_add(1);
        slot.context.final_z=goal(2); slot.context.target_class=cls; slot.context.profile=profile;
        slot.context.task_tag=tag?tag:"descent";
        slot.context.nominal_xy=goal.head<2>().transpose();
        slot.context.config=cfg;
        slot.context.image_width=g_move_cfg.cameras.width; slot.context.image_height=g_move_cfg.cameras.height;
        slot.tcp_to_camera=tool_transform(arm).inverse()*eye;
    }
    std::uint64_t session;
    { std::lock_guard<std::mutex> lock(slot.mutex); session=slot.context.session; }
    std::string snapshot_error;
    if(!write_config_snapshot(cfg,trace_writer().path()+".session_"+std::to_string(session)+".params.yaml",snapshot_error))
        throw std::runtime_error("servo_config_snapshot_failed: "+snapshot_error);
    double read_ms=0; const auto state=measured(arm,slot,read_ms);
    TraceRow row; row.time_s=monotonic_seconds(); row.hand=hand; row.event="begin";
    { std::lock_guard<std::mutex> lock(slot.mutex); row.session=slot.context.session; }
    row.reason="approach"; row.profile_mode=profile.mode; row.final_z=goal(2); row.measured=state;
    row.target_class=cls; row.task_tag=tag?tag:"descent";
    trace_writer().push(row);
    std::cout << "[servo] hand=" << hand << " session=" << row.session << " map=" << profile.mode
              << " kp=" << cfg.kp << " kd=" << cfg.kd << " 双轴闭环；失败禁止夹爪动作\n";
    return !can_io_faulted();
}

void sample_approach_state(Robot_Arm &arm) {
    const int hand=hand_index(arm); double ms=0;
    measured(arm,slots[hand],ms);
}

void end_session(int hand, bool success) {
    if(hand<0 || hand>1) return;
    auto &slot=slots[hand];
    std::lock_guard<std::mutex> lock(slot.mutex);
    slot.context.active=false; slot.context.descending=false; slot.pending.reset();
    slot.succeeded=success;
    if(!success) group_failed.store(true);
}

bool observation_context(int hand, ObservationContext &context) {
    if(hand<0 || hand>1) return false;
    std::lock_guard<std::mutex> lock(slots[hand].mutex);
    context=slots[hand].context; return context.active;
}

bool submit_observation(int hand, const PixelObservation &o) {
    if(hand<0 || hand>1) return false;
    auto &slot=slots[hand]; std::lock_guard<std::mutex> lock(slot.mutex);
    if(!slot.context.active || !slot.context.vision_required || o.session!=slot.context.session) return false;
    if(slot.pending && o.capture_s<=slot.pending->capture_s) return false;
    slot.pending=o; return true;
}

void observation_event(int hand,std::uint64_t session,const char *reason,const PixelObservation &o) {
    TraceRow row; row.time_s=monotonic_seconds(); row.hand=hand; row.session=session;
    row.event="vision"; row.reason=reason; row.observation=o;
    ObservationContext context;
    observation_context(hand,context);
    if(context.session==session) {
        row.measured=context.latest; row.final_z=context.final_z; row.profile_mode=context.profile.mode;
        row.target_class=context.target_class; row.task_tag=context.task_tag;
    }
    trace_writer().push(std::move(row));
}

bool completed_xy(int hand,double &x,double &y) {
    if(hand<0 || hand>1) return false;
    auto &slot=slots[hand]; std::lock_guard<std::mutex> lock(slot.mutex);
    if(!slot.succeeded) return false;
    x=slot.completed_goal.x(); y=slot.completed_goal.y(); return true;
}

void invalidate_result(int hand) {
    if(hand<0 || hand>1) return;
    std::lock_guard<std::mutex> lock(slots[hand].mutex); slots[hand].succeeded=false;
}

void record_gripper_check(int hand,double position,double torque,const char *outcome) {
    if(hand<0 || hand>1) return;
    TraceRow row;
    { std::lock_guard<std::mutex> lock(slots[hand].mutex);
      if(!slots[hand].succeeded) return;
      row.session=slots[hand].context.session; row.measured=slots[hand].context.latest;
      row.profile_mode=slots[hand].context.profile.mode; row.final_z=slots[hand].context.final_z;
      row.target_class=slots[hand].context.target_class; row.task_tag=slots[hand].context.task_tag;
    }
    row.time_s=monotonic_seconds(); row.hand=hand; row.event="gripper_check"; row.reason=outcome;
    row.gripper_position_rad=position; row.gripper_torque_nm=torque;
    trace_writer().push(std::move(row));
}

int run_descent(Robot_Arm &arm,const Eigen::Matrix<double,1,6> &goal,
                const Joints &last_command,const Joints &previous_command,double speed) {
    const int hand=hand_index(arm); Runtime &slot=slots[hand];
    ObservationContext context;
    if(!observation_context(hand,context)) return -30;
    const Config cfg=context.config;
    const Eigen::Matrix3d rotation=TR(goal).block<3,3>(0,0);
    double read_ms=0;
    State state=measured(arm,slot,read_ms);
    Controller controller; controller.reset(cfg,state,goal(2),speed);
    Joints command=last_command, joint_velocity=(last_command-previous_command)/0.005;
    Vec3 reference=arm_tcp_from_joints(arm,command).head<3>().transpose();
    { std::lock_guard<std::mutex> lock(slot.mutex); slot.context.descending=true; }
    double previous_time=state.time_s, last_log=0;
    int failures=0; Phase previous_phase=Phase::Acquiring;
    PixelObservation last_observation; Mapping last_map;
    int result=-31;
    while(true) {
        const auto cycle=std::chrono::steady_clock::now();
        if(hardware_abort_requested()) { result=-4; break; }
        if(can_io_faulted()) { result=-2; break; }
        state=measured(arm,slot,read_ms);
        const double dt=state.time_s-previous_time; previous_time=state.time_s;
        if(dt<=0) continue;
        const double rotation_error=Eigen::AngleAxisd(rotation*state.rotation.transpose()).angle();
        if(rotation_error>cfg.max_orientation_error_rad) controller.fail("orientation_outside_envelope");
        if((command-state.joints).cwiseAbs().maxCoeff()>cfg.max_joint_lead_rad ||
           (reference-state.position).norm()>cfg.max_reference_lead_m)
            controller.fail("following_error");
        if(group_failed.load()) controller.fail("peer_arm_failed");
        if(!trace_writer().healthy()) controller.fail("trace_write_failed");
        std::optional<PixelObservation> observation;
        State capture; bool history_ok=false;
        { std::lock_guard<std::mutex> lock(slot.mutex);
          observation=slot.pending; slot.pending.reset();
          if(observation) history_ok=slot.history.interpolate(observation->capture_s,cfg.max_state_gap_s,capture);
        }
        if(observation) {
            if(history_ok) observation->clock_uncertainty_s+=capture.sampling_uncertainty_s;
            std::string reason="capture_state_unavailable";
            Mapping map;
            bool accepted=false;
            if(history_ok) {
                Eigen::Matrix4d tcp=Eigen::Matrix4d::Identity();
                tcp.block<3,3>(0,0)=capture.rotation; tcp.block<3,1>(0,3)=capture.position;
                const Eigen::Matrix4d camera=tcp*slot.tcp_to_camera;
                map=map_observation(context.profile,*observation,capture,goal(2),
                                    camera.block<3,1>(0,3),camera.block<3,3>(0,0));
                accepted=controller.observe(*observation,capture,map,monotonic_seconds(),reason);
            }
            if(accepted) { last_observation=*observation; last_map=map; }
            TraceRow row; row.time_s=monotonic_seconds(); row.session=context.session; row.hand=hand;
            row.event="observation"; row.reason=reason; row.profile_mode=context.profile.mode;
            row.target_class=context.target_class; row.task_tag=context.task_tag;
            row.observation=*observation; row.mapping=map; row.measured=capture;
            row.final_z=goal(2); row.goal=controller.goal_xy(); trace_writer().push(row);
        }
        if(controller.has_goal() && pair_too_close(hand,state,controller.goal_xy(),cfg.pair_clearance_m))
            controller.fail("dual_arm_clearance");
        Output output=controller.tick(state,dt);
        if(output.phase==Phase::Commit || output.phase==Phase::Done || output.phase==Phase::Failed) {
            std::lock_guard<std::mutex> lock(slot.mutex); slot.context.vision_required=false;
        }
        if(output.phase==Phase::Failed) group_failed.store(true);
        const double step_dt=std::min(dt,cfg.max_tick_gap_s);
        const Vec3 requested_velocity(output.velocity_xy.x(),output.velocity_xy.y(),output.velocity_z);
        Vec3 increment(output.velocity_xy.x()*step_dt,output.velocity_xy.y()*step_dt,output.velocity_z*step_dt);
        Vec3 next_reference=reference+increment;
        double xyz=0, rot=0; Joints candidate=command;
        const double solve_start=monotonic_seconds();
        bool solved=false; double applied_scale=0; std::string solver_reason="local_pose_residual_or_limit";
        for(int attempt=0;attempt<5;++attempt) {
            const double fraction=std::ldexp(1.0,-attempt);
            const Vec3 trial=reference+fraction*increment;
            if(!solve_local_pose(arm,command,trial,rotation,cfg,candidate,xyz,rot)) continue;
            const Joints velocity=(candidate-command)/step_dt;
            if(velocity.cwiseAbs().maxCoeff()>cfg.max_joint_speed_rad_s) { solver_reason="joint_speed_limit"; continue; }
            if(((velocity-joint_velocity)/step_dt).cwiseAbs().maxCoeff()>cfg.max_joint_accel_rad_s2) {
                solver_reason="joint_acceleration_limit"; continue;
            }
            next_reference=trial; joint_velocity=velocity; solved=true; applied_scale=fraction;
            solver_reason=attempt==0?"accepted":"scaled_"+solver_reason; break;
        }
        const double solve_ms=(monotonic_seconds()-solve_start)*1000;
        if(solved) {
            const Vec3 applied_velocity=(next_reference-reference)/step_dt;
            controller.feedback_applied_velocity(applied_velocity.head<2>(),applied_velocity.z());
            output.velocity_xy=applied_velocity.head<2>(); output.velocity_z=applied_velocity.z();
            failures=0; command=candidate; reference=next_reference; send(hand,command);
        } else {
            controller.feedback_applied_velocity(Vec2::Zero(),0);
            output.velocity_xy.setZero(); output.velocity_z=0;
            ++failures; joint_velocity.setZero();
            if(failures>=cfg.max_solver_failures) {
                controller.fail("local_solver_or_joint_limits");
                output.phase=Phase::Failed; output.reason="local_solver_or_joint_limits";
                group_failed.store(true);
            }
        }
        if(output.phase!=previous_phase) {
            std::cout << "[servo] hand=" << hand << " session=" << context.session
                      << " " << phase_name(previous_phase) << " -> " << phase_name(output.phase)
                      << " err_mm=" << output.remaining_m.norm()*1000 << " age_ms=" << output.age_s*1000
                      << " reason=" << output.reason << std::endl;
        }
        const bool terminal=output.phase==Phase::Done ||
            (output.phase==Phase::Failed && output.velocity_xy.norm()<0.0005 && std::abs(output.velocity_z)<0.0005) ||
            failures>=cfg.max_solver_failures;
        if(state.time_s-last_log>=cfg.log_period_s || observation || output.phase!=previous_phase || terminal) {
            TraceRow row; row.time_s=monotonic_seconds(); row.session=context.session; row.hand=hand;
            row.event=terminal?"end":"control"; row.reason=output.reason; row.profile_mode=context.profile.mode;
            row.target_class=context.target_class; row.task_tag=context.task_tag;
            row.observation=last_observation; row.mapping=last_map; row.measured=state; row.control=output;
            row.goal=controller.goal_xy(); row.reference=reference; row.commanded=command; row.final_z=goal(2);
            row.dt_ms=dt*1000; row.read_ms=read_ms; row.solve_ms=solve_ms;
            row.solver_xyz_m=xyz; row.solver_rotation_rad=rot; row.solver_failures=failures;
            row.actual_rotation_error_rad=rotation_error; row.reference_lead_m=(reference-state.position).norm();
            row.joint_lead_rad=(command-state.joints).cwiseAbs().maxCoeff();
            row.requested_velocity=requested_velocity; row.applied_scale=applied_scale; row.solver_reason=solver_reason;
            trace_writer().push(row); last_log=state.time_s;
        }
        previous_phase=output.phase;
        if(terminal) {
            result=output.phase==Phase::Done && solved ? 0 : -31;
            if(result==0) { std::lock_guard<std::mutex> lock(slot.mutex); slot.completed_goal=controller.goal_xy(); }
            break;
        }
        std::this_thread::sleep_until(cycle+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(cfg.period_s)));
    }
    // Hold the last accepted nearby reference; never send the old, uncorrected final waypoint.
    if(!hardware_abort_requested() && !can_io_faulted()) send(hand,command);
    if(result!=0) {
        group_failed.store(true);
        observation_event(hand,context.session,"motion_failed",last_observation);
    }
    return result;
}
} // namespace wrist_servo
