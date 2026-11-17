#include "t170/common/paths.h"
#include "t170/common/logging.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "t170/motion/servo.h"
#include "t170/common/servo_trace.h"
#include "t170/common/pose_math.h"
#include "t170/motion/kinematics_adapter.h"
#include "t170/motion/feedback.h"
#include "Ti5_Arm.h"
#include "t170/hardware/device_bus.h"
#include "t170/skills/legacy/profiles.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>

#include "internal/servo_state.h"
using namespace Eigen;
using namespace std;
namespace wrist_servo {
using namespace detail;
int run_descent(Robot_Arm &arm,const Eigen::Matrix<double,1,6> &goal,
                const Joints &last_command,const Joints &previous_command,double speed) {
    const int hand=hand_index(arm); Runtime &slot=slots[hand];
    ObservationContext context;
    if(!observation_context(hand,context)) return -30;
    const Config cfg=context.config;
    const Eigen::Matrix3d rotation=TR(goal).block<3,3>(0,0);
    double read_ms=0;
    State state=measured(arm,slot,read_ms);
    const bool staged_place=context.target_class==1 && context.tray_target.valid &&
        cfg.tray_place.enabled && cfg.tray_place.observe_at_B;
    orch::TimingSpan descent_timer("servo_descent");descent_timer.fail();
    Controller controller; controller.reset(cfg,state,goal(2),speed,goal.head<2>().transpose(),
        context.target_class==0,staged_place,
        context.target_class==0 && context.task_tag.rfind("belt",0)==0);
    Joints command=last_command, joint_velocity=(last_command-previous_command)/0.005;
    Vec3 reference=arm_tcp_from_joints(arm,command).head<3>().transpose();
    const Vec3 incoming_velocity=(reference-arm_tcp_from_joints(arm,previous_command).head<3>().transpose())/0.005;
    controller.feedback_reference(reference,incoming_velocity);
    if(staged_place) {
        controller.set_observation_z(reference.z());
        task_event(hand,"place_B_observe","hold_height_align_XY_then_lower");
    }
    { std::lock_guard<std::mutex> lock(slot.mutex); slot.context.descending=true; }
    double previous_time=state.time_s, last_log=0, fault_since=-1, fault_mark=-1;
    double mark_lead=0, mark_joint=0, mark_rot=0;
    int failures=0; Phase previous_phase=Phase::Acquiring;
    PixelObservation last_observation; Mapping last_map;
    int result=-31;
    bool recovery_logged=false;
    bool extension_logged=false;
    bool belt_latched_logged=false;
    while(true) {
        const auto cycle=std::chrono::steady_clock::now();
        if(hardware_abort_requested()) { result=-4; break; }
        if(can_io_faulted()) { result=-2; break; }
        const State previous_feedback=state;
        state=measured(arm,slot,read_ms);
        const double dt=state.time_s-previous_time; previous_time=state.time_s;
        if(dt<=0) continue;
        if(std::abs(state.position.z()-previous_feedback.position.z())>std::max(.005,1.5*speed*dt)) {
            TraceRow row;row.time_s=monotonic_seconds();row.hand=hand;row.session=context.session;
            row.event="feedback_step";row.reason="FK_Z_step_exceeds_descent_profile";
            row.task_tag=context.task_tag;row.target_class=context.target_class;row.final_z=goal(2);
            row.measured=state;row.commanded=command;row.reference=reference;row.dt_ms=dt*1000;
            row.read_ms=read_ms;trace_writer().push(std::move(row));
        }
        const double rotation_error=Eigen::AngleAxisd(rotation*state.rotation.transpose()).angle();
        const double lead=(reference-state.position).norm();
        const double joint_lead=(command-state.joints).cwiseAbs().maxCoeff();
        // Early handoff leaves a large but shrinking lag. That catch-up is not a fault.
        // Abort only when an out-of-band channel stops closing, or the catch-up runs past 2s.
        const bool over_lead=lead>cfg.hard_reference_lead_m;
        const bool over_joint=joint_lead>cfg.max_joint_lead_rad;
        const bool over_rot=rotation_error>cfg.max_orientation_error_rad;
        const bool outside=over_lead || over_joint || over_rot;
        if(!outside) fault_since=-1;
        else if(fault_since<0) {
            fault_since=state.time_s; fault_mark=state.time_s;
            mark_lead=lead; mark_joint=joint_lead; mark_rot=rotation_error;
        } else if(state.time_s-fault_mark>cfg.following_timeout_s) {
            auto stalled=[&](bool over,double now,double then,double least) {
                return over && now>then-least;
            };
            const bool stuck=
                stalled(over_lead,lead,mark_lead,std::max(0.008,0.30*(mark_lead-cfg.hard_reference_lead_m))) ||
                stalled(over_joint,joint_lead,mark_joint,std::max(0.015,0.30*(mark_joint-cfg.max_joint_lead_rad))) ||
                stalled(over_rot,rotation_error,mark_rot,std::max(0.008,0.30*(mark_rot-cfg.max_orientation_error_rad)));
            if(stuck || state.time_s-fault_since>2.0)
                controller.fail(over_rot && !over_lead && !over_joint?
                    "persistent_orientation_error":"persistent_following_error");
            else {
                fault_mark=state.time_s; mark_lead=lead; mark_joint=joint_lead; mark_rot=rotation_error;
            }
        }
        if(group_failed.load()) controller.fail("peer_arm_failed");
        if(!trace_writer().healthy()) controller.fail("trace_write_failed");
        std::optional<PixelObservation> observation;
        std::optional<std::pair<double,bool>> view_status;
        State capture; bool history_ok=false;
        { std::lock_guard<std::mutex> lock(slot.mutex);
          observation=slot.pending; slot.pending.reset();
          view_status=slot.pending_view;slot.pending_view.reset();
          if(observation) history_ok=slot.history.interpolate(observation->capture_s,cfg.max_state_gap_s,capture);
        }
        if(view_status) controller.tray_view_status(view_status->first,view_status->second);
        if(observation) {
            if(history_ok && !observation->capture_timing_accounted)
                observation->clock_uncertainty_s+=capture.sampling_uncertainty_s;
            std::string reason="capture_state_unavailable";
            Mapping map;
            bool accepted=false;
            if(history_ok) {
                Eigen::Matrix4d tcp=Eigen::Matrix4d::Identity();
                tcp.block<3,3>(0,0)=capture.rotation; tcp.block<3,1>(0,3)=capture.position;
                const Eigen::Matrix4d camera=tcp*slot.tcp_to_camera;
                auto profile=context.profile;
                if(observation->virtual_target && context.target_class==1)
                    profile.feature_z_offset_m=observation->plane_z_m-goal(2);
                map=map_observation(profile,*observation,capture,goal(2),
                                    camera.block<3,1>(0,3),camera.block<3,3>(0,0));
                accepted=controller.observe(*observation,capture,map,monotonic_seconds(),reason);
            }
            {std::lock_guard<std::mutex> lock(slot.mutex);
             ++slot.metrics[accepted?"accepted_frames":"rejected_frames"];
             if(accepted)++slot.metrics[observation->vision_source=="yolo"?"yolo_accepted_frames":"tray_accepted_frames"];
             slot.metrics["inference_ms_sum"]+=observation->infer_ms;
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
        if(output.alignment_extended && !extension_logged) {
            task_event(hand,"place_alignment_extension","near_target_progress_bounded_extension");extension_logged=true;
        }
        if(staged_place && !recovery_logged && output.reason=="place_B_recovery_lift_align_XY") {
            task_event(hand,"place_observation_recovery","bounded_vertical_lift_final_C_unchanged");
            recovery_logged=true;
        }
        if(staged_place && output.phase==Phase::Commit && previous_phase!=Phase::Commit)
            task_event(hand,"place_XY_locked","lower_to_fixed_C_without_new_vision");
        if(output.phase==Phase::Commit || output.phase==Phase::Settling ||
           output.phase==Phase::Done || output.phase==Phase::Failed || controller.goal_latched()) {
            std::lock_guard<std::mutex> lock(slot.mutex); slot.context.vision_required=false;
        }
        if(controller.goal_latched() && !belt_latched_logged) {
            belt_latched_logged=true;
            std::cout << "[servo] hand=" << hand << " 传送带锁定最后一帧，共 "
                      << controller.belt_samples() << " 帧，下降合爪\n";
        }
        if(output.phase==Phase::Failed) group_failed.store(true);
        const double step_dt=std::min(dt,cfg.max_tick_gap_s);
        const Vec3 requested_velocity(output.velocity_xy.x(),output.velocity_xy.y(),output.velocity_z);
        Vec3 increment=output.reference-reference;
        Vec3 next_reference=reference+increment;
        double xyz=0, rot=0; Joints candidate=command;
        const double solve_start=monotonic_seconds();
        bool solved=false; double applied_scale=0; std::string solver_reason="local_pose_residual_or_limit";
        // Finish orientation continuously during handoff; avoid a one-tick pose jump.
        const auto command_pose=arm_tcp_from_joints(arm,command);
        const Eigen::Matrix3d command_rotation=TR(command_pose).block<3,3>(0,0);
        const double angle=Eigen::AngleAxisd(rotation*command_rotation.transpose()).angle();
        const double rotation_fraction=angle>1e-8?std::min(1.0,0.20*step_dt/angle):1.0;
        // DONE is an exact hold of the last legal joint reference, not another IK step.
        if(output.phase==Phase::Done) {
            solved=true; next_reference=reference; joint_velocity.setZero();
            solver_reason="hold_final_reference";
        }
        for(int attempt=0;attempt<8 && !solved;++attempt) {
            const double fraction=std::ldexp(1.0,-attempt);
            const Vec3 trial=reference+fraction*increment;
            const Eigen::Matrix3d trial_rotation=Eigen::Quaterniond(command_rotation).slerp(
                rotation_fraction*fraction,Eigen::Quaterniond(rotation)).toRotationMatrix();
            if(!solve_local_pose(arm,command,trial,trial_rotation,cfg,candidate,xyz,rot)) continue;
            const Joints velocity=(candidate-command)/step_dt;
            if(velocity.cwiseAbs().maxCoeff()>cfg.max_joint_speed_rad_s) { solver_reason="joint_speed_limit"; continue; }
            if(((velocity-joint_velocity)/step_dt).cwiseAbs().maxCoeff()>cfg.max_joint_accel_rad_s2) {
                solver_reason="joint_acceleration_limit"; continue;
            }
            const Vec3 achieved=arm_tcp_from_joints(arm,candidate).head<3>().transpose();
            const bool tray_place=context.target_class==1 && context.tray_target.valid && cfg.tray_place.enabled;
            if(achieved.z()<goal(2)+(context.target_class==0?cfg.grasp_height_margin_m:
                    (tray_place?cfg.place_height_margin_m:0.0))-cfg.solver_position_tolerance_m) {
                solver_reason="reference_below_safe_Z"; continue;
            }
            next_reference=achieved; joint_velocity=velocity; solved=true; applied_scale=fraction;
            solver_reason=attempt==0?"accepted":"scaled_"+solver_reason; break;
        }
        const double solve_ms=(monotonic_seconds()-solve_start)*1000;
        if(solved) {
            const Vec3 applied_velocity=(next_reference-reference)/step_dt;
            controller.feedback_reference(next_reference,applied_velocity);
            output.velocity_xy=applied_velocity.head<2>(); output.velocity_z=applied_velocity.z();
            failures=0; command=candidate; reference=next_reference; send(hand,command);
        } else {
            controller.feedback_reference(reference,Vec3::Zero());
            output.velocity_xy.setZero(); output.velocity_z=0;
            ++failures; joint_velocity.setZero();
            if(failures>=cfg.max_solver_failures) {
                controller.fail("local_solver_or_joint_limits");
                output.phase=Phase::Failed; output.reason="local_solver_or_joint_limits";
                group_failed.store(true);
            }
        }
        if(output.phase==Phase::Done && (!solved || rotation_error>cfg.max_orientation_error_rad)) {
            controller.fail("final_reference_or_orientation_not_ready");
            output.phase=Phase::Failed; output.reason="final_reference_or_orientation_not_ready";
            group_failed.store(true);
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
            if(result!=0) { std::lock_guard<std::mutex> lock(slot.mutex); slot.failure_reason=output.reason; }
            if(result==0) {
                std::lock_guard<std::mutex> lock(slot.mutex);
                slot.completed_goal=controller.goal_xy();
                slot.final_command=command;
                slot.final_hold=command.allFinite();
            }
            break;
        }
        std::this_thread::sleep_until(cycle+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(cfg.period_s)));
    }
    // Hold the last accepted nearby reference; never send the old, uncorrected final waypoint.
    if(!hardware_abort_requested() && !can_io_faulted()) send(hand,command);
    if(result!=0) {
        { std::lock_guard<std::mutex> lock(slot.mutex);
          if(slot.failure_reason.empty()) slot.failure_reason=result==-4?"hardware_abort":
              (result==-2?"CAN_fault":"execution_failed");
        }
        group_failed.store(true);
        observation_event(hand,context.session,"motion_failed",last_observation);
    }
    descent_timer.set_ok(result==0);
    return result;
}
}
