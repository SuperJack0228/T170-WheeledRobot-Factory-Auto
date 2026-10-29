#include "servo/runtime.h"
#include "servo/trace.h"
#include "function.h"
#include "Ti5_socketcan.h"
#include "hand_shadow.h"
#include "move_box_config.h"
#include "seg_pose_bridge.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
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
    bool final_hold = false;
    Joints final_command = Joints::Zero();
    Vec2 completed_goal = Vec2::Zero();
    std::string failure_reason;
    TrayTarget staged_tray_target;
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

void prepare_group() {
    group_failed.store(false);
    for(auto &slot:slots) {
        std::lock_guard<std::mutex> lock(slot.mutex);
        slot.failure_reason.clear();
        slot.final_hold=false;
    }
}
bool group_is_failed() { return group_failed.load(); }
std::string last_failure(int hand) {
    if(hand<0 || hand>1) return "invalid_hand";
    std::lock_guard<std::mutex> lock(slots[hand].mutex);
    return slots[hand].failure_reason;
}

bool begin_session(Robot_Arm &arm, const Eigen::Matrix<double,1,6> &goal, int cls,const char *tag) {
    const int hand=hand_index(arm);
    if(hand<0 || hand>1) return false;
    if (!trace_writer().start((std::filesystem::path(project_root_dir())/"logs"/"visual_servo").string())) {
        std::cerr << "[servo] 无法创建调试日志，取消视觉动作\n"; return false;
    }
    const Config cfg=g_move_cfg.visual_servo;
    const auto &profile=profile_for(cfg,hand,cls);
    bool tray_placement=false;
    { std::lock_guard<std::mutex> lock(slots[hand].mutex);
      tray_placement=cls==1 && cfg.tray_place.enabled && slots[hand].staged_tray_target.valid;
    }
    Eigen::Matrix4d eye=Eigen::Matrix4d::Identity();
    if (profile.mode=="geometry" || tray_placement) {
        std::array<double,16> raw; std::string error;
        const auto path=hand==1?default_right_hand_to_robot_yaml_path():default_left_hand_to_robot_yaml_path();
        if(!load_cam2robot_matrix(path,raw,error)) { std::cerr << "[servo] " << error << '\n'; return false; }
        for(int r=0;r<4;++r) for(int c=0;c<4;++c) eye(r,c)=raw[r*4+c];
    }
    Runtime &slot=slots[hand];
    {
        std::lock_guard<std::mutex> lock(slot.mutex);
        slot.context={}; slot.history.clear(); slot.pending.reset(); slot.succeeded=false;
        slot.failure_reason.clear();
        slot.context.active=true; slot.context.session=next_session.fetch_add(1);
        slot.context.desired_rotation=TR(goal).block<3,3>(0,0);
        slot.context.final_z=goal(2); slot.context.target_class=cls; slot.context.profile=profile;
        slot.context.task_tag=tag?tag:"descent";
        slot.context.nominal_xy=goal.head<2>().transpose();
        slot.context.config=cfg;
        if(cls==1) slot.context.tray_target=slot.staged_tray_target;
        slot.staged_tray_target={};
        slot.context.image_width=g_move_cfg.cameras.width; slot.context.image_height=g_move_cfg.cameras.height;
        slot.tcp_to_camera=tool_transform(arm).inverse()*eye;
    }
    std::uint64_t session;
    { std::lock_guard<std::mutex> lock(slot.mutex); session=slot.context.session; }
    std::string snapshot_error;
    const std::string prefix=trace_writer().path()+".session_"+std::to_string(session);
    if(!write_config_snapshot(cfg,prefix+".params.yaml",snapshot_error))
        throw std::runtime_error("servo_config_snapshot_failed: "+snapshot_error);
    // Preserve on-disk model for diagnosis. It is explicitly not proof of the SDK's
    // in-memory model if someone edits the YAML after process startup.
    std::error_code copy_error;
    std::filesystem::copy_file(std::filesystem::path(project_root_dir())/"config"/"Robot_Arm_Model.yaml",
        prefix+".arm_model_on_disk.yaml",std::filesystem::copy_options::overwrite_existing,copy_error);
    if(copy_error) std::cerr << "[servo] 模型文件备份失败: " << copy_error.message() << '\n';
    ObservationContext saved;observation_context(hand,saved);
    if(cls==1 && cfg.tray_place.enabled && saved.tray_target.valid) {
        const auto &target=saved.tray_target;const auto &board=target.board;
        std::ofstream snapshot(prefix+".tray_target.yaml");snapshot<<std::setprecision(17);
        snapshot<<"target_row: "<<target.row<<"\ntarget_col: "<<target.col
                <<"\nrows: "<<board.rows<<"\ncols: "<<board.cols<<"\npitch_m: "<<board.pitch_m
                <<"\nradius_m: "<<board.radius_m<<"\nbase_from_tray:\n";
        for(int r=0;r<4;++r) {
            snapshot<<"  - [";
            for(int c=0;c<4;++c) snapshot<<(c ? ", " : "")<<target.base_from_tray(r,c);
            snapshot<<"]\n";
        }
        snapshot.flush();
        if(!snapshot) throw std::runtime_error("tray_target_snapshot_failed");
        const auto board_path=std::filesystem::path(cfg.tray_place.board_config);
        std::filesystem::copy_file(board_path.is_absolute() ? board_path : std::filesystem::path(project_root_dir())/board_path,
            prefix+".tray_board_on_disk.yaml",std::filesystem::copy_options::overwrite_existing,copy_error);
        if(copy_error) std::cerr<<"[tray_place] board snapshot: "<<copy_error.message()<<'\n';
    }
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
    if(!success && slot.failure_reason.empty()) slot.failure_reason="approach_or_execution_failed";
    if(!success) group_failed.store(true);
}

bool observation_context(int hand, ObservationContext &context) {
    if(hand<0 || hand>1) return false;
    std::lock_guard<std::mutex> lock(slots[hand].mutex);
    context=slots[hand].context; return context.active;
}

void stage_tray_target(int hand,const TrayTarget &target) {
    if(hand<0 || hand>1) return;
    std::lock_guard<std::mutex> lock(slots[hand].mutex);
    slots[hand].staged_tray_target=target;
}
bool camera_at_capture(int hand,std::uint64_t session,double time,Eigen::Matrix4d &base_camera) {
    if(hand<0 || hand>1) return false;
    auto &slot=slots[hand]; std::lock_guard<std::mutex> lock(slot.mutex);
    State capture;
    if(!slot.context.active || slot.context.session!=session ||
       !slot.history.interpolate(time,slot.context.config.max_state_gap_s,capture)) return false;
    Eigen::Matrix4d tcp=Eigen::Matrix4d::Identity();
    tcp.block<3,3>(0,0)=capture.rotation; tcp.block<3,1>(0,3)=capture.position;
    base_camera=tcp*slot.tcp_to_camera; return base_camera.allFinite();
}

bool submit_observation(int hand, const PixelObservation &o) {
    if(hand<0 || hand>1) return false;
    auto &slot=slots[hand]; std::lock_guard<std::mutex> lock(slot.mutex);
    if(!slot.context.active || !slot.context.descending || !slot.context.vision_required ||
       o.session!=slot.context.session) return false;
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

bool ready_for_gripper(Robot_Arm &arm) {
    const int hand=hand_index(arm); auto &slot=slots[hand];
    Vec2 goal; Config cfg; double final_z; bool succeeded,place; Eigen::Matrix3d rotation;
    { std::lock_guard<std::mutex> lock(slot.mutex);
      succeeded=slot.succeeded; goal=slot.completed_goal;
      cfg=slot.context.config; final_z=slot.context.final_z;
      rotation=slot.context.desired_rotation;
      place=slot.context.target_class==1;
    }
    if(!succeeded || hardware_abort_requested() || can_io_faulted() || group_failed.load()) return false;
    double read_ms=0; const auto state=measured(arm,slot,read_ms);
    const bool ready=(state.position.head<2>()-goal).norm()<=(place ? cfg.xy_tolerance_m : cfg.grasp_xy_accept_m) &&
        state.position.z()>=final_z-(place ? cfg.z_tolerance_m : 0) &&
        state.position.z()<=final_z+(place ? cfg.z_tolerance_m : cfg.grasp_z_accept_above_m) &&
        Eigen::AngleAxisd(rotation*state.rotation.transpose()).angle()<=cfg.max_orientation_error_rad;
    TraceRow row; row.time_s=monotonic_seconds(); row.hand=hand;
    { std::lock_guard<std::mutex> lock(slot.mutex);
      row.session=slot.context.session; row.task_tag=slot.context.task_tag; row.target_class=slot.context.target_class;
    }
    row.event="gripper_gate"; row.reason=ready?"ready":"pose_changed_after_done";
    row.measured=state; row.final_z=final_z; row.goal=goal;
    row.control.remaining_m=goal-state.position.head<2>(); trace_writer().push(std::move(row));
    if(!ready) {
        { std::lock_guard<std::mutex> lock(slot.mutex); slot.failure_reason="gripper_gate_pose_changed"; }
        invalidate_result(hand); group_failed.store(true);
    }
    return ready;
}

void repeat_final_hold(int hand) {
    if(hand<0 || hand>1 || hardware_abort_requested() || can_io_faulted()) return;
    // Same lock as take_final_hold: it fences the ENTIRE seven-joint send.
    // Once take returns, no sender can still own a cached old hold command.
    std::lock_guard<std::mutex> lock(slots[hand].mutex);
    if(!slots[hand].final_hold) return;
    try { send(hand,slots[hand].final_command); }
    catch(const std::exception &ex) {
        std::cerr << "[servo] 保持最终关节指令失败 hand=" << hand << " " << ex.what() << "\n";
        slots[hand].final_hold=false;
    }
}

void start_final_hold_repeat() {
    struct Repeater {
        std::atomic<bool> running{true};
        std::thread worker;
        Repeater():worker([this] {
            while (running.load()) {
                const auto tick = std::chrono::steady_clock::now();
                bool any = false;
                for (int hand = 0; hand < 2; ++hand) {
                    bool hold = false;
                    { std::lock_guard<std::mutex> lock(slots[hand].mutex); hold = slots[hand].final_hold; }
                    if (!hold) continue;
                    any = true;
                    repeat_final_hold(hand);
                }
                std::this_thread::sleep_until(tick + std::chrono::milliseconds(any ? 5 : 20));
            }
        }) {}
        ~Repeater() { running.store(false); if(worker.joinable()) worker.join(); }
    };
    static Repeater repeater;
}

bool peek_final_hold(int hand, Joints &q) {
    if (hand < 0 || hand > 1) return false;
    std::lock_guard<std::mutex> lock(slots[hand].mutex);
    if (!slots[hand].final_hold || !slots[hand].final_command.allFinite()) return false;
    q = slots[hand].final_command;
    return true;
}

bool take_final_hold(int hand, Joints &q) {
    if (hand < 0 || hand > 1) return false;
    std::lock_guard<std::mutex> lock(slots[hand].mutex);
    if (!slots[hand].final_hold) return false;
    q = slots[hand].final_command;
    slots[hand].final_hold = false;
    return q.allFinite();
}

void handoff_event(Robot_Arm &arm,const Joints &command,const Joints &previous,const char *reason) {
    const int hand=hand_index(arm); double read_ms=0;
    const auto state=measured(arm,slots[hand],read_ms);
    const Vec3 reference=arm_tcp_from_joints(arm,command).head<3>().transpose();
    TraceRow row; row.time_s=monotonic_seconds(); row.hand=hand;
    { std::lock_guard<std::mutex> lock(slots[hand].mutex);
      row.session=slots[hand].context.session; row.final_z=slots[hand].context.final_z;
      row.task_tag=slots[hand].context.task_tag; row.target_class=slots[hand].context.target_class;
    }
    row.event="handoff"; row.reason=reason; row.measured=state; row.commanded=command; row.reference=reference;
    row.requested_velocity=(reference-arm_tcp_from_joints(arm,previous).head<3>().transpose())/.005;
    row.reference_lead_m=(reference-state.position).norm();
    row.joint_lead_rad=(command-state.joints).cwiseAbs().maxCoeff(); row.read_ms=read_ms;
    trace_writer().push(std::move(row));
}

void task_event(int hand,const char *event,const char *reason) {
    if(hand<0 || hand>1) return;
    auto &slot=slots[hand]; TraceRow row;
    { std::lock_guard<std::mutex> lock(slot.mutex);
      row.session=slot.context.session; row.measured=slot.context.latest;
      row.final_z=slot.context.final_z; row.task_tag=slot.context.task_tag;
      row.target_class=slot.context.target_class; row.profile_mode=slot.context.profile.mode;
    }
    row.time_s=monotonic_seconds(); row.hand=hand; row.event=event; row.reason=reason;
    trace_writer().push(std::move(row));
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
    Controller controller; controller.reset(cfg,state,goal(2),speed,goal.head<2>().transpose(),context.target_class==0);
    Joints command=last_command, joint_velocity=(last_command-previous_command)/0.005;
    Vec3 reference=arm_tcp_from_joints(arm,command).head<3>().transpose();
    const Vec3 incoming_velocity=(reference-arm_tcp_from_joints(arm,previous_command).head<3>().transpose())/0.005;
    controller.feedback_reference(reference,incoming_velocity);
    { std::lock_guard<std::mutex> lock(slot.mutex); slot.context.descending=true; }
    double previous_time=state.time_s, last_log=0, fault_since=-1, fault_mark=-1;
    double mark_lead=0, mark_joint=0, mark_rot=0;
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
                auto profile=context.profile;
                if(observation->virtual_target && context.target_class==1)
                    profile.feature_z_offset_m=observation->plane_z_m-goal(2);
                map=map_observation(profile,*observation,capture,goal(2),
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
        if(output.phase==Phase::Commit || output.phase==Phase::Settling || output.phase==Phase::Done || output.phase==Phase::Failed) {
            std::lock_guard<std::mutex> lock(slot.mutex); slot.context.vision_required=false;
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
            if(achieved.z()<goal(2)+(context.target_class==0?cfg.grasp_height_margin_m:0.0)-cfg.solver_position_tolerance_m) {
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
    return result;
}
} // namespace wrist_servo
