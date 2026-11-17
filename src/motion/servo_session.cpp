#include "t170/common/paths.h"
#include "t170/common/logging.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "t170/motion/servo.h"
#include "t170/common/servo_trace.h"
#include "t170/common/pose_math.h"
#include "t170/motion/kinematics_adapter.h"
#include "t170/motion/feedback.h"
#include "t170/motion/arm.h"
#include "t170/hardware/device_bus.h"
#include "t170/skills/legacy/profiles.h"
#include "t170/vision/python_bridge.h"

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
void prepare_group(bool move_r,bool move_l) {
    group_failed.store(false);
    for(int hand=0;hand<2;++hand) {
        if(!(hand==1 ? move_r : move_l)) continue;
        auto &slot=slots[hand];
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
bool begin_session(Arm &arm, const Eigen::Matrix<double,1,6> &goal, int cls,const char *tag) {
    const int hand=hand_index(arm);
    if(hand<0 || hand>1) return false;
    if (!trace_writer().start(t170::logging::file("visual_servo").string())) {
        std::cerr << "[servo] 无法创建调试日志，取消视觉动作\n"; return false;
    }
    Config cfg=g_move_cfg.visual_servo;
    auto profile=profile_for(cfg,hand,cls);
    bool tray_placement=false;
    bool belt_reference=false;
    std::optional<double> tray_grasp_plane;
    { std::lock_guard<std::mutex> lock(slots[hand].mutex);
      tray_placement=cls==1 && cfg.tray_place.enabled && slots[hand].staged_tray_target.valid;
      tray_grasp_plane=slots[hand].staged_tray_grasp_plane;
      belt_reference=slots[hand].staged_belt_grasp_reference;
      slots[hand].staged_tray_grasp_plane.reset();
      slots[hand].staged_belt_grasp_reference=false;
    }
    if(cls==0 && tray_grasp_plane && apply_tray_calibration(cfg,hand,cls,*tray_grasp_plane,goal(2),goal(3),goal(4)))
        profile=profile_for(cfg,hand,cls);
    else if(cls==0 && belt_reference) {
        if(const auto *taught=calibrated_tray_grasp_for(cfg,hand)) {
            auto &dst=cfg.profiles[hand];
            dst.reference_px=taught->image.reference_px;
            dst.plane_relative_reference=taught->image.plane_relative_reference;
            dst.reference_tcp_above_plane_m=taught->image.reference_tcp_above_plane_m;
            dst.reference_height_m=taught->image.reference_height_m;
            profile=dst;
            std::cout<<"[servo] 传送带抓取共用料盘参考点 ("<<dst.reference_px.x()<<","
                     <<dst.reference_px.y()<<")，不使用料盘XY补偿\n";
        } else std::cerr<<"[servo] 传送带抓取没有该手的料盘参考点，沿用夹爪像素\n";
    }
    if(tray_placement) {
        if(const auto *calibrated=calibrated_placement_for(cfg,hand,goal(3),goal(4))) {
            profile=calibrated->image;cfg.profiles[2+hand]=profile;
        } else if(!cfg.calibrated_placement[hand].empty())
            std::cerr<<"[servo] 本放置倾角尚未示教，沿用move_box_params中的显式参考，不套用其他倾角标定\n";
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
        slot.telemetry=orch::timing_context();slot.metrics.clear();slot.began_s=monotonic_seconds();
        slot.context={}; slot.history.clear(); slot.pending.reset();slot.pending_view.reset(); slot.succeeded=false;
        slot.failure_reason.clear();
        slot.context.active=true; slot.context.session=next_session.fetch_add(1);
        slot.context.desired_rotation=TR(goal).block<3,3>(0,0);
        slot.context.final_z=goal(2); slot.context.target_class=cls; slot.context.profile=profile;
        slot.context.task_tag=tag?tag:"descent";
        if(cls==0 && tag && std::strncmp(tag,"belt",4)==0)
            cfg.min_confidence=cfg.belt_min_confidence;
        slot.context.nominal_xy=goal.head<2>().transpose();
        slot.context.config=cfg;
        if(cls==1) slot.context.tray_target=slot.staged_tray_target;
        slot.staged_tray_target={};
        slot.context.image_width=g_move_cfg.cameras.width; slot.context.image_height=g_move_cfg.cameras.height;
        slot.tcp_to_camera=tool_transform(arm).inverse()*eye;
    }
    std::uint64_t session;
    { std::lock_guard<std::mutex> lock(slot.mutex); session=slot.context.session; }
    orch::timing_event("servo_start",tag?tag:"descent","running",trace_writer().path(),{{"hand",double(hand)},{"servo_session",double(session)},{"target_class",double(cls)}},&slot.telemetry);
    std::string snapshot_error;
    const std::string prefix=trace_writer().path()+".session_"+std::to_string(session);
    if(!write_config_snapshot(cfg,prefix+".params.yaml",snapshot_error))
        throw std::runtime_error("servo_config_snapshot_failed: "+snapshot_error);
    // Preserve on-disk model for diagnosis. It is explicitly not proof of the SDK's
    // in-memory model if someone edits the YAML after process startup.
    std::error_code copy_error;
    std::filesystem::copy_file(t170::paths::config_file("Robot_Model.yaml"),
        prefix+".arm_model_on_disk.yaml",std::filesystem::copy_options::overwrite_existing,copy_error);
    if(copy_error) std::cerr << "[servo] 模型文件备份失败: " << copy_error.message() << '\n';
    for(const auto &name:{"arm_binding.yaml","arm_model_metadata.yaml","arm_frames.yaml"}) {
        const auto source=t170::paths::config_file(name);
        if(!std::filesystem::exists(source)) continue;
        copy_error.clear();
        std::filesystem::copy_file(source,prefix+"."+name,
            std::filesystem::copy_options::overwrite_existing,copy_error);
        if(copy_error) std::cerr<<"[servo] "<<name<<" snapshot: "<<copy_error.message()<<'\n';
    }
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
void sample_approach_state(Arm &arm) {
    const int hand=hand_index(arm); double ms=0;
    measured(arm,slots[hand],ms);
}
void end_session(int hand, bool success) {
    if(hand<0 || hand>1) return;
    auto &slot=slots[hand];orch::TimingContext telemetry;std::map<std::string,double> metrics;std::string reason,tag;
    {std::lock_guard<std::mutex> lock(slot.mutex);
     if(!slot.context.active)return;
     slot.context.active=false;slot.context.descending=false;slot.pending.reset();slot.pending_view.reset();slot.succeeded=success;
     if(!success && slot.failure_reason.empty())slot.failure_reason="approach_or_execution_failed";
     if(!success)group_failed.store(true);
     telemetry=slot.telemetry;metrics=slot.metrics;reason=slot.failure_reason;tag=slot.context.task_tag;
     metrics["hand"]=hand;metrics["servo_session"]=slot.context.session;metrics["dur_ms"]=(monotonic_seconds()-slot.began_s)*1000;
    }
    orch::timing_event("servo_end",tag,success?"success":"failed",reason,metrics,&telemetry);
}
void stage_tray_target(int hand,const TrayTarget &target) {
    if(hand<0 || hand>1) return;
    std::lock_guard<std::mutex> lock(slots[hand].mutex);
    slots[hand].staged_tray_target=target;
}
void stage_tray_grasp(int hand,double feature_plane_z) {
    if(hand<0 || hand>1) return;
    std::lock_guard<std::mutex> lock(slots[hand].mutex);
    if(std::isfinite(feature_plane_z)) slots[hand].staged_tray_grasp_plane=feature_plane_z;
    else slots[hand].staged_tray_grasp_plane.reset();
}
void stage_belt_grasp_reference(int hand,bool enable) {
    if(hand<0 || hand>1) return;
    std::lock_guard<std::mutex> lock(slots[hand].mutex);
    slots[hand].staged_belt_grasp_reference=enable;
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
bool ready_for_gripper(Arm &arm) {
    const int hand=hand_index(arm); auto &slot=slots[hand];
    Vec2 goal; Config cfg; double final_z; bool succeeded,place,tray_place,belt; Eigen::Matrix3d rotation;
    { std::lock_guard<std::mutex> lock(slot.mutex);
      succeeded=slot.succeeded; goal=slot.completed_goal;
      cfg=slot.context.config; final_z=slot.context.final_z;
      rotation=slot.context.desired_rotation;
      place=slot.context.target_class==1;
      tray_place=place && slot.context.config.tray_place.enabled && slot.context.tray_target.valid;
      belt=!place && slot.context.task_tag.rfind("belt",0)==0;
    }
    if(!succeeded || hardware_abort_requested() || can_io_faulted() || group_failed.load()) return false;
    double read_ms=0; const auto state=measured(arm,slot,read_ms);
    const double xy_accept=place ? (tray_place?cfg.place_xy_accept_m:cfg.xy_tolerance_m)
                                 : (belt?0.018:cfg.grasp_xy_accept_m);
    const double z_above=place ? (tray_place?cfg.place_z_accept_above_m:cfg.z_tolerance_m)
                               : (belt?0.012:cfg.grasp_z_accept_above_m);
    const bool ready=(state.position.head<2>()-goal).norm()<=xy_accept &&
        state.position.z()>=final_z-(place ? cfg.z_tolerance_m : cfg.grasp_z_feedback_tolerance_m) &&
        state.position.z()<=final_z+z_above &&
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
}
