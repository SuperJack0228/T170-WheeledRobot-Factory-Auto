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
void observation_event(int hand,std::uint64_t session,const char *reason,const PixelObservation &o) {
    if(hand>=0 && hand<2) {
        std::lock_guard<std::mutex> lock(slots[hand].mutex);
        if(slots[hand].context.session==session && reason) {
            const std::string r=reason;
            if(r=="locked_target_missing")++slots[hand].metrics["missing_frames"];
            if(r=="circle_attempt")++slots[hand].metrics["circle_attempts"];
            if(r=="circle_supported")++slots[hand].metrics["circle_candidates"];
            if(r=="frame_expired_before_association"||r=="circle_frame_expired")++slots[hand].metrics["expired_frames"];
        }
    }
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
    orch::timing_event("grasp_check",hand?"right":"left",outcome?outcome:"unknown_gripper_feedback",
        "checked gripper feedback, not external workpiece inspection",{{"q_rad",position},{"torque_nm",torque}});
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
}
