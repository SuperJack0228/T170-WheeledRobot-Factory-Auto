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
namespace detail {
State measured(Arm &arm, Runtime &slot, double &read_ms) {
    const double before = monotonic_seconds();
    Joints q;
    if(!read_joint_feedback(hand_index(arm),q)) {
        std::lock_guard lock(slot.mutex);
        // The caller sees the original timestamp and skips the control tick.
        // Neither the observer history nor velocity/camera estimates see this frame.
        if(slot.context.latest.time_s>0 && before-slot.context.latest.time_s<=.10)return slot.context.latest;
        throw std::runtime_error("joint_feedback_validity_timeout");
    }
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
}

}
