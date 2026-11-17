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
bool observation_context(int hand, ObservationContext &context) {
    if(hand<0 || hand>1) return false;
    std::lock_guard<std::mutex> lock(slots[hand].mutex);
    context=slots[hand].context; return context.active;
}
bool camera_at_capture(int hand,std::uint64_t session,double time,Eigen::Matrix4d &base_camera,State *capture_state) {
    if(hand<0 || hand>1) return false;
    auto &slot=slots[hand]; std::lock_guard<std::mutex> lock(slot.mutex);
    State capture;
    if(!slot.context.active || slot.context.session!=session ||
       !slot.history.interpolate(time,slot.context.config.max_state_gap_s,capture)) return false;
    Eigen::Matrix4d tcp=Eigen::Matrix4d::Identity();
    tcp.block<3,3>(0,0)=capture.rotation; tcp.block<3,1>(0,3)=capture.position;
    base_camera=tcp*slot.tcp_to_camera;
    if(capture_state) *capture_state=capture;
    return base_camera.allFinite();
}
bool submit_observation(int hand, const PixelObservation &o) {
    if(hand<0 || hand>1) return false;
    auto &slot=slots[hand]; std::lock_guard<std::mutex> lock(slot.mutex);
    if(!slot.context.active || !slot.context.descending || !slot.context.vision_required ||
       o.session!=slot.context.session) return false;
    if(slot.pending && o.capture_s<=slot.pending->capture_s) return false;
    slot.pending=o; return true;
}
void report_tray_view(int hand,std::uint64_t session,double completed_s,bool insufficient_view) {
    if(hand<0 || hand>1 || !std::isfinite(completed_s)) return;
    auto &slot=slots[hand];std::lock_guard<std::mutex> lock(slot.mutex);
    if(!slot.context.active || !slot.context.descending || !slot.context.vision_required || slot.context.session!=session) return;
    if(slot.pending_view && slot.pending_view->first>completed_s) return;
    slot.pending_view=std::make_pair(completed_s,insufficient_view);
}
}
