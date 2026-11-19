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
Runtime slots[2];
std::atomic<std::uint64_t> next_session{1};
std::atomic<bool> group_failed{false};
}
namespace detail {
bool pair_too_close(int hand, const State &s, const Vec2 &goal, double clearance) {
    Runtime &other=slots[1-hand];
    std::lock_guard<std::mutex> lock(other.mutex);
    if (!other.context.active || !other.context.descending) return false;
    if (std::abs(other.context.latest.position.z()-s.position.z())>0.08) return false;
    return (other.context.latest.position.head<2>()-s.position.head<2>()).norm()<clearance ||
           (other.context.latest.position.head<2>()-goal).norm()<clearance;
}
}

namespace detail {
void send(int hand, const Joints &q) {
    send_joint_reference(hand,q);
}
}

}
