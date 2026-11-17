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
void hold_joint_reference(int hand,const Joints &q) {
    if(hand<0 || hand>1 || !q.allFinite() || hardware_abort_requested() || can_io_faulted()) return;
    {
        std::lock_guard<std::mutex> lock(slots[hand].mutex);
        slots[hand].final_command=q; slots[hand].final_hold=true;
    }
    start_final_hold_repeat();
}
}
