#include "internal/runtime.h"
#include "t170/skills/conveyor.h"

#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "Ti5_Arm.h"
#include "t170/common/units.h"
#include "t170/motion/cartesian_tracking.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
#include "t170/vision/wrist_observer.h"
#include "t170/motion/servo.h"
#include "t170/vision/head_transform.h"
#include "t170/hardware/head.h"
#include "t170/skills/legacy/profiles.h"
#include "t170/skills/legacy/runtime.h"
#include "t170/vision/python_bridge.h"
#include "t170/vision/dl.h"
#include "t170/vision/debug.h"
#include "t170/hardware/device_bus.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <cerrno>
#include <csignal>
#include <mutex>
#include "t170/motion/retreat_gate.h"
#include <sys/select.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using std::vector;
using std::string;
using std::thread;


namespace orch {
bool live_gripper_holding(bool is_right)
{
    if (g_hw == nullptr)
        return false;
    if (is_right && right_arm_motors_locked())
        return false;
    const bool latch = is_right ? g_hw->last_grasped_r : g_hw->last_grasped_l;
    if (g_hw->g == nullptr ||
        !g_hw->g->hasSide(is_right ? gripper::Side::Right : gripper::Side::Left))
        return latch;
    const auto fb =
        g_hw->g->feedback(is_right ? gripper::Side::Right : gripper::Side::Left);
    if (!fb.have_feedback)
        return latch;
    // q：0=全合，≈5.5=全开。夹零件约 0.8～1.6；松爪/准备态约 4.3～5.5；空抓合到底约 0.03。
    // 实时持件必须落在 [empty_q_rad, hold_q_max_rad]，张开的高 q 不能算持件。
    const double q = fb.position_rad;
    const double q_lo = g_move_cfg.gripper_check.empty_q_rad;
    const double q_hi = g_move_cfg.gripper_check.hold_q_max_rad;
    return q >= q_lo && q <= q_hi;
}

void clear_last_grasped(bool right, bool left)
{
    if (g_hw == nullptr)
        return;
    if (right && g_hw->last_grasped_r)
    {
        g_hw->last_grasped_r = false;
        std::cout << "[grasp] 松爪/放置后清除右手持件闩锁\n";
    }
    if (left && g_hw->last_grasped_l)
    {
        g_hw->last_grasped_l = false;
        std::cout << "[grasp] 松爪/放置后清除左手持件闩锁\n";
    }
}

void wave_grip_ratio(bool is_right, double ratio)
{
    if (g_hw == nullptr || g_hw->g == nullptr)
        return;
    if (is_right && right_arm_motors_locked())
        return;
    if (ratio < 0.0)
        ratio = 0.0;
    if (ratio > 1.0)
        ratio = 1.0;
    const gripper::Side side = is_right ? gripper::Side::Right : gripper::Side::Left;
    std::cout << "[wave] " << (is_right ? "右手" : "左手")
              << " 夹爪收到 " << std::fixed << std::setprecision(2) << ratio
              << "（0全开 1全合）\n";
    g_hw->g->closeAndWait(side, ratio, 0.2, std::chrono::milliseconds(800));
}
}
