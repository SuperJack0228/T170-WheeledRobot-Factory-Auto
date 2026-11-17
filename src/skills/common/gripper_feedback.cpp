#include "t170/vision/tray_localizer.h"
#include "../internal/legacy_helpers.h"
#include "t170/motion/servo.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "t170/vision/wrist_observer.h"
#include "t170/motion/arm.h"
#include "t170/common/units.h"
#include <array>
#include <cstring>
#include "t170/vision/head_transform.h"
#include "t170/hardware/device_bus.h"

using std::vector;
using std::thread;


namespace move_box {
bool gripper_grasp_empty(gripper::Gripper &g, gripper::Side side,
                             const gripper::GraspFeedback &fb, const char *tag)
    {
        const char *hand = side == gripper::Side::Right ? "右手" : "左手";
        const char *result = fb.result == gripper::GraspResult::TorqueLimit ? "力矩到上限"
                             : fb.result == gripper::GraspResult::PositionReached ? "合到底"
                                                                               : "超时";
        const auto &gc = g_move_cfg.gripper_check;
        const bool known = g.hasSide(side) && g.feedback(side).have_feedback;
        // 夹零件时 q 约 ≥0.5；空爪/松到 ~0.3 不能算持件 → q < empty_q_rad 空抓。
        const bool empty = gc.enable && known && fb.position_rad < gc.empty_q_rad;
        const char *servo_outcome=!known || !gc.enable || fb.result==gripper::GraspResult::Timeout
            ? "unknown_gripper_feedback" : (empty?"empty_grasp":"holding_candidate");
        wrist_servo::record_gripper_check(side==gripper::Side::Right?1:0,fb.position_rad,fb.torque,servo_outcome);
        std::cout << std::fixed << std::setprecision(3)
                  << "[gripper] " << tag << ' ' << hand << " 合爪结束 q=" << fb.position_rad
                  << " rad 力矩=" << fb.torque << " N·m（" << result << "）";
        if (!known)
            std::cout << "，夹爪没配对或无反馈，不判空抓\n";
        else if (!gc.enable)
            std::cout << "，空抓检测已关闭\n";
        else if (empty)
            std::cout << " < " << gc.empty_q_rad << "，判为空抓\n";
        else
            std::cout << " ≥ " << gc.empty_q_rad << "，夹到件\n";
        if (empty)
            std::cerr << "[gripper] 警告：" << tag << ' ' << hand << "空抓，没夹到件\n";
        return empty;
    }
}
