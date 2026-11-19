#include "t170/app/maintenance.h"
#include "../../tasks/internal/runtime.h"
#include "t170/skills/conveyor.h"

#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "t170/motion/arm.h"
#include "t170/common/units.h"
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
int go_waist_debug(int pose)
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[orch] waist 调试失败：腰未初始化\n";
        return -1;
    }
    if (pose != 1 && pose != 2)
    {
        std::cerr << "[orch] waist 调试 pose 必须是 1 或 2\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;

    if (g_hw != nullptr)
        g_hw->last_debug_photo.clear();

    const double x_home = g_move_cfg.waist.layer3_home(0);
    const double x_fwd = waist_far_row_x();
    const double x_target = (pose == 2) ? x_fwd : x_home;
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] waist" << pose << " 仅平移腰 X → " << x_target
              << " m（waist1 home=" << x_home
              << "  waist2 前伸=" << x_fwd
              << (pose == 2 ? "，随后低头截一帧" : "，不改 y/z、不动手臂")
              << "）\n";
    if (!move_box::waist_layer3_move_x_only(*g_hw->waist, *g_hw->layer3, x_target, true))
    {
        std::cerr << "[orch] waist" << pose << " 平移失败\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (hardware_abort_requested())
        return -4;
    move_box::waist_layer3_set_synced(true);
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] waist" << pose << " 到位 x=" << (*g_hw->layer3)(0)
              << " z=" << (*g_hw->layer3)(2) << " m\n";

    if (pose != 2)
        return 0;

    const auto &h = g_move_cfg.head;
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] waist2 头部低头到 " << h.far_pitch_deg << " deg 后截一帧\n";
    const int head_rc = move_head_to_rpy_deg(h.yaw_deg, h.far_pitch_deg, h.roll_deg, true);
    if (head_rc != 0)
        return head_rc;
    if (hardware_abort_requested())
        return -4;
    hardware_abort_sleep_ms(500);
    if (hardware_abort_requested())
        return -4;

    if (g_hw->pipeline == nullptr)
    {
        std::cerr << "[orch] waist2 无相机，跳过截图\n";
        return 0;
    }
    RealSenseMultiCam &cameras = g_hw->pipeline->cameras();
    CameraFrameData frame = cameras.grab_fresh(CameraSlot::Head);
    frame = RealSenseMultiCam::prepare_frame_for_slot(std::move(frame), CameraSlot::Head);
    if (!frame.ok || frame.color_bgr.empty() || frame.width <= 0 || frame.height <= 0)
    {
        std::cerr << "[orch] waist2 截帧失败: " << frame.message << "\n";
        return -2;
    }

    const std::string path = vision_debug_record_path("head", "waist2");
    cv::Mat bgr(frame.height, frame.width, CV_8UC3, frame.color_bgr.data());
    if (!cv::imwrite(path, bgr))
    {
        std::cerr << "[orch] waist2 保存截图失败 " << path << "\n";
        return -2;
    }
    g_hw->last_debug_photo = path;
    std::cout << "[orch] waist2 截图 " << path << "\n";
    return 0;
}

int go_waist_cycle()
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[orch] waist_cycle 失败：腰未初始化\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;

    const double x_home = g_move_cfg.waist.layer3_home(0);
    const double z_home = g_move_cfg.waist.layer3_home(2);
    const double x_ready = waist_ready_x();
    const double z_ready = waist_ready_z();
    const double x_far = waist_far_row_x();
    constexpr int kCycles = 3;
    constexpr int kHoldMs = 3000;

    std::cout << std::fixed << std::setprecision(4)
              << "[orch] waist_cycle 只动腰，循环 " << kCycles
              << " 次，每到点停 " << (kHoldMs / 1000) << "s\n"
              << "  home  x=" << x_home << " z=" << z_home << "\n"
              << "  抓取 x=" << x_ready << " z=" << z_ready
              << "（ready_forward=" << g_move_cfg.waist.ready_forward_m << "）\n"
              << "  远排 x=" << x_far << " z=" << z_ready
              << "（far_row_forward=" << g_move_cfg.waist.far_row_forward_m << "）\n"
              << "  顺序：home → 抓取 → 远排 → 抓取 → home\n";

    auto fail = []() { return hardware_abort_requested() ? -4 : -5; };
    auto goto_xz = [&](double x, double z, const char *label) -> int {
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] waist_cycle → " << label
                  << " x=" << x << " z=" << z << " m\n";
        if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, label))
            return fail();
        if (!move_box::waist_layer3_move_x_only(*g_hw->waist, *g_hw->layer3, x, true, false))
        {
            std::cerr << "[orch] waist_cycle " << label << " 平移 X 失败\n";
            return fail();
        }
        if (hardware_abort_requested())
            return -4;
        if (!move_box::waist_layer3_move_z_only(*g_hw->waist, *g_hw->layer3, z, true, false))
        {
            std::cerr << "[orch] waist_cycle " << label << " 升降 Z 失败\n";
            return fail();
        }
        if (hardware_abort_requested())
            return -4;
        move_box::waist_layer3_set_synced(true);
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] waist_cycle " << label << " 到位 x=" << (*g_hw->layer3)(0)
                  << " z=" << (*g_hw->layer3)(2) << "，停 3s\n";
        hardware_abort_sleep_ms(kHoldMs);
        return hardware_abort_requested() ? -4 : 0;
    };

    for (int i = 1; i <= kCycles; ++i)
    {
        std::cout << "\n======== [orch] waist_cycle 第 " << i << "/" << kCycles
                  << " 圈 ========\n";
        if (const int rc = goto_xz(x_home, z_home, "home"); rc != 0)
            return rc;
        if (const int rc = goto_xz(x_ready, z_ready, "料盘抓取"); rc != 0)
            return rc;
        if (const int rc = goto_xz(x_far, z_ready, "远排"); rc != 0)
            return rc;
        if (const int rc = goto_xz(x_ready, z_ready, "料盘抓取"); rc != 0)
            return rc;
        if (const int rc = goto_xz(x_home, z_home, "home"); rc != 0)
            return rc;
    }
    std::cout << "[orch] waist_cycle 完成\n";
    return 0;
}

int go_waist_jog(int joint, double dq_rad)
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[orch] waist_jog 失败：腰未初始化\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;
    constexpr double kMaxAbs = 0.12; // ≈6.9°
    if (!std::isfinite(dq_rad))
    {
        std::cerr << "[orch] waist_jog dq_rad 无效\n";
        return -1;
    }
    dq_rad = std::clamp(dq_rad, -kMaxAbs, kMaxAbs);
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] waist_jog joint=" << joint << " dq=" << dq_rad
              << " rad (" << (dq_rad * 180.0 / M_PI) << " deg)，手臂不动\n";
    const int rc = g_hw->waist->jogJoint(joint, dq_rad, 0.08);
    if (rc != 0)
    {
        std::cerr << "[orch] waist_jog 失败 code=" << rc << "\n";
        return hardware_abort_requested() ? -4 : (rc == -2 ? -2 : -5);
    }
    if (!move_box::sync_waist_layer3_from_encoders(
            *g_hw->waist, *g_hw->layer3, "jog后"))
        return -5;
    move_box::waist_layer3_set_synced(true);
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] waist_jog 到位 x=" << (*g_hw->layer3)(0)
              << " z=" << (*g_hw->layer3)(2) << " m\n";
    return 0;
}
}
