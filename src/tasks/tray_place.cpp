#include "internal/runtime.h"
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
int vision_place_tray2(bool allow_right, bool allow_left, bool return_to_tray)
{
    if (g_hw == nullptr || g_hw->g == nullptr || g_hw->pipeline == nullptr)
        return -1;

    g_hw->last_placed_r=false;g_hw->last_placed_l=false;
    std::array<double, 16> cam2robot{};
    std::string err;
    if (!load_head_cam2robot(cam2robot, err) &&
        !load_cam2robot_matrix(default_camera_to_robot_yaml_path(), cam2robot, err))
    {
        std::cerr << "[orch] 读 cam2robot 失败: " << err << std::endl;
        return -3;
    }

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };

    struct TrayTaskGuard
    {
        ~TrayTaskGuard() { move_box::set_tray_hole_task(move_box::TrayHoleTask::GraspParts); }
    } task_guard;
    move_box::set_tray_hole_task(move_box::TrayHoleTask::PlaceEmpty);

    board_step("去料盘二");
    const bool prep_done = g_tray2_place_prep_done;
    g_tray2_place_prep_done = false;
    int prep_rc = 0;
    int travel_rc = 0;
    if (prep_done)
    {
        std::cout << "[tray2] 去料盘2 " << g_move_cfg.chassis.tray2_station
                  << "：路上已降腰并收手到放置准备，不再重摆，也不再等 0.5s\n";
        travel_rc = chassis_goto_station(g_move_cfg.chassis.tray2_station);
        if (g_hw != nullptr && g_hw->waist != nullptr && g_hw->layer3 != nullptr)
            move_box::waist_layer3_set_synced(true);
    }
    else
    {
        std::cout << "[tray2] 去料盘2 " << g_move_cfg.chassis.tray2_station
                  << "，路上同时摆头、待机臂和下腰\n";
        {
            ChassisTravel travel(g_move_cfg.chassis.tray2_station);
            const int head_rc = enable_head_calib_pose();
            if (head_rc != 0)
                prep_rc = head_rc;
            else if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l))
            {
                std::cerr << "[tray2] 进入时回到 standby 失败\n";
                prep_rc = should_abort() ? -4 : -5;
            }
            else if (g_hw->waist == nullptr || g_hw->layer3 == nullptr)
            {
                std::cerr << "[tray2] 腰未初始化，拒绝进入放置\n";
                prep_rc = -1;
            }
            else if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
            {
                std::cerr << "[tray2] 腰未到 ready 起始位，放弃放置\n";
                prep_rc = should_abort() ? -4 : -5;
            }
            else
                move_box::waist_layer3_set_synced(true);
            travel_rc = travel.join();
        }
    }
    if (should_abort())
        return -4;
    if (travel_rc != 0)
        return travel_rc;
    if (prep_rc != 0)
        return prep_rc;
    board_step("料盘二放置");
    if (!prep_done)
    {
        std::cout << "[tray2] 已在 " << g_move_cfg.chassis.tray2_station << "，停稳 0.5s 再放\n";
        hardware_abort_sleep_ms(500);
        if (should_abort())
            return -4;
    }
    else
        std::cout << "[tray2] 已在 " << g_move_cfg.chassis.tray2_station
                  << "，直接进入放置\n";

    constexpr int kMaxRounds = 40;
    constexpr int kMaxNoneStreak = 2;
    int placed_count = 0;
    int none_streak = 0;
    bool placed_r = false;
    bool placed_l = false;
    g_hw->last_grasp_count = 0;
    // Keep holding latches until each side has confirmed its open feedback.

    const auto both_placed = [&]() -> bool {
        const bool right_ok = !allow_right || right_arm_motors_locked() || placed_r;
        const bool left_ok = !allow_left || placed_l;
        return right_ok && left_ok && (placed_r || placed_l);
    };

    std::cout << "[tray2] 开始循环：YOLO class1 空孔，顺序同 grasp；到位后张爪"
              << (right_arm_motors_locked() ? "（右臂锁定，只要求左手放置）" : "")
              << (!allow_right ? "；右手不放" : "")
              << (!allow_left ? "；左手不放" : "")
              << "\n";

    bool saw_zone = false;
    int last_rc = 1;
    for (int round = 1; round <= kMaxRounds; ++round)
    {
        if (should_abort())
        {
            g_hw->last_grasp_count = placed_count;
            g_hw->last_grasp_status = -4;
            std::cout << "[tray2] 循环中止 已放置 右=" << (placed_r ? 1 : 0)
                      << " 左=" << (placed_l ? 1 : 0) << "\n";
            return -4;
        }

        if (both_placed())
        {
            last_rc = 0;
            std::cout << "[tray2] 双手已放置，结束放置循环\n";
            break;
        }

        std::cout << "\n======== [tray2] 第 " << round << " 轮 已放 右="
                  << (placed_r ? 1 : 0) << " 左=" << (placed_l ? 1 : 0)
                  << " ========\n";
        board_activity("料盘二第 " + std::to_string(round) + " 轮：拍照找空孔并放置");
        move_box::GraspToStandbyResult r = move_box::run_grasp_to_standby(
            *g_hw->g,
            *g_hw->arm_r,
            *g_hw->arm_l,
            *g_hw->waist,
            *g_hw->layer3,
            g_hw->pipeline->cameras(),
            g_hw->pipeline->bridge(),
            cam2robot,
            err,
            should_abort,
            SegEngineId::Default,
            false,
            placed_r || !allow_right,
            placed_l || !allow_left);

        if (r.zone_count_ok)
        {
            int remain_r = r.zone_right - (r.grasped_r ? 1 : 0);
            int remain_l = r.zone_left - (r.grasped_l ? 1 : 0);
            if (remain_r < 0)
                remain_r = 0;
            if (remain_l < 0)
                remain_l = 0;
            saw_zone = true;
            g_unload_empty_known = true;
            g_unload_empty_right = remain_r;
            g_unload_empty_left = remain_l;
            std::cout << "[tray2] 本轮选孔照片计剩余空孔 右=" << remain_r
                      << " 左=" << remain_l << "\n";
            board_tray2(remain_r, remain_l);
        }
        board_tray2_placed((r.grasped_r ? 1 : 0) + (r.grasped_l ? 1 : 0));

        // 放置成功＝该侧已松爪，清流程闩锁；界面持件改看实时 q。
        if (r.grasped_r)
        {
            placed_r = true;
            clear_last_grasped(true, false);
        }
        if (r.grasped_l)
        {
            placed_l = true;
            clear_last_grasped(false, true);
        }
        placed_count = (placed_r ? 1 : 0) + (placed_l ? 1 : 0);
        g_hw->last_placed_r=placed_r;g_hw->last_placed_l=placed_l;
        g_hw->last_grasp_count=placed_count;
        g_hw->last_grasp_status = static_cast<int>(r.status);

        if (r.status == move_box::GraspToStandbyStatus::ReleaseFail) {
            std::cerr << "[tray_place] 张爪未确认，停止任务，保留已确认放置的侧\n";
            return -38;
        }
        if (r.status == move_box::GraspToStandbyStatus::MotionFail) {
            std::cerr << "[retract] 收手未完成，停止当前任务，保留已持件/已放置状态\n";
            return -37;
        }
        if (r.status == move_box::GraspToStandbyStatus::Aborted)
        {
            g_hw->last_grasp_count = placed_count;
            std::cout << "[tray2] 循环中止 已放置 右=" << (placed_r ? 1 : 0)
                      << " 左=" << (placed_l ? 1 : 0) << "\n";
            return -4;
        }

        if (r.status == move_box::GraspToStandbyStatus::Ok ||
            r.grasped_r || r.grasped_l)
        {
            none_streak = 0;
            std::cout << "[tray2] 第 " << round << " 轮完成 本轮新放"
                      << " 右=" << (r.grasped_r ? 1 : 0)
                      << " 左=" << (r.grasped_l ? 1 : 0)
                      << " 现已放 右=" << (placed_r ? 1 : 0)
                      << " 左=" << (placed_l ? 1 : 0);
            if (both_placed())
            {
                std::cout << "，双手已齐\n";
                last_rc = 0;
                break;
            }
            std::cout << "，未放侧继续\n";
            last_rc = 0;
            continue;
        }

        if (r.status == move_box::GraspToStandbyStatus::NoTarget)
        {
            if (!r.zone_count_ok)
            {
                g_hw->last_tray_message = "二维码解算失败";
                std::cerr << "[tray2] 料盘二维码解算失败，本轮不伸手\n";
                last_rc = kVisionQrFail;
                break;
            }
            if (both_placed())
            {
                last_rc = 0;
                break;
            }
            std::cout << "[tray2] 工作区已无空孔，仍缺放"
                      << " 右=" << (placed_r ? 1 : 0)
                      << " 左=" << (placed_l ? 1 : 0) << "，结束循环\n";
            last_rc = (placed_r || placed_l) ? 2 : 1;
            break;
        }

        ++none_streak;
        std::cerr << "[tray2] 第 " << round << " 轮未放到 code="
                  << static_cast<int>(r.status) << " 连续失败=" << none_streak
                  << " 已放 右=" << (placed_r ? 1 : 0)
                  << " 左=" << (placed_l ? 1 : 0) << "\n";
        if (none_streak >= kMaxNoneStreak)
        {
            std::cerr << "[tray2] 连续 " << none_streak << " 轮未放到，退出循环\n";
            last_rc = both_placed() ? 0 : 2;
            break;
        }
        last_rc = 2;
    }

    g_hw->last_grasp_count = placed_count;
    g_hw->last_grasp_status = last_rc;
    std::cout << "\n[tray2] 循环结束 已放 右=" << (placed_r ? 1 : 0)
              << " 左=" << (placed_l ? 1 : 0)
              << " 共 " << placed_count << " 件\n";
    std::cout.flush();

    if (should_abort())
        return -4;
    if (placed_r || placed_l)
    {
        if (!return_to_tray)
        {
            if (!saw_zone)
                g_unload_empty_known = false;
            g_dispatch_stay_at_tray2 = true;
            std::cout << "[tray2] 放完停在 " << g_move_cfg.chassis.tray2_station
                      << "，不升腰，不回 " << g_move_cfg.chassis.tray_station << std::endl;
            return last_rc;
        }
        g_dispatch_stay_at_tray2 = false;
        const std::string next = g_move_cfg.chassis.tray_station;
        std::cout << "[tray2] 放完先升腰，再去 " << next << "，到站后再降腰\n";
        const int head_rc = enable_head_calib_pose();
        if (head_rc != 0)
            return head_rc;
        const int waist_rc = home_waist_upright();
        if (waist_rc != 0)
            return waist_rc;
        if (should_abort())
            return -4;
        const int chassis_rc = chassis_goto_station(next);
        if (chassis_rc != 0)
            return chassis_rc;
        if (should_abort())
            return -4;
        if (g_hw->waist == nullptr || g_hw->layer3 == nullptr ||
            !move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
        {
            std::cerr << "[tray2] 到 " << next << " 后降腰失败\n";
            return should_abort() ? -4 : -5;
        }
        move_box::waist_layer3_set_synced(true);
        if (!saw_zone)
            g_unload_empty_known = false;
        std::cout << "[tray2] 腰和底盘已结束，已在 " << next << std::endl;
        return last_rc;
    }

    std::cout << "[tray2] 本轮没有放下，仍按原顺序站起\n";
    const int home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
    if (home_rc != 0)
        return home_rc;
    {
        const move_box::TrayZoneCount left = move_box::count_tray_zone_holes(
            g_hw->pipeline->cameras(), g_hw->pipeline->bridge(), cam2robot);
        if (left.ok)
        {
            g_unload_empty_known = true;
            g_unload_empty_right = left.right;
            g_unload_empty_left = left.left;
            std::cout << "[tray2] 放下后空孔 右=" << left.right
                      << " 左=" << left.left << "\n";
            board_tray2(left.right, left.left);
        }
        else
        {
            g_unload_empty_known = false;
            std::cerr << "[tray2] 放下后没数清空孔: " << left.message << "\n";
            if (left.message == "取帧失败" || left.message == "无相机")
                g_hw->last_tray_message = left.message;
        }
    }
    return last_rc;
}
}
