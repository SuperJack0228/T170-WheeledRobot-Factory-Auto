#include "internal/runtime.h"
#include "t170/telemetry/dispatch_metrics.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace orch {
int dispatch_load(DispatchClient &client)
{
    const TaskPoints points = read_task_points();

    if (!task_points_ready(dispatch_msg::TASK_LOAD, points))
        return kDispatchPointsMissing;
    StationOverride stations(g_move_cfg.chassis);
    g_move_cfg.chassis.tray_station = points.raw;
    g_move_cfg.chassis.belt_station = points.load;
    std::cout << "[dispatch] 上料：料架 " << points.raw << " 抓件，抓完最后的零件就报料盘空；"
              << "料盘抓完就申请上料点 " << points.load
              << "，底盘到传送带一停稳后再拍照放置，然后发 "
              << dispatch_msg::DONE << "\n";
    struct PlaceGrant
    {
        std::atomic<bool> started{false};
        std::atomic<bool> ok{false};
        std::thread th;
        ~PlaceGrant()
        {
            if (th.joinable())
                th.join();
        }
    } grant;
    struct ClearDepartHook
    {
        ~ClearDepartHook() { g_on_grasp_depart = nullptr; }
    } clear_depart;
    g_on_grasp_depart = [&]() {
        if (grant.started.exchange(true))
            return;
        grant.th = std::thread([&client, &grant]() {
            std::cout << "[dispatch] 料盘一已抓完，去传送带一的同时发送 "
                      << dispatch_msg::APPLY_PLACE_RAW << "，等待 "
                      << dispatch_msg::PLACE << "\n";
            grant.ok = dispatch_apply(
                dispatch_msg::APPLY_PLACE_RAW, dispatch_msg::PLACE, "上料放置",
                [&]() { return client.apply_place_raw_material(kDispatchApplyTimeoutMs); });
        });
    };
    int grasp_rc = 0;
    for (int attempt = 0;; ++attempt)
    {
        grasp_rc = vision_grasp_existing_pipeline();
        if (grasp_rc != kVisionQrFail)
            break;
        std::cerr << "[dispatch] 上料抓取 料盘二维码解算失败\n";
        const int choice = dispatch_report_ex(client, "二维码识别错误");
        if (choice != 0)
        {
            grasp_rc = choice;
            break;
        }
        board_activity("选择重试，再拍一次料盘");
    }
    g_on_grasp_depart = nullptr;
    if (grasp_rc < 0 || grasp_rc == kDispatchTaskSkip || grasp_rc == kDispatchOffline ||
        grasp_rc == kDispatchReportFailed)
        return grasp_rc;
    const bool hold_r = g_hw != nullptr && g_hw->last_grasped_r;
    const bool hold_l = g_hw != nullptr && g_hw->last_grasped_l;
    const int took = (hold_r ? 1 : 0) + (hold_l ? 1 : 0);
    if (g_load_parts_seen == 0 || (g_load_parts_seen > 0 && took >= g_load_parts_seen))
    {
        std::cout << "[dispatch] 上料盘已经没有零件（开抓前 "
                  << g_load_parts_seen << " 件，这次拿走 " << took << " 件）\n";
        move_box::tray1_far_band_clear();
        dispatch_send(client, dispatch_msg::RAW_MATERIAL_EMPTY, client.raw_material_empty());
    }
    if (!hold_r && !hold_l)
        return grasp_rc == 0 ? 1 : grasp_rc;
    if (!g_grasp_departed)
    {
        if (go_home(*g_hw->arm_r, *g_hw->arm_l) != 0)
            return -5;
        if (chassis_goto_station(g_move_cfg.chassis.belt_station) != 0)
            return -6;
    }
    else
        std::cout << "[dispatch] 上料抓取收手时腰已在拍照前伸，底盘已经去 "
                  << g_move_cfg.chassis.belt_station << "\n";
    bool granted = false;
    if (grant.started)
    {
        board_step("申请上料放置");
        if (grant.th.joinable())
            grant.th.join();
        granted = grant.ok.load();
        if (!granted)
        {
            std::cout << "[dispatch] 上料放置申请超时，不再等待，直接放置\n";
            board_note("warn", "上料放置申请超时，直接放置");
        }
        else
            std::cout << "[dispatch] 已收到 " << dispatch_msg::PLACE
                      << "，等传送带一停稳后再拍照放置\n";
    }
    const bool pose_ready = station_name_matches(
        g_photo_ready_station, g_move_cfg.chassis.belt_station);
    // 路上已经申请过就不要再等一轮；没申请过的仍由放置流程等一次，超时同样直接放。
    const int place_rc = dispatch_place_belt(
        client, [](DispatchClient &c, int timeout_ms) { return c.apply_place_raw_material(timeout_ms); },
        dispatch_msg::APPLY_PLACE_RAW, dispatch_msg::PLACE,
        g_move_cfg.conveyor, "belt", hold_r, hold_l, "上料放置", pose_ready,
        grant.started.load());
    if (place_rc != 0)
        return place_rc;
    std::cout << "[dispatch] 上料完成，停在当前位置\n";
    return 0;
}
}
