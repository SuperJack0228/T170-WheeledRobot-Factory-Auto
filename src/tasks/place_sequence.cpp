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
#include "t170/motion/arm.h"
#include "t170/common/units.h"
namespace orch {
int dispatch_place_belt(
    DispatchClient &client, bool (*apply_place)(DispatchClient &, int),
    const char *request_state, const char *grant_state,
    const ConveyorStationConfig &c, const char *tag,
    bool hold_right, bool hold_left, const char *place_what,
    bool skip_ready, bool already_granted)
{
    if (!hold_right && !hold_left)
    {
        std::cout << "[dispatch] 手里没有件，跳过" << place_what << "\n";
        return 0;
    }
    auto raw_apply = [&]() { return apply_place(client, kDispatchApplyTimeoutMs); };
    bool need_apply = !already_granted;
    if (already_granted)
        std::cout << "[dispatch] " << place_what
                  << " 的申请已在来的路上结束，底盘停稳后拍照放置\n";
    int vision_retry = 1;
    for (int attempt = 0;; ++attempt)
    {
        if (need_apply)
        {
            std::cout << "[dispatch] 发送 " << request_state << "，等待 " << grant_state
                      << "，超时 " << kDispatchApplyTimeoutMs << " ms\n";
            if (!dispatch_apply(request_state, grant_state, place_what, raw_apply))
            {
                std::cout << "[dispatch] " << place_what
                          << " 申请超时，不再等待，直接放置\n";
                board_note("warn", std::string(place_what) + " 申请超时，直接放置");
            }
            else
                std::cout << "[dispatch] 已收到 " << grant_state << "，" << place_what
                          << " 放行\n";
        }
        need_apply = true;
        board_step(std::string(tag) == "belt2" ? "传送带二放置" : "传送带一放置");
        board_activity(std::string(place_what) + "：拍照认码、依次放置");
        const int place_rc = go_belt_station(
            c, tag, {}, hold_right, hold_left,
            attempt == 0 ? skip_ready : true);
        if (place_rc == kBeltPlaceOccupied)
        {
            std::cout << "[dispatch] " << place_what << " 放置点有零件\n";
            const int choice = dispatch_report_ex(client, "上料口有料，无法放置");
            if (choice != 0)
                return choice;
            skip_ready = true;
            need_apply = false;
            continue;
        }
        if (place_rc == kVisionQrFail)
        {
            const int again = dispatch_belt_vision_again(client, vision_retry, place_what);
            if (again != 0)
                return again;
            skip_ready = true;
            need_apply = false;
            continue;
        }
        if (place_rc != 0)
            return place_rc;
        const int check_rc = confirm_belt_place_not_fallen(
            client, c, tag, hold_right, hold_left, place_what);
        if (check_rc != 0)
            return check_rc;
        std::cout << "[dispatch] " << place_what
                  << " 放置确认通过，发送 "
                  << dispatch_msg::PLACE_DONE << " 和 " << dispatch_msg::DONE << "\n";
        board_step("汇报完成");
        const bool place_ok = client.place_done();
        board_report(dispatch_msg::PLACE_DONE, place_what, place_ok);
        if (!place_ok)
        {
            std::cerr << "[dispatch] 发送 " << dispatch_msg::PLACE_DONE << " 失败\n";
            return -7;
        }
        std::cout << "[dispatch] 已发送 " << dispatch_msg::PLACE_DONE << "\n";
        const bool done_ok = client.mission_done();
        board_report(dispatch_msg::DONE, "任务完成", done_ok);
        if (!done_ok)
        {
            std::cerr << "[dispatch] 发送 " << dispatch_msg::DONE << " 失败\n";
            return -7;
        }
        std::cout << "[dispatch] 已发送 " << dispatch_msg::DONE << "\n";
        return 0;
    }
}

int place_while_driving(
    DispatchClient &client, bool (*apply_place)(DispatchClient &, int),
    const char *request_state, const char *grant_state,
    const std::string &station, const ConveyorStationConfig &c, const char *tag,
    bool hold_right, bool hold_left, const char *place_what)
{
    if (!hold_right && !hold_left)
    {
        std::cout << "[dispatch] 手里没有件，跳过" << place_what << "\n";
        return 0;
    }
    std::atomic<bool> granted{false};
    board_step("去传送带二并申请放置");
    std::thread apply_th([&]() {
        std::cout << "[dispatch] 去 " << station << " 的同时发送 " << request_state
                  << "，等待 " << grant_state << "\n";
        granted = dispatch_apply(request_state, grant_state, place_what,
                                 [&]() { return apply_place(client, kDispatchApplyTimeoutMs); });
    });
    int ready_rc = 0;
    int ch_rc = 0;
    {
        ChassisTravel travel(station);
        ready_rc = go_belt_ready(*g_hw->arm_r, *g_hw->arm_l, c, tag);
        ch_rc = travel.join();
    }
    if (apply_th.joinable())
        apply_th.join();
    if (ch_rc != 0)
        return ch_rc;
    if (ready_rc != 0)
        return ready_rc;
    if (!granted.load())
    {
        std::cout << "[dispatch] " << place_what
                  << " 申请超时，不再等待，直接放置\n";
        board_note("warn", std::string(place_what) + " 申请超时，直接放置");
    }
    else
        std::cout << "[dispatch] 已收到 " << grant_state << "，" << place_what << " 放行\n";
    auto raw_apply = [&]() { return apply_place(client, kDispatchApplyTimeoutMs); };
    int vision_retry = 1;
    for (int attempt = 0;; ++attempt)
    {
        board_step("传送带二放置");
        board_activity(std::string(place_what) + "：拍照认码、依次放置");
        const int place_rc = go_belt_station(c, tag, {}, hold_right, hold_left, true);
        if (place_rc == kBeltPlaceOccupied)
        {
            std::cout << "[dispatch] " << place_what << " 放置点有零件\n";
            const int choice = dispatch_report_ex(client, "上料口有料，无法放置");
            if (choice != 0)
                return choice;
            continue;
        }
        if (place_rc == kVisionQrFail)
        {
            const int again = dispatch_belt_vision_again(client, vision_retry, place_what);
            if (again != 0)
                return again;
            continue;
        }
        if (place_rc != 0)
            return place_rc;
        const int check_rc = confirm_belt_place_not_fallen(
            client, c, tag, hold_right, hold_left, place_what);
        if (check_rc != 0)
            return check_rc;
        break;
    }
    std::cout << "[dispatch] " << place_what << " 放置确认通过，发送 "
              << dispatch_msg::PLACE_DONE << "\n";
    const bool place_ok = client.place_done();
    board_report(dispatch_msg::PLACE_DONE, place_what, place_ok);
    if (!place_ok)
    {
        std::cerr << "[dispatch] 发送 " << dispatch_msg::PLACE_DONE << " 失败\n";
        return -7;
    }
    std::cout << "[dispatch] 已发送 " << dispatch_msg::PLACE_DONE << "\n";
    return 0;
}
}
