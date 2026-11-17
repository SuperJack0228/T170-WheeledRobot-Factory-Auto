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
bool station_name_matches(const std::string &got, const std::string &want)
{
    if (want.empty())
        return false;
    auto lower = [](std::string s) {
        for (char &c : s)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    };
    const std::string a = lower(got);
    const std::string b = lower(want);
    if (a == b)
        return true;
    return a.size() > b.size() ? a.find(b) != std::string::npos : b.find(a) != std::string::npos && !a.empty();
}

Chassis &chassis_seer() {return t170::tasks::services().chassis;}

bool ensure_chassis()
{
    Chassis &robot = chassis_seer();
    if (robot.IsConnected())
        return true;
    static bool logged_fail = false;
    robot.SetHost(g_move_cfg.chassis.host);
    if (!robot.Connect())
    {
        if (!logged_fail)
        {
            logged_fail = true;
            std::cerr << "[orch] 底盘连接失败: "
                      << (robot.LastMessage().empty() ? "无应答" : robot.LastMessage()) << "\n";
        }
        return false;
    }
    logged_fail = false;
    std::cout << "[orch] 底盘已连接 " << g_move_cfg.chassis.host << "\n";
    return true;
}

std::string chassis_location_station()
{
    if (!ensure_chassis())
        return {};
    double x = 0, y = 0, angle = 0, confidence = 0;
    std::string cur, last;
    int loc_state = 0;
    bool in_forbidden = false;
    if (chassis_seer().GetLocation(
            x, y, angle, cur, last, confidence, loc_state, in_forbidden) != 0)
        return {};
    return cur;
}

namespace {
void note_station_arrived()
{
    g_station_arrived_at = std::chrono::steady_clock::now();
    g_station_arrived = true;
}
}

int chassis_goto_station(
    const std::string &station,
    bool watch_obstacle,
    const std::function<void()> &on_station_name)
{
    TimingSpan span("nav");
    auto done = [&](int code) {
        span.set_ok(code == 0);
        return code;
    };
    if (g_hw == nullptr)
        return done(-1);
    g_hw->last_chassis_ok = false;
    g_hw->last_chassis_station = station;
    g_hw->last_chassis_message.clear();

    const auto &ch = g_move_cfg.chassis;
    if (ch.host.empty() || station.empty())
    {
        g_hw->last_chassis_message = "chassis.host 或站点名为空";
        std::cerr << "[orch] " << g_hw->last_chassis_message << "\n";
        return done(-1);
    }

    auto aborted = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };

    std::cout << "[orch] 底盘去 " << station << "\n";
    if (!ensure_chassis())
    {
        g_hw->last_chassis_message = chassis_seer().LastMessage().empty()
                                         ? "底盘连接失败"
                                         : chassis_seer().LastMessage();
        std::cerr << "[orch] 底盘连接失败: " << g_hw->last_chassis_message << "\n";
        return done(-6);
    }
    Chassis &robot = chassis_seer();

    double x = 0, y = 0, angle = 0, confidence = 0;
    std::string cur, last;
    int loc_state = 0;
    bool in_forbidden = false;
    int loc_rc = robot.GetLocation(x, y, angle, cur, last, confidence, loc_state, in_forbidden);
    if (loc_rc == 0)
        publish_dispatch_pose(x, y, angle, cur);
    if (loc_rc == 0 && station_name_matches(cur, station))
    {
        std::cout << "[orch] 底盘已在 " << cur << "，不再发导航\n";
        note_station_arrived();
        g_hw->last_chassis_ok = true;
        g_hw->last_chassis_station = cur;
        g_hw->last_chassis_message = "已在目标站";
        board_station(cur, {});
        return done(0);
    }
    board_station(loc_rc == 0 ? cur : std::string(), station);

    const int go_rc = robot.GoToStation(station);
    if (go_rc != 0)
    {
        g_hw->last_chassis_message = robot.LastMessage().empty()
                                         ? ("GoToStation 失败 code=" + std::to_string(go_rc))
                                         : robot.LastMessage();
        std::cerr << "[orch] 底盘去 " << station << " 下发失败 code=" << go_rc
                  << " " << g_hw->last_chassis_message << "\n";
        return done(go_rc < 0 ? go_rc : -6);
    }
    std::cout << "[orch] 已通知底盘去 " << station << "，等待到位\n";

    const int timeout_ms = std::max(1000, ch.nav_timeout_ms);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    bool logged_name_early = false;
    bool upper_started = false;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (aborted())
        {
            std::cerr << "[orch] 等待底盘到位时中止，取消导航\n";
            robot.CancelNav();
            g_hw->last_chassis_message = "中止";
            return done(-4);
        }

        loc_rc = robot.GetLocation(x, y, angle, cur, last, confidence, loc_state, in_forbidden);
        int task_status = 0, task_type = 0;
        std::string target_id;
        double tx = 0, ty = 0, ta = 0, dist = 0;
        std::vector<std::string> finished, unfinished;
        const int nav_rc =
            robot.GetNavStatus(task_status, task_type, target_id, tx, ty, ta, finished, unfinished, dist);
        if (watch_obstacle)
        {
            bool blocked = false;
            bool slowed = false;
            double block_x = 0.0;
            double block_y = 0.0;
            std::vector<std::string> obstacles;
            if (robot.GetBlocked(blocked, slowed, block_x, block_y, obstacles) == 0 && blocked)
            {
                std::cerr << "[orch] 底盘遇到障碍，取消导航\n";
                robot.CancelNav();
                g_hw->last_chassis_message = "遇到障碍";
                g_dispatch_nav_fault = dispatch_msg::ERR_AGV_OBSTACLE;
                return done(-11);
            }
        }

        if (nav_rc == 0)
        {
            if (task_status == 4 && station_name_matches(target_id, station))
            {
                std::cout << std::fixed << std::setprecision(3)
                          << "[orch] 底盘导航完成并停稳 target=" << target_id
                          << " x=" << x << " y=" << y << "\n";
                note_station_arrived();
                g_hw->last_chassis_ok = true;
                g_hw->last_chassis_station = target_id;
                g_hw->last_chassis_message = "导航完成";
                board_station(target_id, {});
                return done(0);
            }
            if (loc_rc == 0 && station_name_matches(cur, station) && task_status != 4)
            {
                if (!logged_name_early)
                {
                    logged_name_early = true;
                    std::cout << "[orch] 底盘站名已是 " << cur
                              << "，导航状态=" << task_status << "，尚未停稳，继续等\n";
                }
                if (!upper_started && on_station_name)
                {
                    upper_started = true;
                    std::cout << "[orch] 站名已变为 " << cur
                              << "，提前准备腰和手臂\n" << std::flush;
                    on_station_name();
                }
            }
            if (task_status == 5 || task_status == 6)
            {
                g_hw->last_chassis_message =
                    (task_status == 5 ? "导航失败" : "导航取消") + std::string(" target=") + target_id;
                std::cerr << "[orch] 底盘 " << g_hw->last_chassis_message << "\n";
                board_error("底盘" + g_hw->last_chassis_message);
                return done(-6);
            }
        }
        hardware_abort_sleep_ms(300);
    }

    g_hw->last_chassis_message = "等待到位超时";
    std::cerr << "[orch] 底盘去 " << station << " 超时\n";
    board_error("底盘去 " + station + " 超时");
    robot.CancelNav();
    return done(-9);
}
}
