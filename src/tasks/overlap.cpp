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
#include "Ti5_Arm.h"
#include "t170/common/units.h"
#include "t170/motion/cartesian_tracking.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
namespace orch {
void start_belt_depart_after_right(const std::string &station, const char *hand_zh)
{
    if (station.empty() || g_belt_depart_started)
        return;
    g_belt_depart_started = true;
    g_belt_depart_rc = 0;
    const std::string who = (hand_zh && hand_zh[0]) ? hand_zh : "手";
    g_belt_depart_th = std::thread([station, who]() {
        if (hardware_abort_requested())
        {
            g_belt_depart_rc = -4;
            return;
        }
        std::cout << "[orch] " << who << "收手完成，底盘出发去 " << station << "\n" << std::flush;
        try
        {
            g_belt_depart_rc = chassis_goto_station(station);
        }
        catch (const std::exception &ex)
        {
            std::cerr << "[orch] 底盘去 " << station << " 失败: " << ex.what() << "\n";
            g_belt_depart_rc = -6;
        }
    });
}

void start_tray2_waist_after_right_arrived()
{
    if (g_tray2_waist_started || g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return;
    g_tray2_waist_started = true;
    g_tray2_waist_rc = 0;
    g_tray2_waist_th = std::thread([]() {
        std::cout << "[orch] 右手收手已到位，2s 后腰降到料盘放置高度\n" << std::flush;
        hardware_abort_sleep_ms(2000);
        if (hardware_abort_requested())
        {
            g_tray2_waist_rc = -4;
            return;
        }
        std::cout << "[orch] 右手到位已 2s，开始降腰\n" << std::flush;
        try
        {
            if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
                g_tray2_waist_rc = -5;
        }
        catch (const std::exception &ex)
        {
            std::cerr << "[orch] 去料盘时腰部失败: " << ex.what() << "\n";
            g_tray2_waist_rc = -5;
        }
    });
}

int take_belt_depart()
{
    if (g_belt_depart_th.joinable())
        g_belt_depart_th.join();
    if (!g_belt_depart_started)
        return 0;
    const int rc = g_belt_depart_rc.load();
    g_belt_depart_started = false;
    return rc;
}

int take_tray2_waist()
{
    if (g_tray2_waist_th.joinable())
        g_tray2_waist_th.join();
    if (!g_tray2_waist_started)
        return 0;
    const int rc = g_tray2_waist_rc.load();
    g_tray2_waist_started = false;
    // 手臂已在放置准备位，腰也降到 ready：后面进 vision_place_tray2 不必再摆一次。
    if (rc == 0)
        g_tray2_place_prep_done = true;
    return rc;
}
}
