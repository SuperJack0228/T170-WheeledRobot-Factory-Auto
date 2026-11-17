#include "t170/hardware/body_power.h"
#include "internal/runtime.h"
#include "t170/skills/conveyor.h"

#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "Ti5_Arm.h"
#include "t170/common/units.h"
#include "t170/app/diagnostic_gestures.h"
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
int set_body_motors(bool enable) {
    const int code=t170::hardware::set_body_motors(enable);
    if(code==0)g_body_disabled_for_charge=!enable;
    return code;
}

namespace {
int stop_chassis_charging()
{
    if (!ensure_chassis())
        return -6;
    const int rc = chassis_seer().StopCharging();
    if (rc != 0)
    {
        std::cerr << "[dispatch] 停止充电失败 code=" << rc << " "
                  << chassis_seer().LastMessage() << "\n";
        return rc < 0 ? rc : -6;
    }
    std::cout << "[dispatch] 已调用停止充电\n";
    return 0;
}
}

int run_charge_flow(DispatchClient *client, bool stop_when_charging)
{
    const std::string station = resolve_charge_station();
    if (station.empty())
    {
        std::cerr << "[dispatch] 充电任务缺少充电点（调度 charge_point 或 yaml chassis.charge_station），"
                     "不动作\n";
        return kDispatchPointsMissing;
    }
    if (g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
    {
        std::cerr << "[dispatch] 手臂未初始化，不充电\n";
        return -1;
    }
    const std::string wait_ap = dispatch_idle_station();

    // 先在本机记下准备去充电，再整机 home → 待命点 → 挥手下垂 → 腰降抓取高，最后才失能去充电点。
    // 结束电量阈值每秒从 g_move_cfg 重读，网页改参 reload 后本次充电即可生效。
    std::cout << "[dispatch] 充电："
              << (stop_when_charging
                      ? "调试到吸合即停"
                      : ("调度到 " +
                         std::to_string(static_cast<int>(
                             std::lround(g_move_cfg.chassis.charge_done_percent))) +
                         "%（可热改）"))
              << "，home → " << wait_ap << " → 挥手下垂 → 腰降抓取高，再去 " << station << "\n";
    if (client != nullptr)
    {
        dispatch_note_state(dispatch_msg::GO_CHARGE);
        board_step("准备去充电");
    }
    board_activity("去充电：整机回 home");

    board_step("整机 home");
    const int home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
    if (home_rc != 0)
    {
        std::cerr << "[dispatch] 充电前 home 失败，不继续\n";
        return home_rc;
    }
    if (hardware_abort_requested())
        return -4;

    board_step("去等待点");
    board_activity("去充电：导航到 " + wait_ap);
    std::cout << "[dispatch] 充电前先到等待点 " << wait_ap << "\n";
    {
        ChassisTravel travel(wait_ap);
        const int wait_rc = travel.join();
        if (wait_rc != 0)
            return wait_rc;
    }
    if (hardware_abort_requested())
        return -4;

    board_step("挥手下垂");
    board_activity("去充电：双手挥手前下垂");
    std::cout << "[dispatch] 双手进入挥手前下垂姿态\n";
    const int hang_rc = arm_wave_hang(*g_hw->arm_r, *g_hw->arm_l);
    if (hang_rc != 0)
    {
        std::cerr << "[dispatch] 挥手前下垂失败 code=" << hang_rc << "，不充电\n";
        return hang_rc == -4 || hardware_abort_requested() ? -4 : -5;
    }
    if (hardware_abort_requested())
        return -4;

    board_step("腰降抓取高");
    board_activity("去充电：腰降到抓取高度");
    std::cout << std::fixed << std::setprecision(4)
              << "[dispatch] 腰降到抓取高度 x=" << waist_ready_x()
              << " z=" << waist_ready_z() << "\n";
    if (g_hw->waist == nullptr || g_hw->layer3 == nullptr ||
        !move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
    {
        std::cerr << "[dispatch] 腰未能降到抓取高度，不充电\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (hardware_abort_requested())
        return -4;

    board_step("失能电机");
    board_activity("去充电：失能后导航到充电点");
    if (set_body_motors(false) != 0)
    {
        set_body_motors(true);
        return -5;
    }
    struct ReenableMotors
    {
        ~ReenableMotors()
        {
            if (g_body_disabled_for_charge)
                set_body_motors(true);
        }
    } reenable;
    // 准备点已经到位，底盘出发去充电点。commuting 只记在本机面板。
    board_step("去充电路上");
    if (client != nullptr)
        dispatch_note_state(dispatch_msg::COMMUTING);
    std::cout << "[dispatch] 导航去充电点 " << station << "\n";
    board_step("去充电点");
    board_activity("去充电路上 → " + station);
    const int nav = chassis_goto_station(station);
    if (nav != 0)
        return nav;
    board_step(stop_when_charging ? "等待吸合" : "充电中");
    if (stop_when_charging)
        std::cout << "[dispatch] 已到充电点 " << station << "，等待检测到开始充电后结束指令\n";
    else
        std::cout << "[dispatch] 已到充电点 " << station
                  << "，每秒读电量与 charge_done_percent，达阈值后汇报完成并停止充电\n";
    int shown_pct = -1;
    int board_pct = -100;
    int shown_done_pct = -1;
    bool reported_charging = false;
    double last_pct = 0.0;
    double last_done_pct = g_move_cfg.chassis.charge_done_percent;
    while (!hardware_abort_requested())
    {
        if (!ensure_chassis())
        {
            hardware_abort_sleep_ms(1000);
            continue;
        }
        double level = 0, voltage = 0, current = 0, temp = 0;
        bool charging = false;
        int cycle = 0;
        if (chassis_seer().GetBattery(level, charging, voltage, current, temp, cycle) != 0)
        {
            hardware_abort_sleep_ms(1000);
            continue;
        }
        publish_dispatch_battery(level);
        if (charging && !reported_charging)
        {
            reported_charging = true;
            std::cout << "[dispatch] 检测到正在充电";
            if (client != nullptr)
            {
                std::cout << "，本机记 " << dispatch_msg::CHARGING << "\n";
                dispatch_note_state(dispatch_msg::CHARGING);
            }
            else
            {
                std::cout << "\n";
            }
            board_activity("充电中（已吸合）");
            if (stop_when_charging)
            {
                board_step("检测到充电");
                std::cout << "[dispatch] 调试充电：已开始充电，指令结束（不停充）\n";
                return 0;
            }
        }
        const double pct = level <= 1.5 ? level * 100.0 : level;
        last_pct = pct;
        const double done_pct = g_move_cfg.chassis.charge_done_percent;
        last_done_pct = done_pct;
        const int pct_i = static_cast<int>(std::lround(pct));
        const int done_i = static_cast<int>(std::lround(done_pct));
        const bool thresh_changed = (done_i != shown_done_pct);
        if (thresh_changed)
        {
            shown_done_pct = done_i;
            std::cout << "[dispatch] 充电结束阈值热更新为 " << done_i << "%\n";
        }
        if (pct_i != shown_pct)
        {
            shown_pct = pct_i;
            std::cout << "[dispatch] 充电中 电量 " << pct_i << "%（阈值 " << done_i << "%）"
                      << (charging ? "，充电继电器吸合" : "，尚未充电") << "\n";
        }
        // 电量跳 5% 或阈值被热更新时，立刻刷新调度监控文案。
        if (thresh_changed || std::abs(pct_i - board_pct) >= 5)
        {
            board_pct = pct_i;
            board_activity(std::string(charging ? "充电中" : "等待充电吸合") + " 电量 " +
                           std::to_string(pct_i) + "% / 阈值 " + std::to_string(done_i) + "%");
        }
        if (!stop_when_charging && pct >= done_pct)
            break;
        hardware_abort_sleep_ms(1000);
    }
    if (hardware_abort_requested())
        return -4;
    if (client == nullptr)
        return -5;
    std::cout << "[dispatch] 电量 " << static_cast<int>(std::lround(last_pct)) << "% 达到阈值 "
              << static_cast<int>(std::lround(last_done_pct)) << "%，汇报充电完成\n";
    board_step("汇报完成");
    const bool reported = client->mission_done();
    board_report(dispatch_msg::DONE, "充电完成", reported);
    if (!reported)
        std::cerr << "[dispatch] 汇报充电完成失败，仍停止充电\n";
    else
        std::cout << "[dispatch] 已汇报充电完成\n";
    const int stop = stop_chassis_charging();
    if (!reported)
        return -7;
    return stop;
}

int dispatch_charge(DispatchClient &client)
{
    return run_charge_flow(&client, false);
}
}
