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
namespace {

}

int dispatch_wait_system(DispatchClient &client, const std::string &state)
{
    std::cerr << "[dispatch] 异常 " << state << "，停止动作，保持连接等待系统处理\n";
    board_error("异常 " + state + "（" + dispatch_state_zh(state) + "），停止动作，等待系统处理");
    board_activity("异常停机，等待系统处理");
    dispatch_note_state(state);
    const char *zh = nullptr;
    if (state == dispatch_msg::ERR_MOTOR_LOST || state == dispatch_msg::ERR_MOTOR_DISABLE)
        zh = "电机连接掉线";
    else if (state == dispatch_msg::ERR_CAMERA)
        zh = "相机连接掉线";
    if (zh != nullptr)
        return dispatch_report_ex(client, zh);
    while (!hardware_abort_requested())
        hardware_abort_sleep_ms(500);
    return -4;
}

std::string dispatch_exception_state()
{
    if (!g_dispatch_nav_fault.empty())
        return g_dispatch_nav_fault;
    if (can_io_faulted())
    {
        const std::string msg = can_io_fault_message();
        if (msg.find("使能") != std::string::npos)
            return dispatch_msg::ERR_MOTOR_DISABLE;
        return dispatch_msg::ERR_MOTOR_LOST;
    }
    if (g_hw != nullptr && g_hw->g != nullptr)
    {
        const gripper::SideFeedback right = g_hw->g->feedback(gripper::Side::Right);
        const gripper::SideFeedback left = g_hw->g->feedback(gripper::Side::Left);
        if ((right.have_feedback && !right.io_ok) || (left.have_feedback && !left.io_ok))
            return dispatch_msg::ERR_GRIPPER_LOST;
    }
    if (g_hw != nullptr)
    {
        const std::string &belt = g_hw->last_belt_message;
        const std::string &tray = g_hw->last_tray_message;
        if (belt == "无相机" || belt == "取帧失败" || tray == "取帧失败")
            return dispatch_msg::ERR_CAMERA;
    }
    return {};
}
}
