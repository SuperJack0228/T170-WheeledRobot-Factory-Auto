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
namespace {
std::string dispatch_port(const std::string &port_one, const std::string &port_two)
{
    if (!port_one.empty())
        return port_one;
    return port_two;
}
}

TaskPoints read_task_points()
{
    TaskPoints p;
    p.raw = dispatch_gateway().snapshot().raw_material_point;
    p.well = dispatch_gateway().snapshot().well_done_point;
    // 圆方只写口一。海德曼本次允许的口可能在口二，口一空时用口二。
    p.load = dispatch_port(dispatch_gateway().snapshot().sequence_one_port_one, dispatch_gateway().snapshot().sequence_one_port_two);
    p.unload = dispatch_port(dispatch_gateway().snapshot().sequence_two_port_one, dispatch_gateway().snapshot().sequence_two_port_two);
    std::cout << "[dispatch] 调度点位 上料架=" << p.raw
              << " 工序一口一=" << dispatch_gateway().snapshot().sequence_one_port_one
              << " 口二=" << dispatch_gateway().snapshot().sequence_one_port_two
              << " 采用上料点=" << p.load
              << " 工序二口一=" << dispatch_gateway().snapshot().sequence_two_port_one
              << " 口二=" << dispatch_gateway().snapshot().sequence_two_port_two
              << " 采用下料点=" << p.unload
              << " 下料架=" << p.well
              << " 待命点=" << dispatch_gateway().snapshot().standby_point
              << " 充电点=" << dispatch_gateway().snapshot().charge_point << "\n";
    return p;
}

bool task_points_ready(const std::string &task, const TaskPoints &p)
{
    auto miss = [&](const char *name, const std::string &value) {
        if (!value.empty())
            return false;
        std::cerr << "[dispatch] 任务 " << task << " 缺少点位 " << name << "\n";
        return true;
    };
    if (task == dispatch_msg::TASK_LOAD)
        return !(miss("raw_material_point", p.raw) || miss("sequence_one_port", p.load));
    if (task == dispatch_msg::TASK_TRANSFER)
        return !(miss("sequence_one_port", p.load) || miss("sequence_two_port", p.unload));
    if (task == dispatch_msg::TASK_UNLOAD)
        return !(miss("sequence_two_port", p.unload) || miss("well_done_point", p.well));
    return false;
}

std::string dispatch_idle_station()
{
    if (!dispatch_gateway().snapshot().standby_point.empty())
        return dispatch_gateway().snapshot().standby_point;
    if (!g_move_cfg.chassis.dispatch_wait_station.empty())
        return g_move_cfg.chassis.dispatch_wait_station;
    return g_move_cfg.chassis.tray_station;
}

namespace {

}

std::string resolve_charge_station()
{
    if (!dispatch_gateway().snapshot().charge_point.empty())
        return dispatch_gateway().snapshot().charge_point;
    return g_move_cfg.chassis.charge_station;
}
}
