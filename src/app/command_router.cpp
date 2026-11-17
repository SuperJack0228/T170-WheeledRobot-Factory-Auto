#include "internal/state.h"
#include "t170/calibration/commands.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

namespace t170::app::detail {
Json::Value capabilities_json() {
    Json::Value out=reply(true);out["mode"]=mode_name(state().mode);
    Json::Value commands(Json::arrayValue);
    for(const auto &spec:command_catalog()) {
        Json::Value item(Json::objectValue);item["name"]=std::string(spec.name);
        item["group"]=group_name(spec.group);item["allowed"]=command_allowed(state().mode,spec.name);
        commands.append(item);
    }out["commands"]=commands;return out;
}
Json::Value dispatch(const Json::Value &request) {
    const std::string cmd=request.get("cmd","").asString();
    if(!find_command(cmd)) return reply(false,"未知命令: "+cmd);
    if(find_command(cmd)->group==CommandGroup::Retired) return reply(false,"该实验命令已退役，请使用保留的单步抓放或调度入口");
    if(!command_allowed(state().mode,cmd)) return reply(false,"生产模式只允许运行控制和状态命令；标定/手动动作请使用维护入口");
    if(cmd=="capabilities") return capabilities_json();
    if(cmd=="ping") {auto out=reply(true);out["mode"]=mode_name(state().mode);return out;}
    if(cmd=="calibration_status" || cmd=="placement_calibration")
        return t170::calibration::handle_command(request,{g_placement_calibration,g_session,g_hw_mu,g_config_mu,g_abort,app_stop_requested});
    if(auto out=handle_runtime_commands(request)) return *out;
    if(cmd=="dispatch_status") return dispatch_status_json();
std::unique_lock<std::mutex> lock(g_hw_mu,std::try_to_lock);
if(!lock.owns_lock()) return reply(false,"机器人动作忙，请等待当前动作结束");
if(g_placement_calibration.active() && cmd!="snapshot")
        return reply(false,"标定会话占用机器人，请从标定页面退出后再执行普通动作");
if (app_stop_requested())
    {
        g_abort.store(true);
        hardware_abort_request();
        return reply(false, "进程正在退出");
    }
if(!g_placement_calibration.active()) {g_abort.store(false);hardware_abort_clear();}
if (app_stop_requested())
    {
        g_abort.store(true);
        hardware_abort_request();
        return reply(false, "进程正在退出");
    }
if(!g_placement_calibration.active()) can_io_fault_clear();
can_io_watch_begin();
struct WatchEnd
    {
        ~WatchEnd() { can_io_watch_end(); }
    } watch_end;
    if(auto out=handle_status_commands(request)) return *out;
    if(auto out=handle_operation_commands(request)) return *out;
    if(auto out=handle_maintenance_commands(request)) return *out;
    if(auto out=handle_manual_tasks_commands(request)) return *out;
    if(auto out=handle_inspection_commands(request)) return *out;
    return reply(false,"未知命令: "+cmd);
}
}
