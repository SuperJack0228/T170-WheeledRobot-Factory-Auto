#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

namespace t170::app::detail {
std::optional<Json::Value> handle_operation_commands(const Json::Value &request) {
    const std::string cmd=request.get("cmd","").asString();
if (cmd == "dispatch")
    {
        const int rc = orch::run_dispatch_mode();
        Json::Value out = reply(rc >= 0, rc < 0 ? "调度对接失败/中止" : "");
        out["code"] = rc;
        out["chassis_ok"] = g_session.last_chassis_ok;
        out["chassis_station"] = g_session.last_chassis_station;
        out["chassis_message"] = g_session.last_chassis_message;
        return out;
    }
    return std::nullopt;
}
}
