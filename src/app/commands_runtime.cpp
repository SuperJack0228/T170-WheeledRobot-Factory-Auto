#include "internal/state.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>

namespace t170::app::detail {
std::optional<Json::Value> handle_runtime_commands(const Json::Value &request) {
    const std::string cmd=request.get("cmd","").asString();
if (cmd == "abort" || cmd == "dispatch_exit")
    {
        g_abort.store(true);
        g_placement_calibration.interrupt();
        hardware_abort_request();
        Json::Value out = reply(true);
        out["dispatch_exit"] = (cmd == "dispatch_exit");
        if (cmd == "dispatch_exit")
            out["message"] = "已请求退出调度（中止动作并离开调度模式）";
        return out;
    }
if (cmd == "reload_config")
    {
        if(g_placement_calibration.active()) return reply(false,"标定期间不能重载正式配置");
        std::lock_guard<std::mutex> config_lock(g_config_mu);
        if(g_placement_calibration.active()) return reply(false,"标定期间不能重载正式配置");
        std::string err;
        if (!load_move_box_config(default_move_box_config_path(), g_move_cfg, err))
            return reply(false, err);
        Json::Value out = reply(true);
        out["charge_done_percent"] = g_move_cfg.chassis.charge_done_percent;
        return out;
    }
if (cmd.size() == 6 && cmd.compare(0, 5, "Audio") == 0 && cmd[5] >= '1' && cmd[5] <= '5')
    {
        static const char *kClips[] = {"initialize", "initialize_finish", "dispatch", "standby",
                                        "failure"};
        const char *clip = kClips[cmd[5] - '1'];
        std::string err;
        if (!orch::play_boot_clip(clip, err, false))
            return reply(false, err.empty() ? "播放失败" : err);
        Json::Value out = reply(true);
        out["audio"] = clip;
        return out;
    }
    return std::nullopt;
}
}
