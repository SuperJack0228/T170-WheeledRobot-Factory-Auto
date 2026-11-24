#include "internal/state.h"
#include "t170/config/access.h"
#include "t170/hardware/device_bus.h"
#include "t170/vision/debug.h"
#include "t170/common/paths.h"
#include <filesystem>
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
        const auto current_path=default_move_box_config_path();
        std::string candidate=request.get("candidate_config",current_path).asString();
        if(candidate!=current_path) {
            const auto p=std::filesystem::weakly_canonical(candidate);
            const auto root=std::filesystem::weakly_canonical(t170::paths::resource_root());
            if(p.filename()!="effective_params.yaml" || p.parent_path().filename()!="config" ||
               p.parent_path().parent_path().parent_path()!=root ||
               p.parent_path().parent_path().filename().string().rfind(".config-apply-",0)!=0)
                return reply(false,"配置候选文件不在当前运行目录内");
            candidate=p.string();
        }
        const auto result=t170::config::reload_when_idle(g_hw_mu,
            []{return g_placement_calibration.active();},app_stop_requested,
            candidate,g_move_cfg,candidate==current_path?std::string{}:current_path);
        Json::Value out = reply(result.ok,result.error);
        out["code"]=result.code;
        if(result.ok) {
            std::lock_guard<std::mutex> lock(g_config_mu);
            out["charge_done_percent"] = g_move_cfg.chassis.charge_done_percent;
        }
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
