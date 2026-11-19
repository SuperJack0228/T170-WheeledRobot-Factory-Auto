#include "t170/calibration/commands.h"
#include "t170/hardware/device_bus.h"
namespace t170::calibration {
namespace {Json::Value reply(bool ok,const std::string &error){Json::Value out(Json::objectValue);out["ok"]=ok;out["error"]=error;return out;}}
Json::Value handle_command(const Json::Value &request,CommandContext context) {
    const std::string cmd=request.get("cmd","").asString();
if (cmd == "calibration_status") {
        auto state=context.session.status();state["software_stopped"]=hardware_abort_requested();return state;
    }
if (cmd == "placement_calibration") {
        std::unique_lock<std::mutex> lock(context.hardware_mutex,std::try_to_lock);
        if(!lock.owns_lock()) return reply(false,"机器人正在执行其他动作或调度，请先退出并等待停止");
        if(request.get("action","").asString()=="reset_stop") {
            if(context.session.active()) return reply(false,"请先退出已停止的标定会话");
            if(!request.get("acknowledged",false).asBool()) return reply(false,"请先确认现场状态");
            if(context.process_stopping()) return reply(false,"机器人服务正在退出");
            context.abort.store(false);hardware_abort_clear();
            auto state=context.session.status();state["software_stopped"]=false;return state;
        }
        if(request.get("action","").asString()=="begin") {
            if(hardware_abort_requested()) return reply(false,"停止状态尚未复位，请在向导首页确认现场并解除软件停止");
        }
        std::lock_guard<std::mutex> config_lock(context.config_mutex);
        auto state=context.session.execute(request,context.hardware);state["software_stopped"]=hardware_abort_requested();return state;
    }
    return reply(false,"未知标定命令");
}

}
