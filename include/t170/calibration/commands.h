#pragma once
#include "t170/calibration/placement_session.h"
#include <atomic>
#include <functional>
#include <mutex>
namespace t170::calibration {
struct CommandContext {
    PlacementSession &session;
    orch::HwSession &hardware;
    std::mutex &hardware_mutex,&config_mutex;
    std::atomic<bool> &abort;
    std::function<bool()> process_stopping;
};
Json::Value handle_command(const Json::Value &,CommandContext);
}
