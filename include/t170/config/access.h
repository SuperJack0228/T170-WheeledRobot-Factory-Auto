#pragma once
#include "t170/common/config.h"
#include <functional>
#include <mutex>
#include <string>

namespace t170::config {
// Motion/task readers already own the application hardware mutex. Short-lived
// background readers use this mutex. Writers own BOTH, in hardware -> config order.
std::mutex &access_mutex();
// Call once before starting application threads. These resources are held by
// SDK/camera/Python objects and cannot be changed by reloading MoveBoxConfig.
void capture_startup_resources(const std::string &config_directory);
struct ReloadResult {
    bool ok = false;
    std::string code, error;
};
ReloadResult reload_when_idle(std::mutex &hardware_mutex,
    const std::function<bool()> &calibration_active,
    const std::function<bool()> &stopping,
    const std::string &path, MoveBoxConfig &current,
    const std::string &publish_path = {});
}
