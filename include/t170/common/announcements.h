#pragma once
#include <string>
namespace orch {
bool play_boot_clip(const std::string &name, std::string &err, bool once_if_failure = false);
void boot_announce(const std::string &name);
}
