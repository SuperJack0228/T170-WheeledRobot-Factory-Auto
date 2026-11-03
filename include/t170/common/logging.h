#pragma once

#include <filesystem>
#include <string>

namespace t170::logging {
// Shared destinations and time formatting; existing writers retain their own
// schemas, locking, retention and control-loop buffering policies.
std::filesystem::path file(const std::filesystem::path &relative);
std::string local_time(const char *format);
}
