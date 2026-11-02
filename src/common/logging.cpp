#include "t170/common/logging.h"
#include "t170/common/paths.h"

#include <ctime>
#include <iomanip>
#include <sstream>

namespace t170::logging {
std::filesystem::path file(const std::filesystem::path &relative) {
    return (paths::log_directory()/relative).lexically_normal();
}
std::string local_time(const char *format) {
    const std::time_t now=std::time(nullptr);
    std::tm local{};
    localtime_r(&now,&local);
    std::ostringstream out;
    out<<std::put_time(&local,format);
    return out.str();
}
}
