#include "vision_debug.h"

#include "seg_pose_bridge.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

namespace
{

std::mutex g_mu;
int g_level = -1; // -1 未初始化

void apply_level_unlocked(int level)
{
    if (level < 0)
        level = 0;
    if (level > 2)
        level = 2;
    g_level = level;
}

int ensure_level_unlocked()
{
    if (g_level >= 0)
        return g_level;
    int level = 0;
    if (const char *v = std::getenv("T170_VISION_DEBUG"))
    {
        if (std::strcmp(v, "1") == 0)
            level = 1;
        else if (std::strcmp(v, "2") == 0)
            level = 2;
        else if (v[0] != '\0' && std::strcmp(v, "0") != 0)
        {
            char *end = nullptr;
            const long n = std::strtol(v, &end, 10);
            if (end != v && *end == '\0')
            {
                if (n >= 2)
                    level = 2;
                else if (n == 1)
                    level = 1;
            }
        }
    }
    apply_level_unlocked(level);
    return g_level;
}

std::string stamp_name()
{
    using clock = std::chrono::system_clock;
    const auto tp = clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()) %
                    1000;
    const std::time_t sec = clock::to_time_t(tp);
    std::tm tm{};
    localtime_r(&sec, &tm);
    std::ostringstream name;
    name << std::put_time(&tm, "%Y%m%d_%H%M%S") << '_' << std::setw(3) << std::setfill('0')
         << ms.count() << ".jpg";
    return name.str();
}

} // namespace

void vision_debug_init_from_env()
{
    std::lock_guard<std::mutex> lk(g_mu);
    g_level = -1;
    const int level = ensure_level_unlocked();
    std::cerr << "[vision_debug] T170_VISION_DEBUG=" << level
              << (level == 0   ? "（默认：业务不写调试图）"
                  : level == 1 ? "（写 picture_debug）"
                               : "（写图 + GUI）")
              << "\n";
}

bool vision_debug_annotate()
{
    std::lock_guard<std::mutex> lk(g_mu);
    return ensure_level_unlocked() >= 1;
}

bool vision_debug_save_pairs()
{
    std::lock_guard<std::mutex> lk(g_mu);
    return ensure_level_unlocked() >= 1;
}

bool vision_debug_gui()
{
    std::lock_guard<std::mutex> lk(g_mu);
    return ensure_level_unlocked() >= 2;
}

int vision_debug_level()
{
    std::lock_guard<std::mutex> lk(g_mu);
    return ensure_level_unlocked();
}

std::string vision_debug_root_dir()
{
    return project_root_dir() + "/picture_debug";
}

std::string vision_debug_make_path(const char *slot, const char *stem)
{
    const char *s = (slot && slot[0]) ? slot : "head";
    const char *st = (stem && stem[0]) ? stem : "vision";
    const std::string dir = vision_debug_root_dir() + "/" + s;
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return dir + "/" + st + "_" + stamp_name();
}

std::string vision_debug_annotate_path(const char *slot, const char *stem)
{
    if (!vision_debug_annotate())
        return {};
    return vision_debug_make_path(slot, stem);
}
