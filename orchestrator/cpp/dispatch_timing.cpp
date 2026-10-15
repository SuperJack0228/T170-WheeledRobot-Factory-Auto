#include "dispatch_timing.h"

#include "seg_pose_bridge.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>

namespace orch
{
namespace
{

std::mutex g_mu;
int g_level = -1; // -1=未读环境变量
std::ofstream g_file;
std::string g_path;
std::string g_mission_id;
std::string g_mission;
std::string g_step;
std::int64_t g_mission_t0_ms = 0;
std::int64_t g_step_t0_ms = 0;
bool g_step_open = false;

std::int64_t now_steady_ms()
{
    using clock = std::chrono::steady_clock;
    return std::chrono::duration_cast<std::chrono::milliseconds>(clock::now().time_since_epoch())
        .count();
}

std::string now_wall_iso()
{
    using clock = std::chrono::system_clock;
    const auto tp = clock::now();
    const std::time_t t = clock::to_time_t(tp);
    std::tm tm{};
    localtime_r(&t, &tm);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()) %
                    1000;
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03d", tm.tm_year + 1900,
                  tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec,
                  static_cast<int>(ms.count()));
    return buf;
}

std::string now_stamp()
{
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

std::string json_escape(const std::string &s)
{
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s)
    {
        switch (c)
        {
        case '\\':
            out += "\\\\";
            break;
        case '"':
            out += "\\\"";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            out += c;
            break;
        }
    }
    return out;
}

int read_level_unlocked()
{
    if (g_level >= 0)
        return g_level;
    g_level = 0;
    if (const char *v = std::getenv("T170_TIMING"))
    {
        if (std::strcmp(v, "1") == 0)
            g_level = 1;
        else if (std::strcmp(v, "2") == 0)
            g_level = 2;
        else if (v[0] != '\0' && std::strcmp(v, "0") != 0)
        {
            // 其它非空值：尽量当作开 L1，避免误开过重 L2。
            char *end = nullptr;
            const long n = std::strtol(v, &end, 10);
            if (end != v && *end == '\0' && n >= 2)
                g_level = 2;
            else if (end != v && *end == '\0' && n == 1)
                g_level = 1;
        }
    }
    return g_level;
}

bool ensure_file_unlocked()
{
    if (g_file.is_open())
        return true;
    if (read_level_unlocked() <= 0)
        return false;
    const std::filesystem::path dir = std::filesystem::path(project_root_dir()) / "logs";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    g_path = (dir / ("timing_" + now_stamp() + ".jsonl")).string();
    g_file.open(g_path, std::ios::out | std::ios::app);
    if (!g_file)
    {
        std::cerr << "[timing] 无法写入 " << g_path << "\n";
        g_path.clear();
        return false;
    }
    std::cerr << "[timing] T170_TIMING=" << g_level << "，写入 " << g_path << "\n";
    return true;
}

void write_line_unlocked(const std::string &line)
{
    if (!ensure_file_unlocked())
        return;
    g_file << line << '\n';
    g_file.flush();
}

void close_step_unlocked(bool ok)
{
    if (!g_step_open || g_step.empty())
    {
        g_step_open = false;
        return;
    }
    const std::int64_t now = now_steady_ms();
    const std::int64_t dur = now - g_step_t0_ms;
    std::ostringstream os;
    os << "{\"type\":\"step\",\"mission_id\":\"" << json_escape(g_mission_id)
       << "\",\"mission\":\"" << json_escape(g_mission) << "\",\"step\":\""
       << json_escape(g_step) << "\",\"dur_ms\":" << dur << ",\"ok\":" << (ok ? "true" : "false")
       << ",\"wall\":\"" << now_wall_iso() << "\"}";
    write_line_unlocked(os.str());
    g_step_open = false;
    g_step.clear();
}

} // namespace

int timing_level()
{
    std::lock_guard<std::mutex> lk(g_mu);
    return read_level_unlocked();
}

void timing_mission_begin(const std::string &mission)
{
    std::lock_guard<std::mutex> lk(g_mu);
    if (read_level_unlocked() <= 0)
        return;
    close_step_unlocked(true);
    g_mission = mission;
    g_mission_t0_ms = now_steady_ms();
    {
        std::ostringstream id;
        id << now_stamp() << "_" << (g_mission_t0_ms % 100000);
        g_mission_id = id.str();
    }
    g_step.clear();
    g_step_open = false;
    std::ostringstream os;
    os << "{\"type\":\"mission_start\",\"mission_id\":\"" << json_escape(g_mission_id)
       << "\",\"mission\":\"" << json_escape(mission) << "\",\"level\":" << g_level
       << ",\"wall\":\"" << now_wall_iso() << "\"}";
    write_line_unlocked(os.str());
}

void timing_mission_end(bool ok, const std::string &result)
{
    std::lock_guard<std::mutex> lk(g_mu);
    if (read_level_unlocked() <= 0 || g_mission_id.empty())
        return;
    close_step_unlocked(ok);
    const std::int64_t dur = now_steady_ms() - g_mission_t0_ms;
    std::ostringstream os;
    os << "{\"type\":\"mission_end\",\"mission_id\":\"" << json_escape(g_mission_id)
       << "\",\"mission\":\"" << json_escape(g_mission) << "\",\"dur_ms\":" << dur
       << ",\"ok\":" << (ok ? "true" : "false") << ",\"result\":\"" << json_escape(result)
       << "\",\"wall\":\"" << now_wall_iso() << "\"}";
    write_line_unlocked(os.str());
    g_mission_id.clear();
    g_mission.clear();
}

void timing_step(const std::string &step)
{
    std::lock_guard<std::mutex> lk(g_mu);
    if (read_level_unlocked() <= 0 || g_mission_id.empty())
        return;
    close_step_unlocked(true);
    if (step.empty())
        return;
    g_step = step;
    g_step_t0_ms = now_steady_ms();
    g_step_open = true;
}

TimingSpan::TimingSpan(const char *phase)
{
    if (phase == nullptr || phase[0] == '\0')
        return;
    std::lock_guard<std::mutex> lk(g_mu);
    if (read_level_unlocked() < 2 || g_mission_id.empty())
        return;
    active_ = true;
    phase_ = phase;
    t0_ms_ = now_steady_ms();
}

void TimingSpan::set_ok(bool ok)
{
    ok_ = ok;
}

TimingSpan::~TimingSpan()
{
    if (!active_)
        return;
    std::lock_guard<std::mutex> lk(g_mu);
    if (read_level_unlocked() < 2 || g_mission_id.empty())
        return;
    const std::int64_t dur = now_steady_ms() - t0_ms_;
    std::ostringstream os;
    os << "{\"type\":\"span\",\"mission_id\":\"" << json_escape(g_mission_id)
       << "\",\"mission\":\"" << json_escape(g_mission) << "\",\"step\":\""
       << json_escape(g_step) << "\",\"phase\":\"" << json_escape(phase_)
       << "\",\"dur_ms\":" << dur << ",\"ok\":" << (ok_ ? "true" : "false") << ",\"wall\":\""
       << now_wall_iso() << "\"}";
    write_line_unlocked(os.str());
}

} // namespace orch
