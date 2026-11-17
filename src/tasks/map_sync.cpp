#include "internal/runtime.h"
#include "t170/telemetry/dispatch_metrics.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace orch {
namespace {
const std::filesystem::path kDispatchMapDir{"/opt/ti5-jindi-factory-robot-client/map"};
}

bool stage_dispatch_map_name()
{
    if (!ensure_chassis())
        return false;
    std::string current;
    std::string md5;
    std::vector<std::string> maps;
    std::vector<std::string> names;
    std::vector<std::string> sizes;
    std::vector<std::string> times;
    const int rc = chassis_seer().GetMapInfo(current, maps, md5, names, sizes, times);
    if (rc != 0 || current.empty())
    {
        std::cerr << "[dispatch] 读底盘当前地图名失败 code=" << rc << " "
                  << chassis_seer().LastMessage() << "\n";
        board_error("读底盘当前地图名失败，不连接调度");
        return false;
    }
    const std::string map_name = std::filesystem::path(current).filename().string();
    if (map_name.empty() || map_name == "." || map_name == "..")
    {
        std::cerr << "[dispatch] 底盘地图名无效: " << current << "\n";
        return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(kDispatchMapDir, ec);
    if (ec)
    {
        std::cerr << "[dispatch] 地图目录不可用 " << kDispatchMapDir.string()
                  << ": " << ec.message() << "\n";
        return false;
    }
    std::vector<std::filesystem::path> stale;
    for (const std::filesystem::directory_entry &ent :
         std::filesystem::directory_iterator(kDispatchMapDir, ec))
    {
        if (ec || !ent.is_regular_file() || ent.path().filename() == map_name)
            continue;
        stale.push_back(ent.path());
    }
    for (const std::filesystem::path &path : stale)
        std::filesystem::remove(path, ec);
    const std::filesystem::path named = kDispatchMapDir / map_name;
    if (!std::filesystem::is_regular_file(named))
    {
        std::ofstream out(named);
        if (!out)
        {
            std::cerr << "[dispatch] 写地图名文件失败 " << named.string() << "\n";
            return false;
        }
    }
    std::cout << "[dispatch] 连接调度前上报底盘地图名 " << map_name << "\n";
    board_note("info", "地图检查：上报底盘当前地图名 " + map_name);
    return true;
}

namespace {
bool load_dispatch_map_if_updated()
{
    if (!dispatch_gateway().snapshot().is_map_updated)
    {
        std::cout << "[dispatch] 调度地图未变化\n";
        return true;
    }
    const std::filesystem::path &dir = kDispatchMapDir;
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec))
    {
        std::cerr << "[dispatch] 调度已下发新地图，目录不存在: " << dir << "\n";
        board_error("地图更新失败：调度地图目录不存在");
        return false;
    }
    std::filesystem::path chosen;
    for (const std::filesystem::directory_entry &ent : std::filesystem::directory_iterator(dir, ec))
    {
        if (ec || !ent.is_regular_file())
            continue;
        chosen = ent.path();
        break;
    }
    if (chosen.empty())
    {
        std::cerr << "[dispatch] 调度已下发新地图，目录里没有文件\n";
        board_error("地图更新失败：调度地图目录里没有文件");
        return false;
    }
    std::ifstream in(chosen);
    std::ostringstream body;
    body << in.rdbuf();
    if (!in || body.str().empty())
    {
        std::cerr << "[dispatch] 读地图失败 " << chosen << "\n";
        board_error("地图更新失败：读不到调度下发的地图");
        return false;
    }
    if (!ensure_chassis())
        return false;
    const int rc = chassis_seer().UploadAndLoadMap(body.str());
    if (rc != 0)
    {
        std::cerr << "[dispatch] 上传并载入地图失败 code=" << rc << " "
                  << chassis_seer().LastMessage() << "\n";
        board_error("地图更新失败：上传并载入底盘失败 code=" + std::to_string(rc));
        return false;
    }
    std::cout << "[dispatch] 已载入调度地图 " << chosen.filename().string() << "\n";
    board_note("info", "地图更新：已载入 " + chosen.filename().string());
    return true;
}
}

bool read_pose_before_dispatch(double &x, double &y, double &angle)
{
    if (!ensure_chassis())
        return false;
    std::string station;
    std::string last;
    double confidence = 0;
    int loc_state = 0;
    bool forbidden = false;
    const int rc = chassis_seer().GetLocation(
        x, y, angle, station, last, confidence, loc_state, forbidden);
    if (rc != 0 || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(angle))
    {
        std::cerr << "[dispatch] 读底盘当前坐标失败 code=" << rc << " "
                  << chassis_seer().LastMessage() << "\n";
        board_error("读底盘当前坐标失败，不连接调度");
        return false;
    }
    std::ostringstream line;
    line << std::fixed << std::setprecision(4)
         << "调度前坐标 x=" << x << " y=" << y << " angle=" << angle;
    if (!station.empty())
        line << " 站点=" << station;
    std::cout << "[dispatch] 连接调度前记录当前地图位姿 " << line.str() << "\n";
    board_note("info", line.str());
    return true;
}

namespace {
void report_dispatch_map_fault(DispatchClient &client, const char *what)
{
    std::cerr << "[dispatch] " << what << "，向调度上报 1401，等待处理\n";
    board_error(std::string(what) + "，上报 1401，等待调度处理");
    const bool resolved = client.report_exception(dispatch_gateway().exception_code(DispatchFault::InitFailed));
    const std::string done = resolved ? "1401 调度已解决，本次仍不进入调度"
                                      : "1401 调度未解决，不进入调度";
    std::cout << "[dispatch] " << done << "\n";
    board_note(resolved ? "info" : "warn", done);
}
}

bool finish_dispatch_map(DispatchClient &client, double x, double y, double angle)
{
    if (!dispatch_gateway().snapshot().is_map_updated)
    {
        std::cout << "[dispatch] 地图已是最新，本次不重定位\n";
        board_note("info", "地图检查：已是最新，本次不重定位");
        return true;
    }
    std::cout << "[dispatch] 地图检查：调度下发了新地图，开始载入\n";
    board_note("info", "地图检查：调度下发了新地图，开始载入");
    if (!load_dispatch_map_if_updated())
    {
        report_dispatch_map_fault(client, "地图更新失败");
        return false;
    }
    std::ostringstream pose;
    pose << std::fixed << std::setprecision(4)
         << "重定位：用更新前坐标 x=" << x << " y=" << y << " angle=" << angle;
    std::cout << "[dispatch] " << pose.str() << "\n";
    board_note("info", pose.str());
    const int rc = chassis_seer().DoRelocate(x, y, angle);
    if (rc != 0)
    {
        std::cerr << "[dispatch] 新地图重定位失败 code=" << rc << " "
                  << chassis_seer().LastMessage() << "，不进入调度\n";
        report_dispatch_map_fault(client, "重定位失败");
        return false;
    }
    dispatch_gateway().mark_map_current();
    std::cout << "[dispatch] 新地图重定位成功，进入调度\n";
    board_note("info", "重定位成功，进入调度");
    return true;
}
}
