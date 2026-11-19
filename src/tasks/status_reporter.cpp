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
std::atomic<bool> reporter_stopping{false};
std::mutex reporter_mutex;
std::thread reporter_thread;
}

void publish_dispatch_pose(double x,double y,double angle,const std::string &station) {dispatch_gateway().update_pose(x,y,angle,station);}

void publish_dispatch_battery(double level)
{
    if (!std::isfinite(level))
        return;
    double pct = level;
    if (pct <= 1.5)
        pct *= 100.0;
    pct = std::clamp(pct, 0.0, 100.0);
    dispatch_gateway().update_battery(static_cast<int>(std::lround(pct)));
}

namespace {
void chassis_status_loop()
{
    while (!reporter_stopping.load())
    {
        if (ensure_chassis())
        {
            Chassis &robot = chassis_seer();
            double x = 0, y = 0, angle = 0, confidence = 0;
            std::string cur, last;
            int loc_state = 0;
            bool in_forbidden = false;
            if (robot.GetLocation(x, y, angle, cur, last, confidence, loc_state, in_forbidden) == 0)
                publish_dispatch_pose(x, y, angle, cur);
            double level = 0, voltage = 0, current = 0, temp = 0;
            bool charging = false;
            int cycle = 0;
            if (robot.GetBattery(level, charging, voltage, current, temp, cycle) == 0)
                publish_dispatch_battery(level);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
}

void start_chassis_status_reporter()
{
    std::lock_guard<std::mutex> lock(reporter_mutex);
    if(reporter_thread.joinable()) return;
    reporter_stopping=false;
    reporter_thread=std::thread(chassis_status_loop);
    std::cout << "[orch] 底盘状态上报已启动，2Hz 写入位置、电量和当前站名\n";
}
}

namespace orch {
void stop_chassis_status_reporter() {
    std::lock_guard<std::mutex> lock(reporter_mutex);
    reporter_stopping=true;
    if(reporter_thread.joinable()) reporter_thread.join();
}
}
