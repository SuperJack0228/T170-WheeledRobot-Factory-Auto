#include "internal/runtime.h"
#include "t170/skills/conveyor.h"

#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "t170/motion/arm.h"
#include "t170/common/units.h"
#include "t170/vision/wrist_observer.h"
#include "t170/motion/servo.h"
#include "t170/vision/head_transform.h"
#include "t170/hardware/head.h"
#include "t170/skills/legacy/profiles.h"
#include "t170/skills/legacy/runtime.h"
#include "t170/vision/python_bridge.h"
#include "t170/vision/dl.h"
#include "t170/vision/debug.h"
#include "t170/hardware/device_bus.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <cerrno>
#include <csignal>
#include <mutex>
#include "t170/motion/retreat_gate.h"
#include <sys/select.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using std::vector;
using std::string;
using std::thread;


namespace orch {
int chassis_goto_belt2_and_ready(Arm &arm_r, Arm &arm_l)
{
    const std::string &st = g_move_cfg.chassis.out_station;
    std::cout << "[orch] belt2：先到 " << st << "\n";
    const int ch_rc = chassis_goto_station(st);
    if (ch_rc != 0)
        return ch_rc;
    if (hardware_abort_requested())
        return -4;
    std::cout << "[orch] belt2 底盘已到 " << st << "，停稳 0.5s 再进准备\n";
    hardware_abort_sleep_ms(500);
    if (hardware_abort_requested())
        return -4;
    return go_belt_ready(arm_r, arm_l, g_move_cfg.conveyor2, "belt2");
}

int go_belt2_ready(Arm &arm_r, Arm &arm_l)
{
    return chassis_goto_belt2_and_ready(arm_r, arm_l);
}

int go_belt2_grasp_rpy(Arm &arm_r, Arm &arm_l)
{
    return go_belt_grasp_rpy(arm_r, arm_l, g_move_cfg.conveyor2, "belt2");
}

int go_belt2()
{
    const std::string &st = g_move_cfg.chassis.out_station;
    std::cout << "[orch] belt2：先到 " << st << " 再放置\n";
    const int ch_rc = chassis_goto_station(st);
    if (ch_rc != 0)
        return ch_rc;
    if (hardware_abort_requested())
        return -4;
    std::cout << "[orch] belt2 底盘已到 " << st << "，到站后再等 "
              << g_move_cfg.conveyor2.photo_after_arrival_sec << "s 拍照\n";
    return go_belt_station(g_move_cfg.conveyor2, "belt2");
}



int go_belt2_grasp()
{
    if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
    {
        std::cerr << "[orch] belt2_grasp 失败：手臂未初始化\n";
        return -1;
    }
    const int ready_rc = chassis_goto_belt2_and_ready(*g_hw->arm_r, *g_hw->arm_l);
    if (ready_rc != 0)
        return ready_rc;
    if (hardware_abort_requested())
        return -4;
    return go_belt_grasp(
        true, g_move_cfg.conveyor2, g_move_cfg.chassis.out_station, "belt2_grasp");
}
}
