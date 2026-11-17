#include "internal/runtime.h"
#include "t170/skills/conveyor.h"

#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "Ti5_Arm.h"
#include "t170/common/units.h"
#include "t170/motion/cartesian_tracking.h"
#include "t170/motion/kinematics_adapter.h"
#include "t170/motion/retreat.h"
#include "t170/motion/servo_handoff.h"
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
int waist_rotate_deg(double yaw_deg, double speed_deg_s)
{
    double angles[1] = {yaw_deg};
    uint32_t can_id[] = {1};
    return smooth_motor_move_deg(waist_id, 1, can_id, angles, speed_deg_s);
}

int conveyor_waist_and_head(const ConveyorStationConfig &c, const char *tag)
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return -1;
    if (hardware_abort_requested())
        return -4;
    const char *name = (tag && tag[0]) ? tag : "conveyor";
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] " << name << " 腰/头 x/z=(" << c.waist_x << "," << c.waist_z
              << ") head=" << std::setprecision(1)
              << c.head_yaw_deg << "/" << c.head_pitch_deg << "/" << c.head_roll_deg
              << " deg\n";
    auto waist_fail = []() { return hardware_abort_requested() ? -4 : -5; };
    if (!move_box::sync_waist_layer3_from_encoders(
            *g_hw->waist, *g_hw->layer3, "拍照腰前"))
        return waist_fail();
    const double x_now = (*g_hw->layer3)(0);
    const double z_now = (*g_hw->layer3)(2);
    const bool far_forward = x_now > c.waist_x + 0.02;
    auto tracked = [&]() {
        const double dx = std::abs((*g_hw->layer3)(0) - c.waist_x);
        const double dz = std::abs((*g_hw->layer3)(2) - c.waist_z);
        if (dx <= 0.02 && dz <= 0.02)
            return true;
        std::cerr << std::fixed << std::setprecision(4)
                  << "[orch] " << name << " 腰未跟上 当前 x=" << (*g_hw->layer3)(0)
                  << " z=" << (*g_hw->layer3)(2)
                  << " 目标 x=" << c.waist_x << " z=" << c.waist_z
                  << " （|dx|=" << dx << " |dz|=" << dz << "）\n";
        return false;
    };
    auto move_x = [&](double x) {
        return move_box::waist_layer3_move_x_only(*g_hw->waist, *g_hw->layer3, x, true, false);
    };
    auto move_z = [&](double z) {
        return move_box::waist_layer3_move_z_only(*g_hw->waist, *g_hw->layer3, z, true, false);
    };
    if (far_forward)
    {
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] " << name << " 远排下蹲 x=" << x_now
                  << " z=" << z_now << "，先收到拍照 x=" << c.waist_x
                  << " 再上升\n";
        if (!move_x(c.waist_x))
        {
            std::cerr << "[orch] " << name << " 腰平移 X 失败\n";
            return waist_fail();
        }
        if (hardware_abort_requested())
            return -4;
        if (!move_z(c.waist_z))
        {
            std::cerr << "[orch] " << name << " 腰升降 Z 失败\n";
            return waist_fail();
        }
        if (hardware_abort_requested())
            return -4;
        tracked();
    }
    else
    {
        std::cout << "[orch] " << name << " 先升到拍照高度，再平移到前伸\n";
        if (!move_z(c.waist_z))
        {
            std::cerr << "[orch] " << name << " 腰升降 Z 失败\n";
            return waist_fail();
        }
        if (hardware_abort_requested())
            return -4;
        if (!move_x(c.waist_x))
        {
            std::cerr << "[orch] " << name << " 腰平移 X 失败\n";
            return waist_fail();
        }
        if (hardware_abort_requested())
            return -4;
        tracked();
    }
    move_box::read_arrived_pose(*g_hw->waist, *g_hw->layer3);
    move_box::waist_layer3_set_synced(true);
    const int head_rc = move_head_to_rpy_deg(
        c.head_yaw_deg, c.head_pitch_deg, c.head_roll_deg, true);
    if (head_rc != 0)
        return head_rc;
    if (hardware_abort_requested())
        return -4;
    hardware_abort_sleep_ms(300);
    return hardware_abort_requested() ? -4 : 0;
}

void log_gripper_qs(const char *tag)
{
    if (g_hw == nullptr || g_hw->g == nullptr)
        return;
    const auto r = g_hw->g->feedback(gripper::Side::Right);
    const auto l = g_hw->g->feedback(gripper::Side::Left);
    auto one = [](const char *hand, const gripper::SideFeedback &fb) {
        std::cout << ' ' << hand << "q=" << fb.position_rad << " cmd=" << fb.command_rad
                  << (fb.have_feedback ? "" : " 无反馈")
                  << " io=" << (fb.io_ok ? "ok" : "fail") << " err=" << fb.merror;
        if (fb.merror & 512u)
            std::cout << "(超时保护)";
    };
    std::cout << std::fixed << std::setprecision(2) << "[gripper] " << tag;
    one("右", r);
    one("左", l);
    std::cout << "  (全开≈5.5 全合≈0)\n";
}

int home_waist_upright()
{
    if (hardware_abort_requested())
        return -4;
    waist_rotate_deg(0.0, 30.0);
    if (hardware_abort_requested())
        return -4;
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return 0;
    const double x_home = g_move_cfg.waist.layer3_home(0);
    const double z_home = g_move_cfg.waist.layer3_home(2);
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] 收手同时腰先回 x=" << x_home << "，再升到 z=" << z_home << "\n";
    if (!move_box::ensure_waist_layer3_x_home(*g_hw->waist, *g_hw->layer3))
        return hardware_abort_requested() ? -4 : -5;
    if (hardware_abort_requested())
        return -4;
    if (!move_box::waist_layer3_move_z_only(*g_hw->waist, *g_hw->layer3, z_home, false))
        return hardware_abort_requested() ? -4 : -5;
    move_box::waist_layer3_set_synced(true);
    return 0;
}

int go_home(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    TimingSpan span("motion_home");
    auto done = [&](int code) {
        span.set_ok(code == 0);
        return code;
    };
    std::cout << "[orch] home：先头部标定姿态，再安全抬升走到 yaml home_tcp，再腰 yaw=0，先回 x 再竖直升到 layer3_home\n";
    const int head_rc = enable_head_calib_pose();
    if (head_rc != 0)
    {
        std::cerr << "[orch] 头部使能/到位失败，不继续 home\n";
        return done(head_rc);
    }
    if (hardware_abort_requested())
        return done(-4);
    if (!move_box::move_arms_to_home(arm_r, arm_l))
    {
        std::cerr << "[orch] 双臂安全退出或回 home_tcp 失败，不转腰\n";
        return done(hardware_abort_requested() ? -4 : -5);
    }
    if (hardware_abort_requested())
        return done(-4);
    waist_rotate_deg(0.0, 30.0);
    if (hardware_abort_requested())
        return done(-4);
    if (g_hw != nullptr && g_hw->waist != nullptr && g_hw->layer3 != nullptr)
    {
        const double x_home = g_move_cfg.waist.layer3_home(0);
        const double z_home = g_move_cfg.waist.layer3_home(2);
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] home 腰先平移回 x=" << x_home << "，再竖直到 z=" << z_home << "\n";
        if (!move_box::ensure_waist_layer3_x_home(*g_hw->waist, *g_hw->layer3))
        {
            std::cerr << "[orch] 腰平移回初始 x 失败，不上升\n";
            return done(hardware_abort_requested() ? -4 : -5);
        }
        if (hardware_abort_requested())
            return done(-4);
        if (!move_box::waist_layer3_move_z_only(*g_hw->waist, *g_hw->layer3, z_home, false))
        {
            std::cerr << "[orch] 腰竖直升降失败\n";
            return done(hardware_abort_requested() ? -4 : -5);
        }
        move_box::waist_layer3_set_synced(true);
    }
    return done(0);
}

int go_home_arms_keep_waist_z(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    std::cout << "[orch] 充完电收手：头标定 + 双臂回 home_tcp，腰高度保持不动\n";
    const int head_rc = enable_head_calib_pose();
    if (head_rc != 0)
    {
        std::cerr << "[orch] 头部使能/到位失败，不继续收手\n";
        return head_rc;
    }
    if (hardware_abort_requested())
        return -4;
    if (!move_box::move_arms_to_home(arm_r, arm_l))
    {
        std::cerr << "[orch] 双臂回 home_tcp 失败（腰未动）\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (g_hw != nullptr && g_hw->layer3 != nullptr)
    {
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] 腰保持当前高度 z=" << (*g_hw->layer3)(2)
                  << " x=" << (*g_hw->layer3)(0) << "，不升不降\n";
    }
    return hardware_abort_requested() ? -4 : 0;
}

int go_grasp_ready(Robot_Arm &arm_r, Robot_Arm &arm_l, int row_group)
{
    if (row_group != 6 && (row_group < 1 || row_group > 3))
    {
        std::cerr << "[orch] grasp_ready 组号必须是 1/2/3/6，收到 " << row_group << "\n";
        return -1;
    }
    const GraspRpyDeg &rpy_l = grasp_rpy_deg_for_ready(row_group, false);
    const GraspRpyDeg &rpy_r = grasp_rpy_deg_for_ready(row_group, true);
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] grasp_ready" << row_group
              << "：头/腰同原 ready"
              << " 左手 RPY=(" << rpy_l.rx << "," << rpy_l.ry << "," << rpy_l.rz
              << ") 右手 RPY=(" << rpy_r.rx << "," << rpy_r.ry << "," << rpy_r.rz
              << ")deg （" << ready_rows_label(row_group) << "）\n";
    const int head_rc = enable_head_calib_pose();
    if (head_rc != 0)
    {
        std::cerr << "[orch] 头部使能/到位失败，不继续 grasp_ready\n";
        return head_rc;
    }
    if (hardware_abort_requested())
        return -4;
    if (!move_box::move_arms_to_standby(arm_r, arm_l, row_group))
    {
        std::cerr << "[orch] 双臂回抓取准备姿态失败，不转腰\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (hardware_abort_requested())
        return -4;
    waist_rotate_deg(0.0, 30.0);
    if (hardware_abort_requested())
        return -4;
    if (g_hw != nullptr && g_hw->waist != nullptr && g_hw->layer3 != nullptr)
    {
        const auto &w = g_move_cfg.waist;
        const double x_ready = waist_ready_x();
        const double z_ready = waist_ready_z();
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] grasp_ready 腰到起始 x=" << x_ready
                  << " z=" << z_ready << "（向前 "
                  << w.ready_forward_m << " m）\n";
        if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
        {
            std::cerr << "[orch] 腰到 ready 起始位失败\n";
            return hardware_abort_requested() ? -4 : -5;
        }
        move_box::waist_layer3_set_synced(true);
    }
    return 0;
}

int go_tray2_ready(Robot_Arm &arm_r, Robot_Arm &arm_l, int row_group)
{
    if (row_group != 6 && (row_group < 1 || row_group > 3))
    {
        std::cerr << "[orch] tray2ready 组号必须是 1/2/3/6，收到 " << row_group << "\n";
        return -1;
    }
    const GraspRpyDeg &rpy_l = tray2_place_rpy_deg_for_ready(row_group, false);
    const GraspRpyDeg &rpy_r = tray2_place_rpy_deg_for_ready(row_group, true);
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] tray2ready" << row_group
              << "：头/腰同 grasp ready，手臂用 tray2_place 独立 RPY"
              << " 左手 RPY=(" << rpy_l.rx << "," << rpy_l.ry << "," << rpy_l.rz
              << ") 右手 RPY=(" << rpy_r.rx << "," << rpy_r.ry << "," << rpy_r.rz
              << ")deg （" << ready_rows_label(row_group) << "）\n";
    const int head_rc = enable_head_calib_pose();
    if (head_rc != 0)
    {
        std::cerr << "[orch] 头部使能/到位失败，不继续 tray2ready\n";
        return head_rc;
    }
    if (hardware_abort_requested())
        return -4;
    if (!move_box::move_arms_to_standby(arm_r, arm_l, row_group, true))
    {
        std::cerr << "[orch] 双臂回料盘2放置准备姿态失败，不转腰\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (hardware_abort_requested())
        return -4;
    waist_rotate_deg(0.0, 30.0);
    if (hardware_abort_requested())
        return -4;
    if (g_hw != nullptr && g_hw->waist != nullptr && g_hw->layer3 != nullptr)
    {
        const auto &w = g_move_cfg.waist;
        const double x_ready = waist_ready_x();
        const double z_ready = waist_ready_z();
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] tray2ready 腰到起始 x=" << x_ready
                  << " z=" << z_ready << "（向前 "
                  << w.ready_forward_m << " m）\n";
        if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
        {
            std::cerr << "[orch] 腰到 ready 起始位失败\n";
            return hardware_abort_requested() ? -4 : -5;
        }
        move_box::waist_layer3_set_synced(true);
    }
    return 0;
}

int go_belt_ready(
    Robot_Arm &arm_r, Robot_Arm &arm_l, const ConveyorStationConfig &c, const char *tag)
{
    const char *name = (tag && tag[0]) ? tag : "belt";
    std::cout << "[orch] " << name << "_ready：腰/头到位，手臂到准备 tcp\n";
    const int wh = conveyor_waist_and_head(c, name);
    if (wh != 0)
        return wh;
    if (!move_box::move_arms_to_belt_ready(arm_r, arm_l, c))
    {
        std::cerr << "[orch] " << name << "_ready 手臂到准备姿态失败\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    return 0;
}

int go_belt_ready(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    return go_belt_ready(arm_r, arm_l, g_move_cfg.conveyor, "belt");
}

int go_belt_place(
    Robot_Arm &arm_r, Robot_Arm &arm_l, const ConveyorStationConfig &c, const char *tag)
{
    const char *name = (tag && tag[0]) ? tag : "belt";
    std::cout << "[orch] " << name << "_place：腰/头到位，手臂到 place_tcp（放置末端）\n";
    const int wh = conveyor_waist_and_head(c, name);
    if (wh != 0)
        return wh;
    if (!move_box::move_arms_to_belt_place(arm_r, arm_l, c))
    {
        std::cerr << "[orch] " << name << "_place 手臂到放置姿态失败\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    return 0;
}

int go_belt_place(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    return go_belt_place(arm_r, arm_l, g_move_cfg.conveyor, "belt");
}

int go_belt_grasp_rpy(
    Robot_Arm &arm_r, Robot_Arm &arm_l, const ConveyorStationConfig &c, const char *tag)
{
    const char *name = (tag && tag[0]) ? tag : "belt";
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] " << name << "_grasp_rpy：腰/头同准备位，手臂 tcp XYZ + grasp_rpy_deg"
              << " 左=(" << c.grasp_rpy_left.rx << "," << c.grasp_rpy_left.ry << ","
              << c.grasp_rpy_left.rz << ") 右=(" << c.grasp_rpy_right.rx << ","
              << c.grasp_rpy_right.ry << "," << c.grasp_rpy_right.rz
              << ")deg，不拍照不夹取\n";
    const int wh = conveyor_waist_and_head(c, name);
    if (wh != 0)
        return wh;
    if (!move_box::move_arms_to_belt_grasp_rpy(arm_r, arm_l, c))
    {
        std::cerr << "[orch] " << name << "_grasp_rpy 手臂到位失败\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    return 0;
}

int go_belt_grasp_rpy(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    return go_belt_grasp_rpy(arm_r, arm_l, g_move_cfg.conveyor, "belt");
}

namespace {
bool tcp_near_ready(const Eigen::Matrix<double, 1, 6> &cur, const Eigen::Matrix<double, 1, 6> &ready)
{
    constexpr double kXyzM = 0.012;
    constexpr double kRpyRad = 5.0 * M_PI / 180.0;
    if ((cur.head<3>() - ready.head<3>()).norm() > kXyzM)
        return false;
    return (cur.tail<3>() - ready.tail<3>()).cwiseAbs().maxCoeff() <= kRpyRad;
}
}

namespace {
bool arms_at_belt_ready(const ConveyorStationConfig &c)
{
    if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
        return false;
    const bool right_ok = right_arm_motors_locked() ||
                          tcp_near_ready(arm_get_tcp_pos(*g_hw->arm_r), c.tcp.right);
    const bool left_ok = tcp_near_ready(arm_get_tcp_pos(*g_hw->arm_l), c.tcp.left);
    return right_ok && left_ok;
}
}

namespace {
bool chassis_already_at(const std::string &station)
{
    return g_hw != nullptr && g_hw->last_chassis_ok &&
           station_name_matches(g_hw->last_chassis_station, station);
}
}

bool belt_ready_to_pick(const std::string &station, const ConveyorStationConfig &c, const char *what)
{
    if (!chassis_already_at(station) || !arms_at_belt_ready(c))
        return false;
    std::cout << "[dispatch] " << what << " 已在 " << station
              << " 且手臂在准备位，不回 home，直接申请并抓\n";
    return true;
}

int drive_belt_ready(const std::string &station, const ConveyorStationConfig &c, const char *tag)
{
    std::cout << "[dispatch] 去 " << station << "，路上同时摆到拍照准备，不先回 home\n";
    int ready_rc = 0;
    int ch_rc = 0;
    {
        ChassisTravel travel(station);
        ready_rc = go_belt_ready(*g_hw->arm_r, *g_hw->arm_l, c, tag);
        ch_rc = travel.join();
    }
    if (ready_rc != 0)
    {
        // 充电后常停在挥手下垂等非常规姿态，直接摆传送带准备会逆解失败；先 home 再摆一次。
        std::cerr << "[dispatch] " << tag << " 路上摆准备失败 code=" << ready_rc
                  << "，先回 home 再摆准备（底盘已到 " << station << "）\n";
        board_activity("准备姿态失败，回 home 后再摆传送带准备");
        const int home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
        if (home_rc != 0)
            return home_rc;
        if (hardware_abort_requested())
            return -4;
        ready_rc = go_belt_ready(*g_hw->arm_r, *g_hw->arm_l, c, tag);
        if (ready_rc != 0)
            return ready_rc;
        return ch_rc != 0 ? ch_rc : 0;
    }
    return ch_rc;
}
}
