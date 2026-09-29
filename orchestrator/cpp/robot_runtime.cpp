#include "robot_runtime.h"

#include "function.h"
#include "head_cam2robot.h"
#include "head_control.h"
#include "move_box_config.h"
#include "move_box_runtime.h"
#include "seg_pose_bridge.h"
#include "Ti5_socketcan.h"
#include "Ti5_Seer.hpp"
#include "robot_client.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace orch
{

// 调度库不再提供状态名，协议字符串由本程序自己发送和比对。
namespace dispatch_msg
{
constexpr const char *TASK_LOAD = "TASK_LOAD";
constexpr const char *TASK_TRANSFER = "TASK_TRANSFER";
constexpr const char *TASK_UNLOAD = "TASK_UNLOAD";
constexpr const char *APPLY_PLACE_RAW = "APPLY_PLACE_RAW";
constexpr const char *APPLY_PLACE_HALF = "APPLY_PLACE_HALF";
constexpr const char *APPLY_PICK_HALF = "APPLY_PICK_HALF";
constexpr const char *APPLY_PICK_WELL = "APPLY_PICK_WELL";
constexpr const char *PLACE = "PLACE";
constexpr const char *PICK_HALF = "PICK_HALF";
constexpr const char *PICK_WELL = "PICK_WELL";
constexpr const char *PLACE_DONE = "PLACE_DONE";
constexpr const char *PICK_UP_DONE = "PICK_UP_DONE";
constexpr const char *DONE = "DONE";
constexpr const char *STANDBY = "STANDBY";
constexpr const char *WORKING = "WORKING";
constexpr const char *ERR_MOTOR_LOST = "ERR_MOTOR_LOST";
constexpr const char *ERR_GRIPPER_LOST = "ERR_GRIPPER_LOST";
constexpr const char *ERR_CAMERA = "ERR_CAMERA";
constexpr const char *ERR_MOTOR_DISABLE = "ERR_MOTOR_DISABLE";
constexpr const char *ERR_AGV_OBSTACLE = "ERR_AGV_OBSTACLE";
constexpr const char *RAW_MATERIAL_EMPTY = "RAW_MATERIAL_EMPTY";
constexpr const char *WELL_DONE_MATERIAL_FULL = "WELL_DONE_MATERIAL_FULL";
constexpr const char *kDefaultHost = "192.168.122.120";
constexpr int kDefaultPort = 8080;
} // namespace dispatch_msg

HwSession *g_hw = nullptr;
/** 本次上料开抓前，工作区里的毛坯数。-1 表示没数清。 */
int g_load_parts_seen = -1;
/** 抓取最后一次收手已经同时把腰站起、底盘开到传送带。调用方不必再排队走。 */
bool g_grasp_departed = false;
/** 上次下料放完后，工作区还剩的空孔。未知时下次仍抓两只手。 */
bool g_unload_empty_known = false;
/** 调度下料已在 AP9 放完，本轮完成后停在原地等任务，不回 AP7。 */
bool g_dispatch_stay_at_tray2 = false;
/** 腰和头已经在该站的传送带拍照位，紧接着的放置不必再摆一次。 */
std::string g_photo_ready_station;
int g_unload_empty_right = 0;
int g_unload_empty_left = 0;

namespace
{

int waist_rotate_deg(double yaw_deg, double speed_deg_s)
{
    double angles[1] = {yaw_deg};
    uint32_t can_id[] = {1};
    return smooth_motor_move_deg(waist_id, 1, can_id, angles, speed_deg_s);
}

/** 码平面 Z 改成固定值。XY 和板轴不变；抓取点只平移同样的 ΔZ，offset 仍有效。 */
void apply_fixed_belt_origin_z(BeltDetectResult &det, const ConveyorStationConfig &c, const char *tag)
{
    if (!c.use_fixed_origin_z || !det.in_robot || !std::isfinite(det.origin_z))
        return;
    const double cam_z = det.origin_z;
    const double dz = c.fixed_origin_z_m - cam_z;
    det.origin_z = c.fixed_origin_z_m;
    if (det.have_grasp_l && std::isfinite(det.grasp_lz))
        det.grasp_lz += dz;
    if (det.have_grasp_r && std::isfinite(det.grasp_rz))
        det.grasp_rz += dz;
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] " << tag << " 码平面 Z 不用相机：相机=" << cam_z
              << "m 固定=" << c.fixed_origin_z_m
              << "m（腰高 " << c.waist_z << "m，基座到传送带 "
              << std::abs(c.fixed_origin_z_m) << "m）ΔZ=" << dz << "m\n";
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
        return move_box::waist_layer3_move_x_only(*g_hw->waist, *g_hw->layer3, x, true);
    };
    auto move_z = [&](double z) {
        return move_box::waist_layer3_move_z_only(*g_hw->waist, *g_hw->layer3, z, true);
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
        if (!tracked())
        {
            const double x_back = waist_ready_x();
            std::cout << std::fixed << std::setprecision(4)
                      << "[orch] " << name << " 上升后未跟上，先收回到 x="
                      << x_back << " 再站直，然后伸到拍照 x\n";
            if (!move_x(x_back) || !move_z(c.waist_z) || !move_x(c.waist_x) || !tracked())
            {
                std::cerr << "[orch] " << name << " 腰收回重站仍未到位\n";
                return waist_fail();
            }
        }
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
        if (!tracked())
            return waist_fail();
    }
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

const char *belt_yolo_class_zh(int class_id)
{
    switch (class_id)
    {
    case 0:
        return "毛坯";
    case 1:
        return "半加工料";
    case 2:
        return "精加工料";
    case 3:
        return "空孔";
    default:
        return "未知";
    }
}

/** 同一张头相机帧上跑 YOLO，只判定类别，不用其 XYZ。不另取帧、不加等待。 */
bool belt_grasp_yolo_class_ok(
    SegPoseBridge &bridge,
    const CameraFrameData &frame,
    int require_id,
    std::string &err)
{
    if (require_id < 0)
        return true;
    const PoseRunResult yolo = bridge.run(frame, -1, false, CameraSlot::Head, -1);
    if (!yolo.ok)
    {
        err = "YOLO失败: " + yolo.message;
        return false;
    }
    bool found = false;
    int best_id = -1;
    float best_conf = -1.f;
    for (const auto &t : yolo.targets)
    {
        std::cout << std::fixed << std::setprecision(3)
                  << "[orch] belt_grasp YOLO " << belt_yolo_class_zh(t.class_id)
                  << " class=" << t.class_id << " conf=" << t.confidence << "\n";
        if (t.class_id == require_id)
            found = true;
        if (t.class_id >= 0 && t.class_id <= 2 && t.confidence > best_conf)
        {
            best_conf = t.confidence;
            best_id = t.class_id;
        }
    }
    if (found)
        return true;
    err = std::string("YOLO未看到") + belt_yolo_class_zh(require_id);
    if (best_id < 0)
        err += "（画面无物料）";
    else
        err += std::string("（最高置信是") + belt_yolo_class_zh(best_id) + "）";
    return false;
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

bool station_name_matches(const std::string &got, const std::string &want)
{
    if (want.empty())
        return false;
    auto lower = [](std::string s) {
        for (char &c : s)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    };
    const std::string a = lower(got);
    const std::string b = lower(want);
    if (a == b)
        return true;
    return a.size() > b.size() ? a.find(b) != std::string::npos : b.find(a) != std::string::npos && !a.empty();
}

std::string g_dispatch_nav_fault;
std::chrono::steady_clock::time_point g_station_arrived_at{};
bool g_station_arrived = false;

void note_station_arrived()
{
    g_station_arrived_at = std::chrono::steady_clock::now();
    g_station_arrived = true;
}

int chassis_goto_station(
    const std::string &station,
    bool watch_obstacle = false,
    const std::function<void()> &on_station_name = {})
{
    if (g_hw == nullptr)
        return -1;
    g_hw->last_chassis_ok = false;
    g_hw->last_chassis_station = station;
    g_hw->last_chassis_message.clear();

    const auto &ch = g_move_cfg.chassis;
    if (ch.host.empty() || station.empty())
    {
        g_hw->last_chassis_message = "chassis.host 或站点名为空";
        std::cerr << "[orch] " << g_hw->last_chassis_message << "\n";
        return -1;
    }

    auto aborted = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };

    Seer robot(ch.host);
    std::cout << "[orch] 底盘连接 " << ch.host << "，去 " << station << "\n";
    if (!robot.Connect())
    {
        g_hw->last_chassis_message = robot.LastMessage().empty() ? "底盘连接失败" : robot.LastMessage();
        std::cerr << "[orch] 底盘连接失败: " << g_hw->last_chassis_message << "\n";
        return -6;
    }

    double x = 0, y = 0, angle = 0, confidence = 0;
    std::string cur, last;
    int loc_state = 0;
    bool in_forbidden = false;
    int loc_rc = robot.GetLocation(x, y, angle, cur, last, confidence, loc_state, in_forbidden);
    if (loc_rc == 0 && station_name_matches(cur, station))
    {
        std::cout << "[orch] 底盘已在 " << cur << "，不再发导航\n";
        note_station_arrived();
        g_hw->last_chassis_ok = true;
        g_hw->last_chassis_station = cur;
        g_hw->last_chassis_message = "已在目标站";
        robot.Disconnect();
        return 0;
    }

    const int go_rc = robot.GoToStation(station);
    if (go_rc != 0)
    {
        g_hw->last_chassis_message = robot.LastMessage().empty()
                                         ? ("GoToStation 失败 code=" + std::to_string(go_rc))
                                         : robot.LastMessage();
        std::cerr << "[orch] 底盘去 " << station << " 下发失败 code=" << go_rc
                  << " " << g_hw->last_chassis_message << "\n";
        robot.Disconnect();
        return go_rc < 0 ? go_rc : -6;
    }
    std::cout << "[orch] 已通知底盘去 " << station << "，等待到位\n";

    const int timeout_ms = std::max(1000, ch.nav_timeout_ms);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    bool logged_name_early = false;
    bool upper_started = false;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (aborted())
        {
            std::cerr << "[orch] 等待底盘到位时中止，取消导航\n";
            robot.CancelNav();
            g_hw->last_chassis_message = "中止";
            robot.Disconnect();
            return -4;
        }

        loc_rc = robot.GetLocation(x, y, angle, cur, last, confidence, loc_state, in_forbidden);

        int task_status = 0, task_type = 0;
        std::string target_id;
        double tx = 0, ty = 0, ta = 0, dist = 0;
        std::vector<std::string> finished, unfinished;
        const int nav_rc =
            robot.GetNavStatus(task_status, task_type, target_id, tx, ty, ta, finished, unfinished, dist);
        if (watch_obstacle)
        {
            bool blocked = false;
            bool slowed = false;
            double block_x = 0.0;
            double block_y = 0.0;
            std::vector<std::string> obstacles;
            if (robot.GetBlocked(blocked, slowed, block_x, block_y, obstacles) == 0 && blocked)
            {
                std::cerr << "[orch] 底盘遇到障碍，取消导航\n";
                robot.CancelNav();
                g_hw->last_chassis_message = "遇到障碍";
                g_dispatch_nav_fault = dispatch_msg::ERR_AGV_OBSTACLE;
                robot.Disconnect();
                return -11;
            }
        }

        if (nav_rc == 0)
        {
            if (task_status == 4 && station_name_matches(target_id, station))
            {
                std::cout << std::fixed << std::setprecision(3)
                          << "[orch] 底盘导航完成并停稳 target=" << target_id
                          << " x=" << x << " y=" << y << "\n";
                note_station_arrived();
                g_hw->last_chassis_ok = true;
                g_hw->last_chassis_station = target_id;
                g_hw->last_chassis_message = "导航完成";
                robot.Disconnect();
                return 0;
            }
            if (loc_rc == 0 && station_name_matches(cur, station) && task_status != 4)
            {
                if (!logged_name_early)
                {
                    logged_name_early = true;
                    std::cout << "[orch] 底盘站名已是 " << cur
                              << "，导航状态=" << task_status << "，尚未停稳，继续等\n";
                }
                if (!upper_started && on_station_name)
                {
                    upper_started = true;
                    std::cout << "[orch] 站名已变为 " << cur
                              << "，提前准备腰和手臂\n" << std::flush;
                    on_station_name();
                }
            }
            if (task_status == 5 || task_status == 6)
            {
                g_hw->last_chassis_message =
                    (task_status == 5 ? "导航失败" : "导航取消") + std::string(" target=") + target_id;
                std::cerr << "[orch] 底盘 " << g_hw->last_chassis_message << "\n";
                robot.Disconnect();
                return -6;
            }
        }
        hardware_abort_sleep_ms(300);
    }

    g_hw->last_chassis_message = "等待到位超时";
    std::cerr << "[orch] 底盘去 " << station << " 超时\n";
    robot.CancelNav();
    robot.Disconnect();
    return -9;
}

/** 底盘在自己的线程里赶路，上半身在当前线程同时动。返回底盘结果。 */
struct ChassisTravel
{
    std::thread th;
    std::atomic<int> rc{0};

    ChassisTravel(const std::string &station, bool watch_obstacle = false)
    {
        th = std::thread([this, station, watch_obstacle]() {
            try
            {
                rc = chassis_goto_station(station, watch_obstacle);
            }
            catch (const std::exception &ex)
            {
                std::cerr << "[orch] 底盘导航失败: " << ex.what() << "\n";
                rc = -6;
            }
        });
    }

    ChassisTravel(const ChassisTravel &) = delete;
    ChassisTravel &operator=(const ChassisTravel &) = delete;

    int join()
    {
        if (th.joinable())
            th.join();
        return rc.load();
    }

    ~ChassisTravel()
    {
        if (th.joinable())
            th.join();
    }
};

std::thread g_belt_depart_th;
std::atomic<int> g_belt_depart_rc{0};
bool g_belt_depart_started = false;

void start_belt_depart_after_right(const std::string &station)
{
    if (station.empty() || g_belt_depart_started)
        return;
    g_belt_depart_started = true;
    g_belt_depart_rc = 0;
    g_belt_depart_th = std::thread([station]() {
        std::cout << "[orch] 右手已抓到，2s 后底盘去 " << station
                  << "，不等收手到位\n"
                  << std::flush;
        hardware_abort_sleep_ms(2000);
        if (hardware_abort_requested())
        {
            g_belt_depart_rc = -4;
            return;
        }
        const bool with_tray_waist =
            station_name_matches(station, g_move_cfg.chassis.tray2_station) &&
            g_hw != nullptr && g_hw->waist != nullptr && g_hw->layer3 != nullptr;
        std::atomic<int> waist_rc{0};
        std::thread waist_th;
        if (with_tray_waist)
        {
            std::cout << "[orch] 底盘去 " << station
                      << " 的同时，腰开始到料盘放置高度\n"
                      << std::flush;
            waist_th = std::thread([&]() {
                try
                {
                    if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
                        waist_rc = -5;
                }
                catch (const std::exception &ex)
                {
                    std::cerr << "[orch] 去料盘时腰部失败: " << ex.what() << "\n";
                    waist_rc = -5;
                }
            });
        }
        std::cout << "[orch] 右手抓到已 2s，底盘出发去 " << station << "\n" << std::flush;
        try
        {
            g_belt_depart_rc = chassis_goto_station(station);
        }
        catch (const std::exception &ex)
        {
            std::cerr << "[orch] 底盘去 " << station << " 失败: " << ex.what() << "\n";
            g_belt_depart_rc = -6;
        }
        if (waist_th.joinable())
            waist_th.join();
        if (g_belt_depart_rc.load() == 0 && waist_rc.load() != 0)
            g_belt_depart_rc = waist_rc.load();
    });
}

int take_belt_depart()
{
    if (g_belt_depart_th.joinable())
        g_belt_depart_th.join();
    if (!g_belt_depart_started)
        return 0;
    const int rc = g_belt_depart_rc.load();
    g_belt_depart_started = false;
    return rc;
}

} // namespace

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
    std::cout << "[orch] home：先头部标定姿态，再安全抬升走到 yaml home_tcp，再腰 yaw=0，先回 x 再竖直升到 layer3_home\n";
    const int head_rc = enable_head_calib_pose();
    if (head_rc != 0)
    {
        std::cerr << "[orch] 头部使能/到位失败，不继续 home\n";
        return head_rc;
    }
    if (hardware_abort_requested())
        return -4;
    if (!move_box::move_arms_to_home(arm_r, arm_l))
    {
        std::cerr << "[orch] 双臂安全退出或回 home_tcp 失败，不转腰\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (hardware_abort_requested())
        return -4;
    waist_rotate_deg(0.0, 30.0);
    if (hardware_abort_requested())
        return -4;
    if (g_hw != nullptr && g_hw->waist != nullptr && g_hw->layer3 != nullptr)
    {
        const double x_home = g_move_cfg.waist.layer3_home(0);
        const double z_home = g_move_cfg.waist.layer3_home(2);
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] home 腰先平移回 x=" << x_home << "，再竖直到 z=" << z_home << "\n";
        if (!move_box::ensure_waist_layer3_x_home(*g_hw->waist, *g_hw->layer3))
        {
            std::cerr << "[orch] 腰平移回初始 x 失败，不上升\n";
            return hardware_abort_requested() ? -4 : -5;
        }
        if (hardware_abort_requested())
            return -4;
        if (!move_box::waist_layer3_move_z_only(*g_hw->waist, *g_hw->layer3, z_home, false))
        {
            std::cerr << "[orch] 腰竖直升降失败\n";
            return hardware_abort_requested() ? -4 : -5;
        }
        move_box::waist_layer3_set_synced(true);
    }
    return 0;
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

int go_waist_debug(int pose)
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[orch] waist 调试失败：腰未初始化\n";
        return -1;
    }
    if (pose != 1 && pose != 2)
    {
        std::cerr << "[orch] waist 调试 pose 必须是 1 或 2\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;

    if (g_hw != nullptr)
        g_hw->last_debug_photo.clear();

    const double x_home = g_move_cfg.waist.layer3_home(0);
    const double x_fwd = waist_far_row_x();
    const double x_target = (pose == 2) ? x_fwd : x_home;
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] waist" << pose << " 仅平移腰 X → " << x_target
              << " m（waist1 home=" << x_home
              << "  waist2 前伸=" << x_fwd
              << (pose == 2 ? "，随后低头截一帧" : "，不改 y/z、不动手臂")
              << "）\n";
    if (!move_box::waist_layer3_move_x_only(*g_hw->waist, *g_hw->layer3, x_target, true))
    {
        std::cerr << "[orch] waist" << pose << " 平移失败\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (hardware_abort_requested())
        return -4;
    move_box::waist_layer3_set_synced(true);
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] waist" << pose << " 到位 x=" << (*g_hw->layer3)(0)
              << " z=" << (*g_hw->layer3)(2) << " m\n";

    if (pose != 2)
        return 0;

    const auto &h = g_move_cfg.head;
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] waist2 头部低头到 " << h.far_pitch_deg << " deg 后截一帧\n";
    const int head_rc = move_head_to_rpy_deg(h.yaw_deg, h.far_pitch_deg, h.roll_deg, true);
    if (head_rc != 0)
        return head_rc;
    if (hardware_abort_requested())
        return -4;
    hardware_abort_sleep_ms(500);
    if (hardware_abort_requested())
        return -4;

    if (g_hw->pipeline == nullptr)
    {
        std::cerr << "[orch] waist2 无相机，跳过截图\n";
        return 0;
    }
    RealSenseMultiCam &cameras = g_hw->pipeline->cameras();
    CameraFrameData frame = cameras.grab_fresh(CameraSlot::Head);
    frame = RealSenseMultiCam::prepare_frame_for_slot(std::move(frame), CameraSlot::Head);
    if (!frame.ok || frame.color_bgr.empty() || frame.width <= 0 || frame.height <= 0)
    {
        std::cerr << "[orch] waist2 截帧失败: " << frame.message << "\n";
        return -2;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&tt, &tm);
    std::ostringstream name;
    name << "waist2_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".jpg";
    const std::filesystem::path dir = std::filesystem::path(project_root_dir()) / "picture_debug" / "head";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string path = (dir / name.str()).string();
    cv::Mat bgr(frame.height, frame.width, CV_8UC3, frame.color_bgr.data());
    if (!cv::imwrite(path, bgr))
    {
        std::cerr << "[orch] waist2 保存截图失败 " << path << "\n";
        return -2;
    }
    g_hw->last_debug_photo = path;
    std::cout << "[orch] waist2 截图 " << path << "\n";
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

int go_belt_station(
    const ConveyorStationConfig &c, const char *tag,
    const std::function<int(bool is_right)> &on_hand_placed = {},
    bool do_right = true, bool do_left = true,
    bool skip_ready = false)
{
    const char *name = (tag && tag[0]) ? tag : "belt";
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[orch] " << name << " 失败：腰未初始化\n";
        return -1;
    }
    if (g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
    {
        std::cerr << "[orch] " << name << " 失败：手臂未初始化\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;

    g_hw->last_debug_photo.clear();
    g_hw->last_belt_ok = false;
    g_hw->last_belt_name.clear();
    g_hw->last_belt_message.clear();
    g_hw->last_belt_x = std::numeric_limits<double>::quiet_NaN();
    g_hw->last_belt_y = std::numeric_limits<double>::quiet_NaN();
    g_hw->last_belt_z = std::numeric_limits<double>::quiet_NaN();

    auto aborted = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };

    std::cout << "[orch] " << name << "：准备tcp → 识别传送带6x6 → "
                 "右手放下松爪后，左手立刻过来；右手同时抬起回准备（不回 home）\n";

    if (skip_ready)
    {
        std::cout << "[orch] " << name << " 路上已经摆好拍照姿态，到站后直接等停稳再拍\n";
        g_photo_ready_station.clear();
    }
    else
    {
        const int ready_rc = go_belt_ready(*g_hw->arm_r, *g_hw->arm_l, c, name);
        if (ready_rc != 0)
            return ready_rc;
    }
    if (aborted())
        return -4;
    constexpr int kPhotoAfterArrivalMs = 3000;
    int wait_ms = kPhotoAfterArrivalMs;
    if (g_station_arrived)
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - g_station_arrived_at)
                                 .count();
        wait_ms = static_cast<int>(std::max<int64_t>(0, kPhotoAfterArrivalMs - elapsed));
        std::cout << "[orch] " << name << " 导航完成已 "
                  << elapsed << " ms，再等 " << wait_ms << " ms 后拍照\n";
    }
    else
        std::cout << "[orch] " << name << " 未记录到站时刻，停稳 3s 再拍照\n";
    if (wait_ms > 0)
        hardware_abort_sleep_ms(wait_ms);
    if (aborted())
        return -4;

    if (g_hw->pipeline == nullptr)
    {
        std::cerr << "[orch] belt 无相机，无法识别传送带码\n";
        g_hw->last_belt_message = "无相机";
        return -1;
    }
    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.belt_engine_ready())
    {
        std::cerr << "[orch] 传送带 ArUco 未加载（检查 board_belt_aruco/config.yaml），料盘引擎未动\n";
        g_hw->last_belt_message = "传送带引擎未加载";
        return -5;
    }

    RealSenseMultiCam &cameras = g_hw->pipeline->cameras();
    cameras.flush(CameraSlot::Head);
    CameraFrameData frame = cameras.grab_wait(CameraSlot::Head);
    frame = RealSenseMultiCam::prepare_frame_for_slot(std::move(frame), CameraSlot::Head);
    if (!frame.ok)
    {
        std::cerr << "[orch] belt 头相机取帧失败: " << frame.message << "\n";
        g_hw->last_belt_message = "取帧失败";
        return -2;
    }

    if (kDebugVisualize)
        pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});
    BeltDetectResult det =
        bridge.run_belt_detect(frame, c.prefer_belt, kDebugVisualize, CameraSlot::Head);
    g_hw->last_belt_ok = det.ok && det.in_robot;
    g_hw->last_belt_name = det.name;
    g_hw->last_belt_message = det.message;
    g_hw->last_debug_photo = det.save_path;
    if (det.in_robot)
    {
        g_hw->last_belt_x = det.origin_x;
        g_hw->last_belt_y = det.origin_y;
        g_hw->last_belt_z = det.origin_z;
    }

    std::cout << std::fixed << std::setprecision(4)
              << "[orch] belt 识别 " << (det.ok ? "OK" : "FAIL")
              << " name=" << det.name << " ids=";
    for (size_t i = 0; i < det.used_ids.size(); ++i)
        std::cout << (i ? "," : "") << det.used_ids[i];
    std::cout << " reproj=" << det.reproj_px << "px";
    if (det.in_robot)
        std::cout << " 原点基座=(" << det.origin_x << "," << det.origin_y << ","
                  << det.origin_z << ")";
    else
        std::cout << " 无基座坐标";
    std::cout << " " << det.message << "\n";
    if (!det.save_path.empty())
        std::cout << "[orch] belt 标注图 " << det.save_path << "\n";

    if (!det.ok || !det.in_robot)
    {
        std::cerr << "[orch] belt 未拿到传送带坐标，手臂停在准备姿态，不往码上走\n";
        return det.ok ? -3 : 1;
    }
    if (aborted())
        return -4;

    apply_fixed_belt_origin_z(det, c, name);
    if (det.in_robot)
        g_hw->last_belt_z = det.origin_z;

    const auto &off_r = c.offset_right;
    const auto &off_l = c.offset_left;
    Eigen::Matrix<double, 1, 6> goal_r = move_box::make_belt_goal_right(det, c);
    Eigen::Matrix<double, 1, 6> goal_l = move_box::make_belt_goal_left(det, c);
    if (std::abs(c.place_base_z_bias_m) > 1e-9)
    {
        goal_r(2) += c.place_base_z_bias_m;
        goal_l(2) += c.place_base_z_bias_m;
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] belt 放置松手在基座 Z 再偏 " << c.place_base_z_bias_m
                  << " m，左右手都加\n";
    }
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] belt 放置 原点=(" << det.origin_x << "," << det.origin_y << ","
              << det.origin_z << ") offset_识别系 右=(" << off_r.x << "," << off_r.y << ","
              << off_r.z << ") 左=(" << off_l.x << "," << off_l.y << "," << off_l.z
              << ") have_axes=" << (det.have_axes ? "yes" : "no") << "\n";
    if (det.have_axes)
        std::cout << "[orch] belt 码轴 基座 +X=(" << det.ax_x << "," << det.ax_y << ","
                  << det.ax_z << ") +Y=(" << det.ay_x << "," << det.ay_y << "," << det.ay_z
                  << ") +Z=(" << det.az_x << "," << det.az_y << "," << det.az_z << ")\n";
    std::cout << "[orch] belt 下发目标 右=(" << goal_r(0) << "," << goal_r(1) << ","
              << goal_r(2) << ") 左=(" << goal_l(0) << "," << goal_l(1) << "," << goal_l(2)
              << ") 右手松爪后左手立刻过来，不等右手回准备\n";
    std::cout.flush();

    auto place_arrive_and_open = [&](bool is_right) -> int {
        if (is_right && right_arm_motors_locked())
        {
            std::cout << "[orch] belt 右臂锁定，跳过右手放置\n";
            return 0;
        }
        Eigen::Matrix<double, 1, 6> goal = is_right ? goal_r : goal_l;
        constexpr double kPlaceMoveMps = 0.15;
        std::cout << "[orch] belt " << (is_right ? "右手" : "左手")
                  << " 0.15m/s 抹角到放置点，不在上方停车\n";
        Eigen::Matrix<double, 1, 6> hover = goal;
        if (c.place_hover_above_m > 1e-9)
            hover(2) += c.place_hover_above_m;
        const ArmLineMoveResult approach = arm_dual_rounded_corner_move_selective(
            *g_hw->arm_r, hover, goal, is_right,
            *g_hw->arm_l, hover, goal, !is_right,
            kPlaceMoveMps);
        const int approach_rc = is_right ? approach.ret_r : approach.ret_l;
        if (approach_rc != 0)
        {
            std::cerr << "[orch] belt " << (is_right ? "右手" : "左手")
                      << " 放置伸手失败 code=" << approach_rc << "\n";
            return aborted() ? -4 : -5;
        }
        if (aborted())
            return -4;
        std::cout << "[orch] belt " << (is_right ? "右手" : "左手") << " 放置到位，停 0.5s 再松爪\n";
        hardware_abort_sleep_ms(500);
        if (aborted())
            return -4;
        if (g_hw->g != nullptr)
        {
            std::cout << "[orch] belt " << (is_right ? "右手" : "左手")
                      << " 放置点松爪到 0.2（0全开 1全合）\n";
            log_gripper_qs("放置松爪前");
            const bool opened = g_hw->g->closeAndWait(
                is_right ? gripper::Side::Right : gripper::Side::Left,
                0.2, 0.2, std::chrono::milliseconds(800));
            log_gripper_qs(opened ? "放置松爪到位" : "放置松爪已下发，不等开到位");
            if (aborted())
                return -4;
        }
        return aborted() ? -4 : 0;
    };

    auto lift_and_retract = [&](bool is_right) -> int {
        if (is_right && right_arm_motors_locked())
            return 0;
        if (aborted())
            return -4;
        Robot_Arm &arm = is_right ? *g_hw->arm_r : *g_hw->arm_l;
        constexpr double kPlaceNudgeUpM = 0.03;
        constexpr double kPlaceMoveMps = 0.15;
        Eigen::Matrix<double, 1, 6> lift = arm_get_tcp_pos(arm);
        const double z0 = lift(2);
        lift(2) = z0 + kPlaceNudgeUpM;
        Eigen::Matrix<double, 1, 6> ready = is_right ? c.tcp.right : c.tcp.left;
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] belt " << (is_right ? "右手" : "左手")
                  << " 0.15m/s 松爪后抬 " << kPlaceNudgeUpM << "m 后不停，直接回准备\n";
        const ArmLineMoveResult back = arm_dual_line_then_line_nostop(
            *g_hw->arm_r, lift, ready, is_right,
            *g_hw->arm_l, lift, ready, !is_right,
            kPlaceMoveMps, kPlaceMoveMps, {}, false, "放置收手");
        const int back_rc = is_right ? back.ret_r : back.ret_l;
        if (back_rc != 0)
        {
            std::cerr << "[orch] belt " << (is_right ? "右手" : "左手")
                      << " 放置收手失败 code=" << back_rc << "\n";
            return aborted() ? -4 : -5;
        }
        return aborted() ? -4 : 0;
    };

    if (!do_right && !do_left)
    {
        std::cout << "[orch] " << name << " 这次没有要放的手，跳过放置动作\n";
        return 0;
    }

    int right_back_rc = 0;
    std::thread th_r;
    const bool run_right = do_right && !right_arm_motors_locked();
    if (run_right)
    {
        const int right_rc = place_arrive_and_open(true);
        if (right_rc != 0)
            return right_rc;
        if (on_hand_placed)
        {
            const int note_rc = on_hand_placed(true);
            if (note_rc != 0)
                return note_rc;
        }
        th_r = std::thread([&]() { right_back_rc = lift_and_retract(true); });
    }
    else if (!do_right)
        std::cout << "[orch] belt 右手这次不放\n";

    int left_rc = 0;
    int left_back_rc = 0;
    if (do_left)
    {
        if (run_right)
            std::cout << "[orch] belt 右手已放下，左手立刻过来（不等右手抬起回准备）\n";
        left_rc = place_arrive_and_open(false);
        if (left_rc == 0 && on_hand_placed)
        {
            const int note_rc = on_hand_placed(false);
            if (note_rc != 0)
            {
                if (th_r.joinable())
                    th_r.join();
                return note_rc;
            }
        }
        left_back_rc = (left_rc == 0) ? lift_and_retract(false) : 0;
    }
    else
        std::cout << "[orch] belt 左手这次不放\n";
    if (th_r.joinable())
        th_r.join();
    if (left_rc != 0)
        return left_rc;
    if (left_back_rc != 0)
        return left_back_rc;
    if (right_back_rc != 0)
        return right_back_rc;

    std::cout << "[orch] " << name << " 双手放置结束，停在准备姿态\n";
    return 0;
}

int go_belt_station()
{
    return go_belt_station(g_move_cfg.conveyor, "belt");
}

int go_belt()
{
    const std::string &st = g_move_cfg.chassis.belt_station;
    std::cout << "[orch] belt：先到 " << st << " 再放置\n";
    const int ch_rc = chassis_goto_station(st);
    if (ch_rc != 0)
        return ch_rc;
    if (hardware_abort_requested())
        return -4;
    std::cout << "[orch] belt 底盘已到 " << st << "，到站满 3s 再拍照\n";
    return go_belt_station(g_move_cfg.conveyor, "belt");
}

int go_belt_grasp(
    bool from_place, const ConveyorStationConfig &c, const std::string &station, const char *tag,
    int only_hand = -1, const std::function<int(bool is_right)> &on_grasped = {},
    bool photo_settle = true, const std::string &depart_after_right = {})
{
    struct BeltDepartGuard
    {
        ~BeltDepartGuard() { take_belt_depart(); }
    } belt_depart_guard;
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[orch] belt_grasp 失败：腰未初始化\n";
        return -1;
    }
    if (g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
    {
        std::cerr << "[orch] belt_grasp 失败：手臂未初始化\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;

    g_photo_ready_station.clear();
    g_hw->last_debug_photo.clear();
    g_hw->last_belt_ok = false;
    g_hw->last_belt_name.clear();
    g_hw->last_belt_message.clear();
    g_hw->last_belt_x = std::numeric_limits<double>::quiet_NaN();
    g_hw->last_belt_y = std::numeric_limits<double>::quiet_NaN();
    g_hw->last_belt_z = std::numeric_limits<double>::quiet_NaN();

    auto aborted = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };

    const char *name = (tag && tag[0]) ? tag : "belt_grasp";
    if (from_place)
        std::cout << "[orch] " << name << "：放置后已在准备 tcp，不转腰，直接拍照 → 抓取\n";
    else
        std::cout << "[orch] " << name << "：先到 " << station
                  << " → 腰/头到位 → 拍照 → 再动手臂（不转腰）\n";

    if (!from_place)
    {
        const int ch_rc = chassis_goto_station(station);
        if (ch_rc != 0)
            return ch_rc;
        if (aborted())
            return -4;
        std::cout << "[orch] " << name << " 底盘已到 " << station
                  << "，腰头到位后停稳 1.5s 再拍照\n";
        const int wh = conveyor_waist_and_head(c, name);
        if (wh != 0)
            return wh;
        if (aborted())
            return -4;
    }

    if (photo_settle)
    {
        std::cout << "[orch] belt_grasp 不转腰，停稳 1.5s 再拍照；grasp_offset 按码坐标系，允许过中轴\n";
        hardware_abort_sleep_ms(1500);
    }
    else
        std::cout << "[orch] belt_grasp 这一只手底盘没动过，不再停 1.5s，直接拍照\n";
    if (aborted())
        return -4;

    if (g_hw->pipeline == nullptr)
    {
        std::cerr << "[orch] belt_grasp 无相机，无法识别传送带码\n";
        g_hw->last_belt_message = "无相机";
        return -1;
    }
    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.belt_engine_ready())
    {
        std::cerr << "[orch] 传送带 ArUco 未加载（检查 board_belt_aruco/config.yaml），料盘引擎未动\n";
        g_hw->last_belt_message = "传送带引擎未加载";
        return -5;
    }

    RealSenseMultiCam &cameras = g_hw->pipeline->cameras();
    cameras.flush(CameraSlot::Head);
    CameraFrameData frame = cameras.grab_wait(CameraSlot::Head);
    frame = RealSenseMultiCam::prepare_frame_for_slot(std::move(frame), CameraSlot::Head);
    if (!frame.ok)
    {
        std::cerr << "[orch] belt_grasp 头相机取帧失败: " << frame.message << "\n";
        g_hw->last_belt_message = "取帧失败";
        return -2;
    }

    if (kDebugVisualize)
        pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});
    const double grasp_off_l[3] = {
        c.grasp_offset_left.x, c.grasp_offset_left.y, c.grasp_offset_left.z};
    const double grasp_off_r[3] = {
        c.grasp_offset_right.x, c.grasp_offset_right.y, c.grasp_offset_right.z};
    BeltDetectResult det = bridge.run_belt_detect(
        frame, c.prefer_belt, kDebugVisualize, CameraSlot::Head, grasp_off_l, grasp_off_r);
    const bool need_l = only_hand != 1;
    const bool need_r = only_hand != 0;
    g_hw->last_belt_ok = det.ok && (!need_l || det.have_grasp_l) && (!need_r || det.have_grasp_r);
    g_hw->last_belt_name = det.name;
    g_hw->last_belt_message = det.message;
    g_hw->last_debug_photo = det.save_path;
    if (det.have_grasp_l)
    {
        g_hw->last_belt_x = det.grasp_lx;
        g_hw->last_belt_y = det.grasp_ly;
        g_hw->last_belt_z = det.grasp_lz;
    }
    else if (det.in_robot)
    {
        g_hw->last_belt_x = det.origin_x;
        g_hw->last_belt_y = det.origin_y;
        g_hw->last_belt_z = det.origin_z;
    }

    std::cout << std::fixed << std::setprecision(4)
              << "[orch] belt_grasp 识别 " << (det.ok ? "OK" : "FAIL")
              << " name=" << det.name << " ids=";
    for (size_t i = 0; i < det.used_ids.size(); ++i)
        std::cout << (i ? "," : "") << det.used_ids[i];
    std::cout << " reproj=" << det.reproj_px << "px";
    if (det.in_robot)
        std::cout << " 原点基座=(" << det.origin_x << "," << det.origin_y << ","
                  << det.origin_z << ")";
    else
        std::cout << " 无基座坐标";
    if (det.have_grasp_l)
        std::cout << " 左抓=(" << det.grasp_lx << "," << det.grasp_ly << "," << det.grasp_lz << ")";
    if (det.have_grasp_r)
        std::cout << " 右抓=(" << det.grasp_rx << "," << det.grasp_ry << "," << det.grasp_rz << ")";
    if (!det.have_grasp_l && !det.have_grasp_r)
        std::cout << " 无抓取点";
    if (det.have_axes)
        std::cout << " 板+Y基座=(" << det.ay_x << "," << det.ay_y << "," << det.ay_z << ")";
    std::cout << " " << det.message << "\n";
    if (!det.save_path.empty())
        std::cout << "[orch] belt_grasp 标注图 " << det.save_path << "\n";

    if (!det.ok || (need_l && !det.have_grasp_l) || (need_r && !det.have_grasp_r))
    {
        std::cerr << "[orch] belt_grasp 未拿到"
                  << (only_hand < 0 ? "左右" : (only_hand == 0 ? "左手" : "右手"))
                  << "抓取点，不往码上走\n";
        return det.ok ? -3 : 1;
    }
    apply_fixed_belt_origin_z(det, c, name);
    if (det.have_grasp_l)
        g_hw->last_belt_z = det.grasp_lz;
    else if (det.in_robot)
        g_hw->last_belt_z = det.origin_z;
    if (c.grasp_yolo_enable && c.grasp_yolo_class_id >= 0)
    {
        std::string yolo_err;
        if (!belt_grasp_yolo_class_ok(bridge, frame, c.grasp_yolo_class_id, yolo_err))
        {
            g_hw->last_belt_ok = false;
            g_hw->last_belt_message = yolo_err;
            std::cerr << "[orch] belt_grasp " << yolo_err
                      << "，要求 " << belt_yolo_class_zh(c.grasp_yolo_class_id)
                      << " class=" << c.grasp_yolo_class_id << "，不抓\n";
            return 1;
        }
        std::cout << "[orch] belt_grasp YOLO 判定通过："
                  << belt_yolo_class_zh(c.grasp_yolo_class_id)
                  << " class=" << c.grasp_yolo_class_id << "，定位仍用码\n";
    }
    if (aborted())
        return -4;

    if (from_place)
        std::cout << "[orch] belt_grasp 认码完成，手臂已在放置后的准备 tcp，直接抓\n";
    else
    {
        std::cout << "[orch] belt_grasp 认码完成，手臂到准备 tcp 再抓\n";
        if (!move_box::move_arms_to_belt_ready(*g_hw->arm_r, *g_hw->arm_l, c))
        {
            std::cerr << "[orch] " << name << " 手臂到准备姿态失败\n";
            return aborted() ? -4 : -5;
        }
        if (aborted())
            return -4;
    }

    const auto &off_l = c.grasp_offset_left;
    const auto &off_r = c.grasp_offset_right;
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] belt_grasp 不转腰拍照，抓取点按码坐标系，左右可过中轴\n"
              << "[orch] belt_grasp 码原点=(" << det.origin_x << "," << det.origin_y << ","
              << det.origin_z << ") 左抓=(" << det.grasp_lx << "," << det.grasp_ly << ","
              << det.grasp_lz << ") offset_左=(" << off_l.x << "," << off_l.y << "," << off_l.z
              << ") 右抓=(" << det.grasp_rx << "," << det.grasp_ry << "," << det.grasp_rz
              << ") offset_右=(" << off_r.x << "," << off_r.y << "," << off_r.z
              << ") hover=" << c.grasp_hover_above_m
              << " rpy_deg 左=(" << c.grasp_rpy_left.rx << "," << c.grasp_rpy_left.ry << ","
              << c.grasp_rpy_left.rz << ") 右=(" << c.grasp_rpy_right.rx << ","
              << c.grasp_rpy_right.ry << "," << c.grasp_rpy_right.rz << ")\n";

    if (g_hw->g != nullptr)
    {
        double grip_r = c.grasp_ready_close_ratio;
        if (grip_r < 0.0)
            grip_r = 0.0;
        if (grip_r > 1.0)
            grip_r = 1.0;
        std::cout << std::fixed << std::setprecision(2)
                  << "[orch] belt_grasp 准备姿态夹爪收到 " << grip_r
                  << "（0全开 1全合），避免磕皮带边\n";
        log_gripper_qs("belt_grasp准备夹爪前");
        if (only_hand != 1)
            g_hw->g->closeAndWait(
                gripper::Side::Left, grip_r, 0.15, std::chrono::milliseconds(3000));
        if (only_hand != 0 && !right_arm_motors_locked())
            g_hw->g->closeAndWait(
                gripper::Side::Right, grip_r, 0.15, std::chrono::milliseconds(3000));
        log_gripper_qs("belt_grasp准备夹爪后");
        if (aborted())
            return -4;
    }

    auto grasp_one = [&](bool is_right) -> int {
        if (is_right && right_arm_motors_locked())
        {
            std::cout << "[orch] belt_grasp 右臂锁定，跳过右手抓取\n";
            return 0;
        }
        Eigen::Matrix<double, 1, 6> goal = move_box::make_belt_grasp_goal(
            is_right,
            is_right ? det.grasp_rx : det.grasp_lx,
            is_right ? det.grasp_ry : det.grasp_ly,
            is_right ? det.grasp_rz : det.grasp_lz,
            c);
        const char *hand = is_right ? "右手" : "左手";
        Eigen::Matrix<double, 1, 6> hover = goal;
        const double hover_z = c.grasp_hover_above_m > 1e-9 ? c.grasp_hover_above_m : 0.08;
        hover(2) += hover_z;
        constexpr double reach_mps = 0.15;
        std::cout << std::fixed << std::setprecision(2)
                  << "[orch] belt_grasp " << hand << " " << reach_mps
                  << "m/s 抹角到抓取点，不在上方停车\n";
        const ArmLineMoveResult approach = arm_dual_rounded_corner_move_selective(
            *g_hw->arm_r, hover, goal, is_right,
            *g_hw->arm_l, hover, goal, !is_right,
            reach_mps);
        const int approach_rc = is_right ? approach.ret_r : approach.ret_l;
        if (approach_rc != 0)
        {
            std::cerr << "[orch] belt_grasp " << hand
                      << " 伸手抹角失败 code=" << approach_rc << "\n";
            return aborted() ? -4 : -5;
        }
        if (aborted())
            return -4;
        std::cout << "[orch] belt_grasp " << hand << " 到位，停 0.5s 再合爪\n";
        hardware_abort_sleep_ms(500);
        if (aborted())
            return -4;
        if (g_hw->g != nullptr)
        {
            std::cout << "[orch] belt_grasp " << hand << " 合爪\n";
            log_gripper_qs(is_right ? "belt_grasp右合爪前" : "belt_grasp左合爪前");
            const auto fb = g_hw->g->graspAndWait(
                is_right ? gripper::Side::Right : gripper::Side::Left);
            log_gripper_qs(is_right ? "belt_grasp右合爪后" : "belt_grasp左合爪后");
            if (fb.result != gripper::GraspResult::TorqueLimit &&
                fb.result != gripper::GraspResult::PositionReached)
            {
                std::cerr << "[orch] belt_grasp " << hand
                          << " 夹取异常 result=" << static_cast<int>(fb.result)
                          << " q=" << fb.position_rad << "\n";
            }
            if (is_right)
            {
                g_hw->last_grasped_r = true;
                if (!depart_after_right.empty())
                    start_belt_depart_after_right(depart_after_right);
            }
            else
                g_hw->last_grasped_l = true;
            if (aborted())
                return -4;
        }
        return aborted() ? -4 : 0;
    };

    const bool back_to_belt_ready =
        station_name_matches(station, g_move_cfg.chassis.belt_station);
    const bool back_to_tray_ready =
        station_name_matches(station, g_move_cfg.chassis.out_station);
    const char *back_name = back_to_belt_ready ? "传送带准备 tcp"
                            : back_to_tray_ready ? "料盘放置准备"
                                                 : "grasp_tcp";
    auto grasp_lift_and_back = [&](bool is_right) -> int {
        if (is_right && right_arm_motors_locked())
            return 0;
        if (aborted())
            return -4;
        Robot_Arm &arm = is_right ? *g_hw->arm_r : *g_hw->arm_l;
        const char *hand = is_right ? "右手" : "左手";
        if (aborted())
            return -4;
        const double lift_z = c.grasp_hover_above_m > 1e-9
                                  ? c.grasp_hover_above_m
                                  : g_move_cfg.head_grasp.lift_after_grasp_z;
        Eigen::Matrix<double, 1, 6> lift = arm_get_tcp_pos(arm);
        lift(2) += lift_z;
        if (lift(2) > g_move_cfg.grasp_valid.z_max)
            lift(2) = g_move_cfg.grasp_valid.z_max;
        Eigen::Matrix<double, 1, 6> back_pose =
            is_right ? c.grasp_tcp.right : c.grasp_tcp.left;
        if (back_to_belt_ready)
            back_pose = is_right ? c.tcp.right : c.tcp.left;
        else if (back_to_tray_ready)
            back_pose = is_right ? g_move_cfg.standby.right : g_move_cfg.standby.left;
        // 平移时保持抓取腕角。腕子差得大的目标姿态等到位置到了再慢转，避免接缝抽臂。
        Eigen::Matrix<double, 1, 6> back_move = back_pose;
        back_move(3) = lift(3);
        back_move(4) = lift(4);
        back_move(5) = lift(5);
        constexpr double retract_mps = 0.15;
        std::cout << std::fixed << std::setprecision(3)
                  << "[orch] belt_grasp " << hand << " " << retract_mps
                  << "m/s 抬 " << lift_z << "m 后不停，先平移回 " << back_name
                  << "，腕角保持抓取姿态\n";
        const ArmLineMoveResult stitched = arm_dual_line_then_line_nostop(
            *g_hw->arm_r, lift, back_move, is_right,
            *g_hw->arm_l, lift, back_move, !is_right,
            retract_mps, retract_mps, {}, false, "传送带抓取收手");
        const int back_rc = is_right ? stitched.ret_r : stitched.ret_l;
        if (back_rc == 0)
        {
            auto wrap = [](double a) {
                return std::abs(std::atan2(std::sin(a), std::cos(a)));
            };
            const bool need_wrist =
                wrap(back_pose(3) - lift(3)) > 2.0 * M_PI / 180.0 ||
                wrap(back_pose(4) - lift(4)) > 2.0 * M_PI / 180.0 ||
                wrap(back_pose(5) - lift(5)) > 2.0 * M_PI / 180.0;
            if (!need_wrist)
                return aborted() ? -4 : 0;
            std::cout << "[orch] belt_grasp " << hand
                      << " 已到回程位置，40°/s 慢转腕到 " << back_name << "\n";
            const int rot_rc = arm_reorient_limited(arm, back_pose, 40.0);
            if (rot_rc != 0)
            {
                std::cerr << "[orch] belt_grasp " << hand << " 慢转腕失败 code=" << rot_rc << "\n";
                return aborted() ? -4 : -5;
            }
            return aborted() ? -4 : 0;
        }
        if (aborted())
            return -4;
        std::cerr << "[orch] belt_grasp " << hand
                  << " 收手衔接失败 code=" << back_rc << "，退回抬升后直线\n";
        if (!move_box::move_one_arm_line_to(
                arm, is_right, lift, is_right ? "belt_grasp右手抬升" : "belt_grasp左手抬升",
                false))
        {
            if (aborted())
                return -4;
        }
        if (!move_box::move_one_arm_line_to(
                arm, is_right, back_pose,
                is_right ? "belt_grasp右手回收手" : "belt_grasp左手回收手"))
        {
            std::cerr << "[orch] belt_grasp " << hand << " 回 " << back_name << " 失败\n";
            return aborted() ? -4 : -5;
        }
        return aborted() ? -4 : 0;
    };

    if (only_hand == 0 || only_hand == 1)
    {
        const bool is_right = only_hand == 1;
        const int rc = grasp_one(is_right);
        if (rc != 0)
            return rc;
        int note_rc = 0;
        std::thread note_th;
        if (on_grasped && !(is_right && right_arm_motors_locked()))
            note_th = std::thread([&]() { note_rc = on_grasped(is_right); });
        const int back_rc = grasp_lift_and_back(is_right);
        if (note_th.joinable())
            note_th.join();
        if (note_rc != 0)
            return note_rc;
        if (back_rc != 0)
            return back_rc;
        const int dep_rc = take_belt_depart();
        if (dep_rc != 0)
            return dep_rc;
        std::cout << "[orch] " << name << (is_right ? " 右手" : " 左手")
                  << "单手抓取结束，停在 " << back_name << "\n";
        return 0;
    }

    int left_back_rc = 0;
    const int left_rc = grasp_one(false);
    if (left_rc != 0)
        return left_rc;
    if (right_arm_motors_locked())
    {
        const int back_only = grasp_lift_and_back(false);
        if (back_only != 0)
            return back_only;
    }
    else
    {
        std::thread th_l([&]() { left_back_rc = grasp_lift_and_back(false); });
        std::cout << "[orch] belt_grasp 左手已夹到，右手立刻过来（不等左手抬起回程）\n";
        const int right_rc = grasp_one(true);
        const int right_back_rc = (right_rc == 0) ? grasp_lift_and_back(true) : 0;
        th_l.join();
        if (right_rc != 0)
            return right_rc;
        if (right_back_rc != 0)
            return right_back_rc;
        if (left_back_rc != 0)
            return left_back_rc;
    }

    const int dep_rc = take_belt_depart();
    if (dep_rc != 0)
        return dep_rc;
    std::cout << "[orch] " << name << " 双手抓取结束，停在 " << back_name << "，可接入底盘/放置\n";
    return 0;
}

int go_belt_grasp(bool from_place)
{
    return go_belt_grasp(
        from_place, g_move_cfg.conveyor, g_move_cfg.chassis.belt_station, "belt_grasp");
}

int chassis_goto_belt2_and_ready(Robot_Arm &arm_r, Robot_Arm &arm_l)
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

int go_belt2_ready(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    return chassis_goto_belt2_and_ready(arm_r, arm_l);
}

int go_belt2_grasp_rpy(Robot_Arm &arm_r, Robot_Arm &arm_l)
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
    std::cout << "[orch] belt2 底盘已到 " << st << "，到站满 3s 再拍照\n";
    return go_belt_station(g_move_cfg.conveyor2, "belt2");
}

int go_belt2_place()
{
    return go_belt2();
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

int go_waist_jog(int joint, double dq_rad)
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[orch] waist_jog 失败：腰未初始化\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;
    constexpr double kMaxAbs = 0.12; // ≈6.9°
    if (!std::isfinite(dq_rad))
    {
        std::cerr << "[orch] waist_jog dq_rad 无效\n";
        return -1;
    }
    dq_rad = std::clamp(dq_rad, -kMaxAbs, kMaxAbs);
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] waist_jog joint=" << joint << " dq=" << dq_rad
              << " rad (" << (dq_rad * 180.0 / M_PI) << " deg)，手臂不动\n";
    const int rc = g_hw->waist->jogJoint(joint, dq_rad, 0.08);
    if (rc != 0)
    {
        std::cerr << "[orch] waist_jog 失败 code=" << rc << "\n";
        return hardware_abort_requested() ? -4 : (rc == -2 ? -2 : -5);
    }
    if (!move_box::sync_waist_layer3_from_encoders(
            *g_hw->waist, *g_hw->layer3, "jog后"))
        return -5;
    move_box::waist_layer3_set_synced(true);
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] waist_jog 到位 x=" << (*g_hw->layer3)(0)
              << " z=" << (*g_hw->layer3)(2) << " m\n";
    return 0;
}

namespace
{

int vision_grasp_with_engine(SegEngineId engine_id, bool adaptive_approach_rpy = false)
{
    g_load_parts_seen = -1;
    g_grasp_departed = false;
    if (g_hw == nullptr || g_hw->g == nullptr || g_hw->pipeline == nullptr)
        return -1;

    std::array<double, 16> cam2robot{};
    std::string err;
    if (!load_head_cam2robot(cam2robot, err) &&
        !load_cam2robot_matrix(default_camera_to_robot_yaml_path(), cam2robot, err))
    {
        std::cerr << "[orch] 读 cam2robot 失败: " << err << std::endl;
        return -3;
    }

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };

    std::cout << "[grasp] 去料盘 " << g_move_cfg.chassis.tray_station
              << "，路上同时摆头、手臂、开爪和下腰\n";
    int prep_rc = 0;
    int travel_rc = 0;
    {
        ChassisTravel travel(g_move_cfg.chassis.tray_station);
        const int head_rc = enable_head_calib_pose();
        if (head_rc != 0)
            prep_rc = head_rc;
        else if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l))
        {
            std::cerr << "[grasp] 进入时回到 standby 失败\n";
            prep_rc = should_abort() ? -4 : -5;
        }
        else if (should_abort())
            prep_rc = -4;
        else
        {
            g_hw->g->openAndWait(gripper::Side::Right, 0.2, std::chrono::milliseconds(1500));
            g_hw->g->openAndWait(gripper::Side::Left, 0.2, std::chrono::milliseconds(1500));
            if (should_abort())
                prep_rc = -4;
            else if (g_hw->waist == nullptr || g_hw->layer3 == nullptr)
            {
                std::cerr << "[grasp] 腰未初始化，拒绝进入抓取\n";
                prep_rc = -1;
            }
            else if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
            {
                std::cerr << "[grasp] 腰未到 ready 起始位，放弃抓取\n";
                prep_rc = should_abort() ? -4 : -5;
            }
            else
                move_box::waist_layer3_set_synced(true);
        }
        travel_rc = travel.join();
    }
    if (should_abort())
        return -4;
    if (travel_rc != 0)
        return travel_rc;
    if (prep_rc != 0)
        return prep_rc;
    std::cout << "[grasp] 已在 " << g_move_cfg.chassis.tray_station << "，停稳 0.5s 再抓\n";
    hardware_abort_sleep_ms(500);
    if (should_abort())
        return -4;

    g_load_parts_seen = -1;

    constexpr int kMaxRounds = 40;
    constexpr int kMaxNoneStreak = 2;
    int grasped_count = 0;
    int none_streak = 0;
    bool holding_r = false;
    bool holding_l = false;
    g_hw->last_grasp_count = 0;
    g_hw->last_grasped_r = false;
    g_hw->last_grasped_l = false;

    const auto both_holding = [&]() -> bool {
        return right_arm_motors_locked() ? holding_l : (holding_r && holding_l);
    };

    std::cout << "[grasp] 开始循环：夹后保持合爪；双手都有件才结束，缺的再去拿"
              << (right_arm_motors_locked() ? "（右臂锁定，只要求左手持件）\n" : "\n");

    std::thread chassis_th;
    std::atomic<int> chassis_rc{0};
    bool chassis_started = false;
    struct ChassisJoin
    {
        std::thread &th;
        ~ChassisJoin()
        {
            if (th.joinable())
                th.join();
        }
    } chassis_join{chassis_th};
    const std::string next_station = g_move_cfg.chassis.belt_station;
    auto start_chassis = [&]() {
        if (chassis_started)
            return;
        chassis_started = true;
        std::cout << "[grasp] 收手已完成，底盘立刻去 " << next_station << std::endl;
        chassis_th = std::thread([&]() {
            try
            {
                chassis_rc = chassis_goto_station(next_station);
            }
            catch (const std::exception &ex)
            {
                std::cerr << "[grasp] 底盘导航失败: " << ex.what() << "\n";
                chassis_rc = -6;
            }
        });
    };
    int last_rc = 1;
    for (int round = 1; round <= kMaxRounds; ++round)
    {
        if (should_abort())
        {
            g_hw->last_grasp_count = grasped_count;
            g_hw->last_grasp_status = -4;
            std::cout << "[grasp] 循环中止 已持件 右=" << (holding_r ? 1 : 0)
                      << " 左=" << (holding_l ? 1 : 0) << "\n";
            return -4;
        }

        if (both_holding())
        {
            last_rc = 0;
            std::cout << "[grasp] 双手已持件，结束抓取循环\n";
            break;
        }

        std::cout << "\n======== [grasp] 第 " << round << " 轮 持件 右="
                  << (holding_r ? 1 : 0) << " 左=" << (holding_l ? 1 : 0)
                  << " ========\n";
        move_box::GraspToStandbyResult r = move_box::run_grasp_to_standby(
            *g_hw->g,
            *g_hw->arm_r,
            *g_hw->arm_l,
            *g_hw->waist,
            *g_hw->layer3,
            g_hw->pipeline->cameras(),
            g_hw->pipeline->bridge(),
            cam2robot,
            err,
            should_abort,
            engine_id,
            adaptive_approach_rpy,
            holding_r,
            holding_l);

        if (g_load_parts_seen < 0 && r.zone_count_ok)
        {
            g_load_parts_seen = r.zone_right + r.zone_left;
            std::cout << "[grasp] 选孔与计数同一张照片，开抓前工作区毛坯 "
                      << g_load_parts_seen
                      << " 件（右=" << r.zone_right << " 左=" << r.zone_left << "）\n";
        }

        holding_r = holding_r || r.grasped_r;
        holding_l = holding_l || r.grasped_l;
        grasped_count = (holding_r ? 1 : 0) + (holding_l ? 1 : 0);
        g_hw->last_grasped_r = holding_r;
        g_hw->last_grasped_l = holding_l;
        g_hw->last_grasp_status = static_cast<int>(r.status);

        if (r.status == move_box::GraspToStandbyStatus::Aborted)
        {
            g_hw->last_grasp_count = grasped_count;
            std::cout << "[grasp] 循环中止 已持件 右=" << (holding_r ? 1 : 0)
                      << " 左=" << (holding_l ? 1 : 0) << "\n";
            return -4;
        }

        if (r.status == move_box::GraspToStandbyStatus::Ok || r.grasped_r || r.grasped_l)
        {
            none_streak = 0;
            std::cout << "[grasp] 第 " << round << " 轮完成 本轮新抓"
                      << " 右=" << (r.grasped_r ? 1 : 0)
                      << " 左=" << (r.grasped_l ? 1 : 0)
                      << " 现持件 右=" << (holding_r ? 1 : 0)
                      << " 左=" << (holding_l ? 1 : 0)
                      << "，保持合爪";
            if (both_holding())
            {
                std::cout << "，双手已齐\n";
                last_rc = 0;
                break;
            }
            std::cout << "，缺件继续补抓\n";
            last_rc = 0;
            continue;
        }

        if (r.status == move_box::GraspToStandbyStatus::NoTarget)
        {
            if (both_holding())
            {
                last_rc = 0;
                break;
            }
            std::cout << "[grasp] 工作区已无待抓目标，仍缺件"
                      << " 右=" << (holding_r ? 1 : 0)
                      << " 左=" << (holding_l ? 1 : 0) << "，结束循环\n";
            last_rc = (holding_r || holding_l) ? 2 : 1;
            break;
        }

        ++none_streak;
        std::cerr << "[grasp] 第 " << round << " 轮未夹到 code="
                  << static_cast<int>(r.status) << " 连续失败=" << none_streak
                  << " 持件 右=" << (holding_r ? 1 : 0)
                  << " 左=" << (holding_l ? 1 : 0) << "\n";
        if (none_streak >= kMaxNoneStreak)
        {
            std::cerr << "[grasp] 连续 " << none_streak << " 轮未夹到，退出循环\n";
            last_rc = both_holding() ? 0 : ((holding_r || holding_l) ? 2 : 2);
            break;
        }
        last_rc = 2;
    }

    g_hw->last_grasp_count = grasped_count;
    g_hw->last_grasp_status = last_rc;
    std::cout << "\n[grasp] 循环结束 持件 右=" << (holding_r ? 1 : 0)
              << " 左=" << (holding_l ? 1 : 0)
              << " 共 " << grasped_count << " 件，夹爪保持合上" << std::endl;
    if (holding_r || holding_l)
    {
        start_chassis();
        std::cout << "[grasp] 底盘已出发，腰直接到传送带拍照前伸，不再先收回再伸出"
                  << std::endl;
        waist_rotate_deg(0.0, 30.0);
        const int pose_rc = conveyor_waist_and_head(g_move_cfg.conveyor, "grasp_depart");
        if (chassis_th.joinable())
            chassis_th.join();
        if (pose_rc != 0)
            return pose_rc;
        if (chassis_rc.load() != 0)
            return chassis_rc.load();
        g_grasp_departed = true;
        g_photo_ready_station = next_station;
        std::cout << "[grasp] 腰和底盘已结束，已在 " << next_station
                  << "，拍照姿态已摆好" << std::endl;
    }
    return last_rc;
}

} // namespace

int vision_grasp_existing_pipeline()
{
    return vision_grasp_with_engine(SegEngineId::Default);
}

int vision_place_tray2(bool allow_right, bool allow_left, bool return_to_tray)
{
    if (g_hw == nullptr || g_hw->g == nullptr || g_hw->pipeline == nullptr)
        return -1;

    std::array<double, 16> cam2robot{};
    std::string err;
    if (!load_head_cam2robot(cam2robot, err) &&
        !load_cam2robot_matrix(default_camera_to_robot_yaml_path(), cam2robot, err))
    {
        std::cerr << "[orch] 读 cam2robot 失败: " << err << std::endl;
        return -3;
    }

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };

    struct TrayTaskGuard
    {
        ~TrayTaskGuard() { move_box::set_tray_hole_task(move_box::TrayHoleTask::GraspParts); }
    } task_guard;
    move_box::set_tray_hole_task(move_box::TrayHoleTask::PlaceEmpty);

    std::cout << "[tray2] 去料盘2 " << g_move_cfg.chassis.tray2_station
              << "，路上同时摆头、待机臂和下腰\n";
    int prep_rc = 0;
    int travel_rc = 0;
    {
        ChassisTravel travel(g_move_cfg.chassis.tray2_station);
        const int head_rc = enable_head_calib_pose();
        if (head_rc != 0)
            prep_rc = head_rc;
        else if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l))
        {
            std::cerr << "[tray2] 进入时回到 standby 失败\n";
            prep_rc = should_abort() ? -4 : -5;
        }
        else if (g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        {
            std::cerr << "[tray2] 腰未初始化，拒绝进入放置\n";
            prep_rc = -1;
        }
        else if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
        {
            std::cerr << "[tray2] 腰未到 ready 起始位，放弃放置\n";
            prep_rc = should_abort() ? -4 : -5;
        }
        else
            move_box::waist_layer3_set_synced(true);
        travel_rc = travel.join();
    }
    if (should_abort())
        return -4;
    if (travel_rc != 0)
        return travel_rc;
    if (prep_rc != 0)
        return prep_rc;
    std::cout << "[tray2] 已在 " << g_move_cfg.chassis.tray2_station << "，停稳 0.5s 再放\n";
    hardware_abort_sleep_ms(500);
    if (should_abort())
        return -4;

    constexpr int kMaxRounds = 40;
    constexpr int kMaxNoneStreak = 2;
    int placed_count = 0;
    int none_streak = 0;
    bool placed_r = false;
    bool placed_l = false;
    g_hw->last_grasp_count = 0;
    g_hw->last_grasped_r = false;
    g_hw->last_grasped_l = false;

    const auto both_placed = [&]() -> bool {
        const bool right_ok = !allow_right || right_arm_motors_locked() || placed_r;
        const bool left_ok = !allow_left || placed_l;
        return right_ok && left_ok && (placed_r || placed_l);
    };

    std::cout << "[tray2] 开始循环：YOLO class3 空孔，顺序同 grasp；到位后张爪"
              << (right_arm_motors_locked() ? "（右臂锁定，只要求左手放置）" : "")
              << (!allow_right ? "；右手不放" : "")
              << (!allow_left ? "；左手不放" : "")
              << "\n";

    bool saw_zone = false;
    int last_rc = 1;
    for (int round = 1; round <= kMaxRounds; ++round)
    {
        if (should_abort())
        {
            g_hw->last_grasp_count = placed_count;
            g_hw->last_grasp_status = -4;
            std::cout << "[tray2] 循环中止 已放置 右=" << (placed_r ? 1 : 0)
                      << " 左=" << (placed_l ? 1 : 0) << "\n";
            return -4;
        }

        if (both_placed())
        {
            last_rc = 0;
            std::cout << "[tray2] 双手已放置，结束放置循环\n";
            break;
        }

        std::cout << "\n======== [tray2] 第 " << round << " 轮 已放 右="
                  << (placed_r ? 1 : 0) << " 左=" << (placed_l ? 1 : 0)
                  << " ========\n";
        move_box::GraspToStandbyResult r = move_box::run_grasp_to_standby(
            *g_hw->g,
            *g_hw->arm_r,
            *g_hw->arm_l,
            *g_hw->waist,
            *g_hw->layer3,
            g_hw->pipeline->cameras(),
            g_hw->pipeline->bridge(),
            cam2robot,
            err,
            should_abort,
            SegEngineId::Default,
            false,
            placed_r || !allow_right,
            placed_l || !allow_left);

        if (r.zone_count_ok)
        {
            int remain_r = r.zone_right - (r.grasped_r ? 1 : 0);
            int remain_l = r.zone_left - (r.grasped_l ? 1 : 0);
            if (remain_r < 0)
                remain_r = 0;
            if (remain_l < 0)
                remain_l = 0;
            saw_zone = true;
            g_unload_empty_known = true;
            g_unload_empty_right = remain_r;
            g_unload_empty_left = remain_l;
            std::cout << "[tray2] 本轮选孔照片计剩余空孔 右=" << remain_r
                      << " 左=" << remain_l << "\n";
        }

        placed_r = placed_r || r.grasped_r;
        placed_l = placed_l || r.grasped_l;
        placed_count = (placed_r ? 1 : 0) + (placed_l ? 1 : 0);
        g_hw->last_grasped_r = placed_r;
        g_hw->last_grasped_l = placed_l;
        g_hw->last_grasp_status = static_cast<int>(r.status);

        if (r.status == move_box::GraspToStandbyStatus::Aborted)
        {
            g_hw->last_grasp_count = placed_count;
            std::cout << "[tray2] 循环中止 已放置 右=" << (placed_r ? 1 : 0)
                      << " 左=" << (placed_l ? 1 : 0) << "\n";
            return -4;
        }

        if (r.status == move_box::GraspToStandbyStatus::Ok ||
            r.grasped_r || r.grasped_l)
        {
            none_streak = 0;
            std::cout << "[tray2] 第 " << round << " 轮完成 本轮新放"
                      << " 右=" << (r.grasped_r ? 1 : 0)
                      << " 左=" << (r.grasped_l ? 1 : 0)
                      << " 现已放 右=" << (placed_r ? 1 : 0)
                      << " 左=" << (placed_l ? 1 : 0);
            if (both_placed())
            {
                std::cout << "，双手已齐\n";
                last_rc = 0;
                break;
            }
            std::cout << "，未放侧继续\n";
            last_rc = 0;
            continue;
        }

        if (r.status == move_box::GraspToStandbyStatus::NoTarget)
        {
            if (both_placed())
            {
                last_rc = 0;
                break;
            }
            std::cout << "[tray2] 工作区已无空孔，仍缺放"
                      << " 右=" << (placed_r ? 1 : 0)
                      << " 左=" << (placed_l ? 1 : 0) << "，结束循环\n";
            last_rc = (placed_r || placed_l) ? 2 : 1;
            break;
        }

        ++none_streak;
        std::cerr << "[tray2] 第 " << round << " 轮未放到 code="
                  << static_cast<int>(r.status) << " 连续失败=" << none_streak
                  << " 已放 右=" << (placed_r ? 1 : 0)
                  << " 左=" << (placed_l ? 1 : 0) << "\n";
        if (none_streak >= kMaxNoneStreak)
        {
            std::cerr << "[tray2] 连续 " << none_streak << " 轮未放到，退出循环\n";
            last_rc = both_placed() ? 0 : 2;
            break;
        }
        last_rc = 2;
    }

    g_hw->last_grasp_count = placed_count;
    g_hw->last_grasp_status = last_rc;
    std::cout << "\n[tray2] 循环结束 已放 右=" << (placed_r ? 1 : 0)
              << " 左=" << (placed_l ? 1 : 0)
              << " 共 " << placed_count << " 件\n";
    std::cout.flush();

    if (should_abort())
        return -4;
    if (placed_r || placed_l)
    {
        if (!return_to_tray)
        {
            if (!saw_zone)
                g_unload_empty_known = false;
            g_dispatch_stay_at_tray2 = true;
            std::cout << "[tray2] 放完停在 " << g_move_cfg.chassis.tray2_station
                      << "，不升腰，不回 " << g_move_cfg.chassis.tray_station << std::endl;
            return last_rc;
        }
        g_dispatch_stay_at_tray2 = false;
        const std::string next = g_move_cfg.chassis.tray_station;
        std::cout << "[tray2] 收手已结束，头和腰回初始，同时底盘去 " << next << std::endl;
        int head_rc = 0;
        int waist_rc = 0;
        int chassis_rc = 0;
        {
            ChassisTravel travel(next);
            head_rc = enable_head_calib_pose();
            if (head_rc == 0)
                waist_rc = home_waist_upright();
            chassis_rc = travel.join();
        }
        if (head_rc != 0)
            return head_rc;
        if (waist_rc != 0)
            return waist_rc;
        if (chassis_rc != 0)
            return chassis_rc;
        if (!saw_zone)
            g_unload_empty_known = false;
        std::cout << "[tray2] 腰和底盘已结束，已在 " << next << std::endl;
        return last_rc;
    }

    std::cout << "[tray2] 本轮没有放下，仍按原顺序站起\n";
    const int home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
    if (home_rc != 0)
        return home_rc;
    {
        const move_box::TrayZoneCount left = move_box::count_tray_zone_holes(
            g_hw->pipeline->cameras(), g_hw->pipeline->bridge(), cam2robot);
        if (left.ok)
        {
            g_unload_empty_known = true;
            g_unload_empty_right = left.right;
            g_unload_empty_left = left.left;
            std::cout << "[tray2] 放下后空孔 右=" << left.right
                      << " 左=" << left.left << "\n";
        }
        else
        {
            g_unload_empty_known = false;
            std::cerr << "[tray2] 放下后没数清空孔: " << left.message << "\n";
            if (left.message == "取帧失败" || left.message == "无相机")
                g_hw->last_tray_message = left.message;
        }
    }
    return last_rc;
}

int vision_place_tray2_precision()
{
    if (g_hw == nullptr || g_hw->g == nullptr || g_hw->pipeline == nullptr)
        return -1;

    std::array<double, 16> cam2robot{};
    std::string err;
    if (!load_head_cam2robot(cam2robot, err) &&
        !load_cam2robot_matrix(default_camera_to_robot_yaml_path(), cam2robot, err))
    {
        std::cerr << "[orch] 读 cam2robot 失败: " << err << std::endl;
        return -3;
    }

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };

    struct TrayTaskGuard
    {
        ~TrayTaskGuard()
        {
            move_box::set_tray_hole_task(move_box::TrayHoleTask::GraspParts);
            move_box::set_tray2_precision_test(false);
        }
    } task_guard;
    move_box::set_tray_hole_task(move_box::TrayHoleTask::PlaceEmpty);
    move_box::set_tray2_precision_test(true);

    std::cout << "[tray2test] 精度测试：到 " << g_move_cfg.chassis.tray2_station
              << "，先合爪，不松爪，不起身。先前三排（from_robot 1-"
              << g_move_cfg.grasp_zone.far_row_max << "）再后三排\n";
    const int tray_rc = chassis_goto_station(g_move_cfg.chassis.tray2_station);
    if (tray_rc != 0)
        return tray_rc;
    if (should_abort())
        return -4;
    hardware_abort_sleep_ms(500);
    if (should_abort())
        return -4;

    std::cout << "[tray2test] 夹爪闭合（精度测试不放开工件）\n";
    if (!right_arm_motors_locked())
        g_hw->g->graspAndWait(gripper::Side::Right);
    g_hw->g->graspAndWait(gripper::Side::Left);
    if (should_abort())
        return -4;

    const int head_rc = enable_head_calib_pose();
    if (head_rc != 0)
        return head_rc;

    if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l, -1, true))
    {
        std::cerr << "[tray2test] 进入时回到 standby 失败\n";
        return should_abort() ? -4 : -5;
    }
    if (g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return -1;
    if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
        return should_abort() ? -4 : -5;
    move_box::waist_layer3_set_synced(true);

    constexpr int kMaxRounds = 40;
    int placed_count = 0;
    g_hw->last_grasp_count = 0;
    g_hw->last_grasped_r = false;
    g_hw->last_grasped_l = false;

    std::cout << "[tray2test] 开始循环放置，腰保持下蹲\n";
    int last_rc = 1;
    for (int round = 1; round <= kMaxRounds; ++round)
    {
        if (should_abort())
            return -4;
        std::cout << "\n======== [tray2test] 第 " << round << " 次 ========\n";
        move_box::GraspToStandbyResult r = move_box::run_grasp_to_standby(
            *g_hw->g,
            *g_hw->arm_r,
            *g_hw->arm_l,
            *g_hw->waist,
            *g_hw->layer3,
            g_hw->pipeline->cameras(),
            g_hw->pipeline->bridge(),
            cam2robot,
            err,
            should_abort,
            SegEngineId::Default,
            false,
            false,
            false);
        g_hw->last_grasp_status = static_cast<int>(r.status);
        if (r.status == move_box::GraspToStandbyStatus::Aborted)
            return -4;
        if (r.status == move_box::GraspToStandbyStatus::NoTarget)
        {
            std::cout << "[tray2test] 没有更多空孔，停在蹲姿，不站起。已走 "
                      << placed_count << " 次\n";
            last_rc = placed_count > 0 ? 0 : 1;
            break;
        }
        if (r.status != move_box::GraspToStandbyStatus::Ok && !r.grasped_r && !r.grasped_l)
        {
            std::cerr << "[tray2test] 本次未放到 code=" << static_cast<int>(r.status)
                      << "，停在蹲姿\n";
            last_rc = 2;
            break;
        }
        ++placed_count;
        g_hw->last_grasp_count = placed_count;
        g_hw->last_grasped_r = r.grasped_r;
        g_hw->last_grasped_l = r.grasped_l;
        last_rc = 0;
    }
    std::cout << "[tray2test] 结束，不起身。次数=" << placed_count << "\n";
    return last_rc;
}

int vision_belt2_then_tray2()
{
    std::cout << "[orch] cycle3：" << g_move_cfg.chassis.out_station
              << " 抓取 → " << g_move_cfg.chassis.tray2_station
              << " 放置，放完停在原地，不升腰、不回 "
              << g_move_cfg.chassis.tray_station << "\n";
    if (hardware_abort_requested())
        return -4;
    const int grasp_rc = go_belt_grasp(
        false, g_move_cfg.conveyor2, g_move_cfg.chassis.out_station, "belt2_grasp",
        -1, {}, true, g_move_cfg.chassis.tray2_station);
    if (grasp_rc != 0)
        return grasp_rc;
    if (hardware_abort_requested())
        return -4;
    if (g_hw == nullptr || (!g_hw->last_grasped_r && !g_hw->last_grasped_l))
    {
        std::cerr << "[orch] cycle3 没有抓到成品，不去料盘2\n";
        return grasp_rc == 0 ? 1 : grasp_rc;
    }
    std::cout << "[orch] cycle3 抓取结束，到 "
              << g_move_cfg.chassis.tray2_station << " 放置，放完不升腰\n";
    return vision_place_tray2(g_hw->last_grasped_r, g_hw->last_grasped_l, false);
}

int vision_grasp_then_belt()
{
    std::cout << "[orch] grasp_belt 闭环："
              << g_move_cfg.chassis.tray_station << " 抓 → home → "
              << g_move_cfg.chassis.belt_station << " 放置 → belt_grasp → "
              << g_move_cfg.chassis.out_station << " 放置 → belt2_grasp → "
              << g_move_cfg.chassis.tray2_station << " 空孔放置 → 回 "
              << g_move_cfg.chassis.tray_station << " 再抓，abort 才停\n";

    for (int cycle = 1;; ++cycle)
    {
        std::cout << "\n======== [orch] cycle 第 " << cycle << " 圈 ========\n";
        std::cout.flush();
        if (hardware_abort_requested())
            return -4;

        const int grasp_rc = vision_grasp_existing_pipeline();
        if (grasp_rc < 0)
            return grasp_rc;
        if (g_hw == nullptr || (!g_hw->last_grasped_r && !g_hw->last_grasped_l))
        {
            std::cerr << "[orch] grasp_belt 没有持件，不去 home / 底盘 / 放置\n";
            return grasp_rc == 0 ? 1 : grasp_rc;
        }
        if (hardware_abort_requested())
            return -4;

        if (g_grasp_departed)
            std::cout << "[orch] grasp_belt 收手时腰和底盘已经去 "
                      << g_move_cfg.chassis.belt_station << "\n";
        else
        {
            std::cout << "[orch] grasp_belt 抓完保持合爪，回 home\n";
            if (g_hw->g != nullptr)
                log_gripper_qs("grasp后回home前");
            const int home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
            if (home_rc != 0)
                return home_rc;
            if (hardware_abort_requested())
                return -4;

            const int ch_rc = chassis_goto_station(g_move_cfg.chassis.belt_station);
            if (ch_rc != 0)
                return ch_rc;
        }
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] grasp_belt 底盘已到 " << g_move_cfg.chassis.belt_station
                  << "，准备姿态后停稳 1.5s 再第一次拍照放置\n";
        const int belt_rc = go_belt_station();
        if (belt_rc != 0)
            return belt_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] grasp_belt 第一次放置结束（手臂在准备 tcp），不转腰直接抓取\n";
        const int bg_rc = go_belt_grasp(
            true, g_move_cfg.conveyor, g_move_cfg.chassis.belt_station, "belt_grasp",
            -1, {}, true, g_move_cfg.chassis.out_station);
        if (bg_rc != 0)
            return bg_rc;
        if (hardware_abort_requested())
            return -4;

        const std::string &out = g_move_cfg.chassis.out_station;
        if (out.empty())
        {
            std::cout << "[orch] grasp_belt 未配置 out_station，不再二次放置\n";
            return 0;
        }

        std::cout << "[orch] grasp_belt 抓取结束，确认底盘到 " << out
                  << "（右手抓到 2s 后已出发）\n";
        const int out_rc = chassis_goto_station(out);
        if (out_rc != 0)
            return out_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] grasp_belt 底盘已到 " << out
                  << "，准备姿态后停稳 1.5s 再第二次拍照放置\n";
        const int belt2_rc = go_belt_station(g_move_cfg.conveyor2, "belt2");
        if (belt2_rc != 0)
            return belt2_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] grasp_belt 第二次放置结束（手臂在准备 tcp），不转腰直接抓取\n";
        const int bg2_rc = go_belt_grasp(
            true, g_move_cfg.conveyor2, g_move_cfg.chassis.out_station, "belt2_grasp",
            -1, {}, true, g_move_cfg.chassis.tray2_station);
        if (bg2_rc != 0)
            return bg2_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] grasp_belt 第二传送带抓完，确认底盘到料盘2 "
                  << g_move_cfg.chassis.tray2_station
                  << "（右手抓到 2s 后已出发）\n";
        const int tray2_rc = vision_place_tray2();
        if (tray2_rc < 0)
            return tray2_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] grasp_belt 料盘2放完，底盘回 "
                  << g_move_cfg.chassis.tray_station << " 再抓\n";
        const int back_rc = chassis_goto_station(g_move_cfg.chassis.tray_station);
        if (back_rc != 0)
            return back_rc;
        if (hardware_abort_requested())
            return -4;
        std::cout << "[orch] grasp_belt 已回 " << g_move_cfg.chassis.tray_station
                  << "，进入下一圈\n";
        std::cout.flush();
    }
}

void prepare_next_grasp_upper()
{
    std::cout << "[grasp] 回料盘途中，站名已变，提前摆头、手臂和腰\n" << std::flush;
    enable_head_calib_pose();
    if (g_hw != nullptr && g_hw->arm_r != nullptr && g_hw->arm_l != nullptr)
        move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l);
    if (g_hw != nullptr && g_hw->waist != nullptr && g_hw->layer3 != nullptr)
        move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3);
}

int vision_grasp_belt1()
{
    std::cout << "[orch] grasp_belt1 循环："
              << g_move_cfg.chassis.tray_station << " 抓 → home → "
              << g_move_cfg.chassis.belt_station
              << " 只放置 → 回 " << g_move_cfg.chassis.tray_station
              << " 再抓。不去第二传送带、不在传送带上抓、不去料盘2。abort 才停\n";

    for (int cycle = 1;; ++cycle)
    {
        std::cout << "\n======== [orch] belt1 第 " << cycle << " 圈 ========\n";
        std::cout.flush();
        if (hardware_abort_requested())
            return -4;

        const int grasp_rc = vision_grasp_existing_pipeline();
        if (grasp_rc < 0)
            return grasp_rc;
        if (g_hw == nullptr || (!g_hw->last_grasped_r && !g_hw->last_grasped_l))
        {
            std::cerr << "[orch] grasp_belt1 没有持件，不去放置\n";
            return grasp_rc == 0 ? 1 : grasp_rc;
        }
        if (hardware_abort_requested())
            return -4;

        if (g_grasp_departed)
            std::cout << "[orch] grasp_belt1 收手时腰和底盘已经去 "
                      << g_move_cfg.chassis.belt_station << "\n";
        else
        {
            std::cout << "[orch] grasp_belt1 抓完保持合爪，回 home 再去一号传送带\n";
            if (g_hw->g != nullptr)
                log_gripper_qs("grasp_belt1回home前");
            const int home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
            if (home_rc != 0)
                return home_rc;
            if (hardware_abort_requested())
                return -4;

            const int ch_rc = chassis_goto_station(g_move_cfg.chassis.belt_station);
            if (ch_rc != 0)
                return ch_rc;
        }
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] grasp_belt1 底盘已到 " << g_move_cfg.chassis.belt_station
                  << "，只放置，不抓取\n";
        const int belt_rc = go_belt_station();
        if (belt_rc != 0)
            return belt_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] grasp_belt1 一号传送带放完，底盘回 "
                  << g_move_cfg.chassis.tray_station
                  << "，站名一变就提前摆腰和手臂\n";
        const int back_rc = chassis_goto_station(
            g_move_cfg.chassis.tray_station, false, []() { prepare_next_grasp_upper(); });
        if (back_rc != 0)
            return back_rc;
        if (hardware_abort_requested())
            return -4;
        std::cout << "[orch] grasp_belt1 已回 " << g_move_cfg.chassis.tray_station
                  << "，进入下一圈\n";
        std::cout.flush();
    }
}

namespace
{

Eigen::Matrix<double, 4, 4> pose16_to_T(const std::array<double, 16> &p)
{
    Eigen::Matrix<double, 4, 4> T;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r, c) = p[static_cast<size_t>(r * 4 + c)];
    return T;
}

ArucoHit make_aruco_hit(
    const char *slot_label,
    const ArucoMarkerResult &m,
    const std::array<double, 16> *cam2robot)
{
    constexpr double kRad2Deg = 180.0 / M_PI;
    ArucoHit hit;
    hit.slot = slot_label ? slot_label : "";
    hit.id = m.marker_id;
    hit.side_m = m.side_m;
    hit.reproj_px = m.reproj_px;
    hit.cam_x = m.t_m[0];
    hit.cam_y = m.t_m[1];
    hit.cam_z = m.t_m[2];
    hit.cam_yaw = m.rpy_deg[0];
    hit.cam_pitch = m.rpy_deg[1];
    hit.cam_roll = m.rpy_deg[2];
    if (cam2robot != nullptr)
    {
        const std::array<double, 16> pose_robot =
            transform_pose_cam_to_robot(*cam2robot, m.pose_4x4);
        const Eigen::Matrix<double, 1, 6> row = T2PosEulerAngles(pose16_to_T(pose_robot));
        hit.robot_ok = true;
        hit.pose_robot_4x4 = pose_robot;
        hit.robot_x = row(0);
        hit.robot_y = row(1);
        hit.robot_z = row(2);
        hit.robot_rx_deg = row(3) * kRad2Deg;
        hit.robot_ry_deg = row(4) * kRad2Deg;
        hit.robot_rz_deg = row(5) * kRad2Deg;
    }
    return hit;
}

void print_aruco_hit(const ArucoHit &h)
{
    std::cout << "  ID=" << h.id << " L=" << std::fixed << std::setprecision(0) << (h.side_m * 1000.0)
              << "mm reproj=" << std::setprecision(2) << h.reproj_px << "px\n"
              << "    cam t(m)=(" << std::setprecision(4) << h.cam_x << "," << h.cam_y << ","
              << h.cam_z << ") rpy(deg)=(" << std::setprecision(1) << h.cam_yaw << ","
              << h.cam_pitch << "," << h.cam_roll << ")\n";
    if (h.robot_ok)
    {
        std::cout << "    robot (m,deg)=(" << std::setprecision(4) << h.robot_x << "," << h.robot_y
                  << "," << h.robot_z << "," << std::setprecision(1) << h.robot_rx_deg << ","
                  << h.robot_ry_deg << "," << h.robot_rz_deg << ")\n";
    }
    std::cout << std::defaultfloat << std::setprecision(6);
}

int detect_head_aruco(RealSenseMultiCam &cameras, SegPoseBridge &bridge)
{
    constexpr CameraSlot slot = CameraSlot::Head;
    const char *slot_label = RealSenseMultiCam::slot_name(slot);
    CameraFrameData frame = cameras.grab(slot);
    if (!frame.ok)
    {
        std::cerr << "[aruco] " << slot_label << " 取帧失败: " << frame.message << "\n";
        g_hw->last_aruco_head = 0;
        return 0;
    }
    frame = RealSenseMultiCam::prepare_frame_for_slot(std::move(frame), slot);

    const ArucoDetectResult det = bridge.run_aruco(frame, kDebugVisualize, slot);
    if (!det.ok)
    {
        std::cerr << "[aruco] " << slot_label << " 失败: " << det.message << "\n";
        g_hw->last_aruco_head = 0;
        return -2;
    }

    std::array<double, 16> cam2robot{};
    std::string err;
    const bool have_ext = load_head_cam2robot(cam2robot, err) ||
                          load_cam2robot_matrix(default_camera_to_robot_yaml_path(), cam2robot, err);
    if (!have_ext)
        std::cerr << "[aruco] " << slot_label << " 无 cam2robot: " << err << "\n";

    std::cout << "\n--- ArUco 头相机 (" << det.markers.size() << " 个码) ---\n";
    if (det.markers.empty())
        std::cout << "(无)\n";
    for (const ArucoMarkerResult &m : det.markers)
    {
        ArucoHit hit = make_aruco_hit(slot_label, m, have_ext ? &cam2robot : nullptr);
        print_aruco_hit(hit);
        g_hw->last_aruco_hits.push_back(std::move(hit));
    }
    g_hw->last_aruco_head = static_cast<int>(det.markers.size());
    return 0;
}

} // namespace

int vision_aruco_detect_pipeline()
{
    if (g_hw == nullptr || g_hw->pipeline == nullptr)
        return -1;

    g_hw->last_aruco_head = 0;
    g_hw->last_aruco_status = 1;
    g_hw->last_aruco_hits.clear();

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };
    if (should_abort())
        return -4;

    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.aruco_engine_ready())
    {
        std::cerr << "[orch] ArUco 引擎未加载（检查 online_pose/config.yaml）\n";
        return -5;
    }

    RealSenseMultiCam &cameras = g_hw->pipeline->cameras();
    std::cout << "[orch] 二维码位姿：ArUco 按码 ID 查边长 + 相机内参 PnP（不抓取）\n";

    if (kDebugVisualize)
        pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});
    const int rc = detect_head_aruco(cameras, bridge);
    if (rc < 0)
        return rc;
    if (should_abort())
        return -4;

    std::cout << "[debug] 头相机 ArUco 完成，检测到 "
              << g_hw->last_aruco_head << " 个码\n";
    g_hw->last_aruco_status = (g_hw->last_aruco_head > 0) ? 0 : 1;
    return g_hw->last_aruco_status;
}

int vision_tray_holes_pipeline()
{
    if (g_hw == nullptr || g_hw->pipeline == nullptr)
        return -1;

    g_hw->last_tray_ok = false;
    g_hw->last_tray_message.clear();
    g_hw->last_tray_save_path.clear();
    g_hw->last_tray_reproj_px = -1.0;
    g_hw->last_tray_tilt_deg = std::numeric_limits<double>::quiet_NaN();
    g_hw->last_tray_used_ids.clear();
    g_hw->last_tray_holes.clear();

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };
    if (should_abort())
        return -4;

    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.tray_engine_ready())
    {
        std::cerr << "[orch] 料盘引擎未加载（检查 board_yf100_aruco/config.yaml）\n";
        g_hw->last_tray_message = "料盘引擎未加载";
        return -5;
    }

    RealSenseMultiCam &cameras = g_hw->pipeline->cameras();
    std::cout << "[orch] 料盘孔位：ArUco 孔 XY + YOLO 类别 + 头测顶面 z（不动臂）\n";

    constexpr CameraSlot slot = CameraSlot::Head;
    const int fuse_n = std::max(1, g_move_cfg.tray.fuse_frames);
    cameras.flush(slot);
    std::vector<CameraFrameData> frames;
    frames.reserve(static_cast<size_t>(fuse_n));
    for (int i = 0; i < fuse_n; ++i)
    {
        CameraFrameData one = cameras.grab_wait(slot);
        one = RealSenseMultiCam::prepare_frame_for_slot(std::move(one), slot);
        if (one.ok)
            frames.push_back(std::move(one));
    }
    if (frames.empty())
    {
        std::cerr << "[tray] 头相机取帧失败: 多帧融合没有有效帧\n";
        g_hw->last_tray_message = "取帧失败";
        return -2;
    }
    const CameraFrameData &frame = frames.back();

    PoseDetectionRecords yolo;
    const int algorithm_id = algorithm_id_for_slot(slot);
    const PoseRunResult pose = bridge.run(frame, algorithm_id, false, slot, -1);
    if (!pose.ok)
    {
        std::cerr << "[tray] YOLO 失败: " << pose.message << "（仍解料盘孔位）\n";
    }
    else
    {
        for (const PoseTargetResult &t : pose.targets)
        {
            if (!t.success)
                continue;
            PoseDetectionRecord rec;
            rec.slot = slot;
            rec.slot_name = "head";
            rec.frame_algorithm_id = algorithm_id;
            rec.target = t;
            yolo.push_back(std::move(rec));
        }
        std::cout << "[tray] YOLO 目标 " << yolo.size() << " 个（全部类别）\n";
    }

    if (should_abort())
        return -4;

    if (kDebugVisualize)
        pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});

    const TrayDetectResult det =
        bridge.run_tray_annotate_multiframe(frames, yolo, kDebugVisualize, slot);
    g_hw->last_tray_ok = det.ok;
    g_hw->last_tray_message = det.message;
    g_hw->last_tray_save_path = det.save_path;
    g_hw->last_tray_reproj_px = det.reproj_px;
    g_hw->last_tray_tilt_deg = det.tilt_deg;
    g_hw->last_tray_used_ids = det.used_ids;
    g_hw->last_tray_holes = det.holes;

    if (!det.ok)
    {
        std::cerr << "[tray] 失败: " << det.message << "\n";
        if (!det.save_path.empty())
            std::cout << "[tray] 标注图: " << det.save_path << "\n";
        return 1;
    }

    std::cout << "[debug] 料盘孔位完成，码 " << det.used_ids.size() << "/5  孔 "
              << det.holes.size() << "  图 " << det.save_path << "\n";
    return 0;
}

namespace
{

constexpr int kDispatchApplyTimeoutMs = 5000;
constexpr int kDispatchHandSkipped = 8;
/** 手里还有件，但这次没放成。不报完成，也不回料盘再抓。 */
constexpr int kDispatchHoldStay = 9;

int dispatch_back_to_tray(bool watch_obstacle = false)
{
    g_dispatch_nav_fault.clear();
    g_photo_ready_station.clear();
    std::cout << "[dispatch] 回 " << g_move_cfg.chassis.tray_station
              << "，手臂和腰回 home 与底盘同时动\n";
    int home_rc = 0;
    int ch_rc = 0;
    {
        ChassisTravel travel(g_move_cfg.chassis.tray_station, watch_obstacle);
        home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
        ch_rc = travel.join();
    }
    if (home_rc != 0)
        return home_rc;
    return ch_rc;
}

bool dispatch_report_state(RobotClient &client, const std::string &state)
{
    std::cout << "[dispatch] 上报状态 " << state << "\n";
    if (!client.report_state(state))
    {
        std::cerr << "[dispatch] 上报状态 " << state << " 失败\n";
        return false;
    }
    return true;
}

int dispatch_wait_system(RobotClient &client, const std::string &state)
{
    std::cerr << "[dispatch] 异常 " << state << "，停止动作，保持连接等待系统处理\n";
    dispatch_report_state(client, state);
    while (!hardware_abort_requested())
        hardware_abort_sleep_ms(500);
    return -4;
}

std::string dispatch_exception_state()
{
    if (!g_dispatch_nav_fault.empty())
        return g_dispatch_nav_fault;
    if (can_io_faulted())
    {
        const std::string msg = can_io_fault_message();
        if (msg.find("使能") != std::string::npos)
            return dispatch_msg::ERR_MOTOR_DISABLE;
        return dispatch_msg::ERR_MOTOR_LOST;
    }
    if (g_hw != nullptr && g_hw->g != nullptr)
    {
        const gripper::SideFeedback right = g_hw->g->feedback(gripper::Side::Right);
        const gripper::SideFeedback left = g_hw->g->feedback(gripper::Side::Left);
        if ((right.have_feedback && !right.io_ok) || (left.have_feedback && !left.io_ok))
            return dispatch_msg::ERR_GRIPPER_LOST;
    }
    if (g_hw != nullptr)
    {
        const std::string &belt = g_hw->last_belt_message;
        const std::string &tray = g_hw->last_tray_message;
        if (belt == "无相机" || belt == "取帧失败" || tray == "取帧失败")
            return dispatch_msg::ERR_CAMERA;
    }
    return {};
}

enum class DispatchNote
{
    PlaceHand,
    PickHand,
    Mission
};

int dispatch_report(RobotClient &client, DispatchNote note, const char *what)
{
    const char *state = dispatch_msg::DONE;
    if (note == DispatchNote::PlaceHand)
        state = dispatch_msg::PLACE_DONE;
    else if (note == DispatchNote::PickHand)
        state = dispatch_msg::PICK_UP_DONE;
    std::cout << "[dispatch] " << what << " 完成，停 2s 后发送 " << state << "\n";
    hardware_abort_sleep_ms(2000);
    if (hardware_abort_requested())
        return -4;
    const bool ok = note == DispatchNote::PlaceHand ? client.place_done()
                    : note == DispatchNote::PickHand ? client.pick_up_done()
                                                     : client.mission_done();
    if (!ok)
    {
        std::cerr << "[dispatch] 发送 " << state << " 失败\n";
        return -7;
    }
    std::cout << "[dispatch] 已发送 " << state << "\n";
    return 0;
}

int dispatch_report_after(
    RobotClient &client, DispatchNote note, const char *what,
    std::chrono::steady_clock::time_point since)
{
    const char *state = note == DispatchNote::PickHand ? dispatch_msg::PICK_UP_DONE
                                                       : dispatch_msg::DONE;
    const auto spent = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - since)
                           .count();
    const int remain = static_cast<int>(std::max<int64_t>(0, 2000 - spent));
    std::cout << "[dispatch] " << what << " 距合爪已 " << spent
              << " ms，再停 " << remain << " ms 后发送 " << state << "\n";
    if (remain > 0)
        hardware_abort_sleep_ms(remain);
    if (hardware_abort_requested())
        return -4;
    const bool ok = note == DispatchNote::PickHand ? client.pick_up_done() : client.mission_done();
    if (!ok)
    {
        std::cerr << "[dispatch] 发送 " << state << " 失败\n";
        return -7;
    }
    std::cout << "[dispatch] 已发送 " << state << "\n";
    return 0;
}

void dispatch_send(RobotClient &client, const char *name, bool sent)
{
    std::cout << "[dispatch] 发送 " << name << "\n";
    if (!sent)
        std::cerr << "[dispatch] 发送 " << name << " 失败，继续当前流程\n";
    else
        std::cout << "[dispatch] 已发送 " << name << "\n";
}

void plan_unload_hands(bool &pick_left, bool &pick_right)
{
    pick_left = true;
    pick_right = true;
    if (!g_unload_empty_known)
    {
        std::cout << "[dispatch] 还没有空孔记录，下料抓两只手\n";
        return;
    }
    const int total = g_unload_empty_right + g_unload_empty_left;
    std::cout << "[dispatch] 上次放下后空孔 右=" << g_unload_empty_right
              << " 左=" << g_unload_empty_left << "\n";
    if (total >= 2)
    {
        std::cout << "[dispatch] 空孔不少于 2 个，这次抓两只手\n";
        return;
    }
    if (total == 1)
    {
        pick_left = g_unload_empty_left > 0;
        pick_right = g_unload_empty_right > 0;
        std::cout << "[dispatch] 空孔只剩 1 个，这次只抓"
                  << (pick_left ? "左手" : "右手") << "\n";
        return;
    }
    std::cout << "[dispatch] 上次记录料盘已满，这次按新空盘抓两只手\n";
}

int dispatch_pick_one(
    RobotClient &client, bool (*apply_pick)(RobotClient &, int),
    const std::string &request_state, const std::string &grant_state,
    bool from_place, const ConveyorStationConfig &c, const std::string &station,
    const char *tag, int only_hand, const char *done_what,
    const std::function<void()> &on_closed = {},
    bool photo_settle = true,
    bool grant_done = false,
    bool grant_ok = false,
    const std::string &depart_after_right = {})
{
    if (grant_done && !grant_ok)
    {
        std::cout << "[dispatch] " << done_what << " 申请已在上一只手收手时被拒绝，跳过这只手\n";
        return kDispatchHandSkipped;
    }
    if (!grant_done)
    {
        std::cout << "[dispatch] 发送 " << request_state << "，等待 " << grant_state
                  << "，超时 " << kDispatchApplyTimeoutMs << " ms\n";
        if (!apply_pick(client, kDispatchApplyTimeoutMs))
        {
            std::cout << "[dispatch] " << done_what << " 申请 5s 未允许，跳过这只手，继续子任务\n";
            return kDispatchHandSkipped;
        }
        std::cout << "[dispatch] 已收到 " << grant_state << "，" << done_what << " 放行\n";
    }
    else
        std::cout << "[dispatch] " << done_what << " 申请已在上一只手收手期间完成\n";
    return go_belt_grasp(from_place, c, station, tag, only_hand, [&](bool) {
        const auto closed_at = std::chrono::steady_clock::now();
        if (on_closed)
            on_closed();
        return dispatch_report_after(client, DispatchNote::PickHand, done_what, closed_at);
    }, photo_settle, only_hand == 1 ? depart_after_right : std::string());
}

int dispatch_place_belt(
    RobotClient &client, bool (*apply_place)(RobotClient &, int),
    const char *request_state, const char *grant_state,
    const ConveyorStationConfig &c, const char *tag,
    bool hold_right, bool hold_left, const char *place_what,
    bool skip_ready = false)
{
    if (!hold_right && !hold_left)
    {
        std::cout << "[dispatch] 手里没有件，跳过" << place_what << "\n";
        return 0;
    }
    std::cout << "[dispatch] 发送 " << request_state << "，等待 " << grant_state
              << "，超时 " << kDispatchApplyTimeoutMs << " ms\n";
    if (!apply_place(client, kDispatchApplyTimeoutMs))
    {
        std::cout << "[dispatch] " << place_what
                  << " 申请 5s 未允许，手里还有件，不放置，不报完成\n";
        return kDispatchHoldStay;
    }
    std::cout << "[dispatch] 已收到 " << grant_state << "，" << place_what << " 放行\n";
    const int place_rc = go_belt_station(c, tag, {}, hold_right, hold_left, skip_ready);
    if (place_rc != 0)
        return place_rc;
    return dispatch_report(client, DispatchNote::PlaceHand, place_what);
}

bool apply_pick_half(RobotClient &client, int timeout_ms)
{
    return client.apply_pick_up_half_done(timeout_ms);
}

bool apply_pick_well(RobotClient &client, int timeout_ms)
{
    return client.apply_pick_up_well_done(timeout_ms);
}

bool tcp_near_ready(const Eigen::Matrix<double, 1, 6> &cur, const Eigen::Matrix<double, 1, 6> &ready)
{
    constexpr double kXyzM = 0.012;
    constexpr double kRpyRad = 5.0 * M_PI / 180.0;
    if ((cur.head<3>() - ready.head<3>()).norm() > kXyzM)
        return false;
    return (cur.tail<3>() - ready.tail<3>()).cwiseAbs().maxCoeff() <= kRpyRad;
}

bool arms_at_belt_ready(const ConveyorStationConfig &c)
{
    if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
        return false;
    const bool right_ok = right_arm_motors_locked() ||
                          tcp_near_ready(arm_get_tcp_pos(*g_hw->arm_r), c.tcp.right);
    const bool left_ok = tcp_near_ready(arm_get_tcp_pos(*g_hw->arm_l), c.tcp.left);
    return right_ok && left_ok;
}

bool chassis_already_at(const std::string &station)
{
    return g_hw != nullptr && g_hw->last_chassis_ok &&
           station_name_matches(g_hw->last_chassis_station, station);
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
        return ready_rc;
    return ch_rc;
}

int dispatch_pick_pair(
    RobotClient &client, bool (*apply_pick)(RobotClient &, int),
    const std::string &request_state, const std::string &grant_state,
    const ConveyorStationConfig &c, const std::string &station, const char *tag,
    bool do_left, bool do_right, const char *left_what, const char *right_what,
    const std::string &depart_after_right = {})
{
    struct NextGrant
    {
        bool started = false;
        bool ok = false;
    } next;
    if (do_left)
    {
        const int left_rc = dispatch_pick_one(
            client, apply_pick, request_state, grant_state,
            true, c, station, tag, 0, left_what,
            [&]() {
                if (!do_right || next.started)
                    return;
                next.started = true;
                std::cout << "[dispatch] 左手已合爪，开始申请右手，与收手重叠\n";
                next.ok = apply_pick(client, kDispatchApplyTimeoutMs);
            });
        if (left_rc < 0)
            return left_rc;
    }
    if (do_right)
    {
        const bool known = next.started;
        const int right_rc = dispatch_pick_one(
            client, apply_pick, request_state, grant_state,
            true, c, station, tag, 1, right_what,
            {}, !known, known, next.ok, depart_after_right);
        if (right_rc < 0)
            return right_rc;
    }
    return 0;
}

int place_while_driving(
    RobotClient &client, bool (*apply_place)(RobotClient &, int),
    const char *request_state, const char *grant_state,
    const std::string &station, const ConveyorStationConfig &c, const char *tag,
    bool hold_right, bool hold_left, const char *place_what)
{
    if (!hold_right && !hold_left)
    {
        std::cout << "[dispatch] 手里没有件，跳过" << place_what << "\n";
        return 0;
    }
    std::atomic<bool> granted{false};
    std::thread apply_th([&]() {
        std::cout << "[dispatch] 去 " << station << " 的同时发送 " << request_state
                  << "，等待 " << grant_state << "\n";
        granted = apply_place(client, kDispatchApplyTimeoutMs);
    });
    int ready_rc = 0;
    int ch_rc = 0;
    {
        ChassisTravel travel(station);
        ready_rc = go_belt_ready(*g_hw->arm_r, *g_hw->arm_l, c, tag);
        ch_rc = travel.join();
    }
    if (apply_th.joinable())
        apply_th.join();
    if (ch_rc != 0)
        return ch_rc;
    if (ready_rc != 0)
        return ready_rc;
    if (!granted.load())
    {
        std::cout << "[dispatch] " << place_what
                  << " 申请 5s 未允许，手里还有件，不放置，不报完成\n";
        return kDispatchHoldStay;
    }
    std::cout << "[dispatch] 已收到 " << grant_state << "，" << place_what << " 放行\n";
    const int place_rc = go_belt_station(c, tag, {}, hold_right, hold_left, true);
    if (place_rc != 0)
        return place_rc;
    return dispatch_report(client, DispatchNote::PlaceHand, place_what);
}

int dispatch_load(RobotClient &client)
{
    std::cout << "[dispatch] 上料：AP7 抓件，抓完最后的零件就报料盘空；AP5 申请 5s，允许才放手里有件的手，然后发 "
              << dispatch_msg::DONE << "\n";
    const int grasp_rc = vision_grasp_existing_pipeline();
    if (grasp_rc < 0)
        return grasp_rc;
    const bool hold_r = g_hw != nullptr && g_hw->last_grasped_r;
    const bool hold_l = g_hw != nullptr && g_hw->last_grasped_l;
    const int took = (hold_r ? 1 : 0) + (hold_l ? 1 : 0);
    if (g_load_parts_seen == 0 || (g_load_parts_seen > 0 && took >= g_load_parts_seen))
    {
        std::cout << "[dispatch] 上料盘已经没有零件（开抓前 "
                  << g_load_parts_seen << " 件，这次拿走 " << took << " 件）\n";
        dispatch_send(client, dispatch_msg::RAW_MATERIAL_EMPTY, client.raw_material_empty());
    }
    if (!hold_r && !hold_l)
        return grasp_rc == 0 ? 1 : grasp_rc;
    if (!g_grasp_departed)
    {
        if (go_home(*g_hw->arm_r, *g_hw->arm_l) != 0)
            return -5;
        if (chassis_goto_station(g_move_cfg.chassis.belt_station) != 0)
            return -6;
    }
    else
        std::cout << "[dispatch] 上料抓取收手时腰已在拍照前伸，底盘已经去 "
                  << g_move_cfg.chassis.belt_station << "\n";
    const bool pose_ready = station_name_matches(
        g_photo_ready_station, g_move_cfg.chassis.belt_station);
    const int place_rc = dispatch_place_belt(
        client, [](RobotClient &c, int timeout_ms) { return c.apply_place_raw_material(timeout_ms); },
        dispatch_msg::APPLY_PLACE_RAW, dispatch_msg::PLACE,
        g_move_cfg.conveyor, "belt", hold_r, hold_l, "上料放置", pose_ready);
    if (place_rc != 0)
        return place_rc;
    const int done_rc = dispatch_report(client, DispatchNote::Mission, "上料");
    if (done_rc != 0)
        return done_rc;
    std::cout << "[dispatch] 上料完成，停在当前位置\n";
    return 0;
}

int dispatch_transfer(RobotClient &client)
{
    std::cout << "[dispatch] 转运：AP5 左右手各申请 5s，超时跳过该手；有件才去 AP6 放，然后发 "
              << dispatch_msg::DONE << "\n";
    if (!belt_ready_to_pick(g_move_cfg.chassis.belt_station, g_move_cfg.conveyor, "转运"))
    {
        const int ready_rc = drive_belt_ready(
            g_move_cfg.chassis.belt_station, g_move_cfg.conveyor, "dispatch_transfer");
        if (ready_rc != 0)
            return ready_rc;
    }
    g_hw->last_grasped_r = false;
    g_hw->last_grasped_l = false;
    ConveyorStationConfig transfer_grasp = g_move_cfg.conveyor;
    constexpr double kTransferGraspXBiasM = 0.005;
    transfer_grasp.grasp_offset_left.x += kTransferGraspXBiasM;
    transfer_grasp.grasp_offset_right.x += kTransferGraspXBiasM;
    std::cout << std::fixed << std::setprecision(4)
              << "[dispatch] 转运抓取在传送带 +X 再偏 " << kTransferGraspXBiasM
              << " m，左右手都加\n";
    const int pick_rc = dispatch_pick_pair(
        client, apply_pick_half, dispatch_msg::APPLY_PICK_HALF, dispatch_msg::PICK_HALF,
        transfer_grasp, g_move_cfg.chassis.belt_station, "belt_grasp",
        true, !right_arm_motors_locked(), "转运左手抓取", "转运右手抓取",
        g_move_cfg.chassis.out_station);
    if (pick_rc < 0)
        return pick_rc;
    const bool hold_r = g_hw->last_grasped_r;
    const bool hold_l = g_hw->last_grasped_l;
    ConveyorStationConfig transfer_place = g_move_cfg.conveyor2;
    constexpr double kTransferPlaceZBiasM = -0.005;
    transfer_place.place_base_z_bias_m = kTransferPlaceZBiasM;
    if (hold_r || hold_l)
        std::cout << std::fixed << std::setprecision(4)
                  << "[dispatch] 转运放置松手高度在基座 Z 再低 "
                  << -kTransferPlaceZBiasM << " m，左右手都加\n";
    const int place_rc = place_while_driving(
        client, [](RobotClient &c, int timeout_ms) { return c.apply_place_half_done(timeout_ms); },
        dispatch_msg::APPLY_PLACE_HALF, dispatch_msg::PLACE,
        g_move_cfg.chassis.out_station, transfer_place, "belt2",
        hold_r, hold_l, "转运放置");
    if (place_rc != 0)
        return place_rc;
    const int done_rc = dispatch_report(client, DispatchNote::Mission, "转运");
    if (done_rc != 0)
        return done_rc;
    std::cout << "[dispatch] 转运完成，停在当前位置\n";
    return 0;
}

int dispatch_unload(RobotClient &client)
{
    std::cout << "[dispatch] 下料：按上次剩下的空孔抓 1 或 2 只手，申请 5s 超时跳过该手；放到 AP9 后若没有空孔就报满盘，再发 "
              << dispatch_msg::DONE << "\n";
    if (!belt_ready_to_pick(g_move_cfg.chassis.out_station, g_move_cfg.conveyor2, "下料"))
    {
        const int ready_rc = drive_belt_ready(
            g_move_cfg.chassis.out_station, g_move_cfg.conveyor2, "dispatch_unload");
        if (ready_rc != 0)
            return ready_rc;
    }
    bool pick_left = true;
    bool pick_right = true;
    plan_unload_hands(pick_left, pick_right);
    if (right_arm_motors_locked())
        pick_right = false;
    g_hw->last_grasped_r = false;
    g_hw->last_grasped_l = false;
    const int pick_rc = dispatch_pick_pair(
        client, apply_pick_well, dispatch_msg::APPLY_PICK_WELL, dispatch_msg::PICK_WELL,
        g_move_cfg.conveyor2, g_move_cfg.chassis.out_station, "belt2_grasp",
        pick_left, pick_right, "下料左手抓取", "下料右手抓取",
        g_move_cfg.chassis.tray2_station);
    if (pick_rc < 0)
        return pick_rc;
    const bool hold_r = g_hw->last_grasped_r;
    const bool hold_l = g_hw->last_grasped_l;
    if (hold_r || hold_l)
    {
        const int place_rc = vision_place_tray2(hold_r, hold_l, false);
        if (place_rc < 0)
            return place_rc;
        if (g_unload_empty_known && g_unload_empty_right + g_unload_empty_left <= 0)
        {
            std::cout << "[dispatch] 下料盘已经没有空孔\n";
            dispatch_send(
                client, dispatch_msg::WELL_DONE_MATERIAL_FULL, client.well_done_material_full());
            g_unload_empty_known = false;
        }
    }
    else
        std::cout << "[dispatch] 这次没有抓到成品，不去 AP9 放\n";
    const int done_rc = dispatch_report(client, DispatchNote::Mission, "下料");
    if (done_rc != 0)
        return done_rc;
    std::cout << "[dispatch] 下料完成，停在当前位置\n";
    return 0;
}

struct DispatchInbox
{
    std::thread worker;
    std::mutex mu;
    std::string mission;
    bool finished = false;
    bool running = false;

    ~DispatchInbox()
    {
        if (worker.joinable())
            worker.join();
    }
};

void inbox_start(DispatchInbox &box, RobotClient &client)
{
    if (box.running)
        return;
    {
        std::lock_guard<std::mutex> lk(box.mu);
        box.mission.clear();
        box.finished = false;
    }
    box.running = true;
    box.worker = std::thread([&box, &client]() {
        std::string mission = client.apply_mission();
        std::lock_guard<std::mutex> lk(box.mu);
        box.mission = std::move(mission);
        box.finished = true;
    });
}

bool inbox_poll(DispatchInbox &box, std::string &out)
{
    std::lock_guard<std::mutex> lk(box.mu);
    if (!box.finished)
        return false;
    out = box.mission;
    return true;
}

void inbox_join(DispatchInbox &box)
{
    if (box.worker.joinable())
        box.worker.join();
    box.running = false;
}

bool inbox_wait(DispatchInbox &box, int timeout_ms, std::string &out)
{
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (inbox_poll(box, out) || hardware_abort_requested())
            return inbox_poll(box, out);
        hardware_abort_sleep_ms(100);
    }
    return inbox_poll(box, out);
}

} // namespace

int run_dispatch_mode()
{
    if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
        return -1;
    std::cout << "[dispatch] 系统对接模式，调度 "
              << dispatch_msg::kDefaultHost << ":" << dispatch_msg::kDefaultPort
              << "。先到 " << g_move_cfg.chassis.tray_station << " 待命\n";
    const int ready_rc = dispatch_back_to_tray();
    if (ready_rc != 0)
        return hardware_abort_requested() ? -4 : ready_rc;

    RobotClient client;
    DispatchInbox box;
    std::string mission;
    auto take_ready = [&]() -> bool {
        if (!inbox_poll(box, mission))
            return false;
        inbox_join(box);
        return true;
    };
    auto settle_at_tray_then_take = [&]() -> bool {
        std::cout << "[dispatch] 已到 " << g_move_cfg.chassis.tray_station
                  << "，停稳 2s。这 2s 只缓存任务，不执行\n";
        hardware_abort_sleep_ms(2000);
        if (hardware_abort_requested())
            return false;
        dispatch_report_state(client, dispatch_msg::STANDBY);
        if (take_ready() && !mission.empty())
        {
            std::cout << "[dispatch] 停稳后执行缓存任务 " << mission << "\n";
            return true;
        }
        std::cout << "[dispatch] 停稳后没有缓存任务，继续等待\n";
        return false;
    };
    auto return_tray = [&]() -> int {
        dispatch_report_state(client, dispatch_msg::WORKING);
        const int back = dispatch_back_to_tray(true);
        if (back != 0)
        {
            const std::string fault = dispatch_exception_state();
            if (!fault.empty())
                return dispatch_wait_system(client, fault);
            return hardware_abort_requested() ? -4 : (back < 0 ? back : -6);
        }
        if (hardware_abort_requested())
            return -4;
        settle_at_tray_then_take();
        return 0;
    };
    dispatch_report_state(client, dispatch_msg::STANDBY);
    std::string wait_station = g_move_cfg.chassis.tray_station;
    while (!hardware_abort_requested())
    {
        if (mission.empty())
        {
            std::cout << "[dispatch] 在 " << wait_station << " 等待任务\n";
            inbox_start(box, client);
            while (!take_ready())
            {
                if (hardware_abort_requested())
                    return -4;
                hardware_abort_sleep_ms(100);
            }
        }
        if (hardware_abort_requested())
            return -4;
        if (mission.empty())
        {
            std::cerr << "[dispatch] 调度断线或没有任务，2 秒后重试\n";
            hardware_abort_sleep_ms(2000);
            continue;
        }
        std::cout << "[dispatch] 收到任务 " << mission << "\n";
        const std::string current = mission;
        mission.clear();
        if (current != dispatch_msg::TASK_LOAD && current != dispatch_msg::TASK_TRANSFER &&
            current != dispatch_msg::TASK_UNLOAD)
        {
            std::cerr << "[dispatch] 未知任务 " << current << "，继续等待\n";
            continue;
        }
        g_hw->last_belt_message.clear();
        g_hw->last_tray_message.clear();
        g_dispatch_nav_fault.clear();
        g_dispatch_stay_at_tray2 = false;
        dispatch_report_state(client, dispatch_msg::WORKING);
        int rc = -1;
        if (current == dispatch_msg::TASK_LOAD)
            rc = dispatch_load(client);
        else if (current == dispatch_msg::TASK_TRANSFER)
            rc = dispatch_transfer(client);
        else
            rc = dispatch_unload(client);
        if (rc == -4 || hardware_abort_requested())
            return -4;
        if (rc != 0)
        {
            g_dispatch_stay_at_tray2 = false;
            const std::string fault = dispatch_exception_state();
            if (!fault.empty())
                return dispatch_wait_system(client, fault);
            const bool holding = g_hw != nullptr && (g_hw->last_grasped_r || g_hw->last_grasped_l);
            if (holding)
            {
                wait_station = g_hw->last_chassis_station.empty()
                                   ? wait_station
                                   : g_hw->last_chassis_station;
                std::cerr << "[dispatch] 任务 " << current << " 未完成 code=" << rc
                          << "，手里还有件，不报完成，不停在料盘上再抓。停在 "
                          << wait_station << " 等下一条任务\n";
                dispatch_report_state(client, dispatch_msg::STANDBY);
                mission.clear();
                continue;
            }
            std::cerr << "[dispatch] 任务 " << current << " 失败 code=" << rc
                      << "，不回报完成，回 AP7 再等待。途中收到的任务先缓存\n";
            inbox_start(box, client);
            const int back_rc = return_tray();
            if (back_rc != 0)
                return back_rc;
            continue;
        }

        dispatch_report_state(client, dispatch_msg::STANDBY);
        if (g_dispatch_stay_at_tray2)
        {
            g_dispatch_stay_at_tray2 = false;
            wait_station = g_move_cfg.chassis.tray2_station;
            std::cout << "[dispatch] 下料放完，停在 " << wait_station
                      << " 等待下一条任务，不回 " << g_move_cfg.chassis.tray_station << "\n";
            continue;
        }
        wait_station = g_move_cfg.chassis.tray_station;
        std::cout << "[dispatch] 子任务完成，原地等待 5s\n";
        inbox_start(box, client);
        std::string soon;
        if (inbox_wait(box, 5000, soon))
        {
            inbox_join(box);
            if (hardware_abort_requested())
                return -4;
            if (!soon.empty())
            {
                std::cout << "[dispatch] 5s 内收到 " << soon << "，原地执行，不回 AP7\n";
                mission = std::move(soon);
                continue;
            }
            std::cerr << "[dispatch] 等待期间调度断线，回 AP7 再连接\n";
        }
        else if (hardware_abort_requested())
        {
            return -4;
        }

        std::cout << "[dispatch] 5s 内没有新任务，回 " << g_move_cfg.chassis.tray_station
                  << "。途中收到的任务先缓存，到站停稳后再执行\n";
        wait_station = g_move_cfg.chassis.tray_station;
        if (!box.running)
            inbox_start(box, client);
        const int back_rc = return_tray();
        if (back_rc != 0)
            return back_rc;
    }
    return hardware_abort_requested() ? -4 : 0;
}

} // namespace orch
