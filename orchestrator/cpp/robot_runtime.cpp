#include "robot_runtime.h"

#include "dispatch_board.h"
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
#include <memory>
#include <sstream>
#include <string>
#include <sys/select.h>
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
constexpr const char *TASK_CHARGE = "CHARGE";
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
constexpr const char *GO_CHARGE = "GO_CHARGE";
/** AP14 准备好后，底盘出发去充电点途中。 */
constexpr const char *COMMUTING = "commuting";
constexpr const char *CHARGING = "CHARGING";
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
/** 料盘/传送带二维码解算失败（调度里会上报后重新申请并重试）。 */
constexpr int kVisionQrFail = 10;
/** 二维码解算失败后，最多再申请并重试几次（不含第一次）。 */
constexpr int kDispatchQrRetry = 3;
/** 本次上料开抓前，工作区里的毛坯数。-1 表示没数清。 */
int g_load_parts_seen = -1;
/** 抓取最后一次收手已经同时把腰站起、底盘开到传送带。调用方不必再排队走。 */
bool g_grasp_departed = false;
/** 料盘抓完、底盘刚出发时调用。调度上料用来提前申请放置。 */
std::function<void()> g_on_grasp_depart;
/** 上次下料放完后，工作区还剩的空孔。未知时下次仍抓两只手。 */
bool g_unload_empty_known = false;
/** 调度下料已在 AP9 放完，本轮完成后停在原地等任务，不回 AP7。 */
bool g_dispatch_stay_at_tray2 = false;
/** 腰和头已经在该站的传送带拍照位，紧接着的放置不必再摆一次。 */
std::string g_photo_ready_station;
/** 去料盘二路上已经降腰并收手到放置准备。vision_place_tray2 读一次就清。 */
bool g_tray2_place_prep_done = false;
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
/** 调度下发的上料料架。空闲返回时去这里，不再写死 AP7。 */
std::string g_dispatch_rack_point;
Seer &chassis_seer()
{
    static Seer robot(g_move_cfg.chassis.host);
    return robot;
}

bool ensure_chassis()
{
    Seer &robot = chassis_seer();
    if (robot.IsConnected())
        return true;
    static bool logged_fail = false;
    robot.SetHost(g_move_cfg.chassis.host);
    if (!robot.Connect())
    {
        if (!logged_fail)
        {
            logged_fail = true;
            std::cerr << "[orch] 底盘连接失败: "
                      << (robot.LastMessage().empty() ? "无应答" : robot.LastMessage()) << "\n";
        }
        return false;
    }
    logged_fail = false;
    std::cout << "[orch] 底盘已连接 " << g_move_cfg.chassis.host << "\n";
    return true;
}

std::string chassis_location_station()
{
    if (!ensure_chassis())
        return {};
    double x = 0, y = 0, angle = 0, confidence = 0;
    std::string cur, last;
    int loc_state = 0;
    bool in_forbidden = false;
    if (chassis_seer().GetLocation(
            x, y, angle, cur, last, confidence, loc_state, in_forbidden) != 0)
        return {};
    return cur;
}

void publish_dispatch_pose(double x, double y, double angle)
{
    RobotClient::x = static_cast<float>(x);
    RobotClient::y = static_cast<float>(y);
    RobotClient::angle = static_cast<float>(angle);
}

void publish_dispatch_battery(double level)
{
    if (!std::isfinite(level))
        return;
    double pct = level;
    if (pct <= 1.5)
        pct *= 100.0;
    pct = std::clamp(pct, 0.0, 100.0);
    RobotClient::power_percent = static_cast<int>(std::lround(pct));
}

void chassis_status_loop()
{
    while (true)
    {
        if (ensure_chassis())
        {
            Seer &robot = chassis_seer();
            double x = 0, y = 0, angle = 0, confidence = 0;
            std::string cur, last;
            int loc_state = 0;
            bool in_forbidden = false;
            if (robot.GetLocation(x, y, angle, cur, last, confidence, loc_state, in_forbidden) == 0)
                publish_dispatch_pose(x, y, angle);
            double level = 0, voltage = 0, current = 0, temp = 0;
            bool charging = false;
            int cycle = 0;
            if (robot.GetBattery(level, charging, voltage, current, temp, cycle) == 0)
                publish_dispatch_battery(level);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
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

    std::cout << "[orch] 底盘去 " << station << "\n";
    if (!ensure_chassis())
    {
        g_hw->last_chassis_message = chassis_seer().LastMessage().empty()
                                         ? "底盘连接失败"
                                         : chassis_seer().LastMessage();
        std::cerr << "[orch] 底盘连接失败: " << g_hw->last_chassis_message << "\n";
        return -6;
    }
    Seer &robot = chassis_seer();

    double x = 0, y = 0, angle = 0, confidence = 0;
    std::string cur, last;
    int loc_state = 0;
    bool in_forbidden = false;
    int loc_rc = robot.GetLocation(x, y, angle, cur, last, confidence, loc_state, in_forbidden);
    if (loc_rc == 0)
        publish_dispatch_pose(x, y, angle);
    if (loc_rc == 0 && station_name_matches(cur, station))
    {
        std::cout << "[orch] 底盘已在 " << cur << "，不再发导航\n";
        note_station_arrived();
        g_hw->last_chassis_ok = true;
        g_hw->last_chassis_station = cur;
        g_hw->last_chassis_message = "已在目标站";
        board_station(cur, {});
        return 0;
    }
    board_station(loc_rc == 0 ? cur : std::string(), station);

    const int go_rc = robot.GoToStation(station);
    if (go_rc != 0)
    {
        g_hw->last_chassis_message = robot.LastMessage().empty()
                                         ? ("GoToStation 失败 code=" + std::to_string(go_rc))
                                         : robot.LastMessage();
        std::cerr << "[orch] 底盘去 " << station << " 下发失败 code=" << go_rc
                  << " " << g_hw->last_chassis_message << "\n";
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
                board_station(target_id, {});
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
                board_error("底盘" + g_hw->last_chassis_message);
                return -6;
            }
        }
        hardware_abort_sleep_ms(300);
    }

    g_hw->last_chassis_message = "等待到位超时";
    std::cerr << "[orch] 底盘去 " << station << " 超时\n";
    board_error("底盘去 " + station + " 超时");
    robot.CancelNav();
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
std::thread g_tray2_waist_th;
std::atomic<int> g_tray2_waist_rc{0};
bool g_tray2_waist_started = false;

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
    });
}

void start_tray2_waist_after_right_arrived()
{
    if (g_tray2_waist_started || g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return;
    g_tray2_waist_started = true;
    g_tray2_waist_rc = 0;
    g_tray2_waist_th = std::thread([]() {
        std::cout << "[orch] 右手收手已到位，2s 后腰降到料盘放置高度\n" << std::flush;
        hardware_abort_sleep_ms(2000);
        if (hardware_abort_requested())
        {
            g_tray2_waist_rc = -4;
            return;
        }
        std::cout << "[orch] 右手到位已 2s，开始降腰\n" << std::flush;
        try
        {
            if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
                g_tray2_waist_rc = -5;
        }
        catch (const std::exception &ex)
        {
            std::cerr << "[orch] 去料盘时腰部失败: " << ex.what() << "\n";
            g_tray2_waist_rc = -5;
        }
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

int take_tray2_waist()
{
    if (g_tray2_waist_th.joinable())
        g_tray2_waist_th.join();
    if (!g_tray2_waist_started)
        return 0;
    const int rc = g_tray2_waist_rc.load();
    g_tray2_waist_started = false;
    // 手臂已在放置准备位，腰也降到 ready：后面进 vision_place_tray2 不必再摆一次。
    if (rc == 0)
        g_tray2_place_prep_done = true;
    return rc;
}

} // namespace

bool live_gripper_holding(bool is_right)
{
    if (g_hw == nullptr)
        return false;
    if (is_right && right_arm_motors_locked())
        return false;
    const bool latch = is_right ? g_hw->last_grasped_r : g_hw->last_grasped_l;
    if (g_hw->g == nullptr ||
        !g_hw->g->hasSide(is_right ? gripper::Side::Right : gripper::Side::Left))
        return latch;
    const auto fb =
        g_hw->g->feedback(is_right ? gripper::Side::Right : gripper::Side::Left);
    if (!fb.have_feedback)
        return latch;
    // 与空抓判定同一阈值：q >= empty_q_rad 才算夹着零件。
    return fb.position_rad >= g_move_cfg.gripper_check.empty_q_rad;
}

void clear_last_grasped(bool right, bool left)
{
    if (g_hw == nullptr)
        return;
    if (right && g_hw->last_grasped_r)
    {
        g_hw->last_grasped_r = false;
        std::cout << "[grasp] 松爪/放置后清除右手持件闩锁\n";
    }
    if (left && g_hw->last_grasped_l)
    {
        g_hw->last_grasped_l = false;
        std::cout << "[grasp] 松爪/放置后清除左手持件闩锁\n";
    }
}

void wave_grip_ratio(bool is_right, double ratio)
{
    if (g_hw == nullptr || g_hw->g == nullptr)
        return;
    if (is_right && right_arm_motors_locked())
        return;
    if (ratio < 0.0)
        ratio = 0.0;
    if (ratio > 1.0)
        ratio = 1.0;
    const gripper::Side side = is_right ? gripper::Side::Right : gripper::Side::Left;
    std::cout << "[wave] " << (is_right ? "右手" : "左手")
              << " 夹爪收到 " << std::fixed << std::setprecision(2) << ratio
              << "（0全开 1全合）\n";
    g_hw->g->closeAndWait(side, ratio, 0.2, std::chrono::milliseconds(800));
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

int go_waist_cycle()
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[orch] waist_cycle 失败：腰未初始化\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;

    const double x_home = g_move_cfg.waist.layer3_home(0);
    const double z_home = g_move_cfg.waist.layer3_home(2);
    const double x_ready = waist_ready_x();
    const double z_ready = waist_ready_z();
    const double x_far = waist_far_row_x();
    constexpr int kCycles = 3;
    constexpr int kHoldMs = 3000;

    std::cout << std::fixed << std::setprecision(4)
              << "[orch] waist_cycle 只动腰，循环 " << kCycles
              << " 次，每到点停 " << (kHoldMs / 1000) << "s\n"
              << "  home  x=" << x_home << " z=" << z_home << "\n"
              << "  抓取 x=" << x_ready << " z=" << z_ready
              << "（ready_forward=" << g_move_cfg.waist.ready_forward_m << "）\n"
              << "  远排 x=" << x_far << " z=" << z_ready
              << "（far_row_forward=" << g_move_cfg.waist.far_row_forward_m << "）\n"
              << "  顺序：home → 抓取 → 远排 → 抓取 → home\n";

    auto fail = []() { return hardware_abort_requested() ? -4 : -5; };
    auto goto_xz = [&](double x, double z, const char *label) -> int {
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] waist_cycle → " << label
                  << " x=" << x << " z=" << z << " m\n";
        if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, label))
            return fail();
        if (!move_box::waist_layer3_move_x_only(*g_hw->waist, *g_hw->layer3, x, true, false))
        {
            std::cerr << "[orch] waist_cycle " << label << " 平移 X 失败\n";
            return fail();
        }
        if (hardware_abort_requested())
            return -4;
        if (!move_box::waist_layer3_move_z_only(*g_hw->waist, *g_hw->layer3, z, true, false))
        {
            std::cerr << "[orch] waist_cycle " << label << " 升降 Z 失败\n";
            return fail();
        }
        if (hardware_abort_requested())
            return -4;
        move_box::waist_layer3_set_synced(true);
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] waist_cycle " << label << " 到位 x=" << (*g_hw->layer3)(0)
                  << " z=" << (*g_hw->layer3)(2) << "，停 3s\n";
        hardware_abort_sleep_ms(kHoldMs);
        return hardware_abort_requested() ? -4 : 0;
    };

    for (int i = 1; i <= kCycles; ++i)
    {
        std::cout << "\n======== [orch] waist_cycle 第 " << i << "/" << kCycles
                  << " 圈 ========\n";
        if (const int rc = goto_xz(x_home, z_home, "home"); rc != 0)
            return rc;
        if (const int rc = goto_xz(x_ready, z_ready, "料盘抓取"); rc != 0)
            return rc;
        if (const int rc = goto_xz(x_far, z_ready, "远排"); rc != 0)
            return rc;
        if (const int rc = goto_xz(x_ready, z_ready, "料盘抓取"); rc != 0)
            return rc;
        if (const int rc = goto_xz(x_home, z_home, "home"); rc != 0)
            return rc;
    }
    std::cout << "[orch] waist_cycle 完成\n";
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
    bool skip_ready = false, int photo_after_arrival_ms = -1)
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
    const int settle_ms = photo_after_arrival_ms >= 0
                              ? std::max(0, photo_after_arrival_ms)
                              : static_cast<int>(std::lround(
                                    std::max(0.0, c.photo_after_arrival_sec) * 1000.0));
    int wait_ms = settle_ms;
    if (g_station_arrived)
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - g_station_arrived_at)
                                 .count();
        wait_ms = static_cast<int>(std::max<int64_t>(0, settle_ms - elapsed));
        std::cout << "[orch] " << name << " 导航完成已 "
                  << elapsed << " ms，再等 " << wait_ms << " ms 后拍照"
                  << "（yaml photo_after_arrival_sec=" << c.photo_after_arrival_sec
                  << "s）\n";
    }
    else
        std::cout << "[orch] " << name << " 未记录到站时刻，停稳 " << settle_ms
                  << " ms 再拍照（yaml photo_after_arrival_sec="
                  << c.photo_after_arrival_sec << "s）\n";
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
        g_hw->last_belt_message = "二维码解算失败";
        std::cerr << "[orch] belt 未拿到传送带坐标（二维码解算失败），手臂停在准备姿态，不往码上走\n";
        return kVisionQrFail;
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
        constexpr double place_mps = 0.25;
        std::cout << "[orch] belt " << (is_right ? "右手" : "左手")
                  << " " << std::fixed << std::setprecision(2) << place_mps
                  << "m/s 先到上方再放下\n";
        Eigen::Matrix<double, 1, 6> hover = goal;
        if (c.place_hover_above_m > 1e-9)
            hover(2) += c.place_hover_above_m;
        const ArmLineMoveResult approach = arm_dual_rounded_corner_move_selective(
            *g_hw->arm_r, hover, goal, is_right,
            *g_hw->arm_l, hover, goal, !is_right,
            place_mps);
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
            clear_last_grasped(is_right, !is_right);
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
        constexpr double place_mps = 0.25;
        Eigen::Matrix<double, 1, 6> lift = arm_get_tcp_pos(arm);
        const double z0 = lift(2);
        lift(2) = z0 + kPlaceNudgeUpM;
        Eigen::Matrix<double, 1, 6> ready = is_right ? c.tcp.right : c.tcp.left;
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] belt " << (is_right ? "右手" : "左手")
                  << " " << std::setprecision(2) << place_mps
                  << "m/s 松爪后抬 " << std::setprecision(4) << kPlaceNudgeUpM
                  << "m 后不停，直接回准备\n";
        const ArmLineMoveResult back = arm_dual_line_then_line_nostop(
            *g_hw->arm_r, lift, ready, is_right,
            *g_hw->arm_l, lift, ready, !is_right,
            place_mps, place_mps, {}, false, "放置收手");
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
    int right_note_rc = 0;
    std::thread th_r;
    const bool run_right = do_right && !right_arm_motors_locked();
    // 调度完成上报改到「最后放的那只手」开始上抬时，不再在松爪时发。
    const bool last_hand_is_left = do_left;
    const bool last_hand_is_right = run_right && !do_left;
    if (run_right)
    {
        const int right_rc = place_arrive_and_open(true);
        if (right_rc != 0)
            return right_rc;
        th_r = std::thread([&]() {
            if (last_hand_is_right && on_hand_placed)
            {
                right_note_rc = on_hand_placed(true);
                if (right_note_rc != 0)
                    return;
            }
            right_back_rc = lift_and_retract(true);
        });
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
        if (left_rc == 0)
        {
            if (last_hand_is_left && on_hand_placed)
            {
                const int note_rc = on_hand_placed(false);
                if (note_rc != 0)
                {
                    if (th_r.joinable())
                        th_r.join();
                    return note_rc;
                }
            }
            left_back_rc = lift_and_retract(false);
        }
    }
    else
        std::cout << "[orch] belt 左手这次不放\n";
    if (th_r.joinable())
        th_r.join();
    if (right_note_rc != 0)
        return right_note_rc;
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
    std::cout << "[orch] belt 底盘已到 " << st << "，到站后再等 "
              << g_move_cfg.conveyor.photo_after_arrival_sec << "s 拍照\n";
    return go_belt_station(g_move_cfg.conveyor, "belt");
}

/** 传送带重抓几次后仍空抓。 */
constexpr int kBeltGraspEmpty = 10;
constexpr int kEmptyRight = 1;
constexpr int kEmptyLeft = 2;

/** 拍照抓一次。空抓的手收回准备 tcp 后 empty_mask 置位并返回 0，由外层重拍重抓。 */
static int go_belt_grasp_attempt(
    bool from_place, const ConveyorStationConfig &c, const std::string &station, const char *tag,
    int only_hand, const std::function<int(bool is_right)> &on_grasped,
    bool photo_settle, const std::string &depart_after_right,
    const std::function<void(bool is_right)> &on_lift, int &empty_mask)
{
    empty_mask = 0;
    // 新一轮抓取先清掉上次路上降腰的标记，避免误跳过料盘二准备。
    g_tray2_place_prep_done = false;
    struct BeltDepartGuard
    {
        ~BeltDepartGuard()
        {
            take_belt_depart();
            take_tray2_waist();
        }
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
                  << "，腰头到位后停稳 " << c.grasp_photo_settle_sec << "s 再拍照\n";
        const int wh = conveyor_waist_and_head(c, name);
        if (wh != 0)
            return wh;
        if (aborted())
            return -4;
    }

    const int grasp_settle_ms = static_cast<int>(
        std::lround(std::max(0.0, c.grasp_photo_settle_sec) * 1000.0));
    if (photo_settle)
    {
        std::cout << "[orch] belt_grasp 不转腰，停稳 " << c.grasp_photo_settle_sec
                  << "s 再拍照；grasp_offset 按码坐标系，允许过中轴\n";
        if (grasp_settle_ms > 0)
            hardware_abort_sleep_ms(grasp_settle_ms);
    }
    else
        std::cout << "[orch] belt_grasp 这一只手底盘没动过，不再停 "
                  << c.grasp_photo_settle_sec << "s，直接拍照\n";
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

    if (!det.ok)
    {
        g_hw->last_belt_message = "二维码解算失败";
        std::cerr << "[orch] belt_grasp 二维码解算失败，不往码上走\n";
        return kVisionQrFail;
    }
    if ((need_l && !det.have_grasp_l) || (need_r && !det.have_grasp_r))
    {
        std::cerr << "[orch] belt_grasp 未拿到"
                  << (only_hand < 0 ? "左右" : (only_hand == 0 ? "左手" : "右手"))
                  << "抓取点，不往码上走\n";
        return -3;
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
        constexpr double reach_mps = 0.25;
        std::cout << std::fixed << std::setprecision(2)
                  << "[orch] belt_grasp " << hand << " " << reach_mps
                  << "m/s 先到上方再下去\n";
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
            const gripper::Side side = is_right ? gripper::Side::Right : gripper::Side::Left;
            const auto fb = g_hw->g->graspAndWait(side);
            log_gripper_qs(is_right ? "belt_grasp右合爪后" : "belt_grasp左合爪后");
            if (move_box::gripper_grasp_empty(*g_hw->g, side, fb, name))
                return aborted() ? -4 : kBeltGraspEmpty;
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
    const char *back_name_default = back_to_belt_ready ? "传送带准备 tcp"
                                    : back_to_tray_ready ? "料盘放置准备"
                                                         : "grasp_tcp";
    // to_ready：空抓的手回到伸手前的准备 tcp，外层从这里直接重拍重抓。
    auto grasp_lift_and_back = [&](bool is_right, bool to_ready = false) -> int {
        if (is_right && right_arm_motors_locked())
            return 0;
        if (aborted())
            return -4;
        Robot_Arm &arm = is_right ? *g_hw->arm_r : *g_hw->arm_l;
        const char *hand = is_right ? "右手" : "左手";
        const char *back_name = to_ready ? "传送带准备 tcp（空抓）" : back_name_default;
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
        if (back_to_belt_ready || to_ready)
            back_pose = is_right ? c.tcp.right : c.tcp.left;
        else if (back_to_tray_ready)
            back_pose = is_right ? g_move_cfg.standby.right : g_move_cfg.standby.left;
        // 平移时保持抓取腕角。腕子差得大的目标姿态等到位置到了再慢转，避免接缝抽臂。
        Eigen::Matrix<double, 1, 6> back_move = back_pose;
        back_move(3) = lift(3);
        back_move(4) = lift(4);
        back_move(5) = lift(5);
        constexpr double retract_mps = 0.25;
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
        if (rc == kBeltGraspEmpty)
        {
            const int back_rc = grasp_lift_and_back(is_right, true);
            if (back_rc != 0)
                return back_rc;
            empty_mask = is_right ? kEmptyRight : kEmptyLeft;
            return 0;
        }
        if (rc != 0)
            return rc;
        int note_rc = 0;
        std::thread note_th;
        std::thread lift_th;
        const bool run_cb = !(is_right && right_arm_motors_locked());
        if (on_grasped && run_cb)
            note_th = std::thread([&]() { note_rc = on_grasped(is_right); });
        // 上抬一开始就回调（调度用来申请下一只手），与抬升/收手并行。
        if (on_lift && run_cb)
            lift_th = std::thread([&]() { on_lift(is_right); });
        const int back_rc = grasp_lift_and_back(is_right);
        if (lift_th.joinable())
            lift_th.join();
        if (note_th.joinable())
            note_th.join();
        if (note_rc != 0)
            return note_rc;
        if (back_rc != 0)
            return back_rc;
        if (is_right &&
            station_name_matches(depart_after_right, g_move_cfg.chassis.tray2_station))
            start_tray2_waist_after_right_arrived();
        const int dep_rc = take_belt_depart();
        const int waist_rc = take_tray2_waist();
        if (dep_rc != 0)
            return dep_rc;
        if (waist_rc != 0)
            return waist_rc;
        std::cout << "[orch] " << name << (is_right ? " 右手" : " 左手")
                  << "单手抓取结束，停在 " << back_name_default << "\n";
        return 0;
    }

    int left_back_rc = 0;
    const int left_rc = grasp_one(false);
    if (left_rc == kBeltGraspEmpty)
    {
        // 右手抓到会触发底盘出发，所以左手没抓到时右手这一轮不动，两只手一起重来。
        std::cout << "[orch] " << name << " 左手空抓，右手这一轮不伸，回准备位重拍\n";
        const int back_rc = grasp_lift_and_back(false, true);
        if (back_rc != 0)
            return back_rc;
        empty_mask = kEmptyLeft;
        return 0;
    }
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
        const bool right_empty = right_rc == kBeltGraspEmpty;
        const int right_back_rc =
            (right_rc == 0 || right_empty) ? grasp_lift_and_back(true, right_empty) : 0;
        if (right_rc == 0 && right_back_rc == 0 &&
            station_name_matches(depart_after_right, g_move_cfg.chassis.tray2_station))
            start_tray2_waist_after_right_arrived();
        th_l.join();
        if (right_rc != 0 && !right_empty)
            return right_rc;
        if (right_back_rc != 0)
            return right_back_rc;
        if (left_back_rc != 0)
            return left_back_rc;
        if (right_empty)
            empty_mask = kEmptyRight;
    }

    const int dep_rc = take_belt_depart();
    const int waist_rc = take_tray2_waist();
    if (dep_rc != 0)
        return dep_rc;
    if (waist_rc != 0)
        return waist_rc;
    if (empty_mask == 0)
        std::cout << "[orch] " << name << " 双手抓取结束，停在 " << back_name_default
                  << "，可接入底盘/放置\n";
    return 0;
}

int go_belt_grasp(
    bool from_place, const ConveyorStationConfig &c, const std::string &station, const char *tag,
    int only_hand = -1, const std::function<int(bool is_right)> &on_grasped = {},
    bool photo_settle = true, const std::string &depart_after_right = {},
    const std::function<void(bool is_right)> &on_lift = {})
{
    const char *name = (tag && tag[0]) ? tag : "belt_grasp";
    const int max_retry = g_move_cfg.gripper_check.max_empty_retry;
    int hand = only_hand;
    for (int attempt = 0;; ++attempt)
    {
        int empty_mask = 0;
        const int rc = go_belt_grasp_attempt(
            attempt == 0 ? from_place : true, c, station, tag, hand, on_grasped,
            attempt == 0 ? photo_settle : true, depart_after_right, on_lift, empty_mask);
        if (rc != 0)
        {
            g_tray2_place_prep_done = false;
            return rc;
        }
        if (empty_mask == 0)
            return rc;
        const std::string who = empty_mask == kEmptyRight ? "右手" : "左手";
        if (attempt >= max_retry)
        {
            const std::string msg = std::string(name) + " " + who + "连续空抓 " +
                                    std::to_string(attempt + 1) +
                                    " 次，停止重抓，检查传送带上有没有件、夹爪是否正常";
            std::cerr << "[orch] " << msg << "\n";
            board_error(msg);
            g_tray2_place_prep_done = false;
            return kBeltGraspEmpty;
        }
        board_note("warn", std::string(name) + " " + who +
                               "空抓，不算持件、不汇报，回准备位重拍重抓（第 " +
                               std::to_string(attempt + 1) + " 次重抓）");
        // 双手模式里右手空抓说明左手已经抓到，只补右手；左手空抓时右手还没伸，两只手重来。
        if (only_hand < 0 && empty_mask == kEmptyRight)
            hand = 1;
        if (hardware_abort_requested())
        {
            g_tray2_place_prep_done = false;
            return -4;
        }
    }
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
    std::cout << "[orch] belt2 底盘已到 " << st << "，到站后再等 "
              << g_move_cfg.conveyor2.photo_after_arrival_sec << "s 拍照\n";
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

int vision_grasp_with_engine(SegEngineId engine_id, bool adaptive_approach_rpy = false,
                            bool depart_after_grasp = true, bool clear_tray_when_stay = true)
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

    const std::string here = chassis_location_station();
    const bool from_tray2 = station_name_matches(here, g_move_cfg.chassis.tray2_station);
    const bool from_belt2 =
        !from_tray2 && station_name_matches(here, g_move_cfg.chassis.out_station);
    // 仅料盘二回料盘一：先升腰，到站再降。传送带二回料盘一：出发 2s 后路上降腰摆手。
    const bool lower_after_arrive = from_tray2;
    if (from_tray2)
    {
        std::cout << "[grasp] 现在在料盘二，先升腰，再去 "
                  << g_move_cfg.chassis.tray_station << "，到站后再降腰\n";
        const int up_rc = home_waist_upright();
        if (up_rc != 0)
            return up_rc;
        if (should_abort())
            return -4;
    }
    else if (from_belt2)
    {
        std::cout << "[grasp] 现在在传送带二，去 "
                  << g_move_cfg.chassis.tray_station
                  << "；底盘一出发等 2s 开始降腰并摆手部姿态"
                  << (move_box::tray1_far_band_active() ? "（远三排记忆已开）\n" : "\n");
    }
    else
    {
        std::cout << "[grasp] 去料盘 " << g_move_cfg.chassis.tray_station
                  << "，路上同时摆头、手臂、开爪和下腰"
                  << (move_box::tray1_far_band_active() ? "（远三排记忆已开）\n" : "\n");
    }
    board_step("去料盘一");
    int prep_rc = 0;
    int travel_rc = 0;
    {
        ChassisTravel travel(g_move_cfg.chassis.tray_station);
        auto prep_upper_and_waist = [&](bool do_waist) -> int {
            const bool far_mem = move_box::tray1_far_band_active();
            const int head_rc =
                far_mem ? enable_head_far_pitch() : enable_head_calib_pose();
            if (head_rc != 0)
                return head_rc;
            if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l))
            {
                std::cerr << "[grasp] 进入时回到 standby 失败\n";
                return should_abort() ? -4 : -5;
            }
            if (should_abort())
                return -4;
            g_hw->g->closeAndWait(
                gripper::Side::Right, 0.2, 0.2, std::chrono::milliseconds(800));
            g_hw->g->closeAndWait(
                gripper::Side::Left, 0.2, 0.2, std::chrono::milliseconds(800));
            if (should_abort())
                return -4;
            if (!do_waist)
                return 0;
            if (g_hw->waist == nullptr || g_hw->layer3 == nullptr)
            {
                std::cerr << "[grasp] 腰未初始化，拒绝进入抓取\n";
                return -1;
            }
            if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
            {
                std::cerr << "[grasp] 腰未到 ready 起始位，放弃抓取\n";
                return should_abort() ? -4 : -5;
            }
            if (far_mem)
            {
                std::cout << "[grasp] 远三排记忆：路上腰再前伸到远排位\n";
                move_box::waist_layer3_ensure_far_row_forward(
                    *g_hw->waist, *g_hw->layer3, g_move_cfg.waist.far_row_forward_m);
            }
            move_box::waist_layer3_set_synced(true);
            return should_abort() ? -4 : 0;
        };

        if (from_belt2)
        {
            std::cout << "[grasp] 底盘已出发，等 2s 后降腰摆手\n";
            hardware_abort_sleep_ms(2000);
            if (should_abort())
                prep_rc = -4;
            else
                prep_rc = prep_upper_and_waist(true);
        }
        else if (lower_after_arrive)
        {
            prep_rc = prep_upper_and_waist(false);
            if (prep_rc == 0)
                std::cout << "[grasp] 腰先不动，等底盘到 "
                          << g_move_cfg.chassis.tray_station << "\n";
        }
        else
            prep_rc = prep_upper_and_waist(true);
        travel_rc = travel.join();
    }
    if (should_abort())
        return -4;
    if (travel_rc != 0)
        return travel_rc;
    if (prep_rc != 0)
        return prep_rc;
    if (lower_after_arrive)
    {
        std::cout << "[grasp] 底盘已到 " << g_move_cfg.chassis.tray_station << "，开始降腰\n";
        if (g_hw->waist == nullptr || g_hw->layer3 == nullptr ||
            !move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
        {
            std::cerr << "[grasp] 到站后降腰失败，放弃抓取\n";
            return should_abort() ? -4 : -5;
        }
        if (move_box::tray1_far_band_active())
        {
            move_box::waist_layer3_ensure_far_row_forward(
                *g_hw->waist, *g_hw->layer3, g_move_cfg.waist.far_row_forward_m);
            enable_head_far_pitch();
        }
        move_box::waist_layer3_set_synced(true);
        if (should_abort())
            return -4;
    }
    std::cout << "[grasp] 已在 " << g_move_cfg.chassis.tray_station << "，停稳 0.5s 再抓\n";
    board_step("料盘一抓取");
    hardware_abort_sleep_ms(500);
    if (should_abort())
        return -4;

    g_load_parts_seen = -1;

    const bool clear_tray = !depart_after_grasp && clear_tray_when_stay;
    const int kMaxRounds = clear_tray ? 80 : 40;
    constexpr int kMaxNoneStreak = 2;
    int grasped_count = 0;
    int none_streak = 0;
    int empty_streak_r = 0;
    int empty_streak_l = 0;
    const int max_empty_retry = g_move_cfg.gripper_check.max_empty_retry;
    bool holding_r = false;
    bool holding_l = false;
    g_hw->last_grasp_count = 0;
    g_hw->last_grasped_r = false;
    g_hw->last_grasped_l = false;

    const auto both_holding = [&]() -> bool {
        return right_arm_motors_locked() ? holding_l : (holding_r && holding_l);
    };

    if (clear_tray)
        std::cout << "[grasp] 开始清空料盘：每轮收手后松爪，同时拍照抓下一轮，抓完停在原地"
                  << (right_arm_motors_locked() ? "（右臂锁定，只抓左手）\n" : "\n");
    else
        std::cout << "[grasp] 开始循环：夹后保持合爪；双手都有件才结束，缺的再去拿"
                  << (depart_after_grasp ? "" : "；抓完停在原地不走")
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

        if (!clear_tray && both_holding())
        {
            last_rc = 0;
            std::cout << "[grasp] 双手已持件，结束抓取循环\n";
            break;
        }

        std::cout << "\n======== [grasp] 第 " << round << " 轮 持件 右="
                  << (holding_r ? 1 : 0) << " 左=" << (holding_l ? 1 : 0)
                  << " ========\n";
        board_activity("料盘一第 " + std::to_string(round) + " 轮：拍照识别并抓取");
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
            clear_tray ? false : holding_r,
            clear_tray ? false : holding_l);

        if (g_load_parts_seen < 0 && r.zone_count_ok)
        {
            g_load_parts_seen = r.zone_right + r.zone_left;
            std::cout << "[grasp] 选孔与计数同一张照片，开抓前工作区毛坯 "
                      << g_load_parts_seen
                      << " 件（右=" << r.zone_right << " 左=" << r.zone_left << "）\n";
        }
        if (r.zone_count_ok)
            board_tray1(r.zone_right - (r.grasped_r ? 1 : 0), r.zone_left - (r.grasped_l ? 1 : 0));
        board_tray1_taken((r.grasped_r ? 1 : 0) + (r.grasped_l ? 1 : 0));

        if (!clear_tray)
        {
            holding_r = holding_r || r.grasped_r;
            holding_l = holding_l || r.grasped_l;
            grasped_count = (holding_r ? 1 : 0) + (holding_l ? 1 : 0);
        }
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

        if (r.empty_r || r.empty_l)
            board_note("warn", std::string("料盘一空抓") + (r.empty_r ? " 右手" : "") +
                                   (r.empty_l ? " 左手" : "") + "，不计数，原地重拍重抓");
        empty_streak_r = r.empty_r ? empty_streak_r + 1 : (r.grasped_r ? 0 : empty_streak_r);
        empty_streak_l = r.empty_l ? empty_streak_l + 1 : (r.grasped_l ? 0 : empty_streak_l);
        if (empty_streak_r > max_empty_retry || empty_streak_l > max_empty_retry)
        {
            const bool right = empty_streak_r > max_empty_retry;
            const std::string msg = std::string("料盘一") + (right ? "右手" : "左手") + "连续空抓 " +
                                    std::to_string(right ? empty_streak_r : empty_streak_l) +
                                    " 次，停止重抓，检查夹爪和零件";
            std::cerr << "[grasp] " << msg << "\n";
            board_error(msg);
            last_rc = 2;
            break;
        }

        if (r.status == move_box::GraspToStandbyStatus::Ok || r.grasped_r || r.grasped_l)
        {
            none_streak = 0;
            if (clear_tray)
            {
                const int n = (r.grasped_r ? 1 : 0) + (r.grasped_l ? 1 : 0);
                grasped_count += n;
                g_hw->last_grasp_count = grasped_count;
                std::cout << "[grasp] 第 " << round << " 轮收手完成，松开并开始下一轮拍照"
                          << " 本轮抓=" << n << " 累计=" << grasped_count << "\n";
                if (r.grasped_r && !right_arm_motors_locked())
                    g_hw->g->close(gripper::Side::Right, 0.2);
                if (r.grasped_l)
                    g_hw->g->close(gripper::Side::Left, 0.2);
                clear_last_grasped(r.grasped_r, r.grasped_l);
                last_rc = 0;
                continue;
            }
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
            if (!r.zone_count_ok)
            {
                g_hw->last_tray_message = "二维码解算失败";
                std::cerr << "[grasp] 料盘二维码解算失败，本轮不伸手\n";
                last_rc = kVisionQrFail;
                break;
            }
            if (clear_tray)
            {
                std::cout << "[grasp] 料盘已无待抓零件，累计 " << grasped_count
                          << " 件，结束循环\n";
                last_rc = grasped_count > 0 ? 0 : 1;
                break;
            }
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
              << " 共 " << grasped_count << " 件"
              << (clear_tray ? "，夹爪松开" : "，夹爪保持合上") << std::endl;
    if (clear_tray)
    {
        if (!right_arm_motors_locked())
            g_hw->g->closeAndWait(
                gripper::Side::Right, 0.2, 0.2, std::chrono::milliseconds(800));
        g_hw->g->closeAndWait(
            gripper::Side::Left, 0.2, 0.2, std::chrono::milliseconds(800));
        std::cout << "[grasp] 料盘抓取结束，停在原地，底盘不走" << std::endl;
        return last_rc;
    }
    if ((holding_r || holding_l) && depart_after_grasp)
    {
        board_step("去传送带一");
        start_chassis();
        if (g_on_grasp_depart)
            g_on_grasp_depart();
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
    else if (holding_r || holding_l)
        std::cout << "[grasp] 抓完停在原地，底盘不走，夹爪保持合上" << std::endl;
    return last_rc;
}

} // namespace

int vision_grasp_existing_pipeline(bool depart_after_grasp)
{
    return vision_grasp_with_engine(SegEngineId::Default, false, depart_after_grasp);
}

int vision_grasp_once()
{
    return vision_grasp_with_engine(SegEngineId::Default, false, false, false);
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

    board_step("去料盘二");
    const bool prep_done = g_tray2_place_prep_done;
    g_tray2_place_prep_done = false;
    int prep_rc = 0;
    int travel_rc = 0;
    if (prep_done)
    {
        std::cout << "[tray2] 去料盘2 " << g_move_cfg.chassis.tray2_station
                  << "：路上已降腰并收手到放置准备，不再重摆，也不再等 0.5s\n";
        travel_rc = chassis_goto_station(g_move_cfg.chassis.tray2_station);
        if (g_hw != nullptr && g_hw->waist != nullptr && g_hw->layer3 != nullptr)
            move_box::waist_layer3_set_synced(true);
    }
    else
    {
        std::cout << "[tray2] 去料盘2 " << g_move_cfg.chassis.tray2_station
                  << "，路上同时摆头、待机臂和下腰\n";
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
    }
    if (should_abort())
        return -4;
    if (travel_rc != 0)
        return travel_rc;
    if (prep_rc != 0)
        return prep_rc;
    board_step("料盘二放置");
    if (!prep_done)
    {
        std::cout << "[tray2] 已在 " << g_move_cfg.chassis.tray2_station << "，停稳 0.5s 再放\n";
        hardware_abort_sleep_ms(500);
        if (should_abort())
            return -4;
    }
    else
        std::cout << "[tray2] 已在 " << g_move_cfg.chassis.tray2_station
                  << "，直接进入放置\n";

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
        board_activity("料盘二第 " + std::to_string(round) + " 轮：拍照找空孔并放置");
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
            board_tray2(remain_r, remain_l);
        }
        board_tray2_placed((r.grasped_r ? 1 : 0) + (r.grasped_l ? 1 : 0));

        // 放置成功＝该侧已松爪，清流程闩锁；界面持件改看实时 q。
        if (r.grasped_r)
        {
            placed_r = true;
            clear_last_grasped(true, false);
        }
        if (r.grasped_l)
        {
            placed_l = true;
            clear_last_grasped(false, true);
        }
        placed_count = (placed_r ? 1 : 0) + (placed_l ? 1 : 0);
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
            if (!r.zone_count_ok)
            {
                g_hw->last_tray_message = "二维码解算失败";
                std::cerr << "[tray2] 料盘二维码解算失败，本轮不伸手\n";
                last_rc = kVisionQrFail;
                break;
            }
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
        std::cout << "[tray2] 放完先升腰，再去 " << next << "，到站后再降腰\n";
        const int head_rc = enable_head_calib_pose();
        if (head_rc != 0)
            return head_rc;
        const int waist_rc = home_waist_upright();
        if (waist_rc != 0)
            return waist_rc;
        if (should_abort())
            return -4;
        const int chassis_rc = chassis_goto_station(next);
        if (chassis_rc != 0)
            return chassis_rc;
        if (should_abort())
            return -4;
        if (g_hw->waist == nullptr || g_hw->layer3 == nullptr ||
            !move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
        {
            std::cerr << "[tray2] 到 " << next << " 后降腰失败\n";
            return should_abort() ? -4 : -5;
        }
        move_box::waist_layer3_set_synced(true);
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
            board_tray2(left.right, left.left);
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
    move_box::tray2_precision_clear_visited();

    std::cout << "[tray2] 先到传送带二抓取准备姿态，再导航到 "
              << g_move_cfg.chassis.tray2_station << "，到站后降腰。夹爪全程闭合\n";
    if (g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
        return -1;
    const int ready_rc = go_belt_ready(
        *g_hw->arm_r, *g_hw->arm_l, g_move_cfg.conveyor2, "belt2");
    if (ready_rc != 0)
        return ready_rc;
    if (should_abort())
        return -4;
    const int travel_rc = chassis_goto_station(g_move_cfg.chassis.tray2_station);
    if (travel_rc != 0)
        return travel_rc;
    if (should_abort())
        return -4;
    if (g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return -1;
    if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
        return should_abort() ? -4 : -5;
    move_box::waist_layer3_set_synced(true);

    const int head_rc = enable_head_calib_pose();
    if (head_rc != 0)
        return head_rc;
    if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l, -1, true))
    {
        std::cerr << "[tray2] 进入放置姿态失败\n";
        return should_abort() ? -4 : -5;
    }
    if (g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return -1;
    if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
        return should_abort() ? -4 : -5;
    move_box::waist_layer3_set_synced(true);

    std::cout << "[tray2] 夹爪闭合，之后全程不松开\n";
    if (!right_arm_motors_locked())
        g_hw->g->graspAndWait(gripper::Side::Right);
    g_hw->g->graspAndWait(gripper::Side::Left);
    if (should_abort())
        return -4;

    constexpr int kMaxRounds = 40;
    int placed_count = 0;
    g_hw->last_grasp_count = 0;
    g_hw->last_grasped_r = false;
    g_hw->last_grasped_l = false;

        std::cout << "[tray2] 开始按空孔伸手，腰保持下蹲\n";
    int last_rc = 1;
    for (int round = 1; round <= kMaxRounds; ++round)
    {
        if (should_abort())
            return -4;
        std::cout << "\n======== [tray2] 第 " << round << " 组 ========\n";
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
            std::cout << "[tray2] 空孔都已伸过手，停在蹲姿。已走 "
                      << placed_count << " 次\n";
            last_rc = placed_count > 0 ? 0 : 1;
            break;
        }
        if (r.status != move_box::GraspToStandbyStatus::Ok && !r.grasped_r && !r.grasped_l)
        {
            std::cerr << "[tray2] 本次未伸到 code=" << static_cast<int>(r.status)
                      << "，停在蹲姿\n";
            last_rc = 2;
            break;
        }
        if (r.grasped_r)
        {
            move_box::tray2_precision_mark_visited(r.st.row_aruco_r, r.st.col_aruco_r);
            std::cout << "[tray2] 记住右手已伸 ArUco row=" << r.st.row_aruco_r
                      << " col=" << r.st.col_aruco_r << "\n";
        }
        if (r.grasped_l)
        {
            move_box::tray2_precision_mark_visited(r.st.row_aruco_l, r.st.col_aruco_l);
            std::cout << "[tray2] 记住左手已伸 ArUco row=" << r.st.row_aruco_l
                      << " col=" << r.st.col_aruco_l << "\n";
        }
        ++placed_count;
        g_hw->last_grasp_count = placed_count;
        g_hw->last_grasped_r = r.grasped_r;
        g_hw->last_grasped_l = r.grasped_l;
        last_rc = 0;
    }
    std::cout << "[tray2] 精度测试结束，不起身。伸手次数=" << placed_count << "\n";
    return last_rc;
}

namespace
{

bool wait_tray2_center_key()
{
    std::cout << "\n[tray2center] 已停住。请在运行 t170c_debug 的终端按回车，或输入一个字符后继续\n"
              << std::flush;
    if (!isatty(STDIN_FILENO))
    {
        std::cerr << "[tray2center] 标准输入不是终端，无法等待按键，直接继续\n";
        return !hardware_abort_requested();
    }
    while (!hardware_abort_requested())
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 200000;
        const int sel = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
        if (sel <= 0)
            continue;
        char ch = 0;
        if (read(STDIN_FILENO, &ch, 1) > 0)
        {
            std::cout << "[tray2center] 继续\n" << std::flush;
            return true;
        }
    }
    return false;
}

bool detect_tray2_origin(double &x, double &y, double &z)
{
    if (g_hw == nullptr || g_hw->pipeline == nullptr)
        return false;
    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.tray_engine_ready())
    {
        std::cerr << "[tray2center] 料盘引擎未加载\n";
        return false;
    }
    constexpr CameraSlot slot = CameraSlot::Head;
    RealSenseMultiCam &cameras = g_hw->pipeline->cameras();
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
        std::cerr << "[tray2center] 头相机取帧失败\n";
        return false;
    }
    PoseDetectionRecords yolo;
    const TrayDetectResult det = bridge.run_tray_annotate_multiframe(
        frames, yolo, kDebugVisualize, slot, true);
    if (!det.ok || !std::isfinite(det.origin_x) || !std::isfinite(det.origin_y) ||
        !std::isfinite(det.origin_z))
    {
        std::cerr << "[tray2center] 没解出料盘中心: " << det.message << "\n";
        return false;
    }
    x = det.origin_x;
    y = det.origin_y;
    z = det.origin_z;
    std::cout << std::fixed << std::setprecision(4)
              << "[tray2center] 料盘中心（码坐标系原点）xyz=("
              << x << "," << y << "," << z << ") 码=" << det.used_ids.size()
              << " reproj=" << det.reproj_px << "px\n";
    return true;
}

bool arm_to_tray_center(bool is_right, double x, double y, double z)
{
    if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
        return false;
    if (is_right && right_arm_motors_locked())
    {
        std::cout << "[tray2center] 右臂锁定，跳过右手\n";
        return true;
    }
    constexpr double kCenterZUpM = 0.05;
    const GraspRpyDeg &rpy = tray2_place_rpy_deg_for_ready(2, is_right);
    Eigen::Matrix<double, 1, 6> goal;
    goal << x, y, z + kCenterZUpM, rpy.rx * M_PI / 180.0, rpy.ry * M_PI / 180.0, rpy.rz * M_PI / 180.0;
    Robot_Arm &arm = is_right ? *g_hw->arm_r : *g_hw->arm_l;
    std::cout << std::fixed << std::setprecision(3)
              << "[tray2center] " << (is_right ? "右手" : "左手")
              << " 0.25m/s 伸到料盘中心，Z 再向上 " << kCenterZUpM << " m\n";
    const int rc = arm_line_move_settle(arm, goal, 0.25, 2000);
    if (rc != 0)
    {
        std::cerr << "[tray2center] " << (is_right ? "右手" : "左手")
                  << " 去中心失败 code=" << rc << "\n";
        return false;
    }
    return !hardware_abort_requested();
}

} // namespace

int vision_tray2_center()
{
    if (g_hw == nullptr || g_hw->g == nullptr || g_hw->pipeline == nullptr ||
        g_hw->arm_r == nullptr || g_hw->arm_l == nullptr ||
        g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return -1;
    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };
    std::cout << "[tray2center] 底盘原地不动。夹爪保持闭合。先腰到远端，右手再左手到码原点；"
                 "腰回近处后，左手再右手各去一次中心\n";
    const int waist_home_rc = home_waist_upright();
    if (waist_home_rc != 0)
        return waist_home_rc;
    if (should_abort())
        return -4;
    const int head_rc = enable_head_calib_pose();
    if (head_rc != 0)
        return head_rc;
    if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l, -1, true))
        return should_abort() ? -4 : -5;
    if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
        return should_abort() ? -4 : -5;
    move_box::waist_layer3_set_synced(true);
    std::cout << "[tray2center] 夹爪闭合，之后全程不松开\n";
    if (!right_arm_motors_locked())
        g_hw->g->graspAndWait(gripper::Side::Right);
    g_hw->g->graspAndWait(gripper::Side::Left);
    if (should_abort())
        return -4;

    const double x_near = waist_ready_x();
    const double x_far = waist_far_row_x();
    const int far_head = enable_head_far_pitch();
    if (far_head != 0)
        return far_head;
    std::cout << std::fixed << std::setprecision(4)
              << "[tray2center] 腰平移到远端 x=" << x_far << "\n";
    if (!move_box::waist_layer3_move_x_only(*g_hw->waist, *g_hw->layer3, x_far, true))
        return should_abort() ? -4 : -5;
    double cx = 0, cy = 0, cz = 0;
    if (!detect_tray2_origin(cx, cy, cz))
        return should_abort() ? -4 : -3;
    if (!arm_to_tray_center(true, cx, cy, cz))
        return should_abort() ? -4 : -5;
    if (!wait_tray2_center_key())
        return -4;
    if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l, -1, true))
        return should_abort() ? -4 : -5;
    if (!arm_to_tray_center(false, cx, cy, cz))
        return should_abort() ? -4 : -5;
    if (!wait_tray2_center_key())
        return -4;
    if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l, -1, true))
        return should_abort() ? -4 : -5;

    const int near_head = enable_head_calib_pose();
    if (near_head != 0)
        return near_head;
    std::cout << std::fixed << std::setprecision(4)
              << "[tray2center] 腰平移回近处初始 x=" << x_near << "\n";
    if (!move_box::waist_layer3_move_x_only(*g_hw->waist, *g_hw->layer3, x_near, true))
        return should_abort() ? -4 : -5;
    if (!detect_tray2_origin(cx, cy, cz))
        return should_abort() ? -4 : -3;
    if (!arm_to_tray_center(false, cx, cy, cz))
        return should_abort() ? -4 : -5;
    if (!wait_tray2_center_key())
        return -4;
    if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l, -1, true))
        return should_abort() ? -4 : -5;
    if (!arm_to_tray_center(true, cx, cy, cz))
        return should_abort() ? -4 : -5;
    std::cout << "[tray2center] 结束，右手停在近处料盘中心，夹爪仍闭合\n";
    return 0;
}

int vision_belt2_then_tray2()
{
    std::cout << "[orch] cycle3：先到传送带二抓取准备姿态，再去 "
              << g_move_cfg.chassis.out_station
              << " 抓取 → " << g_move_cfg.chassis.tray2_station
              << " 放置，放完停在原地，不升腰、不回 "
              << g_move_cfg.chassis.tray_station << "\n";
    if (hardware_abort_requested())
        return -4;
    if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
        return -1;
    const int ready_rc = go_belt_ready(
        *g_hw->arm_r, *g_hw->arm_l, g_move_cfg.conveyor2, "belt2");
    if (ready_rc != 0)
        return ready_rc;
    if (hardware_abort_requested())
        return -4;
    const int ch_rc = chassis_goto_station(g_move_cfg.chassis.out_station);
    if (ch_rc != 0)
        return ch_rc;
    if (hardware_abort_requested())
        return -4;
    const int grasp_rc = go_belt_grasp(
        true, g_move_cfg.conveyor2, g_move_cfg.chassis.out_station, "belt2_grasp",
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
                  << "，准备姿态后停稳 "
                  << g_move_cfg.conveyor.photo_after_arrival_sec
                  << "s 再第一次拍照放置\n";
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
                  << "，准备姿态后停稳 "
                  << g_move_cfg.conveyor2.photo_after_arrival_sec
                  << "s 再第二次拍照放置\n";
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
                  << "（右手抓到 2s 后底盘已出发，右手到位 2s 后降腰）\n";
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

constexpr int kDispatchApplyTimeoutMs = 20000;
/** 子任务完成后，原地听下一条任务的最长时间；超时再 home→AP14。 */
constexpr int kDispatchNextMissionWaitMs = 20000;
constexpr int kDispatchHandSkipped = 8;
/** 申请超时或手里还有件但这次没放成：原地待命，不报完成，也不回料盘再抓。 */
constexpr int kDispatchHoldStay = 9;
constexpr int kDispatchPointsMissing = 7;

struct TaskPoints
{
    std::string raw;
    std::string well;
    std::string load;
    std::string unload;
};

TaskPoints read_task_points()
{
    TaskPoints p;
    p.raw = RobotClient::raw_material_point;
    p.well = RobotClient::well_done_point;
    p.load = RobotClient::sequence_one;
    p.unload = RobotClient::sequence_two;
    std::cout << "[dispatch] 调度点位 上料架=" << p.raw
              << " 上料点=" << p.load
              << " 下料点=" << p.unload
              << " 下料架=" << p.well << "\n";
    return p;
}

bool task_points_ready(const std::string &task, const TaskPoints &p)
{
    auto miss = [&](const char *name, const std::string &value) {
        if (!value.empty())
            return false;
        std::cerr << "[dispatch] 任务 " << task << " 缺少点位 " << name << "\n";
        return true;
    };
    if (task == dispatch_msg::TASK_LOAD)
        return !(miss("raw_material_point", p.raw) || miss("sequence_one", p.load));
    if (task == dispatch_msg::TASK_TRANSFER)
        return !(miss("sequence_one", p.load) || miss("sequence_two", p.unload));
    if (task == dispatch_msg::TASK_UNLOAD)
        return !(miss("sequence_two", p.unload) || miss("well_done_point", p.well));
    return false;
}

struct StationOverride
{
    ChassisConfig saved;
    explicit StationOverride(const ChassisConfig &now) : saved(now) {}
    ~StationOverride() { g_move_cfg.chassis = saved; }
};

int dispatch_back_to_tray(bool watch_obstacle = false)
{
    g_dispatch_nav_fault.clear();
    g_photo_ready_station.clear();
    const std::string rack = g_dispatch_rack_point.empty()
                                 ? g_move_cfg.chassis.tray_station
                                 : g_dispatch_rack_point;
    std::cout << "[dispatch] 回 " << rack
              << "，手臂和腰回 home 与底盘同时动\n";
    int home_rc = 0;
    int ch_rc = 0;
    {
        ChassisTravel travel(rack, watch_obstacle);
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
    const bool ok = client.report_state(state);
    board_state(state, ok);
    if (!ok)
    {
        std::cerr << "[dispatch] 上报状态 " << state << " 失败\n";
        return false;
    }
    return true;
}

bool dispatch_apply(
    const std::string &request, const std::string &grant, const std::string &what,
    const std::function<bool()> &call)
{
    board_apply_begin(request, grant, what);
    const bool ok = call();
    board_apply_end(ok);
    return ok;
}

void dispatch_report_qr_fail(RobotClient &client, const char *what)
{
    const std::string msg = std::string(what) + " 二维码解算失败";
    std::cerr << "[dispatch] " << msg << "，上报 " << dispatch_msg::ERR_CAMERA
              << " 后重新申请执行\n";
    board_error(msg);
    board_note("warn", "上报解算失败，重新申请执行任务");
    dispatch_report_state(client, dispatch_msg::ERR_CAMERA);
    dispatch_report_state(client, dispatch_msg::WORKING);
}

int dispatch_reapply_after_qr(
    RobotClient &client, const std::string &request, const std::string &grant,
    const char *what, const std::function<bool()> &apply_fn)
{
    dispatch_report_qr_fail(client, what);
    if (!dispatch_apply(request, grant, what, apply_fn))
    {
        std::cout << "[dispatch] " << what
                  << " 解算失败后重新申请 20s 未允许，原地待命\n";
        return kDispatchHoldStay;
    }
    std::cout << "[dispatch] " << what << " 已重新获准，再次执行\n";
    return 0;
}

int dispatch_wait_system(RobotClient &client, const std::string &state)
{
    std::cerr << "[dispatch] 异常 " << state << "，停止动作，保持连接等待系统处理\n";
    board_error("异常 " + state + "（" + dispatch_state_zh(state) + "），停止动作，等待系统处理");
    board_activity("异常停机，等待系统处理");
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
    if (note == DispatchNote::Mission)
        board_step("汇报完成");
    board_report_pending(state, what);
    hardware_abort_sleep_ms(2000);
    if (hardware_abort_requested())
        return -4;
    const bool ok = note == DispatchNote::PlaceHand ? client.place_done()
                    : note == DispatchNote::PickHand ? client.pick_up_done()
                                                     : client.mission_done();
    board_report(state, what, ok);
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
    board_report_pending(state, what);
    if (remain > 0)
        hardware_abort_sleep_ms(remain);
    if (hardware_abort_requested())
        return -4;
    const bool ok = note == DispatchNote::PickHand ? client.pick_up_done() : client.mission_done();
    board_report(state, what, ok);
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
    (void)client;
    std::cout << "[dispatch] 发送 " << name << "\n";
    const std::string n = name;
    board_report(n, n == dispatch_msg::RAW_MATERIAL_EMPTY        ? "料盘一已空"
                    : n == dispatch_msg::WELL_DONE_MATERIAL_FULL ? "料盘二已满"
                                                                 : n,
                 sent);
    if (sent && n == dispatch_msg::RAW_MATERIAL_EMPTY)
        board_tray1_empty_reported();
    if (sent && n == dispatch_msg::WELL_DONE_MATERIAL_FULL)
        board_tray2_full_reported();
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
    std::cout << "[dispatch] 上次放下后空孔 右=" << g_unload_empty_right
              << " 左=" << g_unload_empty_left << "\n";
    if (g_unload_empty_right + g_unload_empty_left <= 0)
    {
        std::cout << "[dispatch] 上次记录料盘已满，这次按新空盘抓两只手\n";
        return;
    }
    // 每只手只能放到本侧空孔，所以按侧分配，不看总数。
    pick_right = g_unload_empty_right > 0;
    pick_left = g_unload_empty_left > 0;
    if (pick_right && pick_left)
        std::cout << "[dispatch] 左右两侧都有空孔，这次抓两只手\n";
    else
    {
        std::cout << "[dispatch] 只有" << (pick_right ? "右" : "左")
                  << "侧有空孔，这次只抓" << (pick_right ? "右手" : "左手") << "\n";
        board_note("info", std::string("料盘二只有") + (pick_right ? "右" : "左") +
                               "侧有空孔，这次只抓" + (pick_right ? "右手" : "左手"));
    }
}

int dispatch_pick_one(
    RobotClient &client, bool (*apply_pick)(RobotClient &, int),
    const std::string &request_state, const std::string &grant_state,
    bool from_place, const ConveyorStationConfig &c, const std::string &station,
    const char *tag, int only_hand, const char *done_what,
    const std::function<void()> &on_lift = {},
    bool photo_settle = true,
    bool grant_done = false,
    bool grant_ok = false,
    const std::string &depart_after_right = {})
{
    if (grant_done && !grant_ok)
    {
        std::cout << "[dispatch] " << done_what
                  << " 申请已在上一只手上抬时被拒绝，原地待命\n";
        return kDispatchHoldStay;
    }
    auto raw_apply = [&]() { return apply_pick(client, kDispatchApplyTimeoutMs); };
    board_step(only_hand == 1 ? "右手申请抓取" : "左手申请抓取");
    bool need_apply = !grant_done;
    if (grant_done)
        std::cout << "[dispatch] " << done_what << " 申请已在上一只手上抬期间完成\n";
    for (int attempt = 0;; ++attempt)
    {
        if (need_apply)
        {
            std::cout << "[dispatch] 发送 " << request_state << "，等待 " << grant_state
                      << "，超时 " << kDispatchApplyTimeoutMs << " ms\n";
            if (!dispatch_apply(request_state, grant_state, done_what, raw_apply))
            {
                std::cout << "[dispatch] " << done_what
                          << " 申请 20s 未允许，原地待命，不继续抓取\n";
                return kDispatchHoldStay;
            }
            std::cout << "[dispatch] 已收到 " << grant_state << "，" << done_what << " 放行\n";
        }
        need_apply = true;
        board_activity(std::string(done_what) + "：传送带拍照、伸手、合爪");
        const int rc = go_belt_grasp(
            from_place, c, station, tag, only_hand,
            [&](bool) {
                const auto closed_at = std::chrono::steady_clock::now();
                return dispatch_report_after(client, DispatchNote::PickHand, done_what, closed_at);
            },
            photo_settle, only_hand == 1 ? depart_after_right : std::string(),
            [&](bool) {
                if (on_lift)
                    on_lift();
            });
        if (rc == kBeltGraspEmpty)
        {
            std::cerr << "[dispatch] " << done_what
                      << " 重抓后仍空抓，不汇报 PICK_UP_DONE，原地待命\n";
            return kDispatchHoldStay;
        }
        if (rc != kVisionQrFail)
            return rc;
        if (attempt >= kDispatchQrRetry)
        {
            std::cerr << "[dispatch] " << done_what << " 二维码解算连续失败 "
                      << (attempt + 1) << " 次，停止重试\n";
            return kVisionQrFail;
        }
        const int re_rc = dispatch_reapply_after_qr(
            client, request_state, grant_state, done_what, raw_apply);
        if (re_rc != 0)
            return re_rc;
        // 已在 reapply 里申请成功，下一轮不再重复申请。
        need_apply = false;
        from_place = true;
        photo_settle = true;
    }
}

int dispatch_place_belt(
    RobotClient &client, bool (*apply_place)(RobotClient &, int),
    const char *request_state, const char *grant_state,
    const ConveyorStationConfig &c, const char *tag,
    bool hold_right, bool hold_left, const char *place_what,
    bool skip_ready = false, bool already_granted = false)
{
    if (!hold_right && !hold_left)
    {
        std::cout << "[dispatch] 手里没有件，跳过" << place_what << "\n";
        return 0;
    }
    auto raw_apply = [&]() { return apply_place(client, kDispatchApplyTimeoutMs); };
    bool need_apply = !already_granted;
    if (already_granted)
        std::cout << "[dispatch] " << place_what << " 的 " << grant_state
                  << " 已在料盘抓完后收到，底盘停稳后再拍照放置\n";
    for (int attempt = 0;; ++attempt)
    {
        if (need_apply)
        {
            std::cout << "[dispatch] 发送 " << request_state << "，等待 " << grant_state
                      << "，超时 " << kDispatchApplyTimeoutMs << " ms\n";
            if (!dispatch_apply(request_state, grant_state, place_what, raw_apply))
            {
                std::cout << "[dispatch] " << place_what
                          << " 申请 20s 未允许，原地待命，不放置，不报完成\n";
                return kDispatchHoldStay;
            }
            std::cout << "[dispatch] 已收到 " << grant_state << "，" << place_what
                      << " 放行\n";
        }
        need_apply = true;
        bool reported = false;
        auto report_on_last_lift = [&](bool) -> int {
            if (reported)
                return 0;
            reported = true;
            std::cout << "[dispatch] " << place_what
                      << " 最后一手上抬，立刻发送 "
                      << dispatch_msg::PLACE_DONE << " 和 " << dispatch_msg::DONE << "\n";
            board_step("汇报完成");
            const bool place_ok = client.place_done();
            board_report(dispatch_msg::PLACE_DONE, place_what, place_ok);
            if (!place_ok)
            {
                std::cerr << "[dispatch] 发送 " << dispatch_msg::PLACE_DONE << " 失败\n";
                return -7;
            }
            std::cout << "[dispatch] 已发送 " << dispatch_msg::PLACE_DONE << "\n";
            const bool done_ok = client.mission_done();
            board_report(dispatch_msg::DONE, "任务完成", done_ok);
            if (!done_ok)
            {
                std::cerr << "[dispatch] 发送 " << dispatch_msg::DONE << " 失败\n";
                return -7;
            }
            std::cout << "[dispatch] 已发送 " << dispatch_msg::DONE << "\n";
            return 0;
        };
        board_step(std::string(tag) == "belt2" ? "传送带二放置" : "传送带一放置");
        board_activity(std::string(place_what) + "：拍照认码、依次放置");
        const int place_rc = go_belt_station(
            c, tag, report_on_last_lift, hold_right, hold_left,
            attempt == 0 ? skip_ready : true);
        if (place_rc == kVisionQrFail)
        {
            if (attempt >= kDispatchQrRetry)
            {
                std::cerr << "[dispatch] " << place_what << " 二维码解算连续失败 "
                          << (attempt + 1) << " 次，手里有件，原地待命\n";
                return kDispatchHoldStay;
            }
            const int re_rc = dispatch_reapply_after_qr(
                client, request_state, grant_state, place_what, raw_apply);
            if (re_rc != 0)
                return re_rc;
            need_apply = false;
            continue;
        }
        if (place_rc != 0)
            return place_rc;
        if (!reported)
            return report_on_last_lift(false);
        return 0;
    }
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

int dispatch_pick_pair(
    RobotClient &client, bool (*apply_pick)(RobotClient &, int),
    const std::string &request_state, const std::string &grant_state,
    const ConveyorStationConfig &c, const std::string &station, const char *tag,
    bool do_left, bool do_right, const char *left_what, const char *right_what,
    const std::string &depart_after_right = {})
{
    // 一只手整段抓完（合爪+上抬+收手）后再申请下一只，不再与抬升重叠申请。
    if (do_left)
    {
        const int left_rc = dispatch_pick_one(
            client, apply_pick, request_state, grant_state,
            true, c, station, tag, 0, left_what);
        if (left_rc != 0)
            return left_rc;
    }
    if (do_right)
    {
        if (do_left)
            std::cout << "[dispatch] 左手已抓完，开始申请右手\n";
        const int right_rc = dispatch_pick_one(
            client, apply_pick, request_state, grant_state,
            true, c, station, tag, 1, right_what,
            {}, true, false, false, depart_after_right);
        if (right_rc != 0)
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
    board_step("去传送带二并申请放置");
    std::thread apply_th([&]() {
        std::cout << "[dispatch] 去 " << station << " 的同时发送 " << request_state
                  << "，等待 " << grant_state << "\n";
        granted = dispatch_apply(request_state, grant_state, place_what,
                                 [&]() { return apply_place(client, kDispatchApplyTimeoutMs); });
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
                  << " 申请 20s 未允许，原地待命，不放置，不报完成\n";
        return kDispatchHoldStay;
    }
    std::cout << "[dispatch] 已收到 " << grant_state << "，" << place_what << " 放行\n";
    auto raw_apply = [&]() { return apply_place(client, kDispatchApplyTimeoutMs); };
    for (int attempt = 0;; ++attempt)
    {
        board_step("传送带二放置");
        board_activity(std::string(place_what) + "：拍照认码、依次放置");
        const int place_rc = go_belt_station(c, tag, {}, hold_right, hold_left, true);
        if (place_rc == kVisionQrFail)
        {
            if (attempt >= kDispatchQrRetry)
            {
                std::cerr << "[dispatch] " << place_what << " 二维码解算连续失败 "
                          << (attempt + 1) << " 次，手里有件，原地待命\n";
                return kDispatchHoldStay;
            }
            const int re_rc = dispatch_reapply_after_qr(
                client, request_state, grant_state, place_what, raw_apply);
            if (re_rc != 0)
                return re_rc;
            continue;
        }
        if (place_rc != 0)
            return place_rc;
        break;
    }
    std::cout << "[dispatch] " << place_what << " 结束，立刻发送 "
              << dispatch_msg::PLACE_DONE << "\n";
    const bool place_ok = client.place_done();
    board_report(dispatch_msg::PLACE_DONE, place_what, place_ok);
    if (!place_ok)
    {
        std::cerr << "[dispatch] 发送 " << dispatch_msg::PLACE_DONE << " 失败\n";
        return -7;
    }
    std::cout << "[dispatch] 已发送 " << dispatch_msg::PLACE_DONE << "\n";
    return 0;
}

bool g_body_disabled_for_charge = false;

int set_body_motors(bool enable)
{
    uint32_t ids[22];
    int n = 0;
    auto push = [&](uint32_t id) { ids[n++] = id; };
    for (uint32_t id : kWaistAllCanIds)
        push(id);
    if (!right_arm_motors_locked())
    {
        for (uint32_t id : kRightArmCanIds)
            push(id);
    }
    for (uint32_t id : kLeftArmCanIds)
        push(id);
    for (uint32_t id : kHeadCanIds)
        push(id);
    std::cout << "[dispatch] " << (enable ? "使能" : "失能") << "全身电机，共 " << n << " 轴\n";
    if (enable)
    {
        bool ok = true;
        for (int i = 0; i < n; ++i)
            ok = motor_enable_id(ids[i]) && ok;
        if (!ok)
        {
            std::cerr << "[dispatch] 使能全身电机失败\n";
            return -5;
        }
        g_body_disabled_for_charge = false;
        return 0;
    }
    if (socketcan_sendsimplecommand(0, n, ids, 2, nullptr) != 1)
    {
        std::cerr << "[dispatch] 失能全身电机失败\n";
        return -5;
    }
    g_body_disabled_for_charge = true;
    return 0;
}

int stop_chassis_charging()
{
    if (!ensure_chassis())
        return -6;
    const int rc = chassis_seer().SetChargingRelay("{\"status\":false}");
    if (rc != 0)
    {
        std::cerr << "[dispatch] 停止充电失败 code=" << rc << " "
                  << chassis_seer().LastMessage() << "\n";
        return rc < 0 ? rc : -6;
    }
    std::cout << "[dispatch] 已调用停止充电\n";
    return 0;
}

std::string resolve_charge_station()
{
    if (!RobotClient::charge_point.empty())
        return RobotClient::charge_point;
    return g_move_cfg.chassis.charge_station;
}

/** client 非空：调度充电（等到 charge_done_percent 再 mission_done 并停充）。
 *  stop_when_charging：调试用，检测到开始充电就成功返回，不停充。 */
int run_charge_flow(RobotClient *client, bool stop_when_charging)
{
    const std::string station = resolve_charge_station();
    if (station.empty())
    {
        std::cerr << "[dispatch] 充电任务缺少充电点（调度 charge_point 或 yaml chassis.charge_station），"
                     "不动作\n";
        return kDispatchPointsMissing;
    }
    if (g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
    {
        std::cerr << "[dispatch] 手臂未初始化，不充电\n";
        return -1;
    }
    const std::string wait_ap =
        g_move_cfg.chassis.dispatch_wait_station.empty()
            ? g_move_cfg.chassis.tray_station
            : g_move_cfg.chassis.dispatch_wait_station;

    // 先报去充电（调度），再整机 home → AP14 → 挥手下垂 → 腰降抓取高，最后才失能去充电点。
    // 结束电量阈值每秒从 g_move_cfg 重读，网页改参 reload 后本次充电即可生效。
    std::cout << "[dispatch] 充电："
              << (stop_when_charging
                      ? "调试到吸合即停"
                      : ("调度到 " +
                         std::to_string(static_cast<int>(
                             std::lround(g_move_cfg.chassis.charge_done_percent))) +
                         "%（可热改）"))
              << "，home → " << wait_ap << " → 挥手下垂 → 腰降抓取高，再去 " << station << "\n";
    if (client != nullptr)
    {
        dispatch_report_state(*client, dispatch_msg::GO_CHARGE);
        board_step("上报去充电");
    }
    board_activity("去充电：整机回 home");

    board_step("整机 home");
    const int home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
    if (home_rc != 0)
    {
        std::cerr << "[dispatch] 充电前 home 失败，不继续\n";
        return home_rc;
    }
    if (hardware_abort_requested())
        return -4;

    board_step("去等待点");
    board_activity("去充电：导航到 " + wait_ap);
    std::cout << "[dispatch] 充电前先到等待点 " << wait_ap << "\n";
    {
        ChassisTravel travel(wait_ap);
        const int wait_rc = travel.join();
        if (wait_rc != 0)
            return wait_rc;
    }
    if (hardware_abort_requested())
        return -4;

    board_step("挥手下垂");
    board_activity("去充电：双手挥手前下垂");
    std::cout << "[dispatch] 双手进入挥手前下垂姿态\n";
    const int hang_rc = arm_wave_hang(*g_hw->arm_r, *g_hw->arm_l);
    if (hang_rc != 0)
    {
        std::cerr << "[dispatch] 挥手前下垂失败 code=" << hang_rc << "，不充电\n";
        return hang_rc == -4 || hardware_abort_requested() ? -4 : -5;
    }
    if (hardware_abort_requested())
        return -4;

    board_step("腰降抓取高");
    board_activity("去充电：腰降到抓取高度");
    std::cout << std::fixed << std::setprecision(4)
              << "[dispatch] 腰降到抓取高度 x=" << waist_ready_x()
              << " z=" << waist_ready_z() << "\n";
    if (g_hw->waist == nullptr || g_hw->layer3 == nullptr ||
        !move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
    {
        std::cerr << "[dispatch] 腰未能降到抓取高度，不充电\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (hardware_abort_requested())
        return -4;

    board_step("失能电机");
    board_activity("去充电：失能后导航到充电点");
    if (set_body_motors(false) != 0)
    {
        set_body_motors(true);
        return -5;
    }
    struct ReenableMotors
    {
        ~ReenableMotors()
        {
            if (g_body_disabled_for_charge)
                set_body_motors(true);
        }
    } reenable;
    // AP14 已准备好，底盘出发去充电点：上报 commuting。
    board_step("去充电路上");
    if (client != nullptr)
        dispatch_report_state(*client, dispatch_msg::COMMUTING);
    std::cout << "[dispatch] "
              << (client != nullptr ? std::string("上报 ") + dispatch_msg::COMMUTING + "，" : "")
              << "导航去充电点 " << station << "\n";
    board_step("去充电点");
    board_activity("去充电路上 → " + station);
    const int nav = chassis_goto_station(station);
    if (nav != 0)
        return nav;
    board_step(stop_when_charging ? "等待吸合" : "充电中");
    if (stop_when_charging)
        std::cout << "[dispatch] 已到充电点 " << station << "，等待检测到开始充电后结束指令\n";
    else
        std::cout << "[dispatch] 已到充电点 " << station
                  << "，每秒读电量与 charge_done_percent，达阈值后汇报完成并停止充电\n";
    int shown_pct = -1;
    int board_pct = -100;
    int shown_done_pct = -1;
    bool reported_charging = false;
    double last_pct = 0.0;
    double last_done_pct = g_move_cfg.chassis.charge_done_percent;
    while (!hardware_abort_requested())
    {
        if (!ensure_chassis())
        {
            hardware_abort_sleep_ms(1000);
            continue;
        }
        double level = 0, voltage = 0, current = 0, temp = 0;
        bool charging = false;
        int cycle = 0;
        if (chassis_seer().GetBattery(level, charging, voltage, current, temp, cycle) != 0)
        {
            hardware_abort_sleep_ms(1000);
            continue;
        }
        publish_dispatch_battery(level);
        if (charging && !reported_charging)
        {
            reported_charging = true;
            std::cout << "[dispatch] 检测到正在充电";
            if (client != nullptr)
            {
                std::cout << "，上报 " << dispatch_msg::CHARGING << "\n";
                dispatch_report_state(*client, dispatch_msg::CHARGING);
            }
            else
            {
                std::cout << "\n";
            }
            board_activity("充电中（已吸合）");
            if (stop_when_charging)
            {
                board_step("检测到充电");
                std::cout << "[dispatch] 调试充电：已开始充电，指令结束（不停充）\n";
                return 0;
            }
        }
        const double pct = level <= 1.5 ? level * 100.0 : level;
        last_pct = pct;
        const double done_pct = g_move_cfg.chassis.charge_done_percent;
        last_done_pct = done_pct;
        const int pct_i = static_cast<int>(std::lround(pct));
        const int done_i = static_cast<int>(std::lround(done_pct));
        if (done_i != shown_done_pct)
        {
            shown_done_pct = done_i;
            std::cout << "[dispatch] 充电结束阈值 " << done_i << "%\n";
        }
        if (pct_i != shown_pct)
        {
            shown_pct = pct_i;
            std::cout << "[dispatch] 充电中 电量 " << pct_i << "%（阈值 " << done_i << "%）"
                      << (charging ? "，充电继电器吸合" : "，尚未充电") << "\n";
            if (std::abs(pct_i - board_pct) >= 5)
            {
                board_pct = pct_i;
                board_activity(std::string(charging ? "充电中" : "等待充电吸合") + " 电量 " +
                               std::to_string(pct_i) + "% / 阈值 " + std::to_string(done_i) + "%");
            }
        }
        if (!stop_when_charging && pct >= done_pct)
            break;
        hardware_abort_sleep_ms(1000);
    }
    if (hardware_abort_requested())
        return -4;
    if (client == nullptr)
        return -5;
    std::cout << "[dispatch] 电量 " << static_cast<int>(std::lround(last_pct)) << "% 达到阈值 "
              << static_cast<int>(std::lround(last_done_pct)) << "%，汇报充电完成\n";
    board_step("汇报完成");
    const bool reported = client->mission_done();
    board_report(dispatch_msg::DONE, "充电完成", reported);
    if (!reported)
        std::cerr << "[dispatch] 汇报充电完成失败，仍停止充电\n";
    else
        std::cout << "[dispatch] 已汇报充电完成\n";
    const int stop = stop_chassis_charging();
    if (!reported)
        return -7;
    return stop;
}

int dispatch_charge(RobotClient &client)
{
    return run_charge_flow(&client, false);
}

int dispatch_load(RobotClient &client)
{
    const TaskPoints points = read_task_points();
    if (!points.raw.empty())
        g_dispatch_rack_point = points.raw;
    if (!task_points_ready(dispatch_msg::TASK_LOAD, points))
        return kDispatchPointsMissing;
    StationOverride stations(g_move_cfg.chassis);
    g_move_cfg.chassis.tray_station = points.raw;
    g_move_cfg.chassis.belt_station = points.load;
    std::cout << "[dispatch] 上料：料架 " << points.raw << " 抓件，抓完最后的零件就报料盘空；"
              << "料盘抓完就申请上料点 " << points.load
              << "，底盘到传送带一停稳后再拍照放置，然后发 "
              << dispatch_msg::DONE << "\n";
    struct PlaceGrant
    {
        std::atomic<bool> started{false};
        std::atomic<bool> ok{false};
        std::thread th;
        ~PlaceGrant()
        {
            if (th.joinable())
                th.join();
        }
    } grant;
    struct ClearDepartHook
    {
        ~ClearDepartHook() { g_on_grasp_depart = nullptr; }
    } clear_depart;
    g_on_grasp_depart = [&]() {
        if (grant.started.exchange(true))
            return;
        grant.th = std::thread([&client, &grant]() {
            std::cout << "[dispatch] 料盘一已抓完，去传送带一的同时发送 "
                      << dispatch_msg::APPLY_PLACE_RAW << "，等待 "
                      << dispatch_msg::PLACE << "\n";
            grant.ok = dispatch_apply(
                dispatch_msg::APPLY_PLACE_RAW, dispatch_msg::PLACE, "上料放置",
                [&]() { return client.apply_place_raw_material(kDispatchApplyTimeoutMs); });
        });
    };
    int grasp_rc = 0;
    for (int attempt = 0;; ++attempt)
    {
        grasp_rc = vision_grasp_existing_pipeline();
        if (grasp_rc != kVisionQrFail)
            break;
        if (attempt >= kDispatchQrRetry)
        {
            std::cerr << "[dispatch] 上料抓取 料盘二维码解算连续失败 "
                      << (attempt + 1) << " 次，停止重试\n";
            break;
        }
        dispatch_report_qr_fail(client, "上料抓取");
        board_activity("料盘码解算失败，重新拍照识别");
    }
    g_on_grasp_depart = nullptr;
    if (grasp_rc < 0)
        return grasp_rc;
    if (grasp_rc == kVisionQrFail)
        return kVisionQrFail;
    const bool hold_r = g_hw != nullptr && g_hw->last_grasped_r;
    const bool hold_l = g_hw != nullptr && g_hw->last_grasped_l;
    const int took = (hold_r ? 1 : 0) + (hold_l ? 1 : 0);
    if (g_load_parts_seen == 0 || (g_load_parts_seen > 0 && took >= g_load_parts_seen))
    {
        std::cout << "[dispatch] 上料盘已经没有零件（开抓前 "
                  << g_load_parts_seen << " 件，这次拿走 " << took << " 件）\n";
        move_box::tray1_far_band_clear();
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
    bool granted = false;
    if (grant.started)
    {
        board_step("申请上料放置");
        if (grant.th.joinable())
            grant.th.join();
        granted = grant.ok.load();
        if (!granted)
        {
            std::cout << "[dispatch] 上料放置申请 20s 未允许，原地待命，不放置，不报完成\n";
            return kDispatchHoldStay;
        }
        std::cout << "[dispatch] 已收到 " << dispatch_msg::PLACE
                  << "，等传送带一停稳后再拍照放置\n";
    }
    const bool pose_ready = station_name_matches(
        g_photo_ready_station, g_move_cfg.chassis.belt_station);
    const int place_rc = dispatch_place_belt(
        client, [](RobotClient &c, int timeout_ms) { return c.apply_place_raw_material(timeout_ms); },
        dispatch_msg::APPLY_PLACE_RAW, dispatch_msg::PLACE,
        g_move_cfg.conveyor, "belt", hold_r, hold_l, "上料放置", pose_ready, granted);
    if (place_rc != 0)
        return place_rc;
    std::cout << "[dispatch] 上料完成，停在当前位置\n";
    return 0;
}

int dispatch_transfer(RobotClient &client)
{
    const TaskPoints points = read_task_points();
    if (!points.raw.empty())
        g_dispatch_rack_point = points.raw;
    if (!task_points_ready(dispatch_msg::TASK_TRANSFER, points))
        return kDispatchPointsMissing;
    StationOverride stations(g_move_cfg.chassis);
    g_move_cfg.chassis.belt_station = points.load;
    g_move_cfg.chassis.out_station = points.unload;
    std::cout << "[dispatch] 转运：" << points.load
              << " 左右手各申请 20s，超时原地待命；有件才去 " << points.unload
              << " 放，然后发 " << dispatch_msg::DONE << "\n";
    board_step("去传送带一");
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
    const TaskPoints points = read_task_points();
    if (!points.raw.empty())
        g_dispatch_rack_point = points.raw;
    if (!task_points_ready(dispatch_msg::TASK_UNLOAD, points))
        return kDispatchPointsMissing;
    StationOverride stations(g_move_cfg.chassis);
    g_move_cfg.chassis.out_station = points.unload;
    g_move_cfg.chassis.tray2_station = points.well;
    std::cout << "[dispatch] 下料：" << points.unload
              << " 按上次剩下的空孔抓 1 或 2 只手，申请 20s 超时原地待命；放到 "
              << points.well << " 后若没有空孔就报满盘，再发 "
              << dispatch_msg::DONE << "\n";
    board_step("去传送带二");
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
        int place_rc = 0;
        for (int attempt = 0;; ++attempt)
        {
            place_rc = vision_place_tray2(hold_r, hold_l, false);
            if (place_rc != kVisionQrFail)
                break;
            if (attempt >= kDispatchQrRetry)
            {
                std::cerr << "[dispatch] 下料放置 料盘二维码解算连续失败 "
                          << (attempt + 1) << " 次，手里有件，原地待命\n";
                return kDispatchHoldStay;
            }
            dispatch_report_qr_fail(client, "下料放置");
            board_activity("料盘码解算失败，重新拍照找空孔");
        }
        if (place_rc < 0)
            return place_rc;
        if (g_unload_empty_known && g_unload_empty_right + g_unload_empty_left <= 0)
        {
            std::cout << "[dispatch] 下料盘已经没有空孔\n";
            board_tray2(0, 0);
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

    void join_worker()
    {
        if (worker.joinable())
            worker.join();
        running = false;
    }

    ~DispatchInbox()
    {
        join_worker();
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
    box.join_worker();
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

int run_charge_test()
{
    board_mission_begin("CHARGE_TEST");
    const int rc = run_charge_flow(nullptr, true);
    if (rc == 0)
        board_mission_end(true, "已开始充电");
    else if (rc == -4 || hardware_abort_requested())
        board_mission_end(false, "已中止");
    else if (rc == kDispatchPointsMissing)
        board_mission_end(false, "充电点未配置");
    else
        board_mission_end(false, "充电测试失败 code=" + std::to_string(rc));
    return rc;
}

void start_chassis_status_reporter()
{
    std::thread(chassis_status_loop).detach();
    std::cout << "[orch] 底盘状态上报已启动，2Hz 写入位置和电量\n";
}

static int run_dispatch_mode_body()
{
    if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
        return -1;
    const auto &ch = g_move_cfg.chassis;
    if (ch.robot_type != "T170C2.0" && ch.robot_type != "T230C")
    {
        std::cerr << "[dispatch] robot_type 只能是 T170C2.0 或 T230C，当前 "
                  << ch.robot_type << "\n";
        return -1;
    }
    if (ch.sequence_number.empty())
    {
        std::cerr << "[dispatch] sequence_number 为空\n";
        return -1;
    }
    std::cout << "[dispatch] 系统对接模式，调度 "
              << dispatch_msg::kDefaultHost << ":" << dispatch_msg::kDefaultPort
              << " 型号 " << ch.robot_type << " 编号 " << ch.sequence_number
              << "。底盘点位由系统下发，原地先 home，再到 "
              << ch.dispatch_wait_station << " 等任务\n";
    // client 必须活过 inbox 线程；退出时先拆连接，apply_mission 才会返回，否则 join 卡死、
    // board_end 迟迟不跑，网页一直显示「调度中」。
    auto client_holder = std::make_unique<RobotClient>(ch.robot_type, ch.sequence_number);
    RobotClient &client = *client_holder;
    struct BodyMotorRestore
    {
        ~BodyMotorRestore()
        {
            if (g_body_disabled_for_charge)
                set_body_motors(true);
        }
    } body_restore;
    DispatchInbox box;
    struct WakeInboxBeforeJoin
    {
        std::unique_ptr<RobotClient> &holder;
        DispatchInbox &box;
        ~WakeInboxBeforeJoin()
        {
            // 先拆 RobotClient，卡住的 apply_mission 才会返回，再 join inbox 线程。
            if (holder)
            {
                std::cout << "[dispatch] 退出调度：断开调度连接，解除等待任务\n";
                holder.reset();
            }
            box.join_worker();
        }
    } wake_inbox{client_holder, box};
    std::string mission;
    std::string wait_station = ch.dispatch_wait_station.empty()
                                   ? g_move_cfg.chassis.tray_station
                                   : ch.dispatch_wait_station;
    auto take_ready = [&]() -> bool {
        if (!inbox_poll(box, mission))
            return false;
        inbox_join(box);
        return true;
    };
    // 到待命点后：途中缓存的任务立刻取出执行；没有则清掉界面上的旧「收到指令」。
    auto take_cached_or_standby = [&]() {
        g_dispatch_stay_at_tray2 = false;
        if (g_hw != nullptr && !g_hw->last_chassis_station.empty())
            wait_station = g_hw->last_chassis_station;
        dispatch_report_state(client, dispatch_msg::STANDBY);
        std::string cached;
        if (inbox_poll(box, cached))
        {
            inbox_join(box);
            if (!cached.empty())
            {
                mission = std::move(cached);
                std::cout << "[dispatch] 到 " << wait_station << " 发现缓存任务 " << mission
                          << "，立刻执行\n";
                board_activity("到 " + wait_station + "，执行缓存任务");
                return;
            }
            // apply_mission 空返回（断线）：下面重新听
        }
        mission.clear();
        board_clear_command();
        board_activity("在 " + wait_station + " 等待任务");
        if (!box.running)
            inbox_start(box, client);
    };
    // 无任务 / 失败 / 断线：先 home，再去 AP14（dispatch_wait_station）待命。途中缓存任务。
    auto recover_home_ap14 = [&](const std::string &reason) -> int {
        move_box::tray1_far_band_clear();
        wait_station = ch.dispatch_wait_station.empty() ? g_move_cfg.chassis.tray_station
                                                       : ch.dispatch_wait_station;
        std::cout << "[dispatch] " << reason << " → home → " << wait_station << " 待命\n";
        board_activity(reason + "，回 home");
        if (g_body_disabled_for_charge)
        {
            if (set_body_motors(true) != 0)
                return dispatch_wait_system(client, dispatch_msg::ERR_MOTOR_DISABLE);
        }
        if (!box.running)
            inbox_start(box, client);
        dispatch_report_state(client, dispatch_msg::WORKING);
        const int home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
        if (home_rc != 0)
            return home_rc;
        if (hardware_abort_requested())
            return -4;
        board_activity("去等待点 " + wait_station);
        {
            ChassisTravel travel(wait_station);
            const int nav_rc = travel.join();
            if (nav_rc != 0)
                return nav_rc;
        }
        if (hardware_abort_requested())
            return -4;
        take_cached_or_standby();
        return 0;
    };
    // 充完电：手臂回 home_tcp、腰高不动，底盘去 charge_wait_station（默认 AP12）。
    auto recover_after_charge = [&]() -> int {
        wait_station = g_move_cfg.chassis.charge_wait_station.empty()
                           ? "AP12"
                           : g_move_cfg.chassis.charge_wait_station;
        std::cout << "[dispatch] 充电完成 → 手臂 home_tcp（腰高不变）→ " << wait_station
                  << " 待命\n";
        board_activity("充电完成，手臂回 home_tcp（腰高不变）");
        if (g_body_disabled_for_charge)
        {
            if (set_body_motors(true) != 0)
                return dispatch_wait_system(client, dispatch_msg::ERR_MOTOR_DISABLE);
        }
        if (!box.running)
            inbox_start(box, client);
        dispatch_report_state(client, dispatch_msg::WORKING);
        const int home_rc = go_home_arms_keep_waist_z(*g_hw->arm_r, *g_hw->arm_l);
        if (home_rc != 0)
            return home_rc;
        if (hardware_abort_requested())
            return -4;
        board_activity("去充电待命点 " + wait_station);
        {
            ChassisTravel travel(wait_station);
            const int nav_rc = travel.join();
            if (nav_rc != 0)
                return nav_rc;
        }
        if (hardware_abort_requested())
            return -4;
        take_cached_or_standby();
        return 0;
    };
    dispatch_report_state(client, dispatch_msg::STANDBY);
    {
        const int boot_rc = recover_home_ap14("调度启动");
        if (boot_rc != 0)
            return boot_rc;
    }
    while (!hardware_abort_requested())
    {
        if (mission.empty())
        {
            std::cout << "[dispatch] 在 " << wait_station << " 以 home 姿态等待任务\n";
            board_link("等待任务");
            board_clear_command();
            board_activity("在 " + wait_station + " 等待任务");
            if (!box.running)
                inbox_start(box, client);
            bool wait_reported_charging = false;
            auto poll_wait_ms = 0;
            while (!take_ready())
            {
                if (hardware_abort_requested())
                    return -4;
                // 待命时也看一眼有没有在充：吸合就报 CHARGING，断开再回到 STANDBY。
                poll_wait_ms += 100;
                if (poll_wait_ms >= 1000)
                {
                    poll_wait_ms = 0;
                    if (ensure_chassis())
                    {
                        double level = 0, voltage = 0, current = 0, temp = 0;
                        bool charging = false;
                        int cycle = 0;
                        if (chassis_seer().GetBattery(level, charging, voltage, current, temp,
                                                      cycle) == 0)
                        {
                            publish_dispatch_battery(level);
                            if (charging && !wait_reported_charging)
                            {
                                wait_reported_charging = true;
                                std::cout << "[dispatch] 待命时检测到充电，上报 "
                                          << dispatch_msg::CHARGING << "\n";
                                dispatch_report_state(client, dispatch_msg::CHARGING);
                                board_activity("充电中（待命）");
                            }
                            else if (!charging && wait_reported_charging)
                            {
                                wait_reported_charging = false;
                                dispatch_report_state(client, dispatch_msg::STANDBY);
                                board_activity("在 " + wait_station + " 等待任务");
                            }
                        }
                    }
                }
                hardware_abort_sleep_ms(100);
            }
        }
        if (hardware_abort_requested())
            return -4;
        if (mission.empty())
        {
            std::cerr << "[dispatch] 调度断线或没有任务，回 home → " << wait_station << "\n";
            board_link("断线，回 AP14 待命");
            const int r = recover_home_ap14("调度断线或无任务");
            if (r != 0)
                return r;
            continue;
        }
        std::cout << "[dispatch] 收到任务 " << mission << "\n";
        const std::string current = mission;
        mission.clear();
        board_command(current);
        if (current != dispatch_msg::TASK_LOAD && current != dispatch_msg::TASK_TRANSFER &&
            current != dispatch_msg::TASK_UNLOAD && current != dispatch_msg::TASK_CHARGE)
        {
            std::cerr << "[dispatch] 未知任务 " << current << "，回 home → AP14\n";
            board_note("warn", "未知指令 " + current + "，回 AP14 待命");
            const int r = recover_home_ap14("未知指令 " + current);
            if (r != 0)
                return r;
            continue;
        }
        board_mission_begin(current);
        const auto mission_t0 = std::chrono::steady_clock::now();
        auto spent_text = [&]() {
            const double s = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - mission_t0)
                                 .count();
            std::ostringstream os;
            os << std::fixed << std::setprecision(1) << s << " s";
            return os.str();
        };
        if (current != dispatch_msg::TASK_CHARGE && g_body_disabled_for_charge)
        {
            if (set_body_motors(true) != 0)
                return dispatch_wait_system(client, dispatch_msg::ERR_MOTOR_DISABLE);
        }
        g_hw->last_belt_message.clear();
        g_hw->last_tray_message.clear();
        g_dispatch_nav_fault.clear();
        g_dispatch_stay_at_tray2 = false;
        dispatch_report_state(client, dispatch_msg::WORKING);
        int rc = -1;
        if (current == dispatch_msg::TASK_CHARGE)
            rc = dispatch_charge(client);
        else if (current == dispatch_msg::TASK_LOAD)
            rc = dispatch_load(client);
        else if (current == dispatch_msg::TASK_TRANSFER)
            rc = dispatch_transfer(client);
        else
            rc = dispatch_unload(client);
        if (rc == -4 || hardware_abort_requested())
        {
            board_mission_end(false, "已中止");
            return -4;
        }
        if (current == dispatch_msg::TASK_CHARGE)
        {
            if (rc == kDispatchPointsMissing)
            {
                board_mission_end(false, "充电点未下发，未执行");
                const int r = recover_home_ap14("充电点未下发");
                if (r != 0)
                    return r;
                continue;
            }
            if (rc != 0)
            {
                board_mission_end(false, "充电未完成 code=" + std::to_string(rc));
                const int r = recover_home_ap14("充电未完成 code=" + std::to_string(rc));
                if (r != 0)
                    return r;
                continue;
            }
            board_mission_end(true, "充电完成，用时 " + spent_text());
            const int r = recover_after_charge();
            if (r != 0)
                return r;
            continue;
        }
        if (rc == kDispatchPointsMissing)
        {
            board_mission_end(false, "点位未下发齐，未执行");
            const int r = recover_home_ap14("点位未下发齐");
            if (r != 0)
                return r;
            continue;
        }
        if (rc != 0)
        {
            g_dispatch_stay_at_tray2 = false;
            const std::string fault = dispatch_exception_state();
            if (!fault.empty() &&
                (fault == dispatch_msg::ERR_MOTOR_LOST || fault == dispatch_msg::ERR_MOTOR_DISABLE ||
                 fault == dispatch_msg::ERR_GRIPPER_LOST))
            {
                board_mission_end(false, "异常 " + fault + "，等待系统处理");
                return dispatch_wait_system(client, fault);
            }
            const bool holding = g_hw != nullptr && (g_hw->last_grasped_r || g_hw->last_grasped_l);
            const std::string why =
                fault.empty()
                    ? ("任务失败 code=" + std::to_string(rc) +
                       (holding ? "（手里有件）" : "") +
                       (rc == kDispatchHoldStay ? "（申请超时/待命）" : "") +
                       (rc == kVisionQrFail ? "（二维码失败）" : ""))
                    : ("异常 " + fault);
            board_mission_end(false, why + "，回 AP14");
            const int r = recover_home_ap14(why);
            if (r != 0)
                return r;
            continue;
        }

        board_mission_end(true, "完成，用时 " + spent_text());
        dispatch_report_state(client, dispatch_msg::STANDBY);
        g_dispatch_stay_at_tray2 = false;
        if (g_hw != nullptr && !g_hw->last_chassis_station.empty())
            wait_station = g_hw->last_chassis_station;
        mission.clear();
        board_clear_command();
        std::cout << "[dispatch] 子任务完成，原地在 " << wait_station << " 听下一条 "
                  << (kDispatchNextMissionWaitMs / 1000) << "s；收到则连做，超时/断线再 home→AP14\n";
        board_activity("任务完成，原地等 " + std::to_string(kDispatchNextMissionWaitMs / 1000) +
                       "s 看有没有下一条");
        if (!box.running)
            inbox_start(box, client);
        std::string soon;
        if (inbox_wait(box, kDispatchNextMissionWaitMs, soon))
        {
            inbox_join(box);
            if (hardware_abort_requested())
                return -4;
            if (!soon.empty())
            {
                std::cout << "[dispatch] " << (kDispatchNextMissionWaitMs / 1000) << "s 内收到 "
                          << soon << "，原地执行\n";
                mission = std::move(soon);
                continue;
            }
            std::cerr << "[dispatch] 等待期间调度断线，home → AP14\n";
            const int r = recover_home_ap14("等待下一条时断线");
            if (r != 0)
                return r;
            continue;
        }
        if (hardware_abort_requested())
            return -4;
        // 仍连着但超时没任务：回 AP14 待命（途中若又来任务，到站后立刻执行）。
        const int r = recover_home_ap14(std::to_string(kDispatchNextMissionWaitMs / 1000) +
                                        "s 内无新任务");
        if (r != 0)
            return r;
    }
    return hardware_abort_requested() ? -4 : 0;
}

int run_dispatch_mode()
{
    const auto &ch = g_move_cfg.chassis;
    board_begin(std::string(dispatch_msg::kDefaultHost) + ":" +
                    std::to_string(dispatch_msg::kDefaultPort),
                ch.robot_type + " / " + ch.sequence_number);
    int rc = -4;
    {
        DispatchConsole console;
        rc = run_dispatch_mode_body();
    }
    board_end(rc);
    return rc;
}

} // namespace orch
