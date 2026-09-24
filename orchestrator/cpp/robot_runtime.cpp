#include "robot_runtime.h"

#include "function.h"
#include "head_cam2robot.h"
#include "head_control.h"
#include "move_box_config.h"
#include "move_box_runtime.h"
#include "seg_pose_bridge.h"
#include "Ti5_socketcan.h"
#include "Ti5_Seer.hpp"

#include <Eigen/Geometry>

#include <algorithm>
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

HwSession *g_hw = nullptr;

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
    if (!move_box::waist_layer3_move_x_only(*g_hw->waist, *g_hw->layer3, c.waist_x, true))
    {
        std::cerr << "[orch] " << name << " 腰平移 X 失败\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (hardware_abort_requested())
        return -4;
    if (!move_box::waist_layer3_move_z_only(*g_hw->waist, *g_hw->layer3, c.waist_z, true))
    {
        std::cerr << "[orch] " << name << " 腰升降 Z 失败\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (hardware_abort_requested())
        return -4;
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

int chassis_goto_station(const std::string &station)
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
        if (loc_rc == 0 && station_name_matches(cur, station))
        {
            std::cout << std::fixed << std::setprecision(3)
                      << "[orch] 底盘到达 " << cur << " x=" << x << " y=" << y << "\n";
            g_hw->last_chassis_ok = true;
            g_hw->last_chassis_station = cur;
            g_hw->last_chassis_message = "到位";
            robot.Disconnect();
            return 0;
        }

        int task_status = 0, task_type = 0;
        std::string target_id;
        double tx = 0, ty = 0, ta = 0, dist = 0;
        std::vector<std::string> finished, unfinished;
        const int nav_rc =
            robot.GetNavStatus(task_status, task_type, target_id, tx, ty, ta, finished, unfinished, dist);
        if (nav_rc == 0)
        {
            if (task_status == 4 && station_name_matches(target_id, station))
            {
                std::cout << "[orch] 底盘导航完成 target=" << target_id << "\n";
                g_hw->last_chassis_ok = true;
                g_hw->last_chassis_station = target_id;
                g_hw->last_chassis_message = "导航完成";
                robot.Disconnect();
                return 0;
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

} // namespace

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

int go_belt_station(const ConveyorStationConfig &c, const char *tag)
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

    const int ready_rc = go_belt_ready(*g_hw->arm_r, *g_hw->arm_l, c, name);
    if (ready_rc != 0)
        return ready_rc;
    if (aborted())
        return -4;
    std::cout << "[orch] " << name << " 底盘已到且准备姿态到位，停稳 1.5s 再拍照\n";
    hardware_abort_sleep_ms(1500);
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
    const Eigen::Matrix<double, 1, 6> goal_r = move_box::make_belt_goal_right(det, c);
    const Eigen::Matrix<double, 1, 6> goal_l = move_box::make_belt_goal_left(det, c);
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
        Robot_Arm &arm = is_right ? *g_hw->arm_r : *g_hw->arm_l;
        Eigen::Matrix<double, 1, 6> goal = is_right ? goal_r : goal_l;
        const char *place_tag = is_right ? "belt右手放置" : "belt左手放置";
        if (c.place_hover_above_m > 1e-9)
        {
            Eigen::Matrix<double, 1, 6> hover = goal;
            hover(2) += c.place_hover_above_m;
            std::cout << "[orch] belt " << (is_right ? "右手" : "左手")
                      << " 先到放置上方 " << c.place_hover_above_m << "m\n";
            if (!move_box::move_one_arm_line_to(
                    arm, is_right, hover, is_right ? "belt右手放置上方" : "belt左手放置上方"))
            {
                std::cerr << "[orch] belt " << (is_right ? "右手" : "左手") << " 走到上方失败\n";
                return aborted() ? -4 : -5;
            }
            if (aborted())
                return -4;
        }
        std::cout << "[orch] belt " << (is_right ? "右手" : "左手")
                  << " 下降到放置点（不锁J2，左右同一XYZ）\n";
        if (!move_box::move_one_arm_line_to(arm, is_right, goal, place_tag))
        {
            std::cerr << "[orch] belt " << (is_right ? "右手" : "左手") << " 走到放置点失败\n";
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
            std::cout << "[orch] belt " << (is_right ? "右手" : "左手") << " 放置点松爪\n";
            log_gripper_qs("放置松爪前");
            const bool opened = g_hw->g->openAndWait(
                is_right ? gripper::Side::Right : gripper::Side::Left,
                0.30,
                std::chrono::milliseconds(800));
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
        Eigen::Matrix<double, 1, 6> lift = arm_get_tcp_pos(arm);
        const double z0 = lift(2);
        lift(2) = z0 + kPlaceNudgeUpM;
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] belt " << (is_right ? "右手" : "左手")
                  << " 松爪后抬 " << kPlaceNudgeUpM << "m（z " << z0 << "→" << lift(2)
                  << "）再直接回准备\n";
        if (!move_box::move_one_arm_line_to(
                arm, is_right, lift, is_right ? "belt右手放置抬升" : "belt左手放置抬升"))
        {
            std::cerr << "[orch] belt " << (is_right ? "右手" : "左手") << " 放置后抬升失败\n";
            return aborted() ? -4 : -5;
        }
        if (aborted())
            return -4;
        Eigen::Matrix<double, 1, 6> ready = is_right ? c.tcp.right : c.tcp.left;
        const char *back_tag = is_right ? "belt右手回准备" : "belt左手回准备";
        std::cout << "[orch] belt " << (is_right ? "右手" : "左手") << " 回准备 tcp\n";
        if (!move_box::move_one_arm_line_to(arm, is_right, ready, back_tag))
        {
            std::cerr << "[orch] belt " << (is_right ? "右手" : "左手") << " 回准备失败\n";
            return aborted() ? -4 : -5;
        }
        return aborted() ? -4 : 0;
    };

    int right_back_rc = 0;
    if (!right_arm_motors_locked())
    {
        const int right_rc = place_arrive_and_open(true);
        if (right_rc != 0)
            return right_rc;
        std::thread th_r([&]() { right_back_rc = lift_and_retract(true); });
        std::cout << "[orch] belt 右手已放下，左手立刻过来（不等右手抬起回准备）\n";
        const int left_rc = place_arrive_and_open(false);
        const int left_back_rc = (left_rc == 0) ? lift_and_retract(false) : 0;
        th_r.join();
        if (left_rc != 0)
            return left_rc;
        if (left_back_rc != 0)
            return left_back_rc;
        if (right_back_rc != 0)
            return right_back_rc;
    }
    else
    {
        const int left_rc = place_arrive_and_open(false);
        if (left_rc != 0)
            return left_rc;
        const int left_back_rc = lift_and_retract(false);
        if (left_back_rc != 0)
            return left_back_rc;
    }

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
    std::cout << "[orch] belt 底盘已到 " << st << "，准备姿态后停稳 1.5s 再拍照\n";
    return go_belt_station(g_move_cfg.conveyor, "belt");
}

int go_belt_grasp(
    bool from_place, const ConveyorStationConfig &c, const std::string &station, const char *tag)
{
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

    std::cout << "[orch] belt_grasp 不转腰，停稳 1.5s 再拍照；grasp_offset 按码坐标系，允许过中轴\n";
    hardware_abort_sleep_ms(1500);
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
    g_hw->last_belt_ok = det.ok && det.have_grasp_l && det.have_grasp_r;
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

    if (!det.ok || !det.have_grasp_l || !det.have_grasp_r)
    {
        std::cerr << "[orch] belt_grasp 未拿到左右抓取点，不往码上走\n";
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
        g_hw->g->closeAndWait(
            gripper::Side::Left, grip_r, 0.15, std::chrono::milliseconds(3000));
        if (!right_arm_motors_locked())
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
        Robot_Arm &arm = is_right ? *g_hw->arm_r : *g_hw->arm_l;
        Eigen::Matrix<double, 1, 6> goal = move_box::make_belt_grasp_goal(
            is_right,
            is_right ? det.grasp_rx : det.grasp_lx,
            is_right ? det.grasp_ry : det.grasp_ly,
            is_right ? det.grasp_rz : det.grasp_lz,
            c);
        const char *hand = is_right ? "右手" : "左手";
        if (c.grasp_hover_above_m > 1e-9)
        {
            Eigen::Matrix<double, 1, 6> hover = goal;
            hover(2) += c.grasp_hover_above_m;
            std::cout << "[orch] belt_grasp " << hand << " 先到抓取上方\n";
            if (!move_box::move_one_arm_line_to(
                    arm, is_right, hover, is_right ? "belt_grasp右手上方" : "belt_grasp左手上方"))
            {
                std::cerr << "[orch] belt_grasp " << hand << " 走到上方失败\n";
                return aborted() ? -4 : -5;
            }
            if (aborted())
                return -4;
        }
        std::cout << "[orch] belt_grasp " << hand << " 去抓取点（不锁J2，左右同一XYZ）\n";
        if (!move_box::move_one_arm_line_to(
                arm, is_right, goal, is_right ? "belt_grasp右手抓取" : "belt_grasp左手抓取"))
        {
            std::cerr << "[orch] belt_grasp " << hand << " 走到抓取点失败\n";
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
                g_hw->last_grasped_r = true;
            else
                g_hw->last_grasped_l = true;
            if (aborted())
                return -4;
        }
        return aborted() ? -4 : 0;
    };

    auto grasp_lift_and_back = [&](bool is_right) -> int {
        if (is_right && right_arm_motors_locked())
            return 0;
        if (aborted())
            return -4;
        Robot_Arm &arm = is_right ? *g_hw->arm_r : *g_hw->arm_l;
        const char *hand = is_right ? "右手" : "左手";
        Eigen::Matrix<double, 1, 6> goal = move_box::make_belt_grasp_goal(
            is_right,
            is_right ? det.grasp_rx : det.grasp_lx,
            is_right ? det.grasp_ry : det.grasp_ly,
            is_right ? det.grasp_rz : det.grasp_lz,
            c);
        hardware_abort_sleep_ms(300);
        if (aborted())
            return -4;
        const double lift_z = c.grasp_hover_above_m > 1e-9
                                  ? c.grasp_hover_above_m
                                  : g_move_cfg.head_grasp.lift_after_grasp_z;
        if (lift_z > 1e-9)
        {
            Eigen::Matrix<double, 1, 6> lift = goal;
            lift(2) += lift_z;
            std::cout << "[orch] belt_grasp " << hand << " 合爪后先抬升 " << lift_z << "m\n";
            // 不锁 J2。锁肘时 8cm 抬升要靠 J6，第二传送带左手 J6 会顶在约 60° 上，
            // 停稳失败后原来直接返回，回 grasp_tcp 不会下发。
            if (!move_box::move_one_arm_line_to(
                    arm, is_right, lift, is_right ? "belt_grasp右手抬升" : "belt_grasp左手抬升",
                    false))
            {
                if (aborted())
                    return -4;
                std::cerr << "[orch] belt_grasp " << hand
                          << " 抬升未停稳，仍回 grasp_tcp\n";
            }
            if (aborted())
                return -4;
        }
        Eigen::Matrix<double, 1, 6> back = is_right ? c.grasp_tcp.right : c.grasp_tcp.left;
        std::cout << "[orch] belt_grasp " << hand << " 回 grasp_tcp\n";
        if (!move_box::move_one_arm_line_to(
                arm, is_right, back, is_right ? "belt_grasp右手回grasp_tcp" : "belt_grasp左手回grasp_tcp"))
        {
            std::cerr << "[orch] belt_grasp " << hand << " 回 grasp_tcp 失败\n";
            return aborted() ? -4 : -5;
        }
        return aborted() ? -4 : 0;
    };

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

    std::cout << "[orch] " << name << " 双手抓取结束，停在 grasp_tcp，可接入底盘/放置\n";
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
    std::cout << "[orch] belt2 底盘已到 " << st << "，准备姿态后停稳 1.5s 再拍照\n";
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

    std::cout << "[grasp] 先到料盘站 " << g_move_cfg.chassis.tray_station << "\n";
    const int tray_rc = chassis_goto_station(g_move_cfg.chassis.tray_station);
    if (tray_rc != 0)
        return tray_rc;
    if (should_abort())
        return -4;
    std::cout << "[grasp] 已在 " << g_move_cfg.chassis.tray_station << "，停稳 0.5s 再抓\n";
    hardware_abort_sleep_ms(500);
    if (should_abort())
        return -4;

    const int head_rc = enable_head_calib_pose();
    if (head_rc != 0)
        return head_rc;
    if (should_abort())
        return -4;

    std::cout << "[grasp] 进入抓取：双臂回 yaml standby（XYZ+姿态）"
              << (right_arm_motors_locked() ? "；右臂锁定只动左手\n" : "\n");
    if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l))
    {
        std::cerr << "[grasp] 进入时回到 standby 失败\n";
        return should_abort() ? -4 : -5;
    }
    if (should_abort())
        return -4;
    g_hw->g->openAndWait(gripper::Side::Right, 0.2, std::chrono::milliseconds(1500));
    g_hw->g->openAndWait(gripper::Side::Left, 0.2, std::chrono::milliseconds(1500));
    if (should_abort())
        return -4;

    if (g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[grasp] 腰未初始化，拒绝进入抓取\n";
        return -1;
    }
    std::cout << std::fixed << std::setprecision(4)
              << "[grasp] 进入前检查腰到位 x=" << waist_ready_x()
              << " z=" << waist_ready_z() << "，不到位不抓\n";
    if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
    {
        std::cerr << "[grasp] 腰未到 ready 起始位，放弃抓取\n";
        return should_abort() ? -4 : -5;
    }
    if (should_abort())
        return -4;
    move_box::waist_layer3_set_synced(true);

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
              << " 共 " << grasped_count << " 件，夹爪保持合上\n";
    return last_rc;
}

} // namespace

int vision_grasp_existing_pipeline()
{
    return vision_grasp_with_engine(SegEngineId::Default);
}

int vision_place_tray2()
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

    std::cout << "[tray2] 先到料盘2站 " << g_move_cfg.chassis.tray2_station << "\n";
    const int tray_rc = chassis_goto_station(g_move_cfg.chassis.tray2_station);
    if (tray_rc != 0)
        return tray_rc;
    if (should_abort())
        return -4;
    std::cout << "[tray2] 已在 " << g_move_cfg.chassis.tray2_station << "，停稳 0.5s 再放\n";
    hardware_abort_sleep_ms(500);
    if (should_abort())
        return -4;

    const int head_rc = enable_head_calib_pose();
    if (head_rc != 0)
        return head_rc;
    if (should_abort())
        return -4;

    std::cout << "[tray2] 进入放置：双臂回 yaml standby（XYZ+姿态）"
              << (right_arm_motors_locked() ? "；右臂锁定只动左手\n" : "\n");
    if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l))
    {
        std::cerr << "[tray2] 进入时回到 standby 失败\n";
        return should_abort() ? -4 : -5;
    }
    if (should_abort())
        return -4;

    if (g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[tray2] 腰未初始化，拒绝进入放置\n";
        return -1;
    }
    std::cout << std::fixed << std::setprecision(4)
              << "[tray2] 进入前检查腰到位 x=" << waist_ready_x()
              << " z=" << waist_ready_z() << "，不到位不放\n";
    if (!move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3))
    {
        std::cerr << "[tray2] 腰未到 ready 起始位，放弃放置\n";
        return should_abort() ? -4 : -5;
    }
    if (should_abort())
        return -4;
    move_box::waist_layer3_set_synced(true);

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
        return right_arm_motors_locked() ? placed_l : (placed_r && placed_l);
    };

    std::cout << "[tray2] 开始循环：YOLO class3 空孔，顺序同 grasp；到位后张爪"
              << (right_arm_motors_locked() ? "（右臂锁定，只要求左手放置）\n" : "\n");

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
            placed_r,
            placed_l);

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
              << " 共 " << placed_count << " 件，站起回 home_tcp（同料盘1抓好后）\n";
    std::cout.flush();

    if (should_abort())
        return -4;
    const int home_rc = go_home(*g_hw->arm_r, *g_hw->arm_l);
    if (home_rc != 0)
        return home_rc;
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
        const int bg_rc = go_belt_grasp(true);
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

        std::cout << "[orch] grasp_belt 抓取结束，底盘去 " << out << "\n";
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
            true, g_move_cfg.conveyor2, g_move_cfg.chassis.out_station, "belt2_grasp");
        if (bg2_rc != 0)
            return bg2_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] grasp_belt 第二传送带抓完，去料盘2空孔放置 "
                  << g_move_cfg.chassis.tray2_station << "\n";
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
                  << g_move_cfg.chassis.tray_station << " 再抓\n";
        const int back_rc = chassis_goto_station(g_move_cfg.chassis.tray_station);
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

} // namespace orch
