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

/** 转腰俯仰后再只拧回转电机会把躯干抽一下。五轴一起插回直立 home。 */
int waist_return_home_slow()
{
    if (hardware_abort_requested())
        return -4;
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return 0;
    const double x_home = g_move_cfg.waist.layer3_home(0);
    const double y_home = g_move_cfg.waist.layer3_home(1);
    const double z_home = g_move_cfg.waist.layer3_home(2);
    // 80° 回转的等效路程约 0.28 m，0.035 m/s 大约 8 秒。
    constexpr double kHomeReturnVel = 0.035;
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] home 腰五轴慢回直立 x=" << x_home
              << " y=" << y_home << " z=" << z_home
              << " yaw=0 pitch=0 vel=" << kHomeReturnVel << "\n";
    const int rc = g_hw->waist->moveToYawPitch(x_home, y_home, z_home, 0.0, 0.0, kHomeReturnVel);
    if (rc != 0)
    {
        std::cerr << "[orch] 腰慢回 home 失败 code=" << rc << "\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "home慢回后"))
        return hardware_abort_requested() ? -4 : -5;
    move_box::waist_layer3_set_synced(true);
    return 0;
}

/** 腰 TCP 在底盘系的位姿。手臂基座与它同姿态，原点在两肩中心，不在这个 TCP 上。 */
struct TorsoInChassis
{
    bool ok = false;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

/**
 * 两肩中心在腰 TCP 系的位置（米，前左上）。姿态与腰相同。
 * 腰 FK 的平移是 TCP。Q2 只在 TCP 上方 0.110 m，肩还在胸腔上，所以比 Q2 更高。
 * 写在腰姿态系里：俯仰时肩绕 TCP 往前摆，不是钉在 TCP 上。
 */
Eigen::Vector3d arm_base_in_waist_tcp()
{
    return Eigen::Vector3d(0.0, 0.0, 0.30);
}

TorsoInChassis torso_in_chassis()
{
    TorsoInChassis f;
    if (g_hw == nullptr || g_hw->waist == nullptr)
        return f;
    const Eigen::MatrixXd q = g_hw->waist->getJointPos();
    WaistRobot::Joints5 jq = WaistRobot::Joints5::Zero();
    if (q.cols() == LowerBody::Dof)
        jq = q.row(0);
    else if (q.rows() == LowerBody::Dof)
        jq = q.col(0).transpose();
    else
        return f;
    if (!jq.allFinite())
        return f;
    const Eigen::Matrix4d T = g_hw->waist->fk(jq);
    f.R = T.block<3, 3>(0, 0);
    f.t = T.block<3, 1>(0, 3);
    f.ok = f.R.allFinite() && f.t.allFinite() && std::abs(f.R(2, 2)) > 0.3;
    return f;
}

/**
 * 识别点已经在当前手臂基座（两肩中心）。腰 TCP 在肩的下方，姿态相同。
 * 底盘点 = 腰TCP + R * (肩偏移 + 手臂点)。沿世界竖直落到标定水平面，再减回肩偏移。
 * 只转腰时 XY 不变、Z 仍是标定值；有俯仰时 X/Y/Z 都会变。直立且腰高在标定点时结果是 (x,y,fixed_z)。
 */
bool map_neck_point_onto_arm_plane(
    const TorsoInChassis &f, double plane_z, double &x, double &y, double &z)
{
    if (!f.ok || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(plane_z))
        return false;
    const Eigen::Vector3d s = arm_base_in_waist_tcp();
    const Eigen::Vector3d p(x, y, z);
    Eigen::Vector3d p_b = f.t + f.R * (s + p);
    p_b.z() = plane_z;
    const Eigen::Vector3d p_a = f.R.transpose() * (p_b - f.t) - s;
    if (!p_a.allFinite())
        return false;
    x = p_a.x();
    y = p_a.y();
    z = p_a.z();
    return true;
}

/** 沿底盘竖直方向平移。直立时就是改手臂 Z。 */
void shift_along_world_up(Eigen::Matrix<double, 1, 6> &pose, double dist, const TorsoInChassis &f)
{
    if (std::abs(dist) < 1e-12)
        return;
    if (!f.ok)
    {
        pose(2) += dist;
        return;
    }
    pose(0) += dist * f.R(2, 0);
    pose(1) += dist * f.R(2, 1);
    pose(2) += dist * f.R(2, 2);
}

/** 码平面不用相机深度。原点按当前腰姿态落到标定水平面，抓取点平移同一段，识别系 offset 不变。 */
void apply_fixed_belt_origin_z(BeltDetectResult &det, const ConveyorStationConfig &c, const char *tag)
{
    if (!c.use_fixed_origin_z || !det.in_robot || !std::isfinite(det.origin_x) ||
        !std::isfinite(det.origin_y) || !std::isfinite(det.origin_z))
        return;
    const double x0 = det.origin_x;
    const double y0 = det.origin_y;
    const double z0 = det.origin_z;
    double x = x0;
    double y = y0;
    double z = z0;
    const TorsoInChassis torso = torso_in_chassis();
    const Eigen::Vector3d shoulder = arm_base_in_waist_tcp();
    // 标定是直立时肩到传送带的高度。肩比腰 TCP 高 shoulder.z()，水平面不能按腰 TCP 来加。
    const double plane_z = c.waist_z + shoulder.z() + c.fixed_origin_z_m;
    const bool mapped = map_neck_point_onto_arm_plane(torso, plane_z, x, y, z);
    if (!mapped)
    {
        x = x0;
        y = y0;
        z = c.fixed_origin_z_m;
    }
    const double dx = x - x0;
    const double dy = y - y0;
    const double dz = z - z0;
    det.origin_x = x;
    det.origin_y = y;
    det.origin_z = z;
    if (det.have_grasp_l && std::isfinite(det.grasp_lx) && std::isfinite(det.grasp_ly) &&
        std::isfinite(det.grasp_lz))
    {
        det.grasp_lx += dx;
        det.grasp_ly += dy;
        det.grasp_lz += dz;
    }
    if (det.have_grasp_r && std::isfinite(det.grasp_rx) && std::isfinite(det.grasp_ry) &&
        std::isfinite(det.grasp_rz))
    {
        det.grasp_rx += dx;
        det.grasp_ry += dy;
        det.grasp_rz += dz;
    }
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] " << tag << " 码原点 脖子系=(" << x0 << "," << y0 << "," << z0
              << ") → 手臂基座=(" << x << "," << y << "," << z << ")";
    if (mapped)
        std::cout << " 腰TCP=(" << torso.t.x() << "," << torso.t.y() << "," << torso.t.z()
                  << ") 肩偏移=(" << shoulder.x() << "," << shoulder.y() << "," << shoulder.z()
                  << ") 世界上=(" << torso.R(2, 0) << "," << torso.R(2, 1) << ","
                  << torso.R(2, 2) << ") 水平面z=" << plane_z;
    else
        std::cout << " 腰姿态不可用，只替换 Z";
    std::cout << "\n";
}

int move_conveyor_photo_waist_xz(const ConveyorStationConfig &c, const char *tag)
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return -1;
    if (hardware_abort_requested())
        return -4;
    const char *name = (tag && tag[0]) ? tag : "conveyor";
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] " << name << " 腰先平移到拍照 x/z=(" << c.waist_x << "," << c.waist_z
              << ")，保持当前转角，这一步不转腰\n";
    auto waist_fail = []() { return hardware_abort_requested() ? -4 : -5; };
    if (!move_box::sync_waist_layer3_from_encoders(
            *g_hw->waist, *g_hw->layer3, "拍照腰前"))
        return waist_fail();
    const double x_now = (*g_hw->layer3)(0);
    const double z_now = (*g_hw->layer3)(2);
    const double home_x = g_move_cfg.waist.layer3_home(0);
    // 只有真的伸到远排（明显超过 home）又蹲着，才先在低位收回。
    // 拍照 x 在身后时，普通抓取位 x≈0 也会大于它，不能据此在蹲姿往身后蹭。
    const bool crouched = z_now < c.waist_z - 0.05;
    const bool extended = x_now > home_x + 0.08;
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
    // 直立直线会把回转拧回 0。这里只改 x/z，转角留到后面的联动。
    auto slide_keep_turn = [&](double x, double z) -> bool {
        if (!move_box::sync_waist_layer3_from_encoders(
                *g_hw->waist, *g_hw->layer3, "平移前"))
            return false;
        const double x0 = (*g_hw->layer3)(0);
        const double z0 = (*g_hw->layer3)(2);
        if (std::abs(x - x0) <= 0.01 && std::abs(z - z0) <= 0.01)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "[waist] 平移已在 x=" << x0 << " z=" << z0 << "，跳过\n";
            return true;
        }
        const Eigen::MatrixXd q_now = g_hw->waist->getJointPos();
        if (q_now.rows() < 1 || q_now.cols() < 5)
            return false;
        WaistRobot::Joints5 q = q_now.row(0).head(5);
        const Eigen::Matrix3d R = g_hw->waist->fk(q).block<3, 3>(0, 0);
        const double yaw_deg = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
        const double pitch_deg = std::atan2(-R(2, 0), R(2, 2)) * 180.0 / M_PI;
        const double y = (*g_hw->layer3)(1);
        std::cout << std::fixed << std::setprecision(4)
                  << "[waist] 保持转角 yaw=" << std::setprecision(1) << yaw_deg
                  << " pitch=" << pitch_deg << " deg，先平移 x "
                  << std::setprecision(4) << x0 << "→" << x
                  << " z " << z0 << "→" << z << "，先不转腰\n";
        const int rc = g_hw->waist->moveToYawPitch(x, y, z, yaw_deg, pitch_deg, 0.08);
        if (rc != 0)
        {
            std::cerr << "[waist] 保持转角平移失败 code=" << rc << "\n";
            move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "平移失败后");
            return false;
        }
        hardware_abort_sleep_ms(300);
        move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "平移后");
        return !hardware_abort_requested();
    };
    if (crouched && extended)
    {
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] " << name << " 远排下蹲 x=" << x_now
                  << " z=" << z_now << "，先收到 home x=" << home_x
                  << "，再抬到拍照高度，然后到拍照 x。转腰放在平移之后\n";
        if (!slide_keep_turn(home_x, z_now) || !slide_keep_turn(home_x, c.waist_z) ||
            !slide_keep_turn(c.waist_x, c.waist_z) || !tracked())
        {
            std::cerr << "[orch] " << name << " 平移到拍照 x/z 失败\n";
            return waist_fail();
        }
    }
    else
    {
        std::cout << "[orch] " << name << " 先平移到拍照高度和拍照 x，转腰放在后面\n";
        if (!slide_keep_turn(x_now, c.waist_z) || !slide_keep_turn(c.waist_x, c.waist_z) ||
            !tracked())
        {
            std::cerr << "[orch] " << name << " 平移到拍照 x/z 失败\n";
            return waist_fail();
        }
    }
    move_box::waist_layer3_set_synced(true);
    return 0;
}

int conveyor_waist_and_head(const ConveyorStationConfig &c, const char *tag)
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return -1;
    const char *name = (tag && tag[0]) ? tag : "conveyor";
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] " << name << " 头 "
              << c.head_yaw_deg << "/" << c.head_pitch_deg << "/" << c.head_roll_deg
              << " deg\n";
    const int xz_rc = move_conveyor_photo_waist_xz(c, tag);
    if (xz_rc != 0)
        return xz_rc;
    const int head_rc = move_head_to_rpy_deg(
        c.head_yaw_deg, c.head_pitch_deg, c.head_roll_deg, true);
    if (head_rc != 0)
        return head_rc;
    if (hardware_abort_requested())
        return -4;
    if (std::abs(c.same_spot_yaw_deg) > 0.5 || std::abs(c.same_spot_pitch_deg) > 0.5)
    {
        std::cout << std::fixed << std::setprecision(1)
                  << "[orch] " << name << " 拍照前腰联动 yaw=" << c.same_spot_yaw_deg
                  << " pitch=" << c.same_spot_pitch_deg << " deg，手臂基座跟着转\n";
        if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "转腰前"))
            return hardware_abort_requested() ? -4 : -5;
        const double y = (*g_hw->layer3)(1);
        const int pose_rc = g_hw->waist->moveToYawPitch(
            c.waist_x, y, c.waist_z, c.same_spot_yaw_deg, c.same_spot_pitch_deg, 0.10);
        if (pose_rc == -3 || hardware_abort_requested())
            return -4;
        if (pose_rc != 0)
        {
            std::cerr << "[orch] " << name << " 腰联动失败 code=" << pose_rc << "\n";
            return pose_rc == -2 ? -2 : -5;
        }
        if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "转腰后"))
            std::cout << "[orch] " << name << " 关节已到位。转腰后笛卡尔回读可能超出直立行程\n";
    }
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

std::mutex g_chassis_mu;

Seer &shared_chassis()
{
    static Seer robot("127.0.0.1");
    return robot;
}

bool chassis_connected()
{
    std::lock_guard<std::mutex> lk(g_chassis_mu);
    Seer &robot = shared_chassis();
    if (robot.IsConnected())
        return true;
    if (g_move_cfg.chassis.host.empty())
        return false;
    robot.SetHost(g_move_cfg.chassis.host);
    if (!robot.Connect())
    {
        std::cerr << "[orch] 底盘连接失败: "
                  << (robot.LastMessage().empty() ? g_move_cfg.chassis.host : robot.LastMessage())
                  << "\n";
        return false;
    }
    std::cout << "[orch] 底盘已连接 " << g_move_cfg.chassis.host
              << "，导航和状态上报共用这一条连接\n";
    return true;
}

void chassis_release()
{
    std::lock_guard<std::mutex> lk(g_chassis_mu);
    Seer &robot = shared_chassis();
    if (robot.IsConnected())
        robot.Disconnect();
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

    if (!chassis_connected())
    {
        g_hw->last_chassis_message =
            shared_chassis().LastMessage().empty() ? "底盘连接失败" : shared_chassis().LastMessage();
        return -6;
    }
    Seer &robot = shared_chassis();
    std::cout << "[orch] 底盘去 " << station << "\n";

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
                return -6;
            }
        }
        hardware_abort_sleep_ms(300);
    }

    g_hw->last_chassis_message = "等待到位超时";
    std::cerr << "[orch] 底盘去 " << station << " 超时\n";
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
    return waist_return_home_slow();
}

int go_home(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    std::cout << "[orch] home：先头部标定姿态，再安全抬升走到 yaml home_tcp，再腰五轴慢回直立 home\n";
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
    const int waist_rc = waist_return_home_slow();
    if (waist_rc != 0)
        return waist_rc;
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
              << "：头同抓取，腰用料盘二放置 x，手臂用 tray2_place 独立 RPY"
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
        std::cout << std::fixed << std::setprecision(3)
                  << "[orch] tray2ready 腰到料盘二放置 x="
                  << g_move_cfg.tray2_place.waist_x
                  << " z=" << waist_ready_z() << "（与抓取同高，x 往后）\n";
        if (!move_box::move_waist_tray2_place(*g_hw->waist, *g_hw->layer3, false, 0.0, 0.0))
        {
            std::cerr << "[orch] 腰到料盘二放置位失败\n";
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
                 "右手放下松爪后，左手再等 1s 过来；右手同时抬起回准备（不回 home）\n";

    if (skip_ready)
    {
        std::cout << "[orch] " << name << " 准备已经摆好，直接等停稳再拍\n";
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
    const TorsoInChassis place_torso = torso_in_chassis();
    if (std::abs(c.place_base_z_bias_m) > 1e-9)
    {
        shift_along_world_up(goal_r, c.place_base_z_bias_m, place_torso);
        shift_along_world_up(goal_l, c.place_base_z_bias_m, place_torso);
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] belt 放置松手沿竖直方向再偏 " << c.place_base_z_bias_m
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
              << ") 右手松爪后左手再等 1s 过来，不等右手回准备\n";
    std::cout.flush();

    auto place_arrive_and_open = [&](bool is_right) -> int {
        if (is_right && right_arm_motors_locked())
        {
            std::cout << "[orch] belt 右臂锁定，跳过右手放置\n";
            return 0;
        }
        Eigen::Matrix<double, 1, 6> goal = is_right ? goal_r : goal_l;
        constexpr double kPlaceMoveMps = 0.15;
        const char *hand = is_right ? "右手" : "左手";
        Eigen::Matrix<double, 1, 6> hover = goal;
        const double hover_z = c.place_hover_above_m > 1e-9 ? c.place_hover_above_m : 0.06;
        shift_along_world_up(hover, hover_z, place_torso);
        std::cout << std::fixed << std::setprecision(2)
                  << "[orch] belt " << hand << " " << kPlaceMoveMps
                  << "m/s 抹角到放置点，不在上方停车\n";
        const ArmLineMoveResult approach = arm_dual_rounded_corner_move_selective(
            *g_hw->arm_r, hover, goal, is_right,
            *g_hw->arm_l, hover, goal, !is_right,
            kPlaceMoveMps);
        const int approach_rc = is_right ? approach.ret_r : approach.ret_l;
        if (approach_rc != 0)
        {
            std::cerr << "[orch] belt " << hand
                      << " 放置伸手抹角失败 code=" << approach_rc << "\n";
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
        constexpr double kPlaceNudgeUpM = 0.03;
        constexpr double kPlaceMoveMps = 0.15;
        Eigen::Matrix<double, 1, 6> lift = is_right ? goal_r : goal_l;
        shift_along_world_up(lift, kPlaceNudgeUpM, place_torso);
        Eigen::Matrix<double, 1, 6> ready = is_right ? c.tcp.right : c.tcp.left;
        const char *hand = is_right ? "右手" : "左手";
        std::cout << std::fixed << std::setprecision(3)
                  << "[orch] belt " << hand << " " << kPlaceMoveMps
                  << "m/s 松爪后抬 " << kPlaceNudgeUpM
                  << "m，不停，直接回准备\n";
        const ArmLineMoveResult stitched = arm_dual_line_then_line_nostop(
            *g_hw->arm_r, lift, ready, is_right,
            *g_hw->arm_l, lift, ready, !is_right,
            kPlaceMoveMps, kPlaceMoveMps, {}, false, "传送带放置收手");
        const int back_rc = is_right ? stitched.ret_r : stitched.ret_l;
        if (back_rc != 0)
        {
            std::cerr << "[orch] belt " << hand
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
        {
            std::cout << "[orch] belt 右手已放下，左手再等 1s 过来（不等右手抬起回准备）\n";
            hardware_abort_sleep_ms(1000);
            if (aborted())
            {
                if (th_r.joinable())
                    th_r.join();
                return -4;
            }
        }
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
    const auto &c = g_move_cfg.conveyor;
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] belt：不导航，先腰联动 yaw=" << c.same_spot_yaw_deg
              << " pitch=" << c.same_spot_pitch_deg
              << " deg（与 belt1_waist 相同），再拍照放置，结束后原地不动\n";
    return go_belt_station(c, "belt");
}

int go_belt_grasp(
    bool from_place, const ConveyorStationConfig &c, const std::string &station, const char *tag,
    int only_hand = -1, const std::function<int(bool is_right)> &on_grasped = {},
    bool photo_settle = true, const std::string &depart_after_right = {},
    bool navigate = true, bool pose_waist = true, bool retract_to_conveyor_tcp = false)
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
    else if (!navigate && !pose_waist)
        std::cout << "[orch] " << name << "：不导航、不动腰，原地拍照后抓取\n";
    else if (!navigate)
        std::cout << std::fixed << std::setprecision(1)
                  << "[orch] " << name << "：不导航，先腰联动 yaw=" << c.same_spot_yaw_deg
                  << " pitch=" << c.same_spot_pitch_deg
                  << " deg（与 belt1_waist 相同），再拍照抓取，结束后原地不动\n";
    else
        std::cout << "[orch] " << name << "：先到 " << station
                  << " → 腰/头到位 → 拍照 → 再动手臂\n";

    if (!from_place)
    {
        if (navigate)
        {
            const int ch_rc = chassis_goto_station(station);
            if (ch_rc != 0)
                return ch_rc;
            if (aborted())
                return -4;
            std::cout << "[orch] " << name << " 底盘已到 " << station
                      << "，腰头到位后停稳 1.5s 再拍照\n";
        }
        if (pose_waist)
        {
            const int wh = conveyor_waist_and_head(c, name);
            if (wh != 0)
                return wh;
            if (aborted())
                return -4;
        }
    }

    if (photo_settle)
    {
        std::cout << "[orch] belt_grasp 不转腰，停稳 1.5s 再拍照；grasp_offset 按码坐标系，允许过中轴\n";
        hardware_abort_sleep_ms(1500);
    }
    else if (!navigate && !pose_waist)
        std::cout << "[orch] " << name << " 腰和底盘保持现状，直接拍照\n";
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
        const TorsoInChassis grasp_torso = torso_in_chassis();
        shift_along_world_up(hover, hover_z, grasp_torso);
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
        retract_to_conveyor_tcp ||
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
        shift_along_world_up(lift, lift_z, torso_in_chassis());
        // 料盘包络 z_max=-0.05。传送带抓取点已经高于它，再裁会把抬升压回皮带。
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
                  << "m/s 抬 " << lift_z << "m 到 xyz=(" << lift(0) << "," << lift(1) << ","
                  << lift(2) << ") 后不停，先平移回 " << back_name
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
        from_place, g_move_cfg.conveyor, g_move_cfg.chassis.belt_station, "belt_grasp",
        -1, {}, false, std::string(), false, false);
}

int go_belt2_ready(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    std::cout << "[orch] belt2_ready：不导航，只摆 conveyor2 的腰、头和准备 tcp\n";
    return go_belt_ready(arm_r, arm_l, g_move_cfg.conveyor2, "belt2");
}

int go_belt2_grasp_rpy(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    return go_belt_grasp_rpy(arm_r, arm_l, g_move_cfg.conveyor2, "belt2");
}

int go_belt2()
{
    const auto &c = g_move_cfg.conveyor2;
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] belt2：不导航，先腰联动 yaw=" << c.same_spot_yaw_deg
              << " pitch=" << c.same_spot_pitch_deg
              << " deg，再拍照放置，结束后原地不动\n";
    return go_belt_station(c, "belt2");
}

int go_belt2_place()
{
    return go_belt2();
}

int go_belt2_grasp()
{
    // 站点名传 belt_station，只为了收手回到本套准备 tcp，并不导航。
    return go_belt_grasp(
        false, g_move_cfg.conveyor2, g_move_cfg.chassis.belt_station, "belt2_grasp",
        -1, {}, false, std::string(), false, false);
}

int go_belt1_same_spot_waist()
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
    {
        std::cerr << "[orch] belt1_waist 失败：腰未初始化\n";
        return -1;
    }
    if (hardware_abort_requested())
        return -4;
    const auto &c = g_move_cfg.conveyor;
    if (g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
    {
        std::cerr << "[orch] belt1_waist 失败：手臂未初始化\n";
        return -1;
    }
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] belt1_waist 底盘不动：先到拍照 x=" << std::setprecision(4)
              << c.waist_x << " z=" << c.waist_z
              << "，头 " << std::setprecision(1) << c.head_yaw_deg << "/"
              << c.head_pitch_deg << "/" << c.head_roll_deg
              << " deg，再 LowerBody 联动 yaw=" << c.same_spot_yaw_deg
              << " pitch=" << c.same_spot_pitch_deg
              << " deg，最后双手到 conveyor.tcp\n";
    const int xz_rc = move_conveyor_photo_waist_xz(c, "belt1_waist");
    if (xz_rc != 0)
        return xz_rc;
    if (hardware_abort_requested())
        return -4;
    const int head_rc = move_head_to_rpy_deg(
        c.head_yaw_deg, c.head_pitch_deg, c.head_roll_deg, true);
    if (head_rc != 0)
    {
        std::cerr << "[orch] belt1_waist 头部到位失败 code=" << head_rc << "\n";
        return head_rc;
    }
    if (hardware_abort_requested())
        return -4;
    const double y = (*g_hw->layer3)(1);
    const int pose_rc = g_hw->waist->moveToYawPitch(
        c.waist_x, y, c.waist_z, c.same_spot_yaw_deg, c.same_spot_pitch_deg, 0.10);
    if (pose_rc == -3 || hardware_abort_requested())
        return -4;
    if (pose_rc != 0)
    {
        std::cerr << "[orch] belt1_waist 联动失败 code=" << pose_rc << "\n";
        return pose_rc == -2 ? -2 : -5;
    }
    if (!move_box::sync_waist_layer3_from_encoders(
            *g_hw->waist, *g_hw->layer3, "belt1_waist后"))
    {
        std::cout << "[orch] belt1_waist 关节已到位。转腰后笛卡尔回读可能超出直立行程，不据此判失败\n";
    }
    if (hardware_abort_requested())
        return -4;
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] belt1_waist 双手到 tcp"
              << " 右=(" << c.tcp.right(0) << "," << c.tcp.right(1) << "," << c.tcp.right(2)
              << ") rpy=(" << (c.tcp.right(3) / rad) << "," << (c.tcp.right(4) / rad) << ","
              << (c.tcp.right(5) / rad) << ")"
              << " 左=(" << c.tcp.left(0) << "," << c.tcp.left(1) << "," << c.tcp.left(2)
              << ") rpy=(" << (c.tcp.left(3) / rad) << "," << (c.tcp.left(4) / rad) << ","
              << (c.tcp.left(5) / rad) << ")deg\n";
    if (!move_box::move_arms_to_belt_ready(*g_hw->arm_r, *g_hw->arm_l, c))
    {
        std::cerr << "[orch] belt1_waist 手臂到 tcp 失败\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    auto near_tcp = [](const Eigen::Matrix<double, 1, 6> &cur, const Eigen::Matrix<double, 1, 6> &goal) {
        constexpr double kXyzM = 0.012;
        constexpr double kRpyRad = 5.0 * M_PI / 180.0;
        return (cur.head<3>() - goal.head<3>()).norm() <= kXyzM &&
               (cur.tail<3>() - goal.tail<3>()).cwiseAbs().maxCoeff() <= kRpyRad;
    };
    const Eigen::Matrix<double, 1, 6> got_r = arm_get_tcp_pos(*g_hw->arm_r);
    const Eigen::Matrix<double, 1, 6> got_l = arm_get_tcp_pos(*g_hw->arm_l);
    const bool right_ok = right_arm_motors_locked() || near_tcp(got_r, c.tcp.right);
    const bool left_ok = near_tcp(got_l, c.tcp.left);
    if (!right_ok || !left_ok)
    {
        std::cerr << std::fixed << std::setprecision(3)
                  << "[orch] belt1_waist 手臂未到 tcp"
                  << " 右实际xyz=(" << got_r(0) << "," << got_r(1) << "," << got_r(2)
                  << ") rpy=(" << (got_r(3) / rad) << "," << (got_r(4) / rad) << ","
                  << (got_r(5) / rad) << ")"
                  << " 左实际xyz=(" << got_l(0) << "," << got_l(1) << "," << got_l(2)
                  << ") rpy=(" << (got_l(3) / rad) << "," << (got_l(4) / rad) << ","
                  << (got_l(5) / rad) << ")\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    return 0;
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

int vision_grasp_with_engine(
    SegEngineId engine_id,
    bool adaptive_approach_rpy = false,
    bool depart_to_belt = true,
    bool skip_entry_prep = false)
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

    std::cout << (skip_entry_prep
                      ? "[grasp] 腰和手臂已在抓取准备，只开爪，不再重摆\n"
                      : "[grasp] 原地开始：摆头、手臂、开爪和下腰，不去站点\n");
    int prep_rc = 0;
    if (skip_entry_prep)
    {
        if (g_hw->waist == nullptr || g_hw->layer3 == nullptr ||
            g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
            prep_rc = -1;
        else
        {
            g_hw->g->openAndWait(gripper::Side::Right, 0.2, std::chrono::milliseconds(1500));
            g_hw->g->openAndWait(gripper::Side::Left, 0.2, std::chrono::milliseconds(1500));
            if (should_abort())
                prep_rc = -4;
            move_box::waist_layer3_set_synced(true);
        }
    }
    else
    {
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
    }
    if (should_abort())
        return -4;
    if (prep_rc != 0)
        return prep_rc;
    std::cout << "[grasp] 原地准备完成，停稳 0.5s 再抓\n";
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
    if (depart_to_belt && (holding_r || holding_l))
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

int vision_place_tray2(bool allow_right, bool allow_left, bool return_to_tray,
                       bool face_yaw, double face_yaw_deg, const std::string &station)
{
    const std::string place_station =
        station.empty() ? g_move_cfg.chassis.tray2_station : station;
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

    std::cout << "[tray2] 去料盘2 " << place_station
              << "。路上只摆头和待机臂，腰保持高位；到底后再降腰"
              << (face_yaw ? "，并保持当前朝向\n" : "\n");
    int prep_rc = 0;
    int travel_rc = 0;
    {
        ChassisTravel travel(place_station);
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
        travel_rc = travel.join();
    }
    if (should_abort())
        return -4;
    if (travel_rc != 0)
        return travel_rc;
    if (prep_rc != 0)
        return prep_rc;
    if (!move_box::move_waist_tray2_place(
            *g_hw->waist, *g_hw->layer3, false, face_yaw ? face_yaw_deg : 0.0, 0.0))
    {
        std::cerr << "[tray2] 腰未到料盘二放置位，放弃放置\n";
        return should_abort() ? -4 : -5;
    }
    move_box::waist_layer3_set_synced(true);
    if (should_abort())
        return -4;
    std::cout << "[tray2] 已在 " << place_station << "，停稳 0.5s 再放\n";
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
            std::cout << "[tray2] 放完停在 " << place_station
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
    if (!move_box::move_waist_tray2_place(*g_hw->waist, *g_hw->layer3, true))
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

int cycle1_belt_yaw_and_pitch(const ConveyorStationConfig &c);

namespace
{

int cycle_fail()
{
    return hardware_abort_requested() ? -4 : -5;
}

int chassis_query_station(std::string &cur)
{
    cur.clear();
    if (g_hw == nullptr)
        return -1;
    const auto &ch = g_move_cfg.chassis;
    if (ch.host.empty())
    {
        std::cerr << "[orch] cycle 底盘地址为空，无法确认当前站\n";
        return -1;
    }
    if (!chassis_connected())
        return -6;
    Seer &robot = shared_chassis();
    double x = 0, y = 0, angle = 0, confidence = 0;
    std::string last;
    int loc_state = 0;
    bool in_forbidden = false;
    const int loc_rc = robot.GetLocation(x, y, angle, cur, last, confidence, loc_state, in_forbidden);
    if (loc_rc != 0 || cur.empty())
    {
        std::cerr << "[orch] cycle 没有读到当前站名\n";
        cur.clear();
        return -6;
    }
    g_hw->last_chassis_ok = true;
    g_hw->last_chassis_station = cur;
    std::cout << "[orch] cycle 当前底盘站 " << cur << "\n";
    return 0;
}

bool cycle_waist_rpy(double &yaw_deg, double &pitch_deg)
{
    yaw_deg = 0.0;
    pitch_deg = 0.0;
    const TorsoInChassis torso = torso_in_chassis();
    if (!torso.ok)
        return false;
    yaw_deg = std::atan2(torso.R(1, 0), torso.R(0, 0)) / rad;
    pitch_deg = std::atan2(-torso.R(2, 0), torso.R(2, 2)) / rad;
    return true;
}

bool cycle_waist_at_tray_grasp()
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return false;
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle查腰"))
        return false;
    const double z = (*g_hw->layer3)(2);
    const double z_ready = waist_ready_z();
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    cycle_waist_rpy(yaw_deg, pitch_deg);
    const bool low = std::abs(z - z_ready) <= 0.04;
    const bool upright = std::abs(yaw_deg) <= 12.0 && std::abs(pitch_deg) <= 12.0;
    std::cout << std::fixed << std::setprecision(3)
              << "[orch] cycle 腰 z=" << z << " 料盘抓取高度=" << z_ready
              << " yaw=" << std::setprecision(1) << yaw_deg
              << " pitch=" << pitch_deg
              << (low && upright ? "，已在料盘抓取姿态\n" : "，不在料盘抓取姿态\n");
    return low && upright;
}

int cycle_arms_to_tcp(const ConveyorStationConfig &c, const char *tag)
{
    std::cout << "[orch] cycle 双手先到 " << tag << " 准备 tcp，避免转身和开车时碰到周围\n";
    if (!move_box::move_arms_to_belt_ready(*g_hw->arm_r, *g_hw->arm_l, c))
    {
        std::cerr << "[orch] cycle 手到 " << tag << " tcp 失败\n";
        return cycle_fail();
    }
    return hardware_abort_requested() ? -4 : 0;
}

int cycle_waist_to_home()
{
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle回home前"))
        return cycle_fail();
    const auto &home = g_move_cfg.waist.layer3_home;
    const double x = (*g_hw->layer3)(0);
    const double y = (*g_hw->layer3)(1);
    const double z = (*g_hw->layer3)(2);
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    cycle_waist_rpy(yaw_deg, pitch_deg);
    const bool pos_home = std::abs(x - home(0)) <= 0.02 && std::abs(z - home(2)) <= 0.02;
    const bool ori_home = std::abs(yaw_deg) <= 1.5 && std::abs(pitch_deg) <= 1.5;
    if (pos_home && ori_home)
    {
        std::cout << "[orch] cycle 腰已在 home，不重复回正\n";
        move_box::waist_layer3_set_synced(true);
        return 0;
    }
    if (!pos_home && (std::abs(yaw_deg) > 12.0 || std::abs(pitch_deg) > 8.0))
    {
        std::cout << std::fixed << std::setprecision(3)
                  << "[orch] cycle 腰先平移到 home x=" << home(0) << " z=" << home(2)
                  << "，保持当前 yaw=" << std::setprecision(1) << yaw_deg
                  << " pitch=" << pitch_deg << "，到位后再转正\n";
        const int slide_rc = g_hw->waist->moveToYawPitch(
            home(0), y, home(2), yaw_deg, pitch_deg, 0.08);
        if (slide_rc != 0)
        {
            std::cerr << "[orch] cycle 腰平移到 home 失败 code=" << slide_rc << "\n";
            return hardware_abort_requested() ? -4 : slide_rc;
        }
        if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle平移home后"))
            return cycle_fail();
        if (hardware_abort_requested())
            return -4;
    }
    const double y_now = (*g_hw->layer3)(1);
    std::cout << std::fixed << std::setprecision(3)
              << "[orch] cycle 腰转到 home x=" << home(0) << " z=" << home(2)
              << " yaw=0 pitch=0\n";
    const int rc = g_hw->waist->moveToYawPitch(home(0), y_now, home(2), 0.0, 0.0, 0.08);
    if (rc != 0)
    {
        std::cerr << "[orch] cycle 腰回 home 失败 code=" << rc << "\n";
        return hardware_abort_requested() ? -4 : rc;
    }
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle回home后"))
        return cycle_fail();
    move_box::waist_layer3_set_synced(true);
    return hardware_abort_requested() ? -4 : 0;
}

int cycle_ensure_waist_high_before_drive()
{
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle开车前"))
        return cycle_fail();
    const double home_z = g_move_cfg.waist.layer3_home(2);
    auto high_enough = [&]() {
        return std::abs((*g_hw->layer3)(2) - home_z) <= 0.02;
    };
    if (high_enough())
    {
        std::cout << std::fixed << std::setprecision(3)
                  << "[orch] cycle 腰高 z=" << (*g_hw->layer3)(2)
                  << " 已在 home " << home_z << "，允许底盘动\n";
        return 0;
    }
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    cycle_waist_rpy(yaw_deg, pitch_deg);
    const double x = (*g_hw->layer3)(0);
    const double y = (*g_hw->layer3)(1);
    std::cout << std::fixed << std::setprecision(3)
              << "[orch] cycle 腰还在低位 z=" << (*g_hw->layer3)(2)
              << "，先升到 home " << home_z << " 再动底盘\n";
    const int up_rc = g_hw->waist->moveToYawPitch(x, y, home_z, yaw_deg, pitch_deg, 0.08);
    if (up_rc != 0)
    {
        std::cerr << "[orch] cycle 开车前升腰失败 code=" << up_rc << "，底盘不动\n";
        return hardware_abort_requested() ? -4 : up_rc;
    }
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle升腰后"))
        return cycle_fail();
    if (!high_enough())
    {
        std::cerr << std::fixed << std::setprecision(3)
                  << "[orch] cycle 腰高 z=" << (*g_hw->layer3)(2)
                  << " 没到 home " << home_z << "，拒绝动底盘\n";
        return -5;
    }
    std::cout << std::fixed << std::setprecision(3)
              << "[orch] cycle 腰已升到 z=" << (*g_hw->layer3)(2) << "，允许底盘动\n";
    return hardware_abort_requested() ? -4 : 0;
}

int cycle_enter_tray_grasp_pose()
{
    std::cout << "[orch] cycle 进入料盘抓取姿态：头、双手待机，腰降到抓取高度\n";
    std::atomic<int> arm_rc{0};
    std::atomic<int> head_rc{0};
    std::thread arm_th([&]() {
        if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l))
            arm_rc = cycle_fail();
    });
    std::thread head_th([&]() { head_rc = enable_head_calib_pose(); });
    const bool waist_ok = move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3);
    arm_th.join();
    head_th.join();
    if (hardware_abort_requested())
        return -4;
    if (!waist_ok)
    {
        std::cerr << "[orch] cycle 降到料盘抓取腰位失败\n";
        return -5;
    }
    if (arm_rc.load() != 0)
        return arm_rc.load();
    if (head_rc.load() != 0)
    {
        std::cerr << "[orch] cycle 头到抓取标定角失败 code=" << head_rc.load() << "\n";
        return head_rc.load();
    }
    move_box::waist_layer3_set_synced(true);
    return 0;
}

int cycle_prepare_at_tray1(const std::string &ap6)
{
    if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr ||
        g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return -1;
    std::string cur;
    const int loc_rc = chassis_query_station(cur);
    if (loc_rc != 0)
        return loc_rc;
    if (hardware_abort_requested())
        return -4;

    const bool at_ap6 = station_name_matches(cur, ap6);
    bool already_tucked = false;
    if (!at_ap6)
    {
        std::cout << "[orch] cycle 不在 " << ap6
                  << "，先收手、腰回 home，再导航。底盘先不动\n";
        const int arm_rc = cycle_arms_to_tcp(g_move_cfg.conveyor, "传送带一");
        if (arm_rc != 0)
            return arm_rc;
        const int home_rc = cycle_waist_to_home();
        if (home_rc != 0)
            return home_rc;
        if (hardware_abort_requested())
            return -4;
        const int high_rc = cycle_ensure_waist_high_before_drive();
        if (high_rc != 0)
            return high_rc;
        std::cout << "[orch] cycle 手已收好、腰已在 home 高度，底盘去 " << ap6 << "\n";
        const int nav_rc = chassis_goto_station(ap6);
        if (nav_rc != 0)
            return nav_rc;
        already_tucked = true;
    }
    else
        std::cout << "[orch] cycle 已在 " << ap6 << "，底盘不动，只检查腰\n";

    if (hardware_abort_requested())
        return -4;
    if (cycle_waist_at_tray_grasp())
    {
        std::cout << "[orch] cycle 腰已在料盘抓取高度，双手直接进抓取准备\n";
        const int head_rc = enable_head_calib_pose();
        if (head_rc != 0)
            return head_rc;
        if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l))
        {
            std::cerr << "[orch] cycle 双手进料盘抓取准备失败\n";
            return cycle_fail();
        }
        move_box::waist_layer3_set_synced(true);
        return hardware_abort_requested() ? -4 : 0;
    }

    std::cout << "[orch] cycle 腰不在料盘抓取高度，先收手再转正，然后进入抓取姿态\n";
    if (!already_tucked)
    {
        const int arm_rc = cycle_arms_to_tcp(g_move_cfg.conveyor, "传送带一");
        if (arm_rc != 0)
            return arm_rc;
        const int home_rc = cycle_waist_to_home();
        if (home_rc != 0)
            return home_rc;
    }
    return cycle_enter_tray_grasp_pose();
}

int cycle_raise_then_belt1_pose()
{
    const auto &belt = g_move_cfg.conveyor;
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle升腰前"))
        return cycle_fail();
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    cycle_waist_rpy(yaw_deg, pitch_deg);
    const double y = (*g_hw->layer3)(1);
    std::cout << std::fixed << std::setprecision(3)
              << "[orch] cycle 料盘抓取结束，腰平移到 x=" << belt.waist_x
              << " 并升到 z=" << belt.waist_z
              << " 同时做，保持 yaw=" << std::setprecision(1) << yaw_deg
              << " pitch=" << pitch_deg << "，然后再转腰弯腰\n";
    std::atomic<int> head_rc{0};
    std::thread head_th([&]() {
        head_rc = move_head_to_rpy_deg(
            belt.head_yaw_deg, belt.head_pitch_deg, belt.head_roll_deg, true);
    });
    const int xz_rc = g_hw->waist->moveToYawPitch(
        belt.waist_x, y, belt.waist_z, yaw_deg, pitch_deg, 0.08);
    if (xz_rc != 0)
    {
        if (head_th.joinable())
            head_th.join();
        std::cerr << "[orch] cycle 腰平移加升起失败 code=" << xz_rc << "\n";
        return hardware_abort_requested() ? -4 : xz_rc;
    }
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle升腰后"))
    {
        if (head_th.joinable())
            head_th.join();
        return cycle_fail();
    }
    const double dx = std::abs((*g_hw->layer3)(0) - belt.waist_x);
    const double dz = std::abs((*g_hw->layer3)(2) - belt.waist_z);
    if (dx > 0.02 || dz > 0.02)
    {
        if (head_th.joinable())
            head_th.join();
        std::cerr << std::fixed << std::setprecision(3)
                  << "[orch] cycle 腰未到传送带高度 x=" << (*g_hw->layer3)(0)
                  << " z=" << (*g_hw->layer3)(2) << "，先不转腰\n";
        return -5;
    }
    const int lean_rc = cycle1_belt_yaw_and_pitch(belt);
    if (head_th.joinable())
        head_th.join();
    if (lean_rc != 0)
        return lean_rc;
    if (head_rc.load() != 0)
    {
        std::cerr << "[orch] cycle 头到传送带一拍照角失败 code=" << head_rc.load() << "\n";
        return head_rc.load();
    }
    if (hardware_abort_requested())
        return -4;
    std::cout << "[orch] cycle 转腰完成，双手到传送带一准备 tcp\n";
    if (!move_box::move_arms_to_belt_ready(*g_hw->arm_r, *g_hw->arm_l, belt))
    {
        std::cerr << "[orch] cycle 手到传送带一 tcp 失败\n";
        return cycle_fail();
    }
    return hardware_abort_requested() ? -4 : 0;
}

int cycle_travel_to_belt2(const std::string &ap7)
{
    const auto &belt = g_move_cfg.conveyor2;
    const int high_rc = cycle_ensure_waist_high_before_drive();
    if (high_rc != 0)
        return high_rc;
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle去传送带二前"))
        return cycle_fail();
    const double y = (*g_hw->layer3)(1);
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] cycle 底盘出发去 " << ap7
              << "，上身同时开始转。腰直接到 x=" << std::setprecision(3) << belt.waist_x
              << " z=" << belt.waist_z << " yaw=" << std::setprecision(1) << belt.same_spot_yaw_deg
              << " pitch=" << belt.same_spot_pitch_deg << "，不再先平移后转\n";
    ChassisTravel travel(ap7);
    std::atomic<int> arm_rc{0};
    std::atomic<int> head_rc{0};
    std::thread arm_th([&]() {
        if (!move_box::move_arms_to_belt_ready(*g_hw->arm_r, *g_hw->arm_l, belt))
            arm_rc = cycle_fail();
    });
    std::thread head_th([&]() {
        const int rc = move_head_to_rpy_deg(
            belt.head_yaw_deg, belt.head_pitch_deg, belt.head_roll_deg, true);
        if (rc != 0)
            head_rc = rc;
    });
    const int waist_rc = g_hw->waist->moveToYawPitch(
        belt.waist_x, y, belt.waist_z, belt.same_spot_yaw_deg, belt.same_spot_pitch_deg, 0.10);
    head_th.join();
    arm_th.join();
    const int nav_rc = travel.join();
    if (hardware_abort_requested() || waist_rc == -3 || head_rc.load() == -4)
        return -4;
    if (nav_rc != 0)
        return nav_rc;
    if (waist_rc != 0)
    {
        std::cerr << "[orch] cycle 去传送带二时转腰失败 code=" << waist_rc << "\n";
        return waist_rc == -2 ? -2 : -5;
    }
    if (head_rc.load() != 0)
        return head_rc.load();
    if (arm_rc.load() != 0)
        return arm_rc.load();
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle到传送带二后"))
        std::cout << "[orch] cycle 转腰关节已到位。笛卡尔回读可能超出直立行程\n";
    move_box::waist_layer3_set_synced(true);
    return 0;
}

int cycle_stand_then_tray_place()
{
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle起身前"))
        return cycle_fail();
    const auto &home = g_move_cfg.waist.layer3_home;
    const double y = (*g_hw->layer3)(1);
    constexpr double kTray2FaceYawDeg = 180.0;
    std::cout << std::fixed << std::setprecision(3)
              << "[orch] cycle 传送带二抓完，手已在 conveyor2.tcp。"
              << "起身和转腰同时做，目标 x=" << home(0)
              << " z=" << home(2) << " yaw=" << kTray2FaceYawDeg << " pitch=0\n";
    const int up_rc = g_hw->waist->moveToYawPitch(
        home(0), y, home(2), kTray2FaceYawDeg, 0.0, 0.08);
    if (up_rc != 0)
    {
        std::cerr << "[orch] cycle 起身转到 " << kTray2FaceYawDeg
                  << "° 失败 code=" << up_rc << "\n";
        return hardware_abort_requested() ? -4 : up_rc;
    }
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle起身后"))
        return cycle_fail();
    if (hardware_abort_requested())
        return -4;
    std::cout << "[orch] cycle 已起身并朝向 " << kTray2FaceYawDeg
              << "°，再降腰进入料盘放置\n";
    return vision_place_tray2(true, true, false, true, kTray2FaceYawDeg);
}

} // namespace

int vision_grasp_then_belt()
{
    const std::string &ap6 = g_move_cfg.cycle1_station;
    const std::string &ap7 = g_move_cfg.chassis.out_station;
    std::cout << "[orch] cycle 演示整圈，不和调度通讯。站点 " << ap6
              << "：料盘抓 → 传送带一放 → 传送带一抓；然后 " << ap7
              << "：传送带二放 → 传送带二抓（手回 conveyor2.tcp）→ 起身转到 180° → 保持朝向降腰料盘放。abort 才停\n";

    for (int cycle = 1;; ++cycle)
    {
        std::cout << "\n======== [orch] cycle 第 " << cycle << " 圈 ========\n";
        std::cout.flush();
        if (hardware_abort_requested())
            return -4;
        if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
            return -1;

        const int prep_rc = cycle_prepare_at_tray1(ap6);
        if (prep_rc != 0)
            return prep_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] cycle 腰、手、底盘已在料盘抓取姿态，开始抓取\n";
        const int grasp_rc = vision_grasp_with_engine(SegEngineId::Default, false, false, true);
        if (grasp_rc < 0)
            return grasp_rc;
        if (!g_hw->last_grasped_r && !g_hw->last_grasped_l)
        {
            std::cerr << "[orch] cycle 没有持件，不放置\n";
            return grasp_rc == 0 ? 1 : grasp_rc;
        }
        if (hardware_abort_requested())
            return -4;

        const int pose_rc = cycle_raise_then_belt1_pose();
        if (pose_rc != 0)
            return pose_rc;
        std::cout << "[orch] cycle 停稳 1s，拍照放置到传送带一\n";
        hardware_abort_sleep_ms(1000);
        if (hardware_abort_requested())
            return -4;
        const int belt_rc = go_belt_station(g_move_cfg.conveyor, "belt", {}, true, true, true);
        if (belt_rc != 0)
            return belt_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] cycle 传送带一放置结束，原地抓取。底盘留在 " << ap6
                  << "，收手只回 conveyor.tcp\n";
        const int bg_rc = go_belt_grasp(
            true, g_move_cfg.conveyor, std::string(), "belt_grasp",
            -1, {}, true, std::string(), false, false, true);
        if (bg_rc != 0)
            return bg_rc;
        if (hardware_abort_requested())
            return -4;
        if (!g_hw->last_grasped_r && !g_hw->last_grasped_l)
        {
            std::cerr << "[orch] cycle 传送带一没有抓到，不去 " << ap7 << "\n";
            return 1;
        }

        const int travel_rc = cycle_travel_to_belt2(ap7);
        if (travel_rc != 0)
            return travel_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] cycle 已到 " << ap7 << "，传送带二准备已摆好，开始放置\n";
        const int belt2_rc = go_belt_station(g_move_cfg.conveyor2, "belt2", {}, true, true, true);
        if (belt2_rc != 0)
            return belt2_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] cycle 传送带二放置结束，原地抓取。底盘留在 " << ap7
                  << "，收手只回 conveyor2.tcp\n";
        const int bg2_rc = go_belt_grasp(
            true, g_move_cfg.conveyor2, std::string(), "belt2_grasp",
            -1, {}, true, std::string(), false, false, true);
        if (bg2_rc != 0)
            return bg2_rc;
        if (hardware_abort_requested())
            return -4;
        if (!g_hw->last_grasped_r && !g_hw->last_grasped_l)
        {
            std::cerr << "[orch] cycle 传送带二没有抓到，不放料盘\n";
            return 1;
        }

        const int place_rc = cycle_stand_then_tray_place();
        if (place_rc < 0)
            return place_rc;
        if (hardware_abort_requested())
            return -4;
        std::cout << "[orch] cycle 第 " << cycle << " 圈结束，回到入口再检查站和腰\n";
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

int cycle1_stand_then_face_forward()
{
    if (g_hw == nullptr || g_hw->waist == nullptr || g_hw->layer3 == nullptr)
        return -1;
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle1抬腰前"))
        return hardware_abort_requested() ? -4 : -5;
    const TorsoInChassis torso = torso_in_chassis();
    const auto &home = g_move_cfg.waist.layer3_home;
    const double x = torso.ok ? torso.t.x() : home(0);
    const double y = torso.ok ? torso.t.y() : home(1);
    const double yaw_deg = torso.ok ? std::atan2(torso.R(1, 0), torso.R(0, 0)) / rad : 0.0;
    const bool already_up = std::abs(yaw_deg) < 1.0 && std::abs((*g_hw->layer3)(2) - home(2)) < 0.02;
    if (!already_up)
    {
        std::cout << std::fixed << std::setprecision(1)
                  << "[orch] cycle1 先把腰抬到直立 z=" << std::setprecision(3) << home(2)
                  << "，x 保持 " << x << "，朝向先保持 yaw=" << std::setprecision(1)
                  << yaw_deg << " deg\n";
        const int up_rc = g_hw->waist->moveToYawPitch(x, y, home(2), yaw_deg, 0.0, 0.06);
        if (up_rc != 0)
        {
            std::cerr << "[orch] cycle1 抬腰失败 code=" << up_rc << "\n";
            return hardware_abort_requested() ? -4 : -5;
        }
        if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle1抬腰后"))
            return hardware_abort_requested() ? -4 : -5;
        if (hardware_abort_requested())
            return -4;
    }
    if (std::abs(yaw_deg) < 1.0)
    {
        std::cout << "[orch] cycle1 腰已经朝前，不再为转正改 x\n";
        move_box::waist_layer3_set_synced(true);
        return 0;
    }
    const double x_now = (*g_hw->layer3)(0);
    const double y_now = (*g_hw->layer3)(1);
    std::cout << std::fixed << std::setprecision(3)
              << "[orch] cycle1 腰转正，x 保持 " << x_now << " z=" << home(2)
              << " yaw=0 pitch=0\n";
    const int yaw_rc = g_hw->waist->moveToYawPitch(x_now, y_now, home(2), 0.0, 0.0, 0.06);
    if (yaw_rc != 0)
    {
        std::cerr << "[orch] cycle1 转正失败 code=" << yaw_rc << "\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle1转正后"))
        return hardware_abort_requested() ? -4 : -5;
    move_box::waist_layer3_set_synced(true);
    return hardware_abort_requested() ? -4 : 0;
}

int cycle1_belt_yaw_and_pitch(const ConveyorStationConfig &c)
{
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle1转腰前"))
        return hardware_abort_requested() ? -4 : -5;
    const double y = (*g_hw->layer3)(1);
    std::cout << std::fixed << std::setprecision(1)
              << "[orch] cycle1 转腰和弯腰一起做 yaw=" << c.same_spot_yaw_deg
              << " pitch=" << c.same_spot_pitch_deg << " deg\n";
    const int pose_rc = g_hw->waist->moveToYawPitch(
        c.waist_x, y, c.waist_z, c.same_spot_yaw_deg, c.same_spot_pitch_deg, 0.10);
    if (pose_rc != 0)
    {
        std::cerr << "[orch] cycle1 转腰弯腰失败 code=" << pose_rc << "\n";
        return hardware_abort_requested() ? -4 : -5;
    }
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "cycle1转腰弯腰后"))
        std::cout << "[orch] cycle1 关节已到位。弯腰后笛卡尔回读可能超出直立行程\n";
    move_box::waist_layer3_set_synced(true);
    return hardware_abort_requested() ? -4 : 0;
}

int vision_grasp_belt1()
{
    const std::string &station = g_move_cfg.cycle1_station;
    std::cout << "[orch] cycle1：" << station
              << " 上循环。先抬腰、转正、手到防碰姿态，再导航到站；"
              << "到站后降腰同时把手收到料盘抓取准备，然后抓取；"
              << "抓完先升腰并把 x 后退，然后转腰和弯腰一起做，再把手收到传送带准备，然后拍照放置。"
              << "底盘不再去别的站。abort 才停\n";

    for (int cycle = 1;; ++cycle)
    {
        std::cout << "\n======== [orch] cycle1 第 " << cycle << " 圈 ========\n";
        std::cout.flush();
        if (hardware_abort_requested())
            return -4;
        if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr ||
            g_hw->waist == nullptr || g_hw->layer3 == nullptr)
            return -1;

        const int up_rc = cycle1_stand_then_face_forward();
        if (up_rc != 0)
            return up_rc;
        if (!move_box::move_arms_to_pose(
                *g_hw->arm_r, *g_hw->arm_l,
                g_move_cfg.cycle1_transit.right, g_move_cfg.cycle1_transit.left,
                "cycle1 防碰姿态"))
        {
            std::cerr << "[orch] cycle1 手到防碰姿态失败\n";
            return hardware_abort_requested() ? -4 : -5;
        }
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] cycle1 防碰姿态已到，底盘去 " << station << "\n";
        const int nav_rc = chassis_goto_station(station);
        if (nav_rc != 0)
            return nav_rc;
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] cycle1 已在 " << station
                  << "，降腰和双手同时到料盘抓取准备\n";
        std::atomic<int> arm_rc{0};
        std::atomic<int> head_rc{0};
        std::thread arm_th([&]() {
            if (!move_box::move_arms_to_standby(*g_hw->arm_r, *g_hw->arm_l))
                arm_rc = hardware_abort_requested() ? -4 : -5;
        });
        std::thread head_th([&]() { head_rc = enable_head_calib_pose(); });
        const bool waist_ok =
            move_box::ensure_waist_ready_start(*g_hw->waist, *g_hw->layer3);
        arm_th.join();
        head_th.join();
        if (hardware_abort_requested())
            return -4;
        if (!waist_ok)
        {
            std::cerr << "[orch] cycle1 降到抓取腰位失败\n";
            return -5;
        }
        if (arm_rc.load() != 0)
            return arm_rc.load();
        if (head_rc.load() != 0)
        {
            std::cerr << "[orch] cycle1 头到抓取标定角失败 code=" << head_rc.load() << "\n";
            return head_rc.load();
        }
        move_box::waist_layer3_set_synced(true);

        const int grasp_rc = vision_grasp_with_engine(SegEngineId::Default, false, false, true);
        if (grasp_rc < 0)
            return grasp_rc;
        if (!g_hw->last_grasped_r && !g_hw->last_grasped_l)
        {
            std::cerr << "[orch] cycle1 没有持件，不放置\n";
            return grasp_rc == 0 ? 1 : grasp_rc;
        }
        if (hardware_abort_requested())
            return -4;

        const auto &belt = g_move_cfg.conveyor;
        std::cout << "[orch] cycle1 抓取结束，头开始去传送带拍照角，同时升腰并把 x 后退\n";
        std::atomic<int> head_belt_rc{0};
        std::thread belt_head_th([&]() {
            head_belt_rc = move_head_to_rpy_deg(
                belt.head_yaw_deg, belt.head_pitch_deg, belt.head_roll_deg, true);
        });
        const int xz_rc = move_conveyor_photo_waist_xz(belt, "cycle1");
        if (xz_rc != 0)
        {
            if (belt_head_th.joinable())
                belt_head_th.join();
            return xz_rc;
        }
        if (hardware_abort_requested())
        {
            if (belt_head_th.joinable())
                belt_head_th.join();
            return -4;
        }
        const int lean_rc = cycle1_belt_yaw_and_pitch(belt);
        if (lean_rc != 0)
        {
            if (belt_head_th.joinable())
                belt_head_th.join();
            return lean_rc;
        }
        if (hardware_abort_requested())
        {
            if (belt_head_th.joinable())
                belt_head_th.join();
            return -4;
        }
        Eigen::Matrix<double, 1, 6> belt_r = belt.tcp.right;
        Eigen::Matrix<double, 1, 6> belt_l = belt.tcp.left;
        std::cout << "[orch] cycle1 双手直接到传送带准备 tcp，不先按料盘腕角上抬\n";
        const ArmLineMoveResult belt_arm = arm_dual_line_move(
            *g_hw->arm_r, belt_r, *g_hw->arm_l, belt_l, 0.15);
        if (belt_head_th.joinable())
            belt_head_th.join();
        if (belt_arm.ret_r != 0 || belt_arm.ret_l != 0)
        {
            std::cerr << "[orch] cycle1 手到传送带准备 tcp 失败 右=" << belt_arm.ret_r
                      << " 左=" << belt_arm.ret_l << "\n";
            return hardware_abort_requested() ? -4 : -5;
        }
        if (head_belt_rc.load() != 0)
        {
            std::cerr << "[orch] cycle1 头到传送带拍照角失败 code=" << head_belt_rc.load() << "\n";
            return head_belt_rc.load();
        }
        if (hardware_abort_requested())
            return -4;

        std::cout << "[orch] cycle1 腰已到位，停稳 1s 再拍传送带\n";
        hardware_abort_sleep_ms(1000);
        if (hardware_abort_requested())
            return -4;
        std::cout << "[orch] cycle1 开始拍照放置\n";
        const int belt_rc = go_belt_station(belt, "belt", {}, true, true, true);
        if (belt_rc != 0)
            return belt_rc;
        if (hardware_abort_requested())
            return -4;
        std::cout << "[orch] cycle1 本圈放置结束，下一圈仍在 " << station << "\n";
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

struct DispatchPoints
{
    std::string raw;
    std::string load;
    std::string unload;
    std::string well;
    std::string charge;
};

DispatchPoints dispatch_snap_points()
{
    DispatchPoints p;
    p.raw = RobotClient::raw_material_point;
    p.load = RobotClient::load_point;
    p.unload = RobotClient::unload_point;
    p.well = RobotClient::well_done_point;
    p.charge = RobotClient::charge_point;
    return p;
}

void dispatch_log_points(const DispatchPoints &p)
{
    std::cout << "[dispatch] 调度点位 上料抓=" << (p.raw.empty() ? "(空)" : p.raw)
              << " 上料放/转运抓=" << (p.load.empty() ? "(空)" : p.load)
              << " 转运放/下料抓=" << (p.unload.empty() ? "(空)" : p.unload)
              << " 下料放=" << (p.well.empty() ? "(空)" : p.well) << "\n";
}

bool dispatch_need_point(const std::string &station, const char *what)
{
    if (!station.empty())
        return true;
    std::cerr << "[dispatch] 调度没有下发" << what << "，这个子任务不动\n";
    return false;
}

int dispatch_drive(const std::string &station)
{
    if (station.empty())
    {
        std::cerr << "[dispatch] 目标站点为空，底盘不动\n";
        return -1;
    }
    std::string cur;
    const int loc_rc = chassis_query_station(cur);
    if (loc_rc != 0)
        return loc_rc;
    if (station_name_matches(cur, station))
    {
        std::cout << "[dispatch] 底盘已在 " << cur << "，这两个点重复，不再发导航\n";
        return 0;
    }
    const int high_rc = cycle_ensure_waist_high_before_drive();
    if (high_rc != 0)
        return high_rc;
    std::cout << "[dispatch] 腰已在高位，底盘去 " << station << "\n";
    return chassis_goto_station(station);
}

int dispatch_tuck_exit()
{
    std::cout << "[dispatch] 子任务退出：双手到传送带一准备 tcp，腰回 home 高位，再允许底盘动\n";
    const int arm_rc = cycle_arms_to_tcp(g_move_cfg.conveyor, "子任务退出");
    if (arm_rc != 0)
        return arm_rc;
    const int home_rc = cycle_waist_to_home();
    if (home_rc != 0)
        return home_rc;
    return cycle_ensure_waist_high_before_drive();
}

int dispatch_return_standby(const std::string &standby)
{
    g_dispatch_nav_fault.clear();
    g_photo_ready_station.clear();
    if (standby.empty())
    {
        std::cerr << "[dispatch] 还没有上料抓取点，不能回待命\n";
        return -1;
    }
    const int tuck_rc = dispatch_tuck_exit();
    if (tuck_rc != 0)
        return tuck_rc;
    std::cout << "[dispatch] 退出姿态完成，回待命点（上料抓取点）" << standby << "\n";
    return dispatch_drive(standby);
}

int dispatch_face_tray2()
{
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "下料起身前"))
        return hardware_abort_requested() ? -4 : -5;
    const auto &home = g_move_cfg.waist.layer3_home;
    const double y = (*g_hw->layer3)(1);
    constexpr double kFaceYaw = 180.0;
    std::cout << std::fixed << std::setprecision(3)
              << "[dispatch] 下料放置前起身并转到 yaw=" << kFaceYaw
              << "，目标 x=" << home(0) << " z=" << home(2) << "\n";
    const int up_rc = g_hw->waist->moveToYawPitch(home(0), y, home(2), kFaceYaw, 0.0, 0.08);
    if (up_rc != 0)
    {
        std::cerr << "[dispatch] 起身转到 180° 失败 code=" << up_rc << "\n";
        return hardware_abort_requested() ? -4 : up_rc;
    }
    if (!move_box::sync_waist_layer3_from_encoders(*g_hw->waist, *g_hw->layer3, "下料起身后"))
        return hardware_abort_requested() ? -4 : -5;
    return hardware_abort_requested() ? -4 : 0;
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
    }, photo_settle, only_hand == 1 ? depart_after_right : std::string(),
       false, false, true);
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
    std::string cur;
    const int loc_rc = chassis_query_station(cur);
    if (loc_rc != 0)
        return loc_rc;
    if (!station_name_matches(cur, station))
    {
        const int high_rc = cycle_ensure_waist_high_before_drive();
        if (high_rc != 0)
            return high_rc;
    }
    std::cout << "[dispatch] 去 " << station << "，路上同时摆到拍照准备\n";
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

int dispatch_load(RobotClient &client, const std::string &grasp_station, const std::string &place_station)
{
    if (!dispatch_need_point(grasp_station, "上料抓取点") ||
        !dispatch_need_point(place_station, "上料放置点"))
        return -1;
    std::cout << "[dispatch] 上料：底盘去 " << grasp_station
              << " 抓，再去 " << place_station
              << " 放。两点一样就原地放\n";
    std::string cur;
    const int loc_rc = chassis_query_station(cur);
    if (loc_rc != 0)
        return loc_rc;
    if (!station_name_matches(cur, grasp_station))
    {
        const int tuck_rc = dispatch_tuck_exit();
        if (tuck_rc != 0)
            return tuck_rc;
        const int go_rc = dispatch_drive(grasp_station);
        if (go_rc != 0)
            return go_rc;
    }
    const int grasp_rc = vision_grasp_with_engine(SegEngineId::Default, false, false);
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
    const bool same_spot = station_name_matches(grasp_station, place_station);
    if (same_spot || chassis_already_at(place_station))
    {
        std::cout << "[dispatch] 上料放置和抓取是同一个底盘点，原地升腰再转，底盘不动\n";
        const int pose_rc = cycle_raise_then_belt1_pose();
        if (pose_rc != 0)
            return pose_rc;
    }
    else
    {
        const int tuck_rc = dispatch_tuck_exit();
        if (tuck_rc != 0)
            return tuck_rc;
        const int ready_rc = drive_belt_ready(place_station, g_move_cfg.conveyor, "dispatch_load");
        if (ready_rc != 0)
            return ready_rc;
    }
    const int place_rc = dispatch_place_belt(
        client, [](RobotClient &c, int timeout_ms) { return c.apply_place_raw_material(timeout_ms); },
        dispatch_msg::APPLY_PLACE_RAW, dispatch_msg::PLACE,
        g_move_cfg.conveyor, "belt", hold_r, hold_l, "上料放置", true);
    if (place_rc != 0)
        return place_rc;
    const int done_rc = dispatch_report(client, DispatchNote::Mission, "上料");
    if (done_rc != 0)
        return done_rc;
    std::cout << "[dispatch] 上料完成，手已收回，先等下一条是不是转运\n";
    return 0;
}

int dispatch_transfer(RobotClient &client, const std::string &grasp_station, const std::string &place_station)
{
    if (!dispatch_need_point(grasp_station, "转运抓取点") ||
        !dispatch_need_point(place_station, "转运放置点"))
        return -1;
    std::cout << "[dispatch] 转运：在 " << grasp_station
              << " 抓，再到 " << place_station << " 放\n";
    if (!belt_ready_to_pick(grasp_station, g_move_cfg.conveyor, "转运"))
    {
        std::string cur;
        const int loc_rc = chassis_query_station(cur);
        if (loc_rc != 0)
            return loc_rc;
        if (!station_name_matches(cur, grasp_station))
        {
            const int tuck_rc = dispatch_tuck_exit();
            if (tuck_rc != 0)
                return tuck_rc;
        }
        const int ready_rc = drive_belt_ready(grasp_station, g_move_cfg.conveyor, "dispatch_transfer");
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
        transfer_grasp, grasp_station, "belt_grasp",
        true, !right_arm_motors_locked(), "转运左手抓取", "转运右手抓取");
    if (pick_rc < 0)
        return pick_rc;
    const bool hold_r = g_hw->last_grasped_r;
    const bool hold_l = g_hw->last_grasped_l;
    ConveyorStationConfig transfer_place = g_move_cfg.conveyor2;
    constexpr double kTransferPlaceZBiasM = -0.005;
    transfer_place.place_base_z_bias_m = kTransferPlaceZBiasM;
    if (!(hold_r || hold_l))
        std::cout << "[dispatch] 转运没有抓到件，不去放置点\n";
    else
    {
        std::cout << std::fixed << std::setprecision(4)
                  << "[dispatch] 转运放置松手高度在基座 Z 再低 "
                  << -kTransferPlaceZBiasM << " m，左右手都加\n";
        const bool same_spot = station_name_matches(grasp_station, place_station);
        std::atomic<bool> granted{false};
        std::thread apply_th([&]() {
            std::cout << "[dispatch] 发送 " << dispatch_msg::APPLY_PLACE_HALF
                      << "，等待 " << dispatch_msg::PLACE << "\n";
            granted = client.apply_place_half_done(kDispatchApplyTimeoutMs);
        });
        int pose_rc = 0;
        if (same_spot || chassis_already_at(place_station))
        {
            std::cout << "[dispatch] 转运放置和抓取是同一个底盘点，原地摆传送带二，底盘不动\n";
            pose_rc = go_belt_ready(*g_hw->arm_r, *g_hw->arm_l, transfer_place, "belt2");
        }
        else
            pose_rc = cycle_travel_to_belt2(place_station);
        if (apply_th.joinable())
            apply_th.join();
        if (pose_rc != 0)
            return pose_rc;
        if (!granted.load())
        {
            std::cout << "[dispatch] 转运放置 申请 5s 未允许，手里还有件，不放置，不报完成\n";
            return kDispatchHoldStay;
        }
        std::cout << "[dispatch] 已收到 " << dispatch_msg::PLACE << "，转运放置 放行\n";
        const int place_rc = go_belt_station(
            transfer_place, "belt2", {}, hold_r, hold_l, true);
        if (place_rc != 0)
            return place_rc;
        const int hand_rc = dispatch_report(client, DispatchNote::PlaceHand, "转运放置");
        if (hand_rc != 0)
            return hand_rc;
    }
    const int done_rc = dispatch_report(client, DispatchNote::Mission, "转运");
    if (done_rc != 0)
        return done_rc;
    std::cout << "[dispatch] 转运完成，手已收回，先等下一条是不是下料\n";
    return 0;
}

int dispatch_unload(RobotClient &client, const std::string &grasp_station, const std::string &place_station)
{
    if (!dispatch_need_point(grasp_station, "下料抓取点") ||
        !dispatch_need_point(place_station, "下料放置点"))
        return -1;
    std::cout << "[dispatch] 下料：在 " << grasp_station
              << " 抓，再到 " << place_station << " 放\n";
    if (!belt_ready_to_pick(grasp_station, g_move_cfg.conveyor2, "下料"))
    {
        std::string cur;
        const int loc_rc = chassis_query_station(cur);
        if (loc_rc != 0)
            return loc_rc;
        if (!station_name_matches(cur, grasp_station))
        {
            const int tuck_rc = dispatch_tuck_exit();
            if (tuck_rc != 0)
                return tuck_rc;
        }
        const int ready_rc = drive_belt_ready(grasp_station, g_move_cfg.conveyor2, "dispatch_unload");
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
        g_move_cfg.conveyor2, grasp_station, "belt2_grasp",
        pick_left, pick_right, "下料左手抓取", "下料右手抓取");
    if (pick_rc < 0)
        return pick_rc;
    const bool hold_r = g_hw->last_grasped_r;
    const bool hold_l = g_hw->last_grasped_l;
        if (hold_r || hold_l)
    {
        if (!station_name_matches(grasp_station, place_station) &&
            !chassis_already_at(place_station))
        {
            const int tuck_rc = dispatch_tuck_exit();
            if (tuck_rc != 0)
                return tuck_rc;
            const int go_rc = dispatch_drive(place_station);
            if (go_rc != 0)
                return go_rc;
        }
        else
            std::cout << "[dispatch] 下料放置和抓取是同一个底盘点，原地放\n";
        const int face_rc = dispatch_face_tray2();
        if (face_rc != 0)
            return face_rc;
        const int place_rc = vision_place_tray2(hold_r, hold_l, false, true, 180.0, place_station);
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
        std::cout << "[dispatch] 这次没有抓到成品，不去放置点\n";
    const int done_rc = dispatch_report(client, DispatchNote::Mission, "下料");
    if (done_rc != 0)
        return done_rc;
    std::cout << "[dispatch] 下料完成。下一条如果不是顺着来的，就退出并回上料抓取点\n";
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

bool dispatch_flow_continues(const std::string &done, const std::string &next)
{
    return (done == dispatch_msg::TASK_LOAD && next == dispatch_msg::TASK_TRANSFER) ||
           (done == dispatch_msg::TASK_TRANSFER && next == dispatch_msg::TASK_UNLOAD);
}

std::atomic<bool> g_status_writer_run{false};
std::thread g_status_writer;

void write_scheduler_status_once()
{
    if (!chassis_connected())
        return;
    Seer &robot = shared_chassis();
    const int prev_power = RobotClient::power_percent;
    double x = 0, y = 0, angle = 0, confidence = 0;
    std::string cur, last;
    int loc_state = 0;
    bool in_forbidden = false;
    const int loc_rc = robot.GetLocation(x, y, angle, cur, last, confidence, loc_state, in_forbidden);
    if (loc_rc < 0)
        return;
    if (loc_rc == 0)
    {
        RobotClient::x = static_cast<float>(x);
        RobotClient::y = static_cast<float>(y);
        RobotClient::angle = static_cast<float>(angle);
    }
    double level = 0, voltage = 0, current = 0, battery_temp = 0;
    bool charging = false;
    int battery_cycle = 0;
    const int bat_rc = robot.GetBattery(level, charging, voltage, current, battery_temp, battery_cycle);
    if (bat_rc < 0)
        return;
    if (bat_rc == 0)
    {
        double pct = level;
        if (pct <= 1.5)
            pct *= 100.0;
        if (pct < 0.0)
            pct = 0.0;
        if (pct > 100.0)
            pct = 100.0;
        RobotClient::power_percent = static_cast<int>(std::lround(pct));
    }
    static bool logged_once = false;
    if (!logged_once || RobotClient::power_percent != prev_power)
    {
        logged_once = true;
        std::cout << std::fixed << std::setprecision(3)
                  << "[dispatch] 写入上报变量 电量=" << RobotClient::power_percent
                  << "% x=" << RobotClient::x << " y=" << RobotClient::y
                  << " angle=" << std::setprecision(3) << RobotClient::angle
                  << " rad，2Hz；库每 2s 发给调度\n";
    }
}

void scheduler_status_writer_loop()
{
    std::cout << "[dispatch] 状态写入线程已启动，2Hz 更新电量和位姿\n";
    while (g_status_writer_run.load() && !app_stop_requested())
    {
        const auto tick = std::chrono::steady_clock::now();
        write_scheduler_status_once();
        const auto spent = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - tick)
                               .count();
        int remain = static_cast<int>(std::max<int64_t>(0, 500 - spent));
        while (remain > 0 && g_status_writer_run.load() && !app_stop_requested())
        {
            const int slice = std::min(remain, 50);
            std::this_thread::sleep_for(std::chrono::milliseconds(slice));
            remain -= slice;
        }
    }
}

bool disable_body_motors()
{
    std::cout << "[dispatch] 失能腰、双臂、头\n";
    bool ok = true;
    auto one = [&](uint32_t id) {
        if (!motor_disable_id(id))
            ok = false;
    };
    for (uint32_t id : kWaistAllCanIds)
        one(id);
    for (uint32_t id : kLeftArmCanIds)
        one(id);
    if (!right_arm_motors_locked())
    {
        for (uint32_t id : kRightArmCanIds)
            one(id);
    }
    for (uint32_t id : kHeadCanIds)
        one(id);
    return ok;
}

void enable_body_motors()
{
    std::cout << "[dispatch] 重新使能腰、双臂、头\n";
    for (uint32_t id : kWaistAllCanIds)
        motor_enable_id(id);
    for (uint32_t id : kLeftArmCanIds)
        motor_enable_id(id);
    if (!right_arm_motors_locked())
    {
        for (uint32_t id : kRightArmCanIds)
            motor_enable_id(id);
    }
    for (uint32_t id : kHeadCanIds)
        motor_enable_id(id);
}

double battery_percent_now()
{
    if (!chassis_connected())
        return -1.0;
    double level = 0, voltage = 0, current = 0, battery_temp = 0;
    bool charging = false;
    int battery_cycle = 0;
    const int rc = shared_chassis().GetBattery(
        level, charging, voltage, current, battery_temp, battery_cycle);
    if (rc != 0)
        return -1.0;
    double pct = level;
    if (pct <= 1.5)
        pct *= 100.0;
    return pct;
}

int dispatch_charge(RobotClient &client, const std::string &station)
{
    if (station.empty())
    {
        std::cerr << "[dispatch] 充电点为空，不去充电\n";
        return -1;
    }
    std::cout << "[dispatch] 充电：先收到开车姿态，再失能全身电机，然后去 " << station << "\n";
    const int arm_rc = cycle_arms_to_tcp(g_move_cfg.conveyor, "充电");
    if (arm_rc != 0)
        return arm_rc;
    const int high_rc = cycle_ensure_waist_high_before_drive();
    if (high_rc != 0)
        return high_rc;
    if (!disable_body_motors())
    {
        std::cerr << "[dispatch] 失能没有全部成功，仍去充电点\n";
    }
    const int nav_rc = chassis_goto_station(station);
    if (nav_rc != 0)
    {
        enable_body_motors();
        return nav_rc;
    }
    std::cout << "[dispatch] 已到充电点，等待电量到 95%\n";
    int last_shown = -1;
    while (!hardware_abort_requested())
    {
        const double pct = battery_percent_now();
        const int shown = pct < 0.0 ? -1 : static_cast<int>(std::lround(pct));
        if (shown != last_shown)
        {
            last_shown = shown;
            std::cout << "[dispatch] 充电中电量=" << shown << "%\n";
        }
        if (pct >= 95.0)
            break;
        hardware_abort_sleep_ms(1000);
    }
    if (hardware_abort_requested())
    {
        enable_body_motors();
        return -4;
    }
    std::cout << "[dispatch] 电量已到 95%，汇报充电完成\n";
    const bool reported = client.mission_done();
    if (!reported)
        std::cerr << "[dispatch] 发送充电完成失败\n";
    const int stop_rc = shared_chassis().SetChargingRelay("{\"status\":false}");
    if (stop_rc != 0)
        std::cerr << "[dispatch] 停止充电继电器失败 code=" << stop_rc
                  << " " << shared_chassis().LastMessage() << "\n";
    else
        std::cout << "[dispatch] 已关闭充电继电器\n";
    enable_body_motors();
    if (!reported)
        return -7;
    if (stop_rc != 0)
        return stop_rc < 0 ? stop_rc : -6;
    return 0;
}

void start_scheduler_status_writer()
{
    if (g_status_writer_run.exchange(true))
        return;
    g_status_writer = std::thread(scheduler_status_writer_loop);
}

void stop_scheduler_status_writer()
{
    if (!g_status_writer_run.exchange(false))
        return;
    if (g_status_writer.joinable())
        g_status_writer.join();
    chassis_release();
}

int run_dispatch_mode()
{
    if (g_hw == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
        return -1;
    std::cout << "[dispatch] 系统对接。型号 T170C2.0，编号 TH-T170C-0120，调度 "
              << dispatch_msg::kDefaultHost << ":" << dispatch_msg::kDefaultPort
              << "。底盘点由调度下发，不写死。"
              << "上料抓、上料放、转运抓用前两个点；转运放、下料抓、下料放用后两个点。"
              << "顺着上料→转运、转运→下料就原地接着做；否则收成开车姿态，回上料抓取点待命\n";

    RobotClient client("T170C2.0", "TH-T170C-0120");
    DispatchInbox box;
    DispatchPoints points;
    std::string mission;
    std::string done_task;
    auto take_ready = [&]() -> bool {
        if (!inbox_poll(box, mission))
            return false;
        inbox_join(box);
        const std::string keep_raw = points.raw;
        points = dispatch_snap_points();
        if (points.raw.empty())
            points.raw = keep_raw;
        dispatch_log_points(points);
        return true;
    };
    auto go_standby_then_take = [&]() -> int {
        const std::string standby = points.raw;
        dispatch_report_state(client, dispatch_msg::WORKING);
        const int back = dispatch_return_standby(standby);
        if (back != 0)
        {
            const std::string fault = dispatch_exception_state();
            if (!fault.empty())
                return dispatch_wait_system(client, fault);
            return hardware_abort_requested() ? -4 : (back < 0 ? back : -6);
        }
        if (hardware_abort_requested())
            return -4;
        std::cout << "[dispatch] 已到待命点 " << standby << "，停稳 2s。这 2s 只缓存任务，不执行\n";
        hardware_abort_sleep_ms(2000);
        if (hardware_abort_requested())
            return -4;
        dispatch_report_state(client, dispatch_msg::STANDBY);
        if (!mission.empty())
            std::cout << "[dispatch] 停稳后执行已经收到的任务 " << mission << "\n";
        else if (take_ready() && !mission.empty())
            std::cout << "[dispatch] 停稳后执行缓存任务 " << mission << "\n";
        else
            std::cout << "[dispatch] 停稳后没有缓存任务，继续等待\n";
        return 0;
    };

    dispatch_report_state(client, dispatch_msg::STANDBY);
    while (!hardware_abort_requested())
    {
        if (mission.empty())
        {
            const std::string where = points.raw.empty() ? "当前位置" : points.raw;
            std::cout << "[dispatch] 在 " << where << " 等待任务\n";
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
        if (current != "CHARGE" && current != dispatch_msg::TASK_LOAD &&
            current != dispatch_msg::TASK_TRANSFER && current != dispatch_msg::TASK_UNLOAD)
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
        if (current == "CHARGE")
            rc = dispatch_charge(client, points.charge);
        else if (current == dispatch_msg::TASK_LOAD)
            rc = dispatch_load(client, points.raw, points.load);
        else if (current == dispatch_msg::TASK_TRANSFER)
            rc = dispatch_transfer(client, points.load, points.unload);
        else
            rc = dispatch_unload(client, points.unload, points.well);
        if (rc == -4 || hardware_abort_requested())
            return -4;
        if (rc != 0)
        {
            const std::string fault = dispatch_exception_state();
            if (!fault.empty())
                return dispatch_wait_system(client, fault);
            const bool holding = g_hw != nullptr && (g_hw->last_grasped_r || g_hw->last_grasped_l);
            if (holding)
            {
                std::cerr << "[dispatch] 任务 " << current << " 未完成 code=" << rc
                          << "，手里还有件，不报完成，也不回待命点再抓。停在原地等下一条\n";
                dispatch_report_state(client, dispatch_msg::STANDBY);
                mission.clear();
                done_task.clear();
                continue;
            }
            std::cerr << "[dispatch] 任务 " << current << " 失败 code=" << rc
                      << "，不回报完成。退出后回上料抓取点再等\n";
            done_task.clear();
            inbox_start(box, client);
            const int back_rc = go_standby_then_take();
            if (back_rc != 0)
                return back_rc;
            continue;
        }

        done_task = current;
        dispatch_report_state(client, dispatch_msg::STANDBY);
        std::cout << "[dispatch] " << current << " 完成，手臂已收回，等 5s 看下一条是不是顺着的下一个子任务\n";
        const std::string standby_keep = points.raw;
        inbox_start(box, client);
        std::string soon;
        if (inbox_wait(box, 5000, soon))
        {
            inbox_join(box);
            points = dispatch_snap_points();
            if (points.raw.empty())
                points.raw = standby_keep;
            dispatch_log_points(points);
            if (hardware_abort_requested())
                return -4;
            if (dispatch_flow_continues(done_task, soon))
            {
                std::cout << "[dispatch] 5s 内收到顺着的 " << soon
                          << "，保持现在的腰和手，不回待命点\n";
                mission = std::move(soon);
                continue;
            }
            if (!soon.empty())
                std::cout << "[dispatch] 5s 内收到 " << soon
                          << "，不是顺着的下一个子任务。先退出，回上料抓取点，到了再做\n";
            else
                std::cerr << "[dispatch] 等待期间调度断线，退出后回上料抓取点再连接\n";
            mission = std::move(soon);
        }
        else if (hardware_abort_requested())
        {
            return -4;
        }
        else
            std::cout << "[dispatch] 5s 内没有顺着的下一个子任务，退出并回上料抓取点。"
                         "路上收到的任务先缓存\n";

        if (!box.running && mission.empty())
            inbox_start(box, client);
        const int back_rc = go_standby_then_take();
        if (back_rc != 0)
            return back_rc;
    }
    return hardware_abort_requested() ? -4 : 0;
}

} // namespace orch
