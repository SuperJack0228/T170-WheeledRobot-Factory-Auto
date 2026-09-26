/** T170C 调试守护进程。仅监听 127.0.0.1。 */

#include "robot_runtime.h"

#include "function.h"
#include "gripper_interface.hpp"
#include "move_box_config.h"
#include "move_box_runtime.h"
#include "seg_pose_bridge.h"
#include "Ti5_socketcan.h"

#include <json/json.h>
#include <pybind11/embed.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

MoveBoxConfig g_move_cfg;

namespace
{

std::atomic<bool> g_abort{false};
std::mutex g_hw_mu;
orch::HwSession g_session;
std::unique_ptr<gripper::Gripper> g_grip;
std::unique_ptr<Robot_Arm> g_arm_r;
std::unique_ptr<Robot_Arm> g_arm_l;
std::unique_ptr<WaistRobot> g_waist;
std::unique_ptr<PosePipeline> g_pipeline;
Eigen::Matrix<double, 1, 6> g_layer3;

Json::Value reply(bool ok, const std::string &error = {})
{
    Json::Value out(Json::objectValue);
    out["ok"] = ok;
    if (!error.empty())
        out["error"] = error;
    return out;
}

bool read_line(int fd, std::string &out)
{
    out.clear();
    char c = 0;
    while (out.size() <= (1u << 20))
    {
        const ssize_t n = ::recv(fd, &c, 1, 0);
        if (n <= 0)
            return false;
        if (c == '\n')
            return true;
        if (c != '\r')
            out.push_back(c);
    }
    return false;
}

void write_json(int fd, const Json::Value &value)
{
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    const std::string body = Json::writeString(builder, value) + "\n";
    ::send(fd, body.data(), body.size(), MSG_NOSIGNAL);
}

Json::Value xyz_json(double x, double y, double z)
{
    Json::Value out(Json::arrayValue);
    out.append(x);
    out.append(y);
    out.append(z);
    return out;
}

Json::Value with_arm_arrivals(Json::Value out)
{
    const std::vector<ArmArrivalRecord> recs = arm_arrival_log_copy();
    if (recs.empty())
        return out;
    Json::Value arr(Json::arrayValue);
    for (const ArmArrivalRecord &rec : recs)
    {
        Json::Value item(Json::objectValue);
        item["stage"] = rec.stage;
        item["hand"] = rec.hand;
        item["goal_xyz"] = xyz_json(rec.goal_x, rec.goal_y, rec.goal_z);
        item["actual_xyz"] = xyz_json(rec.actual_x, rec.actual_y, rec.actual_z);
        item["err_xyz"] = xyz_json(rec.err_x, rec.err_y, rec.err_z);
        item["err_m"] = rec.err_m;
        item["settled"] = rec.settled;
        arr.append(item);
    }
    out["arm_arrivals"] = arr;
    return out;
}

Json::Value pose_json(const Eigen::Matrix<double, 1, 6> &p)
{
    constexpr double kRadToDeg = 180.0 / M_PI;
    Json::Value out(Json::arrayValue);
    out.append(p(0));
    out.append(p(1));
    out.append(p(2));
    out.append(p(3) * kRadToDeg);
    out.append(p(4) * kRadToDeg);
    out.append(p(5) * kRadToDeg);
    return out;
}

Json::Value markers_json()
{
    Json::Value out(Json::arrayValue);
    for (const auto &hit : g_session.last_aruco_hits)
    {
        Json::Value item(Json::objectValue);
        item["id"] = hit.id;
        item["side_mm"] = hit.side_m * 1000.0;
        item["reproj_px"] = hit.reproj_px;
        item["camera_m"]["x"] = hit.cam_x;
        item["camera_m"]["y"] = hit.cam_y;
        item["camera_m"]["z"] = hit.cam_z;
        if (hit.robot_ok)
        {
            item["robot_m_deg"]["x"] = hit.robot_x;
            item["robot_m_deg"]["y"] = hit.robot_y;
            item["robot_m_deg"]["z"] = hit.robot_z;
            item["robot_m_deg"]["rx"] = hit.robot_rx_deg;
            item["robot_m_deg"]["ry"] = hit.robot_ry_deg;
            item["robot_m_deg"]["rz"] = hit.robot_rz_deg;
        }
        out.append(item);
    }
    return out;
}

Json::Value json_finite_or_null(double v)
{
    if (!std::isfinite(v))
        return Json::Value(Json::nullValue);
    return Json::Value(v);
}

Json::Value tray_holes_json()
{
    Json::Value out(Json::arrayValue);
    for (const auto &h : g_session.last_tray_holes)
    {
        Json::Value item(Json::objectValue);
        item["id"] = h.id;
        item["row"] = h.row;
        item["col"] = h.col;
        item["x"] = h.x;
        item["y"] = h.y;
        item["tray_z"] = json_finite_or_null(h.tray_z);
        item["tray_z_level"] = json_finite_or_null(h.tray_z_level);
        item["top_z"] = json_finite_or_null(h.top_z);
        item["top_z_level"] = json_finite_or_null(h.top_z_level);
        item["height_on_tray"] = json_finite_or_null(h.height_on_tray);
        item["class_id"] = h.class_id;
        item["class_name"] = h.class_name;
        item["conf"] = h.conf;
        item["dxy"] = (h.dxy >= 0.0) ? Json::Value(h.dxy) : Json::Value(Json::nullValue);
        item["depth_pts"] = h.depth_pts;
        out.append(item);
    }
    return out;
}

bool init_hardware(std::string &err)
{
    if (!load_move_box_config(default_move_box_config_path(), g_move_cfg, err))
        return false;
    print_move_box_config(g_move_cfg);
    pose_vis_set_save_debug(kDebugVisualize);
    pose_vis_set_gui_enabled(false);

    if (!init_socketcan())
    {
        err = device_scan_error();
        return false;
    }
    hand_socketid_bind();

    gripper::Config grip_cfg;
    grip_cfg.grasp_torque_limit_nm = 4.5;
    grip_cfg.soft_torque_limit_nm = 4.0;
    grip_cfg.pos_filter = 0.5;
    grip_cfg.soft_slew_rate = 100.0;
    grip_cfg.soft_coast_margin_rad = 0.12;
    grip_cfg.detect_retries = 2;
    try
    {
        g_grip = std::make_unique<gripper::Gripper>(grip_cfg);
        if (!g_grip->start())
        {
            err = "夹爪启动失败：一个夹爪都没有";
            return false;
        }
    }
    catch (const std::exception &ex)
    {
        err = std::string("夹爪初始化失败: ") + ex.what();
        return false;
    }

    g_pipeline = std::make_unique<PosePipeline>();
    if (!g_pipeline->init(err))
        return false;

    g_waist = std::make_unique<WaistRobot>();
    const std::string arm_yaml = project_root_dir() + "/config/Robot_Arm_Model.yaml";
    g_arm_r = std::make_unique<Robot_Arm>("T7", "T170", "right", arm_yaml);
    g_arm_l = std::make_unique<Robot_Arm>("T7", "T170", "left", arm_yaml);
    rev_motor_error(0);
    rev_motor_error(1);
    motor_speed_change();

    // 腰的 layer3 是绝对笛卡尔位姿，不能假设上电时它在 home：上一次进程可能把它停在别处，
    // 按错误的起点算出的"降腰"目标会把躯干移到非预期高度。这里只回读不运动。
    g_layer3 = g_move_cfg.waist.layer3_home;
    if (!move_box::sync_waist_layer3_from_encoders(*g_waist, g_layer3, "启动"))
    {
        g_layer3 = g_move_cfg.waist.layer3_home;
        std::cerr << "[waist] 警告：腰部位置未知，抓取流程中的腰部运动已禁用，请先执行 home\n";
    }
    g_session.g = g_grip.get();
    g_session.arm_r = g_arm_r.get();
    g_session.arm_l = g_arm_l.get();
    g_session.waist = g_waist.get();
    g_session.layer3 = &g_layer3;
    g_session.pipeline = g_pipeline.get();
    g_session.abort = &g_abort;
    orch::g_hw = &g_session;
    can_io_fault_clear();
    return true;
}

Json::Value dispatch(const Json::Value &request)
{
    const std::string cmd = request.get("cmd", "").asString();
    if (cmd == "ping")
        return reply(true);
    if (cmd == "abort")
    {
        g_abort.store(true);
        hardware_abort_request();
        return reply(true);
    }

    std::lock_guard<std::mutex> lock(g_hw_mu);
    g_abort.store(false);
    hardware_abort_clear();
    can_io_fault_clear();
    can_io_watch_begin();
    struct WatchEnd
    {
        ~WatchEnd() { can_io_watch_end(); }
    } watch_end;

    if (cmd == "snapshot")
    {
        Json::Value out = reply(true);
        out["right_tcp_m_deg"] = pose_json(arm_get_tcp_pos(*g_arm_r));
        out["left_tcp_m_deg"] = pose_json(arm_get_tcp_pos(*g_arm_l));
        return out;
    }
    if (cmd == "reload_config")
    {
        std::string err;
        if (!load_move_box_config(default_move_box_config_path(), g_move_cfg, err))
            return reply(false, err);
        return reply(true);
    }
    if (cmd == "home")
    {
        const int rc = orch::go_home(*g_arm_r, *g_arm_l);
        return rc == 0 ? reply(true) : reply(false, "归位失败/中止，code=" + std::to_string(rc));
    }
    if (cmd == "tray2ready" || cmd == "tray2ready1" || cmd == "tray2ready2" ||
        cmd == "tray2ready3" || cmd == "tray2ready6")
    {
        int group = 1;
        if (cmd == "tray2ready6")
            group = 6;
        else if (cmd.size() >= 11 && cmd.back() >= '1' && cmd.back() <= '3')
            group = cmd.back() - '0';
        else if (request.isMember("row_group") && request["row_group"].isInt())
            group = request["row_group"].asInt();
        const int rc = orch::go_tray2_ready(*g_arm_r, *g_arm_l, group);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("料盘2准备姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["row_group"] = group;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
    if (cmd == "grasp_ready" || cmd == "grasp_ready1" || cmd == "grasp_ready2" ||
        cmd == "grasp_ready3" || cmd == "grasp_ready6")
    {
        int group = 1;
        if (cmd == "grasp_ready6")
            group = 6;
        else if (cmd.size() >= 12 && cmd.back() >= '1' && cmd.back() <= '3')
            group = cmd.back() - '0';
        else if (request.isMember("row_group") && request["row_group"].isInt())
            group = request["row_group"].asInt();
        const int rc = orch::go_grasp_ready(*g_arm_r, *g_arm_l, group);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("抓取准备姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["row_group"] = group;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
    if (cmd == "waist1" || cmd == "waist2")
    {
        const int pose = (cmd == "waist2") ? 2 : 1;
        const int rc = orch::go_waist_debug(pose);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("腰平移失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["pose"] = pose;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        if (pose == 2 && !g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }
    if (cmd == "belt_ready")
    {
        const int rc = orch::go_belt_ready(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("皮带准备姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
    if (cmd == "belt_grasp_rpy")
    {
        const int rc = orch::go_belt_grasp_rpy(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("皮带抓取姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
    if (cmd == "belt_place")
    {
        const int rc = orch::go_belt_place(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("皮带放置姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
    if (cmd == "belt")
    {
        const int rc = orch::go_belt();
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("皮带工位失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["station"] = g_move_cfg.chassis.belt_station;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        out["head_yaw_deg"] = g_move_cfg.conveyor.head_yaw_deg;
        out["head_pitch_deg"] = g_move_cfg.conveyor.head_pitch_deg;
        out["head_roll_deg"] = g_move_cfg.conveyor.head_roll_deg;
        out["offset_right_x"] = g_move_cfg.conveyor.offset_right.x;
        out["offset_right_y"] = g_move_cfg.conveyor.offset_right.y;
        out["offset_right_z"] = g_move_cfg.conveyor.offset_right.z;
        out["offset_left_x"] = g_move_cfg.conveyor.offset_left.x;
        out["offset_left_y"] = g_move_cfg.conveyor.offset_left.y;
        out["offset_left_z"] = g_move_cfg.conveyor.offset_left.z;
        out["belt_ok"] = g_session.last_belt_ok;
        out["belt_name"] = g_session.last_belt_name;
        out["belt_message"] = g_session.last_belt_message;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["belt_x"] = g_session.last_belt_x;
            out["belt_y"] = g_session.last_belt_y;
            out["belt_z"] = g_session.last_belt_z;
        }
        if (!g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }
    if (cmd == "belt2_grasp_rpy")
    {
        const int rc = orch::go_belt2_grasp_rpy(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("第二传送带抓取姿态失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
    if (cmd == "belt2_ready")
    {
        const int rc = orch::go_belt2_ready(*g_arm_r, *g_arm_l);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("第二传送带准备失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["station"] = g_move_cfg.chassis.out_station;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
    if (cmd == "belt2" || cmd == "belt2_place")
    {
        const int rc = orch::go_belt2();
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("第二传送带放置失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["station"] = g_move_cfg.chassis.out_station;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        out["offset_right_x"] = g_move_cfg.conveyor2.offset_right.x;
        out["offset_right_y"] = g_move_cfg.conveyor2.offset_right.y;
        out["offset_right_z"] = g_move_cfg.conveyor2.offset_right.z;
        out["offset_left_x"] = g_move_cfg.conveyor2.offset_left.x;
        out["offset_left_y"] = g_move_cfg.conveyor2.offset_left.y;
        out["offset_left_z"] = g_move_cfg.conveyor2.offset_left.z;
        out["belt_ok"] = g_session.last_belt_ok;
        out["belt_name"] = g_session.last_belt_name;
        out["belt_message"] = g_session.last_belt_message;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["belt_x"] = g_session.last_belt_x;
            out["belt_y"] = g_session.last_belt_y;
            out["belt_z"] = g_session.last_belt_z;
        }
        if (!g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }
    if (cmd == "belt2_grasp")
    {
        const int rc = orch::go_belt2_grasp();
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("第二传送带抓取失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["station"] = g_move_cfg.chassis.out_station;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        out["grasp_offset_left_x"] = g_move_cfg.conveyor2.grasp_offset_left.x;
        out["grasp_offset_left_y"] = g_move_cfg.conveyor2.grasp_offset_left.y;
        out["grasp_offset_left_z"] = g_move_cfg.conveyor2.grasp_offset_left.z;
        out["grasp_offset_right_x"] = g_move_cfg.conveyor2.grasp_offset_right.x;
        out["grasp_offset_right_y"] = g_move_cfg.conveyor2.grasp_offset_right.y;
        out["grasp_offset_right_z"] = g_move_cfg.conveyor2.grasp_offset_right.z;
        out["grasp_hover_above_m"] = g_move_cfg.conveyor2.grasp_hover_above_m;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["grasp_x"] = g_session.last_belt_x;
            out["grasp_y"] = g_session.last_belt_y;
            out["grasp_z"] = g_session.last_belt_z;
        }
        out["belt_ok"] = g_session.last_belt_ok;
        out["belt_name"] = g_session.last_belt_name;
        out["belt_message"] = g_session.last_belt_message;
        out["grasped_right"] = g_session.last_grasped_r;
        out["grasped_left"] = g_session.last_grasped_l;
        if (!g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }
    if (cmd == "belt_grasp")
    {
        const int rc = orch::go_belt_grasp();
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("皮带抓取失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        out["grasp_offset_left_x"] = g_move_cfg.conveyor.grasp_offset_left.x;
        out["grasp_offset_left_y"] = g_move_cfg.conveyor.grasp_offset_left.y;
        out["grasp_offset_left_z"] = g_move_cfg.conveyor.grasp_offset_left.z;
        out["grasp_offset_right_x"] = g_move_cfg.conveyor.grasp_offset_right.x;
        out["grasp_offset_right_y"] = g_move_cfg.conveyor.grasp_offset_right.y;
        out["grasp_offset_right_z"] = g_move_cfg.conveyor.grasp_offset_right.z;
        out["grasp_hover_above_m"] = g_move_cfg.conveyor.grasp_hover_above_m;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["grasp_x"] = g_session.last_belt_x;
            out["grasp_y"] = g_session.last_belt_y;
            out["grasp_z"] = g_session.last_belt_z;
        }
        out["belt_ok"] = g_session.last_belt_ok;
        out["belt_name"] = g_session.last_belt_name;
        out["belt_message"] = g_session.last_belt_message;
        out["grasped_right"] = g_session.last_grasped_r;
        out["grasped_left"] = g_session.last_grasped_l;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["belt_x"] = g_session.last_belt_x;
            out["belt_y"] = g_session.last_belt_y;
            out["belt_z"] = g_session.last_belt_z;
        }
        if (!g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }
    if (cmd == "waist_jog")
    {
        const int joint = request.get("joint", 2).asInt();
        const double dq = request.get("dq_rad", 0.05).asDouble();
        const int rc = orch::go_waist_jog(joint, dq);
        Json::Value out = reply(
            rc == 0, rc == 0 ? std::string() : ("腰关节点动失败/中止，code=" + std::to_string(rc)));
        out["code"] = rc;
        out["joint"] = joint;
        out["dq_rad"] = dq;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
    if (cmd == "tray2_precision" || cmd == "tray2test")
    {
        const int rc = orch::vision_place_tray2_precision();
        Json::Value out = reply(rc >= 0, rc < 0 ? "料盘2精度测试失败/中止" : "");
        out["code"] = rc;
        out["station"] = g_move_cfg.chassis.tray2_station;
        out["placed_count"] = g_session.last_grasp_count;
        return out;
    }
    if (cmd == "tray2" || cmd == "tray2_place")
    {
        const int rc = orch::vision_place_tray2();
        Json::Value out = reply(rc >= 0, rc < 0 ? "料盘2放置失败/中止" : "");
        out["code"] = rc;
        out["station"] = g_move_cfg.chassis.tray2_station;
        out["placed_right"] = g_session.last_grasped_r;
        out["placed_left"] = g_session.last_grasped_l;
        out["placed_count"] = g_session.last_grasp_count;
        if (g_session.layer3 != nullptr)
        {
            out["waist_x"] = (*g_session.layer3)(0);
            out["waist_z"] = (*g_session.layer3)(2);
        }
        return out;
    }
    if (cmd == "vision_grasp")
    {
        const int rc = orch::vision_grasp_existing_pipeline();
        Json::Value out = reply(rc >= 0, rc < 0 ? "视觉抓取失败/中止" : "");
        out["code"] = rc;
        out["grasped_right"] = g_session.last_grasped_r;
        out["grasped_left"] = g_session.last_grasped_l;
        out["grasped_count"] = g_session.last_grasp_count;
        return out;
    }
    if (cmd == "grasp_belt1")
    {
        const int rc = orch::vision_grasp_belt1();
        Json::Value out = reply(rc >= 0, rc < 0 ? "料盘1与一号传送带循环失败/中止" : "");
        out["code"] = rc;
        out["grasped_right"] = g_session.last_grasped_r;
        out["grasped_left"] = g_session.last_grasped_l;
        out["grasped_count"] = g_session.last_grasp_count;
        out["chassis_ok"] = g_session.last_chassis_ok;
        out["chassis_station"] = g_session.last_chassis_station;
        out["chassis_message"] = g_session.last_chassis_message;
        out["belt_ok"] = g_session.last_belt_ok;
        out["belt_name"] = g_session.last_belt_name;
        out["belt_message"] = g_session.last_belt_message;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["belt_x"] = g_session.last_belt_x;
            out["belt_y"] = g_session.last_belt_y;
            out["belt_z"] = g_session.last_belt_z;
        }
        if (!g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }
    if (cmd == "grasp_belt")
    {
        const int rc = orch::vision_grasp_then_belt();
        Json::Value out = reply(rc >= 0, rc < 0 ? "抓取放置失败/中止" : "");
        out["code"] = rc;
        out["grasped_right"] = g_session.last_grasped_r;
        out["grasped_left"] = g_session.last_grasped_l;
        out["grasped_count"] = g_session.last_grasp_count;
        out["chassis_ok"] = g_session.last_chassis_ok;
        out["chassis_station"] = g_session.last_chassis_station;
        out["chassis_message"] = g_session.last_chassis_message;
        out["belt_ok"] = g_session.last_belt_ok;
        out["belt_name"] = g_session.last_belt_name;
        out["belt_message"] = g_session.last_belt_message;
        if (std::isfinite(g_session.last_belt_x))
        {
            out["belt_x"] = g_session.last_belt_x;
            out["belt_y"] = g_session.last_belt_y;
            out["belt_z"] = g_session.last_belt_z;
        }
        if (!g_session.last_debug_photo.empty())
            out["photo"] = g_session.last_debug_photo;
        return out;
    }
    if (cmd == "dispatch")
    {
        const int rc = orch::run_dispatch_mode();
        Json::Value out = reply(rc >= 0, rc < 0 ? "调度对接失败/中止" : "");
        out["code"] = rc;
        out["chassis_ok"] = g_session.last_chassis_ok;
        out["chassis_station"] = g_session.last_chassis_station;
        out["chassis_message"] = g_session.last_chassis_message;
        return out;
    }
    if (cmd == "aruco_detect")
    {
        const int rc = orch::vision_aruco_detect_pipeline();
        Json::Value out = reply(rc >= 0, rc < 0 ? "头相机 ArUco 检测失败/中止" : "");
        out["code"] = rc;
        out["count"] = g_session.last_aruco_head;
        out["markers"] = markers_json();
        return out;
    }
    if (cmd == "detect_tray_holes")
    {
        const int rc = orch::vision_tray_holes_pipeline();
        Json::Value out = reply(
            rc == 0,
            rc == 0 ? std::string()
                    : (rc < 0 ? std::string("料盘孔位检测失败/中止") : g_session.last_tray_message));
        out["code"] = rc;
        out["ok_tray"] = g_session.last_tray_ok;
        out["message"] = g_session.last_tray_message;
        out["save_path"] = g_session.last_tray_save_path;
        out["reproj_px"] = g_session.last_tray_reproj_px;
        out["tilt_deg"] = json_finite_or_null(g_session.last_tray_tilt_deg);
        Json::Value ids(Json::arrayValue);
        for (int id : g_session.last_tray_used_ids)
            ids.append(id);
        out["used_ids"] = ids;
        out["holes"] = tray_holes_json();
        if (rc > 0 && g_session.last_tray_message.size())
            out["error"] = g_session.last_tray_message;
        return out;
    }
    return reply(false, "未知命令: " + cmd);
}

void handle_client(int fd)
{
    std::string line;
    if (!read_line(fd, line))
    {
        ::close(fd);
        return;
    }
    Json::Value request;
    JSONCPP_STRING error;
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(line.data(), line.data() + line.size(), &request, &error))
        write_json(fd, reply(false, "JSON 解析失败"));
    else
    {
        try
        {
            arm_arrival_log_clear();
            write_json(fd, with_arm_arrivals(dispatch(request)));
        }
        catch (const std::exception &ex)
        {
            write_json(fd, reply(false, ex.what()));
        }
    }
    ::close(fd);
}

} // namespace

int main(int argc, char **argv)
{
    int port = 8099;
    for (int i = 1; i + 1 < argc; ++i)
        if (std::string(argv[i]) == "--port")
            port = std::atoi(argv[++i]);

    install_app_sigint_handler();
    std::string error;
    if (!init_hardware(error))
    {
        std::cerr << "[t170c_debug] 初始化失败: " << error << "\n";
        return 1;
    }

    const int server = ::socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
    if (bind(server, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 || listen(server, 8) != 0)
    {
        std::cerr << "[t170c_debug] 无法监听 127.0.0.1:" << port << "\n";
        return 1;
    }

    std::cerr << "T170C_DEBUG_READY port=" << port << " chassis=" << g_move_cfg.chassis.host
              << " belt=" << g_move_cfg.chassis.belt_station
              << " out=" << g_move_cfg.chassis.out_station << "\n";
    pybind11::gil_scoped_release release_gil;
    while (!app_stop_requested())
    {
        const int client = accept(server, nullptr, nullptr);
        if (client >= 0)
            std::thread(handle_client, client).detach();
    }
    ::close(server);
    return 0;
}
