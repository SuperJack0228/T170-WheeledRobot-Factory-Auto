/** 编排台硬件守护进程：无底盘。JSON 行协议 127.0.0.1:8099 */

#include "robot_runtime.h"

#include "gripper_interface.hpp"
#include "head.h"
#include "move_box_config.h"
#include "move_box_runtime.h"
#include "reals_tcp.h"
#include "seg_pose_bridge.h"
#include "Ti5_Arm.h"
#include "Ti5_socketcan.h"
#include "waist.h"

#include <json/json.h>
#include <pybind11/embed.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

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

constexpr double kDeg = M_PI / 180.0;

Json::Value pose_to_json(const Eigen::Matrix<double, 1, 6> &p)
{
    Json::Value o(Json::objectValue);
    o["x"] = p(0);
    o["y"] = p(1);
    o["z"] = p(2);
    o["rx"] = p(3) / kDeg;
    o["ry"] = p(4) / kDeg;
    o["rz"] = p(5) / kDeg;
    return o;
}

Json::Value xyz_to_json(const Eigen::Vector3d &p)
{
    Json::Value o(Json::objectValue);
    o["x"] = p.x();
    o["y"] = p.y();
    o["z"] = p.z();
    return o;
}

Json::Value links_to_json(const ArmLinkFk &L)
{
    Json::Value o(Json::objectValue);
    o["shoulder"] = xyz_to_json(L.shoulder);
    o["elbow"] = xyz_to_json(L.elbow);
    o["wrist"] = xyz_to_json(L.wrist);
    o["flange"] = xyz_to_json(L.flange);
    o["tcp"] = xyz_to_json(L.tcp);
    o["tcp_mdh_err"] = L.tcp_mdh_err;
    return o;
}

orch::PoseMrd pose_from_json(const Json::Value &v)
{
    orch::PoseMrd p;
    p.x = v.get("x", 0.0).asDouble();
    p.y = v.get("y", 0.0).asDouble();
    p.z = v.get("z", 0.0).asDouble();
    p.rx = v.get("rx", 0.0).asDouble() * kDeg;
    p.ry = v.get("ry", 0.0).asDouble() * kDeg;
    p.rz = v.get("rz", 0.0).asDouble() * kDeg;
    return p;
}

bool read_line(int fd, std::string &out)
{
    out.clear();
    char c;
    while (true)
    {
        const ssize_t n = ::recv(fd, &c, 1, 0);
        if (n <= 0)
            return false;
        if (c == '\n')
            return true;
        if (c != '\r')
            out.push_back(c);
        if (out.size() > 1 << 20)
            return false;
    }
}

void write_json(int fd, const Json::Value &v)
{
    Json::StreamWriterBuilder b;
    b["indentation"] = "";
    const std::string s = Json::writeString(b, v) + "\n";
    ::send(fd, s.data(), s.size(), MSG_NOSIGNAL);
}

Json::Value ok_reply()
{
    Json::Value r(Json::objectValue);
    r["ok"] = true;
    return r;
}

Json::Value err_reply(const std::string &msg)
{
    Json::Value r(Json::objectValue);
    r["ok"] = false;
    r["error"] = msg;
    return r;
}

char parse_arm_force(const Json::Value &req)
{
    const std::string a = req.get("arm", "auto").asString();
    if (a == "left" || a == "l")
        return 'l';
    if (a == "right" || a == "r")
        return 'r';
    return 0;
}

orch::ArucoHoverExtras extras_from_json(const Json::Value &req)
{
    orch::ArucoHoverExtras e;
    e.include_target = req.get("include_target", true).asBool();
    e.marker_id = req.get("marker_id", 0).asInt();
    e.marker_id_b = req.get("marker_id_b", 0).asInt();
    e.dual_pos_thresh_m = req.get("dual_pos_thresh_m", 0.015).asDouble();
    e.dual_rot_thresh_deg = req.get("dual_rot_thresh_deg", 3.0).asDouble();
    if (req.isMember("dual_rel") && req["dual_rel"].isObject() && !req["dual_rel"].isNull())
    {
        e.dual_rel = pose_from_json(req["dual_rel"]);
        e.dual_rel_valid = true;
    }
    const Json::Value &wps = req["waypoints"];
    if (wps.isArray())
    {
        for (Json::ArrayIndex i = 0; i < wps.size(); ++i)
        {
            orch::RelWaypoint w;
            w.name = wps[i].get("name", "").asString();
            w.rel = pose_from_json(wps[i]);
            e.waypoints.push_back(std::move(w));
        }
    }
    return e;
}

void fill_hover_reply(Json::Value &r)
{
    r["arm"] = g_session.last_conveyor_r ? "right" : (g_session.last_conveyor_l ? "left" : "");
    r["holding_r"] = g_session.holding_r;
    r["holding_l"] = g_session.holding_l;
    r["head"] = g_session.last_aruco_head;
    r["status"] = g_session.last_aruco_status;
    r["marker_id"] = g_session.last_hover_marker_id;
    r["marker_id_b"] = g_session.last_hover_marker_id_b;
    r["dual_pos_m"] = g_session.last_dual_pos_m;
    r["dual_rot_deg"] = g_session.last_dual_rot_deg;
    r["dual_corrected"] = g_session.last_dual_corrected;
}

Json::Value markers_to_json()
{
    Json::Value marks(Json::arrayValue);
    for (const orch::ArucoHit &h : g_session.last_aruco_hits)
    {
        Json::Value m(Json::objectValue);
        m["slot"] = h.slot;
        m["id"] = h.id;
        m["side_mm"] = h.side_m * 1000.0;
        m["reproj_px"] = h.reproj_px;
        m["cam_x"] = h.cam_x;
        m["cam_y"] = h.cam_y;
        m["cam_z"] = h.cam_z;
        m["cam_yaw"] = h.cam_yaw;
        m["cam_pitch"] = h.cam_pitch;
        m["cam_roll"] = h.cam_roll;
        if (h.robot_ok)
        {
            m["robot_x"] = h.robot_x;
            m["robot_y"] = h.robot_y;
            m["robot_z"] = h.robot_z;
            m["robot_rx"] = h.robot_rx_deg;
            m["robot_ry"] = h.robot_ry_deg;
            m["robot_rz"] = h.robot_rz_deg;
        }
        marks.append(m);
    }
    return marks;
}

bool init_hardware(std::string &err)
{
    std::string cfg_err;
    if (!load_move_box_config(default_move_box_config_path(), g_move_cfg, cfg_err))
    {
        std::cerr << "[cfg] 加载失败，用默认: " << cfg_err << std::endl;
        g_move_cfg = default_move_box_config();
    }
    print_move_box_config(g_move_cfg);
    pose_vis_set_save_debug(kDebugVisualize);
    pose_vis_set_gui_enabled(false);
    std::cerr << "[orch_hw] 无窗口模式：调试图写入 picture_debug，不打开 OpenCV/Qt 窗\n";

    gripper::Config cfg;
    cfg.grasp_torque_limit_nm = 4.5;
    cfg.soft_torque_limit_nm = 4.0;
    cfg.pos_filter = 0.5;
    cfg.soft_slew_rate = 100.0;
    cfg.soft_coast_margin_rad = 0.12;
    try
    {
        g_grip = std::make_unique<gripper::Gripper>(cfg);
        if (!g_grip->start())
        {
            err = "夹爪启动失败";
            return false;
        }
    }
    catch (const std::exception &ex)
    {
        err = std::string("夹爪串口打开失败（检查 /dev/ttyUSB* 权限、是否在 dialout 组）: ") + ex.what();
        return false;
    }

    g_pipeline = std::make_unique<PosePipeline>();
    if (!g_pipeline->init(err))
        return false;

    init_socketcan();
    hand_socketid_bind();

    g_waist = std::make_unique<WaistRobot>();
    const std::string arm_yaml = project_root_dir() + "/config/Robot_Arm_Model.yaml";
    g_arm_r = std::make_unique<Robot_Arm>("T7", "T170", "right", arm_yaml);
    g_arm_l = std::make_unique<Robot_Arm>("T7", "T170", "left", arm_yaml);

    rev_motor_error(0);
    rev_motor_error(1);
    motor_speed_change();

    g_layer3 = g_move_cfg.waist.layer3_home;
    std::cerr << "[orch_hw] 头/腰/双臂启动姿态改由编排台「启动初始动作」下发，此处只完成硬件上电\n";

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

void sync_layer3_from_waist()
{
    if (!g_waist || !g_session.layer3)
        return;
    Eigen::MatrixXd tcp = g_waist->getTcpPos();
    if (tcp.size() < 6)
        return;
    for (int i = 0; i < 6; ++i)
        (*g_session.layer3)(i) = tcp(i);
}

Json::Value dispatch(const Json::Value &req)
{
    const std::string cmd = req.get("cmd", "").asString();
    if (cmd == "ping")
        return ok_reply();
    if (cmd == "abort")
    {
        g_abort.store(true);
        hardware_abort_request();
        std::cerr << "[orch_hw] STOP：中止当前动作并退出序列\n";
        return ok_reply();
    }
    if (cmd == "snapshot")
    {
        Json::Value r = ok_reply();
        r["tcp"]["right"] = pose_to_json(arm_get_tcp_pos(*g_arm_r));
        r["tcp"]["left"] = pose_to_json(arm_get_tcp_pos(*g_arm_l));
        r["links"]["right"] = links_to_json(arm_get_link_fk(*g_arm_r));
        r["links"]["left"] = links_to_json(arm_get_link_fk(*g_arm_l));
        if (g_waist)
        {
            Eigen::MatrixXd w = g_waist->getTcpPos();
            if (w.size() >= 6)
            {
                Eigen::Matrix<double, 1, 6> wp;
                for (int i = 0; i < 6; ++i)
                    wp(i) = w(i);
                r["waist_tcp"] = pose_to_json(wp);
            }
        }
        return r;
    }
    if (cmd == "reload_config")
    {
        std::lock_guard<std::mutex> lock(g_hw_mu);
        std::string cfg_err;
        if (!load_move_box_config(default_move_box_config_path(), g_move_cfg, cfg_err))
            return err_reply("重载配置失败: " + cfg_err);
        print_move_box_config(g_move_cfg);
        return ok_reply();
    }

    std::lock_guard<std::mutex> lock(g_hw_mu);
    g_abort.store(false);
    hardware_abort_clear();
    can_io_fault_clear();
    can_io_watch_begin();
    struct CanWatchEnd
    {
        ~CanWatchEnd() { can_io_watch_end(); }
    } can_watch_end;

    if (cmd == "arm")
    {
        const std::string side = req.get("side", "").asString();
        const double speed = req.get("speed", 0.2).asDouble();
        orch::PoseMrd p = pose_from_json(req["pose"]);
        Robot_Arm *arm = (side == "left") ? g_arm_l.get() : g_arm_r.get();
        const int rc = orch::move_arm_line(*arm, p, speed);
        if (rc == -4 || hardware_abort_requested())
            return err_reply("已中止");
        if (rc != 0)
            return err_reply("arm_line_move 失败 " + std::to_string(rc));
        return ok_reply();
    }
    if (cmd == "gripper")
    {
        const std::string side = req.get("side", "").asString();
        const std::string act = req.get("action", "").asString();
        auto s = (side == "left") ? gripper::Side::Left : gripper::Side::Right;
        if (act == "open")
            orch::gripper_open(*g_grip, s);
        else
            orch::gripper_close(*g_grip, s);
        return ok_reply();
    }
    if (cmd == "waist_rotate")
    {
        orch::waist_rotate_deg(req.get("angle_deg", 0).asDouble(), req.get("speed_deg_s", 30).asDouble());
        return ok_reply();
    }
    if (cmd == "waist_fold")
    {
        orch::waist_fold_deg(req.get("angle_deg", 0).asDouble(), req.get("speed_deg_s", 30).asDouble());
        return ok_reply();
    }
    if (cmd == "waist_delta")
    {
        orch::waist_delta(
            *g_waist,
            req.get("dx", 0).asDouble(),
            req.get("dz", 0).asDouble(),
            req.get("speed", 0.1).asDouble(),
            []() { return g_abort.load() || hardware_abort_requested() || can_io_faulted(); });
        sync_layer3_from_waist();
        if (hardware_abort_requested())
            return err_reply("已中止");
        return ok_reply();
    }
    if (cmd == "waist_goto")
    {
        orch::PoseMrd p = pose_from_json(req["pose"]);
        const int rc = orch::waist_goto(
            *g_waist, p, req.get("speed", 0.1).asDouble(),
            []() { return g_abort.load() || hardware_abort_requested() || can_io_faulted(); });
        sync_layer3_from_waist();
        if (rc != 0 || hardware_abort_requested())
            return err_reply(hardware_abort_requested() ? "已中止" : "腰坐标运动失败 " + std::to_string(rc));
        return ok_reply();
    }
    if (cmd == "head")
    {
        const double m0 = req.get("m0", 0).asDouble();
        const double m1 = req.get("m1", 40).asDouble();
        const double m2 = req.get("m2", 0).asDouble();
        int head_p[3] = {
            static_cast<int>(m0 * 65536 * 4 / 360),
            static_cast<int>(m1 * 65536 * 4 / 360),
            static_cast<int>(m2 * 65536 * 4 / 360),
        };
        uint32_t head_can[] = {30, 31, 32};
        socketcan_sendcommand(head_id, 3, head_can, 30, head_p);
        return ok_reply();
    }
    if (cmd == "home")
    {
        orch::go_home(*g_arm_r, *g_arm_l);
        if (hardware_abort_requested())
            return err_reply("已中止");
        return ok_reply();
    }
    if (cmd == "vision_grasp")
    {
        const int rc = orch::vision_grasp_existing_pipeline();
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["grasped_r"] = g_session.last_grasped_r;
        r["grasped_l"] = g_session.last_grasped_l;
        r["status"] = g_session.last_grasp_status;
        if (rc < 0)
        {
            r["ok"] = false;
            r["error"] = (rc == -4) ? "已中止" : "视觉抓取失败";
        }
        return r;
    }
    if (cmd == "vision_grasp_factory")
    {
        const int rc = orch::vision_grasp_factory_pipeline();
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["grasped_r"] = g_session.last_grasped_r;
        r["grasped_l"] = g_session.last_grasped_l;
        r["status"] = g_session.last_grasp_status;
        if (rc < 0)
        {
            r["ok"] = false;
            if (rc == -4)
                r["error"] = "已中止";
            else if (rc == -5)
                r["error"] = "工厂检测模型未加载";
            else
                r["error"] = "视觉抓取失败";
        }
        return r;
    }
    if (cmd == "vision_grasp_jindi")
    {
        const int rc = orch::vision_grasp_jindi_pipeline();
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["grasped_r"] = g_session.last_grasped_r;
        r["grasped_l"] = g_session.last_grasped_l;
        r["status"] = g_session.last_grasp_status;
        if (rc < 0)
        {
            r["ok"] = false;
            if (rc == -4)
                r["error"] = "已中止";
            else if (rc == -5)
                r["error"] = "金帝四类模型未加载";
            else
                r["error"] = "视觉抓取失败";
        }
        return r;
    }
    if (cmd == "vision_detect")
    {
        const int rc = orch::vision_detect_factory_pipeline();
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["head"] = g_session.last_detect_head;
        r["right"] = g_session.last_detect_right;
        r["left"] = g_session.last_detect_left;
        r["status"] = g_session.last_detect_status;
        if (rc < 0)
        {
            r["ok"] = false;
            if (rc == -4)
                r["error"] = "已中止";
            else if (rc == -5)
                r["error"] = "工厂检测模型未加载";
            else
                r["error"] = "视觉检测失败";
        }
        return r;
    }
    if (cmd == "vision_detect_jindi")
    {
        const int rc = orch::vision_detect_jindi_pipeline();
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["head"] = g_session.last_detect_head;
        r["right"] = g_session.last_detect_right;
        r["left"] = g_session.last_detect_left;
        r["status"] = g_session.last_detect_status;
        r["detail"] = g_session.last_detect_detail;
        r["blank"] = g_session.last_detect_cls[0];
        r["semi"] = g_session.last_detect_cls[1];
        r["finished"] = g_session.last_detect_cls[2];
        r["hole"] = g_session.last_detect_cls[3];
        if (rc < 0)
        {
            r["ok"] = false;
            if (rc == -4)
                r["error"] = "已中止";
            else if (rc == -5)
                r["error"] = "金帝四类模型未加载";
            else
                r["error"] = "视觉检测失败";
        }
        return r;
    }
    if (cmd == "aruco_detect")
    {
        const int rc = orch::vision_aruco_detect_pipeline();
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["head"] = g_session.last_aruco_head;
        r["right"] = g_session.last_aruco_right;
        r["left"] = g_session.last_aruco_left;
        r["status"] = g_session.last_aruco_status;
        r["markers"] = markers_to_json();
        if (rc < 0)
        {
            r["ok"] = false;
            if (rc == -4)
                r["error"] = "已中止";
            else if (rc == -5)
                r["error"] = "ArUco 引擎未加载";
            else
                r["error"] = "二维码位姿失败";
        }
        return r;
    }
    if (cmd == "conveyor_goto")
    {
        const std::string side = req.get("side", "right").asString();
        const std::string hand = req.get("hand", side == "left" ? "empty" : "holding").asString();
        const bool is_right = (side != "left");
        const bool want_holding = (hand != "empty");
        const char arm_force = parse_arm_force(req);
        const orch::ArucoHoverExtras extras = extras_from_json(req);
        const int rc = orch::conveyor_goto_pipeline(is_right, want_holding, arm_force, &extras);
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["side"] = is_right ? "right" : "left";
        r["hand"] = want_holding ? "holding" : "empty";
        fill_hover_reply(r);
        if (rc != 0)
        {
            r["ok"] = false;
            if (rc == -4)
                r["error"] = "已中止";
            else if (rc == -5)
                r["error"] = "ArUco 引擎未加载";
            else if (rc == 1)
                r["error"] = "头相机未看到二维码";
            else if (rc == -3)
                r["error"] = "传送带上方直线失败";
            else if (rc == -6)
                r["error"] = want_holding ? "没有持料手，请先抓取" : "没有可取料的空手";
            else if (rc == -7)
                r["error"] = "两只手都空，无法自动选哪只去取料";
            else
                r["error"] = "传送带定位失败";
        }
        return r;
    }
    if (cmd == "aruco_above")
    {
        const std::string hand = req.get("hand", "holding").asString();
        const bool want_holding = (hand != "empty");
        const char arm_force = parse_arm_force(req);
        const orch::ArucoHoverExtras extras = extras_from_json(req);
        const int rc = orch::aruco_above_pipeline(want_holding, arm_force, &extras);
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["hand"] = want_holding ? "holding" : "empty";
        fill_hover_reply(r);
        if (rc != 0)
        {
            r["ok"] = false;
            if (rc == -4)
                r["error"] = "已中止";
            else if (rc == -5)
                r["error"] = "ArUco 引擎未加载";
            else if (rc == 1)
                r["error"] = "头相机未看到二维码";
            else if (rc == -3)
                r["error"] = "二维码上方直线失败";
            else if (rc == -6)
                r["error"] = want_holding ? "没有持料手，请先抓取" : "没有可移动的空手";
            else
                r["error"] = "二维码上方定位失败";
        }
        return r;
    }
    if (cmd == "path_record")
    {
        const char arm_side = parse_arm_force(req);
        const int marker_id = req.get("marker_id", 0).asInt();
        const int marker_id_b = req.get("marker_id_b", 0).asInt();
        orch::PathRecordResult rec;
        const int rc = orch::path_record_pipeline(arm_side == 'l' ? 'l' : 'r', marker_id, marker_id_b, rec);
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["status"] = rec.status;
        r["marker_id"] = rec.marker_id;
        r["marker_id_b"] = rec.marker_id_b;
        r["have_b"] = rec.have_b;
        r["tcp"] = pose_to_json(orch::to_row(rec.tcp));
        r["rel"] = pose_to_json(orch::to_row(rec.rel));
        r["marker"] = pose_to_json(orch::to_row(rec.marker));
        if (rec.have_b)
        {
            r["marker_b"] = pose_to_json(orch::to_row(rec.marker_b));
            r["dual_rel"] = pose_to_json(orch::to_row(rec.dual_rel));
        }
        r["markers"] = markers_to_json();
        r["head"] = g_session.last_aruco_head;
        if (rc != 0)
        {
            r["ok"] = false;
            if (rc == -4)
                r["error"] = "已中止";
            else if (rc == -5)
                r["error"] = "ArUco 引擎未加载";
            else if (rc == 1)
                r["error"] = "头相机未看到二维码";
            else
                r["error"] = "示教记录失败";
        }
        return r;
    }
    if (cmd == "hand_grasp")
    {
        const int rc = orch::vision_hand_grasp_pipeline();
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["grasped_r"] = g_session.last_grasped_r;
        r["grasped_l"] = g_session.last_grasped_l;
        r["status"] = g_session.last_grasp_status;
        if (rc < 0)
        {
            r["ok"] = false;
            r["error"] = (rc == -4) ? "已中止" : "手相机抓取失败";
        }
        else if (rc != 0)
        {
            r["ok"] = false;
            r["error"] = (rc == 1) ? "手相机未识别到目标" : "手相机未抓住";
        }
        return r;
    }
    if (cmd == "vision_place")
    {
        const int rc = orch::vision_place_pipeline();
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["holes"] = g_session.last_place_holes;
        r["released_r"] = g_session.last_released_r;
        r["released_l"] = g_session.last_released_l;
        r["holding_r"] = g_session.holding_r;
        r["holding_l"] = g_session.holding_l;
        r["status"] = g_session.last_place_status;
        if (rc != 0)
        {
            r["ok"] = false;
            if (rc == -4)
                r["error"] = "已中止";
            else if (rc == -3)
                r["error"] = "读相机外参失败";
            else if (rc == 1)
                r["error"] = "未检测到料盘空位或没有持料手";
            else if (rc == 2)
                r["error"] = "放货运动失败";
            else
                r["error"] = "视觉放货失败";
        }
        return r;
    }
    if (cmd == "hand_grasp_factory")
    {
        const int rc = orch::vision_hand_grasp_factory_pipeline();
        Json::Value r = ok_reply();
        r["code"] = rc;
        r["grasped_r"] = g_session.last_grasped_r;
        r["grasped_l"] = g_session.last_grasped_l;
        r["status"] = g_session.last_grasp_status;
        if (rc < 0)
        {
            r["ok"] = false;
            if (rc == -4)
                r["error"] = "已中止";
            else if (rc == -5)
                r["error"] = "工厂检测模型未加载";
            else
                r["error"] = "手相机抓取失败";
        }
        else if (rc != 0)
        {
            r["ok"] = false;
            r["error"] = (rc == 1) ? "手相机未识别到目标" : "手相机未抓住";
        }
        return r;
    }
    if (cmd == "chassis")
        return err_reply("底盘已禁用");
    return err_reply("未知命令 " + cmd);
}

void handle_client(int fd)
{
    std::string line;
    if (!read_line(fd, line))
    {
        ::close(fd);
        return;
    }
    Json::Value req;
    JSONCPP_STRING perr;
    Json::CharReaderBuilder rb;
    std::unique_ptr<Json::CharReader> reader(rb.newCharReader());
    if (!reader->parse(line.data(), line.data() + line.size(), &req, &perr))
    {
        write_json(fd, err_reply("JSON 解析失败"));
        ::close(fd);
        return;
    }
    try
    {
        const std::string cmd = req.get("cmd", "").asString();
        Json::Value reply = dispatch(req);
        if (cmd != "ping" && cmd != "abort" && cmd != "snapshot")
        {
            if (can_io_faulted())
            {
                const std::string msg = can_io_fault_message();
                std::cerr << "[orch_hw] " << msg << "，中止当前动作\n";
                g_abort.store(true);
                hardware_abort_request();
                reply = err_reply(msg);
            }
            else if (g_abort.load() || hardware_abort_requested())
            {
                std::cerr << "[orch_hw] 已中止，退出当前动作\n";
                reply = err_reply("已中止");
            }
        }
        write_json(fd, reply);
    }
    catch (const std::exception &ex)
    {
        write_json(fd, err_reply(ex.what()));
    }
    ::close(fd);
}

} // namespace

int main(int argc, char **argv)
{
    ::dup2(STDERR_FILENO, STDOUT_FILENO);
    int port = 8099;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--port" && i + 1 < argc)
            port = std::atoi(argv[++i]);
    }

    install_app_sigint_handler();
    std::string err;
    if (!init_hardware(err))
    {
        std::cerr << "[orch_hw] 初始化失败: " << err << std::endl;
        return 1;
    }

    const int srv = ::socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    if (bind(srv, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
    {
        std::cerr << "[orch_hw] bind 失败\n";
        return 1;
    }
    listen(srv, 8);
    std::cerr << "ORCH_HW_READY port=" << port << " chassis=disabled" << std::endl;

    // 解释器在主线程 init，命令在 detach 线程执行。主线程若不放下 GIL，
    // 工作线程拿不到锁，调用 pybind11 会 PyGILState_Check() failure。
    pybind11::gil_scoped_release gil_release;
    while (!app_stop_requested())
    {
        sockaddr_in cli{};
        socklen_t n = sizeof(cli);
        const int fd = accept(srv, reinterpret_cast<sockaddr *>(&cli), &n);
        if (fd < 0)
            continue;
        std::thread(handle_client, fd).detach();
    }
    return 0;
}
