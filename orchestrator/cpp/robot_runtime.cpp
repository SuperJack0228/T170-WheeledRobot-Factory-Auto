#include "robot_runtime.h"

#include "function.h"
#include "move_box_runtime.h"
#include "reals_tcp.h"
#include "seg_pose_bridge.h"
#include "Ti5_socketcan.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>

namespace orch
{

HwSession *g_hw = nullptr;

int move_arm_line(Robot_Arm &arm, const PoseMrd &goal, double vel_mps)
{
    Eigen::Matrix<double, 1, 6> pos = to_row(goal);
    return arm_line_move(arm, pos, vel_mps);
}

ArmLineMoveResult move_arms_line(
    Robot_Arm &arm_r,
    const PoseMrd *goal_r,
    Robot_Arm &arm_l,
    const PoseMrd *goal_l,
    double vel_mps)
{
    Eigen::Matrix<double, 1, 6> pr, pl;
    if (goal_r)
        pr = to_row(*goal_r);
    if (goal_l)
        pl = to_row(*goal_l);
    return arm_dual_line_move_selective(
        arm_r, pr, goal_r != nullptr, arm_l, pl, goal_l != nullptr, vel_mps);
}

int gripper_open(gripper::Gripper &g, gripper::Side side)
{
    g.openAndWait(side);
    if (g_hw != nullptr)
    {
        if (side == gripper::Side::Right)
            g_hw->holding_r = false;
        else
            g_hw->holding_l = false;
    }
    return 0;
}

int gripper_close(gripper::Gripper &g, gripper::Side side)
{
    g.graspAndWait(side);
    return 0;
}

int waist_rotate_deg(double yaw_deg, double speed_deg_s)
{
    double angles[1] = {yaw_deg};
    uint32_t can_id[] = {1};
    return smooth_motor_move_deg(waist_id, 1, can_id, angles, speed_deg_s);
}

int waist_fold_deg(double pitch_deg, double speed_deg_s)
{
    double angles[1] = {pitch_deg};
    uint32_t can_id[] = {2};
    return smooth_motor_move_deg(waist_id, 1, can_id, angles, speed_deg_s);
}

int waist_delta(WaistRobot &waist, double dx, double dz, double vel_mps, const AbortFn &abort)
{
    Eigen::MatrixXd tcp = waist.getTcpPos();
    Eigen::Matrix<double, 1, 6> pos = Eigen::Matrix<double, 1, 6>::Zero();
    if (tcp.size() >= 6)
    {
        for (int i = 0; i < 6; ++i)
            pos(i) = tcp(i);
    }
    // 与 stagger 一致：用户「前进」为正，软件 pos(0) 减小
    pos(0) -= dx;
    pos(2) += dz;
    if (move_box::clamp_waist_layer3_xyz(pos))
        std::cout << "[orch] 腰行程限位 x=[" << g_move_cfg.waist.x_min << ","
                  << g_move_cfg.waist.x_max << "] z=[" << g_move_cfg.waist.z_min << ","
                  << g_move_cfg.waist.z_max << "] → x=" << pos(0) << " z=" << pos(2) << "\n";
    if (abort)
        return waist.moveLToPos(pos, vel_mps, abort);
    return waist.moveLToPos(pos, vel_mps);
}

int waist_goto(WaistRobot &waist, const PoseMrd &goal, double vel_mps, const AbortFn &abort)
{
    Eigen::Matrix<double, 1, 6> pos = to_row(goal);
    if (move_box::clamp_waist_layer3_xyz(pos))
        std::cout << "[orch] 腰 goto 行程限位 → x=" << pos(0) << " z=" << pos(2) << "\n";
    if (abort)
        return waist.moveLToPos(pos, vel_mps, abort);
    return waist.moveLToPos(pos, vel_mps);
}

int go_home(Robot_Arm &arm_r, Robot_Arm &arm_l)
{
    std::cout << "[orch] 归位：先抬手回 standby，再腰 yaw=0 + layer3_home（避免手在料盘上方时转腰）\n";
    move_box::move_arms_to_standby(arm_r, arm_l);
    if (hardware_abort_requested())
        return -4;
    waist_rotate_deg(0.0, 30.0);
    if (hardware_abort_requested())
        return -4;
    if (g_hw != nullptr && g_hw->waist != nullptr && g_hw->layer3 != nullptr)
    {
        *g_hw->layer3 = g_move_cfg.waist.layer3_home;
        PoseMrd p;
        p.x = (*g_hw->layer3)(0);
        p.y = (*g_hw->layer3)(1);
        p.z = (*g_hw->layer3)(2);
        p.rx = (*g_hw->layer3)(3);
        p.ry = (*g_hw->layer3)(4);
        p.rz = (*g_hw->layer3)(5);
        waist_goto(*g_hw->waist, p, 0.1, nullptr);
    }
    return 0;
}

namespace
{

int vision_grasp_with_engine(SegEngineId engine_id)
{
    if (g_hw == nullptr || g_hw->g == nullptr || g_hw->pipeline == nullptr)
        return -1;

    std::array<double, 16> cam2robot{};
    std::string err;
    if (!load_cam2robot_matrix(default_camera_to_robot_yaml_path(), cam2robot, err))
    {
        std::cerr << "[orch] 读 cam2robot 失败: " << err << std::endl;
        return -3;
    }

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };
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
        engine_id);

    g_hw->last_grasped_r = r.grasped_r;
    g_hw->last_grasped_l = r.grasped_l;
    g_hw->last_grasp_status = static_cast<int>(r.status);
    g_hw->holding_r = r.grasped_r;
    g_hw->holding_l = r.grasped_l;

    if (r.status == move_box::GraspToStandbyStatus::Aborted)
        return -4;
    if (r.status == move_box::GraspToStandbyStatus::Ok)
        return 0;
    if (r.status == move_box::GraspToStandbyStatus::NoTarget)
        return 1;
    return 2;
}

} // namespace

int vision_grasp_existing_pipeline()
{
    return vision_grasp_with_engine(SegEngineId::Default);
}

int vision_grasp_factory_pipeline()
{
    if (g_hw == nullptr || g_hw->pipeline == nullptr)
        return -1;
    if (!g_hw->pipeline->bridge().factory_engine_ready())
    {
        std::cerr << "[orch] 工厂检测引擎未加载（检查工程根 best.pt）\n";
        return -5;
    }
    std::cout << "[orch] 工厂单类视觉抓取（根目录 best.pt）\n";
    return vision_grasp_with_engine(SegEngineId::Factory);
}

int vision_grasp_jindi_pipeline()
{
    if (g_hw == nullptr || g_hw->pipeline == nullptr)
        return -1;
    if (!g_hw->pipeline->bridge().jindi_engine_ready())
    {
        std::cerr << "[orch] 金帝四类引擎未加载（检查 seg_model/best.pt）\n";
        return -5;
    }
    std::cout << "[orch] 金帝四类视觉抓取（只抓毛胚 class0）\n";
    return vision_grasp_with_engine(SegEngineId::Jindi);
}

int vision_detect_factory_pipeline()
{
    if (g_hw == nullptr || g_hw->pipeline == nullptr)
        return -1;

    g_hw->last_detect_head = 0;
    g_hw->last_detect_right = 0;
    g_hw->last_detect_left = 0;
    g_hw->last_detect_status = 1;

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };
    if (should_abort())
        return -4;

    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.factory_engine_ready())
    {
        std::cerr << "[orch] 工厂检测引擎未加载（检查工程根 best.pt）\n";
        return -5;
    }

    RealSenseMultiCam &cameras = g_hw->pipeline->cameras();
    std::cout << "[orch] 工厂视觉检测：单类模型，三相机拍图（不抓取）\n";

    if (kDebugVisualize)
        pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});
    PoseDetectionRecords head = detect_pose_at_slot(
        cameras, bridge, CameraSlot::Head, kDebugVisualize, kGraspDetectClassId, SegEngineId::Factory);
    if (should_abort())
        return -4;
    print_pose_records_summary("工厂检测 头相机", head);
    g_hw->last_detect_head = static_cast<int>(head.size());

    if (kDebugVisualize)
        pose_vis_begin_phase(
            PoseVisLayout::DualHand, PoseVisPanel::RightHand, {PoseVisPanel::RightHand, PoseVisPanel::LeftHand});
    PoseDetectionRecords right = detect_pose_at_slot(
        cameras, bridge, CameraSlot::RightHand, kDebugVisualize, kGraspDetectClassId, SegEngineId::Factory);
    if (should_abort())
        return -4;
    print_pose_records_summary("工厂检测 右手", right);
    g_hw->last_detect_right = static_cast<int>(right.size());

    PoseDetectionRecords left = detect_pose_at_slot(
        cameras, bridge, CameraSlot::LeftHand, kDebugVisualize, kGraspDetectClassId, SegEngineId::Factory);
    if (should_abort())
        return -4;
    print_pose_records_summary("工厂检测 左手", left);
    g_hw->last_detect_left = static_cast<int>(left.size());

    const int total = g_hw->last_detect_head + g_hw->last_detect_right + g_hw->last_detect_left;
    std::cout << "[orch] 工厂检测完成 头=" << g_hw->last_detect_head
              << " 右=" << g_hw->last_detect_right
              << " 左=" << g_hw->last_detect_left << "\n";
    g_hw->last_detect_status = (total > 0) ? 0 : 1;
    return g_hw->last_detect_status;
}

namespace
{

const char *jindi_class_cn(int class_id)
{
    switch (class_id)
    {
    case kJindiBlankClassId:
        return "毛胚";
    case kJindiSemiClassId:
        return "半加工";
    case kJindiFinishedClassId:
        return "加工";
    case kJindiHoleClassId:
        return "料盘孔";
    default:
        return "?";
    }
}

void add_jindi_class_counts(const PoseDetectionRecords &recs, int out[4])
{
    for (const auto &rec : recs)
    {
        const int id = rec.target.class_id;
        if (id >= 0 && id < 4)
            ++out[id];
    }
}

std::string format_jindi_counts(const int c[4])
{
    std::string s;
    for (int i = 0; i < 4; ++i)
    {
        if (i)
            s += " ";
        s += jindi_class_cn(i);
        s += "=";
        s += std::to_string(c[i]);
    }
    return s;
}

} // namespace

int vision_detect_jindi_pipeline()
{
    if (g_hw == nullptr || g_hw->pipeline == nullptr)
        return -1;

    g_hw->last_detect_head = 0;
    g_hw->last_detect_right = 0;
    g_hw->last_detect_left = 0;
    g_hw->last_detect_status = 1;
    g_hw->last_detect_detail.clear();
    for (int &n : g_hw->last_detect_cls)
        n = 0;

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };
    if (should_abort())
        return -4;

    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.jindi_engine_ready())
    {
        std::cerr << "[orch] 金帝四类引擎未加载（检查 seg_model/best.pt）\n";
        return -5;
    }

    RealSenseMultiCam &cameras = g_hw->pipeline->cameras();
    std::cout << "[orch] 金帝四类视觉检测：毛胚/半加工/加工/料盘孔（不抓取）\n";

    if (kDebugVisualize)
        pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});
    PoseDetectionRecords head = detect_pose_at_slot(
        cameras, bridge, CameraSlot::Head, kDebugVisualize, -1, SegEngineId::Jindi);
    if (should_abort())
        return -4;
    print_pose_records_summary("金帝检测 头相机", head);
    g_hw->last_detect_head = static_cast<int>(head.size());
    add_jindi_class_counts(head, g_hw->last_detect_cls);

    if (kDebugVisualize)
        pose_vis_begin_phase(
            PoseVisLayout::DualHand, PoseVisPanel::RightHand, {PoseVisPanel::RightHand, PoseVisPanel::LeftHand});
    PoseDetectionRecords right = detect_pose_at_slot(
        cameras, bridge, CameraSlot::RightHand, kDebugVisualize, -1, SegEngineId::Jindi);
    if (should_abort())
        return -4;
    print_pose_records_summary("金帝检测 右手", right);
    g_hw->last_detect_right = static_cast<int>(right.size());
    add_jindi_class_counts(right, g_hw->last_detect_cls);

    PoseDetectionRecords left = detect_pose_at_slot(
        cameras, bridge, CameraSlot::LeftHand, kDebugVisualize, -1, SegEngineId::Jindi);
    if (should_abort())
        return -4;
    print_pose_records_summary("金帝检测 左手", left);
    g_hw->last_detect_left = static_cast<int>(left.size());
    add_jindi_class_counts(left, g_hw->last_detect_cls);

    g_hw->last_detect_detail = format_jindi_counts(g_hw->last_detect_cls);
    const int total = g_hw->last_detect_head + g_hw->last_detect_right + g_hw->last_detect_left;
    std::cout << "[orch] 金帝检测完成 头=" << g_hw->last_detect_head
              << " 右=" << g_hw->last_detect_right
              << " 左=" << g_hw->last_detect_left
              << " " << g_hw->last_detect_detail << "\n";
    g_hw->last_detect_status = (total > 0) ? 0 : 1;
    return g_hw->last_detect_status;
}

namespace
{

std::string cam2robot_yaml_for_slot(CameraSlot slot)
{
    switch (slot)
    {
    case CameraSlot::RightHand:
        return default_right_hand_to_robot_yaml_path();
    case CameraSlot::LeftHand:
        return default_left_hand_to_robot_yaml_path();
    default:
        return default_camera_to_robot_yaml_path();
    }
}

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

int detect_aruco_one_slot(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    CameraSlot slot,
    const char *title,
    int *out_count)
{
    const char *slot_label = RealSenseMultiCam::slot_name(slot);
    CameraFrameData frame;
    if (slot == CameraSlot::Head)
        frame = cameras.grab(slot);
    else
    {
        cameras.flush(slot);
        frame = cameras.grab_fresh(slot);
    }
    if (!frame.ok)
    {
        std::cerr << "[aruco] " << slot_label << " 取帧失败: " << frame.message << "\n";
        *out_count = 0;
        return 0;
    }
    frame = RealSenseMultiCam::prepare_frame_for_slot(std::move(frame), slot);

    const ArucoDetectResult det = bridge.run_aruco(frame, kDebugVisualize, slot);
    if (!det.ok)
    {
        std::cerr << "[aruco] " << slot_label << " 失败: " << det.message << "\n";
        *out_count = 0;
        return -2;
    }

    std::array<double, 16> cam2robot{};
    std::string err;
    const bool have_ext = load_cam2robot_matrix(cam2robot_yaml_for_slot(slot), cam2robot, err);
    if (!have_ext)
        std::cerr << "[aruco] " << slot_label << " 无 cam2robot: " << err << "\n";

    std::cout << "\n--- " << title << " (" << det.markers.size() << " 个码) ---\n";
    if (det.markers.empty())
        std::cout << "(无)\n";
    for (const ArucoMarkerResult &m : det.markers)
    {
        ArucoHit hit = make_aruco_hit(slot_label, m, have_ext ? &cam2robot : nullptr);
        print_aruco_hit(hit);
        g_hw->last_aruco_hits.push_back(std::move(hit));
    }
    *out_count = static_cast<int>(det.markers.size());
    return 0;
}

} // namespace

int vision_aruco_detect_pipeline()
{
    if (g_hw == nullptr || g_hw->pipeline == nullptr)
        return -1;

    g_hw->last_aruco_head = 0;
    g_hw->last_aruco_right = 0;
    g_hw->last_aruco_left = 0;
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
    int rc = detect_aruco_one_slot(
        cameras, bridge, CameraSlot::Head, "ArUco 头相机", &g_hw->last_aruco_head);
    if (rc < 0)
        return rc;
    if (should_abort())
        return -4;

    if (kDebugVisualize)
        pose_vis_begin_phase(
            PoseVisLayout::DualHand, PoseVisPanel::RightHand, {PoseVisPanel::RightHand, PoseVisPanel::LeftHand});
    rc = detect_aruco_one_slot(
        cameras, bridge, CameraSlot::RightHand, "ArUco 右手", &g_hw->last_aruco_right);
    if (rc < 0)
        return rc;
    if (should_abort())
        return -4;

    rc = detect_aruco_one_slot(
        cameras, bridge, CameraSlot::LeftHand, "ArUco 左手", &g_hw->last_aruco_left);
    if (rc < 0)
        return rc;
    if (should_abort())
        return -4;

    const int total = g_hw->last_aruco_head + g_hw->last_aruco_right + g_hw->last_aruco_left;
    std::cout << "[orch] ArUco 完成 头=" << g_hw->last_aruco_head
              << " 右=" << g_hw->last_aruco_right
              << " 左=" << g_hw->last_aruco_left << "\n";
    g_hw->last_aruco_status = (total > 0) ? 0 : 1;
    return g_hw->last_aruco_status;
}

namespace
{

enum class ArucoHover
{
    Right,
    Left,
    Above,
};

PoseMrd row_to_mrd(const Eigen::Matrix<double, 1, 6> &row)
{
    PoseMrd p;
    p.x = row(0);
    p.y = row(1);
    p.z = row(2);
    p.rx = row(3);
    p.ry = row(4);
    p.rz = row(5);
    return p;
}

Eigen::Matrix<double, 1, 6> mrd_to_row(const PoseMrd &p)
{
    Eigen::Matrix<double, 1, 6> row;
    row << p.x, p.y, p.z, p.rx, p.ry, p.rz;
    return row;
}

Eigen::Matrix4d row_to_T(const Eigen::Matrix<double, 1, 6> &row)
{
    const Eigen::MatrixXd M = TR(row);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (M.rows() >= 4 && M.cols() >= 4)
        T = M.block<4, 4>(0, 0);
    return T;
}

void se3_error(const Eigen::Matrix4d &a, const Eigen::Matrix4d &b, double &pos_m, double &rot_deg)
{
    pos_m = (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
    const Eigen::Matrix3d R = a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
    const double c = std::clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
    rot_deg = std::acos(c) * 180.0 / M_PI;
}

Eigen::Matrix4d average_se3(const Eigen::Matrix4d &a, const Eigen::Matrix4d &b)
{
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m.block<3, 1>(0, 3) = 0.5 * (a.block<3, 1>(0, 3) + b.block<3, 1>(0, 3));
    const Eigen::Quaterniond qa(a.block<3, 3>(0, 0));
    const Eigen::Quaterniond qb(b.block<3, 3>(0, 0));
    m.block<3, 3>(0, 0) = qa.slerp(0.5, qb).normalized().toRotationMatrix();
    return m;
}

const ArucoHit *pick_robot_hit(int want_id)
{
    const ArucoHit *any = nullptr;
    const ArucoHit *pref = nullptr;
    for (const ArucoHit &h : g_hw->last_aruco_hits)
    {
        if (!h.robot_ok)
            continue;
        if (any == nullptr)
            any = &h;
        if (want_id > 0 && h.id == want_id)
        {
            pref = &h;
            break;
        }
    }
    return pref != nullptr ? pref : any;
}

const ArucoHit *pick_second_hit(const ArucoHit *primary, int want_id)
{
    if (want_id > 0)
    {
        const ArucoHit *h = pick_robot_hit(want_id);
        if (h != nullptr && h != primary)
            return h;
        return nullptr;
    }
    for (const ArucoHit &h : g_hw->last_aruco_hits)
    {
        if (h.robot_ok && &h != primary)
            return &h;
    }
    return nullptr;
}

Eigen::Matrix4d maybe_correct_marker_frame(const ArucoHit &hit_a, const ArucoHoverExtras &ex)
{
    Eigen::Matrix4d T_a = pose16_to_T(hit_a.pose_robot_4x4);
    g_hw->last_hover_marker_id = hit_a.id;
    g_hw->last_hover_marker_id_b = -1;
    g_hw->last_dual_pos_m = -1.0;
    g_hw->last_dual_rot_deg = -1.0;
    g_hw->last_dual_corrected = false;

    if (ex.marker_id_b <= 0 && !ex.dual_rel_valid)
        return T_a;

    const ArucoHit *hit_b = pick_second_hit(&hit_a, ex.marker_id_b);
    if (hit_b == nullptr || hit_b == &hit_a)
    {
        std::cout << "[orch] 双码：未见副码，只用主码 ID=" << hit_a.id << "\n";
        return T_a;
    }
    g_hw->last_hover_marker_id_b = hit_b->id;
    const Eigen::Matrix4d T_b = pose16_to_T(hit_b->pose_robot_4x4);
    const Eigen::Matrix4d T_ab_live = T_a.inverse() * T_b;
    if (!ex.dual_rel_valid)
    {
        const Eigen::Matrix<double, 1, 6> live = T2PosEulerAngles(T_ab_live);
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] 双码 ID " << hit_a.id << "/" << hit_b->id
                  << " 现场相对 xyz=(" << live(0) << "," << live(1) << "," << live(2)
                  << ") 尚未示教 dual_rel，不做修正\n"
                  << std::defaultfloat << std::setprecision(6);
        return T_a;
    }

    const Eigen::Matrix4d T_ab_taught = row_to_T(mrd_to_row(ex.dual_rel));
    const Eigen::Matrix4d T_b_from_a = T_a * T_ab_taught;
    double pos_m = 0.0;
    double rot_deg = 0.0;
    se3_error(T_b_from_a, T_b, pos_m, rot_deg);
    g_hw->last_dual_pos_m = pos_m;
    g_hw->last_dual_rot_deg = rot_deg;
    const bool small = (pos_m <= ex.dual_pos_thresh_m) && (rot_deg <= ex.dual_rot_thresh_deg);
    std::cout << std::fixed << std::setprecision(4)
              << "[orch] 双码 ID " << hit_a.id << "/" << hit_b->id
              << " 误差 pos=" << pos_m << " m rot=" << rot_deg << " deg"
              << " 阈值 " << ex.dual_pos_thresh_m << " m / " << ex.dual_rot_thresh_deg << " deg"
              << (small ? " → 低于阈值，不修正\n" : " → 超阈值，用双码平均修正主码\n")
              << std::defaultfloat << std::setprecision(6);
    if (small)
        return T_a;
    const Eigen::Matrix4d T_a_from_b = T_b * T_ab_taught.inverse();
    g_hw->last_dual_corrected = true;
    return average_se3(T_a, T_a_from_b);
}

int move_rel_waypoints(
    Robot_Arm &arm,
    const Eigen::Matrix4d &T_marker,
    const ArucoHoverExtras &ex,
    double speed,
    const std::function<bool()> &should_abort)
{
    for (size_t i = 0; i < ex.waypoints.size(); ++i)
    {
        if (should_abort())
            return -4;
        const RelWaypoint &wp = ex.waypoints[i];
        const Eigen::Matrix4d T_tcp = T_marker * row_to_T(mrd_to_row(wp.rel));
        Eigen::Matrix<double, 1, 6> goal = T2PosEulerAngles(T_tcp);
        const double z_before = goal(2);
        const std::string stage = wp.name.empty()
                                      ? ("相对路径点 " + std::to_string(i + 1))
                                      : ("相对路径 " + wp.name);
        clamp_cart_goal_z(goal, stage.c_str());
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] 路径点[" << (i + 1) << "/" << ex.waypoints.size() << "] "
                  << (wp.name.empty() ? "(未命名)" : wp.name)
                  << " xyz=(" << goal(0) << "," << goal(1) << "," << goal(2)
                  << ") rpy_deg=(" << (goal(3) * 180.0 / M_PI) << ","
                  << (goal(4) * 180.0 / M_PI) << "," << (goal(5) * 180.0 / M_PI) << ")\n"
                  << std::defaultfloat << std::setprecision(6);
        if (std::abs(goal(2) - z_before) > 1e-9)
            std::cerr << "[orch][WARN] 路径点 z 被 clamp\n";
        ArmLineMoveDebugStage dbg(stage.c_str());
        const int rc = arm_line_move(arm, goal, speed);
        if (rc != 0)
        {
            std::cerr << "[orch] 相对路径点失败 rc=" << rc << " " << stage << "\n";
            return -3;
        }
    }
    return 0;
}

int aruco_hover_pipeline(
    ArucoHover hover, bool want_holding, char arm_force, const ArucoHoverExtras *extras)
{
    if (g_hw == nullptr || g_hw->pipeline == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
        return -1;

    const ArucoHoverExtras empty;
    const ArucoHoverExtras &ex = extras != nullptr ? *extras : empty;

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };
    if (should_abort())
        return -4;

    const bool above = (hover == ArucoHover::Above);
    const bool geo_right = (hover != ArucoHover::Left);
    bool move_r = false;
    bool move_l = false;
    if (arm_force == 'r')
        move_r = true;
    else if (arm_force == 'l')
        move_l = true;
    else
    {
        move_r = want_holding ? g_hw->holding_r : !g_hw->holding_r;
        move_l = want_holding ? g_hw->holding_l : !g_hw->holding_l;
        if (!want_holding)
        {
            if (g_hw->last_placed_side == 'r')
                move_r = false;
            if (g_hw->last_placed_side == 'l')
                move_l = false;
        }
        const int nmove = (move_r ? 1 : 0) + (move_l ? 1 : 0);
        if (nmove != 1)
        {
            move_r = geo_right;
            move_l = !geo_right;
            std::cout << "[orch] 持料无法唯一选臂（持料 R=" << g_hw->holding_r
                      << " L=" << g_hw->holding_l << "），单步改走"
                      << (move_r ? "右臂" : "左臂") << "\n";
        }
    }

    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.aruco_engine_ready())
    {
        std::cerr << "[orch] ArUco 引擎未加载，无法走码旁定位\n";
        return -5;
    }

    const auto &cv = g_move_cfg.conveyor;
    const char *side_cn =
        above ? "正上方" : (hover == ArucoHover::Right ? "右侧" : "左侧");
    const char *pick_cn = (arm_force == 'r' || arm_force == 'l')
                              ? "指定臂"
                              : (want_holding ? "持料手" : "空手");
    const char *arm_cn = move_r ? "右臂" : "左臂";
    const double lat =
        above ? 0.0
              : (hover == ArucoHover::Right ? cv.right_offset_m : -cv.left_offset_m);
    const double height = above ? cv.above_height_m : cv.height_above_m;
    std::cout << "[orch] 二维码" << side_cn << "（" << pick_cn << "→" << arm_cn
              << "）：沿码 X 偏移 " << lat << " m，法向抬高 " << height << " m"
              << " 路径点 " << ex.waypoints.size()
              << " 持料 R=" << g_hw->holding_r << " L=" << g_hw->holding_l << "\n";

    g_hw->last_aruco_head = 0;
    g_hw->last_aruco_right = 0;
    g_hw->last_aruco_left = 0;
    g_hw->last_aruco_status = 1;
    g_hw->last_aruco_hits.clear();
    g_hw->last_hover_marker_id = -1;
    g_hw->last_hover_marker_id_b = -1;
    g_hw->last_dual_pos_m = -1.0;
    g_hw->last_dual_rot_deg = -1.0;
    g_hw->last_dual_corrected = false;

    if (kDebugVisualize)
        pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});
    int rc = detect_aruco_one_slot(
        g_hw->pipeline->cameras(), bridge, CameraSlot::Head, "码旁定位 头相机 ArUco",
        &g_hw->last_aruco_head);
    if (rc < 0)
        return rc;
    if (should_abort())
        return -4;

    const ArucoHit *hit = pick_robot_hit(ex.marker_id);
    if (hit == nullptr)
    {
        std::cerr << "[orch] 头相机未看到可用二维码（需基座位姿）\n";
        g_hw->last_aruco_status = 1;
        return 1;
    }
    if (ex.marker_id > 0 && hit->id != ex.marker_id)
        std::cerr << "[orch][WARN] 未找到主码 ID=" << ex.marker_id
                  << "，改用 ID=" << hit->id << "\n";
    g_hw->last_aruco_status = 0;

    const Eigen::Matrix4d T = maybe_correct_marker_frame(*hit, ex);
    const Eigen::Vector3d x_axis = T.block<3, 1>(0, 0);
    const Eigen::Vector3d z_axis = T.block<3, 1>(0, 2);

    Robot_Arm &arm = move_r ? *g_hw->arm_r : *g_hw->arm_l;
    const double speed = (cv.speed > 1e-4) ? cv.speed : 0.2;

    rc = move_rel_waypoints(arm, T, ex, speed, should_abort);
    if (rc != 0)
        return rc;

    const bool go_target = ex.waypoints.empty() || ex.include_target;
    if (go_target)
    {
        Eigen::Vector4d local;
        local << lat, 0.0, height, 1.0;
        const Eigen::Vector4d world = T * local;

        /** 只改 xyz，保持当前 TCP 姿态。套 standby rpy 会在到位后再拧腕/肩到限位（抵死）。 */
        const Eigen::Matrix<double, 1, 6> cur = arm_get_tcp_pos(arm);
        Eigen::Matrix<double, 1, 6> goal;
        goal << world(0), world(1), world(2), cur(3), cur(4), cur(5);
        const double z_before = goal(2);
        clamp_cart_goal_z(
            goal,
            above ? "二维码正上方" : (hover == ArucoHover::Right ? "传送带右侧上方" : "传送带左侧上方"));

        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] 码 ID=" << hit->id << " 基座=(" << hit->robot_x << "," << hit->robot_y
                  << "," << hit->robot_z << ") 码+X=(" << x_axis(0) << "," << x_axis(1) << ","
                  << x_axis(2) << ") 码+Z=(" << z_axis(0) << "," << z_axis(1) << "," << z_axis(2)
                  << ")\n"
                  << "[orch] → " << side_cn << " " << arm_cn << " goal xyz=(" << goal(0) << ","
                  << goal(1) << "," << goal(2) << ") 姿态=当前TCP rpy=("
                  << (goal(3) * 180.0 / M_PI) << "," << (goal(4) * 180.0 / M_PI) << ","
                  << (goal(5) * 180.0 / M_PI) << ") deg\n";
        if (std::abs(goal(2) - z_before) > 1e-9)
            std::cerr << "[orch][WARN] z 被 clamp_cart_goal_z 从 " << z_before << " 改成 " << goal(2)
                      << "（硬下限 -0.33→-0.30）\n";
        if (move_r && goal(1) > g_move_cfg.grasp_valid.right_y_max)
            std::cerr << "[orch][WARN] 右臂目标 y=" << goal(1) << " 超过右手工作带 right_y_max="
                      << g_move_cfg.grasp_valid.right_y_max << "\n";
        if (move_l && goal(1) < g_move_cfg.grasp_valid.left_y_min)
            std::cerr << "[orch][WARN] 左臂目标 y=" << goal(1) << " 低于左手工作带 left_y_min="
                      << g_move_cfg.grasp_valid.left_y_min << "\n";
        std::cout << std::defaultfloat << std::setprecision(6);

        ArmLineMoveDebugStage stage(
            above ? "二维码正上方"
                  : (hover == ArucoHover::Right ? "传送带右侧上方" : "传送带左侧上方"));
        rc = arm_line_move(arm, goal, speed);
        if (should_abort())
            return -4;
        if (rc != 0)
        {
            std::cerr << "[orch] 码旁直线失败 rc=" << rc << "\n";
            return -3;
        }
        const Eigen::Matrix<double, 1, 6> tcp = arm_get_tcp_pos(arm);
        const double arrive_err = (tcp.head<3>() - goal.head<3>()).norm();
        std::cout << std::fixed << std::setprecision(4)
                  << "[orch] 码旁到位 TCP=(" << tcp(0) << "," << tcp(1) << "," << tcp(2)
                  << ") 目标=(" << goal(0) << "," << goal(1) << "," << goal(2)
                  << ") 误差=" << arrive_err << " m\n"
                  << std::defaultfloat << std::setprecision(6);
    }
    else
        std::cout << "[orch] include_target=false，路径点走完即停，不走几何目标\n";

    g_hw->last_conveyor_r = move_r;
    g_hw->last_conveyor_l = move_l;
    if (arm_force == 0 && want_holding && !above)
        g_hw->last_placed_side = move_r ? 'r' : 'l';
    return 0;
}

} // namespace

int conveyor_goto_pipeline(
    bool is_right, bool want_holding, char arm_force, const ArucoHoverExtras *extras)
{
    return aruco_hover_pipeline(
        is_right ? ArucoHover::Right : ArucoHover::Left, want_holding, arm_force, extras);
}

int aruco_above_pipeline(bool want_holding, char arm_force, const ArucoHoverExtras *extras)
{
    return aruco_hover_pipeline(ArucoHover::Above, want_holding, arm_force, extras);
}

int path_record_pipeline(char arm_side, int marker_id, int marker_id_b, PathRecordResult &out)
{
    out = PathRecordResult{};
    if (g_hw == nullptr || g_hw->pipeline == nullptr || g_hw->arm_r == nullptr || g_hw->arm_l == nullptr)
        return -1;

    SegPoseBridge &bridge = g_hw->pipeline->bridge();
    if (!bridge.aruco_engine_ready())
    {
        std::cerr << "[orch] ArUco 引擎未加载，无法示教相对点\n";
        out.status = -5;
        return -5;
    }

    g_hw->last_aruco_head = 0;
    g_hw->last_aruco_right = 0;
    g_hw->last_aruco_left = 0;
    g_hw->last_aruco_status = 1;
    g_hw->last_aruco_hits.clear();

    if (kDebugVisualize)
        pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});
    int rc = detect_aruco_one_slot(
        g_hw->pipeline->cameras(), bridge, CameraSlot::Head, "路径示教 头相机 ArUco",
        &g_hw->last_aruco_head);
    if (rc < 0)
    {
        out.status = rc;
        return rc;
    }

    const ArucoHit *hit = pick_robot_hit(marker_id);
    if (hit == nullptr)
    {
        std::cerr << "[orch] 示教：头相机未看到可用二维码\n";
        g_hw->last_aruco_status = 1;
        out.status = 1;
        return 1;
    }
    g_hw->last_aruco_status = 0;

    Robot_Arm &arm = (arm_side == 'l') ? *g_hw->arm_l : *g_hw->arm_r;
    const Eigen::Matrix<double, 1, 6> tcp = arm_get_tcp_pos(arm);
    const Eigen::Matrix4d T_tcp = row_to_T(tcp);
    const Eigen::Matrix4d T_m = pose16_to_T(hit->pose_robot_4x4);
    const Eigen::Matrix<double, 1, 6> rel = T2PosEulerAngles(T_m.inverse() * T_tcp);
    const Eigen::Matrix<double, 1, 6> marker_row = T2PosEulerAngles(T_m);

    out.tcp = row_to_mrd(tcp);
    out.rel = row_to_mrd(rel);
    out.marker = row_to_mrd(marker_row);
    out.marker_id = hit->id;

    const ArucoHit *hit_b = pick_second_hit(hit, marker_id_b);
    if (hit_b != nullptr && hit_b != hit)
    {
        const Eigen::Matrix4d T_b = pose16_to_T(hit_b->pose_robot_4x4);
        out.have_b = true;
        out.marker_id_b = hit_b->id;
        out.marker_b = row_to_mrd(T2PosEulerAngles(T_b));
        out.dual_rel = row_to_mrd(T2PosEulerAngles(T_m.inverse() * T_b));
    }

    std::cout << std::fixed << std::setprecision(4)
              << "[orch] 示教 " << (arm_side == 'l' ? "左臂" : "右臂")
              << " 主码 ID=" << hit->id
              << " 相对 xyz=(" << rel(0) << "," << rel(1) << "," << rel(2)
              << ") rpy_deg=(" << (rel(3) * 180.0 / M_PI) << ","
              << (rel(4) * 180.0 / M_PI) << "," << (rel(5) * 180.0 / M_PI) << ")";
    if (out.have_b)
        std::cout << " 副码 ID=" << out.marker_id_b
                  << " dual_rel xyz=(" << out.dual_rel.x << "," << out.dual_rel.y << ","
                  << out.dual_rel.z << ")";
    std::cout << "\n" << std::defaultfloat << std::setprecision(6);
    out.status = 0;
    return 0;
}

namespace
{

int vision_hand_grasp_with_engine(SegEngineId engine_id)
{
    if (g_hw == nullptr || g_hw->g == nullptr || g_hw->pipeline == nullptr)
        return -1;

    bool enable_r = true;
    bool enable_l = true;
    if (g_hw->last_conveyor_r || g_hw->last_conveyor_l)
    {
        enable_r = g_hw->last_conveyor_r;
        enable_l = g_hw->last_conveyor_l;
        std::cout << "[orch] 手相机只动刚到传送带的手臂 右=" << enable_r << " 左=" << enable_l << "\n";
    }
    else
    {
        enable_r = !g_hw->holding_r;
        enable_l = !g_hw->holding_l;
        if (!enable_r && !enable_l)
        {
            enable_r = true;
            enable_l = true;
        }
        std::cout << "[orch] 手相机按空手 右=" << enable_r << " 左=" << enable_l
                  << " 持料 R=" << g_hw->holding_r << " L=" << g_hw->holding_l << "\n";
    }

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };
    std::string err;
    move_box::GraspToStandbyResult r = move_box::run_hand_grasp_only(
        *g_hw->g,
        *g_hw->arm_r,
        *g_hw->arm_l,
        g_hw->pipeline->cameras(),
        g_hw->pipeline->bridge(),
        err,
        should_abort,
        engine_id,
        enable_r,
        enable_l);

    g_hw->last_grasped_r = r.grasped_r;
    g_hw->last_grasped_l = r.grasped_l;
    g_hw->last_grasp_status = static_cast<int>(r.status);
    if (r.grasped_r)
        g_hw->holding_r = true;
    if (r.grasped_l)
        g_hw->holding_l = true;

    if (r.status == move_box::GraspToStandbyStatus::Aborted)
        return -4;
    if (r.status == move_box::GraspToStandbyStatus::Ok)
        return 0;
    if (r.status == move_box::GraspToStandbyStatus::NoTarget)
        return 1;
    return 2;
}

} // namespace

int vision_hand_grasp_pipeline()
{
    return vision_hand_grasp_with_engine(SegEngineId::Default);
}

int vision_hand_grasp_factory_pipeline()
{
    if (g_hw == nullptr || g_hw->pipeline == nullptr)
        return -1;
    if (!g_hw->pipeline->bridge().factory_engine_ready())
    {
        std::cerr << "[orch] 工厂检测引擎未加载（检查工程根 best.pt）\n";
        return -5;
    }
    std::cout << "[orch] 手相机抓取(小毛胚)：根目录 best.pt\n";
    return vision_hand_grasp_with_engine(SegEngineId::Factory);
}

int vision_place_pipeline()
{
    if (g_hw == nullptr || g_hw->g == nullptr || g_hw->pipeline == nullptr)
        return -1;

    g_hw->last_place_status = 1;
    g_hw->last_place_holes = 0;
    g_hw->last_released_r = false;
    g_hw->last_released_l = false;

    std::array<double, 16> cam2robot{};
    std::string err;
    if (!load_cam2robot_matrix(default_camera_to_robot_yaml_path(), cam2robot, err))
    {
        std::cerr << "[orch] 读 cam2robot 失败: " << err << std::endl;
        return -3;
    }

    auto should_abort = []() {
        return hardware_abort_requested() ||
               (g_hw && g_hw->abort && g_hw->abort->load()) || can_io_faulted();
    };

    std::cout << "[orch] 视觉放货：4类模型 class1 空位 持料 R=" << g_hw->holding_r
              << " L=" << g_hw->holding_l << "\n";
    move_box::PlaceToTrayResult r = move_box::run_place_holding_to_tray(
        *g_hw->g,
        *g_hw->arm_r,
        *g_hw->arm_l,
        g_hw->pipeline->cameras(),
        g_hw->pipeline->bridge(),
        cam2robot,
        g_hw->holding_r,
        g_hw->holding_l,
        err,
        should_abort);

    g_hw->last_place_status = static_cast<int>(r.status);
    g_hw->last_place_holes = r.holes_found;
    g_hw->last_released_r = r.released_r;
    g_hw->last_released_l = r.released_l;
    if (r.released_r)
        g_hw->holding_r = false;
    if (r.released_l)
        g_hw->holding_l = false;

    if (r.status == move_box::PlaceToTrayStatus::Aborted)
        return -4;
    if (r.status == move_box::PlaceToTrayStatus::Ok)
        return 0;
    if (r.status == move_box::PlaceToTrayStatus::MoveFail)
        return 2;
    return 1;
}

} // namespace orch
