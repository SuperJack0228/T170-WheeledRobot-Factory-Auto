#include "t170/app/maintenance.h"
#include "../../tasks/internal/runtime.h"
#include "t170/skills/conveyor.h"

#include "t170/telemetry/factory_status.h"
#include "t170/telemetry/dispatch_metrics.h"
#include "t170/motion/arm.h"
#include "t170/common/units.h"
#include "t170/motion/cartesian_tracking.h"
#include "t170/common/pose_math.h"
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
namespace {
Eigen::Matrix<double, 4, 4> pose16_to_T(const std::array<double, 16> &p)
{
    Eigen::Matrix<double, 4, 4> T;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r, c) = p[static_cast<size_t>(r * 4 + c)];
    return T;
}
}

namespace {
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
}

namespace {
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
}

namespace {
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
}

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
        std::cerr << "[orch] ArUco 引擎未加载（检查 config/vision/marker.yaml）\n";
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
        std::cerr << "[orch] 料盘引擎未加载（检查 config/vision/tray.yaml）\n";
        g_hw->last_tray_message = "料盘引擎未加载";
        return -5;
    }

    RealSenseMultiCam &cameras = g_hw->pipeline->cameras();
    std::cout << "[orch] 料盘孔位：ArUco 孔 XY + YOLO 类别 + 头测顶面 z（不动臂）\n";

    if (kDebugVisualize)
        pose_vis_begin_phase(PoseVisLayout::Single, PoseVisPanel::Head, {PoseVisPanel::Head});

    PerceiveTrayOptions opts;
    opts.run_yolo = true;
    opts.show_tray_visualization = kDebugVisualize;
    const HeadTrayPerception perc = perceive_tray_head(cameras, bridge, opts);
    if (perc.frames.empty())
    {
        std::cerr << "[tray] 头相机取帧失败: 多帧融合没有有效帧\n";
        g_hw->last_tray_message = "取帧失败";
        return -2;
    }
    if (!perc.yolo.ok)
        std::cerr << "[tray] YOLO 失败: " << perc.yolo.message << "（仍解料盘孔位）\n";
    else
        std::cout << "[tray] YOLO 目标 " << perc.yolo_records.size() << " 个（全部类别）\n";

    if (should_abort())
        return -4;

    const TrayDetectResult &det = perc.tray;
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
}
