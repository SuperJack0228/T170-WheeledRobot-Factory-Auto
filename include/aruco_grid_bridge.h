#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "realsense_get.h"

/** 单次 ArUco 网格检测：48 个相机系 4×4（row-major，米） */
struct ArucoGridDetectResult
{
    bool ok = false;
    std::string message;
    int num_grid = 0;
    std::vector<int> marker_ids;
    int num_corners = 0;
    double reproj_px = 0.0;
    std::vector<std::array<double, 16>> poses_cam_4x4;
};

/** 多次检测 + 共识后，基座系 48 个 1×6（x,y,z m; rx,ry,rz rad） */
struct ArucoGridRobotPoses
{
    bool ok = false;
    std::string message;
    int num_trials = 0;
    int cluster_size = 0;
    int picked_trial_index = -1;
    std::vector<Eigen::Matrix<double, 1, 6>> poses_robot;
};

/**
 * pybind11 内嵌 Python：调用 seg_circle_pose/aruco_grid_api.py
 */
class ArucoGridBridge
{
public:
    ArucoGridBridge();
    ~ArucoGridBridge();

    ArucoGridBridge(const ArucoGridBridge &) = delete;
    ArucoGridBridge &operator=(const ArucoGridBridge &) = delete;

    bool init(const std::string &seg_root, std::string &err);
    void shutdown();
    ArucoGridDetectResult detect(const CameraFrameData &frame);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/** 相机系 4×4 → 基座系 1×6（弧度） */
Eigen::Matrix<double, 1, 6> aruco_pose_cam4x4_to_robot6(
    const std::array<double, 16> &cam2robot,
    const std::array<double, 16> &pose_cam_4x4);

/** 拍头相机并多次检测，取最大相似簇中的一组，再 cam2robot 变换 */
ArucoGridRobotPoses detect_aruco_grid_robot_poses_with_consensus(
    ArucoGridBridge &bridge,
    RealSenseMultiCam &cameras,
    const std::array<double, 16> &cam2robot,
    int num_trials = 7,
    double pos_tol_m = 0.02,
    double rot_tol_rad = 0.05,
    bool show_place_visualization = false);
