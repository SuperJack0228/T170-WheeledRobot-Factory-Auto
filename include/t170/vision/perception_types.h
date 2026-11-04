#pragma once
#include "t170/common/frame.h"
#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

struct PoseTargetResult
{
    int class_id = -1;
    std::string class_name;
    int algorithm_id = 0;
    float confidence = 0.f;
    float radius_m = 0.f;
    /** 检测框中心像素；<0 表示未知。 */
    float box_u = -1.f;
    float box_v = -1.f;
    /** 检测框像素角点；<0 表示未知。 */
    float box_x0 = -1.f;
    float box_y0 = -1.f;
    float box_x1 = -1.f;
    float box_y1 = -1.f;
    bool success = false;
    std::string message;
    std::array<double, 16> pose_4x4{}; // 行主序 4×4，单位 m
};

struct PoseRunResult
{
    bool ok = false;
    std::string message;
    std::vector<PoseTargetResult> targets;
};

struct PoseDetectionRecord
{
    CameraSlot slot = CameraSlot::Head;
    std::string slot_name;
    int frame_algorithm_id = 0;
    PoseTargetResult target;
};

using PoseDetectionRecords = std::vector<PoseDetectionRecord>;

enum class SegEngineId
{
    /** 按相机槽：手相机用手部权重，头相机用头相机权重。 */
    Default = 0,
    /** 料盘、传送带。对应 model_key tray_parts / belt_presence。 */
    Head = 1,
    /** 左右手相机。对应 model_key hand_parts。 */
    Hand = 2,
};

struct ArucoMarkerResult
{
    int marker_id = -1;
    double side_m = 0.0;
    double reproj_px = 0.0;
    std::array<double, 16> pose_4x4{};
    double t_m[3]{};
    double rpy_deg[3]{};
};

struct ArucoDetectResult
{
    bool ok = false;
    std::string message;
    std::vector<ArucoMarkerResult> markers;
};

struct TrayHoleResult
{
    int id = 0;
    int row = 0;
    int col = 0;
    double x = 0.0;
    double y = 0.0;
    double x_level = std::numeric_limits<double>::quiet_NaN();
    double y_level = std::numeric_limits<double>::quiet_NaN();
    double tray_z = std::numeric_limits<double>::quiet_NaN();
    double tray_z_level = std::numeric_limits<double>::quiet_NaN();
    double top_z = std::numeric_limits<double>::quiet_NaN();
    double top_z_level = std::numeric_limits<double>::quiet_NaN();
    double height_on_tray = std::numeric_limits<double>::quiet_NaN();
    int class_id = -1;
    std::string class_name;
    double conf = 0.0;
    double dxy = -1.0;
    int depth_pts = 0;
    bool in_robot = false;
};

struct TrayDetectResult
{
    bool ok = false;
    std::string message;
    std::vector<int> used_ids;
    double reproj_px = -1.0;
    std::string save_path;
    double tilt_deg = std::numeric_limits<double>::quiet_NaN();
    /** 扶平后料盘原点（中心码 / 网格中心）在手臂基座系。NaN=未知。 */
    double origin_x = std::numeric_limits<double>::quiet_NaN();
    double origin_y = std::numeric_limits<double>::quiet_NaN();
    double origin_z = std::numeric_limits<double>::quiet_NaN();
    std::vector<TrayHoleResult> holes;
};

struct BeltDetectResult
{
    bool ok = false;
    std::string message;
    std::string name;
    std::vector<int> used_ids;
    double reproj_px = -1.0;
    std::string save_path;
    double cam_x = std::numeric_limits<double>::quiet_NaN();
    double cam_y = std::numeric_limits<double>::quiet_NaN();
    double cam_z = std::numeric_limits<double>::quiet_NaN();
    double origin_x = std::numeric_limits<double>::quiet_NaN();
    double origin_y = std::numeric_limits<double>::quiet_NaN();
    double origin_z = std::numeric_limits<double>::quiet_NaN();
    /** 码坐标系三轴在手臂基座：列向量 +X/+Y/+Z。have_axes=false 则未知。 */
    double ax_x = 1.0, ax_y = 0.0, ax_z = 0.0;
    double ay_x = 0.0, ay_y = 1.0, ay_z = 0.0;
    double az_x = 0.0, az_y = 0.0, az_z = 1.0;
    bool have_axes = false;
    bool in_robot = false;
    /** 左右手抓取点（识别系 offset 在相机里加完再变到基座）。 */
    double grasp_lx = std::numeric_limits<double>::quiet_NaN();
    double grasp_ly = std::numeric_limits<double>::quiet_NaN();
    double grasp_lz = std::numeric_limits<double>::quiet_NaN();
    double grasp_rx = std::numeric_limits<double>::quiet_NaN();
    double grasp_ry = std::numeric_limits<double>::quiet_NaN();
    double grasp_rz = std::numeric_limits<double>::quiet_NaN();
    bool have_grasp_l = false;
    bool have_grasp_r = false;
    /** 码平面在相机系的位姿（Rodrigues + 原点），用于把码系 ROI 投到图像。 */
    double rvec[3]{0, 0, 0};
    double tvec[3]{0, 0, 0};
    bool have_cam_pose = false;
};

struct PerceiveTrayOptions
{
    bool run_yolo = true;
    bool tray2_markers = false;
    bool show_tray_visualization = false;
    bool show_yolo_visualization = false;
    SegEngineId engine_id = SegEngineId::Default;
    int fuse_n = 0;
};

struct HeadTrayPerception
{
    std::vector<CameraFrameData> frames;
    PoseRunResult yolo;
    PoseDetectionRecords yolo_records;
    TrayDetectResult tray;
};

struct HeadBeltPerception
{
    CameraFrameData frame;
    BeltDetectResult belt;
    /** parallel_yolo 时与检码同时跑的整图 YOLO。裁切 ROI 仍要等码位姿，不走这里。 */
    bool yolo_ran = false;
    PoseRunResult yolo;
};

struct HeadHandAssignZones
{
    bool r_from_side = false;
    bool l_from_side = false;
    bool r_from_middle = false;
    bool l_from_middle = false;
};

struct PoseVisPanel
{
    static constexpr int Head = 0;
    static constexpr int RightHand = 1;
    static constexpr int LeftHand = 2;
    static constexpr int Count = 3;
};

struct PoseVisLayout
{
    static constexpr int Single = 0;
    static constexpr int DualHand = 1;
};

constexpr int kGraspDetectClassId = 0; // workpiece，立着的工件
constexpr int kEmptyHoleClassId = 1;   // feeding_hole / 空孔，料盘2放置用
constexpr int kFallenClassId = 2;      // fallen / 倒件
