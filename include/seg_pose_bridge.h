#pragma once

#include <array>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "realsense_get.h"

/** 单个检测目标的位姿结果（来自 Python CirclePoseEngine） */
struct PoseTargetResult
{
    int class_id = -1;
    std::string class_name;
    int algorithm_id = 0;
    float confidence = 0.f;
    float radius_m = 0.f;
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

/** 单条检测结果：相机部位 + 该目标完整字段（含 pose_4x4） */
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
    Default = 0,
};

/** 单码 ArUco（online_pose）：相机系 4×4，t 单位 m */
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

/** 料盘孔：ArUco 几何 XY + YOLO 类别 + 头相机顶面 z（检测-only） */
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

/** 传送带 6×6：原点=两码中点，基座系坐标。与料盘 TrayDetectResult 分开。 */
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
};

/**
 * pybind11 内嵌 Python：加载 feeding_cylindrical_parts_alg 的 CirclePoseEngine。
 * seg_root 为 feeding_cylindrical_parts_alg 目录，用于 sys.path。
 */
class SegPoseBridge
{
public:
    SegPoseBridge();
    ~SegPoseBridge();

    SegPoseBridge(const SegPoseBridge &) = delete;
    SegPoseBridge &operator=(const SegPoseBridge &) = delete;

    bool init(const std::string &pose_config_path, const std::string &seg_root, std::string &err);

    /** online_pose 单码 ArUco：按码 ID 查边长 + 相机内参 PnP；失败不影响抓取 */
    bool init_aruco_engine(const std::string &config_path, std::string &err);
    bool aruco_engine_ready() const;

    ArucoDetectResult run_aruco(
        const CameraFrameData &frame,
        bool show_visualization = true,
        CameraSlot vis_slot = CameraSlot::Head);

    /** board_yf100 五码料盘：孔 1–36 + YOLO 类别 + 头测顶面 z；失败不影响抓取 */
    bool init_tray_engine(const std::string &config_path, std::string &err);
    bool tray_engine_ready() const;

    /** board_belt_aruco 传送带 6×6（独立引擎，不覆盖料盘 5×5） */
    bool init_belt_engine(const std::string &config_path, std::string &err);
    bool belt_engine_ready() const;

    BeltDetectResult run_belt_detect(
        const CameraFrameData &frame,
        const std::string &prefer_name,
        bool show_visualization = true,
        CameraSlot vis_slot = CameraSlot::Head,
        const double *grasp_offset_left = nullptr,
        const double *grasp_offset_right = nullptr);

    TrayDetectResult run_tray_annotate(
        const CameraFrameData &frame,
        const PoseDetectionRecords &yolo,
        bool show_visualization = true,
        CameraSlot vis_slot = CameraSlot::Head);

    /** 多帧料盘位姿融合：各帧解扶平 T_robot_tray 后平均，YOLO/叠加用最后一帧。 */
    TrayDetectResult run_tray_annotate_multiframe(
        const std::vector<CameraFrameData> &frames,
        const PoseDetectionRecords &yolo,
        bool show_visualization = true,
        CameraSlot vis_slot = CameraSlot::Head,
        bool tray2_markers = false);

    PoseRunResult run(
        const CameraFrameData &frame,
        int algorithm_id = -1,
        bool show_visualization = true,
        CameraSlot vis_slot = CameraSlot::Head,
        int vis_panel = -1,
        SegEngineId engine_id = SegEngineId::Default);

    void close_visualization();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

void print_pose_run_result(const PoseRunResult &result, const char *slot_label, int algorithm_id);

/** 当前唯一启用的 YOLO 类别：0=圆柱物料。 */
constexpr int kGraspDetectClassId = 0; // bestlatest.pt: original_product
constexpr int kEmptyHoleClassId = 3;   // feeding_hole / 空孔，料盘2放置用

/** 抓取阶段：按 move_box_params.yaml vision_detect 与相机槽返回 algorithm_id（-1/0/1） */
int algorithm_id_for_slot(CameraSlot slot);

/**
 * 把一次 PoseRunResult 收成 records。filter_class_id>=0 时仅保留该 YOLO 类别。
 * apply_head_edge_filter 只对头相机生效。
 */
PoseDetectionRecords pose_records_from_run(
    const PoseRunResult &result,
    CameraSlot slot,
    const CameraFrameData &frame,
    int filter_class_id,
    bool apply_head_edge_filter);

PoseDetectionRecords detect_pose_at_slot(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    CameraSlot slot,
    bool show_visualization = false,
    int filter_class_id = kGraspDetectClassId,
    SegEngineId engine_id = SegEngineId::Default);

void append_pose_records(
    PoseDetectionRecords &records,
    const char *slot_name,
    int frame_algorithm_id,
    const PoseRunResult &result);

/** 打印某一部位自己的缓存数组 */
void print_pose_records_summary(const char *slot_title, const PoseDetectionRecords &records);

/** 解析工程根目录（环境变量 MARKET_SIMPLE_ROOT，或从 build/ 向上查找） */
std::string project_root_dir();
std::string default_cameras_yaml_path();
std::string default_pose_yaml_path();
std::string default_cylinder_pose_root();
std::string default_online_pose_root();
std::string default_aruco_pose_yaml_path();
std::string default_board_yf100_root();
std::string default_board_yf100_yaml_path();
std::string default_board_belt_root();
std::string default_board_belt_yaml_path();
std::string default_camera_to_robot_yaml_path();
std::string default_left_hand_to_robot_yaml_path();
std::string default_right_hand_to_robot_yaml_path();

/** 从 config/camera_to_base_result.yaml 读取 a.matrix（yaml 平移 mm，内部转为 m） */
bool load_cam2robot_matrix(const std::string &yaml_path, std::array<double, 16> &out, std::string &err);

/** pose_robot = cam2robot @ det_pose；cam2robot 与 det_pose 均为米 */
std::array<double, 16> transform_pose_cam_to_robot(
    const std::array<double, 16> &cam2robot,
    const std::array<double, 16> &det_pose_cam_m);

/** 过滤 robot 系检测结果，按 y 分区分配给左右手；无效时 x=-1,y=0,z=0 */
struct HeadHandAssignZones
{
    bool r_from_side = false;
    bool l_from_side = false;
    bool r_from_middle = false;
    bool l_from_middle = false;
};

void assign_hand_targets_in_robot_frame(
    const PoseDetectionRecords &records,
    const std::array<double, 16> &cam2robot,
    double right_hand_pos[3],
    double left_hand_pos[3],
    HeadHandAssignZones *out_zones = nullptr);

/** 将单条检测从相机系转换到给定标定系的 6D 位姿；仅接受抓取类别。 */
bool transform_grasp_detection_to_pose(
    const PoseDetectionRecord &record,
    const std::array<double, 16> &cam2robot,
    Eigen::Matrix<double, 1, 6> &out_pose);

/** 调试合成窗格：头相机、右手相机、左手相机。 */
struct PoseVisPanel
{
    static constexpr int Head = 0;
    static constexpr int RightHand = 1;
    static constexpr int LeftHand = 2;
    static constexpr int Count = 3;
};

/** 调试窗布局：头相机单格或双手相机左右并排。 */
struct PoseVisLayout
{
    static constexpr int Single = 0;
    static constexpr int DualHand = 1;
};

/** 切换布局；Single 时 single_panel_index 指定显示的相机。 */
void pose_vis_set_layout(int layout, int single_panel_index = PoseVisPanel::Head);

/** 清空指定窗格为黑图（py::none），并刷新当前布局窗口 */
void pose_vis_clear_panels(std::initializer_list<int> panels);

/** 新一轮识别开始前：清空旧图 → 切换布局 → 刷新（等待本次识别结果再显示） */
void pose_vis_begin_phase(
    int layout,
    int single_panel_index,
    std::initializer_list<int> clear_panels);

/** 与 kDebugVisualize 联动：保存头相机/手相机原图和标注图。 */
void pose_vis_set_save_debug(bool enable);

/** 是否弹出 OpenCV/Qt 窗口。orch_hw 无显示器时必须关闭，否则 imshow 会 abort */
void pose_vis_set_gui_enabled(bool enable);

/** 保存一对调试图+原图（同名、各分子目录）；某一侧 ptr 为空或 bytes=0 则只存另一侧 */
void pose_vis_save_debug_pair_bgr(
    int panel_index,
    int orig_h,
    int orig_w,
    const uint8_t *orig_bgr,
    size_t orig_bytes,
    int proc_h,
    int proc_w,
    const uint8_t *proc_bgr,
    size_t proc_bytes);

/** 设置某一窗格 BGR 图（H×W×3 行连续）并刷新调试窗口 */
void pose_vis_set_panel_bgr(int panel_index, int height, int width, const uint8_t *bgr, size_t bgr_bytes);
void pose_vis_refresh_composite();

/** Ctrl+C 请求退出循环 */
void request_app_stop();
bool app_stop_requested();
void install_app_sigint_handler();
void interruptible_sleep_ms(int ms);

/** 三台相机 + Python 位姿引擎，init 后可直接 grab / run */
class PosePipeline
{
public:
    PosePipeline();
    ~PosePipeline();

    PosePipeline(const PosePipeline &) = delete;
    PosePipeline &operator=(const PosePipeline &) = delete;

    /** 解析路径、初始化 RealSense 与 CirclePoseEngine */
    bool init(std::string &err);

    void shutdown(bool close_visualization = false);

    RealSenseMultiCam &cameras();
    SegPoseBridge &bridge();
    const std::string &root() const { return root_; }

private:
    std::string root_;
    std::unique_ptr<RealSenseMultiCam> cameras_;
    std::unique_ptr<SegPoseBridge> bridge_;
};
