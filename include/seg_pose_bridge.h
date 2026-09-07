#pragma once

#include <array>
#include <cstddef>
#include <initializer_list>
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

/** Default = 旧 4 类；Factory = 根目录单类；Jindi = seg_model 四类（毛胚/半加工/加工/孔） */
enum class SegEngineId
{
    Default = 0,
    Factory = 1,
    Jindi = 2,
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

    /** 在已有解释器上再加载工厂单类引擎；失败不影响抓取引擎 */
    bool init_factory_engine(const std::string &pose_config_path, std::string &err);
    bool factory_engine_ready() const;

    /** 金帝四类 seg_model/best.pt；失败不影响另外两套引擎 */
    bool init_jindi_engine(const std::string &pose_config_path, std::string &err);
    bool jindi_engine_ready() const;

    /** online_pose 单码 ArUco：按码 ID 查边长 + 相机内参 PnP；失败不影响抓取 */
    bool init_aruco_engine(const std::string &config_path, std::string &err);
    bool aruco_engine_ready() const;

    ArucoDetectResult run_aruco(
        const CameraFrameData &frame,
        bool show_visualization = true,
        CameraSlot vis_slot = CameraSlot::Head);

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

/** 抓取/放置 YOLO 类别（旧 4 类 / 工厂单类：0=零件；旧模型 1=孔） */
constexpr int kGraspDetectClassId = 0;
constexpr int kPlaceDetectClassId = 1;
/** 金帝四类：0 毛胚 1 半加工 2 加工 3 料盘孔 */
constexpr int kJindiBlankClassId = 0;
constexpr int kJindiSemiClassId = 1;
constexpr int kJindiFinishedClassId = 2;
constexpr int kJindiHoleClassId = 3;

/** 抓取阶段：按 move_box_params.yaml vision_detect 与相机槽返回 algorithm_id（-1/0/1） */
int algorithm_id_for_slot(CameraSlot slot);

/** 放货空位检测：vision_detect.place_holes */
int algorithm_id_for_place_holes();

/** 头相机空位检测（class 1）共识结果，基座系 1×6 */
struct PlaceHoleDetectResult
{
    bool ok = false;
    std::string message;
    int num_trials = 0;
    int cluster_size = 0;
    int picked_trial_index = -1;
    std::vector<Eigen::Matrix<double, 1, 6>> poses_robot;
};

/** 基座系 x 查配置 row_x_bounds 得排号（0=第1排），无效返回 -1 */
int place_pose_row_from_x(double x_robot_m);

/**
 * 按部位采图并跑位姿算法；filter_class_id>=0 时仅保留该 YOLO 类别且 success 的目标。
 */
PoseDetectionRecords detect_pose_at_slot(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    CameraSlot slot,
    bool show_visualization = false,
    int filter_class_id = kGraspDetectClassId,
    SegEngineId engine_id = SegEngineId::Default);

/** 头相机单次检测空位(class 1)，变换到基座系 */
PlaceHoleDetectResult detect_place_holes_with_consensus(
    SegPoseBridge &bridge,
    RealSenseMultiCam &cameras,
    const std::array<double, 16> &cam2robot,
    int num_trials = 1,
    double pos_tol_m = 0.02,
    double rot_tol_rad = 0.05,
    bool show_place_visualization = false);

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
std::string default_factory_pose_yaml_path();
std::string default_jindi_pose_yaml_path();
std::string default_seg_circle_pose_root();
std::string default_online_pose_root();
std::string default_aruco_pose_yaml_path();
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

/** 手相机多目标时取离相机最近的一个，cam2robot 变换到基座系 6D 位姿；无效时 x=-1 */
void assign_nearest_hand_cam_target_in_robot_frame(
    const PoseDetectionRecords &records,
    const std::array<double, 16> &cam2robot,
    Eigen::Matrix<double, 1, 6> &out_pose);

/** 调试合成窗格：0=Head 1=RightHand 2=LeftHand 3=HeadPlace(放货空位) */
struct PoseVisPanel
{
    static constexpr int Head = 0;
    static constexpr int RightHand = 1;
    static constexpr int LeftHand = 2;
    static constexpr int HeadPlace = 3;
    static constexpr int Count = 4;
};

/** 调试窗布局：单格(头/放货) 或 双手左右并排(宽度×2，避免手画面被压扁) */
struct PoseVisLayout
{
    static constexpr int Single = 0;
    static constexpr int DualHand = 1;
};

/** 切换布局；Single 时 single_panel_index 指定显示哪一格(Head 或 HeadPlace) */
void pose_vis_set_layout(int layout, int single_panel_index = PoseVisPanel::Head);

/** 清空指定窗格为黑图（py::none），并刷新当前布局窗口 */
void pose_vis_clear_panels(std::initializer_list<int> panels);

/** 新一轮识别开始前：清空旧图 → 切换布局 → 刷新（等待本次识别结果再显示） */
void pose_vis_begin_phase(
    int layout,
    int single_panel_index,
    std::initializer_list<int> clear_panels);

/** 与 main 中 kDebugVisualize 联动：调试图→picture_debug/{head,...}/，同名原图→picture_debug/original/{head,...}/ */
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
