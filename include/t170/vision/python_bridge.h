#pragma once
#include "t170/vision/visualization.h"
#include "t170/vision/perception_types.h"

#include <array>
#include <cstdint>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "t170/vision/realsense_camera.h"
#include "t170/common/paths.h"

/** 单个检测目标的位姿结果（来自 Python CirclePoseEngine） */

/** 单条检测结果：相机部位 + 该目标完整字段（含 pose_4x4） */

/** 单码 ArUco（online_pose）：相机系 4×4，t 单位 m */

/** 料盘孔：ArUco 几何 XY + YOLO 类别 + 头相机顶面 z（检测-only） */

/** 传送带 6×6：原点=两码中点，基座系坐标。与料盘 TrayDetectResult 分开。 */

/**
 * pybind11 内嵌 Python：加载 t170_vision 包中的 CirclePoseEngine。
 * seg_root 保留用于旧调用兼容；Python 代码从工程 python/ 或安装包导入。
 * 本步仍使用原有模型、板配置和标定资源位置。
 */
class SegPoseBridge
{
public:
    SegPoseBridge();
    ~SegPoseBridge();

    SegPoseBridge(const SegPoseBridge &) = delete;
    SegPoseBridge &operator=(const SegPoseBridge &) = delete;

    bool init(const std::string &pose_config_path, const std::string &seg_root, std::string &err);
    /** Read-only identity from the actual configured engines; empty before init. */
    std::string model_path(SegEngineId engine_id) const;

    /** online_pose 单码 ArUco：按码 ID 查边长 + 相机内参 PnP；失败不影响抓取 */
    bool init_aruco_engine(const std::string &config_path, std::string &err);
    bool aruco_engine_ready() const;

    ArucoDetectResult run_aruco(
        const CameraFrameData &frame,
        bool show_visualization = false,
        CameraSlot vis_slot = CameraSlot::Head);

    /** board_yf100 五码料盘：孔 1–36 + YOLO 类别 + 头测顶面 z；失败不影响抓取 */
    bool init_tray_engine(const std::string &config_path, std::string &err);
    bool tray_engine_ready() const;

    /** board_belt_aruco 传送带 6×6（独立引擎，不覆盖料盘 5×5） */
    bool init_belt_engine(const std::string &config_path, std::string &err);
    bool belt_engine_ready() const;

    /** 空图推理一次。已预热则直接返回，不重新加载权重。 */
    bool warmup_yolo(int height, int width, std::string &err);

    /**
     * 只检料盘码并缓存，供紧接着的 run_tray 跳过第二次 detectMarkers。
     * 返回 0 表示没缓存。与 YOLO 并行调用。
     */
    std::uint64_t locate_tray_markers(
        const std::vector<CameraFrameData> &frames,
        bool tray2_markers,
        std::string &err);
    void drop_tray_locate(std::uint64_t token);

    BeltDetectResult run_belt_detect(
        const CameraFrameData &frame,
        const std::string &prefer_name,
        bool show_visualization = false,
        CameraSlot vis_slot = CameraSlot::Head,
        const double *grasp_offset_left = nullptr,
        const double *grasp_offset_right = nullptr);

    TrayDetectResult run_tray_annotate(
        const CameraFrameData &frame,
        const PoseDetectionRecords &yolo,
        bool show_visualization = false,
        CameraSlot vis_slot = CameraSlot::Head);

    /** 多帧料盘位姿融合：各帧解扶平 T_robot_tray 后平均，YOLO/叠加用最后一帧。 */
    TrayDetectResult run_tray_annotate_multiframe(
        const std::vector<CameraFrameData> &frames,
        const PoseDetectionRecords &yolo,
        bool show_visualization = false,
        CameraSlot vis_slot = CameraSlot::Head,
        bool tray2_markers = false,
        std::uint64_t marker_token = 0);

    PoseRunResult run(
        const CameraFrameData &frame,
        int algorithm_id = -1,
        bool show_visualization = false,
        CameraSlot vis_slot = CameraSlot::Head,
        int vis_panel = -1,
        SegEngineId engine_id = SegEngineId::Default);

    /** 稳定业务名：YOLO / 料盘 / 传送带（实现仍走 run / run_tray_annotate* / run_belt_detect）。 */
    PoseRunResult run_yolo(
        const CameraFrameData &frame,
        int algorithm_id = -1,
        bool show_visualization = false,
        CameraSlot vis_slot = CameraSlot::Head,
        int vis_panel = -1,
        SegEngineId engine_id = SegEngineId::Default)
    {
        return run(frame, algorithm_id, show_visualization, vis_slot, vis_panel, engine_id);
    }

    TrayDetectResult run_tray(
        const std::vector<CameraFrameData> &frames,
        const PoseDetectionRecords &yolo,
        bool show_visualization = false,
        CameraSlot vis_slot = CameraSlot::Head,
        bool tray2_markers = false,
        std::uint64_t marker_token = 0)
    {
        return run_tray_annotate_multiframe(
            frames, yolo, show_visualization, vis_slot, tray2_markers, marker_token);
    }

    BeltDetectResult run_belt(
        const CameraFrameData &frame,
        const std::string &prefer_name,
        bool show_visualization = false,
        CameraSlot vis_slot = CameraSlot::Head,
        const double *grasp_offset_left = nullptr,
        const double *grasp_offset_right = nullptr)
    {
        return run_belt_detect(
            frame, prefer_name, show_visualization, vis_slot, grasp_offset_left,
            grasp_offset_right);
    }

    void close_visualization();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

void print_pose_run_result(const PoseRunResult &result, const char *slot_label, int algorithm_id);

/** 头相机 best1015.pt 与手相机 best1103.pt 类别号相同：0 立件，1 空孔，2 倒件。没有类别 3。 */

/** 抓取阶段：按 move_box_params.yaml vision_detect 与相机槽返回 algorithm_id（-1/0/1/2） */
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

/** 头相机：flush → 取 fuse_n 帧并 prepare。fuse_n<=0 用 yaml tray.fuse_frames。 */
std::vector<CameraFrameData> grab_head_fused_frames(
    RealSenseMultiCam &cameras,
    int fuse_n = 0);

/** 料盘头视感知编排选项。run_yolo 时 YOLO 与检码并行，再融合。 */

/** 已有多帧：可选 YOLO（末帧）→ run_tray。frames 空则 tray.message=取帧失败。 */
HeadTrayPerception perceive_tray_from_frames(
    SegPoseBridge &bridge,
    std::vector<CameraFrameData> frames,
    const PerceiveTrayOptions &opts = {});

/** 取头相机融合帧 + perceive_tray_from_frames。 */
HeadTrayPerception perceive_tray_head(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const PerceiveTrayOptions &opts = {});

/** 头相机单帧 + run_belt。parallel_yolo 时整图 YOLO 与检码并行。 */
HeadBeltPerception perceive_belt_head(
    RealSenseMultiCam &cameras,
    SegPoseBridge &bridge,
    const std::string &prefer_name,
    bool show_visualization = false,
    const double *grasp_offset_left = nullptr,
    const double *grasp_offset_right = nullptr,
    bool parallel_yolo = false);

void append_pose_records(
    PoseDetectionRecords &records,
    const char *slot_name,
    int frame_algorithm_id,
    const PoseRunResult &result);

/** 打印某一部位自己的缓存数组 */
void print_pose_records_summary(const char *slot_title, const PoseDetectionRecords &records);

/** 从 config/camera_to_base_result.yaml 读取 a.matrix（yaml 平移 mm，内部转为 m） */
bool load_cam2robot_matrix(const std::string &yaml_path, std::array<double, 16> &out, std::string &err);

/** pose_robot = cam2robot @ det_pose；cam2robot 与 det_pose 均为米 */
std::array<double, 16> transform_pose_cam_to_robot(
    const std::array<double, 16> &cam2robot,
    const std::array<double, 16> &det_pose_cam_m);

/** 过滤 robot 系检测结果，按 y 分区分配给左右手；无效时 x=-1,y=0,z=0 */

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

/** 调试窗布局：头相机单格或双手相机左右并排。 */

/** Ctrl+C / SIGTERM 请求退出循环。处理函数只打标记，不在信号里关 CAN。 */
void request_app_stop();
bool app_stop_requested();
void install_app_sigint_handler();
/** 开始收尾后再次收到信号才硬退出，避免清理卡死时无法打断。 */
void mark_app_cleanup_started();
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
