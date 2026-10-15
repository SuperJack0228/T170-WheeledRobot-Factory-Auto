#pragma once

/**
 * M2 DL：模型注册表 + 统一 DlResult（有无/数量/框）。
 * 实现经 SegPoseBridge::run_yolo；皮带 ROI 计数走 model_key=belt_presence。
 */
#include <string>
#include <unordered_map>
#include <vector>

#include "seg_pose_bridge.h"
#include "vision/types.h"

namespace vision
{

struct DlDetection
{
    int class_id = -1;
    std::string class_name;
    float confidence = 0.f;
    bool success = false;
    /** 框中心，相对送进 YOLO 的那张图（裁切后即裁切坐标）。<0 未知。 */
    float u = -1.f;
    float v = -1.f;
};

struct DlResult
{
    bool ok = false;
    bool has_part = false;
    int count = 0;
    std::string model_key;
    std::string message;
    std::vector<DlDetection> detections;
    /** 兼容旧关联路径；业务新代码优先读 detections/count。 */
    PoseRunResult raw;
};

struct DlRunOpts
{
    int algorithm_id = -1;
    bool show_visualization = false;
    CameraSlot vis_slot = CameraSlot::Head;
    SegEngineId engine_id = SegEngineId::Default;
    /** belt_presence：像素 ROI；宽高须 >0。 */
    bool use_roi = false;
    int roi_x = 0;
    int roi_y = 0;
    int roi_w = 0;
    int roi_h = 0;
    double min_conf = 0.25;
    std::vector<int> part_class_ids; /**< 空则默认 0,1,2 */
};

struct ModelEntry
{
    std::string key;
    std::string type; /**< detect | segment */
    std::string weights;
    int default_algorithm = -1;
    std::string note;
};

class ModelRegistry
{
public:
    void clear();
    void upsert(ModelEntry entry);
    /** 内置：tray_parts / belt_presence（权重路径相对 feeding 包）。 */
    void register_defaults();
    bool has(const std::string &key) const;
    const ModelEntry *get(const std::string &key) const;

private:
    std::unordered_map<std::string, ModelEntry> entries_;
};

class IDlEngine
{
public:
    virtual ~IDlEngine() = default;
    virtual bool load(const ModelRegistry &reg, std::string &err) = 0;
    virtual DlResult run(
        const Frame &frame,
        const std::string &model_key,
        const DlRunOpts &opts = {}) = 0;
    /** 预留；落地 S8 时接真预热。 */
    virtual bool warmup(const std::string & /*model_key*/, std::string &err)
    {
        err = "warmup 未实现（S8）";
        return false;
    }
};

class BridgeDlEngine final : public IDlEngine
{
public:
    explicit BridgeDlEngine(SegPoseBridge &bridge);

    bool load(const ModelRegistry &reg, std::string &err) override;
    DlResult run(
        const Frame &frame,
        const std::string &model_key,
        const DlRunOpts &opts = {}) override;

    SegPoseBridge &bridge() { return bridge_; }

private:
    DlResult run_full_frame(const Frame &frame, const std::string &model_key, const DlRunOpts &opts);
    DlResult run_roi_count(const Frame &frame, const std::string &model_key, const DlRunOpts &opts);

    SegPoseBridge &bridge_;
    ModelRegistry registry_;
};

/** PoseRunResult → DlResult（不计 ROI）。 */
DlResult dl_from_pose_run(const PoseRunResult &raw, const std::string &model_key);

} // namespace vision
