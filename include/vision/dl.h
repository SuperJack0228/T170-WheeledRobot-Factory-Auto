#pragma once

/**
 * M2 DL：模型注册表 + 统一 DlResult（有无/数量/框）。
 * 实现经 SegPoseBridge::run_yolo。传送带用 model_key=belt_presence，整图检测框。
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
    /** 框中心，相对送进 YOLO 的原图。<0 未知。 */
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
    double min_conf = 0.25;
    std::vector<int> part_class_ids; /**< 空则由调用方决定；传送带默认立件 0、倒件 2 */
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
    /** 空图推理一次，让权重留在进程里。重复调用不得重新 YOLO()。 */
    virtual bool warmup(const std::string &model_key, std::string &err) = 0;
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
    bool warmup(const std::string &model_key, std::string &err) override;

    SegPoseBridge &bridge() { return bridge_; }

private:
    DlResult run_full_frame(const Frame &frame, const std::string &model_key, const DlRunOpts &opts);

    SegPoseBridge &bridge_;
    ModelRegistry registry_;
};

/** PoseRunResult → DlResult。 */
DlResult dl_from_pose_run(const PoseRunResult &raw, const std::string &model_key);

} // namespace vision
