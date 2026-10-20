#include "vision/dl.h"

namespace vision
{

void ModelRegistry::clear()
{
    entries_.clear();
}

void ModelRegistry::upsert(ModelEntry entry)
{
    const std::string key = entry.key;
    entries_[key] = std::move(entry);
}

void ModelRegistry::register_defaults()
{
    upsert(ModelEntry{
        "tray_parts",
        "detect",
        "models/best1015.pt",
        2,
        "料盘关联用；业务仍按 class 挂孔（S2 延期）",
    });
    upsert(ModelEntry{
        "belt_presence",
        "detect",
        "models/best1015.pt",
        2,
        "传送带 ROI 有无+数量；不参与定位",
    });
}

bool ModelRegistry::has(const std::string &key) const
{
    return entries_.find(key) != entries_.end();
}

const ModelEntry *ModelRegistry::get(const std::string &key) const
{
    const auto it = entries_.find(key);
    return it == entries_.end() ? nullptr : &it->second;
}

DlResult dl_from_pose_run(const PoseRunResult &raw, const std::string &model_key)
{
    DlResult out;
    out.model_key = model_key;
    out.raw = raw;
    out.ok = raw.ok;
    out.message = raw.message;
    int n = 0;
    for (const auto &t : raw.targets)
    {
        DlDetection d;
        d.class_id = t.class_id;
        d.class_name = t.class_name;
        d.confidence = t.confidence;
        d.success = t.success;
        d.u = t.box_u;
        d.v = t.box_v;
        if (t.success)
            ++n;
        out.detections.push_back(std::move(d));
    }
    out.count = n;
    out.has_part = n > 0;
    return out;
}

BridgeDlEngine::BridgeDlEngine(SegPoseBridge &bridge) : bridge_(bridge)
{
    registry_.register_defaults();
}

bool BridgeDlEngine::load(const ModelRegistry &reg, std::string &err)
{
    registry_ = reg;
    if (!registry_.has("tray_parts") && !registry_.has("belt_presence"))
    {
        err = "ModelRegistry 为空：至少需要 tray_parts 或 belt_presence";
        return false;
    }
    err.clear();
    return true;
}

bool BridgeDlEngine::warmup(const std::string &model_key, std::string &err)
{
    if (!model_key.empty() && !registry_.has(model_key))
    {
        err = "未知 model_key: " + model_key;
        return false;
    }
    return bridge_.warmup_yolo(0, 0, err);
}

DlResult BridgeDlEngine::run(
    const Frame &frame,
    const std::string &model_key,
    const DlRunOpts &opts)
{
    return run_full_frame(frame, model_key, opts);
}

DlResult BridgeDlEngine::run_full_frame(
    const Frame &frame,
    const std::string &model_key,
    const DlRunOpts &opts)
{
    int algo = opts.algorithm_id;
    if (algo < 0)
    {
        if (const ModelEntry *e = registry_.get(model_key))
            algo = e->default_algorithm;
    }
    const PoseRunResult raw = bridge_.run_yolo(
        frame, algo, opts.show_visualization, opts.vis_slot, -1, opts.engine_id);
    return dl_from_pose_run(raw, model_key);
}

} // namespace vision
