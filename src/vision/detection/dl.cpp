#include "t170/vision/python_bridge.h"
#include "t170/vision/dl.h"

namespace vision
{

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
    registry_.register_defaults(bridge_.model_path(SegEngineId::Head), bridge_.model_path(SegEngineId::Hand));
}

bool BridgeDlEngine::load(const ModelRegistry &reg, std::string &err)
{
    registry_ = reg;
    if (!registry_.has("tray_parts") && !registry_.has("belt_presence"))
    {
        err = "ModelRegistry 为空：至少需要 tray_parts 或 belt_presence";
        return false;
    }
    // Registry selects logical roles. Actual model loading is owned by the
    // configured Python bridge, so metadata must not claim a different weight.
    for (const char *key : {"tray_parts", "belt_presence", "hand_parts"})
    {
        if (const auto *entry = registry_.get(key))
        {
            auto bound = *entry;
            bound.weights = bridge_.model_path(std::string(key)=="hand_parts" ? SegEngineId::Hand : SegEngineId::Head);
            registry_.upsert(std::move(bound));
        }
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
    SegEngineId engine_id = opts.engine_id;
    if (model_key == "hand_parts")
        engine_id = SegEngineId::Hand;
    else if (model_key == "tray_parts" || model_key == "belt_presence")
        engine_id = SegEngineId::Head;
    const PoseRunResult raw = bridge_.run_yolo(
        frame, algo, opts.show_visualization, opts.vis_slot, -1, engine_id);
    return dl_from_pose_run(raw, model_key);
}

} // namespace vision
